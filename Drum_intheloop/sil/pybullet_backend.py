import atexit
import math
from pathlib import Path
import shutil
import tempfile
from typing import Dict, Optional, Tuple

from .colors import PLANE_RGBA, ROBOT_THEME
from .joint_map import DRUM_HEAD_INDICES, DRUM_PAD_OFFSET, DRUM_PAD_SKIP_INDICES, DRUM_PAD_SPEC, PEDAL_JOINTS, PEDAL_SPEC
from .urdf_tools import build_runtime_urdf

try:
    import pybullet as p
    import pybullet_data
except ImportError as exc:
    raise RuntimeError(
        "pybullet is required to run SIL. Install it with "
        "`python -m pip install -r phil_intheloop/requirements.txt`."
    ) from exc


class PyBulletBackend:
    def __init__(self, urdf_path: Path, mode: str = "gui"):
        self._source_urdf_path = urdf_path.resolve()
        self._mode = mode
        self._client_id: Optional[int] = None
        self._robot_id: Optional[int] = None
        self._patched_dir: Optional[str] = None
        self._joint_index_by_name: Dict[str, int] = {}
        self._r_pedal_id: Optional[int] = None
        self._l_pedal_id: Optional[int] = None
        self._base_z_shift: float = 0.0

    # PyBullet 켜고 로봇 URDF 올림
    def start(self) -> None:
        connection_mode = p.GUI if self._mode == "gui" else p.DIRECT # GUI 모드면 시뮬레이터 창 띄우고, 아니면 백그라운드에서 실행
        client_id = p.connect(connection_mode) # PyBullet에 연결하고 클라이언트 ID 받음
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
        plane_id = p.loadURDF("plane.urdf", physicsClientId=client_id)
        p.changeVisualShape(plane_id, -1, rgbaColor=PLANE_RGBA, physicsClientId=client_id)

        #=====================
        # GUI 창의 기본 디버그 패널을 숨기고, 처음부터 보기 쉬운 카메라 위치를 잡아둡니다.
        #=====================
        if self._mode == "gui":
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=client_id)
            p.resetDebugVisualizerCamera(
                cameraDistance=1.8,
                cameraYaw=45.0,
                cameraPitch=-20.0,
                cameraTargetPosition=[0.0, 0.0, 0.85],
                physicsClientId=client_id,
            )

        patched_urdf_path = self._build_runtime_urdf() # PyBullet에서 사용할 URDF 파일을 빌드하고 경로 받음
        base_orientation_quat = p.getQuaternionFromEuler((-math.pi / 2.0, 0.0, 0.0)) # URDF에서 정의된 기본 방향이 PyBullet의 기본 평면과 다르므로, 회전 변환을 적용합니다. X축을 기준으로 90도 회전하여 Z축이 위로 향하도록 합니다.

        # URDF 파일을 PyBullet에 로드해서 시뮬레이터에 로봇 객체 생성
        robot_id = p.loadURDF(
            str(patched_urdf_path),
            baseOrientation=base_orientation_quat,
            useFixedBase=True,
            physicsClientId=client_id,
        )

        self._client_id = client_id
        self._robot_id = robot_id
        self._joint_index_by_name = self._read_joint_indices()
        self._place_robot_on_ground()
        self._add_pedals()
        self._apply_visual_theme()
        self._add_drum_pad()  # 드럼 패드 활성화 시 주석 해제
        
    def close(self) -> None:
        if self._client_id is not None and p.isConnected(self._client_id):
            p.disconnect(physicsClientId=self._client_id)

        if self._patched_dir:
            shutil.rmtree(self._patched_dir, ignore_errors=True)

    def step(self) -> None:
        if self._client_id is None:
            return

        p.stepSimulation(physicsClientId=self._client_id)

    def read_joint_states(self) -> Dict[str, float]:
        """PyBullet에서 현재 joint 각도(deg)를 읽어 반환한다.

        반환값: {urdf_joint_name: angle_deg}
        페달이나 존재하지 않는 joint는 포함되지 않는다.
        vcan_state_writer.send_all() 에 그대로 전달한다.
        """
        if self._robot_id is None or self._client_id is None:
            return {}

        result: Dict[str, float] = {}
        for joint_name, joint_index in self._joint_index_by_name.items():
            state = p.getJointState(
                self._robot_id,
                joint_index,
                physicsClientId=self._client_id,
            )
            # state[0] = position in rad
            result[joint_name] = math.degrees(state[0])
        return result

    def apply_targets(
        self,
        joint_targets_deg: Dict[str, float],
    ) -> None:
        if self._robot_id is None:
            return

        for urdf_joint_name, target_deg in joint_targets_deg.items():
            # 어떤 키가 페달인지는 joint_map.PEDAL_JOINTS 가 정의한다.
            side = PEDAL_JOINTS.get(urdf_joint_name)
            if side is not None:
                pedal_id = self._r_pedal_id if side == "right" else self._l_pedal_id
                base_pos = list(PEDAL_SPEC["pos_right"] if side == "right" else PEDAL_SPEC["pos_left"])
                self._tilt_pedal(pedal_id, base_pos, target_deg)
                continue

            joint_index = self._joint_index_by_name.get(urdf_joint_name)
            if joint_index is None:
                continue

            self._set_joint_position_deg(joint_index, target_deg)

    def _set_joint_position_deg(self, joint_index: int, target_deg: float) -> None:
        if self._robot_id is None or self._client_id is None:
            return

        target_rad = math.radians(target_deg)
        p.resetJointState(
            self._robot_id,
            jointIndex=joint_index,
            targetValue=target_rad,
            targetVelocity=0.0,
            physicsClientId=self._client_id,
        )

    def _read_joint_indices(self) -> Dict[str, int]:
        if self._robot_id is None or self._client_id is None:
            return {}

        indices: Dict[str, int] = {}
        joint_count = p.getNumJoints(self._robot_id, physicsClientId=self._client_id)
        for joint_index in range(joint_count):
            joint_info = p.getJointInfo(self._robot_id, joint_index, physicsClientId=self._client_id)
            joint_name = joint_info[1].decode("utf-8")
            indices[joint_name] = joint_index

        return indices

    def _build_runtime_urdf(self) -> Path:
        patched_dir = tempfile.mkdtemp(prefix="drum_intheloop_urdf_")
        atexit.register(lambda: shutil.rmtree(patched_dir, ignore_errors=True))

        self._patched_dir = patched_dir
        return build_runtime_urdf(self._source_urdf_path, Path(patched_dir))

    def _disable_default_joint_motors(self) -> None:
        if self._robot_id is None or self._client_id is None:
            return

        joint_count = p.getNumJoints(self._robot_id, physicsClientId=self._client_id)
        for joint_index in range(joint_count):
            p.setJointMotorControl2(
                self._robot_id,
                joint_index,
                controlMode=p.VELOCITY_CONTROL,
                force=0.0,
                physicsClientId=self._client_id,
            )

    def _add_pedals(self) -> None:
        """R_foot / L_foot 위치에 박스 페달 두 개를 배치한다.
        URDF joint가 아니라 독립 MultiBody로 생성하므로 URDF를 건드리지 않는다."""
        if self._client_id is None:
            return

        half = PEDAL_SPEC["half_extents"]

        def make_pedal(pos, color):
            col = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=half,
                physicsClientId=self._client_id,
            )
            vis = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=half,
                rgbaColor=color,
                physicsClientId=self._client_id,
            )
            return p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=col,
                baseVisualShapeIndex=vis,
                basePosition=pos,
                physicsClientId=self._client_id,
            )

        self._r_pedal_id = make_pedal(PEDAL_SPEC["pos_right"], PEDAL_SPEC["color_right"])
        self._l_pedal_id = make_pedal(PEDAL_SPEC["pos_left"],  PEDAL_SPEC["color_left"])

    def _apply_visual_theme(self) -> None:
        """ROBOT_THEME 팔레트를 각 링크에 적용한다."""
        if self._robot_id is None or self._client_id is None:
            return

        base_color = ROBOT_THEME.get("base_link")
        if base_color is not None:
            p.changeVisualShape(
                self._robot_id,
                -1,
                rgbaColor=base_color,
                physicsClientId=self._client_id,
            )

        joint_count = p.getNumJoints(self._robot_id, physicsClientId=self._client_id)
        for joint_index in range(joint_count):
            joint_info = p.getJointInfo(self._robot_id, joint_index, physicsClientId=self._client_id)
            link_name = joint_info[12].decode("utf-8")
            link_color = ROBOT_THEME.get(link_name)
            if link_color is None:
                continue
            p.changeVisualShape(
                self._robot_id,
                joint_index,
                rgbaColor=link_color,
                physicsClientId=self._client_id,
            )

    def _load_drum_positions(self) -> list:
        """drum_position.txt를 읽어 10개 악기의 PyBullet world 좌표 리스트를 반환한다.
        파일이 없으면 빈 리스트를 반환한다.
        C++ IK 좌표계와 PyBullet world 좌표계가 동일(x=좌우, y=앞뒤, z=위)하므로
        오른손·왼손 타격 위치를 평균내어 직접 사용한다."""
        drum_pos_path = (
            Path(__file__).parent.parent.parent
            / "DrumRobot2" / "include" / "drum" / "drum_position.txt"
        )
        if not drum_pos_path.exists():
            return []

        values: list = []
        with drum_pos_path.open() as f:
            for line in f:
                values.extend(float(v) for v in line.split())

        # 6×10 행렬: 행 0-2 = 오른손 x/y/z, 행 3-5 = 왼손 x/y/z
        # 오른손·왼손 타격 좌표를 평균내어 악기 중심으로 사용한다.
        # C++ IK 좌표계(x=좌우, y=앞뒤, z=위)가 PyBullet world 좌표계와 동일하므로
        # 변환 없이 직접 사용한다.
        positions = []
        for col in range(10):
            rx_r = values[col];       ry_r = values[10 + col]; rz_r = values[20 + col]
            rx_l = values[30 + col];  ry_l = values[40 + col]; rz_l = values[50 + col]
            wx = (rx_r + rx_l) * 0.5 + DRUM_PAD_OFFSET[0]
            wy = (ry_r + ry_l) * 0.5 + DRUM_PAD_OFFSET[1]
            wz = (rz_r + rz_l) * 0.5 + DRUM_PAD_OFFSET[2]
            positions.append((wx, wy, wz))

        return positions

    def _add_drum_pad(self) -> None:
        """drum_position.txt 좌표를 기반으로 10개 악기 위치에 드럼 패드를 배치한다.
        드럼 헤드(S/FT/MT/HT)와 심벌(HH/R/RC/LC/OHH/RB)은 반지름이 다르다."""
        if self._client_id is None:
            return

        positions = self._load_drum_positions()
        if not positions:
            return

        h = DRUM_PAD_SPEC["height"]

        for i, pos in enumerate(positions):
            if i in DRUM_PAD_SKIP_INDICES:
                continue
            is_drum = i in DRUM_HEAD_INDICES
            radius = DRUM_PAD_SPEC["drum_radius_outer"] if is_drum else DRUM_PAD_SPEC["cymbal_radius_outer"]

            col = p.createCollisionShape(
                p.GEOM_CYLINDER,
                radius=radius,
                height=h,
                physicsClientId=self._client_id,
            )
            vis = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=radius,
                length=h,
                rgbaColor=DRUM_PAD_SPEC["color_outer"],
                physicsClientId=self._client_id,
            )
            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=col,
                baseVisualShapeIndex=vis,
                basePosition=list(pos),
                physicsClientId=self._client_id,
            )

    def _tilt_pedal(self, pedal_id: Optional[int], base_pos: list, angle_deg: float) -> None:
        """페달을 발뒤꿈치 축 기준으로 기울인다.
        angle_deg는 모터 joint 각도 그대로 넘어오므로 적당히 스케일링해 쓴다."""
        if pedal_id is None or self._client_id is None:
            return

        tilt = max(0.0, min(abs(angle_deg) * 0.6, PEDAL_SPEC["max_tilt_deg"]))
        tilt_rad = math.radians(tilt)

        # 발뒤꿈치(후방 끝)를 피벗으로: 페달 앞이 아래로 내려가도록 -Y축 회전
        orn = p.getQuaternionFromEuler([0.0, -tilt_rad, 0.0])

        # 피벗 보정: 뒤끝을 고정하면 중심이 살짝 앞·아래로 이동한다
        half_len = PEDAL_SPEC["half_extents"][1]
        pivot_offset_x = half_len * (1.0 - math.cos(tilt_rad))
        pivot_offset_z = half_len * math.sin(tilt_rad)
        corrected_pos = [
            base_pos[0] + pivot_offset_x,
            base_pos[1],
            base_pos[2] + pivot_offset_z * 0.5,
        ]

        p.resetBasePositionAndOrientation(
            pedal_id,
            corrected_pos,
            orn,
            physicsClientId=self._client_id,
        )

    def _place_robot_on_ground(self, clearance: float = 0.02) -> None:
        if self._robot_id is None or self._client_id is None:
            return

        # 로봇 전체에서 가장 낮은 AABB 최소 좌표를 찾기 위한 초기값입니다.
        # 특히 mins[2]는 모든 링크 중 가장 낮게 내려간 z 값을 담게 됩니다.
        mins = [float("inf"), float("inf"), float("inf")]

        # 조인트 수를 기준으로 base(-1)와 모든 링크의 bounding box를 순회합니다.
        link_count = p.getNumJoints(self._robot_id, physicsClientId=self._client_id)

        # link_index=-1은 base_link이고, 0..N-1은 각 조인트/링크입니다.
        # 각 링크의 AABB 최소 corner를 읽어 전체 로봇의 최저점을 계산합니다.
        for link_index in range(-1, link_count):
            aabb_min, _ = p.getAABB(self._robot_id, link_index, physicsClientId=self._client_id)
            for axis in range(3):
                mins[axis] = min(mins[axis], aabb_min[axis])

        # 현재 base 위치와 orientation을 읽습니다.
        # 여기서 orientation은 loadURDF 때 준 방향을 그대로 유지하기 위해 다시 사용합니다.
        base_position, base_orientation = p.getBasePositionAndOrientation(
            self._robot_id,
            physicsClientId=self._client_id,
        )

        # 로봇의 가장 낮은 점(mins[2])이 clearance 높이에 오도록
        # base의 z 위치를 얼마나 들어올려야 하는지 계산합니다.
        z_shift = clearance - mins[2]
        self._base_z_shift = z_shift  # _add_drum_pad() 좌표 변환에 사용

        # x, y는 유지하고 z만 올려 로봇이 바닥을 뚫지 않도록 보정합니다.
        corrected_base_position = [
            base_position[0],
            base_position[1],
            base_position[2] + z_shift,
        ]

        # 방향(base_orientation)은 바꾸지 않고, 위치만 새 z 값으로 재설정합니다.
        p.resetBasePositionAndOrientation(
            self._robot_id,
            corrected_base_position,
            base_orientation,
            physicsClientId=self._client_id,
        )
