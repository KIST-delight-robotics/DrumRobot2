import atexit
import math
from pathlib import Path
import shutil
import tempfile
from typing import Dict, Optional

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

    # PyBullet 켜고 로봇 URDF 올림
    def start(self) -> None:
        connection_mode = p.GUI if self._mode == "gui" else p.DIRECT # GUI 모드면 시뮬레이터 창 띄우고, 아니면 백그라운드에서 실행
        client_id = p.connect(connection_mode) # PyBullet에 연결하고 클라이언트 ID 받음
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
        p.loadURDF("plane.urdf", physicsClientId=client_id)

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
        
    def close(self) -> None:
        if self._client_id is not None and p.isConnected(self._client_id):
            p.disconnect(physicsClientId=self._client_id)

        if self._patched_dir:
            shutil.rmtree(self._patched_dir, ignore_errors=True)

    def step(self) -> None:
        if self._client_id is None:
            return

        p.stepSimulation(physicsClientId=self._client_id)

    def apply_targets(
        self,
        joint_targets_deg: Dict[str, float],
    ) -> None:
        if self._robot_id is None:
            return

        # URDF 조인트 이름을 사용하여 각 조인트에 목표 각도를 적용합니다. 조인트 이름이 매핑에 없는 경우 해당 조인트는 무시됩니다.
        for urdf_joint_name, target_deg in joint_targets_deg.items():
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
