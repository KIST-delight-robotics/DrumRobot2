import atexit
import math
from pathlib import Path
import shutil
import tempfile
from typing import Dict, Optional

from .colors import GUI_BACKGROUND_RGB, PLANE_RGBA, ROBOT_THEME
from .joint_map import LOOK_JOINTS, PRODUCTION_TO_URDF_JOINT, PRODUCTION_TO_URDF_SIGN
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

    def start(self) -> None:
        connection_mode = p.GUI if self._mode == "gui" else p.DIRECT
        if connection_mode == p.GUI and GUI_BACKGROUND_RGB is not None:
            bg_r, bg_g, bg_b = GUI_BACKGROUND_RGB
            options = (
                f"--background_color_red={bg_r} "
                f"--background_color_green={bg_g} "
                f"--background_color_blue={bg_b}"
            )
            self._client_id = p.connect(connection_mode, options=options)
        else:
            self._client_id = p.connect(connection_mode)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self._client_id)
        p.setGravity(0.0, 0.0, -9.81, physicsClientId=self._client_id)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self._client_id)
        plane_id = p.loadURDF("plane.urdf", physicsClientId=self._client_id)
        p.changeVisualShape(
            plane_id,
            -1,
            rgbaColor=PLANE_RGBA,
            physicsClientId=self._client_id,
        )
        patched_urdf_path = self._build_runtime_urdf()
        base_orientation = p.getQuaternionFromEuler((-math.pi / 2.0, 0.0, 0.0))
        self._robot_id = p.loadURDF(
            str(patched_urdf_path),
            basePosition=[0.0, 0.0, 0.0],
            baseOrientation=base_orientation,
            useFixedBase=True,
            flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
            physicsClientId=self._client_id,
        )
        self._joint_index_by_name = self._read_joint_indices()
        self._apply_visual_theme()
        self._disable_default_joint_motors()
        self._place_robot_on_ground()
        self.reset_camera()

    def close(self) -> None:
        if self._client_id is not None and p.isConnected(self._client_id):
            p.disconnect(physicsClientId=self._client_id)
        if self._patched_dir:
            shutil.rmtree(self._patched_dir, ignore_errors=True)

    def step(self) -> None:
        if self._client_id is None:
            return
        p.stepSimulation(physicsClientId=self._client_id)

    def apply_targets(self, body_targets_deg: Dict[str, float], head_pan_deg: float, head_tilt_deg: float) -> None:
        if self._robot_id is None:
            return

        for production_name, target_deg in body_targets_deg.items():
            urdf_joint_name = PRODUCTION_TO_URDF_JOINT.get(production_name)
            if urdf_joint_name is None:
                continue
            joint_index = self._joint_index_by_name.get(urdf_joint_name)
            if joint_index is None:
                continue
            sign = PRODUCTION_TO_URDF_SIGN.get(production_name, 1.0)
            if production_name == "R_arm1":
                mapped_deg = 90.0 + (-1.0) * (target_deg - 90.0)
            else:
                sign = PRODUCTION_TO_URDF_SIGN.get(production_name, 1.0)
                mapped_deg = sign * target_deg
            self._set_joint_position_deg(joint_index, mapped_deg)

        for axis_name, target_deg in {"pan": head_pan_deg, "tilt": head_tilt_deg}.items():
            urdf_joint_name = LOOK_JOINTS.get(axis_name)
            joint_index = self._joint_index_by_name.get(urdf_joint_name)
            if joint_index is None:
                continue
            if production_name == "R_arm1":
                mapped_deg = 90.0 + (-1.0) * (target_deg - 90.0)
            else:
                sign = PRODUCTION_TO_URDF_SIGN.get(production_name, 1.0)
                mapped_deg = sign * target_deg
            self._set_joint_position_deg(joint_index, mapped_deg)

    def reset_camera(self) -> None:
        if self._client_id is None:
            return
        p.resetDebugVisualizerCamera(
            cameraDistance=2.6,
            cameraYaw=45.0,
            cameraPitch=-20.0,
            cameraTargetPosition=[0.0, 0.0, 0.9],
            physicsClientId=self._client_id,
        )

    def save_screenshot(self, output_path: Path) -> Path:
        if self._client_id is None:
            raise RuntimeError("PyBullet is not started.")

        output_path = output_path.resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)

        width, height, rgba, _, _ = p.getCameraImage(
            width=1280,
            height=720,
            renderer=p.ER_TINY_RENDERER,
            physicsClientId=self._client_id,
        )
        rgb = self._extract_rgb_bytes(rgba, width, height)
        with output_path.open("wb") as handle:
            handle.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
            handle.write(rgb)
        return output_path

    def _set_joint_position_deg(self, joint_index: int, target_deg: float) -> None:
        target_rad = target_deg * math.pi / 180.0
        p.resetJointState(
            self._robot_id,
            jointIndex=joint_index,
            targetValue=target_rad,
            targetVelocity=0.0,
            physicsClientId=self._client_id,
        )

    def _read_joint_indices(self) -> Dict[str, int]:
        assert self._robot_id is not None
        indices: Dict[str, int] = {}
        for joint_index in range(p.getNumJoints(self._robot_id, physicsClientId=self._client_id)):
            info = p.getJointInfo(self._robot_id, joint_index, physicsClientId=self._client_id)
            joint_name = info[1].decode("utf-8")
            indices[joint_name] = joint_index
        return indices

    def _build_runtime_urdf(self) -> Path:
        self._patched_dir = tempfile.mkdtemp(prefix="phil_intheloop_urdf_")
        atexit.register(lambda: shutil.rmtree(self._patched_dir, ignore_errors=True))
        return build_runtime_urdf(self._source_urdf_path, Path(self._patched_dir))

    def _extract_rgb_bytes(self, rgba, width: int, height: int) -> bytes:
        """
        pybullet 반환값이 flat buffer 인지, row-major nested sequence 인지에 따라
        numpy 없이 RGB 바이트만 뽑아낸다.
        """
        if rgba is None:
            return b""

        try:
            if len(rgba) == 0:
                return b""
        except TypeError:
            return b""

        # numpy.ndarray 같은 객체는 tobytes()가 가장 안정적이다.
        if hasattr(rgba, "tobytes") and hasattr(rgba, "ndim"):
            raw = rgba.tobytes()
            if getattr(rgba, "ndim") == 3:
                rgb_bytes = bytearray()
                for index in range(0, len(raw), 4):
                    rgb_bytes.extend(raw[index:index + 3])
                return bytes(rgb_bytes)

        first = rgba[0]
        if isinstance(first, (list, tuple)):
            rgb_bytes = bytearray()
            for row in rgba:
                for pixel in row:
                    rgb_bytes.extend(int(channel) & 0xFF for channel in pixel[:3])
            return bytes(rgb_bytes)

        if hasattr(first, "__len__"):
            rgb_bytes = bytearray()
            for row in rgba:
                for pixel in row:
                    rgb_bytes.extend(int(channel) & 0xFF for channel in pixel[:3])
            return bytes(rgb_bytes)

        flat = list(rgba)
        expected_len = width * height * 4
        if len(flat) < expected_len:
            expected_len = len(flat) - (len(flat) % 4)

        rgb_bytes = bytearray()
        for index in range(0, expected_len, 4):
            rgb_bytes.extend(
                (
                    int(flat[index]) & 0xFF,
                    int(flat[index + 1]) & 0xFF,
                    int(flat[index + 2]) & 0xFF,
                )
            )
        return bytes(rgb_bytes)

    def _disable_default_joint_motors(self) -> None:
        if self._robot_id is None:
            return

        for joint_index in range(p.getNumJoints(self._robot_id, physicsClientId=self._client_id)):
            p.setJointMotorControl2(
                self._robot_id,
                joint_index,
                controlMode=p.VELOCITY_CONTROL,
                force=0.0,
                physicsClientId=self._client_id,
            )

    def _apply_visual_theme(self) -> None:
        if self._robot_id is None:
            return

        p.changeVisualShape(
            self._robot_id,
            -1,
            rgbaColor=ROBOT_THEME["base_link"],
            physicsClientId=self._client_id,
        )

        for joint_index in range(p.getNumJoints(self._robot_id, physicsClientId=self._client_id)):
            info = p.getJointInfo(self._robot_id, joint_index, physicsClientId=self._client_id)
            link_name = info[12].decode("utf-8")
            rgba = ROBOT_THEME.get(link_name)
            if rgba is None:
                continue
            p.changeVisualShape(
                self._robot_id,
                joint_index,
                rgbaColor=rgba,
                physicsClientId=self._client_id,
            )

    def _place_robot_on_ground(self, clearance: float = 0.02) -> None:
        if self._robot_id is None:
            return

        mins = [float("inf"), float("inf"), float("inf")]
        maxs = [float("-inf"), float("-inf"), float("-inf")]
        for link_index in range(-1, p.getNumJoints(self._robot_id, physicsClientId=self._client_id)):
            aabb_min, aabb_max = p.getAABB(self._robot_id, link_index, physicsClientId=self._client_id)
            for axis in range(3):
                mins[axis] = min(mins[axis], aabb_min[axis])
                maxs[axis] = max(maxs[axis], aabb_max[axis])

        base_pos, base_orn = p.getBasePositionAndOrientation(self._robot_id, physicsClientId=self._client_id)
        z_shift = clearance - mins[2]
        p.resetBasePositionAndOrientation(
            self._robot_id,
            [base_pos[0], base_pos[1], base_pos[2] + z_shift],
            base_orn,
            physicsClientId=self._client_id,
        )
