import atexit
import math
from pathlib import Path
import shutil
import tempfile
from typing import Dict, Optional

import pybullet as p

from .mapping import PEDAL_JOINTS
from .urdf_tools import build_runtime_urdf
from .visuals import (
    add_drum_pads,
    apply_robot_theme,
    create_pedals,
    setup_world,
    tilt_pedal,
)


class PyBulletBackend:
    def __init__(self, urdf_path: Path, mode: str = "gui"):
        self._source_urdf_path = urdf_path.resolve()
        self._mode = mode
        self._client_id: Optional[int] = None
        self._robot_id: Optional[int] = None
        self._patched_dir: Optional[str] = None
        self._joint_index_by_name: Dict[str, int] = {}
        self._pedal_ids: Dict[str, int] = {}

    # Lifecycle
    def start(self) -> None:
        client_id = p.connect(p.GUI if self._mode == "gui" else p.DIRECT)
        setup_world(client_id, self._mode)

        patched_urdf_path = self._build_runtime_urdf()
        robot_id = self._load_robot(client_id, patched_urdf_path)

        self._client_id = client_id
        self._robot_id = robot_id
        self._joint_index_by_name = self._read_joint_indices()
        self._place_robot_on_ground()
        self._pedal_ids = create_pedals(client_id)
        apply_robot_theme(client_id, robot_id)
        add_drum_pads(client_id)

    def close(self) -> None:
        if self._client_id is not None and p.isConnected(self._client_id):
            p.disconnect(physicsClientId=self._client_id)

        if self._patched_dir:
            shutil.rmtree(self._patched_dir, ignore_errors=True)

    def step(self) -> None:
        if self._client_id is None:
            return

        p.stepSimulation(physicsClientId=self._client_id)

    # Joint IO
    def read_joint_states(self) -> Dict[str, float]:
        if self._robot_id is None or self._client_id is None:
            return {}

        result: Dict[str, float] = {}
        for joint_name, joint_index in self._joint_index_by_name.items():
            state = p.getJointState(
                self._robot_id,
                joint_index,
                physicsClientId=self._client_id,
            )
            result[joint_name] = math.degrees(state[0])
        return result

    def apply_targets(self, joint_targets_deg: Dict[str, float]) -> None:
        if self._robot_id is None or self._client_id is None:
            return

        for joint_name, target_deg in joint_targets_deg.items():
            side = PEDAL_JOINTS.get(joint_name)
            if side is not None:
                tilt_pedal(self._client_id, self._pedal_ids.get(side), side, target_deg)
                continue

            joint_index = self._joint_index_by_name.get(joint_name)
            if joint_index is not None:
                self._set_joint_position_deg(joint_index, target_deg)

    # Robot loading
    def _load_robot(self, client_id: int, urdf_path: Path) -> int:
        base_orn = p.getQuaternionFromEuler((-math.pi / 2.0, 0.0, 0.0))
        return p.loadURDF(
            str(urdf_path),
            baseOrientation=base_orn,
            useFixedBase=True,
            physicsClientId=client_id,
        )

    def _build_runtime_urdf(self) -> Path:
        patched_dir = tempfile.mkdtemp(prefix="drum_intheloop_urdf_")
        atexit.register(lambda: shutil.rmtree(patched_dir, ignore_errors=True))

        self._patched_dir = patched_dir
        return build_runtime_urdf(self._source_urdf_path, Path(patched_dir))

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

    def _place_robot_on_ground(self, clearance: float = 0.02) -> None:
        if self._robot_id is None or self._client_id is None:
            return

        mins = [float("inf"), float("inf"), float("inf")]
        link_count = p.getNumJoints(self._robot_id, physicsClientId=self._client_id)
        for link_index in range(-1, link_count):
            aabb_min, _ = p.getAABB(self._robot_id, link_index, physicsClientId=self._client_id)
            for axis in range(3):
                mins[axis] = min(mins[axis], aabb_min[axis])

        base_pos, base_orn = p.getBasePositionAndOrientation(
            self._robot_id,
            physicsClientId=self._client_id,
        )
        pos = [
            base_pos[0],
            base_pos[1],
            base_pos[2] + clearance - mins[2],
        ]
        p.resetBasePositionAndOrientation(
            self._robot_id,
            pos,
            base_orn,
            physicsClientId=self._client_id,
        )

    # Joint writers
    def _set_joint_position_deg(self, joint_index: int, target_deg: float) -> None:
        if self._robot_id is None or self._client_id is None:
            return

        p.resetJointState(
            self._robot_id,
            jointIndex=joint_index,
            targetValue=math.radians(target_deg),
            targetVelocity=0.0,
            physicsClientId=self._client_id,
        )
