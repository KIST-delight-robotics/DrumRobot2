from typing import Dict, Optional

from .decoder import CanCommand, DxlCommand, dxl_goal_deg
from .mapping import (
    DXL_MOTORS,
    MAXON_SPEC,
    PRODUCTION_TO_URDF_JOINT,
    STARTUP_CAN_POSE_DEG,
    STARTUP_DXL_POSE_DEG,
    TMOTOR_SPEC,
    dxl_to_urdf_deg,
    motor_to_joint_deg,
    production_to_urdf_deg,
)

MAXON_TORQUE_GAIN = 3.0
MAXON_TORQUE_DAMPING = 8.0
MAXON_VELOCITY_LIMIT = 720.0
MAX_DT = 0.02


class MotorRouter:
    # 내부 상태
    def __init__(self):
        self.motor_target: Dict[str, float] = dict(STARTUP_CAN_POSE_DEG)
        self.dxl_target: Dict[str, float] = dict(STARTUP_DXL_POSE_DEG)
        self.motor_mode: Dict[str, str] = {}
        self.joint_velocity: Dict[str, float] = {}
        self.motor_torque: Dict[str, float] = {}

    # 시작 시 분배
    def startup_targets(self) -> Dict[str, float]:
        targets: Dict[str, float] = {}
        for motor, joint_deg in self.motor_target.items():
            targets.update(self._can_target(motor, joint_deg))

        for motor, dxl_deg in self.dxl_target.items():
            dxl_target = dxl_to_urdf_deg(motor, dxl_deg)
            if dxl_target is not None:
                targets.update(dxl_target)

        return targets

    # CAN 명령 분배
    def route_can(self, command: CanCommand) -> Optional[Dict[str, float]]:
        motor = command.motor

        if command.kind in {"tmotor_position", "maxon_position"} and command.position is not None:
            joint_deg = motor_to_joint_deg(motor, command.position)
            self.motor_target[motor] = joint_deg
            self.motor_mode[motor] = "position"
            self.joint_velocity[motor] = 0.0
            return self._can_target(motor, joint_deg)

        if command.kind == "tmotor_velocity" and command.velocity is not None:
            self.motor_mode[motor] = "velocity"
            self.joint_velocity[motor] = self._tmotor_velocity(motor, command.velocity)
            return None

        if command.kind == "maxon_velocity" and command.velocity is not None:
            self.motor_mode[motor] = "velocity"
            self.joint_velocity[motor] = self._maxon_velocity(motor, command.velocity)
            return None

        if command.kind == "maxon_torque" and command.torque_mnm is not None:
            self.motor_mode[motor] = "torque"
            self.motor_torque[motor] = command.torque_mnm
            return None

        return None

    # CAN 모션 적분
    def advance(self, dt: float) -> Dict[str, float]:
        dt = max(0.0, min(dt, MAX_DT))
        if dt == 0.0:
            return {}

        targets: Dict[str, float] = {}
        for motor, mode in self.motor_mode.items():
            if mode == "velocity":
                joint_deg = self._advance_velocity(motor, dt)
            elif mode == "torque":
                joint_deg = self._advance_torque(motor, dt)
            else:
                continue

            self.motor_target[motor] = joint_deg
            targets.update(self._can_target(motor, joint_deg))

        return targets

    def _can_target(self, motor: str, joint_deg: float) -> Dict[str, float]:
        joint_name = PRODUCTION_TO_URDF_JOINT.get(motor)
        if joint_name is None:
            return {}
        return {joint_name: production_to_urdf_deg(motor, joint_deg)}

    def _advance_velocity(self, motor: str, dt: float) -> float:
        joint_deg = self.motor_target.get(motor, STARTUP_CAN_POSE_DEG.get(motor, 0.0))
        velocity = self.joint_velocity.get(motor, 0.0)
        return joint_deg + velocity * dt

    def _advance_torque(self, motor: str, dt: float) -> float:
        joint_deg = self.motor_target.get(motor, STARTUP_CAN_POSE_DEG.get(motor, 0.0))
        velocity = self.joint_velocity.get(motor, 0.0)
        torque_mnm = self.motor_torque.get(motor, 0.0)

        accel = torque_mnm * MAXON_TORQUE_GAIN - velocity * MAXON_TORQUE_DAMPING
        velocity += accel * dt
        velocity = max(-MAXON_VELOCITY_LIMIT, min(MAXON_VELOCITY_LIMIT, velocity))
        self.joint_velocity[motor] = velocity

        return joint_deg + velocity * dt

    def _tmotor_velocity(self, motor: str, velocity_erpm: float) -> float:
        spec = TMOTOR_SPEC.get(motor)
        if spec is None:
            return 0.0

        pole = spec["pole"]
        gear_ratio = spec["gear_ratio"]
        joint_rad_s = velocity_erpm * 2.0 * 3.141592653589793 / (pole * gear_ratio * 60.0)
        return joint_rad_s * spec["cw_dir"] * 180.0 / 3.141592653589793

    def _maxon_velocity(self, motor: str, velocity_enc: float) -> float:
        spec = MAXON_SPEC.get(motor)
        if spec is None:
            return 0.0

        gear_ratio = spec["gear_ratio"]
        motor_deg_s = velocity_enc * 360.0 / (gear_ratio * 4096.0)
        return motor_deg_s * spec["cw_dir"]

    # DXL 명령 분배
    def route_dxl(self, command: DxlCommand) -> Optional[Dict[str, float]]:
        motor = DXL_MOTORS.get(command.dxl_id)
        if motor is None:
            return None

        if command.kind == "sync_write" and command.address == 108:
            goal_deg = dxl_goal_deg(command.data)
            if goal_deg is None:
                return None
            self.dxl_target[motor] = goal_deg
            return dxl_to_urdf_deg(motor, goal_deg)

        return None
