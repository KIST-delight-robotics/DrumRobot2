from typing import Dict

from .command_types import CanMotorCommand, MaxonData, TMotorData
from .joint_map import (
    LOOK_JOINTS,
    PRODUCTION_TO_URDF_CAN_TRANSFORM,
    PRODUCTION_TO_URDF_JOINT,
)
from .robot_spec import get_can_motor, get_dxl_motor

# CommandApplier는 로봇의 CAN 모터와 DXL 모터에 대한 명령을 적용하는 클래스
class CommandApplier:

    # apply_can_command 메서드는 주어진 모터 이름과 CAN 명령을 받아 해당 명령을 URDF 조인트 명령으로 변환
    def apply_can_command(self, motor_name: str, command: CanMotorCommand) -> Dict[str, float]:
        spec = get_can_motor(motor_name)
        urdf_joint_name = PRODUCTION_TO_URDF_JOINT.get(motor_name)
        if urdf_joint_name is None:
            return {}

        self._validate_can_command_kind(motor_name, spec["kind"], command)
        mapped_deg = self._map_can_target_deg(motor_name, command.position)
        return {urdf_joint_name: mapped_deg}

    # apply_dxl_command 메서드는 주어진 모터 이름과 DXL 명령을 받아 해당 명령을 URDF 조인트 명령으로 변환
    def apply_dxl_command(self, motor_name: str, position: float) -> Dict[str, float]:
        spec = get_dxl_motor(motor_name)
        if motor_name == "head_tilt":
            position = position - 90
        logical_joint = spec["logical_joint"]
        urdf_joint_name = LOOK_JOINTS.get(logical_joint)
        if urdf_joint_name is None:
            return {}

        return {urdf_joint_name: position}

    # build_default_can_command 메서드는 모터 이름과 위치를 받아 해당 모터 종류에 맞는 기본 CAN 명령 객체를 생성
    def build_default_can_command(self, motor_name: str, position: float) -> CanMotorCommand:
        spec = get_can_motor(motor_name)
        if spec["kind"] == "tmotor":
            return TMotorData(position=position)
        if spec["kind"] == "maxon":
            return MaxonData(position=position)
        raise ValueError(f"Unsupported CAN motor kind for {motor_name}: {spec['kind']}")

    # _map_can_target_deg 메서드는 모터 이름과 목표 각도를 받아 해당 모터의 URDF 조인트 명령으로 변환
    def _map_can_target_deg(self, motor_name: str, target_deg: float) -> float:
        # ==========
        # transform lookup
        # ==========
        transform = PRODUCTION_TO_URDF_CAN_TRANSFORM.get(
            motor_name,
            {
                "reference_deg": 0.0,
                "sign": 1.0,
                "bias_deg": 0.0,
            },
        )

        # ==========
        # transform parameters
        # ==========
        reference_deg = transform["reference_deg"]
        sign = transform["sign"]
        bias_deg = transform["bias_deg"]

        # ==========
        # mapping formula
        # ==========
        # mapped_deg = bias_deg + reference_deg + sign * (target_deg - reference_deg)
        # - reference_deg=0, sign=-1  -> 단순 부호 반전
        # - reference_deg=90, sign=-1 -> 90도 기준 mirror
        return bias_deg + reference_deg + sign * (target_deg - reference_deg)

    # _validate_can_command_kind 메서드는 모터 종류가 command 타입과 맞는지 검사
    def _validate_can_command_kind(
        self,
        motor_name: str,
        expected_kind: str,
        command: CanMotorCommand,
    ) -> None:
        if expected_kind == "tmotor" and isinstance(command, TMotorData):
            return
        if expected_kind == "maxon" and isinstance(command, MaxonData):
            return

        raise TypeError(
            f"CAN command kind mismatch for {motor_name}: "
            f"expected {expected_kind}, got {type(command).__name__}"
        )
