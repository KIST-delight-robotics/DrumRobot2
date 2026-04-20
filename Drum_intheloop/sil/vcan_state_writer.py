"""
vcan_state_writer.py

PyBullet의 현재 joint state를 읽어 vcan0에 struct can_frame 피드백을 쓴다.

C++ CanManager recv loop (100us 주기)가 vcan0 소켓에서 이 프레임을 읽으면
distributeFramesToMotors() 가 motor.jointAngle 을 갱신한다.

이 모듈이 채우는 경로:
  PyBullet stepSimulation()
      ↓  read_joint_states()
  vcan_state_writer.send_all(urdf_joint_deg_map)
      ↓  struct can_frame (socketcan)
  vcan0
      ↓
  C++ CanManager.readFramesFromAllSockets()
      ↓
  distributeFramesToMotors() → motor.jointAngle 갱신

CAN 프레임 포맷은 C++ CommandParser.cpp 와 1:1 대응해야 한다.

TMotor (TMotorServoCommandParser::motor_receive):
  can_id  = nodeId
  data[0..1] = int16_t big-endian:  pos_int = round(motor_pos_rad * 1800 / π)
  data[2..7] = 0

Maxon (MaxonCommandParser::parseRecieveCommand):
  can_id  = rxPdoIds[0]
  data[0] = 0
  data[1] = 0x37   (statusBit = 동작 정상)
  data[2..5] = int32_t little-endian: pos_enc = round(motor_pos_deg * 35 * 4096 / 360)
  data[6..7] = 0
"""

import math
import struct
import logging
from typing import Dict, Optional

try:
    import can
    _CAN_AVAILABLE = True
except ImportError:
    _CAN_AVAILABLE = False

from .joint_map import PRODUCTION_TO_URDF_JOINT, PRODUCTION_TO_URDF_CAN_TRANSFORM

logger = logging.getLogger(__name__)

# ────────────────────────────────────────────────────────────
# 모터 스펙 (C++ DrumRobot.cpp initializeMotors() 와 일치해야 함)
#
# cwDir / initialJointAngle_deg 는 TMotor/Maxon 모두
#   motorPositionToJointAngle(mp) = mp * cwDir + initialJointAngle
# 의 역변환에 쓰인다.
# ────────────────────────────────────────────────────────────

_TMOTOR_SPEC: Dict[str, Dict] = {
    "waist":  {"node_id": 0x00, "cw_dir":  1.0, "init_deg": 10.0},
    "R_arm1": {"node_id": 0x01, "cw_dir": -1.0, "init_deg": 90.0},
    "L_arm1": {"node_id": 0x02, "cw_dir": -1.0, "init_deg": 90.0},
    "R_arm2": {"node_id": 0x03, "cw_dir":  1.0, "init_deg":  0.0},
    "R_arm3": {"node_id": 0x04, "cw_dir": -1.0, "init_deg": 90.0},
    "L_arm2": {"node_id": 0x05, "cw_dir": -1.0, "init_deg":  0.0},
    "L_arm3": {"node_id": 0x06, "cw_dir":  1.0, "init_deg": 90.0},
}

_MAXON_SPEC: Dict[str, Dict] = {
    "R_wrist":      {"rx_pdo": 0x187, "cw_dir": -1.0, "init_deg": 90.0},
    "L_wrist":      {"rx_pdo": 0x188, "cw_dir": -1.0, "init_deg": 90.0},
    "maxonForTest": {"rx_pdo": 0x189, "cw_dir":  1.0, "init_deg":  0.0},
    "R_foot":       {"rx_pdo": 0x18A, "cw_dir":  1.0, "init_deg":  0.0},
    "L_foot":       {"rx_pdo": 0x18B, "cw_dir": -1.0, "init_deg":  0.0},
}

# PRODUCTION_TO_URDF_JOINT 의 역방향 매핑 (URDF joint name → production motor name)
_URDF_TO_PRODUCTION: Dict[str, str] = {
    urdf: prod for prod, urdf in PRODUCTION_TO_URDF_JOINT.items()
}


def _urdf_to_production_deg(prod_name: str, urdf_deg: float) -> float:
    """URDF joint angle(deg) → production motor command angle(deg) 역변환.

    joint_map.py 의 순방향 수식:
        mapped = bias + ref + sign * (prod - ref)
    역변환:
        prod = sign * (mapped - bias) + ref * (1 - sign)
    (sign = ±1 이므로 1/sign = sign)
    """
    transform = PRODUCTION_TO_URDF_CAN_TRANSFORM.get(prod_name)
    if transform is None:
        return urdf_deg  # 변환 테이블 없으면 그대로

    sign = transform["sign"]
    ref = transform["reference_deg"]
    bias = transform["bias_deg"]
    return sign * (urdf_deg - bias) + ref * (1.0 - sign)


def _joint_angle_to_motor_pos_rad(prod_deg: float, cw_dir: float, init_deg: float) -> float:
    """production joint angle(deg) → motor position(rad).

    C++ Motor.cpp (non-fourBar):
        motorPosition = (jointAngle - initialJointAngle) * cwDir
    """
    return math.radians(prod_deg - init_deg) * cw_dir


def _encode_tmotor_frame(node_id: int, motor_pos_rad: float) -> bytes:
    """TMotorServoCommandParser::motor_receive() 역변환 프레임.

    pos_int (int16) big-endian = round(motor_pos_rad * 1800 / π)
    data[2..7] = 0
    """
    pos_int = int(round(motor_pos_rad * 1800.0 / math.pi))
    pos_int = max(-32768, min(32767, pos_int))
    data = struct.pack(">h6x", pos_int)
    return data


def _encode_maxon_frame(rx_pdo: int, motor_pos_rad: float) -> bytes:
    """MaxonCommandParser::parseRecieveCommand() 역변환 프레임.

    data[0]  = 0
    data[1]  = 0x37 (statusBit)
    data[2..5] = int32 little-endian:
        pos_enc = round(motor_pos_deg * 35 * 4096 / 360)
    data[6..7] = 0
    """
    motor_pos_deg = math.degrees(motor_pos_rad)
    pos_enc = int(round(motor_pos_deg * 35.0 * 4096.0 / 360.0))
    pos_enc = max(-2147483648, min(2147483647, pos_enc))
    data = bytearray(8)
    data[1] = 0x37
    struct.pack_into("<i", data, 2, pos_enc)
    return bytes(data)


class VcanStateWriter:
    """PyBullet joint state → vcan0 CAN 피드백 프레임 송신기.

    사용 예:
        writer = VcanStateWriter()
        if writer.open():
            writer.send_all(joint_deg_map)   # tick마다 호출
        writer.close()
    """

    def __init__(self, channel: str = "vcan0"):
        self._channel = channel
        self._bus: Optional[object] = None

    def open(self) -> bool:
        """vcan0 소켓을 연다. 실패하면 False 반환 (SIL은 경고만 내고 계속 실행)."""
        if not _CAN_AVAILABLE:
            logger.warning("[vcan] python-can not installed, feedback disabled.")
            return False

        try:
            self._bus = can.interface.Bus(
                channel=self._channel,
                bustype="socketcan",
                receive_own_messages=False,
            )
            logger.info("[vcan] opened %s", self._channel)
            return True
        except Exception as exc:
            logger.warning("[vcan] failed to open %s: %s", self._channel, exc)
            self._bus = None
            return False

    def close(self) -> None:
        if self._bus is not None:
            try:
                self._bus.shutdown()
            except Exception:
                pass
            self._bus = None

    def send_all(self, urdf_joint_deg: Dict[str, float]) -> None:
        """urdf_joint_deg (URDF joint name → 현재 각도 deg) 전체를 CAN 프레임으로 송신.

        페달(pedal_right / pedal_left)은 URDF joint가 아니므로 무시.
        DXL(head / head_2)은 C++ recv 경로가 다르므로 무시.
        """
        if self._bus is None:
            return

        for urdf_name, urdf_deg in urdf_joint_deg.items():
            prod_name = _URDF_TO_PRODUCTION.get(urdf_name)
            if prod_name is None:
                continue

            self._send_motor(prod_name, urdf_deg)

    def _send_motor(self, prod_name: str, urdf_deg: float) -> None:
        prod_deg = _urdf_to_production_deg(prod_name, urdf_deg)

        if prod_name in _TMOTOR_SPEC:
            spec = _TMOTOR_SPEC[prod_name]
            motor_pos_rad = _joint_angle_to_motor_pos_rad(prod_deg, spec["cw_dir"], spec["init_deg"])
            data = _encode_tmotor_frame(spec["node_id"], motor_pos_rad)
            can_id = spec["node_id"]
        elif prod_name in _MAXON_SPEC:
            spec = _MAXON_SPEC[prod_name]
            motor_pos_rad = _joint_angle_to_motor_pos_rad(prod_deg, spec["cw_dir"], spec["init_deg"])
            data = _encode_maxon_frame(spec["rx_pdo"], motor_pos_rad)
            can_id = spec["rx_pdo"]
        else:
            return

        try:
            msg = can.Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=False,
                dlc=8,
            )
            self._bus.send(msg)
        except Exception as exc:
            logger.debug("[vcan] send error for %s: %s", prod_name, exc)
