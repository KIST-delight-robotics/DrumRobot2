"""시뮬레이터가 보낼 피드백 CAN 프레임 인코더.

이 모듈이 만드는 CANopen 프레임:
- Maxon TPDO1 (0x180+node): SYNC 수신 시 위치 피드백 (encode_maxon_feedback).
- Maxon SDO 응답 (0x580+node): 발견용 ack (encode_maxon_sdo_ack).

NMT/SYNC는 simulator가 수신하는 제어 프레임이라 이 encoder가 만들지 않는다.
heartbeat, EMCY, 그리고 단발 ack를 넘는 SDO 전송은 모델링하지 않는다.
TMotor servo status 프레임은 CubeMars 자체 포맷이며 CANopen이 아니다.
"""

import struct
from typing import Dict, Optional

import can

from .decoder import DXL_HEADER, DXL_STATUS, dxl_crc
from .mapping import (
    DXL_MOTORS,
    LOOK_JOINTS,
    MAXON_SPEC,
    TMOTOR_SPEC,
    joint_to_motor_rad,
    maxon_ids,
    urdf_to_dxl_deg,
    urdf_to_production_deg,
)


# Maxon

def encode_maxon_feedback(motor: str, urdf_deg: float):
    ids = maxon_ids(motor)
    joint_deg = urdf_to_production_deg(motor, urdf_deg)
    motor_rad = joint_to_motor_rad(motor, joint_deg)
    motor_deg = motor_rad * 180.0 / 3.141592653589793
    position_enc = int(round(motor_deg * 35.0 * 4096.0 / 360.0))
    position_enc = max(-2147483648, min(2147483647, position_enc))

    data = bytearray(8)
    data[1] = 0x37
    struct.pack_into("<i", data, 2, position_enc)
    return can.Message(
        arbitration_id=ids["rx_state"],
        data=bytes(data),
        is_extended_id=False,
        dlc=8,
    )


def encode_maxon_sdo_ack(motor: str):
    ids = maxon_ids(motor)
    data = bytes([0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    return can.Message(
        arbitration_id=ids["can_receive"],
        data=data,
        is_extended_id=False,
        dlc=8,
    )


# TMotor

def encode_tmotor_feedback(motor: str, urdf_deg: float):
    spec = TMOTOR_SPEC[motor]
    joint_deg = urdf_to_production_deg(motor, urdf_deg)
    motor_rad = joint_to_motor_rad(motor, joint_deg)
    position_int = int(round(motor_rad * 1800.0 / 3.141592653589793))
    position_int = max(-32768, min(32767, position_int))
    data = struct.pack(">h", position_int) + b"\x00\x00\x00\x00\x20\x00"
    return can.Message(
        arbitration_id=int(spec["node_id"]),
        data=data,
        is_extended_id=False,
        dlc=8,
    )


# DXL

def encode_dxl_status(dxl_id: int, params: bytes = b"", error: int = 0) -> bytes:
    body = bytes([dxl_id])
    length = len(params) + 4
    body += bytes([length & 0xFF, (length >> 8) & 0xFF])
    body += bytes([DXL_STATUS, error]) + params
    packet = DXL_HEADER + body
    crc = dxl_crc(packet)
    return packet + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def encode_dxl_ping(dxl_id: int) -> Optional[bytes]:
    if dxl_id not in DXL_MOTORS:
        return None
    # XL/XM 계열 모델 바이트는 SDK가 유효한 서보인지 확인할 때만 쓴다.
    return encode_dxl_status(dxl_id, params=bytes([0x30, 0x04, 0x26]))


def encode_dxl_write(dxl_id: int) -> Optional[bytes]:
    if dxl_id not in DXL_MOTORS:
        return None
    return encode_dxl_status(dxl_id)


def encode_dxl_read(dxl_id: int, joint_deg: float) -> Optional[bytes]:
    if dxl_id not in DXL_MOTORS:
        return None

    tick = angle_to_tick(joint_deg)
    params = struct.pack("<i", tick)
    return encode_dxl_status(dxl_id, params=params)


def dxl_joint_deg(dxl_id: int, urdf_state: Dict[str, float]) -> float:
    motor = DXL_MOTORS[dxl_id]
    joint_name = LOOK_JOINTS["pan"] if motor == "head_pan" else LOOK_JOINTS["tilt"]
    urdf_deg = urdf_state.get(joint_name, 0.0)
    return urdf_to_dxl_deg(motor, urdf_deg)


def angle_to_tick(angle_deg: float) -> int:
    angle_deg = max(-180.0, min(180.0, angle_deg))
    tick = 2048.0 - angle_deg * 4096.0 / 360.0
    return int(round(tick))


# 피드백 공통 분배
def motor_feedback(motor: str, urdf_deg: float):
    if motor in TMOTOR_SPEC:
        return encode_tmotor_feedback(motor, urdf_deg)
    if motor in MAXON_SPEC:
        return encode_maxon_feedback(motor, urdf_deg)
    return None
