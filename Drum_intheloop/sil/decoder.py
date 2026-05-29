import struct
from dataclasses import dataclass
from typing import List, Optional, Tuple

from .mapping import DXL_MOTORS, MAXON_SPEC, TMOTOR_SPEC, maxon_ids


# 공용 명령 객체

@dataclass(frozen=True)
class CanCommand:
    kind: str
    motor: Optional[str] = None
    position: Optional[float] = None
    velocity: Optional[float] = None
    torque_mnm: Optional[float] = None
    can_id: int = 0


@dataclass(frozen=True)
class DxlCommand:
    kind: str
    dxl_id: int
    address: int = 0
    length: int = 0
    data: bytes = b""


# NMT

NMT_COB_ID = 0x000

_NMT_COMMAND_KIND: dict = {
    0x01: "nmt_start",
    0x02: "nmt_stop",
    0x80: "nmt_preop",
    0x81: "nmt_reset",
    0x82: "nmt_reset",
}


def _decode_nmt(can_id: int, data: bytes) -> Optional[CanCommand]:
    if can_id != NMT_COB_ID or len(data) < 2:
        return None

    kind = _NMT_COMMAND_KIND.get(data[0])
    if kind is None:
        return None

    node_id = data[1]
    motor = _maxon_by_node(node_id) if node_id != 0 else None
    return CanCommand(kind=kind, motor=motor, can_id=can_id)


def _maxon_by_node(node_id: int) -> Optional[str]:
    for motor, spec in MAXON_SPEC.items():
        if int(spec["node_id"]) == node_id:
            return motor
    return None


# Maxon

def _decode_maxon(can_id: int, data: bytes) -> Optional[CanCommand]:
    for motor in MAXON_SPEC:
        ids = maxon_ids(motor)
        if can_id == ids["can_send"]:
            return CanCommand(kind="maxon_sdo", motor=motor, can_id=can_id)

        if can_id == ids["tx_position"] and len(data) >= 4:
            position_enc = int.from_bytes(data[:4], byteorder="little", signed=True)
            position_deg = position_enc * 360.0 / (35.0 * 4096.0)
            position = position_deg * 3.141592653589793 / 180.0
            return CanCommand(kind="maxon_position", motor=motor, position=position, can_id=can_id)

        if can_id == ids["tx_velocity"] and len(data) >= 4:
            velocity = float(int.from_bytes(data[:4], byteorder="little", signed=True))
            return CanCommand(kind="maxon_velocity", motor=motor, velocity=velocity, can_id=can_id)

        if can_id == ids["tx_torque"] and len(data) >= 2:
            torque_raw = int.from_bytes(data[:2], byteorder="little", signed=True)
            torque_mnm = float(torque_raw) * 31.052 / 1000.0
            return CanCommand(kind="maxon_torque", motor=motor, torque_mnm=torque_mnm, can_id=can_id)

        if can_id == ids["tx_control"]:
            return CanCommand(kind="maxon_control", motor=motor, can_id=can_id)

    return None


# TMotor

TMOTOR_SET_RPM = 3
TMOTOR_SET_POS = 4


def _decode_tmotor(can_id: int, data: bytes) -> Optional[CanCommand]:
    node_id = can_id & 0xFF
    packet_id = (can_id >> 8) & 0xFF
    motor = _tmotor_by_id(node_id)
    if motor is None:
        return None

    if packet_id == TMOTOR_SET_POS and len(data) >= 4:
        position_int = int.from_bytes(data[:4], byteorder="big", signed=True)
        position_deg = position_int / 10000.0
        position = position_deg * 3.141592653589793 / 180.0
        return CanCommand(kind="tmotor_position", motor=motor, position=position, can_id=can_id)

    if packet_id == TMOTOR_SET_RPM and len(data) >= 4:
        velocity = float(int.from_bytes(data[:4], byteorder="big", signed=True))
        return CanCommand(kind="tmotor_velocity", motor=motor, velocity=velocity, can_id=can_id)

    return None


def _tmotor_by_id(node_id: int) -> Optional[str]:
    for motor, spec in TMOTOR_SPEC.items():
        if int(spec["node_id"]) == node_id:
            return motor
    return None


# CAN 프레임 분기

def decode_can_frame(frame) -> Optional[CanCommand]:
    can_id = int(frame.arbitration_id) & 0x1FFFFFFF
    data = bytes(frame.data)

    nmt_command = _decode_nmt(can_id, data)
    if nmt_command is not None:
        return nmt_command

    tmotor_command = _decode_tmotor(can_id, data)
    if tmotor_command is not None:
        return tmotor_command

    maxon_command = _decode_maxon(can_id, data)
    if maxon_command is not None:
        return maxon_command

    if can_id == 0x80:
        return CanCommand(kind="maxon_sync", can_id=can_id)

    return None


# DXL

DXL_HEADER = b"\xff\xff\xfd\x00"
DXL_PING = 0x01
DXL_STATUS = 0x55
DXL_WRITE = 0x03
DXL_SYNC_READ = 0x82
DXL_SYNC_WRITE = 0x83
DXL_BROADCAST = 0xFE


def split_dxl_packets(buffer: bytes) -> Tuple[List[bytes], bytes]:
    packets: List[bytes] = []
    index = 0

    while True:
        header_index = buffer.find(DXL_HEADER, index)
        if header_index < 0:
            return packets, b""

        if len(buffer) - header_index < 10:
            return packets, buffer[header_index:]

        length = buffer[header_index + 5] | (buffer[header_index + 6] << 8)
        packet_size = 7 + length
        if len(buffer) - header_index < packet_size:
            return packets, buffer[header_index:]

        packet = buffer[header_index:header_index + packet_size]
        if check_dxl_crc(packet):
            packets.append(packet)
        index = header_index + packet_size


def decode_dxl_packet(packet: bytes) -> List[DxlCommand]:
    if len(packet) < 10:
        return []

    dxl_id = packet[4]
    instruction = packet[7]
    params = packet[8:-2]

    if instruction == DXL_PING:
        return [DxlCommand(kind="ping", dxl_id=dxl_id)]

    if instruction == DXL_WRITE and len(params) >= 2:
        address = params[0] | (params[1] << 8)
        return [DxlCommand(kind="write", dxl_id=dxl_id, address=address, data=params[2:])]

    if instruction == DXL_SYNC_WRITE and len(params) >= 4:
        address = params[0] | (params[1] << 8)
        length = params[2] | (params[3] << 8)
        return _decode_sync_write(address, length, params[4:])

    if instruction == DXL_SYNC_READ and len(params) >= 4:
        address = params[0] | (params[1] << 8)
        length = params[2] | (params[3] << 8)
        return [
            DxlCommand(kind="sync_read", dxl_id=dxl_id, address=address, length=length, data=params[4:])
        ]

    return []


def _decode_sync_write(address: int, length: int, data: bytes) -> List[DxlCommand]:
    commands: List[DxlCommand] = []
    step = length + 1

    for offset in range(0, len(data), step):
        item = data[offset:offset + step]
        if len(item) != step:
            continue

        dxl_id = item[0]
        if dxl_id not in DXL_MOTORS:
            continue
        commands.append(DxlCommand(kind="sync_write", dxl_id=dxl_id, address=address, length=length, data=item[1:]))

    return commands


def dxl_goal_deg(data: bytes) -> Optional[float]:
    if len(data) < 12:
        return None

    goal_tick = struct.unpack_from("<i", data, 8)[0]
    goal_deg = (2048.0 - float(goal_tick)) * 360.0 / 4096.0
    return goal_deg


def dxl_read_ids(command: DxlCommand) -> List[int]:
    return [dxl_id for dxl_id in command.data if dxl_id in DXL_MOTORS]


def check_dxl_crc(packet: bytes) -> bool:
    if len(packet) < 2:
        return False
    expected = packet[-2] | (packet[-1] << 8)
    actual = dxl_crc(packet[:-2])
    return expected == actual


def dxl_crc(data: bytes) -> int:
    crc_table = _dxl_crc_table()
    crc = 0

    for byte in data:
        table_index = ((crc >> 8) ^ byte) & 0xFF
        crc = ((crc << 8) ^ crc_table[table_index]) & 0xFFFF

    return crc


def _dxl_crc_table() -> Tuple[int, ...]:
    return (
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202,
    )
