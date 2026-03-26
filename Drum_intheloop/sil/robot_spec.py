# robot_spec.py

# CAN_MOTORS와 DXL_MOTORS는 로봇의 모터 사양을 정의하는 딕셔너리입니다. 각 모터는 고유한 이름과 함께 종류, 노드 ID 또는 DXL ID, 그리고 논리적 조인트 이름을 포함합니다.
CAN_MOTORS = {
    "waist": {"kind": "tmotor", "node_id": 0x00, "joint_index": 0},
    "R_arm1": {"kind": "tmotor", "node_id": 0x01, "joint_index": 1},
    "L_arm1": {"kind": "tmotor", "node_id": 0x02, "joint_index": 2},
    "R_arm2": {"kind": "tmotor", "node_id": 0x03, "joint_index": 3},
    "R_arm3": {"kind": "tmotor", "node_id": 0x04, "joint_index": 4},
    "L_arm2": {"kind": "tmotor", "node_id": 0x05, "joint_index": 5},
    "L_arm3": {"kind": "tmotor", "node_id": 0x06, "joint_index": 6},
    "R_wrist": {"kind": "maxon", "node_id": 0x07, "joint_index": 7},
    "L_wrist": {"kind": "maxon", "node_id": 0x08, "joint_index": 8},
    "maxonForTest": {"kind": "maxon", "node_id": 0x09, "joint_index": 9},
    "R_foot": {"kind": "maxon", "node_id": 0x0A, "joint_index": 10},
    "L_foot": {"kind": "maxon", "node_id": 0x0B, "joint_index": 11},
}

DXL_MOTORS = {
    "head_pan": {"kind": "dxl", "dxl_id": 1, "logical_joint": "pan"},
    "head_tilt": {"kind": "dxl", "dxl_id": 2, "logical_joint": "tilt"},
}

# DrumRobot2 실기 시작 전 수동으로 맞춰두는 초기 preset 자세입니다. [deg]
# 실제 C++ 초기화가 진행되면 이후 HOME/READY command 가 pipe를 통해 들어오며
# 시뮬레이터 자세도 그 흐름을 따라 바뀝니다.
STARTUP_CAN_POSE_DEG = {
    "waist": 10.0,
    "R_arm1": 90.0,
    "L_arm1": 90.0,
    "R_arm2": 0.0,
    "R_arm3": 90.0,
    "L_arm2": 0.0,
    "L_arm3": 90.0,
    "R_wrist": 90.0,
    "L_wrist": 90.0,
}

STARTUP_DXL_POSE_DEG = {
    "head_pan": 0.0,
    "head_tilt": 90.0,
}

# 모터 사양을 가져오는 함수입니다. 모터 이름을 입력받아 해당 모터의 사양을 반환합니다.
def get_can_motor(name: str):
    if name not in CAN_MOTORS:
        raise KeyError(f"Unknown CAN motor: {name}")
    return CAN_MOTORS[name]

def get_dxl_motor(name: str):
    if name not in DXL_MOTORS:
        raise KeyError(f"Unknown DXL motor: {name}")
    return DXL_MOTORS[name]
