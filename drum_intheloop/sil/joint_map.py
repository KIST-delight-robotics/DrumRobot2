from typing import Dict, Tuple


PRODUCTION_TO_URDF_JOINT: Dict[str, str] = {
    "waist": "waist_joint",
    "L_arm1": "left_shoulder_1",
    "L_arm2": "left_shoulder_2",
    "L_arm3": "left_elbow",
    "L_wrist": "left_wrist",
    "R_arm1": "right_shoulder_1",
    "R_arm2": "right_shoulder_2",
    "R_arm3": "right_elbow",
    "R_wrist": "right_wrist",
}

# ================================================================
# PRODUCTION_TO_URDF_CAN_TRANSFORM
# ================================================================
# CAN command-level joint angle(target_deg)를 URDF joint angle로 바꿀 때 쓰는
# per-joint 변환 테이블입니다.
#
# 공통 수식:
# mapped_deg = bias_deg + reference_deg + sign * (target_deg - reference_deg)
#
# 해석:
# - sign = +1.0 이면 기준각(reference)을 중심으로 같은 방향을 유지합니다.
# - sign = -1.0 이면 기준각(reference)을 중심으로 반대 방향으로 뒤집습니다.
# - bias_deg 는 영점(offset) 차이가 있을 때 추가로 더하는 값입니다.
#
# 예:
# - reference_deg=0, sign=-1   -> 단순 부호 반전
# - reference_deg=90, sign=-1  -> 90도 기준 mirror
PRODUCTION_TO_URDF_CAN_TRANSFORM: Dict[str, Dict[str, float]] = {
    # ==========
    # waist
    # ==========
    "waist": {
        "reference_deg": 0.0,
        "sign": 1.0,
        "bias_deg": 0.0,
    },
    # ==========
    # left arm
    # ==========
    "L_arm1": {
        "reference_deg": 0.0,
        "sign": 1.0,
        "bias_deg": 0.0,
    },
    "L_arm2": {
        "reference_deg": 0.0,
        "sign": -1.0,
        "bias_deg": 0.0,
    },
    "L_arm3": {
        "reference_deg": 0.0,
        "sign": -1.0,
        "bias_deg": 0.0,
    },
    "L_wrist": {
        "reference_deg": 0.0,
        "sign": 1.0,
        "bias_deg": 0.0,
    },
    # ==========
    # right arm
    # ==========
    # R_arm1은 기존 구현을 유지하기 위해 90도 기준 mirror를 씁니다.
    # 기존 식: 90.0 - (target_deg - 90.0)
    "R_arm1": {
        "reference_deg": 90.0,
        "sign": -1.0,
        "bias_deg": 0.0,
    },
    "R_arm2": {
        "reference_deg": 0.0,
        "sign": 1.0,
        "bias_deg": 0.0,
    },
    "R_arm3": {
        "reference_deg": 0.0,
        "sign": 1.0,
        "bias_deg": 0.0,
    },
    "R_wrist": {
        "reference_deg": 0.0,
        "sign": -1.0,
        "bias_deg": 0.0,
    },
}

# sign만 빠르게 보고 싶을 때 쓰는 shorthand입니다.
PRODUCTION_TO_URDF_SIGN: Dict[str, float] = {
    joint_name: transform["sign"]
    for joint_name, transform in PRODUCTION_TO_URDF_CAN_TRANSFORM.items()
}

LOOK_JOINTS = {
    "pan": "head",
    "tilt": "head_2",
}

URDF_JOINT_LIMITS_DEG: Dict[str, Tuple[float, float]] = {
    PRODUCTION_TO_URDF_JOINT["waist"]: (-90.0, 90.0),
    PRODUCTION_TO_URDF_JOINT["R_arm1"]: (0.0, 150.0),
    PRODUCTION_TO_URDF_JOINT["L_arm1"]: (30.0, 180.0),
    PRODUCTION_TO_URDF_JOINT["R_arm2"]: (-60.0, 90.0),
    PRODUCTION_TO_URDF_JOINT["R_arm3"]: (0.0, 140.1),
    PRODUCTION_TO_URDF_JOINT["L_arm2"]: (-60.0, 90.0),
    PRODUCTION_TO_URDF_JOINT["L_arm3"]: (0.0, 140.1),
    PRODUCTION_TO_URDF_JOINT["R_wrist"]: (-108.0, 135.0),
    PRODUCTION_TO_URDF_JOINT["L_wrist"]: (-108.0, 135.0),
    LOOK_JOINTS["pan"]: (-90.0, 90.0),
    LOOK_JOINTS["tilt"]: (0.0, 120.0),
}
