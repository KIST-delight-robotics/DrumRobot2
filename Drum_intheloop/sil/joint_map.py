from typing import Dict, Tuple
from .colors import SILVER, CHARCOAL

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
    # 발 페달: URDF joint가 아니라 PyBulletBackend가 별도 MultiBody로 처리하는 가상 키
    "R_foot": "pedal_right",
    "L_foot": "pedal_left",
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
    # L_arm1은 기존 구현을 유지하기 위해 90도 기준 mirror를 씁니다.
    # 기존 식: 90.0 - (target_deg - 90.0)
    "L_arm1": {
        "reference_deg": 90.0,
        "sign": -1.0,
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

# URDF joint가 아닌 가상 페달 키 → side 매핑.
# 어떤 virtual key가 페달인지에 대한 정의는 여기서만 관리한다.
PEDAL_JOINTS: dict = {
    "pedal_right": "right",
    "pedal_left": "left",
}

# 페달 geometry / 배치 스펙.
# 위치나 크기를 바꾸고 싶으면 여기만 고치면 된다.
PEDAL_SPEC: dict = {
    "half_extents": (0.04, 0.13, 0.015),       # 8cm × 26cm × 3cm
    "color_right":  [0.20, 0.10, 0.10, 1.0],   # 오른발 - 적갈색
    "color_left":   [0.10, 0.10, 0.20, 1.0],   # 왼발   - 남색
    "pos_right":    [-0.08, 0.22, 0.2],
    "pos_left":     [0.18,  0.22, 0.2],
    "max_tilt_deg": 28.0,
}

# 드럼 패드 전체 위치 보정치 (x, y, z).
# 숫자만 바꾸면 패드 세트 전체가 해당 방향으로 이동한다.
DRUM_PAD_OFFSET: tuple = (0.0, 0.0, -0.03)  

# 드럼 패드 geometry 스펙.
# 위치는 DrumRobot2/include/drum/drum_position.txt에서 읽어 런타임에 결정합니다.
# 크기/색상을 바꾸고 싶으면 여기만 고치면 됩니다.
DRUM_PAD_SPEC: dict = {
    "height":            0.006,
    "height_inner_extra": 0.002,   # z-fighting 방지용 inner 추가 높이
    "color_outer": SILVER,  # 은색 바깥
    "color_inner": CHARCOAL,   # 짙은 회색 중앙
    # 드럼 헤드 (S, FT, MT, HT)
    "drum_radius_outer": 0.085,
    "drum_radius_inner": 0.06,
    # 심벌 (HH, R, RC, LC, OHH, RB)
    "cymbal_radius_outer": 0.11,
    "cymbal_radius_inner": 0.08,
}

# drum_position.txt 열 순서 (0-9)
DRUM_INSTRUMENT_NAMES: list = ["S", "FT", "MT", "HT", "HH", "R", "RC", "LC", "OHH", "RB"]

# 드럼 헤드 인덱스 (나머지는 심벌)
DRUM_HEAD_INDICES: set = {0, 1, 2, 3}  # S, FT, MT, HT

# 배치에서 제외할 인덱스 (HH와 XY 완전 중복이라 위쪽 패드만 제거)
DRUM_PAD_SKIP_INDICES: set = {8}  # OHH — HH(4)와 같은 XY, wz만 더 높음

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
    LOOK_JOINTS["tilt"]: (60.0, 120.0),
}
