from pathlib import Path
from typing import Dict, Optional, Tuple

from .colors import CHARCOAL, SILVER


# Paths
PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF_PATH = (
    PROJECT_ROOT
    / "urdf"
    / "drumrobot_RL_urdf"
    / "urdf"
    / "drumrobot_RL_urdf.urdf"
)


# Motor specs
MAXON_SPEC: Dict[str, Dict[str, float]] = {
    "R_wrist": {"node_id": 0x07, "cw_dir": -1.0, "init_deg": 90.0, "gear_ratio": 35.0},
    "L_wrist": {"node_id": 0x08, "cw_dir": -1.0, "init_deg": 90.0, "gear_ratio": 35.0},
    "R_foot": {"node_id": 0x0A, "cw_dir": 1.0, "init_deg": 0.0, "gear_ratio": 35.0},
    "L_foot": {"node_id": 0x0B, "cw_dir": -1.0, "init_deg": 0.0, "gear_ratio": 35.0},
}

TMOTOR_SPEC: Dict[str, Dict[str, float]] = {
    "waist": {"node_id": 0x00, "cw_dir": 1.0, "init_deg": 10.0, "pole": 21.0, "gear_ratio": 9.0},
    "R_arm1": {"node_id": 0x01, "cw_dir": -1.0, "init_deg": 90.0, "pole": 21.0, "gear_ratio": 10.0},
    "L_arm1": {"node_id": 0x02, "cw_dir": -1.0, "init_deg": 90.0, "pole": 21.0, "gear_ratio": 10.0},
    "R_arm2": {"node_id": 0x03, "cw_dir": 1.0, "init_deg": 0.0, "pole": 21.0, "gear_ratio": 10.0},
    "R_arm3": {"node_id": 0x04, "cw_dir": -1.0, "init_deg": 90.0, "pole": 21.0, "gear_ratio": 10.0},
    "L_arm2": {"node_id": 0x05, "cw_dir": -1.0, "init_deg": 0.0, "pole": 21.0, "gear_ratio": 10.0},
    "L_arm3": {"node_id": 0x06, "cw_dir": 1.0, "init_deg": 90.0, "pole": 21.0, "gear_ratio": 10.0},
}

DXL_MOTORS: Dict[int, str] = {
    1: "head_pan",
    2: "head_tilt",
}


# Bus layout
CAN_BUS_MOTORS: Dict[str, Tuple[str, ...]] = {
    "vcan0": ("L_arm1", "L_arm2", "L_arm3", "waist"),
    "vcan1": ("R_arm1", "R_arm2", "R_arm3"),
    "vcan2": ("L_foot", "R_foot"),
    "vcan3": ("L_wrist", "R_wrist"),
}


# Joint mapping
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
    "R_foot": "pedal_right",
    "L_foot": "pedal_left",
}

PRODUCTION_TO_URDF_CAN_TRANSFORM: Dict[str, Dict[str, float]] = {
    "waist": {"reference_deg": 0.0, "sign": 1.0, "bias_deg": 0.0},
    "L_arm1": {"reference_deg": 90.0, "sign": -1.0, "bias_deg": 0.0},
    "L_arm2": {"reference_deg": 0.0, "sign": -1.0, "bias_deg": 0.0},
    "L_arm3": {"reference_deg": 0.0, "sign": -1.0, "bias_deg": 0.0},
    "L_wrist": {"reference_deg": 0.0, "sign": 1.0, "bias_deg": 0.0},
    "R_arm1": {"reference_deg": 90.0, "sign": -1.0, "bias_deg": 0.0},
    "R_arm2": {"reference_deg": 0.0, "sign": 1.0, "bias_deg": 0.0},
    "R_arm3": {"reference_deg": 0.0, "sign": 1.0, "bias_deg": 0.0},
    "R_wrist": {"reference_deg": 0.0, "sign": -1.0, "bias_deg": 0.0},
}

PRODUCTION_TO_URDF_SIGN: Dict[str, float] = {
    name: transform["sign"]
    for name, transform in PRODUCTION_TO_URDF_CAN_TRANSFORM.items()
}

LOOK_JOINTS = {
    "pan": "head",
    "tilt": "head_2",
}

# Visual mapping
PEDAL_JOINTS: dict = {
    "pedal_right": "right",
    "pedal_left": "left",
}

PEDAL_SPEC: dict = {
    "half_extents": (0.04, 0.13, 0.015),
    "color_right": [0.20, 0.10, 0.10, 1.0],
    "color_left": [0.10, 0.10, 0.20, 1.0],
    "pos_right": [-0.08, 0.22, 0.2],
    "pos_left": [0.18, 0.22, 0.2],
    "max_tilt_deg": 28.0,
}

DRUM_PAD_OFFSET: tuple = (0.0, 0.0, -0.03)

DRUM_PAD_SPEC: dict = {
    "height": 0.006,
    "height_inner_extra": 0.002,
    "color_outer": SILVER,
    "color_inner": CHARCOAL,
    "drum_radius_outer": 0.085,
    "drum_radius_inner": 0.06,
    "cymbal_radius_outer": 0.11,
    "cymbal_radius_inner": 0.08,
}

DRUM_INSTRUMENT_NAMES: list = ["S", "FT", "MT", "HT", "HH", "R", "RC", "LC", "OHH", "RB"]
DRUM_HEAD_INDICES: set = {0, 1, 2, 3}
DRUM_PAD_SKIP_INDICES: set = {8}


# Startup poses
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
    "R_foot": 0.0,
    "L_foot": 0.0,
}

STARTUP_DXL_POSE_DEG = {
    "head_pan": 0.0,
    "head_tilt": 90.0,
}


# Motor helpers
def motor_spec(name: str) -> Optional[Dict[str, float]]:
    if name in TMOTOR_SPEC:
        return TMOTOR_SPEC[name]
    return MAXON_SPEC.get(name)


def maxon_ids(name: str) -> Dict[str, int]:
    spec = MAXON_SPEC[name]
    node_id = int(spec["node_id"])
    return {
        "can_send": 0x600 + node_id,
        "can_receive": 0x580 + node_id,
        "tx_control": 0x200 + node_id,
        "tx_position": 0x300 + node_id,
        "tx_velocity": 0x400 + node_id,
        "tx_torque": 0x500 + node_id,
        "rx_state": 0x180 + node_id,
    }


# Angle mapping
def production_to_urdf_deg(name: str, target_deg: float) -> float:
    transform = PRODUCTION_TO_URDF_CAN_TRANSFORM.get(
        name,
        {"reference_deg": 0.0, "sign": 1.0, "bias_deg": 0.0},
    )
    reference = transform["reference_deg"]
    sign = transform["sign"]
    bias = transform["bias_deg"]
    return bias + reference + sign * (target_deg - reference)


def urdf_to_production_deg(name: str, urdf_deg: float) -> float:
    transform = PRODUCTION_TO_URDF_CAN_TRANSFORM.get(name)
    if transform is None:
        return urdf_deg

    sign = transform["sign"]
    reference = transform["reference_deg"]
    bias = transform["bias_deg"]
    return sign * (urdf_deg - bias) + reference * (1.0 - sign)


def motor_to_joint_deg(name: str, motor_rad: float) -> float:
    spec = motor_spec(name)
    if spec is None:
        return 0.0
    return math_deg(motor_rad) * spec["cw_dir"] + spec["init_deg"]


def joint_to_motor_rad(name: str, joint_deg: float) -> float:
    spec = motor_spec(name)
    if spec is None:
        return 0.0
    return math_rad((joint_deg - spec["init_deg"]) * spec["cw_dir"])


def dxl_to_urdf_deg(name: str, dxl_deg: float) -> Optional[Dict[str, float]]:
    if name == "head_tilt":
        dxl_deg = 90.0 - dxl_deg

    logical = "pan" if name == "head_pan" else "tilt"
    joint_name = LOOK_JOINTS.get(logical)
    if joint_name is None:
        return None
    return {joint_name: dxl_deg}


def urdf_to_dxl_deg(name: str, urdf_deg: float) -> float:
    if name == "head_tilt":
        return 90.0 - urdf_deg
    return urdf_deg


def math_rad(value_deg: float) -> float:
    return value_deg * 3.141592653589793 / 180.0


def math_deg(value_rad: float) -> float:
    return value_rad * 180.0 / 3.141592653589793
