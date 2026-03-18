from typing import Dict, List, Tuple


# DrumRobot2/include/tasks/DrumRobot.hpp 의 범위를 우선 사용한다.
JOINT_LIMITS_DEG: Dict[str, Tuple[float, float]] = {
    "waist": (-90.0, 90.0),
    "R_arm1": (0.0, 150.0),
    "L_arm1": (30.0, 180.0),
    "R_arm2": (-60.0, 90.0),
    "R_arm3": (0.0, 140.1),
    "L_arm2": (-60.0, 90.0),
    "L_arm3": (0.0, 140.1),
    "R_wrist": (-108.0, 135.0),
    "L_wrist": (-108.0, 135.0),
    "R_foot": (-90.0, 200.0),
    "L_foot": (-90.0, 200.0),
}

INITIAL_JOINT_ANGLES_DEG: Dict[str, float] = {
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

HOME_JOINT_ANGLES_DEG: Dict[str, float] = {
    "waist": 0.0,
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

PRODUCTION_TO_URDF_SIGN: Dict[str, float] = {
    "waist": 1.0,
    "L_arm1": 1.0,
    # left arm forward/back semantics need sign correction because the URDF axis
    # direction differs from the right arm.
    "L_arm2": -1.0,
    "L_arm3": -1.0,
    "L_wrist": 1.0,
    "R_arm1": 1.0,
    "R_arm2": 1.0,
    "R_arm3": 1.0,
    "R_wrist": -1.0,
}

LOOK_JOINTS = {
    "pan": "head",
    "tilt": "head_2",
}

LOOK_LIMITS_DEG = {
    "pan": (-90.0, 90.0),
    "tilt": (0.0, 120.0),
}

URDF_JOINT_LIMITS_DEG: Dict[str, Tuple[float, float]] = {
    PRODUCTION_TO_URDF_JOINT["waist"]: JOINT_LIMITS_DEG["waist"],
    PRODUCTION_TO_URDF_JOINT["R_arm1"]: JOINT_LIMITS_DEG["R_arm1"],
    PRODUCTION_TO_URDF_JOINT["L_arm1"]: JOINT_LIMITS_DEG["L_arm1"],
    PRODUCTION_TO_URDF_JOINT["R_arm2"]: JOINT_LIMITS_DEG["R_arm2"],
    PRODUCTION_TO_URDF_JOINT["R_arm3"]: JOINT_LIMITS_DEG["R_arm3"],
    PRODUCTION_TO_URDF_JOINT["L_arm2"]: JOINT_LIMITS_DEG["L_arm2"],
    PRODUCTION_TO_URDF_JOINT["L_arm3"]: JOINT_LIMITS_DEG["L_arm3"],
    PRODUCTION_TO_URDF_JOINT["R_wrist"]: JOINT_LIMITS_DEG["R_wrist"],
    PRODUCTION_TO_URDF_JOINT["L_wrist"]: JOINT_LIMITS_DEG["L_wrist"],
    LOOK_JOINTS["pan"]: LOOK_LIMITS_DEG["pan"],
    LOOK_JOINTS["tilt"]: LOOK_LIMITS_DEG["tilt"],
}

SUPPORTED_GESTURES = {"wave", "hi", "nod", "shake", "hurray", "happy"}


def clamp_angle(joint_name: str, angle_deg: float) -> float:
    limits = JOINT_LIMITS_DEG.get(joint_name)
    if limits is None:
        return angle_deg
    lower, upper = limits
    return max(lower, min(upper, angle_deg))


def get_ready_pose() -> Dict[str, float]:
    return dict(INITIAL_JOINT_ANGLES_DEG)


def get_home_pose() -> Dict[str, float]:
    return dict(HOME_JOINT_ANGLES_DEG)


def get_gesture_sequence(gesture_name: str) -> List[Tuple[float, Dict[str, float], Tuple[float, float]]]:
    """
    반환 형식:
    - delay_sec
    - body pose
    - (pan, tilt)
    """
    name = (gesture_name or "").strip().lower()

    if name in {"wave", "hi"}:
        return [
            (0.0, {"R_wrist": 0.0, "R_arm1": 30.0, "R_arm2": 70.0, "R_arm3": 75.0}, (30.0, 100.0)),
            (1.0, {"R_wrist": 45.0, "R_arm3": 45.0}, (30.0, 100.0)),
            (2.0, {"R_wrist": 0.0, "R_arm3": 75.0}, (30.0, 100.0)),
            (3.0, {"R_wrist": 45.0, "R_arm3": 45.0}, (30.0, 100.0)),
            (4.0, {"R_wrist": 0.0, "R_arm3": 45.0}, (0.0, 90.0)),
        ]

    if name == "nod":
        return [
            (0.0, {}, (0.0, 110.0)),
            (1.0, {}, (0.0, 80.0)),
            (2.0, {}, (0.0, 95.0)),
        ]

    if name == "shake":
        return [
            (0.0, {}, (-30.0, 90.0)),
            (1.0, {}, (30.0, 90.0)),
            (2.0, {}, (0.0, 90.0)),
        ]

    if name in {"hurray", "happy"}:
        return [
            (0.0, {"R_arm2": 70.0, "L_arm2": 70.0, "R_arm3": 15.0, "L_arm3": 15.0}, (0.0, 70.0)),
            (1.0, {"R_arm2": 60.0, "L_arm2": 60.0, "R_arm3": 25.0, "L_arm3": 25.0}, (0.0, 80.0)),
            (2.0, {"R_arm2": 70.0, "L_arm2": 70.0, "R_arm3": 15.0, "L_arm3": 15.0}, (0.0, 70.0)),
        ]

    return []
