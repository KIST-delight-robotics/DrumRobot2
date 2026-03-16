import copy
import re
from typing import Dict

SONG_LABELS = {
    "TIM": "This Is Me",
    "TY_short": "그대에게",
    "BI": "Baby I Need You",
    "test_one": "Test Beat",
    "None": "None",
}

DEFAULT_CURRENT_ANGLES = {
    "waist": None,
    "R_arm1": None,
    "L_arm1": None,
    "R_arm2": None,
    "R_arm3": None,
    "L_arm2": None,
    "L_arm3": None,
    "R_wrist": None,
    "L_wrist": None,
    "R_foot": None,
    "L_foot": None,
}

JOINT_QUERY_ALIASES = [
    ("왼쪽 손목", "L_wrist", "왼쪽 손목"),
    ("왼손목", "L_wrist", "왼쪽 손목"),
    ("오른쪽 손목", "R_wrist", "오른쪽 손목"),
    ("오른손목", "R_wrist", "오른쪽 손목"),
    ("허리", "waist", "허리"),
    ("왼쪽 팔", "L_arm1", "왼쪽 팔"),
    ("왼팔", "L_arm1", "왼쪽 팔"),
    ("오른쪽 팔", "R_arm1", "오른쪽 팔"),
    ("오른팔", "R_arm1", "오른쪽 팔"),
    ("왼쪽 발", "L_foot", "왼쪽 발"),
    ("왼발", "L_foot", "왼쪽 발"),
    ("오른쪽 발", "R_foot", "오른쪽 발"),
    ("오른발", "R_foot", "오른쪽 발"),
]

ANGLE_QUERY_PATTERN = re.compile(r"(각도|몇\s*도|몇도)")


def adapt_robot_state(raw_state):
    """
    C++에서 온 raw 상태를 LLM 친화 상태로 정규화한다.
    이후 C++ 상태 스키마가 달라져도 이 레이어만 고치면 프롬프트 계약은 유지된다.
    """
    source = copy.deepcopy(raw_state) if isinstance(raw_state, dict) else {}

    current_song = source.get("current_song", "None")
    error_detail = source.get("error_detail", source.get("error_message", "None"))

    current_angles = copy.deepcopy(DEFAULT_CURRENT_ANGLES)
    source_angles = source.get("current_angles")
    if isinstance(source_angles, dict):
        current_angles.update(source_angles)

    return {
        "state": source.get("state", 0),
        "bpm": source.get("bpm", 100),
        "is_fixed": source.get("is_fixed", True),
        "current_song": current_song,
        "current_song_label": SONG_LABELS.get(current_song, current_song),
        "progress": source.get("progress", "unknown"),
        "last_action": source.get("last_action", "None"),
        "is_lock_key_removed": source.get("is_lock_key_removed", False),
        "error_detail": error_detail,
        "current_angles": current_angles,
    }


def build_classifier_state_summary(robot_state: Dict) -> Dict:
    """
    classifier 는 의도 분류가 목적이므로 고수준 상태만 본다.
    관절각 전체처럼 저수준 제어용 상태는 여기서 제외한다.
    """
    state_value = robot_state.get("state", 0)
    is_fixed = robot_state.get("is_fixed", True)

    return {
        "mode": state_value,
        "can_move": robot_state.get("is_lock_key_removed", False),
        "busy": state_value != 0 or not is_fixed,
        "current_song": robot_state.get("current_song", "None"),
        "current_song_label": robot_state.get("current_song_label", "None"),
        "last_action": robot_state.get("last_action", "None"),
        "error_detail": robot_state.get("error_detail", "None"),
    }


def build_planner_state_summary(robot_state: Dict) -> Dict:
    """
    planner 역시 고수준 상태 요약만 사용한다.
    실제 관절각/세부 제어 정보는 Python resolver 와 validator 가 사용한다.
    """
    state_value = robot_state.get("state", 0)
    is_fixed = robot_state.get("is_fixed", True)

    return {
        "state": state_value,
        "can_move": robot_state.get("is_lock_key_removed", False),
        "is_fixed": is_fixed,
        "busy": state_value != 0 or not is_fixed,
        "current_song": robot_state.get("current_song", "None"),
        "current_song_label": robot_state.get("current_song_label", "None"),
        "bpm": robot_state.get("bpm", 100),
        "progress": robot_state.get("progress", "unknown"),
        "last_action": robot_state.get("last_action", "None"),
        "error_detail": robot_state.get("error_detail", "None"),
        # status/planner 가 현재 자세를 설명할 수 있도록 관절 스냅샷을 함께 전달한다.
        "current_angles": copy.deepcopy(robot_state.get("current_angles", DEFAULT_CURRENT_ANGLES)),
    }


def detect_joint_angle_query(user_text: str):
    """
    사용자가 특정 관절의 "현재 각도"를 묻는 질의인지 감지한다.
    각도 조회는 LLM 자유 생성보다 deterministic 응답이 더 안전하다.
    """
    text = (user_text or "").strip()
    if not text or not ANGLE_QUERY_PATTERN.search(text):
        return None

    for alias, joint_name, display_name in JOINT_QUERY_ALIASES:
        if alias in text:
            return {
                "joint_name": joint_name,
                "display_name": display_name,
            }

    return None


def build_joint_angle_answer(robot_state: Dict, joint_info: Dict):
    """
    상태 스냅샷에서 현재 관절 각도를 직접 읽어 사용자용 설명 문장으로 만든다.
    """
    if not isinstance(joint_info, dict):
        return None

    current_angles = robot_state.get("current_angles", {})
    angle_value = current_angles.get(joint_info["joint_name"])
    if not isinstance(angle_value, (int, float)):
        return f"{joint_info['display_name']}의 현재 각도는 아직 확인할 수 없습니다."

    return f"현재 {joint_info['display_name']} 각도는 {float(angle_value):.1f}도입니다."
