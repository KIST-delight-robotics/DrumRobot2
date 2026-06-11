import re
from typing import Dict

SONG_LABELS = {
    "TIM": "This Is Me",
    "TY_short": "그대에게",
    "BI": "Baby I Need You",
    "test_one": "Test Beat",
    "None": "None",
}
PLAY_SKILL_BY_SONG = {
    "TIM": "play_tim",
    "TY_short": "play_ty_short",
    "BI": "play_bi",
    "test_one": "play_test_one",
}
SONG_QUERY_ALIASES = {
    "TIM": ["this is me", "tim"],
    "TY_short": ["그대에게", "ty_short"],
    "BI": ["baby i need you", "bi"],
    "test_one": ["test beat", "테스트 비트", "test_one"],
}
WAVE_REQUEST_KEYWORDS = ["손흔들", "손 흔들", "인사", "wave"]
PLAY_REQUEST_SUFFIXES = ["해줘", "해주세요", "해", "줘", "틀어", "연주", "쳐", "시작"]
ROBOT_NAME_ALIASES = {"필", "phil"}

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
REPERTOIRE_QUERY_PATTERNS = [
    re.compile(r"(무슨|어떤)\s*노래.*연주할\s*수\s*있"),
    re.compile(r"(무슨|어떤)\s*곡.*연주할\s*수\s*있"),
    re.compile(r"연주할\s*수\s*있는\s*(노래|곡)"),
    re.compile(r"(노래|곡)\s*(목록|리스트)"),
    re.compile(r"레퍼토리"),
]
AVAILABLE_SONG_CODES = ["TIM", "test_one", "TY_short", "BI"]
IDENTITY_CONFIRMATION_PATTERN = re.compile(
    r"(?:너의\s*)?이름(?:은)?\s*([A-Za-z가-힣]+)\s*(맞(?:지|죠|니|나요)|이니|인가|인가요)"
)


def adapt_robot_state(robot_state):
    """
    C++에서 온 상태 스냅샷을 그대로 pipeline에 넘긴다.
    get_robot_state_snapshot() 이 이미 deepcopy를 반환하므로 여기서 다시 복사하지 않는다.
    """
    return robot_state if isinstance(robot_state, dict) else {}


def build_classifier_state_summary(robot_state: Dict) -> Dict:
    """
    classifier 는 의도 분류가 목적이므로 고수준 상태만 본다.
    관절각 전체처럼 저수준 제어용 상태는 여기서 제외한다.
    """
    state_value = robot_state.get("state", 0)
    is_fixed = robot_state.get("is_fixed", True)

    summary = {
        "mode": state_value,
        "can_move": robot_state.get("is_lock_key_removed", False),
        "busy": state_value != 0 or not is_fixed,
        "current_song": robot_state.get("current_song", "None"),
        "last_action": robot_state.get("last_action", "None"),
    }

    if "error_message" in robot_state:
        summary["error_message"] = robot_state["error_message"]

    return summary


def build_planner_state_summary(robot_state: Dict) -> Dict:
    """
    planner 역시 고수준 상태 요약만 사용한다.
    실제 관절각/세부 제어 정보는 Python resolver 와 validator 가 사용한다.
    """
    state_value = robot_state.get("state", 0)
    is_fixed = robot_state.get("is_fixed", True)

    summary = {
        "state": state_value,
        "can_move": robot_state.get("is_lock_key_removed", False),
        "is_fixed": is_fixed,
        "busy": state_value != 0 or not is_fixed,
        "block_reason": block_reason_of(robot_state),
        "current_song": robot_state.get("current_song", "None"),
        "bpm": robot_state.get("bpm", 100),
        "progress": robot_state.get("progress", "unknown"),
        "last_action": robot_state.get("last_action", "None"),
    }

    if "error_message" in robot_state:
        summary["error_message"] = robot_state["error_message"]
    if "current_angles" in robot_state:
        # status/planner 가 현재 자세를 설명할 수 있도록 원본 관절 스냅샷을 전달한다.
        summary["current_angles"] = robot_state["current_angles"]

    return summary


def block_reason_of(robot_state: Dict) -> str:
    """
    현재 robot_state 가 motion/play 를 막는 단일 사유 코드를 만든다.
    validator(repair 사유)와 planner state 요약이 같은 판정을 쓰도록 한 곳에 둔다.

    반환값: "safety_key" | "playing" | "error" | "moving" | "none"
    우선순위는 안전 키 > 연주 > 에러 > 이동 순이다.
    (에러 state 표기가 코드 경로마다 4/6 으로 섞여 있어 둘 다 error 로 본다.)
    """
    if not robot_state.get("is_lock_key_removed", False):
        return "safety_key"
    state_value = robot_state.get("state", 0)
    if state_value == 2:
        return "playing"
    if state_value in (4, 6):
        return "error"
    if not robot_state.get("is_fixed", True):
        return "moving"
    return "none"


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


def detect_repertoire_query(user_text: str) -> bool:
    text = (user_text or "").strip()
    if not text:
        return False

    return any(pattern.search(text) for pattern in REPERTOIRE_QUERY_PATTERNS)


def build_repertoire_answer() -> str:
    song_names = [SONG_LABELS[song_code] for song_code in AVAILABLE_SONG_CODES]
    return "저는 " + ", ".join(song_names) + "를 연주할 수 있어요."


def detect_song_request_code(user_text: str):
    text = (user_text or "").strip().lower()
    if not text:
        return None

    for song_code, alias_list in SONG_QUERY_ALIASES.items():
        if any(alias in text for alias in alias_list):
            return song_code

    return None


def detect_wave_play_request(user_text: str):
    text = (user_text or "").strip().lower()
    if not text:
        return None

    song_code = detect_song_request_code(text)
    if song_code is None:
        return None

    has_wave = any(keyword in text for keyword in WAVE_REQUEST_KEYWORDS)
    has_play_suffix = any(keyword in text for keyword in PLAY_REQUEST_SUFFIXES)
    if not has_wave or not has_play_suffix:
        return None

    return {
        "song_code": song_code,
        "song_label": SONG_LABELS[song_code],
        "play_skill": PLAY_SKILL_BY_SONG[song_code],
    }


def detect_identity_confirmation_query(user_text: str):
    text = (user_text or "").strip()
    if not text:
        return None

    match_obj = IDENTITY_CONFIRMATION_PATTERN.search(text)
    if match_obj is None:
        return None

    candidate_name = match_obj.group(1).strip()
    normalized_name = candidate_name.lower()
    return {
        "candidate_name": candidate_name,
        "is_robot_name": normalized_name in ROBOT_NAME_ALIASES,
    }
