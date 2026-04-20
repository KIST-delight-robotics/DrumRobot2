import json
import re

try:
    from .failure import build_classifier_failure_result
    from .state_adapter import build_classifier_state_summary, detect_joint_angle_query, detect_repertoire_query
except (ImportError, ValueError):
    from pipeline.failure import build_classifier_failure_result
    from pipeline.state_adapter import build_classifier_state_summary, detect_joint_angle_query, detect_repertoire_query

try:
    from ..config import CLASSIFIER_MODEL
except (ImportError, ValueError):
    from config import CLASSIFIER_MODEL

DEFAULT_INTENT_RESULT = build_classifier_failure_result()

MOTION_REQUIRED_INTENTS = {"motion_request", "play_request", "stop_request"}
INTENT_CODE_MAP = {
    "C": "chat",
    "M": "motion_request",
    "P": "play_request",
    "Q": "status_question",
    "X": "stop_request",
    "U": "unknown",
}
RISK_CODE_MAP = {
    "L": "low",
    "M": "medium",
    "H": "high",
}
IDENTITY_CHAT_KEYWORDS = ["이름", "누구", "정체", "자기소개"]
IDENTITY_CONFIRMATION_PATTERNS = [
    re.compile(r"맞(?:지|죠|니|나요)\s*[?!.\s]*$"),
    re.compile(r"(?:이니|인가|인가요)\s*[?!.\s]*$"),
]
PLAY_ACTION_KEYWORDS = [
    "연주",
    "재생",
    "틀어",
    "쳐",
    "치자",
    "시작해",
    "시작해줘",
]
PLAY_SONG_KEYWORDS = [
    "this is me",
    "그대에게",
    "baby i need you",
    "test beat",
    "test_one",
    "tim",
    "ty_short",
    "bi",
]
READY_POSE_TEXTS = {
    "준비",
    "준비해",
    "준비해줘",
    "준비해주세요",
    "준비자세",
    "준비자세해",
    "준비자세해줘",
    "준비자세로가",
    "준비자세로가줘",
}
MOTION_CHAT_KEYWORDS = [
    "손",
    "팔",
    "손목",
    "허리",
    "고개",
    "시선",
    "흔들",
    "움직",
    "들어",
    "돌려",
    "봐",
    "만세",
    "인사",
]
ANGLE_STATUS_PATTERN = re.compile(r"(각도|몇\s*도|몇도)")
AMBIGUOUS_FOLLOW_UPS = {
    "왜",
    "왜요",
    "왜지",
    "뭐",
    "뭐지",
    "응",
    "응왜",
}

CLASSIFIER_SYSTEM_PROMPT = """당신은 로봇 에이전트의 1차 intent classifier 다.
반드시 JSON 객체 하나만 출력한다. 설명문, 코드블록, 마크다운은 절대 출력하지 않는다.
가능하면 공백 없는 한 줄 JSON 으로 출력한다.

출력 스키마:
{"i":"C|M|P|Q|X|U","m":1,"d":1,"r":"L|M|H"}

코드 의미:
- i: intent
  - C = chat
  - M = motion_request
  - P = play_request
  - Q = status_question
  - X = stop_request
  - U = unknown
- m: motion 필요 여부. 1 또는 0 만 사용한다.
- d: dialogue 필요 여부. 1 또는 0 만 사용한다.
- r: risk_level
  - L = low
  - M = medium
  - H = high

분류 기준:
- chat: 일반 대화, 인사, 감정 표현, 상식 질문, 이름/정체/자기소개 질문
- motion_request: 손/팔/허리/손목/시선/제스처 등 물리 동작 요청
- play_request: 연주 시작/곡 재생/드럼 연주 요청
- status_question: 현재 상태, 직전 행동, 왜 멈췄는지, 무엇을 했는지 질문
- stop_request: 멈춰, 그만, 정지, 종료, 일시정지, 잠깐, 스톱 요청 또는 연주 재개(다시 해, 계속 해, 이어서 해) 요청
- unknown: 의도를 분명히 정할 수 없는 경우

판단 규칙:
- 물리 동작이 필요하면 m=1
- 사용자에게 말로 응답해야 하면 d=1
- 안전/상태 제약이 강하게 얽히거나 물리 동작이면 r 을 낮게 잡지 말고 최소 M 이상을 고려한다.
- "준비", "준비 자세"처럼 짧은 자세 전환 명령은 motion_request 로 보고 m=1 로 둔다.
- "무슨 노래 연주할 수 있니?", "연주할 수 있는 곡이 뭐야?" 같은 곡 목록/레퍼토리 질문은 play_request 가 아니라 chat 로 보고 m=0 으로 둔다.
- 이름/정체를 확인하면서 "맞지?", "~이니?"처럼 예/아니오를 몸으로 같이 보여줘야 하는 질문은 motion_request 로 보고 m=1 로 둔다.
"""


def build_classifier_input_json(robot_state, user_text):
    """
    classifier 에 넘길 입력 JSON 문자열을 만든다.
    planner보다 단순한 상태 요약만 넣어 의도 분류에 집중시킨다.
    이 단계에서는 전체 관절각보다 상태/바쁨/최근 행동이 더 중요하다.
    """
    summary = build_classifier_state_summary(robot_state)

    return json.dumps(
        {
            "system_info": summary,
            "user_text": user_text,
        },
        ensure_ascii=False,
        indent=2,
    )


def parse_intent_response(response_text):
    """
    classifier 출력은 planner 앞단에서 바로 쓰이므로 실패 시 보수적 기본값을 사용한다.
    """
    if not isinstance(response_text, str):
        return build_classifier_failure_result()

    try:
        response_data = json.loads(response_text)
    except json.JSONDecodeError:
        return extract_partial_intent_response(response_text)

    if not isinstance(response_data, dict):
        return build_classifier_failure_result()

    return parse_intent_payload(response_data)


def parse_intent_payload(response_data):
    result = build_classifier_failure_result()

    raw_intent = read_str_field(response_data, "intent", "i")
    if raw_intent:
        result["intent"] = decode_intent(raw_intent)

    raw_motion = read_bool_field(response_data, "needs_motion", "m")
    if raw_motion is not None:
        result["needs_motion"] = raw_motion

    raw_dialogue = read_bool_field(response_data, "needs_dialogue", "d")
    if raw_dialogue is not None:
        result["needs_dialogue"] = raw_dialogue

    raw_risk = read_str_field(response_data, "risk_level", "r")
    if raw_risk:
        result["risk_level"] = decode_risk(raw_risk)

    return result


def read_str_field(response_data, long_key, short_key):
    raw_value = response_data.get(long_key)
    if not isinstance(raw_value, str):
        raw_value = response_data.get(short_key)
    if not isinstance(raw_value, str):
        return ""
    return raw_value.strip()


def read_bool_field(response_data, long_key, short_key):
    raw_value = response_data.get(long_key)
    if raw_value is None:
        raw_value = response_data.get(short_key)

    if isinstance(raw_value, bool):
        return raw_value
    if isinstance(raw_value, int):
        if raw_value in {0, 1}:
            return bool(raw_value)
        return None
    if isinstance(raw_value, str):
        raw_text = raw_value.strip().upper()
        if raw_text in {"1", "TRUE", "T"}:
            return True
        if raw_text in {"0", "FALSE", "F"}:
            return False
    return None


def decode_intent(raw_intent):
    upper_text = raw_intent.upper()
    return INTENT_CODE_MAP.get(upper_text, raw_intent or DEFAULT_INTENT_RESULT["intent"])


def decode_risk(raw_risk):
    upper_text = raw_risk.upper()
    return RISK_CODE_MAP.get(upper_text, raw_risk.lower() or DEFAULT_INTENT_RESULT["risk_level"])


def extract_partial_intent_response(response_text):
    """
    모델 출력이 중간에서 끊겨도
    앞부분의 핵심 필드는 최대한 회수한다.
    """
    if not isinstance(response_text, str):
        return build_classifier_failure_result()

    partial_data = {}

    intent_match = re.search(r'"(?:intent|i)"\s*:\s*"([^"]+)"', response_text, re.IGNORECASE)
    if intent_match:
        partial_data["i"] = intent_match.group(1).strip()

    motion_match = re.search(r'"(?:needs_motion|m)"\s*:\s*(true|false|0|1|"0"|"1"|"T"|"F")', response_text, re.IGNORECASE)
    if motion_match:
        partial_data["m"] = motion_match.group(1).strip().strip('"')

    dialogue_match = re.search(r'"(?:needs_dialogue|d)"\s*:\s*(true|false|0|1|"0"|"1"|"T"|"F")', response_text, re.IGNORECASE)
    if dialogue_match:
        partial_data["d"] = dialogue_match.group(1).strip().strip('"')

    risk_match = re.search(r'"(?:risk_level|r)"\s*:\s*"([^"]+)"', response_text, re.IGNORECASE)
    if risk_match:
        partial_data["r"] = risk_match.group(1).strip()

    if partial_data:
        return parse_intent_payload(partial_data)

    return build_classifier_failure_result()


def normalize_intent_result(intent_result, user_text):
    """
    classifier 결과를 코드에서 한 번 더 정규화한다.
    이름/정체 질문처럼 반복되는 오분류는 deterministic rule 로 보정한다.
    """
    result = dict(intent_result or DEFAULT_INTENT_RESULT)
    normalized_text = (user_text or "").strip()

    # [WARNING] 대화 history 가 없는 현재 구조에서는 "왜?", "뭐?" 같은 초단문을
    # 상태 질문으로 해석하면 근거 없는 이유를 지어낼 가능성이 크다.
    if is_ambiguous_follow_up(normalized_text):
        result["intent"] = "unknown"
        result["needs_motion"] = False
        result["needs_dialogue"] = True
        result["risk_level"] = "low"
        return result

    if detect_repertoire_query(normalized_text):
        result["intent"] = "chat"
        result["needs_motion"] = False
        result["needs_dialogue"] = True
        result["risk_level"] = "low"
        return result

    if looks_like_ready_pose_request(normalized_text):
        result["intent"] = "motion_request"
        result["needs_motion"] = True
        result["needs_dialogue"] = True
        if result.get("risk_level") == "low":
            result["risk_level"] = "medium"
    elif looks_like_identity_confirmation_motion(normalized_text):
        result["intent"] = "motion_request"
        result["needs_motion"] = True
        result["needs_dialogue"] = True
        if result.get("risk_level") == "low":
            result["risk_level"] = "medium"
    elif any(keyword in normalized_text for keyword in IDENTITY_CHAT_KEYWORDS):
        result["intent"] = "chat"
        result["needs_motion"] = False
        result["needs_dialogue"] = True
        if not result.get("risk_level"):
            result["risk_level"] = "low"

    if detect_joint_angle_query(normalized_text) or ANGLE_STATUS_PATTERN.search(normalized_text):
        result["intent"] = "status_question"
        result["needs_motion"] = False
        result["needs_dialogue"] = True
        if not result.get("risk_level"):
            result["risk_level"] = "low"

    if result["intent"] in {"unknown", "chat"} and looks_like_play_request(normalized_text):
        result["intent"] = "play_request"
        result["needs_motion"] = True
        result["needs_dialogue"] = True
        if result.get("risk_level") == "low":
            result["risk_level"] = "medium"

    # greeting 이 섞여 있어도 실제 물리 동작 요청이 보이면 motion_request 로 승격한다.
    if (
        result["intent"] == "chat"
        and result.get("needs_motion", False)
        and any(keyword in normalized_text for keyword in MOTION_CHAT_KEYWORDS)
    ):
        result["intent"] = "motion_request"
        result["needs_motion"] = True
        result["needs_dialogue"] = True
        if result.get("risk_level") == "low":
            result["risk_level"] = "medium"

    # classifier 가 intent 는 맞췄지만 motion flag 를 놓치는 경우를 코드에서 보정한다.
    if result["intent"] in MOTION_REQUIRED_INTENTS:
        result["needs_motion"] = True

    return result


def looks_like_play_request(user_text):
    normalized_text = (user_text or "").strip().lower()
    if not normalized_text:
        return False

    if any(keyword in normalized_text for keyword in PLAY_ACTION_KEYWORDS):
        return True

    has_song_keyword = any(keyword in normalized_text for keyword in PLAY_SONG_KEYWORDS)
    has_request_suffix = any(keyword in normalized_text for keyword in ["해줘", "해", "주세요"])
    if has_song_keyword and has_request_suffix:
        return True

    return False


def is_ambiguous_follow_up(user_text):
    """
    history 없이 해석하기 어려운 초단문 후속 발화를 감지한다.
    이런 입력은 planner 자유 생성보다 clarification 으로 처리하는 편이 안전하다.
    """
    normalized_text = (user_text or "").strip()
    condensed_text = re.sub(r"[\s\?\!\.\,~]+", "", normalized_text)
    return condensed_text in AMBIGUOUS_FOLLOW_UPS


def looks_like_identity_confirmation_motion(user_text):
    normalized_text = (user_text or "").strip()
    if not normalized_text:
        return False

    if not any(keyword in normalized_text for keyword in IDENTITY_CHAT_KEYWORDS):
        return False

    return any(pattern.search(normalized_text) for pattern in IDENTITY_CONFIRMATION_PATTERNS)


def looks_like_ready_pose_request(user_text):
    normalized_text = (user_text or "").strip()
    condensed_text = re.sub(r"[\s\?\!\.\,~]+", "", normalized_text)
    return condensed_text in READY_POSE_TEXTS
