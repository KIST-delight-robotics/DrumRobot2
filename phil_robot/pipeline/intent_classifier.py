import json
import re

try:
    from .failure import build_classifier_failure_result
    from .state_adapter import build_classifier_state_summary, detect_joint_angle_query
except (ImportError, ValueError):
    from pipeline.failure import build_classifier_failure_result
    from pipeline.state_adapter import build_classifier_state_summary, detect_joint_angle_query

try:
    from ..config import CLASSIFIER_MODEL
except (ImportError, ValueError):
    from config import CLASSIFIER_MODEL

DEFAULT_INTENT_RESULT = build_classifier_failure_result()

MOTION_REQUIRED_INTENTS = {"motion_request", "play_request", "stop_request"}
IDENTITY_CHAT_KEYWORDS = ["이름", "누구", "정체", "자기소개"]
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

출력 스키마:
{
  "intent": "chat | motion_request | play_request | status_question | stop_request | unknown",
  "needs_motion": true,
  "needs_dialogue": true,
  "risk_level": "low | medium | high"
}

분류 기준:
- chat: 일반 대화, 인사, 감정 표현, 상식 질문, 이름/정체/자기소개 질문
- motion_request: 손/팔/허리/손목/시선/제스처 등 물리 동작 요청
- play_request: 연주 시작/곡 재생/드럼 연주 요청
- status_question: 현재 상태, 직전 행동, 왜 멈췄는지, 무엇을 했는지 질문
- stop_request: 멈춰, 그만, 정지, 종료 요청
- unknown: 의도를 분명히 정할 수 없는 경우

판단 규칙:
- 물리 동작이 필요하면 needs_motion=true
- 사용자에게 말로 응답해야 하면 needs_dialogue=true
- 안전/상태 제약이 강하게 얽히거나 물리 동작이면 risk_level 을 낮게 잡지 말고 최소 medium 이상을 고려한다.
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
        return build_classifier_failure_result()

    if not isinstance(response_data, dict):
        return build_classifier_failure_result()

    result = build_classifier_failure_result()
    if isinstance(response_data.get("intent"), str):
        result["intent"] = response_data["intent"].strip() or DEFAULT_INTENT_RESULT["intent"]
    if isinstance(response_data.get("needs_motion"), bool):
        result["needs_motion"] = response_data["needs_motion"]
    if isinstance(response_data.get("needs_dialogue"), bool):
        result["needs_dialogue"] = response_data["needs_dialogue"]
    if isinstance(response_data.get("risk_level"), str):
        result["risk_level"] = response_data["risk_level"].strip() or DEFAULT_INTENT_RESULT["risk_level"]

    return result


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

    if any(keyword in normalized_text for keyword in IDENTITY_CHAT_KEYWORDS):
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


def is_ambiguous_follow_up(user_text):
    """
    history 없이 해석하기 어려운 초단문 후속 발화를 감지한다.
    이런 입력은 planner 자유 생성보다 clarification 으로 처리하는 편이 안전하다.
    """
    normalized_text = (user_text or "").strip()
    condensed_text = re.sub(r"[\s\?\!\.\,~]+", "", normalized_text)
    return condensed_text in AMBIGUOUS_FOLLOW_UPS
