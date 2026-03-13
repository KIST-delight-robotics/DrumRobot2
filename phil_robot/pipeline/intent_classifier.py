import json

try:
    from .state_adapter import build_classifier_state_summary
except ImportError:
    from state_adapter import build_classifier_state_summary

CLASSIFIER_MODEL = "qwen3:4b-instruct-2507-q4_K_M"

DEFAULT_INTENT_RESULT = {
    "intent": "unknown",
    "needs_motion": False,
    "needs_dialogue": True,
    "risk_level": "medium",
}

MOTION_REQUIRED_INTENTS = {"motion_request", "play_request", "stop_request"}
IDENTITY_CHAT_KEYWORDS = ["이름", "누구", "정체", "자기소개"]

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


def build_classifier_payload(robot_state, user_text):
    """
    classifier 는 planner보다 단순한 상태 요약만 받는다.
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


def parse_intent_response(ai_text):
    """
    classifier 출력은 planner 앞단에서 바로 쓰이므로 실패 시 보수적 기본값을 사용한다.
    """
    if not isinstance(ai_text, str):
        return dict(DEFAULT_INTENT_RESULT)

    try:
        payload = json.loads(ai_text)
    except json.JSONDecodeError:
        return dict(DEFAULT_INTENT_RESULT)

    if not isinstance(payload, dict):
        return dict(DEFAULT_INTENT_RESULT)

    result = dict(DEFAULT_INTENT_RESULT)
    if isinstance(payload.get("intent"), str):
        result["intent"] = payload["intent"].strip() or DEFAULT_INTENT_RESULT["intent"]
    if isinstance(payload.get("needs_motion"), bool):
        result["needs_motion"] = payload["needs_motion"]
    if isinstance(payload.get("needs_dialogue"), bool):
        result["needs_dialogue"] = payload["needs_dialogue"]
    if isinstance(payload.get("risk_level"), str):
        result["risk_level"] = payload["risk_level"].strip() or DEFAULT_INTENT_RESULT["risk_level"]

    return result


def normalize_intent_result(intent_result, user_text):
    """
    classifier 결과를 코드에서 한 번 더 정규화한다.
    이름/정체 질문처럼 반복되는 오분류는 deterministic rule 로 보정한다.
    """
    result = dict(intent_result or DEFAULT_INTENT_RESULT)
    normalized_text = (user_text or "").strip()

    if any(keyword in normalized_text for keyword in IDENTITY_CHAT_KEYWORDS):
        result["intent"] = "chat"
        result["needs_motion"] = False
        result["needs_dialogue"] = True
        if not result.get("risk_level"):
            result["risk_level"] = "low"

    # classifier 가 intent 는 맞췄지만 motion flag 를 놓치는 경우를 코드에서 보정한다.
    if result["intent"] in MOTION_REQUIRED_INTENTS:
        result["needs_motion"] = True

    return result
