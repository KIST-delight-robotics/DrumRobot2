import json

FALLBACK_MESSAGE = "죄송해요. 응답을 정리하다가 잠시 헷갈렸어요. 다시 말씀해 주세요."


def build_fallback_payload(reason):
    """
    LLM 호출이 실패했을 때 공통적으로 쓰는 fallback JSON.
    planner, parser 어느 단계에서든 동일한 계약으로 복구할 수 있게 유지한다.
    """
    return json.dumps(
        {
            "commands": [],
            "message": FALLBACK_MESSAGE,
            "thinking": reason,
        },
        ensure_ascii=False,
    )
