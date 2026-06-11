import json
import re
from typing import Dict


FALLBACK_MESSAGE = "죄송해요. 응답을 정리하다가 잠시 헷갈렸어요. 다시 말씀해 주세요."

DEFAULT_CLASSIFIER_FAILURE = {
    "intent": "unknown",
    "needs_motion": False,
}

DEFAULT_PLANNER_FAILURE = {
    "skills": [],
    "op_cmd": [],
    "speech": FALLBACK_MESSAGE,
    "reason": "",
}


def sanitize_message(message) -> str:
    if not isinstance(message, str):
        return FALLBACK_MESSAGE

    clean_msg = re.sub(r"\([^)]*\)", "", message)
    sanitized = re.sub(r"\s+", " ", clean_msg).strip()
    return sanitized or FALLBACK_MESSAGE


def build_classifier_failure_result() -> Dict:
    return dict(DEFAULT_CLASSIFIER_FAILURE)


def build_planner_failure_result(reason: str = "") -> Dict:
    result = dict(DEFAULT_PLANNER_FAILURE)
    if isinstance(reason, str):
        result["reason"] = reason.strip()
    return result


def build_llm_call_failure_json(reason: str) -> str:
    detail = reason if isinstance(reason, str) else str(reason)
    return json.dumps(
        {
            "intent": DEFAULT_CLASSIFIER_FAILURE["intent"],
            "needs_motion": DEFAULT_CLASSIFIER_FAILURE["needs_motion"],
            "skills": [],
            "op_cmd": [],
            "commands": [],
            "speech": FALLBACK_MESSAGE,
            "message": FALLBACK_MESSAGE,
            "reason": detail,
            "thinking": detail,
        },
        ensure_ascii=False,
    )

# NOTE: build_motion_block_message 는 제거됐다. 막힘 상태(safety_key/playing/error/moving)
# 설명·되묻기는 이제 planner 가 robot_state.block_reason 을 보고 speech 로 직접 생성하고,
# planner 명령이 전부 거부되는 희귀 케이스는 validator 의 SAFETY_NET_FALLBACK 이 덮는다.
