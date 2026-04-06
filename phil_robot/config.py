"""
phil_robot 공용 설정 상수.

엔트리포인트(`phil_brain.py`)와 하위 pipeline 계층이 함께 참조하는 값을
한 곳에 모아 의존성 방향을 단순하게 유지한다.
"""

import os
from typing import Any, Dict

CLASSIFIER_MODEL = "qwen3:4b-instruct-2507-q4_K_M"
PLANNER_MODEL = "qwen3:30b-a3b-instruct-2507-q4_K_M"


def env_int(key_name: str, default_val: int) -> int:
    raw_val = os.getenv(key_name)
    if raw_val is None:
        return default_val
    try:
        return int(raw_val)
    except ValueError:
        return default_val


def env_bool(key_name: str, default_val: bool) -> bool:
    raw_val = os.getenv(key_name)
    if raw_val is None:
        return default_val

    text_val = raw_val.strip().lower()
    if text_val in {"1", "true", "yes", "on"}:
        return True
    if text_val in {"0", "false", "no", "off"}:
        return False
    return default_val


def env_keep(key_name: str, default_val: str) -> Any:
    raw_val = os.getenv(key_name, default_val).strip()
    try:
        return float(raw_val)
    except ValueError:
        return raw_val


def build_chat_opt(num_ctx: int) -> Dict[str, Any]:
    return {
        "temperature": 0,
        "num_ctx": num_ctx,
    }


OLLAMA_THINK = env_bool("PHIL_OLLAMA_THINK", False)
OLLAMA_KEEP_ALIVE = env_keep("qwen3:30b-a3b-q4_K_M", "-1")

CLASSIFIER_NUM_CTX = env_int("PHIL_CLASSIFIER_NUM_CTX", 1024)

PLANNER_NUM_CTX = env_int("PHIL_PLANNER_NUM_CTX", 2048)

CLASSIFIER_CHAT_OPT = build_chat_opt(CLASSIFIER_NUM_CTX)
PLANNER_CHAT_OPT = build_chat_opt(PLANNER_NUM_CTX)
