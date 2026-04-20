"""
phil_robot 공용 설정 상수.

엔트리포인트(`phil_brain.py`)와 하위 pipeline 계층이 함께 참조하는 값을
한 곳에 모아 의존성 방향을 단순하게 유지한다.
"""

import os
from typing import Any, Dict


def _load_dotenv() -> None:
    env_path = os.path.join(os.path.dirname(__file__), ".env")
    if not os.path.isfile(env_path):
        return
    with open(env_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, val = line.partition("=")
            key = key.strip()
            val = val.strip().strip('"').strip("'")
            if key and key not in os.environ:
                os.environ[key] = val


_load_dotenv()

CLASSIFIER_MODEL = os.getenv("PHIL_CLASSIFIER_MODEL", "gpt-4o-mini")
PLANNER_MODEL = os.getenv("PHIL_PLANNER_MODEL", "gpt-4o-mini")

OPENAI_API_KEY = os.getenv("OM_API_KEY", "")

_OPENAI_PREFIXES = ("gpt-", "o1", "o3", "o4", "o5", "text-")


def detect_backend(model_name: str) -> str:
    lower = model_name.lower()
    for prefix in _OPENAI_PREFIXES:
        if lower.startswith(prefix):
            return "openai"
    return "ollama"


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
