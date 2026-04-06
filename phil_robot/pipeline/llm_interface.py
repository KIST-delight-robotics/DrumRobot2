import time
from typing import Any, Dict, Optional

import ollama

try:
    from ..config import (
        CLASSIFIER_CHAT_OPT,
        CLASSIFIER_MODEL,
        OLLAMA_KEEP_ALIVE,
        OLLAMA_THINK,
        PLANNER_CHAT_OPT,
        PLANNER_MODEL,
    )
    from .failure import build_llm_call_failure_json
except (ImportError, ValueError):
    from config import (
        CLASSIFIER_CHAT_OPT,
        CLASSIFIER_MODEL,
        OLLAMA_KEEP_ALIVE,
        OLLAMA_THINK,
        PLANNER_CHAT_OPT,
        PLANNER_MODEL,
    )
    try:
        from pipeline.failure import build_llm_call_failure_json
    except ImportError:
        from failure import build_llm_call_failure_json


def read_meta_value(resp: Any, key_name: str) -> Any:
    if isinstance(resp, dict):
        return resp.get(key_name)
    if hasattr(resp, key_name):
        return getattr(resp, key_name)
    return None


def ns_to_sec(time_ns: Any) -> Optional[float]:
    if not isinstance(time_ns, int):
        return None
    if time_ns < 0:
        return None
    return float(time_ns) / 1_000_000_000.0


def calc_token_rate(tok_count: Any, time_ns: Any) -> Optional[float]:
    time_sec = ns_to_sec(time_ns)
    if not isinstance(tok_count, int):
        return None
    if time_sec is None or time_sec <= 0:
        return None
    return float(tok_count) / time_sec


def pick_chat_opt(model_name: str) -> Dict[str, Any]:
    if model_name == CLASSIFIER_MODEL:
        return dict(CLASSIFIER_CHAT_OPT)
    if model_name == PLANNER_MODEL:
        return dict(PLANNER_CHAT_OPT)
    return dict(PLANNER_CHAT_OPT)


def build_chat_req(model_name: str, system_prompt: str, user_input_json: str) -> Dict[str, Any]:
    return {
        "model": model_name,
        "messages": [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_input_json},
        ],
        "format": "json",
        "think": OLLAMA_THINK,
        "keep_alive": OLLAMA_KEEP_ALIVE,
        "options": pick_chat_opt(model_name),
    }


def build_chat_metrics(resp: Any, wall_sec: float, err_msg: str = "") -> Dict[str, Any]:
    load_ns = read_meta_value(resp, "load_duration")
    prompt_tok = read_meta_value(resp, "prompt_eval_count")
    prompt_ns = read_meta_value(resp, "prompt_eval_duration")
    eval_tok = read_meta_value(resp, "eval_count")
    eval_ns = read_meta_value(resp, "eval_duration")

    load_sec = ns_to_sec(load_ns)
    prompt_sec = ns_to_sec(prompt_ns)
    eval_sec = ns_to_sec(eval_ns)

    infer_sec = None
    if prompt_sec is not None and eval_sec is not None:
        infer_sec = prompt_sec + eval_sec

    meta_sec = None
    if load_sec is not None and infer_sec is not None:
        meta_sec = load_sec + infer_sec
    elif infer_sec is not None:
        meta_sec = infer_sec

    over_sec = None
    if meta_sec is not None:
        over_sec = max(wall_sec - meta_sec, 0.0)

    return {
        "wall_sec": wall_sec,
        "load_sec": load_sec,
        "prompt_tokens": prompt_tok if isinstance(prompt_tok, int) else None,
        "prompt_sec": prompt_sec,
        "prompt_tps": calc_token_rate(prompt_tok, prompt_ns),
        "eval_tokens": eval_tok if isinstance(eval_tok, int) else None,
        "eval_sec": eval_sec,
        "eval_tps": calc_token_rate(eval_tok, eval_ns),
        "infer_sec": infer_sec,
        "meta_sec": meta_sec,
        "overhead_sec": over_sec,
        "error": err_msg,
    }


def call_json_llm(model_name, system_prompt, user_input_json, capture_metrics: bool = False):
    """
    현재는 단일 JSON 호출 래퍼다.
    이후 intent 분류 호출과 planner 호출을 분리할 때 이 파일이 공통 입구가 된다.
    """
    start_sec = time.time()
    try:
        response = ollama.chat(**build_chat_req(model_name, system_prompt, user_input_json))
        raw_text = response["message"]["content"]
        if not capture_metrics:
            return raw_text

        wall_sec = time.time() - start_sec
        return raw_text, build_chat_metrics(response, wall_sec)
    except Exception as exc:
        err_msg = f"LLM call failed: {exc}"
        raw_text = build_llm_call_failure_json(err_msg)
        if not capture_metrics:
            return raw_text

        wall_sec = time.time() - start_sec
        return raw_text, build_chat_metrics({}, wall_sec, err_msg)
