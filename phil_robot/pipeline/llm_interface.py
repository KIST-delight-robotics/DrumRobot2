import ollama

try:
    from .failure import build_llm_call_failure_json
except ImportError:
    from failure import build_llm_call_failure_json


def call_json_llm(model_name, system_prompt, user_input_json):
    """
    현재는 단일 JSON 호출 래퍼다.
    이후 intent 분류 호출과 planner 호출을 분리할 때 이 파일이 공통 입구가 된다.
    """
    try:
        response = ollama.chat(
            model=model_name,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_input_json},
            ],
            format="json",
        )
        return response["message"]["content"]
    except Exception as exc:
        return build_llm_call_failure_json(f"LLM call failed: {exc}")
