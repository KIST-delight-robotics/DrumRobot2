import json

try:
    from .failure import FALLBACK_MESSAGE, sanitize_message
except ImportError:
    from failure import FALLBACK_MESSAGE, sanitize_message


def _sanitize_message(message):
    return sanitize_message(message)

def parse_llm_response(response_text):
    """
    JSON 형식의 LLM 응답을 분석하여 명령어 리스트와 발화 메시지를 분리 반환합니다.
    이 레이어는 형식 해석에 집중하고, 의미 검증은 validator가 맡습니다.
    """
    op_cmds = []
    thinking_log = ""
    message = FALLBACK_MESSAGE

    if not isinstance(response_text, str):
        print("[Warning] LLM 응답이 문자열이 아니어서 fallback 응답을 사용합니다.")
        return op_cmds, message, thinking_log

    try:
        response_data = json.loads(response_text)
    except json.JSONDecodeError as exc:
        print(f"[Warning] JSON 파싱 실패로 fallback 응답을 사용합니다: {exc}")
        return op_cmds, message, thinking_log

    if not isinstance(response_data, dict):
        print("[Warning] JSON 응답이 객체가 아니어서 fallback 응답을 사용합니다.")
        return op_cmds, message, thinking_log

    raw_op_cmds = response_data.get("op_cmd", response_data.get("commands", []))
    raw_message = response_data.get("message", FALLBACK_MESSAGE)
    raw_thinking = response_data.get("thinking", "")

    if isinstance(raw_thinking, str):
        thinking_log = raw_thinking.strip()

    message = _sanitize_message(raw_message)

    if not isinstance(raw_op_cmds, list):
        print("[Warning] op_cmd 필드가 리스트가 아니어서 빈 명령으로 처리합니다.")
        raw_op_cmds = []

    for cmd in raw_op_cmds:
        if not isinstance(cmd, str):
            print(f"[Warning] 문자열이 아닌 명령어를 건너뜁니다: {cmd}")
            continue

        normalized_cmd = cmd.strip()
        if not normalized_cmd:
            continue

        op_cmds.append(normalized_cmd)

    return op_cmds, message, thinking_log

# (테스트용 코드 - 직접 실행 시에만 작동)
if __name__ == "__main__":
    test_text = json.dumps({
        "op_cmd": ["look:0,0", "gesture:wave", "led:beat"],
        "message": "안녕! (반갑게 손을 흔들며) 만나서 정말 반가워요! 🥁🔥",
        "thinking": "1. 의도: 반가움을 표시하는 인사."
    }, ensure_ascii=False)

    cmds, msg, thinking = parse_llm_response(test_text)

    print("=== [Phil's Brain Log] ===")
    print(thinking)
    print("\n=== [C++ Socket 전송용] ===")
    print(f"명령어: {cmds}")
    print("\n=== [MeloTTS 출력용] ===")
    print(f"메시지: {msg}")
