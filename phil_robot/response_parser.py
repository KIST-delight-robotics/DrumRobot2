import re

def parse_llm_response(ai_text):
    """
    LLM의 응답 텍스트를 분석하여 명령어 리스트와 발화 메시지를 분리 반환합니다.
    
    Args:
        ai_text (str): LLM이 생성한 원본 문자열 
                       예) "[CMD:look:30,10][CMD:gesture:wave] >> 안녕하세요!"
    
    Returns:
        tuple: (commands_list, message_text)
               - commands_list: 명령어 문자열 리스트 (예: ["look:30,10", "gesture:wave"])
               - message_text: 로봇이 말할 텍스트 (예: "안녕하세요!")
    """
    commands = []
    message = ai_text
    thinking_log = ""

    # 1. <Thinking> 태그 추출 및 제거 (TTS로 읽지 않도록)
    thinking_match = re.search(r'<Thinking>(.*?)</Thinking>', ai_text, re.DOTALL)
    if thinking_match:
        thinking_log = thinking_match.group(1).strip()
        # 메시지 파싱을 위해 원본 텍스트에서 Thinking 블록을 완전히 날림
        ai_text = re.sub(r'<Thinking>.*?</Thinking>', '', ai_text, flags=re.DOTALL).strip()

    # 2. 구분자 '>>' 기준으로 명령어 파트와 메시지 파트 분리
    if ">>" in ai_text:
        parts = ai_text.split(">>", 1) # 첫 번째 '>>'에서만 자름
        cmd_part = parts[0].strip()
        message = parts[1].strip()
        
        # 정규표현식으로 [CMD:...] 패턴 안의 내용만 모두 추출
        # r"\[CMD:(.*?)\]" : [CMD: 로 시작하고 ] 로 끝나는 내부 문자열을 찾음
        extracted_cmds = re.findall(r"\[CMD:(.*?)\]", cmd_part)
        
        valid_leds = ["happy", "thinking", "angry", "idle", "play"]
        valid_gestures = ["hi", "nod", "shake", "wave", "hurray", "happy"]

        # led 환각 필터링
        for cmd in extracted_cmds:
            cmd_parts= cmd.split(":")
            cmd_type = cmd_parts[0].strip().lower() # 명령어 타입 (예: "led", "gesture")

            if cmd_type == "led" and len(cmd_parts) > 1 and cmd_parts[1] not in valid_leds:
                print(f"[Warning] 무효한 LED 명령어 차단됨: {cmd}")
                continue
            # gesture 환각 필터링
            elif cmd_type == "gesture" and len(cmd_parts) > 1 and cmd_parts[1] not in valid_gestures:
                print(f"[Warning] 무효한 Gesture 차단됨: {cmd}")
                continue

            commands.append(cmd)

    else:
    # 명령어가 없고 '>>'도 없는 순수 대화일 경우
        message = ai_text.strip()        
    
    # 4. TTS용 텍스트 정제 (Sanitization)
    # 괄호 '(' 와 ')' 및 그 안의 내용 모두 제거
    clean_msg = re.sub(r'\([^)]*\)', '', message)
    message = re.sub(r'\s+', ' ', clean_msg).strip()
    
    return commands, message, thinking_log

# (테스트용 코드 - 직접 실행 시에만 작동)
if __name__ == "__main__":
    test_text = """<Thinking>
1. 의도: 반가움을 표시하는 인사.
2. 상태 검증: 제어 가능.
3. 계획: 손을 흔들며 환각 명령어(led:beat)를 섞어봄.
</Thinking>
[CMD:look:0,0][CMD:gesture:wave][CMD:led:beat] >> 안녕! (반갑게 손을 흔들며) 만나서 정말 반가워요! 🥁🔥"""

    cmds, msg, thinking = parse_llm_response(test_text)
    
    print("=== [Phil's Brain Log] ===")
    print(thinking)
    print("\n=== [C++ Socket 전송용] ===")
    print(f"명령어: {cmds}")
    print("\n=== [MeloTTS 출력용] ===")
    print(f"메시지: {msg}")