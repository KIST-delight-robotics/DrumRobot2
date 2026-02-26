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

    # 1. 구분자 '>>' 기준으로 명령어 파트와 메시지 파트 분리
    if ">>" in ai_text:
        parts = ai_text.split(">>", 1) # 첫 번째 '>>'에서만 자름
        cmd_part = parts[0].strip()
        message = parts[1].strip()
        
        # 2. 정규표현식으로 [CMD:...] 패턴 안의 내용만 모두 추출
        # r"\[CMD:(.*?)\]" : [CMD: 로 시작하고 ] 로 끝나는 내부 문자열을 찾음
        commands = re.findall(r"\[CMD:(.*?)\]", cmd_part)
        
    return commands, message

# (테스트용 코드 - 직접 실행 시에만 작동)
if __name__ == "__main__":
    test_text = "[CMD:look:0,0][CMD:gesture:wave] >> 안녕! 반가워."
    cmds, msg = parse_llm_response(test_text)
    print(f"명령어: {cmds}")
    print(f"메시지: {msg}")