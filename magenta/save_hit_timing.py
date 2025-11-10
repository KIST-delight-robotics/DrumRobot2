import mido
import datetime
import time
import sys
import os

# --- 1. 사용 가능한 MIDI 장치 목록 출력 ---
print("사용 가능한 MIDI 입력 장치:")
port_names = mido.get_input_names()

if not port_names:
    print("연결된 MIDI 장치가 없습니다. 스크립트를 종료합니다.")
    print("전자 드럼이 PC에 연결되어 있고 전원이 켜져 있는지 확인하세요.")
    sys.exit()
    
for port in port_names:
    print(f"- {port}")

# --- 2. [중요] 사용자 설정 ---
# !!! 이 아래 변수 값을 사용자의 전자 드럼 포트 이름으로 변경하세요 !!!
# 예: PORT_NAME = "TD-07
#  1" 또는 PORT_NAME = "USB MIDI Interface"
PORT_NAME = "TD-17:TD-17 MIDI 1 20:0"

# 저장될 로그 파일 이름 설정
LOG_FILE_NAME = "drum_log.txt"
# ---------------------------------

if PORT_NAME == "Your_MIDI_Port_Name_Here":
    print(f"\n[경고] 코드 28번째 줄의 'PORT_NAME' 변수 값을")
    print("       위 목록에 있는 실제 장치 이름으로 변경한 후 다시 실행해주세요.")
    sys.exit()

try:
    # `with` 구문을 사용하면 스크립트 종료 시 포트와 파일이 자동으로 닫힙니다.
    # mido.open_input()으로 MIDI 포트 열기
    # open()으로 로그 파일 열기 (모드 'a' = append, 이어쓰기)
    with mido.open_input(PORT_NAME) as midi_input, open(LOG_FILE_NAME, 'a', encoding='utf-8') as log_file:
        
        print(f"\n[알림] '{PORT_NAME}' 장치에서 입력을 대기합니다...")
        print(f"[알림] 모든 타격 로그를 '{LOG_FILE_NAME}' 파일에 저장합니다.")
        print("첫 번째 타격부터 모든 타격 시간을 기록합니다. (종료: Ctrl+C)")
        
        # 프로그램 시작 시 파일에 구분선 추가
        start_time_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_file.write(f"\n--- 로그 기록 시작: {start_time_str} ---\n")
        log_file.flush() # 버퍼를 비워 파일에 즉시 쓰도록 함

        # 무한 루프를 돌며 MIDI 메시지 확인
        while True:
            
            # midi_input.iter_pending() : 입력 버퍼에 쌓인 모든 메시지를 순회
            for msg in midi_input.iter_pending():
                
                # 'note_on' 메시지이고, velocity(강도)가 0보다 클 때만 (타격으로 간주)
                if msg.type == 'note_on' and msg.velocity > 0:
                    
                    # 1. 현재 시간 가져오기
                    now = datetime.datetime.now()
                    
                    # 2. 시간 포맷팅 (시:분:초.밀리초)
                    timestamp = now.strftime("%H:%M:%S.%f")[:-3]
                    
                    # 3. 로그 문자열 생성 (매핑 번호, 시간)
                    log_entry = f"{msg.note}, {timestamp}"
                    
                    # 4. 콘솔에 출력
                    print(f"타격 감지! -> {log_entry} (강도: {msg.velocity})")
                    
                    # 5. 파일에 저장 (줄바꿈 \n 추가)
                    log_file.write(log_entry + "\n")
                    log_file.flush() # 버퍼를 비워 파일에 즉시 쓰도록 함
            
            # CPU 과다 사용 방지를 위한 짧은 대기
            time.sleep(0.001)

except OSError:
    print(f"\n[오류] '{PORT_NAME}' 포트를 열 수 없습니다.")
    print("1. PORT_NAME 변수가 위 목록의 이름과 정확히 일치하는지 확인하세요.")
    print("2. 전자 드럼이 컴퓨터에 연결되어 있고 전원이 켜져 있는지 확인하세요.")
except KeyboardInterrupt:
    print(f"\n[알림] 사용자가 프로그램을 종료했습니다. '{LOG_FILE_NAME}' 저장이 완료되었습니다.")
    print(f"저장 경로: {os.path.abspath(LOG_FILE_NAME)}")