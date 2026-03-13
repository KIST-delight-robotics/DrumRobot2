# plil_client.py
import socket
import time
import threading
import json

# 로봇의 상태를 기억할 전역 변수 (초기값)
ROBOT_STATE = {
    "state": 0,          # 0: Ideal, 2: Play, 4: Error 등
    "bpm": 100,          # 기본 템포
    "is_fixed": True,    # 기본값: 가만히 있음 (True)
    "current_song": "None",
    "last_action": "None",
    "is_lock_key_removed": False,  # 락키 제거 여부 (추가)
    "current_angles": {
        "waist": 0.0, "R_arm1": 45.0, "L_arm1": 45.0, 
        "R_arm2": 0.0, "R_arm3": 20.0, "L_arm2": 0.0, "L_arm3": 20.0,
        "R_wrist": 90.0, "L_wrist": 90.0
    }
}

class RobotClient:

    # 클라이언트 소켓 생성자
    def __init__(self, host, port):
        self.host=host
        self.port=port
        self.sock=None
        self.keep_receiving = False # 수신 스레드 제어용 깃발

    # 소켓 연결
    def connect(self):
        """로봇(C++) 서버에 연결될 때까지 재시도"""
        print(f"로봇 서버 ({self.host}:{self.port})에 연결 시도..")
        
        while True:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(5) # 타임아웃 5초 설정
                self.sock.connect((self.host, self.port))
                self.sock.settimeout(None)
                print(f"로봇 서버 ({self.host}:{self.port})에 연결되었습니다.")
                self.start_receiving() # 수신 스레드 시작
                return True

            except (socket.error, ConnectionRefusedError):
                self.sock.close()
                print("⏳ 로봇 서버 대기 중... (main.out 실행 후 'o'를 눌러주세요)")
                time.sleep(3)

    # 수신 데몬 스레드 
    def start_receiving(self):
        self.keep_receiving = True
        recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        recv_thread.start()
        print("📡 [잠재의식] 로봇 상태 수신 스레드 가동")
    
    # 소켓 수신
    def _receive_loop(self):
        global ROBOT_STATE
        buffer = ""
        last_printed_state = None # 마지막으로 출력한 상태 저장

        while self.keep_receiving:
            try:
                # 1. 소켓에서 데이터 읽기
                data = self.sock.recv(1024).decode('utf-8')
                if not data:
                    break
                
                buffer += data
                # 2. 줄바꿈(\n) 단위로 쪼개서 JSON 해석
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        print(f"👀 [Debug 수신] {line.strip()}") # 디버깅 용
                        try:
                            # 3. 전역 변수 업데이트
                            new_state = json.loads(line)
                            ROBOT_STATE.update(new_state)
                        
                            # 이전과 다르게 출력될 때만 상태 갱신 메시지 출력
                            if last_printed_state != ROBOT_STATE:
                                print(f"\n[상태 갱신] {ROBOT_STATE}")
                                last_printed_state = ROBOT_STATE.copy() # 현재 상태를 복사하여 저장
                        
                        except json.JSONDecodeError:
                            pass
            except Exception as e:
                print(f"📡 수신 에러: {e}")
                break

    # 소켓 송신
    def send_command(self, cmd_char):
        """"명령어 한 글자 전송 (예: 'p', 'r', 's' ...)"""
        if self.sock:
            try:
                # 바이너리로 인코딩하여 서버에 전송
                self.sock.sendall(cmd_char.encode())
                return True
            except Exception as e:
                print(f"⚠️ 전송 실패: {e}")
                return False
        else:
            print("⚠️ 연결이 되어있지 않습니다.")
            return False
    
    # 소켓 닫기
    def close(self):
        self.sock.close()
        print("연결 종료")