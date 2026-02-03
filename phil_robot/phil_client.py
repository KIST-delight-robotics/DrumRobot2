import socket
import time

class RobotClient:

    # 클라이언트 소켓 생성자
    def __init__(self, host, port):
        self.host=host
        self.port=port
        self.sock=None

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
                return True

            except (socket.error, ConnectionRefusedError):
                self.sock.close()
                print("⏳ 로봇 서버 대기 중... (main.out 실행 후 'o'를 눌러주세요)")
                time.sleep(3)

    
    # 소켓 전송
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