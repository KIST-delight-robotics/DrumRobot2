import socket

#localhost, testport
HOST = '127.0.0.1'
PORT = 9999

#1. 소켓 객체 생성
# AF_INET: IPv4, SOCK_STREAM: TCP
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
    
    #2. 소켓 연결
    client_socket.connect((HOST, PORT))
    print(f"서버({HOST}:{PORT}에 연결되었습니다.)")
    
    #3. 말하기 루프    
    while True:
        cmd = input("명령(r/p/h/t/s/quit): ")
        if cmd == 'quit':
            break
        
        #4. 소켓 전송    
        client_socket.sendall(cmd.encode())