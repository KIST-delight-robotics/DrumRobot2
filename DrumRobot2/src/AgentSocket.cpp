// AgentSocket.cpp
#include "../include/managers/AgentSocket.hpp"

AgentSocket::AgentSocket(int port) : port(port), server_fd(-1), new_socket(-1), keepRunning(false) {
    addrlen = sizeof(address);
}

AgentSocket::~AgentSocket() {
    stop();
}

void AgentSocket::start() {
    if (keepRunning) return; // 이미 실행 중이면 패스

    // 1. 소켓 생성
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("[AgentSocket] Socket failed");
        return;
    }

    // 2. 포트 재사용 설정 (껐다 켰을 때 Bind Error 방지)
    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("[AgentSocket] setsockopt failed");
        return;
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY; // 모든 IP 접속 허용
    address.sin_port = htons(port);

    // 3. 바인딩
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("[AgentSocket] Bind failed");
        return;
    }

    // 4. 리슨 (대기열 3개)
    if (listen(server_fd, 3) < 0) {
        perror("[AgentSocket] Listen failed");
        return;
    }

    // 5. 스레드 시작
    keepRunning = true;
    serverThread = std::thread(&AgentSocket::runServerLoop, this);
    std::cout << ">>> [Agent] Listening on Port " << port << " (Waiting for Brain)..." << std::endl;
}

void AgentSocket::stop() {
    keepRunning = false;
    
    // 소켓 닫기 (accept()에서 대기 중인 스레드를 깨우기 위함)
    if (new_socket > 0) close(new_socket);
    if (server_fd > 0) close(server_fd);

    if (serverThread.joinable()) {
        serverThread.join();
    }
    std::cout << "[Agent] Socket Server Stopped." << std::endl;
}

void AgentSocket::runServerLoop() {
    while (keepRunning) {
        // 1. 클라이언트(Python) 접속 대기 (Blocking)
        // 소켓이 닫히면 accept는 -1을 리턴하므로 루프 종료됨
        new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen);
        
        if (new_socket < 0) {
            if (keepRunning) perror("[Agent] Accept failed (Retrying...)");
            continue; 
        }

        std::cout << ">>> [Agent] Brain Connected! (Python Client)" << std::endl;

        // 2. 데이터 수신 루프
        char buffer[1024] = {0};
        while (keepRunning) {
            memset(buffer, 0, sizeof(buffer));
            int valread = read(new_socket, buffer, sizeof(buffer) - 1); // -1 to leave space for null terminator

            if (valread <= 0) { // 연결 끊김
                std::cout << ">>> [Agent] Brain Disconnected." << std::endl;
                close(new_socket);
                new_socket = -1;
                break; // 다시 accept 대기로 돌아감
            }

            std::string cmd(buffer);
            
            // ★ 중요: 큐에 넣을 때는 자물쇠(Mutex) 잠그기
            {
                std::lock_guard<std::mutex> lock(queueMutex);
                commandQueue.push(cmd);
            }
            // std::cout << "[Agent] Command Received: " << cmd << std::endl;
        }
    }
}

std::string AgentSocket::popCommand() {
    std::lock_guard<std::mutex> lock(queueMutex); // 자물쇠 잠그기
    
    if (commandQueue.empty()) {
        return ""; // 명령 없음
    }

    std::string cmd = commandQueue.front();
    commandQueue.pop();
    return cmd;
}

bool AgentSocket::isConnected() {
    return new_socket != -1;
}

void AgentSocket::sendState(const std::string& jsonStr) {
    if (new_socket > 0 && keepRunning) {
        // 파이썬 쪽에서 한 묶음씩 읽기 쉽도록 끝에 줄바꿈(\n)을 붙여줍니다.
        std::string packet = jsonStr + "\n"; 
        
        // MSG_NOSIGNAL: 파이썬 쪽 연결이 갑자기 끊겨도 C++이 뻗지 않도록 하는 핵심 방어 옵션입니다.
        send(new_socket, packet.c_str(), packet.length(), MSG_NOSIGNAL);
    }
}