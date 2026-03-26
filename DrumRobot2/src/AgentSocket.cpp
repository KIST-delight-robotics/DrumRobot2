// AgentSocket.cpp
#include "../include/managers/AgentSocket.hpp"
#include <cctype>

AgentSocket::AgentSocket(int port) : server_fd(-1), new_socket(-1), port(port), keepRunning(false) {
    addrlen = sizeof(address);
}

AgentSocket::~AgentSocket() {
    stop();
}

// 문자열 앞뒤 공백 제거 함수
std::string AgentSocket::trimText(const std::string& text) const {
    size_t left_idx = 0;
    while (left_idx < text.size() && std::isspace(static_cast<unsigned char>(text[left_idx]))) {
        left_idx++;
    }

    size_t right_idx = text.size();
    while (right_idx > left_idx && std::isspace(static_cast<unsigned char>(text[right_idx - 1]))) {
        right_idx--;
    }

    return text.substr(left_idx, right_idx - left_idx);
}

bool AgentSocket::isJsonStart(const std::string& text) const {
    std::string clean_text = trimText(text);
    return !clean_text.empty() && clean_text.front() == '{';
}

// JSON 문자열이 완전한지 확인하는 함수 (중괄호 짝 검사 + 문자열 내부 무시)
bool AgentSocket::hasFullJson(const std::string& text) const {
    int brace_level = 0;
    bool in_quote = false;
    bool escape_next = false;
    bool saw_brace = false;

    for (char ch : text) {
        if (escape_next) {
            escape_next = false;
            continue;
        }

        if (ch == '\\') {
            escape_next = true;
            continue;
        }

        if (ch == '"') {
            in_quote = !in_quote;
            continue;
        }

        if (in_quote) {
            continue;
        }

        if (ch == '{') {
            brace_level++;
            saw_brace = true;
        } else if (ch == '}') {
            brace_level--;
            if (brace_level < 0) {
                return false;
            }
        }
    }

    return saw_brace && brace_level == 0;
}

// 큐 비우기 함수
void AgentSocket::clearQueue() {
    std::lock_guard<std::mutex> lock(queueMutex);
    std::queue<std::string> empty;
    std::swap(commandQueue, empty);
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
        char read_buf[1024] = {0};
        std::string line_buf = ""; // ★ TCP 스트림 조립용 누적 버퍼 추가
        std::string json_buf = "";
        bool read_json = false;

        while (keepRunning) {
            memset(read_buf, 0, sizeof(read_buf));
            int read_len = read(new_socket, read_buf, sizeof(read_buf) - 1); // -1 to leave space for null terminator

            if (read_len <= 0) { // 연결 끊김
                std::cout << ">>> [Agent] Brain Disconnected." << std::endl;
                close(new_socket);
                new_socket = -1;
                break; // 다시 accept 대기로 돌아감
            }

            // ★ 받은 데이터를 무조건 누적 버퍼에 뒤이어 붙입니다.
            line_buf.append(read_buf, read_len);

            // ★ '\n' 구분자를 찾아서 완전한 명령어 단위로 쪼개기
            size_t line_end = 0;
            while ((line_end = line_buf.find('\n')) != std::string::npos) {
                // '\n' 앞까지 잘라서 하나의 명령어로 추출
                std::string cmd_text = line_buf.substr(0, line_end);

                // 처리한 명령어와 '\n'은 누적 버퍼에서 삭제
                line_buf.erase(0, line_end + 1); 

                // 혹시 모를 캐리지 리턴(\r)이나 공백 찌꺼기 제거
                if (!cmd_text.empty() && cmd_text.back() == '\r') {
                    cmd_text.pop_back();
                }

                cmd_text = trimText(cmd_text);
                if (cmd_text.empty()) continue; // 빈 줄은 무시

                if (read_json || isJsonStart(cmd_text)) {
                    if (!read_json) {
                        json_buf.clear();
                        read_json = true;
                    }

                    if (!json_buf.empty()) {
                        json_buf += "\n";
                    }
                    json_buf += cmd_text;

                    if (!hasFullJson(json_buf)) {
                        continue;
                    }

                    cmd_text = json_buf;
                    json_buf.clear();
                    read_json = false;
                }

                // ★ 안전장치: 셔터(게이트)가 닫혀있으면 명령 폐기
                if (!isGateOpen.load()) {
                    std::cout << "🚫 [Safeguard] 로봇 보호 상태: 명령 폐기 -> " << cmd_text << std::endl;
                    continue; 
                }

                // ★ 중요: 큐에 넣을 때는 자물쇠(Mutex) 잠그기
                std::lock_guard<std::mutex> lock(queueMutex);
                commandQueue.push(cmd_text);
                
            }
        }
    }
}

std::string AgentSocket::popCommand() {
    std::lock_guard<std::mutex> lock(queueMutex); // 자물쇠 잠그기
    
    if (commandQueue.empty()) {
        return ""; // 명령 없음
    }

    std::string cmd_text = commandQueue.front();
    commandQueue.pop();
    return cmd_text;
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
