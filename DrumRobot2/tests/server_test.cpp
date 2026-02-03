#include <iostream>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#define PORT 9999
#define BUFFER_SIZE 1024

int main() {

    // 1. 소켓 생성
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("Socket failed");
        return -1;
    }

    // 2. 주소 재사용 옵션 설정
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

    // 3. 주소 구조체 설정
    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;  // 모든 IP에서 접속 허용
    address.sin_port = htons(PORT);         // 포트 9999

    // 4. 소켓에 주소 바인딩
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("Bind failed");
        close(server_fd);
        return -1;
    }

    // 5. 연결 대기 상태로 전환
    if (listen(server_fd, 3) < 0) {
        perror("Listen");
        close(server_fd);
        return -1;
    }

    std::cout << "Server listening on port 9999..." << std::endl;

    // 6. 클라이언트 연결 수락
    socklen_t addrlen = sizeof(address);
    int client_fd = accept(server_fd, (struct sockaddr*)&address, &addrlen);
    if (client_fd < 0) {
        perror("Accept failed");
        close(server_fd);
        return -1;
    }

    std::cout << "Client connected!" << std::endl;

    // 7. 데이터 송수신
    char buffer[BUFFER_SIZE] = {0};
    while (true) {
        // 데이터 수신
        memset(buffer, 0, BUFFER_SIZE);
        int bytes_read = read(client_fd, buffer, BUFFER_SIZE);
        if (bytes_read <= 0) {
            std::cout << "Client disconnected" << std::endl;
            break;
        }

        std::cout << "받은 명령: " << buffer << std::endl;

        //실제 로봇 코드에 들어갈 로직 예시
        if (strcmp(buffer, "r") == 0) {
            std::cout << "[ACTION] READY mode" << std::endl; 
        } else if (strcmp(buffer, "p") == 0) {
            std::cout << "[ACTION] PLAY mode" << std::endl; 
        } else if (strcmp(buffer, "h") == 0) {
            std::cout << "[ACTION] HOME mode" << std::endl; 
        } else if (strcmp(buffer, "t") == 0) {
            std::cout << "[ACTION] TEST mode" << std::endl; 
        } else if (strcmp(buffer, "s") == 0) {
            std::cout << "[ACTION] STOP mode" << std::endl; 
        } else if (strcmp(buffer, "quit") == 0) {
            std::cout << "QUIT" << std::endl; 
        }
    }

    // 8. 소켓 종료
    close(client_fd);
    close(server_fd);

    return 0;
}
