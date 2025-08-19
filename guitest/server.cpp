#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include "protocol.h"

#define PORT 8888
#define BUF_SIZE sizeof(ClientToServerMessage)

int main() {
    int sockfd;
    struct sockaddr_in serverAddr, clientAddr;
    ClientToServerMessage receivedMessage;
    ServerToClientMessage responseMessage;

    // 소켓 생성
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        perror("socket creation failed");
        return 1;
    }

    // 서버 주소 설정
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddr.sin_port = htons(PORT);

    // 소켓에 주소 바인딩
    if (bind(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == -1) {
        perror("bind failed");
        return 1;
    }

    std::cout << "Server listening on port " << PORT << std::endl;

    while (true) {
        socklen_t clientLen = sizeof(clientAddr);

        // 클라이언트로부터 데이터 수신
        int recvBytes = recvfrom(sockfd, &receivedMessage, BUF_SIZE, 0, (struct sockaddr *)&clientAddr, &clientLen);
        if (recvBytes == -1) {
            perror("recvfrom failed");
            return 1;
        }

        std::cout << "Received message from client:" << std::endl;
        std::cout << "State Demand: " << receivedMessage.stateDemand << std::endl;
        std::cout << "Shutdown: " << receivedMessage.shutdown << std::endl;
        std::cout << "All Home: " << receivedMessage.allHome << std::endl;
        std::cout << "Axis Home: " << receivedMessage.axisHome << std::endl;
        std::cout << "Music Path: " << receivedMessage.musicPath << std::endl;

        // 사용자로부터 CAN 상태 입력 받기
        int can1, can2, can3;
        std::cout << "Enter CAN 1 status (0/1): ";
        std::cin >> can1;
        std::cout << "Enter CAN 2 status (0/1): ";
        std::cin >> can2;
        std::cout << "Enter CAN 3 status (0/1): ";
        std::cin >> can3;

        // 클라이언트에게 응답 설정
        responseMessage.mainState = 1; // 예시로 설정
        responseMessage.can1Status = (can1 == 1); // 사용자 입력 값으로 설정
        responseMessage.can2Status = (can2 == 1); // 사용자 입력 값으로 설정
        responseMessage.can3Status = (can3 == 1); // 사용자 입력 값으로 설정

        auto setLimbStatus = [](ServerToClientMessage::LimbStatus& limb, float posDes, float vel, float cur, bool status, bool homeDone) {
            limb.posDes = posDes;
            limb.vel = vel;
            limb.cur = cur;
            limb.status = status;
            limb.homeDone = homeDone;
        };

        setLimbStatus(responseMessage.L1, 0.0f, 0.0f, 0.0f, true, true);
        setLimbStatus(responseMessage.L2, 0.0f, 0.0f, 0.0f, true, true);
        setLimbStatus(responseMessage.L3, 0.0f, 0.0f, 0.0f, true, true);
        setLimbStatus(responseMessage.LW, 0.0f, 0.0f, 0.0f, true, true);
        setLimbStatus(responseMessage.R1, 0.0f, 0.0f, 0.0f, true, true);
        setLimbStatus(responseMessage.R2, 0.0f, 0.0f, 0.0f, true, true);
        setLimbStatus(responseMessage.R3, 0.0f, 0.0f, 0.0f, true, true);
        setLimbStatus(responseMessage.RW, 0.0f, 0.0f, 0.0f, true, true);
        setLimbStatus(responseMessage.Waist, 0.0f, 0.0f, 0.0f, true, true);

        sendto(sockfd, &responseMessage, sizeof(responseMessage), 0, (struct sockaddr *)&clientAddr, clientLen);
    }

    close(sockfd);
    return 0;
}
