#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include "protocol.h"

#define SERVER_IP "127.0.0.1"
#define PORT 8888
#define BUF_SIZE sizeof(ClientToServerMessage)

int main() {
    int sockfd;
    struct sockaddr_in serverAddr;
    ClientToServerMessage sendMessage;
    ServerToClientMessage receivedMessage;

    // 소켓 생성
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        perror("socket creation failed");
        return 1;
    }

    // 서버 주소 설정
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PORT);
    if (inet_pton(AF_INET, SERVER_IP, &serverAddr.sin_addr) <= 0) {
        perror("inet_pton failed");
        return 1;
    }

    while (true) {
        // 메시지 생성
        std::cout << "Enter state demand: ";
        std::cin >> sendMessage.stateDemand;
        std::cout << "Enter shutdown (0/1): ";
        std::cin >> sendMessage.shutdown;
        std::cout << "Enter all home (0/1): ";
        std::cin >> sendMessage.allHome;
        std::cout << "Enter axis home (0-8): ";
        std::cin >> sendMessage.axisHome;
        std::cout << "Enter music path: ";
        std::cin.ignore();
        std::cin.getline(sendMessage.musicPath, 256);

        // 서버에 데이터 전송
        sendto(sockfd, &sendMessage, sizeof(sendMessage), 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));

        // 서버로부터 응답 수신
        int recvBytes = recvfrom(sockfd, &receivedMessage, sizeof(receivedMessage), 0, nullptr, nullptr);
        if (recvBytes == -1) {
            perror("recvfrom failed");
            return 1;
        }

        std::cout << "Received response from server:" << std::endl;
        std::cout << "Main State: " << receivedMessage.mainState << std::endl;
        std::cout << "CAN 1 Status: " << receivedMessage.can1Status << std::endl;
        std::cout << "CAN 2 Status: " << receivedMessage.can2Status << std::endl;
        std::cout << "CAN 3 Status: " << receivedMessage.can3Status << std::endl;

        auto printLimbStatus = [](const ServerToClientMessage::LimbStatus& limb, const std::string& name) {
            std::cout << name << " Position Desired: " << limb.posDes << std::endl;
            std::cout << name << " Velocity: " << limb.vel << std::endl;
            std::cout << name << " Current: " << limb.cur << std::endl;
            std::cout << name << " Status: " << limb.status << std::endl;
            std::cout << name << " Home Done: " << limb.homeDone << std::endl;
        };

        printLimbStatus(receivedMessage.L1, "L1");
        printLimbStatus(receivedMessage.L2, "L2");
        printLimbStatus(receivedMessage.L3, "L3");
        printLimbStatus(receivedMessage.LW, "LW");
        printLimbStatus(receivedMessage.R1, "R1");
        printLimbStatus(receivedMessage.R2, "R2");
        printLimbStatus(receivedMessage.R3, "R3");
        printLimbStatus(receivedMessage.RW, "RW");
        printLimbStatus(receivedMessage.Waist, "Waist");

        
    }

    close(sockfd);
    return 0;
}
