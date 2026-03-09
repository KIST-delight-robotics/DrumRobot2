//../include/managers/AgentSocket.hpp
#ifndef AGENTSOCKET_HPP
#define AGENTSOCKET_HPP

#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <queue>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <atomic>

// 로봇의 상태를 확인하기 위해 필요하다면 include
// #include "../tasks/SystemState.hpp"

class AgentSocket
{
public:
    // 생성자: 포트 번호를 받음 (기본 9999)
    AgentSocket(int port = 9999);
    ~AgentSocket();

    // 서버 시작 (스레드 생성)
    void start();

    // 서버 종료 (자원 해제)
    void stop();

    // 메인 루프에서 명령 꺼내기 (Thread-Safe)
    // 명령이 없으면 빈 문자열 "" 반환
    std::string popCommand();

    // 연결 상태 확인
    bool isConnected();

    // 상태 정보 전송 (예: JSON 문자열)
    void sendState(const std::string &jsonStr);

    // 셔터(게이트) 제어 함수
    void openGate() { isGateOpen = true; }
    void closeGate()
    {
        isGateOpen = false;
        clearQueue(); // 셔터 닫을 때 혹시 남아있는 쓰레기도 싹 비움
    }
    void clearQueue();

private:
    int server_fd;
    int new_socket;
    int port;
    struct sockaddr_in address;
    int addrlen;

    std::atomic<bool> keepRunning; // 스레드 제어 플래그
    std::thread serverThread;      // 백그라운드 통신 스레드

    // 명령어를 저장할 큐와 자물쇠(Mutex)
    std::queue<std::string> commandQueue;
    std::mutex queueMutex;

    // 스레드 내부에서 돌아갈 실제 루프 함수
    void runServerLoop();

    // 셔터(게이트) 상태 플래그
    std::atomic<bool> isGateOpen{false};
};

#endif // AGENTSOCKET_HPP