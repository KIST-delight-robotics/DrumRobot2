#ifndef GUI_MANAGER_H
#define GUI_MANAGER_H

#include <iostream>
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include "SystemState.hpp"
#include "CanManager.hpp"
#include "Motor.hpp"

#define PORT 8888
#define BUF_SIZE 1024

struct ClientToServerMessage {
    int stateDemand;
    int shutdown;
    int allHome;
    int axisHome;
    char musicPath[256]; 
};

struct ServerToClientMessage {
    int mainState;
    bool can1Status;
    bool can2Status;
    bool can3Status;

    struct LimbStatus {
        float posDes;
        float vel;
        float cur;
        bool status;
        bool homeDone;
    };

    LimbStatus L1;
    LimbStatus L2;
    LimbStatus L3;
    LimbStatus LW;
    LimbStatus R1;
    LimbStatus R2;
    LimbStatus R3;
    LimbStatus RW;
    LimbStatus Waist;
};


class GuiManager
{
public:
    GuiManager(State &stateRef, CanManager &canManagerRef, std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef);
    ~GuiManager();


    void guiThread();
    void initializeServer();
    bool guiConnected = false;

    // send ServerToClientMessage
    bool send();

    // recive ClientToServerMessage
    bool receive();

    // Message Data
    ClientToServerMessage receiveMessage;
    ServerToClientMessage sendMessage;


private:
    int sockfd;
    struct sockaddr_in serverAddr, clientAddr;
    char buffer[BUF_SIZE];
    socklen_t clientLen;
    State &state;
    CanManager &canManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;
};

#endif // QT_MANAGER_H
