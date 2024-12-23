#ifndef CAN_SOCKET_UTILS_H
#define CAN_SOCKET_UTILS_H

#include <linux/can.h>
#include <math.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <bits/types.h>
#include <linux/can/raw.h>
#include <sys/time.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <map>
#include <queue>
#include <memory>
#include <chrono>
#include <fstream>

#include "Motor.hpp"
#include "CommandParser.hpp"
#include "../include/tasks/Functions.hpp"

// position loop mode 에서 step input 각도 제한
#define POS_DIFF_LIMIT 30.0*M_PI/180.0

#define SEND_SIGN 100
#define INIT_SIGN 99.9

using namespace std;

class CanManager
{
public:
    static const int ERR_SOCKET_CREATE_FAILURE = -1;
    static const int ERR_SOCKET_CONFIGURE_FAILURE = -2;

    CanManager(std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef, Functions &funRef);

    ~CanManager();

    std::map<std::string, int> sockets;      ///< 모터와 통신하는 소켓의 맵.
    std::map<std::string, bool> isConnected; ///< 모터의 연결 상태를 나타내는 맵.
    std::vector<std::string> ifnames;
    int errorCnt = 0;
    int maxonCnt = 0;

    // tMotor 제어 주기 결정
    const float deltaT = 0.005;

    void initializeCAN();

    void setSocketsTimeout(int sec, int usec);
    void flushCanBuffer(int socket);
    void resetCanFilter(int socket);
    void checkCanPortsStatus();

    void setMotorsSocket();

    bool sendAndRecv(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame);

    bool sendFromBuff(std::shared_ptr<GenericMotor> &motor);

    bool sendMotorFrame(std::shared_ptr<GenericMotor> motor);

    bool checkMaxon();

    bool checkAllMotors_Fixed();

    bool sendForCheck_Fixed(std::shared_ptr<GenericMotor> motor);

    bool recvToBuff(std::shared_ptr<GenericMotor> &motor, int readCount);

    bool txFrame(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame);

    bool rxFrame(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame);

    void readFramesFromAllSockets();

    bool distributeFramesToMotors(bool setlimit);

    void clearReadBuffers();

    void setSocketNonBlock();
    void setSocketBlock();

    bool setCANFrame();

    bool safetyCheck_Tmotor(std::shared_ptr<TMotor> tMotor, TMotorData tData);
    bool safetyCheck_T(std::shared_ptr<GenericMotor> &motor);
    bool safetyCheck_M(std::shared_ptr<GenericMotor> &motor);

    map<std::string, int> motor_mapping = { ///< 각 관절에 해당하는 열 정보.
        {"waist", 0},
        {"R_arm1", 1},
        {"L_arm1", 2},
        {"R_arm2", 3},
        {"R_arm3", 4},
        {"L_arm2", 5},
        {"L_arm3", 6},
        {"R_wrist", 7},
        {"L_wrist", 8},
        {"maxonForTest", 8},
        {"R_foot", 9},
        {"L_foot", 10}};

private:

    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;
    Functions &fun;
    
    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;
    TMotorServoCommandParser tservocmd;

    std::map<int, std::vector<can_frame>> tempFrames;

    bool getCanPortStatus(const char *port);
    void activateCanPort(const char *port);
    void deactivateCanPort(const char *port);
    void deactivateAllCanPorts();
    void list_and_activate_available_can_ports();

    int createSocket(const std::string &ifname);
    int setSocketTimeout(int socket, int sec, int usec);
    void clearCanBuffer(int canSocket);
};

#endif // CAN_SOCKET_UTILS_H
