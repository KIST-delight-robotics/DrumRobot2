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

#include "../include/eigen-3.4.0/Eigen/Dense"
#include "Motor.hpp"
#include "CommandParser.hpp"
#include "../include/tasks/Functions.hpp"
#include "../include/USBIO_advantech/USBIO_advantech.hpp"


#define SEND_SIGN 100
#define INIT_SIGN 99

// position loop mode 에서 step input 각도 제한
#define POS_DIFF_LIMIT 35.0*M_PI/180.0

using namespace std;
using namespace Eigen;

class CanManager
{
public:
    static const int ERR_SOCKET_CREATE_FAILURE = -1;
    static const int ERR_SOCKET_CONFIGURE_FAILURE = -2;

    CanManager(std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef, Functions &funcRef, USBIO &usbioRef);

    ~CanManager();

    std::map<std::string, int> sockets;      ///< 모터와 통신하는 소켓의 맵.
    std::map<std::string, bool> isConnected; ///< 모터의 연결 상태를 나타내는 맵.
    std::vector<std::string> ifnames;

    // tMotor 제어 주기 결정
    const double DTSECOND = 0.005;

    map<std::string, int> motorMapping = { ///< 각 관절에 해당하는 정보 [이름, CAN ID]
        {"waist", 0},
        {"R_arm1", 1},
        {"L_arm1", 2},
        {"R_arm2", 3},
        {"R_arm3", 4},
        {"L_arm2", 5},
        {"L_arm3", 6},
        {"R_wrist", 7},
        {"L_wrist", 8},
        {"maxonForTest", 9},
        {"R_foot", 10},
        {"L_foot", 11}
    };

    
    //////////////////////////////////////// Initialize
    void initializeCAN();

    //////////////////////////////////////// Setting
    void checkCanPortsStatus();
    void setSocketNonBlock();
    void setSocketBlock();
    bool txFrame(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame);
    bool rxFrame(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame);
    bool sendAndRecv(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame);
    bool recvToBuff(std::shared_ptr<GenericMotor> &motor, int readCount);

    void clearReadBuffers();

    void setSocketsTimeout(int sec, int usec);
    bool setMotorsSocket();

    //////////////////////////////////////// Send
    int errorCnt = 0;   // 수신 에러 카운트

    bool sendMotorFrame(const std::shared_ptr<GenericMotor> &motor);
    bool setCANFrame(std::map<std::string, bool>& fixFlags, int cycleCounter);

    //////////////////////////////////////// Receive
    void readFramesFromAllSockets();
    bool distributeFramesToMotors(bool setlimit);

    //////////////////////////////////////// 토크 제어 관련
    int maxonCnt = 0;
    bool isHitR = false;
    bool isHitL = false;
    bool isPlay = false;
    bool isCST = false;
    bool isCSTR = false;
    bool isCSTL = false;
    float wristStayAngle;
    float backTorque = -10;
    bool torqueOff = false;

    // SDO communication으로 받아오는 현재 위치 값
    float currentPosition = 0.0;

private:

    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;
    Functions &func;
    USBIO &usbio;
    
    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;
    TMotorServoCommandParser tservocmd;

    std::map<int, std::vector<can_frame>> tempFrames;

    //////////////////////////////////////// Initialize
    bool getCanPortStatus(const char *port);
    void activateCanPort(const char *port);
    void listAndActivateAvailableCANPorts();
    int createSocket(const std::string &ifname);

    //////////////////////////////////////// Setting
    void flushCanBuffer(int socket);
    void resetCanFilter(int socket);
    
    void clearCanBuffer(int canSocket);
    int setSocketTimeout(int socket, int sec, int usec);

    //////////////////////////////////////// Send
    void setMaxonCANFrame(std::shared_ptr<MaxonMotor> &maxonMotor, const MaxonData &mData);
    float calTorque(std::shared_ptr<MaxonMotor> &maxonMotor, const MaxonData &mData);
    void setTMotorCANFrame(std::shared_ptr<TMotor> &tMotor, const TMotorData &tData);
    bool safetyCheckSendT(std::shared_ptr<TMotor> &tMotor, TMotorData &tData);

    void deactivateCanPort(const char *port);
    void deactivateAllCanPorts();

    //////////////////////////////////////// Receive
    bool safetyCheckRecvT(std::shared_ptr<GenericMotor> &motor);
    bool safetyCheckRecvM(std::shared_ptr<GenericMotor> &motor);
    
};

#endif // CAN_SOCKET_UTILS_H
