#ifndef MOTOR_H
#define MOTOR_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdlib.h>
#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include <queue>
#include <linux/can/raw.h>
#include <iostream>
#include <cmath>
#include <map>
using namespace std;

class GenericMotor
{
public:
    // For CAN communication
    uint32_t nodeId;
    int socket;
    bool isConected;

    // Motors Feature
    float cwDir;
    float rMin, rMax;
    std::string myName;

    // Receive
    float motorPosition, motorVelocity;
    float jointAngle;
    float initialJointAngle;

    // Fixed
    bool isError = false;
    float fixedMotorPosition;
    bool isfixed = false;

    // parseSendCommand()
    float desPos, desVel, desTor;

    // int32_t spd = 0; // ERPM
    // int32_t acl = 0; // ERPA

    std::queue<can_frame> sendBuffer;
    std::queue<can_frame> recieveBuffer;

    struct can_frame sendFrame;
    struct can_frame recieveFrame;

    GenericMotor(uint32_t nodeId);
    virtual ~GenericMotor() = default;

    void clearSendBuffer();
    void clearReceiveBuffer();
};

struct TMotorData
{
    float position;
    int32_t spd;
    int32_t acl;
    bool isBrake;
};

class TMotor : public GenericMotor
{
public:
    TMotor(uint32_t nodeId, const std::string &motorType);
    
    std::string motorType;
    // Gear ratio
    std::map<std::string, int> R_Ratio = {
        {"AK80_64", 64},
        {"AK70_10", 10},
        {"AK10_9", 9}
    };
    const int PolePairs = 21;

    // current [A]
    float currentLimit;
    float motorCurrent;
    int currentErrorCnt = 0;

    // brake
    bool brakeState;

    std::queue<TMotorData> commandBuffer;

    void clearCommandBuffer();

    bool useFourBarLinkage;
    float initialMotorAngle;    // Four Bar Linkage 사용시 모터의 초기 위치
    float jointAngleToMotorPosition(float jointAngle);
    float motorPositionToJointAngle(float motorPosition);
    void setInitialMotorAngle(float jointAngle);

private:
};

struct MaxonData
{
    float position;
    double WristState;
};

class MaxonMotor : public GenericMotor
{
public:
    MaxonMotor(uint32_t nodeId);

    uint32_t canSendId;
    uint32_t canReceiveId;

    uint32_t txPdoIds[4];
    uint32_t rxPdoIds[4];

    float motorTorque;

    float positionValues[4] = {0}; // 포지션 값 저장을 위한 정적 배열
    int posIndex = 0;

    bool stay = false;
    bool hitting = false;
    bool isPositionMode = false;
    bool atPosition = false;
    bool positioning = false;

    bool checked = false;

    unsigned char statusBit;
    float bumperLocation = 0.0;

    queue<MaxonData> commandBuffer;
    queue<float> wrist_BackArr;
    void clearCommandBuffer();
    void clearWrist_BackArr();

    float jointAngleToMotorPosition(float jointAngle);
    float motorPositionToJointAngle(float motorPosition);
};

#endif // MOTOR_H