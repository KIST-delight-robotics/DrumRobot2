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
    GenericMotor(uint32_t nodeId);
    virtual ~GenericMotor() = default;

    // For CAN communication
    uint32_t nodeId;
    int socket;
    bool isConected = false;

    // Motors Feature
    float cwDir;
    float rMin, rMax;
    std::string myName;

    // Receive
    float motorPosition, motorVelocity;
    float jointAngle;
    float initialJointAngle;

    // parseSendCommand() -> 현재 사용 안함
    float desPos, desVel, desTor;

    // 명령의 최종 위치 저장
    float finalMotorPosition;

    std::queue<can_frame> sendBuffer;
    std::queue<can_frame> recieveBuffer;

    struct can_frame sendFrame;
    struct can_frame recieveFrame;

    void clearSendBuffer();
    void clearReceiveBuffer();

    ///////////////////////////////////////////////////////////

    // 에러 (쓰이는 곳은 없음)
    bool isError = false;

    //판별함수
    virtual bool isTMotor() const { return false; }  // 기본값: false
    virtual bool isMaxonMotor() const { return false; }  // 기본값: false
};

struct TMotorData
{
    float position;
    int mode;
    int is_brake;
};

class TMotor : public GenericMotor
{
public:
    TMotor(uint32_t nodeId, const std::string &motorType);
    
    std::string motorType;

    // Receive
    float motorCurrent;

    // Current Limit
    float currentLimit;
    int currentErrorCnt = 0;

    // 제어 모드
    const int Position = 0;
    const int Idle = 1;

    int controlMode = Position; 

    // commandBuffer
    std::queue<TMotorData> commandBuffer;
    void clearCommandBuffer();

    bool useFourBarLinkage;
    float initialMotorAngle;    // Four Bar Linkage 사용시 모터의 초기 위치
    float jointAngleToMotorPosition(float jointAngle);
    float motorPositionToJointAngle(float motorPosition);
    void setInitialMotorAngle(float jointAngle);

    bool isTMotor() const override { return true; }

private:
};

struct MaxonData
{
    float position;
    int mode;
    int kp;
    int kd;
};

class MaxonMotor : public GenericMotor
{
public:
    MaxonMotor(uint32_t nodeId);

    uint32_t canSendId;
    uint32_t canReceiveId;

    uint32_t txPdoIds[4];
    uint32_t rxPdoIds[4];

    // Receive
    float motorTorque;
    unsigned char statusBit;

    // 제어 모드
    const int HMM = 0;  // Homming Mode
    const int CSP = 1;  // Cyclic Sync Position Mode
    const int CSV = 2;  // Cyclic Sync Velocity Mode
    const int CST = 3;  // Cyclic Sync Torque Mode

    int controlMode = CSP;    // 현재 제어 모드

    // 1ms 궤적 만들기
    float pre_q;

    // 마찰 토크 보상 시 사용되는 값
    float preMotorPosition;

    // 타격 감지용 변수
    // queue<double> positionValues; // 포지션 값 저장을 위한 queue
    // int maxIndex = 5;
    bool hitting = false; // 드럼이 실제로 타격일 때 켜짐
    float hittingPos = 0; // 타격 시점의 각도
    // float hittingDrumAngle = 0; // 드럼 별 타격 각도
    bool isHit = false; // 타격 궤적 시작 (내려가는 궤적)
    bool drumReached = false; // 올라오는 궤적 시작

    // 입력 토크 계산시 사용되는 에러
    float pre_err = 0;

    // commandBuffer
    queue<MaxonData> commandBuffer;
    void clearCommandBuffer();

    float jointAngleToMotorPosition(float jointAngle);
    float motorPositionToJointAngle(float motorPosition);

    bool isMaxonMotor() const override { return true; }
};

#endif // MOTOR_H