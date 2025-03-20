#pragma once

#include <stdio.h>
#include <map>
#include <memory>
#include <string>
#include <functional>
#include <queue>
#include <algorithm>
#include <thread>
#include <cerrno>  // errno
#include <cstring> // strerror
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <iostream>
#include <vector>
#include <limits>
#include <ctime>
#include <fstream>
#include <atomic>
#include <cmath>
#include <chrono>
#include <set>

#include "SystemState.hpp"
#include "../include/managers/CanManager.hpp"
#include "../include/managers/PathManager.hpp"
#include "../include/motors/CommandParser.hpp"
#include "../include/motors/Motor.hpp"
#include "../include/managers/TestManager.hpp"
#include "../include/USBIO_advantech/USBIO_advantech.hpp"
#include "../include/tasks/Functions.hpp"
// #include "../include/managers/SignalHandler.hpp"

using namespace std;


class DrumRobot
{
public:
    DrumRobot(State &StateRef,
              CanManager &canManagerRef,
              PathManager &pathManagerRef,
              TestManager &testManagerRef,
              std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef,
              USBIO &usbioRef,
              Functions &funRef);

    void stateMachine();
    void sendLoopForThread();
    void recvLoopForThread();

private:
    State &state;
    CanManager &canManager;
    PathManager &pathManager;
    TestManager &testManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;
    USBIO &usbio;
    Functions &fun;

    // Parsing
    TMotorServoCommandParser tservocmd;
    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;

    // 제어 주기
    chrono::system_clock::time_point ReadStandard;
    chrono::system_clock::time_point SendStandard;
    chrono::system_clock::time_point SendMaxon;
    chrono::system_clock::time_point addStandard;

    // Sync command
    std::shared_ptr<MaxonMotor> virtualMaxonMotor;

    // 쓰레드 루프 주기
    std::chrono::_V2::steady_clock::time_point sendLoopPeriod;
    std::chrono::_V2::steady_clock::time_point recvLoopPeriod;
    std::chrono::_V2::steady_clock::time_point stateMachinePeriod;

    // State Utility 메소드들
    void displayAvailableCommands() const;
    bool processInput(const std::string &input);
    void idealStateRoutine();
    void checkUserInput();
    int kbhit();

    // 로봇 상태 플래그
    bool settingInitPos = false;
    bool isReady = false;   // 스네어 위치
    bool isHome = false;    // 고정 위치
    bool isRestart = false; // 재시작 위치
    void setRobotFlag(string flag);

    // AddStance
    bool goToHome = false;
    bool goToSemiReady = false;
    bool goToReady = false;
    bool goToShutdown = false;
    void setAddStanceFlag(string flag);
    int hommingCnt = 0; ///< Home 이동이 여러 단계일 경우 각 단계
    
    // System Initialize 메소드들
    void initializeMotors();
    void initializecanManager();
    void deactivateControlTask();
    void clearBufferforRecord();
    void printCurrentPositions();
    void motorSettingCmd();
    void setMaxonMode(std::string targetMode);

    // Send Thread Loop 메소드들
    int writeFailCount;
    int maxonMotorCount = 0;
    void initializePathManager();
    void clearMotorsSendBuffer();
    void sendPlayProcess(int periodMicroSec, string musicName);
    void sendAddStanceProcess(int periodMicroSec);
    void unfixedMotor();
    void clearMotorsCommandBuffer();

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
        {"L_foot", 11}};

    // 로봇 고정했을 때 각 모터의 관절각      Waist   Rarm1   Larm1   Rarm2   Rarm3   Larm2   Larm3   Rwrist   Lwrist   maxonForTest   Rfoot   Lfoot   [deg]
    const float initialJointAngles[12] = {10.0,   90.0,   90.0,   0.0,    135.0,  0.0,    135.0,   -90.0,     0,    0.0,           0.0,    0.0};

    // 로봇의 관절각 범위
    //                                 Waist   Rarm1   Larm1   Rarm2   Rarm3   Larm2   Larm3   Rwrist   Lwrist   maxonForTest   Rfoot   Lfoot    [deg]
    const float jointRangeMax[12] = {90.0,  150.0,  180.0,  90.0,   140.0,  90.0,   140.0,  135.0,  135.0,     135.0,         135.0,  135.0};
    // const float jointRangeMin[11] = {-90.0, 0.0,    30.0,   -60.0,    0.0,  -60.0,    0.0,    0.0,    0.0,  -90.0,  -90.0};
    const float jointRangeMin[12] = {-90.0, 0.0,    30.0,   -60.0,  -30.0,  -60.0,  -30.0,  -108.0, -108.0,    -90.0 ,       -90.0,  -90.0};

    // Receive Thread Loop 메소드들
    void readProcess(int periodMicroSec);

    // Maxon 모터 초기화 함수
    void maxonMotorEnable();
    void setMaxonMotorMode(std::string targetMode);
    
    //play 관련 전역변수들
    std::string basePath = "/home/shy/DrumRobot/include/codes/";    // 악보 위치
    std::string musicName;
    int fileIndex;
    bool bpmFlag;
    double timeSum = 0.0;
    bool openFlag;
    std::ifstream inputFile; 
    int preCreatedLine = 3; // 미리 궤적을 생성할 줄
};
