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
#include <SFML/Audio.hpp>

#include "SystemState.hpp"
#include "../include/managers/CanManager.hpp"
#include "../include/managers/PathManager.hpp"
#include "../include/motors/CommandParser.hpp"
#include "../include/motors/Motor.hpp"
#include "../include/managers/TestManager.hpp"
#include "../include/USBIO_advantech/USBIO_advantech.hpp"
#include "../include/tasks/Functions.hpp"

using namespace std;

class FlagClass
{
public:
    FlagClass();
    ~FlagClass();

    // AddStance
    const int ISSTART = 0;
    const int ISHOME = 1;
    const int ISREADY = 2;
    const int ISSHUTDOWN = 3;

    void setAddStanceFlag(string flagName);
    string getAddStanceFlag();

    // fixed
    void setFixationFlag(string flagName);
    bool getFixationFlag();

private:

    int addStanceFlag = ISSTART;
    bool isFixed = false;
};

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

    // thread
    void stateMachine();
    void sendLoopForThread();
    void recvLoopForThread();
    void musicMachine();
    void runPythonInThread();

    // init
    void initializeDrumRobot();

    bool file_found = false;    // mid 파일 들어왔는지 확인
    
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

    // Sync command
    vector<std::shared_ptr<MaxonMotor>> virtualMaxonMotor;
    // std::shared_ptr<MaxonMotor> virtualMaxonMotor;

    // 쓰레드 루프 주기
    std::chrono::_V2::steady_clock::time_point sendLoopPeriod;
    std::chrono::_V2::steady_clock::time_point recvLoopPeriod;

    // 로봇 고정했을 때 각 모터의 관절각      Waist   Rarm1   Larm1   Rarm2   Rarm3   Larm2   Larm3   Rwrist   Lwrist   maxonForTest   Rfoot   Lfoot   [deg]
    const float initialJointAngles[12] = {10.0,   90.0,   90.0,   0.0,     90.0,    0.0,    90.0,   90.0,    90.0,        0.0,        0.0,    0.0};

    // 로봇의 관절각 범위
    //                                 Waist    Rarm1   Larm1   Rarm2   Rarm3   Larm2   Larm3   Rwrist  Lwrist   maxonForTest  Rfoot   Lfoot    [deg]
    const float jointRangeMax[12] = {   90.0,   150.0,  180.0,  90.0,   140.0,  90.0,   140.0,  135.0,  135.0,      135.0,     200.0,  200.0};
    const float jointRangeMin[12] = {   -90.0,  0.0,    30.0,   -60.0,    0.0,  -60.0,    0.0,  -108.0, -108.0,     -90.0,     -90.0,  -90.0};
    // TestManager.hpp 에서도 수정해줘야 함

    FlagClass flagObj;
    bool allMotorsUnConected = true;    // 모든 모터 연결 안됨 - 모터 없이 테스트하는 경우

    //////////////////////////////////////////////////////////////// Initialize
    int maxonMotorSocketCount = 0;    // 1 이상이면 virtual Maxon Motor를 사용하기 위해

    void initializeMotors();
    void initializeCanManager();
    void motorSettingCmd();
    bool initializePos(const std::string &input);

    //////////////////////////////////////////////////////////////// Exit
    void deactivateControlTask();

    //////////////////////////////////////////////////////////////// Maxon 모터 초기화 함수
    void maxonMotorEnable();
    void setMaxonMotorMode(std::string targetMode);

    //////////////////////////////////////////////////////////////// Ideal State
    void displayAvailableCommands(string flagName) const;
    void processInput(const std::string &input, string flagName);
    void idealStateRoutine();

    //////////////////////////////////////////////////////////////// AddStance State
    void sendAddStanceProcess();
    
    //////////////////////////////////////////////////////////////// Play State
    MatrixXd measureMatrix;     ///< 궤적을 생성하기 위해 읽은 악보 부분 (마디)
    const double measureThreshold = 2.4;     ///< 한번에 읽을 악보의 크기. [s]
    double measureTotalTime = 0.0;     ///< 악보를 읽는 동안 누적 시간. [s]
    bool endOfScore = false;           ///< 악보의 종료 코드 확인

    std::string txtBasePath = "/home/shy/DrumRobot/include/codes/";    // 악보 위치
    std::string wavBasePath = "/home/shy/DrumRobot/include/music/";    // 음악 위치
    std::string magentaPath = "/home/shy/DrumRobot/DrumSound/";        // 마젠타 경로
    std::string wavPath;        // wav 파일 경로

    // 싱크 연주를 위한 변수들
    bool playMusic = false;
    std::string syncPath = "/home/shy/DrumRobot/include/sync/sync.txt";      // 싱크 파일 경로
    std::chrono::system_clock::time_point syncTime;
    bool setWaitingTime = false;
    bool runPython = false;
    int pythonClass = 0;    // 어떤 파이썬을 실행할지 (1 : 시간 측정, 0 : 마젠타)

    // 마젠타 반복 생성을 위한 변수들
    int repeatNum = 1;
    int currentIterations = 1;
    queue<float> delayTime;
    float delayTime_i;
    queue<float> makeTime;
    float makeTime_i;
    queue<float> recordTime;
    float recordTime_i;
    queue<float> waitTime;
    float waitTime_i;

    void initializePlayState();
    void setSyncTime(int waitingTime);
    std::string selectPlayMode();
    string trimWhitespace(const std::string &str);
    bool readMeasure(ifstream& inputFile);  // 한번에 읽을 악보의 크기(measureThreshold)만큼 읽으면 true 반환
    void sendPlayProcess();

    //////////////////////////////////////////////////////////////// python (magenta)

    void runPythonForMagenta();         // 기존 파이썬 코드 실행 후 악보 생성
    void getMagentaSheet(std::string midPath, std::string veloPath ,int recordingIndex);    // 파이썬 코드 실행 x 악보만 생성

};
