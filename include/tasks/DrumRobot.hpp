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
#include <termios.h>

#include "SystemState.hpp"
#include "../include/managers/CanManager.hpp"
#include "../include/managers/PathManager.hpp"
#include "../include/motors/CommandParser.hpp"
#include "../include/motors/Motor.hpp"
#include "../include/managers/TestManager.hpp"
#include "../include/USBIO_advantech/USBIO_advantech.hpp"
#include "../include/tasks/Functions.hpp"
#include "../DynamixelSDK-3.8.4/c++/include/dynamixel_sdk/dynamixel_sdk.h"

using namespace std;

class FlagClass
{
public:
    FlagClass();
    ~FlagClass();

    // AddStance
    enum AddStanceFlag
    {
        START,
        HOME,
        READY,
        SHUTDOWN
    };

    void setAddStanceFlag(AddStanceFlag flag);
    string getAddStanceFlag();

    // fixed
    void setFixationFlag(string flagName);
    bool getFixationFlag();

private:

    AddStanceFlag addStanceFlag = START;
    bool isFixed = false;
};

class DXL
{
public:
    DXL();
    ~DXL();

    void initialize();
    void DXLTorqueOff();
    void syncWrite(vector<vector<float>> command);
    std::map<int, float> syncRead();

private:
    dynamixel::PortHandler *port;
    dynamixel::PacketHandler *pkt;
    std::unique_ptr<dynamixel::GroupSyncWrite> sw;
    std::unique_ptr<dynamixel::GroupSyncRead> sr;

    bool useDXL = false;
    vector<uint8_t> motorIDs; // 연결된 다이나믹셀 ID
    
    int32_t angleToTick(float angle);
    float tickToAngle(int32_t ticks);
    void commandToValues(int32_t values[], vector<float> command);
};

class Arduino
{
public:
    Arduino();
    ~Arduino();

    enum Action
    {
        POWER_ON,
        IDLE,
        PLAYING,
        POWER_OFF
    };

    bool connect(const char* port_name);
    void disconnect();
    void setHeadLED(Action action);

private:
    int arduino_port;
    bool is_connected = false;
    Action headLED = POWER_OFF;

    bool sendCommand(int command_num);

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
              Functions &funcRef);

    // thread
    void stateMachine();
    void sendLoopForThread();
    void recvLoopForThread();
    void musicMachine();
    void runPythonInThread();

    // init
    void initializeDrumRobot();
    
private:
    State &state;
    CanManager &canManager;
    PathManager &pathManager;
    TestManager &testManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;
    USBIO &usbio;
    Functions &func;

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
    const float jointRangeMax[12] = {   90.0,   150.0,  180.0,  90.0,   140.1,  90.0,   140.1,  135.0,  135.0,      135.0,     200.0,  200.0};
    const float jointRangeMin[12] = {   -90.0,  0.0,    30.0,   -60.0,    0.0,  -60.0,    0.0,  -108.0, -108.0,     -90.0,     -90.0,  -90.0};
    // TestManager.hpp 에서도 수정해줘야 함

    FlagClass flagObj;
    bool allMotorsUnConected = true;    // 모든 모터 연결 안됨 - 모터 없이 테스트하는 경우
    DXL dxl;
    Arduino arduino;

    //////////////////////////////////////////////////////////////// Initialize
    int maxonMotorSocketCount = 0;    // 1 이상이면 virtual Maxon Motor를 사용하기 위해

    void initializeMotors();
    void initializeCanManager();
    void motorSettingCmd();
    void initializeFolder();
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
    void runAddStanceProcess();
    
    //////////////////////////////////////////////////////////////// Play State
    MatrixXd measureMatrix;     ///< 궤적을 생성하기 위해 읽은 악보 부분 (마디)
    const double measureThreshold = 2.4;     ///< 한번에 읽을 악보의 크기. [s]
    double measureTotalTime = 0.0;     ///< 악보를 읽는 동안 누적 시간. [s]
    bool endOfScore = false;           ///< 악보의 종료 코드 확인

    std::string txtBaseFolderPath = "../include/codes/";    // 악보 폴더 경로
    std::string wavBaseFolderPath = "../include/music/";    // 음악 폴더 경로
    std::string magentaCodePath = "../include/magenta/codeMagenta";        // 마젠타 생성 악보 경로
    std::string wavPath;        // wav 파일 경로

    // 싱크 연주를 위한 변수들
    bool playMusic = false;
    std::string syncPath = "../magenta/sync/sync.txt";      // 싱크 파일 경로
    std::chrono::system_clock::time_point syncTime;
    bool setWaitingTime = false;
    bool runPython = false;
    std::string pythonArgs;
    std::string pythonScript = "../magenta/magenta_env/bin/python3 ../magenta/script.py";

    // 마젠타 반복 생성을 위한 변수들
    int repeatNum = 1;
    int currentIterations = 1;
    queue<int> delayTime;
    queue<int> makeBarNum;
    queue<int> recordBarNum;
    queue<float> waitTime;

    void initializePlayState();
    void setSyncTime(int waitingTime);
    void displayPlayCommands(bool useMagenta, bool useDrumPad, float inputWaitMs, std::string txtFileName);
    void setPythonArgs();
    bool checkPreconditions(bool useMagenta, std::string txtPath);
    std::string selectPlayMode();
    string trimWhitespace(const std::string &str);
    bool readMeasure(ifstream& inputFile);  // 한번에 읽을 악보의 크기(measureThreshold)만큼 읽으면 true 반환
    void runPlayProcess();

    //////////////////////////////////////////////////////////////// 

    void generateCodeFromMIDI(std::string midPath, std::string veloPath, int recordingIndex, bool endFlag);

};
