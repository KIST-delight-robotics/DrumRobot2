#pragma once

#include <stdio.h>
#include "../include/managers/CanManager.hpp"
#include "../include/managers/PathManager.hpp"
#include "../include/motors/CommandParser.hpp"
#include "../include/motors/Motor.hpp"
#include "../include/tasks/SystemState.hpp"
#include "../include/USBIO_advantech/USBIO_advantech.hpp"
#include "../include/tasks/Functions.hpp"
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

using namespace std;

class TestManager
{
public:
    TestManager(State &stateRef, CanManager &canManagerRef, std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef, USBIO &usbioRef, Functions &funRef);

    void SendTestProcess();

    bool isMaxonEnable = false;
    bool hitTest = 0;

private:

    chrono::system_clock::time_point standardTime;
    State &state;
    CanManager &canManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;
    USBIO &usbio;
    Functions &fun;


    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;
    TMotorServoCommandParser tservocmd;

    bool error = false;

    // Robot Parameters
    typedef struct{

        // float upperArm = 0.250;         ///< 상완 길이. [m]
        float upperArm = 0.230;         ///< 상완 길이. [m]
        // float lowerArm = 0.328;         ///< 하완 길이. [m]
        // float lowerArm = 0.178;         ///< 하완 길이. [m]
        float lowerArm = 0.200;         ///< 하완 길이. [m]
        float stick = 0.325+0.048;      ///< 스틱 길이 + 브라켓 길이. [m]
        float waist = 0.520;            ///< 허리 길이. [m]
        float height = 1.020-0.0605;    ///< 바닥부터 허리까지의 높이. [m]

    }PartLength;

    /*For SendTestProcess*/
    int method = 0;
    float q[12] = {0.0};

    // 로봇의 관절각 범위
    //                                 Waist    Rarm1   Larm1   Rarm2   Rarm3   Larm2   Larm3   Rwrist  Lwrist   maxonForTest  Rfoot   Lfoot    [deg]
    const float jointRangeMax[12] = {   90.0,   150.0,  180.0,  90.0,   140.0,  90.0,   140.0,  135.0,  135.0,      135.0,     200.0,  200.0};
    const float jointRangeMin[12] = {   -90.0,  0.0,    30.0,   -60.0,    0.0,  -60.0,    0.0,  -108.0, -108.0,     -90.0,     -90.0,  -90.0};

    std::shared_ptr<MaxonMotor> virtualMaxonMotor;
    int maxonMotorCount = 0;
    struct can_frame frame;

    /*Value Test Code*/
    void getMotorPos(float c_MotorAngle[]);
    vector<float> makeProfile(float Q1[], float Q2[], vector<float> &Vmax, float acc, float t, float t2);
    vector<float> cal_Vmax(float q1[], float q2[],  float acc, float t2);
    vector<float> ikfun_final(float pR[], float pL[]);
    void FK(float arr[]);
    void getArr(float arr[]);

    // setQ

    bool brake_flag[7] = {false, false, false, false, false, false, false};
    bool single_brake_flag[7] = {false, false, false, false, false, false, false};
    float brake_start_time[7] = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
    float brake_end_time[7] = {4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0};
    float t = 4.0;
    float extra_time = 1.0;
    int n_repeat = 1;

    // setXYZ
    float R_xyz[3] = {0.0};
    float L_xyz[3] = {0.0};

    // TestMaxon
    string curMode;
    int hitMode = 1;
    bool isReached = false;
    float torque = 0;
    double dt = 0.005; // 0.001로 세팅하면 천천히, TimeCheck를 1ms단위마다 돌게 해야함.
    tuple <double, int, int, int> params;
    double hit_time;
    int repeat;
    int hitstate;
    int intensity;
    float Kp_normal, Kd_normal;
    float Kp_hit, Kd_hit;
    double pre_err;
    int index;
    double hit_duration;
    double release_duration;

    float makeWristAngle(float t1, float t2, float t, int state, int intensity, shared_ptr<MaxonMotor> maxonMotor);
    float makeWristAngle_CST(float t1, float t2, float t, int state, int intensity, bool &hitting, float hittingPos);
    float makeWristAngle_TC(float t1, float t2, float t, int state, int intensity, shared_ptr<MaxonMotor> maxonMotor);
    tuple <double, int, int, int> MaxonHitLoop();
    float getDesiredTorque(float desiredPosition, shared_ptr<MaxonMotor> maxonMotor);
    
    // test table
    void testTable();
    string trimWhitespace(const std::string &str);
    bool hex2TableData(char hex1, char hex2, int index);

    VectorXd IKFixedWaist(VectorXd pR, VectorXd pL, double theta0);
    VectorXd calWaistAngle(VectorXd pR, VectorXd pL);
};