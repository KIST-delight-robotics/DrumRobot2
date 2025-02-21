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

    void SendTestProcess(int periodMicroSec);
    void MaxonEnable();
    void setMaxonMode(std::string targetMode);

    bool isMaxonEnable = false;
    bool hitTest = 0;

    vector<vector<float>> Input_pos;

private:

    chrono::system_clock::time_point SendStandard;
    State &state;
    CanManager &canManager;
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors;
    USBIO &usbio;
    Functions &fun;

    TMotorCommandParser tmotorcmd;
    MaxonCommandParser maxoncmd;
    TMotorServoCommandParser tservocmd;

    vector<string> InputData;
    bool error = false;

    // Robot Parameters
    float part_length[6] = {0.250, 0.328, 0.250, 0.328, 0.325+0.048, 0.325+0.048};
    float s = 0.520;  ///< 허리 길이.
    float z0 = 1.020-0.0605; ///< 바닥부터 허리까지의 높이.

    /*For SendTestProcess*/
    int method = 0;
    float q[10] = {0.0};

    map<std::string, int> motor_mapping = { //< 각 관절에 해당하는 열 정보.
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

    //                                 Waist  Rarm1   Larm1   Rarm2   Rarm3   Larm2   Larm3   Rwrist  Lwrist  maxonForTest  Rfoot   Lfoot   [deg]
    const float joint_range_max[12] = {90.0,  150.0,  180.0,  90.0,   130.0,  90.0,   130.0,  135.0,  135.0,  135.0,        135.0,  135.0};
    const float joint_range_min[12] = {-90.0, 0.0,    30.0,   -60.0,  -30.0,  -60.0,  -30.0,  -108.0, -108.0, -108.0,      -90.0,  -90.0};

    std::shared_ptr<MaxonMotor> virtualMaxonMotor;
    int maxonMotorCount = 0;
    struct can_frame frame;

    /*Value Test Code*/
    void getMotorPos(float c_MotorAngle[]);
    vector<float> makeProfile(float Q1[], float Q2[], vector<float> &Vmax, float acc, float t, float t2);
    vector<float> cal_Vmax(float q1[], float q2[],  float acc, float t2);
    vector<float> sinProfile(float q1[], float q2[], float t, float t2);
    vector<float> ikfun_final(float pR[], float pL[], float part_length[], float s, float z0);
    void fkfun(float arr[]);
    void GetArr(float arr[]);

    // setQ
    bool sin_flag = false;
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
    double dt = 0.005;
    tuple <double, int, int> params;
    double hit_time;
    int repeat;
    int hitstate;
    int intensity;
    void setMaxonMotorMode(std::string targetMode, string motorName);
    void maxonMotorEnable();
    void CSTLoop();
    void TestStickLoop();
    void TestStick(const std::string selectedMotor, int des_tff, float tffThreshold, float posThreshold, int backTorqueUnit);
    float makeWristAngle(float t1, float t2, float t, int state, int intensity, bool &hitting, float hittingPos);
    tuple <double, int, int> CSTHitLoop();

    void UnfixedMotor();
};