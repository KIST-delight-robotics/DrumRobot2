#pragma once

#include <stdio.h>
#include "../include/managers/CanManager.hpp"
#include "../include/motors/CommandParser.hpp"
#include "../include/motors/Motor.hpp"
#include "../include/tasks/SystemState.hpp"
#include "../include/USBIO_advantech/USBIO_advantech.hpp"
#include "../include/tasks/Functions.hpp"

#include <iostream>
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
#include <numeric>
#include <signal.h>
#include "../include/eigen-3.4.0/Eigen/Dense"

//For Qt
/*
#include "CanManager.hpp"
#include "../motors/CommandParser.hpp"
#include "../motors/Motor.hpp"
#include "../tasks/SystemState.hpp"
*/

using namespace std;
using namespace Eigen;

/**
 * @class PathManager
 * @brief 드럼 로봇의 연주 경로를 생성하는 부분을 담당하는 클래스입니다.
 *
 * 이 클래스는 주어진 악보를 분석하여 정해진 알고리즘을 따라 알맞은 경로를 생성하도록 합니다.
 */

class PathManager
{

public:
    
    PathManager(State &stateRef,
                CanManager &canManagerRef,
                std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef,
                USBIO &usbioRef,
                Functions &funRef);

    
    /////////////////////////////////////////////////////////////////////////// Init

    void getDrumPositoin();
    void setReadyAngle();

    /////////////////////////////////////////////////////////////////////////// AddStance
    void pushAddStancePath(string flagName);
    
    /////////////////////////////////////////////////////////////////////////// Play
    bool endOfPlayCommand = false;
    int MaxonMode;
    double bpmOfScore = 0;      ///< 악보의 BPM 정보.

    void initializeValue(int bpm, int mode);
    void generateTrajectory(MatrixXd &measureMatrix);
    void solveIKandPushCommand();

private:
    TMotorCommandParser TParser; ///< T 모터 명령어 파서.
    MaxonCommandParser MParser;  ///< Maxon 모터 명령어 파서

    State &state;                                                 ///< 시스템의 현재 상태입니다.
    CanManager &canManager;                                       ///< CAN 통신을 통한 모터 제어를 담당합니다.
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors; ///< 연결된 모터들의 정보입니다.
    USBIO &usbio;
    Functions &fun;

    typedef struct{

        float upperArm = 0.250;         ///< 상완 길이. [m]
        float lowerArm = 0.328;         ///< 하완 길이. [m]
        float stick = 0.325+0.048;      ///< 스틱 길이 + 브라켓 길이. [m]
        float waist = 0.520;            ///< 허리 길이. [m]
        float height = 1.020-0.0605;    ///< 바닥부터 허리까지의 높이. [m]

    }PartLength;

    /////////////////////////////////////////////////////////////////////////// Init
    MatrixXd drumCoordinateR;                               ///< 오른팔의 각 악기별 위치 좌표 벡터.
    MatrixXd drumCoordinateL;                                ///< 왼팔의 각 악기별 위치 좌표 벡터.

    MatrixXd wristAnglesR;                               ///< 오른팔의 각 악기별 타격 시 손목 각도.
    MatrixXd wristAnglesL;                                ///< 왼팔의 각 악기별 타격 시 손목 각도.

    // AddStace 에서 사용하는 위치
    VectorXd readyAngle;
    VectorXd homeAngle;
    VectorXd shutdownAngle;

    /////////////////////////////////////////////////////////////////////////// AddStance
    MatrixXd addStanceCoefficient;
    
    // q1[rad], q2[rad], acc[rad/s^2], t2[s]
    VectorXd calVmax(VectorXd &q1, VectorXd &q2, float acc, float t2);
    // q1[rad], q2[rad], Vmax[rad/s], acc[rad/s^2], t[s], t2[s]
    VectorXd makeProfile(VectorXd &q1, VectorXd &q2, VectorXd &Vmax, float acc, float t, float t2);
    VectorXd getMotorPos();
    void getAddStanceCoefficient(VectorXd Q1, VectorXd Q2, double t);

    /////////////////////////////////////////////////////////////////////////// Parse Measure
    VectorXd initialInstrument = VectorXd::Zero(18);   // 전체 궤적에서 출발 악기
    VectorXd finalInstrument = VectorXd::Zero(18);   // 전체 궤적에서 도착 악기

    float initialTimeR, finalTimeR;       // 전체 궤적에서 출발 시간, 도착 시간
    float initialTimeL, finalTimeL;
    float t1, t2;           // 궤적 생성 시간

    VectorXd drumR;     // 오른손 악기 전체 저장
    VectorXd drumL;     // 왼손 악기 전체 저장
    VectorXd timeR;
    VectorXd timeL;
    VectorXd statesNTimeR;    // 오른손 state, time 전체 저장
    VectorXd statesNTimeL;    // 왼손 state, time 전체 저장

    MatrixXd measureState = MatrixXd::Zero(2, 3); // [이전 시간, 이전 악기, 상태] // state
                                                                                // 0 : 0 <- 0
                                                                                // 1 : 0 <- 1
                                                                                // 2 : 1 <- 0
                                                                                // 3 : 1 <- 1

    VectorXd hitState;
    VectorXd intensity;

    void parseMeasure(MatrixXd &measureMatrix);
    pair<VectorXd, VectorXd> parseOneArm(VectorXd t, VectorXd inst, VectorXd stateVector);
    VectorXd makeState(MatrixXd measureMatrix);

    /////////////////////////////////////////////////////////////////////////// Make Trajectory
    // 궤적 저장할 구조체
    typedef struct {
        VectorXd trajectoryR; ///< 오른팔 스틱 끝 좌표 (x, y, z)
        VectorXd trajectoryL; ///< 왼팔 스틱 끝 좌표 (x, y, z)

        double wristAngleR;  ///> IK를 풀기 위한 오른 손목 각도
        double wristAngleL;  ///> IK를 풀기 위한 왼 손목 각도
    }Position;
    queue<Position> trajectoryQueue;

    double roundSum = 0;    ///< 5ms 스텝 단위에서 누적되는 오차 보상

    // 악보 한 줄의 데이터 저장한 Matrix
    // [명령 개수 / q0 최적값 / q0 min / q0 max / t1 / t2 / state R / state L / intensity R / intensity L]
    MatrixXd lineData;

    pair<VectorXd, VectorXd> getTargetPosition(VectorXd inst_vector);
    float timeScaling(float ti, float tf, float t);
    VectorXd makePath(VectorXd Pi, VectorXd Pf, float s);
    void saveLineData(int n, VectorXd minmax);
    VectorXd calWaistAngle(VectorXd pR, VectorXd pL);

    /////////////////////////////////////////////////////////////////////////// Waist
    MatrixXd waistCoefficient;
    double q0_t1;               // 시작 위치 저장
    double q0_t0, t0 = -1;      // 이전 위치, 이전 시간 저장

    // threshold 관련 변수
    double nextq0_t1;
    int status = 0;
    double q0_threshold = 0.01;

    vector<double> cubicInterpolation(const vector<double>& q, const vector<double>& t);
    std::pair<double, vector<double>> getNextQ0();
    void makeWaistCoefficient();
    double getWaistAngle(int i);

    /////////////////////////////////////////////////////////////////////////// Solve IK
    float getLength(double theta);
    double getTheta(float l1, double theta);
    VectorXd solveIK(double q0);
    VectorXd IKFixedWaist(VectorXd pR, VectorXd pL, double theta0, double theta7, double theta8);

    /////////////////////////////////////////////////////////////////////////// Wrist (Hit)
    typedef struct {
        double stayTime;
        double liftTime;
        double hitTime;     // 전체 시간
    }elbowTime;

    typedef struct {
        double releaseTime;
        double stayTime;
        double liftTime;
        double hitTime;     // 전체 시간
    }wristTime;

    typedef struct {
        double stayAngle = 5*M_PI/180.0;
        double liftAngle = 15*M_PI/180.0;
    }elbowAngle;

    typedef struct {
        double stayAngle = 10*M_PI/180.0;
        double pressAngle = -5*M_PI/180.0;
        double liftAngle = 30*M_PI/180.0;
    }wristAngle;
    
    elbowTime elbowTimeR, elbowTimeL;
    wristTime wristTimeR, wristTimeL;
    
    MatrixXd elbowCoefficientR;
    MatrixXd elbowCoefficientL;
    MatrixXd wristCoefficientR;
    MatrixXd wristCoefficientL;

    void makeHitCoefficient();
    PathManager::elbowTime getElbowTime(float t1, float t2, int intensity);
    PathManager::wristTime getWristTime(float t1, float t2, int intensity);
    PathManager::elbowAngle getElbowAngle(float t1, float t2, int intensity);
    PathManager::wristAngle getWristAngle(float t1, float t2, int intensity);
    MatrixXd makeElbowCoefficient(int state, elbowTime eT, elbowAngle eA);
    MatrixXd makeWristCoefficient(int state, wristTime wT, wristAngle wA);
    double makeElbowAngle(double t, elbowTime eT, MatrixXd coefficientMatrix);
    double makeWristAngle(double t, wristTime wT, MatrixXd coefficientMatrix);
    void generateHit(VectorXd &q, int index);
    MatrixXd makeState(VectorXd drums, VectorXd time, bool dir);
    VectorXd makeTempState(VectorXd drums);
    MatrixXd makeArrangedState(VectorXd drums, VectorXd time, float threshold);

    /////////////////////////////////////////////////////////////////////////// Push Command Buffer
    void pushCommandBuffer(VectorXd Qi);

    /////////////////////////////////////////////////////////////////////////// brake
    void clearBrake(); // 모든 brake끄기

    /////////////////////////////////////////////////////////////////////////// Detect Collision
    int aNumOfLine = 0;
    string trimWhitespace(const std::string &str);
    int predictCollision(MatrixXd measureMatrix);
    MatrixXd parseAllLine(VectorXd t, VectorXd inst, VectorXd stateVector, char RL);
    MatrixXd getOneDrumPosition(int InstNum, char RL);
    bool checkTable(VectorXd PR, VectorXd PL, double hitR, double hitL);
    bool hex2TableData(char hex1, char hex2, int index);

};