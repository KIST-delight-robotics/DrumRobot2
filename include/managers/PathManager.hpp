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
    
    bool endOfPlayCommand = false;
    bool startOfPlay = false;
    //
    double bpmOfScore = 0;      ///< 악보의 BPM 정보.
    string MaxonMode = "CSP";
    int Kp, Kd;

    void initPathManager();
    void pushAddStancePath(string flagName);
    void initPlayStateValue();
    void processLine(MatrixXd &measureMatrix);

private:
    TMotorCommandParser TParser; ///< T 모터 명령어 파서.
    MaxonCommandParser MParser;  ///< Maxon 모터 명령어 파서

    State &state;                                                 ///< 시스템의 현재 상태입니다.
    CanManager &canManager;                                       ///< CAN 통신을 통한 모터 제어를 담당합니다.
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors; ///< 연결된 모터들의 정보입니다.
    USBIO &usbio;
    Functions &fun;

    /////////////////////////////////////////////////////////////////////////// struct def
    // 로봇 링크 길이 저장할 구조체
    typedef struct{

        // float upperArm = 0.250;         ///< 상완 길이. [m]
        float upperArm = 0.230;         ///< 상완 길이. [m]
        // float lowerArm = 0.328;         ///< 하완 길이. [m]
        // float lowerArm = 0.178;         ///< 하완 길이. [m]
        float lowerArm = 0.200;         ///< 하완 길이. [m]
        float stick = 0.325+0.048;      ///< 스틱 길이 + 브라켓 길이. [m]
        float waist = 0.520;            ///< 허리 길이. [m]
        float height = 1.020-0.0605;    ///< 바닥부터 허리까지의 높이. [m]

        // TestManager.hpp 에서도 수정해줘야 함

    }PartLength;

    // measureMatrix 파싱한 데이터
    typedef struct {
        double t1, t2;    // 궤적 생성 시간

        double initialTimeR, finalTimeR;       // 전체 궤적에서 출발 시간, 도착 시간
        double initialTimeL, finalTimeL;

        VectorXd initialPositionR, finalPositionR;  // 전체 궤적에서 출발 위치
        VectorXd initialPositionL, finalPositionL;  // 전체 궤적에서 도착 위치

        double initialWristAngleR, finalWristAngleR;    // 손목 각도 출발 위치
        double initialWristAngleL, finalWristAngleL;    // 손목 각도 도착 위치

        VectorXd nextStateR;
        VectorXd nextStateL;

    }parsedData;

    // task space 궤적 저장할 구조체
    typedef struct {
        VectorXd trajectoryR; ///< 오른팔 스틱 끝 좌표 (x, y, z)
        VectorXd trajectoryL; ///< 왼팔 스틱 끝 좌표 (x, y, z)

        double wristAngleR;  ///> IK를 풀기 위한 오른 손목 각도
        double wristAngleL;  ///> IK를 풀기 위한 왼 손목 각도
    }Position;

    // 허리 파라미터 저장할 구조체
    typedef struct {
        int n;                  // 명령 개수
        double min_q0;          // 최소
        double max_q0;          // 최대
        double optimized_q0;    // 최적화
    }waistParameter;

    // 타격 궤적 저장할 구조체
    typedef struct {
        double elbowR;  ///> 오른팔 팔꿈치 관절에 더해줄 각도
        double elbowL;  ///> 왼팔 팔꿈치 관절에 더해줄 각도
        double wristR;  ///> 오른팔 손목 관절에 더해줄 각도
        double wristL;  ///> 왼팔 손목 관절에 더해줄 각도
        double bass;    ///> 오른발 관절에 더해줄 각도
        double hihat;   ///> 왼발 관절에 더해줄 각도
    }HitAngle;

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
        double stayTime;
        double liftTime;
        double hitTime;
    }bassTime;
    
    typedef struct {
        double settlingTime;
        double liftTime;
        double hitTime;
        double splashTime;
    }HHTime;

    typedef struct {
        // double stayAngle = 5*M_PI/180.0;
        double stayAngle = 0.0;
        double liftAngle = 15*M_PI/180.0;
    }elbowAngle;

    typedef struct {
        double stayAngle = 10*M_PI/180.0;
        double pressAngle = -5*M_PI/180.0;
        double liftAngle = 40*M_PI/180.0;
    }wristAngle;

    typedef struct {
        double stayAngle = 0*M_PI/180.0;
        double pressAngle = -20*M_PI/180.0;
    }bassAngle;

    typedef struct {
        double openAngle = -3*M_PI/180.0;
        double closedAngle = -13*M_PI/180.0;
    }HHAngle;

    /////////////////////////////////////////////////////////////////////////// Init
    MatrixXd drumCoordinateR;                               ///< 오른팔의 각 악기별 위치 좌표 벡터.
    MatrixXd drumCoordinateL;                                ///< 왼팔의 각 악기별 위치 좌표 벡터.

    MatrixXd wristAngleOnImpactR;                               ///< 오른팔의 각 악기별 타격 시 손목 각도.
    MatrixXd wristAngleOnImpactL;                                ///< 왼팔의 각 악기별 타격 시 손목 각도.

    void setDrumCoordinate();
    void setWristAngleOnImpact();
    void setAddStanceAngle();
    void setReadyAngle();

    /////////////////////////////////////////////////////////////////////////// AddStance
    VectorXd readyAngle;                // AddStace 에서 사용하는 위치 (자세)
    VectorXd homeAngle;
    VectorXd shutdownAngle;

    // q1[rad], q2[rad], acc[rad/s^2], t2[s]
    VectorXd calVmax(VectorXd &q1, VectorXd &q2, float acc, float t2);
    // q1[rad], q2[rad], Vmax[rad/s], acc[rad/s^2], t[s], t2[s]
    VectorXd makeProfile(VectorXd &q1, VectorXd &q2, VectorXd &Vmax, float acc, float t, float t2);
    VectorXd getFinalMotorPosition();

    /////////////////////////////////////////////////////////////////////////// Play
    int lineOfScore = 0;            ///< 현재 악보 읽은 줄.
    const int preCreatedLine = 3;   ///< 미리 궤적을 생성할 줄

    void avoidCollision(MatrixXd &measureMatrix);
    void genTrajectory(MatrixXd &measureMatrix);
    void solveIKandPushCommand();

    //////////////////////////////////// Task Space Trajectory
    double n;
    double roundSum = 0.0;    ///< 5ms 스텝 단위에서 누적되는 오차 보상

    VectorXd measureStateR, measureStateL;  // [이전 시간, 이전 악기, 상태] 0 : 0 <- 0 / 1 : 0 <- 1 / 2 : 1 <- 0 / 3 : 1 <- 1

    queue<Position> trajectoryQueue;
    queue<waistParameter> waistParameterQueue;

    int genTaskSpaceTrajectory(MatrixXd &measureMatrix);
    PathManager::parsedData parseMeasure(MatrixXd &measureMatrix, VectorXd &stateR, VectorXd &stateL);
    pair<VectorXd, VectorXd> parseOneArm(VectorXd &t, VectorXd &inst, VectorXd &hh, VectorXd &stateVector);
    int checkOpenHihat(int instNum, int isHH);
    pair<VectorXd, double> getTargetPosition(VectorXd &inst, char RL);
    double timeScaling(double ti, double tf, double t);
    VectorXd makePath(VectorXd &Pi, VectorXd &Pf, double s);
    VectorXd getWaistParams(VectorXd &pR, VectorXd &pL);
    void storeWaistParams(int n, VectorXd &waistParams);

    //////////////////////////////////// Hitting Trajectory
    VectorXd prevLine = VectorXd::Zero(9);  // 악보 나눌 때 시작 악보 기록

    MatrixXd hitState;              // [이전 시간, 이전 State, intensity]  R
                                    // [이전 시간, 이전 State, intensity]  L

    double hitR_t1, hitR_t2;       // 전체 타격에서 출발 시간, 도착 시간
    double hitL_t1, hitL_t2;

    queue<HitAngle> hitAngleQueue;

    double roundSumHit = 0.0;     ///< 5ms 스텝 단위에서 누적되는 오차 보상

    elbowTime elbowTimeR, elbowTimeL;
    wristTime wristTimeR, wristTimeL;
    bassTime bassTimeR;
    HHTime HHTimeL;
    
    MatrixXd elbowCoefficientR;
    MatrixXd elbowCoefficientL;
    MatrixXd wristCoefficientR;
    MatrixXd wristCoefficientL;

    void genHitTrajectory(MatrixXd &measureMatrix, int n);
    MatrixXd divideMatrix(MatrixXd &measureMatrix);
    void parseHitData(MatrixXd &measureMatrix);
    int getBassState(bool bassHit, bool nextBaseHit);
    void makeHitCoefficient();
    PathManager::elbowTime getElbowTime(float t1, float t2, int intensity);
    PathManager::wristTime getWristTime(float t1, float t2, int intensity, int state);
    PathManager::bassTime getBassTime(float t1, float t2);
    PathManager::elbowAngle getElbowAngle(float t1, float t2, int intensity);
    PathManager::wristAngle getWristAngle(float t1, float t2, int intensity);
    MatrixXd makeElbowCoefficient(int state, elbowTime eT, elbowAngle eA);
    MatrixXd makeWristCoefficient(int state, wristTime wT, wristAngle wA);
    double makecosineprofile(double qi, double qf, double ti, double tf, double t);
    void generateHit(float tHitR, float tHitL, HitAngle &Pt);
    double makeElbowAngle(double t, elbowTime eT, MatrixXd coefficientMatrix);
    double makeWristAngle(double t, wristTime wT, MatrixXd coefficientMatrix);
    double makeBassAngle(double t, bassTime bt, int bassState);
    int getHHstate(bool HHclosed, bool nextHHclosed);
    PathManager::HHTime getHHTime(float t1, float t2);
    double makeHHAngle(double t, HHTime ht, int HHstate, int nextHHclosed);

    //////////////////////////////////// Solve IK & Push Command Buffer
    double q0_t1;               // 시작 위치 저장
    double q0_t0, t0 = -1;      // 이전 위치, 이전 시간 저장
    float prevWaistPos = 0.0;   // 브레이크 판단에 사용될 허리 전 값
    float preDiff = 0.0;        // 브레이크 판단(필터)에 사용될 전 허리 차이값

    std::vector<PathManager::waistParameter> waistParamsQueueToVector();
    vector<double> cubicInterpolation(const vector<double>& q, const vector<double>& t);
    std::pair<double, vector<double>> getNextQ0(std::vector<waistParameter> &wPs);
    MatrixXd makeWaistCoefficient(std::vector<waistParameter> &wPs);
    double getWaistAngle(MatrixXd &waistCoefficient, int index);
    VectorXd getJointAngles(double q0);
    void pushCommandBuffer(VectorXd &Qi);

    //////////////////////////////////// Detect Collision
    std::string tablePath = "/home/shy/DrumRobot_table/TABLE.bin";    // 테이블 위치

    bool detectCollision(MatrixXd &measureMatrix);
    int findDetectionRange(MatrixXd &measureMatrix);
    bool checkTable(VectorXd PR, VectorXd PL, double hitR, double hitL);
    size_t getFlattenIndex(const std::vector<size_t>& indices, const std::vector<size_t>& dims);
    std::pair<size_t, size_t> getBitIndex(size_t offsetIndex);

    //////////////////////////////////// Avoid Collision
    map<int, std::string> modificationMethods = { ///< 악보 수정 방법 중 우선 순위
        { 0, "Crash"},
        { 1, "WaitAndMove"},
        { 2, "MoveAndWait"},
        { 3, "Switch"},
        { 4, "Delete"}
    };

    bool modifyMeasure(MatrixXd &measureMatrix, int priority);
    pair<int, int> findModificationRange(VectorXd t, VectorXd instR, VectorXd instL);
    bool modifyCrash(MatrixXd &measureMatrix, int num);
    bool switchHands(MatrixXd &measureMatrix, int num);
    bool waitAndMove(MatrixXd &measureMatrix, int num);
    bool moveAndWait(MatrixXd &measureMatrix, int num);
    bool deleteInst(MatrixXd &measureMatrix, int num);

    //////////////////////////////////// IK
    VectorXd solveGeometricIK(VectorXd &pR, VectorXd &pL, double theta0, double theta7, double theta8, bool printError);
    double getLength(double theta);
    double getTheta(double l1, double theta);
};