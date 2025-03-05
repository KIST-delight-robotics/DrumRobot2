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
    
    /////////////////////////////////////////////////////////////////////////// Play

    int lineOfScore = 0;  ///< 악보를 읽고 궤적을 생성하고 있는 줄
    MatrixXd measureMatrix; ///< 궤적을 생성하기 위해 읽은 악보 부분 (마디)

    // 궤적 저장할 구조체
    typedef struct {
        VectorXd endEffectorR; ///< 오른팔 스틱 끝 좌표 (x, y, z)
        VectorXd endEffectorL; ///< 왼팔 스틱 끝 좌표 (x, y, z)

        double wristAngleR;  ///> IK를 풀기 위한 오른 손목 각도
        double wristAngleL;  ///> IK를 풀기 위한 왼 손목 각도
    }Position;
    queue<Position> trajectoryQueue;

    bool readMeasure(ifstream& inputFile, bool &bpmFlag);
    void generateTrajectory();
    bool solveIKandPushConmmand();
    
    /////////////////////////////////////////////////////////////////////////// AddStance

    //   Ready Pos Array   :  waist         , R_arm1        , L_arm1        , R_arm2        , R_arm3        , L_arm2        , L_arm3        , R_wrist       , L_wrist       , maxonForTest
    //                       { 0            , 90            , 90            , 45            , 75            , 45            , 75            , 30            , 30            , 30                  } [deg]
    vector<float> readyArr = { 0            , M_PI / 2.0    , M_PI / 2.0    , M_PI * 0.25   , M_PI / 2.4    , M_PI * 0.25   , M_PI / 2.4    , M_PI / 6.0    , M_PI / 6.0    , 10.0 * M_PI / 180.0};

    //   Home Pos Array    : waist          , R_arm1        , L_arm1        , R_arm2    , R_arm3         , L_arm2    , L_arm3         , R_wrist        , L_wrist        , maxonForTest
    //                      { 10            , 90            , 90            , 0         , 135            , 0         , 135            , 60             , 60             , 90        } [deg]
    vector<float> homeArr = { M_PI / 18.0   , M_PI / 2.0    , M_PI / 2.0    , 0         , M_PI * (0.75)  , 0         , M_PI * (0.75)  , M_PI / 3.0   , M_PI / 3.0   , M_PI / 2.0};

    //   Back Pos Array    : waist      , R_arm1        , L_arm1        , R_arm2    , R_arm3    , L_arm2    , L_arm3    , R_wrist       , L_wrist
    //                      { 0         , 135           , 45            , 0         , 0         , 0         , 0         , 90            , 90         } [deg]
    vector<float> backArr = { 0         ,M_PI * 0.75    , M_PI * 0.25   , 0         , 0         , 0         , 0         , M_PI / 2.0    , M_PI / 2.0 };

    void getArr(vector<float> &arr);
    vector<float> makeHomeArr(int cnt);

    /////////////////////////////////////////////////////////////////////////// brake
    //                      q0  q1  q2  q3  q4  q5  q6  q7
    vector<int> brakeArr = {0, 0,  0,  0,  0,  0,  0,  0}; // 1: true // 0: false

    void toBrake(double motornum, double nowval, double nextval, double threshold); // 해당 모터의 현재값과 다음값 차이가 threshold보다 작거나 같으면 brake걸고 아니면 끄기
    void clearBrake(); // 모든 brake끄기

    /////////////////////////////////////////////////////////////////////////// 기타
    double q1_state[2] = {readyArr[1], readyArr[1]};
    double q2_state[2] = {readyArr[2], readyArr[2]};


    vector<float> FK();

private:
    TMotorCommandParser TParser; ///< T 모터 명령어 파서.
    MaxonCommandParser MParser;  ///< Maxon 모터 명령어 파서

    State &state;                                                 ///< 시스템의 현재 상태입니다.
    CanManager &canManager;                                       ///< CAN 통신을 통한 모터 제어를 담당합니다.
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors; ///< 연결된 모터들의 정보입니다.
    USBIO &usbio;
    Functions &fun;

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
        {"maxonForTest", 9}};


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

    /////////////////////////////////////////////////////////////////////////// Play (read measure)
    float bpmOfScore = 0;       ///< txt 악보의 BPM 정보.
    double threshold = 2.4;     ///< 한번에 읽을 악보의 크기. [s]
    double totalTime = 0.0;     ///< 악보를 읽는 동안 누적 시간. [s]

    void initVal();
    string trimWhitespace(const std::string &str);

    /////////////////////////////////////////////////////////////////////////// Play (parse measure)
    VectorXd initialInstrument = VectorXd::Zero(18);   // 전체 궤적에서 출발 악기
    VectorXd finalInstrument = VectorXd::Zero(18);   // 전체 궤적에서 도착 악기

    float initialTimeR, finalTimeR;       // 전체 궤적에서 출발 시간, 도착 시간
    float initialTimeL, finalTimeL;
    float t1, t2;           // 궤적 생성 시간

    MatrixXd measureState = MatrixXd::Zero(2, 3); // [이전 시간, 이전 악기, 상태] // state
                                                                                // 0 : 0 <- 0
                                                                                // 1 : 0 <- 1
                                                                                // 2 : 1 <- 0
                                                                                // 3 : 1 <- 1

    VectorXd hitState = VectorXd::Zero(4);

    void parseMeasure(MatrixXd &measureMatrix);
    pair<VectorXd, VectorXd> parseOneArm(VectorXd t, VectorXd inst, VectorXd stateVector);
    VectorXd makeState(MatrixXd measureMatrix);

    /////////////////////////////////////////////////////////////////////////// Play (make trajectory)
    double roundSum = 0;    ///< 5ms 스텝에서 누적되는 오차 보상

    // 한 줄 데이터 저장할 Matrix
    // [n min max state_hit_R state_hit_L]
    MatrixXd lineData;

    VectorXd getTargetPosition(VectorXd &inst_vector);
    float timeScaling(float ti, float tf, float t);
    VectorXd makePath(VectorXd Pi, VectorXd Pf, float s);
    void saveLineData(int n, VectorXd minmax, VectorXd intesity);
    VectorXd waistRange(VectorXd &pR, VectorXd &pL);
    VectorXd getWristHitAngle(VectorXd &inst_vector);

    /////////////////////////////////////////////////////////////////////////// Play (solve IK)
    int indexSolveIK = 0;

    float getLength(double theta);
    double getTheta(float l1, double theta);
    void solveIK(VectorXd &q, double q0);
    VectorXd IKFixedWaist(VectorXd &pR, VectorXd &pL, double theta0, double theta7, double theta8);
    void pushConmmandBuffer(VectorXd &Qi);
    
    /////////////////////////////////////////////////////////////////////////// Play (waist)
    MatrixXd waistCoefficient;
    double q0_t1;               // 시작 위치 저장
    double q0_t0, t0 = -1;      // 이전 위치, 이전 시간 저장

    // threshold 관련 변수
    double nextq0_t1;
    int status = 0;
    double q0_threshold = 0.01;

    vector<double> cubicInterpolation(const vector<double>& q, const vector<double>& t);
    std::pair<double, vector<double>> getQ0t2(int mode);
    void getWaistCoefficient();
    double getWaistAngle(int i);
    
    /////////////////////////////////////////////////////////////////////////// Play (wrist & elbow)
    // 0.5초 기준 각도
    const float baseTime = 0.5;
    const float wristStayBaseAngle = 10.0 * M_PI / 180.0;
    const float wristContactBaseAngle = 5.0 * M_PI / 180.0;
    const float wristLiftBaseAngle = 25.0 * M_PI / 180.0;

    const float elbowStayBaseAngle = 5.0 * M_PI / 180.0;
    const float elbowLiftBaseAngle = 15.0 * M_PI / 180.0;
    
    // 타격 궤적 생성 파라미터
    typedef struct {

        // 각도
        float wristStayAngle = 10.0 * M_PI / 180.0;
        float wristContactAngle = 5.0 * M_PI / 180.0;
        float wristLiftAngle = 25.0 * M_PI / 180.0;

        float elbowStayAngle = 5.0 * M_PI / 180.0;
        float elbowLiftAngle = 10.0 * M_PI / 180.0;

        // 시간
        float wristStayTime;
        float wristContactTime;
        float wristReleaseTime;
        float wristLiftTime;

        float elbowStayTime;
        float elbowLiftTime;

    }HitParameter;

    float makeElbowAngle(float t1, float t2, float t, int state, HitParameter param, int intensity, bool targetChangeFlag);
    float makeWristAngle(float t1, float t2, float t, int state, HitParameter param, int intensity, bool targetChangeFlag);
    float makeWristAngle_TEST(float t1, float t2, float t, int state, HitParameter param, int intensity, bool targetChangeFlag, bool &hitting, float hittingPos);

    HitParameter getHitParameter(float t1, float t2, int hitState, HitParameter preParam, int intensity);
    VectorXd makeHitTrajetory(float t1, float t2, float t, int hitState, int wristIntesity, bool targetChangeFlag);

    HitParameter preParametersR, preParametersL, preParametersTmp;

    float nnR, ntR, nnL, ntL; // 1박 타격 시간이 짧은 경우 2박 시간 사용하기 위한 변수 (nextn, nextt)
    bool readyRflag = 0;
    bool readyLflag = 0; // 미리 시작하는 경우 플래그 
    int next_stateR, next_stateL;
    int next_intensityR, next_intensityL;
    int i_wristR, i_wristL = 0; 
    bool shadow_flag = 0;

    void getHitAngle(VectorXd &q, int index);

    /////////////////////////////////////////////////////////////////////////// Play (dijkstra)
    double imin, imax, fmin, fmax;
    double preq0_t1;
    void updateRange(const VectorXd& output, double& min, double& max);
    struct Node {
        int x_idx;
        double y_val;
        double cost;
        bool operator>(const Node &other) const {
            return cost > other.cost;
        }
    };
    int y_to_index(double y, double global_y_min, double step_size);
    double select_top10_with_median(const vector<double>& y_vals, double current_y, double y_min, double y_max);
    double dijkstra_top10_with_median(const vector<double>& x_values, const vector<pair<double, double>>& y_ranges, double start_y);

    /////////////////////////////////////////////////////////////////////////// AddStance
    vector<float> currentMotorAngle = {0, 0, 0, 0, 0, 0, 0, 0, 0}; ///< 경로 생성 시 사용되는 현재 모터 위치 값

    // q1[rad], q2[rad], acc[rad/s^2], t2[s]
    VectorXd calVmax(VectorXd &q1, VectorXd &q2, float acc, float t2);
    // q1[rad], q2[rad], Vmax[rad/s], acc[rad/s^2], t[s], t2[s]
    VectorXd makeProfile(VectorXd &q1, VectorXd &q2, VectorXd &Vmax, float acc, float t, float t2);
    void getMotorPos();
};