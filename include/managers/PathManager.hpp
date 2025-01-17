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

    int line = 0;  ///< 연주를 진행하고 있는 줄
    MatrixXd measureMatrix;

    // 궤적 저장할 구조체
    typedef struct {
        // 오른팔 좌표
        VectorXd pR; // 0: x, 1: y, 2: z
        // 왼팔 좌표
        VectorXd pL; // 0: x, 1: y, 2: z
        // 브레이크
        bool brake_state[8];
        // 손목 각도
        double thetaR; 
        double thetaL;
    }Position;
    queue<Position> P_buffer;

    bool readMeasure(ifstream& inputFile, bool &BPMFlag);
    void generateTrajectory();
    bool solveIKandPushConmmand();
    
    /////////////////////////////////////////////////////////////////////////// AddStance

    //   Ready Pos Array   :  waist         , R_arm1        , L_arm1        , R_arm2        , R_arm3        , L_arm2        , L_arm3        , R_wrist       , L_wrist
    //                       { 0            , 90            , 90            , 45            , 75            , 45            , 75            , 30            , 30         } [deg]
    vector<float> readyArr = { 0            , M_PI / 2.0    , M_PI / 2.0    , M_PI * 0.25   , M_PI / 2.4    , M_PI * 0.25   , M_PI / 2.4    , M_PI / 6.0    , M_PI / 6.0 };

    //   Home Pos Array    : waist          , R_arm1        , L_arm1        , R_arm2    , R_arm3            , L_arm2    , L_arm3            , R_wrist               , L_wrist
    //                      { 10            , 90            , 90            , 0         , 120               , 0         , 120               , 95                    , 95                    } [deg]
    vector<float> homeArr = { M_PI / 18.0   , M_PI / 2.0    , M_PI / 2.0    , 0         , M_PI * (2.0/3.0)  , 0         , M_PI * (2.0/3.0)  , M_PI * (95.0/180.0)   , M_PI * (95.0/180.0)   };

    //   Back Pos Array    : waist      , R_arm1        , L_arm1        , R_arm2    , R_arm3    , L_arm2    , L_arm3    , R_wrist       , L_wrist
    //                      { 0         , 135           , 45            , 0         , 0         , 0         , 0         , 90            , 90         } [deg]
    vector<float> backArr = { 0         ,M_PI * 0.75    , M_PI * 0.25   , 0         , 0         , 0         , 0         , M_PI / 2.0    , M_PI / 2.0 };

    void GetArr(vector<float> &arr);
    vector<float> makeHomeArr(int cnt);

    /////////////////////////////////////////////////////////////////////////// 기타

    vector<float> fkfun();

private:
    TMotorCommandParser TParser; ///< T 모터 명령어 파서.
    MaxonCommandParser MParser;  ///< Maxon 모터 명령어 파서

    State &state;                                                 ///< 시스템의 현재 상태입니다.
    CanManager &canManager;                                       ///< CAN 통신을 통한 모터 제어를 담당합니다.
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors; ///< 연결된 모터들의 정보입니다.
    USBIO &usbio;
    Functions &fun;

    map<std::string, int> motor_mapping = { ///< 각 관절에 해당하는 열 정보.
        {"waist", 0},
        {"R_arm1", 1},
        {"L_arm1", 2},
        {"R_arm2", 3},
        {"R_arm3", 4},
        {"L_arm2", 5},
        {"L_arm3", 6},
        {"R_wrist", 7},
        {"L_wrist", 8},
        {"maxonForTest", 8}};


    typedef struct{

        float upperArm = 0.250;         ///< 상완 길이.
        float lowerArm = 0.328;         ///< 하완 길이.
        float stick = 0.325+0.048;      ///< 스틱 길이 + 브라켓 길이.
        float waist = 0.520;            ///< 허리 길이.
        float height = 1.020-0.0605;    ///< 바닥부터 허리까지의 높이.

    }PartLength;

    /////////////////////////////////////////////////////////////////////////// Init
    MatrixXd right_drum_position;                               ///< 오른팔의 각 악기별 위치 좌표 벡터.
    MatrixXd left_drum_position;                                ///< 왼팔의 각 악기별 위치 좌표 벡터.

    MatrixXd right_wrist_hit_angle;                               ///< 오른팔의 각 악기별 타격 시 손목 각도.
    MatrixXd left_wrist_hit_angle;                                ///< 왼팔의 각 악기별 타격 시 손목 각도.

    VectorXd ikfun_final(VectorXd &pR, VectorXd &pL);

    /////////////////////////////////////////////////////////////////////////// Play (read measure)
    float bpm = 0;         /// txt 악보의 BPM 정보.
    double threshold = 2.4;
    double totalTime = 0.0;

    void initVal();
    string trimWhitespace(const std::string &str);

    /////////////////////////////////////////////////////////////////////////// Play (parse measure)
    VectorXd inst_i = VectorXd::Zero(18);   // 전체 궤적에서 출발 악기
    VectorXd inst_f = VectorXd::Zero(18);   // 전체 궤적에서 도착 악기

    float t_i_R, t_f_R;       // 전체 궤적에서 출발 시간, 도착 시간
    float t_i_L, t_f_L;
    float t1, t2;           // 궤적 생성 시간

    MatrixXd measureState = MatrixXd::Zero(2, 3); // [이전 시간, 이전 악기, 상태] // state
                                                                                // 0 : 0 <- 0
                                                                                // 1 : 0 <- 1
                                                                                // 2 : 1 <- 0
                                                                                // 3 : 1 <- 1

    VectorXd hitState = VectorXd::Zero(2);

    void parseMeasure(MatrixXd &measureMatrix);
    pair<VectorXd, VectorXd> parseOneArm(VectorXd t, VectorXd inst, VectorXd stateVector);
    VectorXd makeState(MatrixXd measureMatrix);

    /////////////////////////////////////////////////////////////////////////// Play (make trajectory)
    double round_sum = 0;

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
    int i_solveIK = 0;

    float getLength(double theta);
    double getTheta(float l1, double theta);
    void solveIK(VectorXd &q, double q0);
    VectorXd ikFixedWaist(VectorXd &pR, VectorXd &pL, double theta0, double theta7, double theta8);
    void pushConmmandBuffer(VectorXd &Qi);

    /////////////////////////////////////////////////////////////////////////// Play (waist)
    MatrixXd waistCoefficient;
    double q0_t1;               // 시작 위치 저장
    double q0_t0, t0 = -1;      // 이전 위치, 이전 시간 저장
    vector<double> m;

    // threshold 관련 변수
    double nextq0_t1;
    int status = 0;
    double qthreshold = 0.05;

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

    float makeElbowAngle(float t1, float t2, float t, int state, HitParameter param, int intensity);
    float makeWristAngle(float t1, float t2, float t, int state, HitParameter param, int intensity);

    HitParameter getHitParameter(float t1, float t2, int hitState, HitParameter preParam, int intensity);
    VectorXd makeHitTrajetory(float t1, float t2, float t, int hitState, int wristIntesity);

    HitParameter pre_parameters_R, pre_parameters_L, pre_parameters_tmp;

    float nnR, ntR, nnL, ntL; // 1박 타격 시간이 짧은 경우 2박 시간 사용하기 위한 변수 (nextn, nextt)
    bool readyRflag = 0;
    bool readyLflag = 0; // 미리 시작하는 경우 플래그 
    int next_stateR, next_stateL;
    int next_intensityR, next_intensityL;
    int i_wristR, i_wristL = 0; 

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
    vector<float> c_MotorAngle = {0, 0, 0, 0, 0, 0, 0, 0, 0}; ///< 경로 생성 시 사용되는 현재 모터 위치 값

    // q1[rad], q2[rad], acc[rad/s^2], t2[s]
    VectorXd calVmax(VectorXd &q1, VectorXd &q2, float acc, float t2);
    // q1[rad], q2[rad], Vmax[rad/s], acc[rad/s^2], t[s], t2[s]
    VectorXd makeProfile(VectorXd &q1, VectorXd &q2, VectorXd &Vmax, float acc, float t, float t2);
    void getMotorPos();
    
};