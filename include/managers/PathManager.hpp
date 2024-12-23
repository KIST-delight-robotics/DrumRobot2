#pragma once

#include <stdio.h>
#include "../include/managers/CanManager.hpp"
#include "../include/motors/CommandParser.hpp"
#include "../include/motors/Motor.hpp"
#include "../include/tasks/SystemState.hpp"
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
                Functions &funRef);

    
    /////////////////////////////////////////////////////////////////////////// Init

    void GetDrumPositoin();
    void SetReadyAngle();
    void InitVal();
    
    /////////////////////////////////////////////////////////////////////////// Play
    
    bool readMeasure(ifstream& inputFile, bool &BPMFlag, double &timeSum);
    void parseMeasure(double &timeSum);

    bool readMeasure___(ifstream& inputFile, bool &BPMFlag);

    void generateTrajectory();
    void generateTrajectory___();
    void solveIK();

    // 브레이크 상태 저장할 구조체
    typedef struct {
        // 브레이크
        bool state[8];
    }Brake;

    queue<Brake> brake_buffer;
    queue<vector<string>> Q; // 읽은 악보 저장한 큐
    int line = 0;  ///< 연주를 진행하고 있는 줄. 필요 없음
    MatrixXd measureMatrix;

    /////////////////////////////////////////////////////////////////////////// AddStance

    void GetArr(vector<float> &arr);

    vector<float> makeHomeArr(int cnt);

    //   Ready Pos Array   :  waist         , R_arm1        , L_arm1        , R_arm2        , R_arm3        , L_arm2        , L_arm3        , R_wrist       , L_wrist
    //                       { 0            , 90            , 90            , 45            , 75            , 45            , 75            , 30            , 30         } [deg]
    vector<float> readyArr = { 0            , M_PI / 2.0    , M_PI / 2.0    , M_PI * 0.25   , M_PI / 2.4    , M_PI * 0.25   , M_PI / 2.4    , M_PI / 6.0    , M_PI / 6.0 };

    //   Home Pos Array    : waist          , R_arm1        , L_arm1        , R_arm2    , R_arm3            , L_arm2    , L_arm3            , R_wrist               , L_wrist
    //                      { 10            , 90            , 90            , 0         , 120               , 0         , 120               , 95                    , 95                    } [deg]
    vector<float> homeArr = { M_PI / 18.0   , M_PI / 2.0    , M_PI / 2.0    , 0         , M_PI * (2.0/3.0)  , 0         , M_PI * (2.0/3.0)  , M_PI * (95.0/180.0)   , M_PI * (95.0/180.0)   };

    //   Back Pos Array    : waist      , R_arm1        , L_arm1        , R_arm2    , R_arm3    , L_arm2    , L_arm3    , R_wrist       , L_wrist
    //                      { 0         , 135           , 45            , 0         , 0         , 0         , 0         , 90            , 90         } [deg]
    vector<float> backArr = { 0         ,M_PI * 0.75    , M_PI * 0.25   , 0         , 0         , 0         , 0         , M_PI / 2.0    , M_PI / 2.0 };

    /////////////////////////////////////////////////////////////////////////// 기타

    VectorXd ikfun_final(VectorXd &pR, VectorXd &pL);
    vector<float> fkfun();

private:
    TMotorCommandParser TParser; ///< T 모터 명령어 파서.
    MaxonCommandParser MParser;  ///< Maxon 모터 명령어 파서

    State &state;                                                 ///< 시스템의 현재 상태입니다.
    CanManager &canManager;                                       ///< CAN 통신을 통한 모터 제어를 담당합니다.
    std::map<std::string, std::shared_ptr<GenericMotor>> &motors; ///< 연결된 모터들의 정보입니다.
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
    
    VectorXd default_right; /// 오른팔 시작 위치
    VectorXd default_left;  /// 왼팔 시작 위치
    MatrixXd right_drum_position;                               ///< 오른팔의 각 악기별 위치 좌표 벡터.
    MatrixXd left_drum_position;                                ///< 왼팔의 각 악기별 위치 좌표 벡터.

    /////////////////////////////////////////////////////////////////////////// Read & Parse Measure
    string trimWhitespace(const std::string &str);

    float bpm = 0;         /// txt 악보의 BPM 정보.

    double round_sum = 0;
    double threshold = 2.4;
    double total_time = 0.0;
    double detect_time_R = 0;
    double detect_time_L = 0;
    double current_time = 0;
    double moving_start_R = 0;
    double moving_start_L = 0;
    int line_n = 0; 
    vector<string> prev_col = { "0","0","1","1","0","0","0","0" };  // default 악기 위치로 맞추기

    double totalTime = 0.0;

    /////////////////////////////////////////////////////////////////////////// Play (make trajectory)
    // x, y, z 저장할 구조체
    typedef struct {
        // 오른팔 좌표
        VectorXd pR; // 0: x, 1: y, 2: z
        // 왼팔 좌표
        VectorXd pL; // 0: x, 1: y, 2: z
    }Position;

    // 허리 각도와 손목, 팔꿈치 각도 저장할 구조체
    typedef struct {
        // 허리 각도
        float q0;
        // 손목 각도, 팔꿈치 추가 각도
        VectorXd add_qR;
        VectorXd add_qL;
    }AddAngle;

    queue<Position> P_buffer;
    queue<AddAngle> q_buffer;

    void getInstrument();
    VectorXd getTargetPosition(VectorXd &inst_vector);
    float timeScaling(float ti, float tf, float t);
    VectorXd makePath(VectorXd Pi, VectorXd Pf, float s);

    VectorXd pre_inst_R = VectorXd::Zero(9);
    VectorXd pre_inst_L = VectorXd::Zero(9);      /// 연주 중 현재 위치하는 악기 저장

    VectorXd inst_i = VectorXd::Zero(18);   // 전체 궤적에서 출발 악기
    VectorXd inst_f = VectorXd::Zero(18);   // 전체 궤적에서 도착 악기

    float t_i_R, t_f_R;       // 전체 궤적에서 출발 시간, 도착 시간
    float t_i_L, t_f_L;
    float t1, t2;           // 궤적 생성 시간

    void parseMeasure___(MatrixXd &measureMatrix);
    pair<VectorXd, VectorXd> parseOneArm___(VectorXd t, VectorXd inst, VectorXd stateVector);

    // state
    // 0 : 0 <- 0
    // 1 : 0 <- 1
    // 2 : 1 <- 0
    // 3 : 1 <- 1

    MatrixXd state___ = MatrixXd::Zero(2, 3); // [이전 시간, 이전 악기, 상태]
    

    // 타격 궤적 생성 파라미터
    typedef struct {

        float wristStayAngle = 10.0 * M_PI / 180.0;
        float wristContactAngle = -5.0 * M_PI / 180.0;
        float wristLiftAngle = 25.0 * M_PI / 180.0;

        float elbowStayAngle = 3.0 * M_PI / 180.0;
        float elbowLiftAngle = 6.0 * M_PI / 180.0;

    }HitParameter;

    VectorXd makeHitTrajetory(float t1, float t2, float t, VectorXd hitState, HitParameter param);
    float makeElbowAngle(float t1, float t2, float t, int state, HitParameter param);
    float makeWristAngle(float t1, float t2, float t, int state, HitParameter param);
    
    VectorXd hit_state_R = VectorXd::Zero(2);
    VectorXd hit_state_L = VectorXd::Zero(2);

    /////////////////////////////////////////////////////////////////////////// Play (solve IK)
    VectorXd ikFixedWaist(VectorXd &pR, VectorXd &pL, float theta0);
    void pushConmmandBuffer(VectorXd &Qi);

    /////////////////////////////////////////////////////////////////////////// AddStance
    // q1[rad], q2[rad], acc[rad/s^2], t2[s]
    VectorXd calVmax(VectorXd &q1, VectorXd &q2, float acc, float t2);
    // q1[rad], q2[rad], Vmax[rad/s], acc[rad/s^2], t[s], t2[s]
    VectorXd makeProfile(VectorXd &q1, VectorXd &q2, VectorXd &Vmax, float acc, float t, float t2);
    void getMotorPos();

    vector<float> c_MotorAngle = {0, 0, 0, 0, 0, 0, 0, 0, 0}; ///< 경로 생성 시 사용되는 현재 모터 위치 값
    
};