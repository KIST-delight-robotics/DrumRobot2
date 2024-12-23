#ifndef COMMANDPARSER_H
#define COMMANDPARSER_H

#include "Motor.hpp"
#include <linux/can.h>
#include <cmath>
#include <tuple>
#include <iostream>

using namespace std;

class TMotorServoCommandParser
{

public:
    // 0:tempoaray origin  1: setting a permanent zero point
    void comm_can_set_origin(TMotor &motor, struct can_frame *frame, uint8_t set_origin_mode);
    
    // pos [rad], speed[erpm], acceleration [erpm/s^2]
    void comm_can_set_pos_spd(TMotor &motor, struct can_frame *frame, float pos, int32_t spd, int32_t RPA);

    // id, position, speed, current, temperature, error
    std::tuple<int, float, float, float, int8_t, int8_t> motor_receive(struct can_frame *frame);
    // current break
    void comm_can_set_cb(TMotor &motor, struct can_frame *frame, float current);
    // speed [erpm]
    void comm_can_set_spd(TMotor &motor, struct can_frame *frame, float spd_erpm);
    // pos [rad]
    void comm_can_set_pos(TMotor &motor, struct can_frame *frame, float pos);

private:
};

class TMotorCommandParser
{
public:
    float GLOBAL_P_MIN = -12.5;
    float GLOBAL_P_MAX = 12.5;
    float GLOBAL_KP_MIN = 0;
    float GLOBAL_KP_MAX = 500;
    float GLOBAL_KD_MIN = 0;
    float GLOBAL_KD_MAX = 5;
    float GLOBAL_V_MIN, GLOBAL_V_MAX, GLOBAL_T_MIN, GLOBAL_T_MAX;
    float GLOBAL_I_MAX, Kt;

    void parseSendCommand(TMotor &motor, struct can_frame *frame, int canId, int dlc, float p_des, float v_des, float kp, float kd, float t_ff);

    std::tuple<int, float, float, float> parseRecieveCommand(TMotor &motor, struct can_frame *frame);

    void getCheck(TMotor &motor, struct can_frame *frame);
    void getControlMode(TMotor &motor, struct can_frame *frame);
    void getExit(TMotor &motor, struct can_frame *frame);
    void getZero(TMotor &motor, struct can_frame *frame);
    void getQuickStop(TMotor &motor, struct can_frame *frame);

private:
    int floatToUint(float x, float x_min, float x_max, unsigned int bits);
    float uintToFloat(int x_int, float x_min, float x_max, int bits);

    void setMotorLimits(TMotor &motor);
};

class MaxonCommandParser
{
public:
    std::tuple<int, float, float, unsigned char> parseRecieveCommand(MaxonMotor &motor, struct can_frame *frame);

    void getCheck(MaxonMotor &motor, struct can_frame *frame);
    void getStop(MaxonMotor &motor, struct can_frame *frame);
    void getQuickStop(MaxonMotor &motor, struct can_frame *frame);
    void getOperational(MaxonMotor &motor, struct can_frame *frame);
    void getShutdown(MaxonMotor &motor, struct can_frame *frame);
    void getEnable(MaxonMotor &motor, struct can_frame *frame);
    void getSync(struct can_frame *frame);

    // CSP 모드 관련 명령 구성 메서드.
    void getCSPMode(MaxonMotor &motor, struct can_frame *frame);
    void getTorqueOffset(MaxonMotor &motor, struct can_frame *frame);
    void getPosOffset(MaxonMotor &motor, struct can_frame *frame);
    void getTargetPosition(MaxonMotor &motor, struct can_frame *frame, float p_des_radians);

    // HMM 모드 관련 명령 구성 메서드.
    void getHomeMode(MaxonMotor &motor, struct can_frame *frame);
    void getFlowingErrorWindow(MaxonMotor &motor, struct can_frame *frame);
    void getHomeoffsetDistance(MaxonMotor &motor, struct can_frame *frame, int degree);
    void getHomePosition(MaxonMotor &motor, struct can_frame *frame, int degree);
    void getHomingMethodL(MaxonMotor &motor, struct can_frame *frame);
    void getHomingMethodR(MaxonMotor &motor, struct can_frame *frame);
    void getHomingMethodTest(MaxonMotor &motor, struct can_frame *frame);
    void getStartHoming(MaxonMotor &motor, struct can_frame *frame);
    void getCurrentThresholdR(MaxonMotor &motor, struct can_frame *frame);
    void getCurrentThresholdL(MaxonMotor &motor, struct can_frame *frame);

    // CSV 모드 관련 명령 구성 메서드.
    void getCSVMode(MaxonMotor &motor, struct can_frame *frame);
    void getVelOffset(MaxonMotor &motor, struct can_frame *frame);
    void getTargetVelocity(MaxonMotor &motor, struct can_frame *frame, int targetVelocity);

    // CST 모드 관련 명령 구성 메서드.
    void getCSTMode(MaxonMotor &motor, struct can_frame *frame);
    void getTargetTorque(MaxonMotor &motor, struct can_frame *frame, int targetTorque);
};

#endif // COMMANDPARSER_H
