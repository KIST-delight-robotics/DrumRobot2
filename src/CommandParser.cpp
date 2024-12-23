#include "../include/motors/CommandParser.hpp"

//For Qt
// #include "../motors/CommandParser.hpp"

////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*                                                      Tmotor Servo Mode Parser definition                           */
///////////////////////////////////////////////////////////////////////////////////////////////////////////

enum class CAN_PACKET_ID
{
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_DUTY_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_SET_ORIGIN_HERE,
    CAN_PACKET_SET_POS_SPD
};

std::tuple<int, float, float, float, int8_t, int8_t> TMotorServoCommandParser::motor_receive(struct can_frame *frame)
{
    int id = frame->can_id & 0xFF;
    int16_t pos_int = (frame)->data[0] << 8 | (frame)->data[1];
    int16_t spd_int = (frame)->data[2] << 8 | (frame)->data[3];
    int16_t cur_int = (frame)->data[4] << 8 | (frame)->data[5];

    float pos = (float)(pos_int * 0.1f * M_PI / 180.0f); // Motor Position in radians
    float spd = (float)(spd_int * 10.0f); // Motor Speed in radians per second
    float cur = (float)(cur_int * 0.01f); // Motor Current
    int8_t temp = frame->data[6];         // Motor Temperature
    int8_t error = frame->data[7];        // Motor Error Code

    return std::make_tuple(id, pos, spd, cur, temp, error);
}

void TMotorServoCommandParser::comm_can_set_origin(TMotor &motor, struct can_frame *frame, uint8_t set_origin_mode)
{
    frame->can_id = motor.nodeId |
                    ((uint32_t)CAN_PACKET_ID::CAN_PACKET_SET_ORIGIN_HERE << 8 | CAN_EFF_FLAG);
    frame->can_dlc = 1;
    frame->data[0] = set_origin_mode; 
}

void TMotorServoCommandParser::comm_can_set_pos_spd(TMotor &motor, struct can_frame *frame, float pos, int32_t spd, int32_t RPA)
{
    // 라디안에서 도로 변환
    float pos_deg = pos * (180.0 / M_PI);

    frame->can_id = motor.nodeId |
                    ((uint32_t)CAN_PACKET_ID::CAN_PACKET_SET_POS_SPD << 8 | CAN_EFF_FLAG);
    
    frame->can_dlc=8;
    int32_t pos_int = static_cast<int32_t>(pos_deg * 10000.0);
    frame->data[0] = (pos_int >> 24) & 0xFF;
    frame->data[1] = (pos_int >> 16) & 0xFF;
    frame->data[2] = (pos_int >> 8) & 0xFF;
    frame->data[3] = pos_int & 0xFF;

    // spd_erpm를 CAN 프레임 데이터에 저장
    frame->data[4] = (spd / 10) >> 8;
    frame->data[5] = (spd / 10) & 0xFF;

    // RPA_unit를 CAN 프레임 데이터에 저장
    frame->data[6] = (RPA / 10) >> 8;
    frame->data[7] = (RPA / 10) & 0xFF;
}

void TMotorServoCommandParser::comm_can_set_cb(TMotor &motor, struct can_frame *frame, float current)
{
    frame->can_id = motor.nodeId |
                    ((uint32_t)CAN_PACKET_ID::CAN_PACKET_SET_DUTY_CURRENT_BRAKE << 8 | CAN_EFF_FLAG);
    frame->can_dlc = 4;
    int32_t current_int = static_cast<int32_t>(current * 1000.0);
    frame->data[0] = (current_int >> 24) & 0xFF;
    frame->data[1] = (current_int >> 16) & 0xFF;
    frame->data[2] = (current_int >> 8) & 0xFF;
    frame->data[3] = current_int & 0xFF;
}

void TMotorServoCommandParser::comm_can_set_spd(TMotor &motor, struct can_frame *frame, float spd_erpm)
{
    frame->can_id = motor.nodeId |
                    ((uint32_t)CAN_PACKET_ID::CAN_PACKET_SET_RPM << 8 | CAN_EFF_FLAG);
    frame->can_dlc = 4;

    // 변환된 ERPM 값을 정수로 변환
    int32_t spd_int = static_cast<int32_t>(spd_erpm);
    
    frame->data[0] = (spd_int >> 24) & 0xFF;
    frame->data[1] = (spd_int >> 16) & 0xFF;
    frame->data[2] = (spd_int >> 8) & 0xFF;
    frame->data[3] = spd_int & 0xFF;
}

void TMotorServoCommandParser::comm_can_set_pos(TMotor &motor, struct can_frame *frame, float pos){
    frame->can_id = motor.nodeId |
                    ((uint32_t)CAN_PACKET_ID::CAN_PACKET_SET_POS << 8 | CAN_EFF_FLAG);
    frame->can_dlc = 4;
    // 라디안에서 도로 변환
    float pos_deg = pos * (180.0 / M_PI);

    // 변환된 ERPM 값을 정수로 변환
    int32_t pos_int = static_cast<int32_t>(pos_deg*10000.0);
    
    frame->data[0] = (pos_int >> 24) & 0xFF;
    frame->data[1] = (pos_int >> 16) & 0xFF;
    frame->data[2] = (pos_int >> 8) & 0xFF;
    frame->data[3] = pos_int & 0xFF;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*                                                      Tmotor Parser definition                           */
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void TMotorCommandParser::parseSendCommand(TMotor &motor, struct can_frame *frame, int canId, int dlc, float p_des, float v_des, float kp, float kd, float t_ff)
{
    // 모터 타입에 따른 제한값 설정
    setMotorLimits(motor);

    // 기존 변수를 계산
    p_des = fminf(fmaxf(GLOBAL_P_MIN, p_des), GLOBAL_P_MAX);
    v_des = fminf(fmaxf(GLOBAL_V_MIN, v_des), GLOBAL_V_MAX);
    kp = fminf(fmaxf(GLOBAL_KP_MIN, kp), GLOBAL_KP_MAX);
    kd = fminf(fmaxf(GLOBAL_KD_MIN, kd), GLOBAL_KD_MAX);
    t_ff = fminf(fmaxf(GLOBAL_T_MIN, t_ff), GLOBAL_T_MAX);

    motor.desPos = p_des;
    motor.desVel = v_des;
    motor.desTor = t_ff;

    // 계산된 변수를 이용하여 unsigned int로 변환
    int p_int = floatToUint(p_des, GLOBAL_P_MIN, GLOBAL_P_MAX, 16);
    int v_int = floatToUint(v_des, GLOBAL_V_MIN, GLOBAL_V_MAX, 12);
    int kp_int = floatToUint(kp, GLOBAL_KP_MIN, GLOBAL_KP_MAX, 12);
    int kd_int = floatToUint(kd, GLOBAL_KD_MIN, GLOBAL_KD_MAX, 12);
    int t_int = floatToUint(t_ff, GLOBAL_T_MIN, GLOBAL_T_MAX, 12);
    // Set CAN frame id and data length code
    frame->can_id = canId & CAN_SFF_MASK; // Replace YOUR_CAN_ID with the appropriate id
    frame->can_dlc = dlc;                 // Data Length Code is set to maximum allowed length

    /// pack ints into the can buffer ///
    frame->data[0] = p_int >> 8;                           // Position 8 higher
    frame->data[1] = p_int & 0xFF;                         // Position 8 lower
    frame->data[2] = v_int >> 4;                           // Speed 8 higher
    frame->data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8); // Speed 4 bit lower KP 4bit higher
    frame->data[4] = kp_int & 0xFF;                        // KP 8 bit lower
    frame->data[5] = kd_int >> 4;                          // Kd 8 bit higher
    frame->data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8); // KP 4 bit lower torque 4 bit higher
    frame->data[7] = t_int & 0xff;                         // torque 4 bit lower
}

void TMotorCommandParser::setMotorLimits(TMotor &motor)
{
    if (motor.motorType == "AK10_9")
    {
        GLOBAL_V_MIN = -50;
        GLOBAL_V_MAX = 50;
        GLOBAL_T_MIN = -65;
        GLOBAL_T_MAX = 65;
        GLOBAL_I_MAX = 29.8;
        Kt = 0.198;
    }
    else if (motor.motorType == "AK70_10")
    {
        GLOBAL_V_MIN = -50;
        GLOBAL_V_MAX = 50;
        GLOBAL_T_MIN = -25;
        GLOBAL_T_MAX = 25;
        GLOBAL_I_MAX = 23.2;
        Kt = 0.123;
    }
    else if (motor.motorType == "AK60_6")
    {
        GLOBAL_V_MIN = -45;
        GLOBAL_V_MAX = 45;
        GLOBAL_T_MIN = -15;
        GLOBAL_T_MAX = 15;
        GLOBAL_I_MAX = 23.2;
    }
    else if (motor.motorType == "AK80_6")
    {
        GLOBAL_V_MIN = -76;
        GLOBAL_V_MAX = 76;
        GLOBAL_T_MIN = -12;
        GLOBAL_T_MAX = 12;
        GLOBAL_I_MAX = 23.2;
    }
    else if (motor.motorType == "AK80_9")
    {
        GLOBAL_V_MIN = -50;
        GLOBAL_V_MAX = 50;
        GLOBAL_T_MIN = -18;
        GLOBAL_T_MAX = 18;
    }
    else if (motor.motorType == "AK80_80" || motor.motorType == "AK80_64")
    {
        GLOBAL_V_MIN = -8;
        GLOBAL_V_MAX = 8;
        GLOBAL_T_MIN = -144;
        GLOBAL_T_MAX = 144;
    }
    else if (motor.motorType == "AK80_8")
    {
        GLOBAL_V_MIN = -37.5;
        GLOBAL_V_MAX = 37.5;
        GLOBAL_T_MIN = -32;
        GLOBAL_T_MAX = 32;
    }
    else
    {
        std::cout << "Error: Invalid motor motorType entered!" << std::endl;
    }
}

std::tuple<int, float, float, float> TMotorCommandParser::parseRecieveCommand(TMotor &motor, struct can_frame *frame)
{
    int id;
    float position, speed, torque;
    setMotorLimits(motor);
    /// unpack ints from can buffer ///
    id = frame->data[0];
    int p_int = (frame->data[1] << 8) | frame->data[2];
    int v_int = (frame->data[3] << 4) | (frame->data[4] >> 4);
    int i_int = ((frame->data[4] & 0xF) << 8) | frame->data[5];

    /// convert ints to floats ///
    position = uintToFloat(p_int, GLOBAL_P_MIN, GLOBAL_P_MAX, 16);
    speed = uintToFloat(v_int, GLOBAL_V_MIN, GLOBAL_V_MAX, 12);
    torque = uintToFloat(i_int, -GLOBAL_I_MAX, GLOBAL_I_MAX, 12);

    return std::make_tuple(id, position, speed, torque);
}

int TMotorCommandParser::floatToUint(float x, float x_min, float x_max, unsigned int bits)
{
    float span = x_max - x_min;
    if (x < x_min)
        x = x_min;
    else if (x > x_max)
        x = x_max;
    return (int)((x - x_min) * ((float)((1 << bits) / span)));
};

float TMotorCommandParser::uintToFloat(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void TMotorCommandParser::getCheck(TMotor &motor, struct can_frame *frame)
{
    // Set CAN frame id and data length code
    frame->can_id = motor.nodeId; // Replace YOUR_CAN_ID with the appropriate id
    frame->can_dlc = 8;           // Data Length Code is set to maximum allowed length

    /// pack ints into the can buffer ///
    frame->data[0] = 0x80;
    frame->data[1] = 0x00;
    frame->data[2] = 0x80;
    frame->data[3] = 0x00;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x08;
    frame->data[7] = 0x00;
}

void TMotorCommandParser::getControlMode(TMotor &motor, struct can_frame *frame)
{
    // Set CAN frame id and data length code
    frame->can_id = motor.nodeId; // Replace YOUR_CAN_ID with the appropriate id
    frame->can_dlc = 8;           // Data Length Code is set to maximum allowed length

    /// pack ints into the can buffer ///
    frame->data[0] = 0xFF;
    frame->data[1] = 0xFF;
    frame->data[2] = 0xFF;
    frame->data[3] = 0xFF;
    frame->data[4] = 0xFF;
    frame->data[5] = 0xFF;
    frame->data[6] = 0xFF;
    frame->data[7] = 0xFC;
}

void TMotorCommandParser::getExit(TMotor &motor, struct can_frame *frame)
{
    // Set CAN frame id and data length code
    frame->can_id = motor.nodeId; // Replace YOUR_CAN_ID with the appropriate id
    frame->can_dlc = 8;           // Data Length Code is set to maximum allowed length

    /// pack ints into the can buffer ///
    frame->data[0] = 0xFF;
    frame->data[1] = 0xFF;
    frame->data[2] = 0xFF;
    frame->data[3] = 0xFF;
    frame->data[4] = 0xFF;
    frame->data[5] = 0xFF;
    frame->data[6] = 0xFF;
    frame->data[7] = 0xFD;
}

void TMotorCommandParser::getZero(TMotor &motor, struct can_frame *frame)
{
    // Set CAN frame id and data length code
    frame->can_id = motor.nodeId; // Replace YOUR_CAN_ID with the appropriate id
    frame->can_dlc = 8;           // Data Length Code is set to maximum allowed length

    /// pack ints into the can buffer ///
    frame->data[0] = 0xFF;
    frame->data[1] = 0xFF;
    frame->data[2] = 0xFF;
    frame->data[3] = 0xFF;
    frame->data[4] = 0xFF;
    frame->data[5] = 0xFF;
    frame->data[6] = 0xFF;
    frame->data[7] = 0xFE;
}

void TMotorCommandParser::getQuickStop(TMotor &motor, struct can_frame *frame)
{
    // Set CAN frame id and data length code
    frame->can_id = motor.nodeId; // Replace YOUR_CAN_ID with the appropriate id
    frame->can_dlc = 8;           // Data Length Code is set to maximum allowed length

    /// pack ints into the can buffer ///
    frame->data[0] = 0x80;
    frame->data[1] = 0x00;
    frame->data[2] = 0x80;
    frame->data[3] = 0x00;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x08;
    frame->data[7] = 0x00;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*                                                      Maxon Parser definition                           */
///////////////////////////////////////////////////////////////////////////////////////////////////////////

std::tuple<int, float, float, unsigned char> MaxonCommandParser::parseRecieveCommand(MaxonMotor &motor, struct can_frame *frame)
{
    int id = frame->can_id;

    unsigned char statusBit = frame->data[1];

    int32_t currentPosition = 0;
    currentPosition |= static_cast<uint8_t>(frame->data[2]);
    currentPosition |= static_cast<uint8_t>(frame->data[3]) << 8;
    currentPosition |= static_cast<uint8_t>(frame->data[4]) << 16;
    currentPosition |= static_cast<uint8_t>(frame->data[5]) << 24;

    int16_t torqueActualValue = 0;
    torqueActualValue |= static_cast<uint8_t>(frame->data[6]);
    torqueActualValue |= static_cast<uint8_t>(frame->data[7]) << 8;

    // Motor rated torque 값을 N·m 단위로 변환 (mNm -> N·m)
    const float motorRatedTorquemNm = 31.052; //

    // 실제 토크 값을 N·m 단위로 계산
    // Torque actual value는 천분의 일 단위이므로, 실제 토크 값은 (torqueActualValue / 1000) * motorRatedTorqueNm
    float currentTorqueNm = (static_cast<float>(torqueActualValue) / 1000.0f) * motorRatedTorquemNm;

    float currentPositionDegrees = (static_cast<float>(currentPosition) / (35.0f * 4096.0f)) * 360.0f;
    float currentPositionRadians = currentPositionDegrees * (M_PI / 180.0f);

    return std::make_tuple(id, currentPositionRadians, currentTorqueNm, statusBit);
}

// System
void MaxonCommandParser::getCheck(MaxonMotor &motor, struct can_frame *frame)
{

    frame->can_id = motor.canSendId;
    frame->can_dlc = 8;
    frame->data[0] = 0x00;
    frame->data[1] = 0x00;
    frame->data[2] = 0x00;
    frame->data[3] = 0x00;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void MaxonCommandParser::getStop(MaxonMotor &motor, struct can_frame *frame)
{
    frame->can_id = 0x00;
    frame->can_dlc = 8;
    frame->data[0] = 0x02;
    frame->data[1] = motor.nodeId;
    frame->data[2] = 0x00;
    frame->data[3] = 0x00;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void MaxonCommandParser::getQuickStop(MaxonMotor &motor, struct can_frame *frame)
{
    frame->can_id = motor.txPdoIds[0];
    frame->can_dlc = 8;
    frame->data[0] = 0x06;
    frame->data[1] = 0x00;
    frame->data[2] = 0x00;
    frame->data[3] = 0x00;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void MaxonCommandParser::getOperational(MaxonMotor &motor, struct can_frame *frame)
{
    frame->can_id = 0x00;
    frame->can_dlc = 8;
    frame->data[0] = 0x01;
    frame->data[1] = motor.nodeId;
    frame->data[2] = 0x00;
    frame->data[3] = 0x00;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void MaxonCommandParser::getShutdown(MaxonMotor &motor, struct can_frame *frame)
{
    frame->can_id = motor.txPdoIds[0];
    frame->can_dlc = 8;
    frame->data[0] = 0x06;
    frame->data[1] = 0x00;
    frame->data[2] = 0x00;
    frame->data[3] = 0x00;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void MaxonCommandParser::getEnable(MaxonMotor &motor, struct can_frame *frame)
{
    frame->can_id = motor.txPdoIds[0];
    frame->can_dlc = 8;
    frame->data[0] = 0x0F;
    frame->data[1] = 0x00;
    frame->data[2] = 0x00;
    frame->data[3] = 0x00;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void MaxonCommandParser::getSync(struct can_frame *frame)
{
    frame->can_id = 0x80;
    frame->can_dlc = 0;
}

// CSP
void MaxonCommandParser::getCSPMode(MaxonMotor &motor, struct can_frame *frame)
{
    frame->can_id = motor.canSendId;
    frame->can_dlc = 8;
    frame->data[0] = 0x22;
    frame->data[1] = 0x60;
    frame->data[2] = 0x60;
    frame->data[3] = 0x00;
    frame->data[4] = 0x08;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void MaxonCommandParser::getTorqueOffset(MaxonMotor &motor, struct can_frame *frame)
{
    frame->can_id = motor.canSendId;
    frame->can_dlc = 8;
    frame->data[0] = 0x22;
    frame->data[1] = 0xB2;
    frame->data[2] = 0x60;
    frame->data[3] = 0x00;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void MaxonCommandParser::getPosOffset(MaxonMotor &motor, struct can_frame *frame)
{
    frame->can_id = motor.canSendId;
    frame->can_dlc = 8;
    frame->data[0] = 0x22;
    frame->data[1] = 0xB0;
    frame->data[2] = 0x60;
    frame->data[3] = 0x00;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void MaxonCommandParser::getTargetPosition(MaxonMotor &motor, struct can_frame *frame, float p_des_radians)
{
    // 라디안 값을 인코더 값으로 변환
    float p_des_degrees = p_des_radians * (180.0f / M_PI);                        // 라디안을 도로 변환
    int p_des_enc = static_cast<int>(p_des_degrees * (35.0f * 4096.0f) / 360.0f); // 도를 인코더 값으로 변환

    unsigned char posByte0 = p_des_enc & 0xFF;         // 하위 8비트
    unsigned char posByte1 = (p_des_enc >> 8) & 0xFF;  // 다음 8비트
    unsigned char posByte2 = (p_des_enc >> 16) & 0xFF; // 다음 8비트
    unsigned char posByte3 = (p_des_enc >> 24) & 0xFF; // 최상위 8비트

    // Set CAN frame id and data length code
    frame->can_id = motor.txPdoIds[1];
    frame->can_dlc = 4;

    /// pack ints into the can buffer ///
    frame->data[0] = posByte0;
    frame->data[1] = posByte1;
    frame->data[2] = posByte2;
    frame->data[3] = posByte3;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

// HMM
void MaxonCommandParser::getHomeMode(MaxonMotor &motor, struct can_frame *frame)
{
    frame->can_id = motor.canSendId;
    frame->can_dlc = 8;
    frame->data[0] = 0x22;
    frame->data[1] = 0x60;
    frame->data[2] = 0x60;
    frame->data[3] = 0x00;
    frame->data[4] = 0x06;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void MaxonCommandParser::getFlowingErrorWindow(MaxonMotor &motor, struct can_frame *frame)
{
    frame->can_id = motor.canSendId;
    frame->can_dlc = 8;
    frame->data[0] = 0x22;
    frame->data[1] = 0x65;
    frame->data[2] = 0x60;
    frame->data[3] = 0x00;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void MaxonCommandParser::getHomeoffsetDistance(MaxonMotor &motor, struct can_frame *frame, int degree)
{
    // 1도당 값
    float value_per_degree = 398.22;

    // 입력된 각도에 대한 값을 계산
    int value = static_cast<int>(degree * value_per_degree);

    frame->can_id = motor.canSendId;
    frame->can_dlc = 8;
    frame->data[0] = 0x22;
    frame->data[1] = 0xB1;
    frame->data[2] = 0x30;
    frame->data[3] = 0x00;
    // 계산된 값의 리틀 엔디언 형식으로 분할하여 할당
    frame->data[4] = value & 0xFF;        // 하위 바이트
    frame->data[5] = (value >> 8) & 0xFF; // 상위 바이트
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void MaxonCommandParser::getHomePosition(MaxonMotor &motor, struct can_frame *frame, int degree)
{
    float value_per_degree = 398.22 * motor.cwDir;
    // int 대신 int32_t 사용하여 플랫폼 독립적인 크기 보장
    int32_t value = static_cast<int32_t>(degree * value_per_degree);

    frame->can_id = motor.canSendId;
    frame->can_dlc = 8;
    frame->data[0] = 0x22;
    frame->data[1] = 0xB0;
    frame->data[2] = 0x30;
    frame->data[3] = 0x00;
    // 음수 값을 포함하여 value를 올바르게 바이트로 변환
    frame->data[4] = value & 0xFF;         // 가장 낮은 바이트
    frame->data[5] = (value >> 8) & 0xFF;  // 다음 낮은 바이트
    frame->data[6] = (value >> 16) & 0xFF; // 다음 높은 바이트
    frame->data[7] = (value >> 24) & 0xFF; // 가장 높은 바이트
}

void MaxonCommandParser::getHomingMethodL(MaxonMotor &motor, struct can_frame *frame)
{
    frame->can_id = motor.canSendId;
    frame->can_dlc = 8;
    frame->data[0] = 0x22;
    frame->data[1] = 0x98;
    frame->data[2] = 0x60;
    frame->data[3] = 0x00;
    frame->data[4] = 0x25;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void MaxonCommandParser::getHomingMethodR(MaxonMotor &motor, struct can_frame *frame)
{
    frame->can_id = motor.canSendId;
    frame->can_dlc = 8;
    frame->data[0] = 0x22;
    frame->data[1] = 0x98;
    frame->data[2] = 0x60;
    frame->data[3] = 0x00;
    frame->data[4] = 0x25;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void MaxonCommandParser::getHomingMethodTest(MaxonMotor &motor, struct can_frame *frame)
{
    frame->can_id = motor.canSendId;
    frame->can_dlc = 8;
    frame->data[0] = 0x22;
    frame->data[1] = 0x98;
    frame->data[2] = 0x60;
    frame->data[3] = 0x00;
    frame->data[4] = 0xFC;
    frame->data[5] = 0xFF;
    frame->data[6] = 0xFF;
    frame->data[7] = 0xFF;
}

void MaxonCommandParser::getStartHoming(MaxonMotor &motor, struct can_frame *frame)
{
    frame->can_id = motor.txPdoIds[0];
    frame->can_dlc = 8;
    frame->data[0] = 0x1F; // 00011111
    frame->data[1] = 0x00;
    frame->data[2] = 0x00;
    frame->data[3] = 0x00;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void MaxonCommandParser::getCurrentThresholdR(MaxonMotor &motor, struct can_frame *frame)
{
    // 1000 = 3E8
    // 500 = 01F4
    frame->can_id = motor.canSendId;
    frame->can_dlc = 8;
    frame->data[0] = 0x23;
    frame->data[1] = 0xB2;
    frame->data[2] = 0x30;
    frame->data[3] = 0x00;
    frame->data[4] = 0xF4;
    frame->data[5] = 0x01;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void MaxonCommandParser::getCurrentThresholdL(MaxonMotor &motor, struct can_frame *frame)
{
    // 1000 =03E8
    // 500 = 01F4
    frame->can_id = motor.canSendId;
    frame->can_dlc = 8;
    frame->data[0] = 0x23;
    frame->data[1] = 0xB2;
    frame->data[2] = 0x30;
    frame->data[3] = 0x00;
    frame->data[4] = 0xF4;
    frame->data[5] = 0x01;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

// CSV
void MaxonCommandParser::getCSVMode(MaxonMotor &motor, struct can_frame *frame)
{
    frame->can_id = motor.canSendId;
    frame->can_dlc = 8;
    frame->data[0] = 0x22;
    frame->data[1] = 0x60;
    frame->data[2] = 0x60;
    frame->data[3] = 0x00;
    frame->data[4] = 0x09;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void MaxonCommandParser::getVelOffset(MaxonMotor &motor, struct can_frame *frame)
{
    frame->can_id = motor.canSendId;
    frame->can_dlc = 8;
    frame->data[0] = 0x22;
    frame->data[1] = 0xB1;
    frame->data[2] = 0x60;
    frame->data[3] = 0x00;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void MaxonCommandParser::getTargetVelocity(MaxonMotor &motor, struct can_frame *frame, int targetVelocity)
{
    unsigned char velByte0 = targetVelocity & 0xFF;         // 하위 8비트
    unsigned char velByte1 = (targetVelocity >> 8) & 0xFF;  // 다음 8비트
    unsigned char velByte2 = (targetVelocity >> 16) & 0xFF; // 다음 8비트
    unsigned char velByte3 = (targetVelocity >> 24) & 0xFF; // 최상위 8비트

    // Set CAN frame id and data length code
    frame->can_id = motor.txPdoIds[2];
    frame->can_dlc = 4;

    /// pack ints into the can buffer ///
    frame->data[0] = velByte0;
    frame->data[1] = velByte1;
    frame->data[2] = velByte2;
    frame->data[3] = velByte3;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

// CST
void MaxonCommandParser::getCSTMode(MaxonMotor &motor, struct can_frame *frame)
{
    frame->can_id = motor.canSendId;
    frame->can_dlc = 8;
    frame->data[0] = 0x22;
    frame->data[1] = 0x60;
    frame->data[2] = 0x60;
    frame->data[3] = 0x00;
    frame->data[4] = 0x0A;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}

void MaxonCommandParser::getTargetTorque(MaxonMotor &motor, struct can_frame *frame, int targetTorque)
{
    unsigned char torByte0 = targetTorque & 0xFF;        // 하위 8비트
    unsigned char torByte1 = (targetTorque >> 8) & 0xFF; // 다음 8비트

    // Set CAN frame id and data length code
    frame->can_id = motor.txPdoIds[3];
    frame->can_dlc = 2;

    /// pack ints into the can buffer ///
    frame->data[0] = torByte0;
    frame->data[1] = torByte1;
    frame->data[2] = 0x00;
    frame->data[3] = 0x00;
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0x00;
}
