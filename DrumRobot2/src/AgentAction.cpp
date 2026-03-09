//AgentAction.cpp
#include "../include/tasks/AgentAction.hpp"

// 기존 매니저들 참조를 위해 포함
#include "../include/managers/PathManager.hpp"
#include "../include/tasks/DrumRobot.hpp" // FlagClass, Arduino 등 타입 참조용
#include "../include/motors/Motor.hpp" // 모터 제어를 위해 필요

// 생성자: 로봇 제어 모듈들의 참조를 저장
AgentAction::AgentAction(PathManager& pathRef,
                        FlagClass& flagRef,
                        Arduino& arduinoRef,
                        std::map<std::string, std::shared_ptr<GenericMotor>>& motorsRef)
    : pathManager(pathRef),
    flagObj(flagRef),
    arduino(arduinoRef),
    motors(motorsRef)
{
    std::cout << "[AgentAction] Policy Module Initialized." << std::endl;
}

AgentAction::~AgentAction() {}

// =============================================================
// [핵심] 명령어 해석기 (Command Dispatcher)
// =============================================================
void AgentAction::executeCommand(std::string fullCmd)
{
    // 1. 전처리: "[CMD:look:10,10]" -> "look:10,10"
    std::string cleanCmd = cleanCommand(fullCmd);
    
    // 2. 분리: "look:10,10" -> ["look", "10,10"]
    std::vector<std::string> tokens = split(cleanCmd, ':');
    if (tokens.empty()) return;

    std::string action = tokens[0]; // 메인 행동 키워드

    // 3. 정책 매핑 (Routing)
    // (1) ex)시선 제어: look:pan,tilt(수평, 수직)-> home: 0,90 (정면)
    if (action == "look" && tokens.size() >= 2) 
    {
        std::vector<std::string> angles = split(tokens[1], ',');
        if (angles.size() == 2) {
            try {
                float pan = std::stof(angles[0]);
                float tilt = std::stof(angles[1]);
                policy_lookAt(pan, tilt);
            } catch (...) { std::cerr << "[Agent] Look parsing error\n"; }
        }
    }
    // (2) ex)제스처: gesture:nod
    else if (action == "gesture" && tokens.size() >= 2)
    {
        policy_gesture(tokens[1]);
    }
    // (3) ex)감정표현: led:happy
    else if (action == "led" && tokens.size() >= 2)
    {
        policy_emotion(tokens[1]);
    }
    /*
    // (4) ex)자세변경: pose:ready
    else if (action == "pose" && tokens.size() >= 2)
    {
        policy_pose(tokens[1]);
    }
    */
    // (5) ex)개별 관절 이동: move:R_arm1,45
    else if (action == "move" && tokens.size() >= 2)
    {
        // tokens[1]은 "R_arm1,45" 형태
        std::vector<std::string> params = split(tokens[1], ',');
        if (params.size() == 2)
        {
            std::string motorName = params[0];
            try {
                float angle = std::stof(params[1]);
                policy_moveJoint(motorName, angle);
            } catch (...) { std::cerr << "[Agent] Move parsing error\n"; }
        }
    }
    else 
    {
        std::cout << "[Agent] Unknown Action: " << action << std::endl;
    }
}

// =============================================================
// Action Policies (행동 구체화)
// =============================================================

void AgentAction::policy_lookAt(float pan, float tilt)
{
    std::cout << "[Action] Look At -> Pan:" << pan << ", Tilt:" << tilt << std::endl;

    // DXL 모터 제어 (ID 12: Pan, ID 13: Tilt라고 가정)
    // PathManager의 dxlCommandBuffer를 활용
    float radPan = pan * M_PI / 180.0f;
    float radTilt = tilt * M_PI / 180.0f;
    float moveTime = 1.0f; // 1초 동안 이동

    // DXL 명령 생성: {{time, totalTime, pos}, ...}
    // DXL ID 12(index 0), 13(index 1)에 해당한다고 가정 (Robot 구조에 따라 확인 필요)
    std::vector<std::vector<float>> dxlCmd;
    dxlCmd.push_back({moveTime, moveTime, radPan});  // DXL 1
    dxlCmd.push_back({moveTime, moveTime, radTilt}); // DXL 2

    // PathManager 큐에 삽입 (Thread Safe 여부 확인 필요, 보통 큐는 안전)
    pathManager.dxlCommandBuffer.push(dxlCmd);
}

void AgentAction::policy_gesture(std::string type)
{
    std::cout << "[Action] Perform Gesture: " << type << std::endl;

    // 1. 인사 (Wave) - [추가됨] 어깨를 들고 팔꿈치를 흔듦
    if (type == "wave" || type == "hi")
    {
        // (1) 시선 처리: 움직이는 오른손을 바라봄 (자연스러움 유도)
        policy_lookAt(30, 100); // 오른쪽 아래를 바라보도록

        // (2) 안전을 위해 손목 살짝 들기 (드럼 충돌 방지)
        policy_moveJoint("R_wrist", 0.0);

        // (3) 팔 바깥으로 돌리기 (R_arm1: 어깨 roll 모터, Max 150도)
        policy_moveJoint("R_arm1", 30.0);

        // (4) 팔 위로 들기 (R_arm2: 어깨 pitch 모터, Max 90도)
        policy_moveJoint("R_arm2", 70.0); 
        
        // (5) 팔꿈치와 손목 굽혔다 폈다 반복 (R_arm3, R_wrist)
        // moveJoint 명령은 큐(Queue)에 쌓이므로 순차적으로 실행됩니다.
        // 1. 팔꿈치, 손목 굽히기
        policy_moveJoint("R_arm3", 75.0); // 안쪽으로 굽힘
        policy_moveJoint("R_wrist", 45.0); // 손목도 함께 굽힘
        // 2. 팔꿈치, 손목 펴기
        policy_moveJoint("R_arm3", 45.0); // 바깥쪽으로 폄
        policy_moveJoint("R_wrist", 0.0); // 손목 원위치
        // 3. 팔꿈치, 손목 굽히기
        policy_moveJoint("R_arm3", 75.0); // 다시 굽힘
        policy_moveJoint("R_wrist", 45.0); // 손목도 함께 굽힘
        // 4. 팔꿈치, 손목 펴기 (마무리)
        policy_moveJoint("R_arm3", 45.0); // 다시 폄
        policy_moveJoint("R_wrist", 0.0); // 손목 원위치
    }
    // 2. 끄덕임 (Nod) - DXL 모터 사용
    else if (type == "nod"){
        // 끄덕임: 아래 -> 위 -> 정면
        policy_lookAt(0, 110); // 아래
        // 연속 동작은 큐에 순차적으로 넣으면 됨 (PathManager 구조상)
        // 실제로는 시간 지연이 필요할 수 있음. 
        // V1에서는 단순하게 구현.
        policy_lookAt(0, 90);  // 정면
    }
    // 3. 도리도리 (Shake) - 좌우로 고개 흔들기
    else if (type == "shake"){
        policy_lookAt(-30, 0);
        policy_lookAt(30, 0);
        policy_lookAt(0, 0);
    }
    // 4. 환호 (Hurray) - 양팔 만세
    else if (type == "hurray" || type == "happy") {
        // 양쪽 팔 들어 올림
        policy_moveJoint("R_arm2", 70.0);
        policy_moveJoint("L_arm2", 70.0);
        
        // 양쪽 팔꿈치 펴기
        policy_moveJoint("R_arm3", 15.0);
        policy_moveJoint("L_arm3", 15.0);
        
        // 고개 들기 (DXL)
        policy_lookAt(0, 70); 
    }
}

void AgentAction::policy_emotion(std::string emotion)
{
    std::cout << "[Action] Express Emotion: " << emotion << std::endl;

    if (emotion == "happy" || emotion == "play") {
        arduino.setHeadLED(Arduino::PLAYING); // 화려한 색
    }
    else if (emotion == "thinking" || emotion == "idle"|| emotion == "angry") {
        arduino.setHeadLED(Arduino::IDLE); // 대기 색
    }
    else if (emotion == "off") {
        arduino.setHeadLED(Arduino::POWER_OFF);
    }
    else {
        arduino.setHeadLED(Arduino::POWER_ON);
    }
}

/*
void AgentAction::policy_pose(std::string pose)
{
    std::cout << "[Action] Set Pose: " << pose << std::endl;

    if (pose == "ready") {
        flagObj.setAddStanceFlag(FlagClass::READY);
        // 상태 변경 트리거는 DrumRobot 메인 루프에서 처리되므로 플래그만 세팅
    }
    else if (pose == "home") {
        flagObj.setAddStanceFlag(FlagClass::HOME);
    }
    else if (pose == "shutdown") {
        flagObj.setAddStanceFlag(FlagClass::SHUTDOWN);
    }
}
*/

void AgentAction::policy_moveJoint(std::string motorName, float angleDeg)
{    
    // 1. 모터 이름 확인
    if (motors.find(motorName) == motors.end()) {
        std::cerr << "[Agent] Unknown Motor: " << motorName << std::endl;
        return;
    }
    std::cout << "[Action] Move " << motorName << " to " << angleDeg << " deg" << std::endl;

    auto motor = motors[motorName];
    float targetRad = angleDeg * M_PI / 180.0f;
    float move_time = 2.0f;
    float dt = 0.005f;      // 5ms 주기 (CAN 통신 주기)

    // 2. 모터 타입별 명령 생성 (GenericMotor -> TMotor/MaxonMotor 캐스팅)
    // T-Motor 처리
    if (auto tMotor = std::dynamic_pointer_cast<TMotor>(motor)) 
    {
        // [안전장치] 각도 제한 확인
        if (targetRad < tMotor->rMin || targetRad > tMotor->rMax) {
            std::cerr << "[Warning] Joint Limit Exceeded (T-Motor): " << motorName << " Curr: " << angleDeg << " Limit: " << tMotor->rMin << "~" << tMotor->rMax << std::endl;
            return;
        }

        // [보간 준비] 시작점과 목표점 계산 (Radian 단위)
        float startRad = tMotor->motorPositionToJointAngle(tMotor->finalMotorPosition); // 현재 위치를 기준으로 보간 시작
        
        std::lock_guard<std::mutex> lock(tMotor->bufferMutex); // T-Motor 전용 자물쇠 

        for (float t = dt; t <= move_time; t += dt) {
            
            // 사인 보간 공식: current = (target - start) / 2 * cos(pi * t / T + pi) + (target + start) / 2
            float current_rad = ((targetRad - startRad) / 2.0f) * cos(M_PI * (t / move_time + 1.0f)) + ((targetRad + startRad) / 2.0f);

            // T-Motor 명령 생성
            TMotorData newData;
            newData.position = tMotor->jointAngleToMotorPosition(current_rad);
            newData.mode = tMotor->Position;
            newData.velocityERPM = 0;
            newData.useBrake = 0;

            // T-Motor 명령 버퍼에 추가
            tMotor->commandBuffer.push(newData);
            tMotor->finalMotorPosition = newData.position; // 최종 위치 업데이트 (다음 명령의 시작점이 됨)
        }
    }

    // Maxon 모터 처리
    else if (auto maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
    {
        // [안전장치] 각도 제한 확인
        if (targetRad < maxonMotor->rMin || targetRad > maxonMotor->rMax) {
            std::cerr << "[Warning] Joint Limit Exceeded (Maxon): " << motorName << " Curr: " << angleDeg << " Limit: " << maxonMotor->rMin << "~" << maxonMotor->rMax << std::endl;
            return;
        }

        // [보간 준비] 시작점과 목표점 계산
        // finalMotorPosition은 이전에 마지막으로 명령 내린 위치입니다.
        float startRad = maxonMotor->motorPositionToJointAngle(maxonMotor->finalMotorPosition); // 현재 위치를 기준으로 보간 시작

        std::lock_guard<std::mutex> lock(maxonMotor->bufferMutex);

        for (float t = dt; t <= move_time; t += dt) {
            
            // 사인 보간 공식: start + (target - start) * (1 - cos(pi * t / T)) / 2
            float current_rad = startRad + (targetRad - startRad) * (1 - cos(M_PI * t / move_time)) / 2.0f;

            // Maxon 모터 전용 1ms 5단계 쪼개기
            for (int i = 0; i < 5; i++) { 
                
                MaxonData newData;
                newData.position = maxonMotor->jointAngleToMotorPosition(current_rad);
                newData.mode = maxonMotor->CSP;
                newData.kp = 0;
                newData.kd = 0;

                maxonMotor->commandBuffer.push(newData);
                maxonMotor->finalMotorPosition = newData.position; // 최종 위치 업데이트 (다음 명령의 시작점이 됨)
            }
        }
    }
    else 
    {
        std::cerr << "[Agent] Unsupported Motor Type for: " << motorName << std::endl;
    }

}

// =============================================================
// Utilities
// =============================================================

std::vector<std::string> AgentAction::split(const std::string& str, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(str);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

std::string AgentAction::cleanCommand(std::string cmd)
{
    // "[CMD:look...]" -> "look..."
    size_t start = cmd.find("CMD:");
    if (start != std::string::npos) {
        cmd = cmd.substr(start + 4);
    }
    // 닫는 대괄호 제거
    size_t end = cmd.find("]");
    if (end != std::string::npos) {
        cmd = cmd.substr(0, end);
    }
    return cmd;
}