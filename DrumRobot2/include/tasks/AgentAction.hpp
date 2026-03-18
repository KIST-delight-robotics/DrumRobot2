//../include/tasks/AgentAction.hpp
#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <map>
#include <cmath>
#include <cctype>
#include <memory>  // [필수 추가] std::shared_ptr를 쓰려면 이게 꼭 있어야 합니다. -> 전방 선언에 따른 문제 해결용

// [수정] 헤더 충돌 방지를 위한 전방 선언 (Forward Declaration)
class PathManager;
class FlagClass;
class Arduino;
class GenericMotor;

class AgentAction
{
public:
    // [수정] 생성자: PathManager의 motors를 직접 받지 않고, 외부에서 주입받도록 통일
    AgentAction(PathManager& pathRef, FlagClass& flagRef, Arduino& arduinoRef,
                std::map<std::string, std::shared_ptr<GenericMotor>>& motorsRef);
    
    ~AgentAction();

    // [Main Function] LLM에서 받은 문자열 명령을 해석하고 실행하는 함수
    // 예: "[CMD:look:10,20]" -> 고개 돌리기 실행
    void executeCommand(std::string fullCmd);

    // JSON payload 또는 기존 단일 문자열 명령을 공통 포맷으로 풀어낸다.
    std::vector<std::string> unpackCommands(const std::string& payload);

private:
    // --- External References (로봇의 몸체 제어권) ---
    PathManager& pathManager;
    FlagClass& flagObj;
    Arduino& arduino;

    // 모터 제어
    std::map<std::string, std::shared_ptr<GenericMotor>>& motors;

    // --- Internal Logic (명령어 파서) ---
    // 명령어를 ':' 기준으로 분리하는 유틸리티
    std::vector<std::string> split(const std::string& str, char delimiter);
    
    // 명령어 포맷 정리 (대괄호 제거 등)
    std::string cleanCommand(std::string cmd);

    std::string trim(const std::string& value);
    bool parseJsonCommandPayload(const std::string& payload, std::vector<std::string>& commands);
    bool extractJsonStringArray(const std::string& payload, const std::string& key, std::vector<std::string>& values);
    bool extractJsonStringValue(const std::string& payload, const std::string& key, std::string& value);
    bool readJsonString(const std::string& payload, size_t startQuote, std::string& value, size_t& nextPos);

    // --- Action Policies (구체적 행동 정의) ---
    
    // 1. 시선 제어 (Look At)
    // pan: 좌우(-90~90), tilt: 상하(-45~45)
    void policy_lookAt(float pan, float tilt);

    // 2. 제스처 표현 (Body Language)
    // type: "nod"(끄덕임), "shake"(도리도리), "home"(정면)
    void policy_gesture(std::string type);

    // 3. 감정 표현 (LED Expression)
    // emotion: "happy", "thinking", "angry", "idle"
    void policy_emotion(std::string emotion);

    // 4. 자세 변경 (Pose Change)
    // pose: "ready", "home", "shutdown"
    // void policy_pose(std::string pose);

    // 5. 개별 관절 이동 (Joint Move)
    // motorName: 모터 이름, angleDeg: 목표 각도 (도 단위)
    void policy_moveJoint(std::string motorName, float angleDeg);
};
