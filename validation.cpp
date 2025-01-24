#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <string>
#include <fstream>
#include <sstream>

// 드럼 위치를 나타내는 구조체
struct Position {
    double x, y, z;
};

// 드럼 키트의 위치 매핑
std::unordered_map<int, Position> drumPositions = {
    {0, {0, 0, 0}},    // 연주하지 않음
    {1, {0, 1, 0.5}},    // 스네어
    {2, {1, 0, 0.4}},    // 플로어 탐
    {3, {0.5, 1.5, 0.7}},// 미드 탐
    {4, {0.5, 2, 0.8}},  // 하이 탐
    {5, {-0.5, 2, 1.0}}, // 하이햇
    {6, {-1, 2, 1.2}},   // 라이드 벨
    {7, {-1.5, 1.5, 1.1}}, // 라이트 크래쉬
    {8, {-1.5, 0.5, 0.9}} // 레프트 크래쉬
};

// 로봇 팔의 제약 조건
const double armLength = 1.5;          // 로봇 팔 최대 길이 (m)
const double minSafeDistance = 0.3;   // 두 손의 최소 안전 거리 (m)

// 안전성 검사 함수
std::pair<bool, std::string> checkSafety(int rightPos, int leftPos) {
    // 오른손과 왼손의 위치 좌표 가져오기
    Position rightHand = drumPositions[rightPos];
    Position leftHand = drumPositions[leftPos];

    // 1) 팔 길이 제한 확인
    double rightDistance = std::sqrt(rightHand.x * rightHand.x + rightHand.y * rightHand.y + rightHand.z * rightHand.z);
    double leftDistance = std::sqrt(leftHand.x * leftHand.x + leftHand.y * leftHand.y + leftHand.z * leftHand.z);

    if (rightDistance > armLength || leftDistance > armLength) {
        return {false, "팔 길이 초과"};
    }

    // 2) 두 손의 충돌 위험 확인
    double distanceBetweenHands = std::sqrt(std::pow(rightHand.x - leftHand.x, 2) +
                                            std::pow(rightHand.y - leftHand.y, 2) +
                                            std::pow(rightHand.z - leftHand.z, 2));

    if (distanceBetweenHands < minSafeDistance) {
        return {false, "충돌 위험: 손 간 거리 " + std::to_string(distanceBetweenHands) + "m"};
    }

    // 안전한 경우
    return {true, "안전한 동작"};
}

// 특정 조건 검사 함수
bool checkSpecialCondition(int currentRightPos, int currentLeftPos, const std::vector<int>& nextLeftPositions) {
    // 오른손이 하이 탐(4)을 치고 왼손이 스네어(1)을 친 후, 다음 왼손이 플로어 탐(2)이나 라이드(6)을 치려는 경우
    if (currentRightPos == 4 && currentLeftPos == 1) {
        for (int nextLeftPos : nextLeftPositions) {
            if (nextLeftPos == 2 || nextLeftPos == 6) {
                return true;
            }
        }
    }
    return false;
}

// 드럼 악보 데이터에서 충돌 검사
void checkDrumScore(const std::vector<std::tuple<int, double, int, int>>& drumData) {
    bool allSafe = true;

    for (size_t i = 0; i < drumData.size(); ++i) {
        int measure, nextMeasure = 0;
        double time, nextTime = 0;
        int rightPos, leftPos;

        // 현재 데이터 추출
        std::tie(measure, time, rightPos, leftPos) = drumData[i];

        // 다음, 다다음, 다다다음 왼손 위치 추출
        std::vector<int> nextLeftPositions;
        for (size_t j = 1; j <= 3; ++j) {
            if (i + j < drumData.size()) {
                int tempMeasure, tempRightPos, tempLeftPos;
                double tempTime;
                std::tie(tempMeasure, tempTime, tempRightPos, tempLeftPos) = drumData[i + j];
                nextLeftPositions.push_back(tempLeftPos);
            }
        }

        // 안전성 검사 수행
        auto [isSafe, message] = checkSafety(rightPos, leftPos);

        if (!isSafe) {
            allSafe = false;
            std::cout << "문제 발생: Measure " << measure << ", Time " << time << ": " << message << std::endl;
        }

        // 특정 조건 검사
        if (checkSpecialCondition(rightPos, leftPos, nextLeftPositions)) {
            allSafe = false;
            std::cout << "특정 조건 위반: Measure " << measure << ", Time " << time
                      << " -> 다음 왼손 동작에서 위험 발생" << std::endl;
        }
    }

    if (allSafe) {
        std::cout << "모든 동작이 안전합니다." << std::endl;
    }
}

// 텍스트 파일에서 드럼 악보 데이터를 읽기
std::vector<std::tuple<int, double, int, int>> readDrumData(const std::string& filePath) {
    std::vector<std::tuple<int, double, int, int>> drumData;
    std::ifstream file(filePath);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "파일을 열 수 없습니다: " << filePath << std::endl;
        return drumData;
    }

    while (std::getline(file, line)) {
        if (line.empty() || line.find("bpm") != std::string::npos) {
            continue; // 빈 줄 또는 bpm 정보 무시
        }

        std::istringstream iss(line);
        int measure, rightPos, leftPos;
        double time;
        int dummy1, dummy2, dummy3; // 사용하지 않는 값들

        if (iss >> measure >> time >> rightPos >> leftPos >> dummy1 >> dummy2 >> dummy3) {
            drumData.emplace_back(measure, time, rightPos, leftPos);
        }
    }

    file.close();
    return drumData;
}

// 메인 함수
int main() {
    // 텍스트 파일 경로
    std::string filePath = "./include/codes/codeMOY_easy0.txt";

    // 드럼 악보 데이터 읽기
    std::vector<std::tuple<int, double, int, int>> drumData = readDrumData(filePath);

    if (drumData.empty()) {
        std::cerr << "드럼 데이터를 불러오지 못했습니다." << std::endl;
        return 1;
    }

    // 드럼 악보에서 충돌 및 조건 검사 수행
    checkDrumScore(drumData);

    return 0;
}


// 위험한 경우(오른손 위치 기준)

// 1. 오른손 S칠 때, 왼손이 FT치면 안됨, HT, R, MT, RC 치는 경우
// 2. 오른손 FT칠 때, 제약 없음
// 3. 오른손 MT칠 때, 왼손이 LC나 HH, HT, S중에 하나 치고 있다가 RC, R, FT를 치려는 경우
// 4. 오른손 HT칠 때, 왼손이 FT치면 위험. 또는 왼손이 LC나 HH이나 S치고 있다가 MT, RC, R, FT를 치려는 경우
// 5. 오른손 HH칠 때, 왼손으로 HT, MT, FT, R, RC중에 하나 치려하는 경우
// 6. 오른손 R칠 때, 제약 없음
// 7. 오른손 RC칠 때, 왼손으로 R을 치려는 경우
// 8. 오른손 LC칠 때, 왼손으로 RC, R치려는 경우