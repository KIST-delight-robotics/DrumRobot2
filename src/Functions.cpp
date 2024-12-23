#include "../include/tasks/Functions.hpp" // Include header file

Functions::Functions(std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef)
    : motors(motorsRef)
{
    // 파일 저장 시각 계산
    start = std::chrono::high_resolution_clock::now(); 
}

Functions::~Functions()
{

}

int Functions::get_com_number_by_hostname() {
    char hostname[1024];
    
    // 호스트 이름을 가져오기
    if (gethostname(hostname, sizeof(hostname)) != 0) {
        perror("gethostname");
        exit(1);
    }
    hostname[1023] = '\0';

    // 호스트 이름에 따라 값을 반환
    if (strcmp(hostname, "shy-desktop") == 0) {
        return 1;
    } else if (strcmp(hostname, "shy-MINIPC-VC66-C2") == 0) {
        return 2;
    } else {
        std::cerr << "Unrecognized hostname: " << hostname << std::endl;
        exit(1);
    }
}

void Functions::restCanPort(int com_number)
{
    char can1_on[100], can2_on[100], can3_on[100], can1_off[100], can2_off[100], can3_off[100];
    //com_number = 1 드럼로봇 컴퓨터 com_number = 2 테스트 환경 컴퓨터 
    // Reset the commands based on com_number
    if (com_number == 1) {
        //sudo uhubctl 이 명령어 실행하면 포트 검색가능
        //0c72:000c PEAK-System Technik GmbH PCAN-USB (8.6.1)]
        //요런식으로 나오는 애들이 can 통신선임
        //Current status for hub 1-4.1.1 [0bda:5411 Generic USB2.1 Hub, USB 2.10, 5 ports]
        //Port 1: 0503 power highspeed enable connect []
        //Port 2: 0103 power enable connect [0c72:000c PEAK-System Technik GmbH PCAN-USB (8.6.1)]
        //Port 3: 0103 power enable connect [0c72:000c PEAK-System Technik GmbH PCAN-USB (8.6.1)]
        // 예를 들어 포트 3번을 끄려고한다면 다음과 같이 명령어 넣어주면된다.
        //키려면
        //sudo uhubctl -l 1-4.1.1 -p 3 -a off
        //끄려면
        //sudo uhubctl -l 1-4.1.1 -p 3 -a off

        // com_number_1
        snprintf(can1_off, sizeof(can1_off), "sudo uhubctl -l 1-4 -p 1 -a off");
        snprintf(can2_off, sizeof(can2_off), "sudo uhubctl -l 1-4 -p 2 -a off");
        snprintf(can3_off, sizeof(can3_off), "sudo uhubctl -l 1-4 -p 3 -a off");

        snprintf(can1_on, sizeof(can1_on), "sudo uhubctl -l 1-4 -p 1 -a on");
        snprintf(can2_on, sizeof(can2_on), "sudo uhubctl -l 1-4 -p 2 -a on");
        snprintf(can3_on, sizeof(can3_on), "sudo uhubctl -l 1-4 -p 3 -a on");
    } else if (com_number == 2) {
        // com_number_2
        snprintf(can1_off, sizeof(can1_off), "sudo uhubctl -l 1-6.1 -p 1 -a off");
        snprintf(can1_on, sizeof(can1_on), "sudo uhubctl -l 1-6.1 -p 1 -a on");

        // For com_number_2, we only have can1_off and can1_on
        snprintf(can2_off, sizeof(can2_off), " "); // Empty command
        snprintf(can3_off, sizeof(can3_off), " "); // Empty command
        snprintf(can2_on, sizeof(can2_on), " ");  // Empty command
        snprintf(can3_on, sizeof(can3_on), " ");  // Empty command
    } else {
        fprintf(stderr, "Invalid com_number: %d\n", com_number);
        return;
    }
    //만든 명령줄 실행시키기 
    int ret1 = system(can1_off);
    std::cout << std::endl;
    int ret2 = system(can2_off);
    std::cout << std::endl;
    int ret3 = system(can3_off);
    std::cout << std::endl;

    sleep(2);

    int ret4 = system(can1_on);
    std::cout << std::endl;
    int ret5 = system(can2_on);
    std::cout << std::endl;
    int ret6 = system(can3_on);
    std::cout << std::endl;

    
    if (ret1 != 0 || ret2 != 0 || ret3 != 0 || ret4 != 0 || ret5 != 0 || ret6 != 0)
    {
        fprintf(stderr, "Failed to reset port\n");
    }

    sleep(2);
}

void Functions::openCSVFile()
{
    for (int i = 1; i < 100; i++)
    {
        // 기본 경로와 파일 이름을 결합
        std::string fullPath = basePath + file_name + to_string(i) + ".txt";

        // 파일이 이미 존재하는지 확인
        bool fileExists = std::ifstream(fullPath).good();

        if(!fileExists)
        {
            // 처음 실행 시
            file_name = file_name + to_string(i);
            std::cout << "Start Logging of Log Data : " << file_name << ".txt\n";

            for (auto &motor_pair : motors)
            {
                std::shared_ptr<GenericMotor> motor = motor_pair.second;
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
                {
                    appendToCSV_DATA(file_name, (float)motor->nodeId, tMotor->initialJointAngle, INIT_SIGN);
                }
                else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
                {
                    appendToCSV_DATA(file_name, (float)motor->nodeId, maxonMotor->initialJointAngle, INIT_SIGN);
                }
            }

            return;
        }
    }

    std::cerr << "Unable to open file" << std::endl;
}

// 시간를 CSV 파일에 한 줄씩 저장하는 함수
void Functions::appendToCSV_time(const std::string& filename) {
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> elapsed = now - start;
    std::ofstream file;
    std::string fullPath = basePath + filename + ".txt";  // 기본 경로와 파일 이름을 결합

    // 파일이 이미 존재하는지 확인
    bool fileExists = std::ifstream(fullPath).good();

    // 파일을 열 때 새로 덮어쓰기 모드로 열거나, 이미 존재할 경우 append 모드로 열기
    if (!fileExists) {
        file.open(fullPath, std::ios::out | std::ios::trunc);  // 처음 실행 시 덮어쓰기 모드로 열기
    } else {
        file.open(fullPath, std::ios::app);  // 이미 파일이 존재하면 append 모드로 열기
    }
    // 파일이 제대로 열렸는지 확인
    if (file.is_open()) {
        // 데이터 추가
        file << elapsed.count() << "\n";
        // 파일 닫기
        file.close();
    } else {
        std::cerr << "Unable to open file: " << fullPath << std::endl;
    }
}

// 시간과 변수를 CSV 파일에 한 줄씩 저장하는 함수
void Functions::appendToCSV_DATA(const std::string& filename, float A_DATA, float B_DATA, float C_DATA) {
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> elapsed = now - start;
    std::ofstream file;
    std::string fullPath = basePath + filename + ".txt";  // 기본 경로와 파일 이름을 결합

    // 파일이 이미 존재하는지 확인
    bool fileExists = std::ifstream(fullPath).good();

    // 파일을 열 때 새로 덮어쓰기 모드로 열거나, 이미 존재할 경우 append 모드로 열기
    if (!fileExists) {
        file.open(fullPath, std::ios::out | std::ios::trunc);  // 처음 실행 시 덮어쓰기 모드로 열기
    } else {
        file.open(fullPath, std::ios::app);  // 이미 파일이 존재하면 append 모드로 열기
    }
    // 파일이 제대로 열렸는지 확인

    if (file.is_open()) {
        // 데이터 추가
        file << elapsed.count() << "," << A_DATA << "," << B_DATA << "," << C_DATA << "\n";  // 시간과 float 변수들을 CSV 형식으로 한 줄에 기록
       
        // 파일 닫기
        file.close();
    } else {
        std::cerr << "Unable to open file: " << fullPath << std::endl;
    }
}

// 시간과 state(string)를 CSV 파일에 한 줄씩 저장하는 함수
void Functions::appendToCSV_State(const std::string& filename, string state, string sub_state) {
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> elapsed = now - start;
    std::ofstream file;
    std::string fullPath = basePath + filename + ".txt";  // 기본 경로와 파일 이름을 결합

    // 파일이 이미 존재하는지 확인
    bool fileExists = std::ifstream(fullPath).good();

    // 파일을 열 때 새로 덮어쓰기 모드로 열거나, 이미 존재할 경우 append 모드로 열기
    if (!fileExists) {
        file.open(fullPath, std::ios::out | std::ios::trunc);  // 처음 실행 시 덮어쓰기 모드로 열기
    } else {
        file.open(fullPath, std::ios::app);  // 이미 파일이 존재하면 append 모드로 열기
    }
    // 파일이 제대로 열렸는지 확인

    if (file.is_open()) {
        // 데이터 추가
        file << elapsed.count() << "," << state << ',' << sub_state << "\n";  // 시간과 state 변수들을 CSV 형식으로 한 줄에 기록
       
        // 파일 닫기
        file.close();
    } else {
        std::cerr << "Unable to open file: " << fullPath << std::endl;
    }
}

// 시간과 CAN Frame을 CSV 파일에 한 줄씩 저장하는 함수
void Functions::appendToCSV_CAN(const std::string& filename, can_frame& c_frame) {
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> elapsed = now - start;
    std::ofstream file;
    std::string fullPath = basePath + filename + ".txt";  // 기본 경로와 파일 이름을 결합

    // 파일이 이미 존재하는지 확인
    bool fileExists = std::ifstream(fullPath).good();

    // 파일을 열 때 새로 덮어쓰기 모드로 열거나, 이미 존재할 경우 append 모드로 열기
    if (!fileExists) {
        file.open(fullPath, std::ios::out | std::ios::trunc);  // 처음 실행 시 덮어쓰기 모드로 열기
    } else {
        file.open(fullPath, std::ios::app);  // 이미 파일이 존재하면 append 모드로 열기
    }
    // 파일이 제대로 열렸는지 확인

    if (file.is_open()) {
        // 데이터 추가
        file << elapsed.count(); // 시간과 float 변수들을 CSV 형식으로 한 줄에 기록
       
       // can_frame의 data 배열을 CSV 형식으로 저장
        for (int i = 0; i < 8; ++i) {
            file << "," << static_cast<int>(c_frame.data[i]);
        }
        file << "\n";

        // 파일 닫기
        file.close();
    } else {
        std::cerr << "Unable to open file: " << fullPath << std::endl;
    }
}
