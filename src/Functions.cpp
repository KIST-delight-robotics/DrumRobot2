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

int Functions::getComNumberByHostname() {
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
    } 
    else if (strcmp(hostname, "taehwang-14ZD90Q-GX56K") == 0) {  // 현재 컴퓨터 추가
        return 3; 
    }
    else {
        std::cerr << "Unrecognized hostname: " << hostname << std::endl;
        exit(1);
    }
}

void Functions::restCanPort()
{
    //shy-desktop -> 1반환
    //shy-MINIPC-VC66-C2 -> 2반환
    int com_number = getComNumberByHostname();   //com_number = 1 드럼로봇 컴퓨터 com_number = 2 테스트 환경 컴퓨터

    char can1_on[100], can2_on[100], can3_on[100], can4_on[100], can1_off[100], can2_off[100], can3_off[100], can4_off[100];

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
        snprintf(can4_off, sizeof(can4_off), "sudo uhubctl -l 1-4 -p 4 -a off");

        snprintf(can1_on, sizeof(can1_on), "sudo uhubctl -l 1-4 -p 1 -a on");
        snprintf(can2_on, sizeof(can2_on), "sudo uhubctl -l 1-4 -p 2 -a on");
        snprintf(can3_on, sizeof(can3_on), "sudo uhubctl -l 1-4 -p 3 -a on");
        snprintf(can4_on, sizeof(can4_on), "sudo uhubctl -l 1-4 -p 4 -a on");
    } else if (com_number == 2) {
        // com_number_2
        snprintf(can1_off, sizeof(can1_off), "sudo uhubctl -l 1-6.1 -p 1 -a off");
        snprintf(can1_on, sizeof(can1_on), "sudo uhubctl -l 1-6.1 -p 1 -a on");

        // For com_number_2, we only have can1_off and can1_on
        snprintf(can2_off, sizeof(can2_off), " "); // Empty command
        snprintf(can3_off, sizeof(can3_off), " "); // Empty command
        snprintf(can2_on, sizeof(can2_on), " ");  // Empty command
        snprintf(can3_on, sizeof(can3_on), " ");  // Empty command
    } 
    else if(com_number == 3){
        return;
    }
    else{
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
    int ret4 = system(can4_off);
    std::cout << std::endl;

    sleep(2);

    int ret5 = system(can1_on);
    std::cout << std::endl;
    int ret6 = system(can2_on);
    std::cout << std::endl;
    int ret7 = system(can3_on);
    std::cout << std::endl;
    int ret8 = system(can4_on);
    std::cout << std::endl;

    
    if (ret1 != 0 || ret2 != 0 || ret3 != 0 || ret4 != 0 || ret5 != 0 || ret6 != 0 || ret7 != 0 || ret8 != 0)
    {
        fprintf(stderr, "Failed to reset port\n");
    }

    sleep(2);
}

// CSV 파일에 생성 및 초기값 저장
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
                    appendToCSV(file_name, false, (float)motor->nodeId, tMotor->initialJointAngle, INIT_SIGN);
                }
                else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
                {
                    appendToCSV(file_name, false, (float)motor->nodeId, maxonMotor->initialJointAngle, INIT_SIGN);
                }
            }

            return;
        }
    }

    std::cerr << "Unable to open file" << std::endl;
}

std::ostringstream Functions::getAbsTime()
{
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::ostringstream timestamp;
    timestamp << std::put_time(std::localtime(&now_c), "%H:%M:%S");
    timestamp << "." << std::setfill('0') << std::setw(3) << millis.count();  // .밀리초 붙이기

    return timestamp;
}

void Functions::appendToCSV(const std::string& filename, bool useAbsTime) {
    std::ostringstream timestamp;
    std::chrono::duration<float> elapsed;
    std::ofstream file;
    std::string fullPath = basePath + filename + ".txt";    // 기본 경로와 파일 이름을 결합

    if (useAbsTime) {
        timestamp = getAbsTime();
    }
    else {
        auto now = std::chrono::high_resolution_clock::now();
        elapsed = now - start;
    }

    // 파일을 열 때 새로 덮어쓰기 모드로 열거나, 이미 존재할 경우 append 모드로 열기
    bool fileExists = std::ifstream(fullPath).good();
    if (!fileExists) {
        file.open(fullPath, std::ios::out | std::ios::trunc);   // 처음 실행 시 덮어쓰기 모드로 열기
    } else {
        file.open(fullPath, std::ios::app); // 이미 파일이 존재하면 append 모드로 열기
    }

    // 파일이 제대로 열렸는지 확인
    if (file.is_open()) {
        // 데이터 추가
        if (useAbsTime) {
            file << timestamp.str();
        }
        else {
            file << elapsed.count();
        }
        file << "\n";
        // 파일 닫기
        file.close();
    } else {
        std::cerr << "Unable to open file: " << fullPath << std::endl;
    }
}

void Functions::appendToCSV(const std::string& filename, bool useAbsTime, float A_DATA) {
    std::ostringstream timestamp;
    std::chrono::duration<float> elapsed;
    std::ofstream file;
    std::string fullPath = basePath + filename + ".txt";    // 기본 경로와 파일 이름을 결합

    if (useAbsTime) {
        timestamp = getAbsTime();
    }
    else {
        auto now = std::chrono::high_resolution_clock::now();
        elapsed = now - start;
    }

    // 파일을 열 때 새로 덮어쓰기 모드로 열거나, 이미 존재할 경우 append 모드로 열기
    bool fileExists = std::ifstream(fullPath).good();
    if (!fileExists) {
        file.open(fullPath, std::ios::out | std::ios::trunc);   // 처음 실행 시 덮어쓰기 모드로 열기
    } else {
        file.open(fullPath, std::ios::app); // 이미 파일이 존재하면 append 모드로 열기
    }

    // 파일이 제대로 열렸는지 확인
    if (file.is_open()) {
        // 데이터 추가
        if (useAbsTime) {
            file << timestamp.str();
        }
        else {
            file << elapsed.count();
        }
        file << "," << A_DATA << "\n";
        // 파일 닫기
        file.close();
    } else {
        std::cerr << "Unable to open file: " << fullPath << std::endl;
    }
}

void Functions::appendToCSV(const std::string& filename, bool useAbsTime, float A_DATA, float B_DATA) {
    std::ostringstream timestamp;
    std::chrono::duration<float> elapsed;
    std::ofstream file;
    std::string fullPath = basePath + filename + ".txt";    // 기본 경로와 파일 이름을 결합

    if (useAbsTime) {
        timestamp = getAbsTime();
    }
    else {
        auto now = std::chrono::high_resolution_clock::now();
        elapsed = now - start;
    }

    // 파일을 열 때 새로 덮어쓰기 모드로 열거나, 이미 존재할 경우 append 모드로 열기
    bool fileExists = std::ifstream(fullPath).good();
    if (!fileExists) {
        file.open(fullPath, std::ios::out | std::ios::trunc);   // 처음 실행 시 덮어쓰기 모드로 열기
    } else {
        file.open(fullPath, std::ios::app); // 이미 파일이 존재하면 append 모드로 열기
    }

    // 파일이 제대로 열렸는지 확인
    if (file.is_open()) {
        // 데이터 추가
        if (useAbsTime) {
            file << timestamp.str();
        }
        else {
            file << elapsed.count();
        }
        file << "," << A_DATA << "," << B_DATA << "\n";
        // 파일 닫기
        file.close();
    } else {
        std::cerr << "Unable to open file: " << fullPath << std::endl;
    }
}

void Functions::appendToCSV(const std::string& filename, bool useAbsTime, float A_DATA, float B_DATA, float C_DATA) {
    std::ostringstream timestamp;
    std::chrono::duration<float> elapsed;
    std::ofstream file;
    std::string fullPath = basePath + filename + ".txt";    // 기본 경로와 파일 이름을 결합

    if (useAbsTime) {
        timestamp = getAbsTime();
    }
    else {
        auto now = std::chrono::high_resolution_clock::now();
        elapsed = now - start;
    }

    // 파일을 열 때 새로 덮어쓰기 모드로 열거나, 이미 존재할 경우 append 모드로 열기
    bool fileExists = std::ifstream(fullPath).good();
    if (!fileExists) {
        file.open(fullPath, std::ios::out | std::ios::trunc);   // 처음 실행 시 덮어쓰기 모드로 열기
    } else {
        file.open(fullPath, std::ios::app); // 이미 파일이 존재하면 append 모드로 열기
    }

    // 파일이 제대로 열렸는지 확인
    if (file.is_open()) {
        // 데이터 추가
        if (useAbsTime) {
            file << timestamp.str();
        }
        else {
            file << elapsed.count();
        }
        file << "," << A_DATA << "," << B_DATA << "," << C_DATA << "\n";
        // 파일 닫기
        file.close();
    } else {
        std::cerr << "Unable to open file: " << fullPath << std::endl;
    }
}

void Functions::appendToCSV(const std::string& filename, bool useAbsTime, float A_DATA, float B_DATA, float C_DATA, float D_DATA) {
    std::ostringstream timestamp;
    std::chrono::duration<float> elapsed;
    std::ofstream file;
    std::string fullPath = basePath + filename + ".txt";    // 기본 경로와 파일 이름을 결합

    if (useAbsTime) {
        timestamp = getAbsTime();
    }
    else {
        auto now = std::chrono::high_resolution_clock::now();
        elapsed = now - start;
    }

    // 파일을 열 때 새로 덮어쓰기 모드로 열거나, 이미 존재할 경우 append 모드로 열기
    bool fileExists = std::ifstream(fullPath).good();
    if (!fileExists) {
        file.open(fullPath, std::ios::out | std::ios::trunc);   // 처음 실행 시 덮어쓰기 모드로 열기
    } else {
        file.open(fullPath, std::ios::app); // 이미 파일이 존재하면 append 모드로 열기
    }

    // 파일이 제대로 열렸는지 확인
    if (file.is_open()) {
        // 데이터 추가
        if (useAbsTime) {
            file << timestamp.str();
        }
        else {
            file << elapsed.count();
        }
        file << "," << A_DATA << "," << B_DATA << "," << C_DATA << "," << D_DATA << "\n";
        // 파일 닫기
        file.close();
    } else {
        std::cerr << "Unable to open file: " << fullPath << std::endl;
    }
}

void Functions::appendToCSV(const std::string& filename, bool useAbsTime, can_frame& c_frame) {
    std::ostringstream timestamp;
    std::chrono::duration<float> elapsed;
    std::ofstream file;
    std::string fullPath = basePath + filename + ".txt";    // 기본 경로와 파일 이름을 결합

    if (useAbsTime) {
        timestamp = getAbsTime();
    }
    else {
        auto now = std::chrono::high_resolution_clock::now();
        elapsed = now - start;
    }

    // 파일을 열 때 새로 덮어쓰기 모드로 열거나, 이미 존재할 경우 append 모드로 열기
    bool fileExists = std::ifstream(fullPath).good();
    if (!fileExists) {
        file.open(fullPath, std::ios::out | std::ios::trunc);   // 처음 실행 시 덮어쓰기 모드로 열기
    } else {
        file.open(fullPath, std::ios::app); // 이미 파일이 존재하면 append 모드로 열기
    }

    // 파일이 제대로 열렸는지 확인
    if (file.is_open()) {
        // 데이터 추가
        if (useAbsTime) {
            file << timestamp.str();
        }
        else {
            file << elapsed.count();
        }
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

void Functions::appendToCSV(const std::string& filename, bool useAbsTime, std::chrono::_V2::system_clock::time_point &__t) {
    std::ostringstream timestamp;
    std::chrono::duration<float> elapsed;
    std::ofstream file;
    std::string fullPath = basePath + filename + ".txt";    // 기본 경로와 파일 이름을 결합

    std::chrono::duration<float> elapsed2 = __t - start;

    if (useAbsTime) {
        timestamp = getAbsTime();
    }
    else {
        auto now = std::chrono::high_resolution_clock::now();
        elapsed = now - start;
    }

    // 파일을 열 때 새로 덮어쓰기 모드로 열거나, 이미 존재할 경우 append 모드로 열기
    bool fileExists = std::ifstream(fullPath).good();
    if (!fileExists) {
        file.open(fullPath, std::ios::out | std::ios::trunc);   // 처음 실행 시 덮어쓰기 모드로 열기
    } else {
        file.open(fullPath, std::ios::app); // 이미 파일이 존재하면 append 모드로 열기
    }

    // 파일이 제대로 열렸는지 확인
    if (file.is_open()) {
        // 데이터 추가
        if (useAbsTime) {
            file << timestamp.str();
        }
        else {
            file << elapsed.count();
        }
        file << "," << elapsed2.count() << "\n";
        // 파일 닫기
        file.close();
    } else {
        std::cerr << "Unable to open file: " << fullPath << std::endl;
    }
}

void Functions::appendToCSV(const std::string& filename, bool useAbsTime, std::vector<double> &__v) {
    std::ostringstream timestamp;
    std::chrono::duration<float> elapsed;
    std::ofstream file;
    std::string fullPath = basePath + filename + ".txt";    // 기본 경로와 파일 이름을 결합

    if (useAbsTime) {
        timestamp = getAbsTime();
    }
    else {
        auto now = std::chrono::high_resolution_clock::now();
        elapsed = now - start;
    }

    // 파일을 열 때 새로 덮어쓰기 모드로 열거나, 이미 존재할 경우 append 모드로 열기
    bool fileExists = std::ifstream(fullPath).good();
    if (!fileExists) {
        file.open(fullPath, std::ios::out | std::ios::trunc);   // 처음 실행 시 덮어쓰기 모드로 열기
    } else {
        file.open(fullPath, std::ios::app); // 이미 파일이 존재하면 append 모드로 열기
    }

    // 파일이 제대로 열렸는지 확인
    if (file.is_open()) {
        // 데이터 추가
        if (useAbsTime) {
            file << timestamp.str();
        }
        else {
            file << elapsed.count();
        }
        // vector의 data 배열을 CSV 형식으로 저장
        int n = __v.size();
        for (int i = 0; i < n; i++) {
            file << "," << __v[i];
        }
        file << "\n";
        // 파일 닫기
        file.close();
    } else {
        std::cerr << "Unable to open file: " << fullPath << std::endl;
    }
}

//midi to chord parsing
std::vector<std::string> Functions::splitByWhitespace(const std::string& line) {
    std::istringstream iss(line);
    std::vector<std::string> tokens;
    std::string temp;
    while (iss >> temp) {
        tokens.push_back(temp);
    }
    return tokens;
}

bool Functions::readMidiFile(const std::string& filename, std::vector<unsigned char>& buffer) {
    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    if (!file) {
        std::cerr << "Cannot open file: " << filename << std::endl;
        return false;
    }
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    buffer.resize(size);
    if (!file.read(reinterpret_cast<char*>(buffer.data()), size)) {
        std::cerr << "Failed to read file: " << filename << std::endl;
        return false;
    }
    return true;
}

size_t Functions::readTime(const std::vector<unsigned char>& data, size_t& pos) {
    size_t value = 0;
    while (pos < data.size()) {
        unsigned char byte = data[pos];
        value = (value << 7) | (byte & 0x7F);
        pos++;
        if ((byte & 0x80) == 0) break;
    }
    return value;
}

void Functions::handleMetaEvent(const std::vector<unsigned char>& data, size_t& pos, int &bpm) {
    unsigned char metaType = data[pos++];
    int length = static_cast<int>(data[pos++]);
    size_t startPos = pos;
    if (metaType == 0x21 && length == 1) {
    } else if (metaType == 0x58 && length == 4) {
        unsigned char numerator = data[pos];
        unsigned char denominator = 1 << data[pos + 1];
        // std::cout << "  - Time Signature: " << (int)numerator << "/" << (int)denominator << "\n";
    } else if (metaType == 0x51 && length == 3) {
        int tempo = ((data[pos] & 0xFF) << 16) |
                    ((data[pos + 1] & 0xFF) << 8) |
                    (data[pos + 2] & 0xFF);
        bpm = 60000000 / tempo;
        // std::cout << "  - Tempo Change: " << bpm << " BPM\n";
    } else if (metaType == 0x2F) {
        // std::cout << "  - End of Track reached\n";
    }
    pos = startPos + length;
}

void Functions::handleChannel10(const std::vector<unsigned char>& data, size_t& pos, unsigned char eventType) {
    unsigned char control = data[pos++];
    if (eventType == 0xB9) pos++;
}

void Functions::filterSmallDurations(const std::string& inputFilename, const std::string& outputFilename)
{
    double threshold = 0.05;
    std::ifstream inputFile(inputFilename);
    std::ofstream outputFile(outputFilename);

    if (!inputFile.is_open()) {
        std::cerr << "filterSmallDurations 입력 파일 열기 실패: " << inputFilename << std::endl;
        return;
    }

    if (!outputFile.is_open()) {
        std::cerr << "filterSmallDurations 출력 파일 열기 실패: " << outputFilename << std::endl;
        return;
    }

    std::string line;

    while (std::getline(inputFile, line)) {
        std::istringstream iss(line);
        double duration;
        int note;

        if (!(iss >> duration >> note)) {
            std::cerr << "filterSmallDurations 잘못된 형식: " << line << std::endl;
            continue;
        }

        // 임계값 이하의 시간은 0으로 설정
        if (duration < threshold) {
            duration = 0.0;
        }

        // 변경된 시간과 악기 정보 출력
        outputFile << std::fixed << std::setprecision(3)
                   << duration << "\t" << note << std::endl;
    }

    inputFile.close();
    outputFile.close();
}


void Functions::roundDurationsToStep(const std::string& inputFilename, const std::string& outputFilename)
{
    std::ifstream inputFile(inputFilename);
    std::ofstream outputFile(outputFilename);

    if (!inputFile.is_open()) {
        std::cerr << "roundDurationsToStep 입력 파일 열기 실패: " << inputFilename << std::endl;
        return;
    }

    if (!outputFile.is_open()) {
        std::cerr << "roundDurationsToStep 출력 파일 열기 실패: " << outputFilename << std::endl;
        return;
    }

    std::string line;
    const double step = 0.05;

    while (std::getline(inputFile, line)) {
        std::istringstream iss(line);
        double duration;
        int note;

        if (!(iss >> duration >> note)) {
            std::cerr << "roundDurationsToStep 잘못된 형식: " << line << std::endl;
            continue;
        }

        // 0.05 단위로 반올림
        double roundedDuration = std::round(duration / step) * step;

        outputFile << std::fixed << std::setprecision(3)
                   << roundedDuration << "\t" << note << std::endl;
    }

    inputFile.close();
    outputFile.close();

}

void Functions::handleNoteOn(const std::vector<unsigned char>& data, size_t& pos, double &note_on_time, int tpqn,int bpm, const std::string& midiFilePath) {
    if (pos + 2 > data.size()) return;
    unsigned char drumNote = data[pos++];
    unsigned char velocity = data[pos++];
    std::string drumName;
    switch ((int)drumNote) {
        case 36: drumName = "Bass Drum 1"; break;
        case 41: drumName = "Low Floor Tom"; break;
        case 38: drumName = "Acoustic Snare"; break;
        case 45: drumName = "Low Tom"; break;
        case 47: case 48: case 50: drumName = "Low Mid Tom"; break;
        case 42: drumName = "Closed Hi-Hat"; break;
        case 46: drumName = "Open Hi-Hat"; break;
        case 49: drumName = "Crash Cymbal 1"; break;
        case 51: drumName = "Ride Cymbal 1"; break;
        case 57: drumName = "Crash Cymbal 2"; break;
        default: drumName = "Unknown Drum"; break;
    }
    if (velocity > 0) {
        note_on_time = ((note_on_time * 60000) / (bpm * tpqn)) / 1000;
        // std::cout << std::fixed << std::setprecision(1) << note_on_time << "s\t" << "Hit Drum: " << drumName << " -> " << (int)drumNote << "\n";
        this->save_to_csv(midiFilePath, note_on_time, drumNote);
    }
}

void Functions::analyzeMidiEvent(const std::vector<unsigned char>& data, size_t& pos, unsigned char& runningStatus, double &note_on_time, int &tpqn,int &bpm, const std::string& midiFilePath) {
    if (pos >= data.size()) return;
    unsigned char eventType = data[pos];
   if (eventType == 0xFF || eventType == 0xB9 || eventType == 0xC9 || eventType == 0x99 || eventType == 0x89|| eventType == 0xA9) {
        runningStatus = eventType;
        pos++;
    } else {
        eventType = runningStatus;
    }
    if (eventType == 0xFF) {
        handleMetaEvent(data, pos, bpm);
    } else if (eventType == 0xB9 || eventType == 0xC9) {
        handleChannel10(data, pos, eventType);
    } else if (eventType == 0x99) {
        handleNoteOn(data, pos, note_on_time, tpqn, bpm, midiFilePath);
    } else if (eventType == 0x89 || eventType == 0xA9) {
        pos += 2;
    }
    else {
        std::cout << "unknown midiEvent - analyzeMidiEvent\n";
        pos++;
    }
}


void Functions::convertMcToC(const std::string& inputFilename, const std::string& outputFilename) {
    std::ifstream input(inputFilename);
    if (!input.is_open()) {
        std::cerr << " 입력 파일 열기 실패: " << inputFilename << "\n";
        return;
    }
    std::ofstream output(outputFilename);
    if (!output.is_open()) {
        std::cerr << " 출력 파일 생성 실패: " << outputFilename << "\n";
        return;
    }

    std::vector<Event> mergedEvents;  
    std::string line;
    double currentTime = 0.0;
    int hihatState = 1;

    while (std::getline(input, line)) {
        auto tokens = splitByWhitespace(line);
        if (tokens.size() != 2) continue;
        try {
            double delta = std::stod(tokens[0]);
            int rawNote = std::stoi(tokens[1]);
            int mapped = rawNote;
            if (mapped < 1 || mapped > 11) continue;
            if (mergedEvents.empty() || delta > 0) {
                currentTime += delta;
                mergedEvents.push_back({currentTime, {mapped}});
            } else {
                mergedEvents.back().notes.push_back(mapped);
            }
        } catch (...) { continue; }
    }

    double prevTime = 0.0;
    for (const auto& e : mergedEvents) {
        int inst1 = 0, inst2 = 0;
        int bassHit = 0;
        int hihat = hihatState;
        for (int note : e.notes) {
            if (note >= 1 && note <= 8) {
                if (inst1 == 0) inst1 = note;
                else if (inst2 == 0) inst2 = note;
            } else if (note == 10) {
                bassHit = 1;
            } else if (note == 11) {
                if (inst1 == 0) inst1 = 5;
                else if (inst2 == 0) inst2 = 5;
                hihatState = 1;
            } else if (note == 5) {
                hihatState = 0;
                if (inst1 == 0) inst1 = 5;
                else if (inst2 == 0) inst2 = 5;
            }
        }
        hihat = hihatState;
        double deltaTime = e.time - prevTime;
        prevTime = e.time;
        output << std::fixed << std::setprecision(3)
               << std::setw(6) << deltaTime
               << std::setw(6) << inst1
               << std::setw(6) << inst2
               << std::setw(6) << 0
               << std::setw(6) << 0
               << std::setw(6) << bassHit
               << std::setw(6) << hihat << "\n";
    }

    // std::cout << "변환 완료! 저장 위치 → " << outputFilename << "\n";
}

enum Hand { LEFT, RIGHT, SAME };

struct Coord {
    double x, y, z;
};

double Functions::dist(const Coord& a, const Coord& b) {
    return std::sqrt(
        (a.x - b.x)*(a.x - b.x) +
        (a.y - b.y)*(a.y - b.y) +
        (a.z - b.z)*(a.z - b.z)
    );
}

Functions::Hand Functions::getPreferredHandByDistance(int instCurrent, int prevRightNote, int prevLeftNote, double prevRightHit, double prevLeftHit) {
    // if (instCurrent <= 0 || instCurrent >= 9) return RIGHT;
    Coord curr = drumXYZ[instCurrent];
    Coord right = drumXYZ[prevRightNote];
    Coord left = drumXYZ[prevLeftNote];

    double dMax = 0.754;
    double dRight = Functions::dist(curr, right);
    double dLeft = Functions::dist(curr, left);
    double real_tRight = std::min(prevRightHit, 0.6) * 1.38;
    double real_tLeft  = std::min(prevLeftHit, 0.6) * 1.38;
    double normRight = std::min(dRight / dMax, 1.0);
    double normLeft = std::min(dLeft / dMax, 1.0);

    double rScore = (real_tRight / 0.6) * (1 - normRight);
    double lScore = (real_tLeft  / 0.6) * (1 - normLeft);

    // // 디버깅용 프린트문
    // std::cout << "\n[Hand 선택 판단]\n";
    // std::cout << " - instCurrent: " << instCurrent << "\n";
    // std::cout << " - prevRightNote: " << prevRightNote << ", prevLeftNote: " << prevLeftNote << "\n";
    // std::cout << " - 거리: right = " << dRight << ", left = " << dLeft << "\n";
    // std::cout << " - 시간누적: right = " << prevRightHit << ", left = " << prevLeftHit << "\n";
    // std::cout << " - 정규화 거리: right = " << normRight << ", left = " << normLeft << "\n";
    // std::cout << " - 점수: rScore = " << rScore << ", lScore = " << lScore << "\n";


    if (std::abs(rScore - lScore) < 1e-6) return SAME;

    return (lScore <= rScore) ? RIGHT : LEFT;
}

std::pair<int, int> Functions::assignHandsByPosition(int inst1, int inst2) {
    auto getSection = [](int inst) {
        if (inst == 5) return 1;
        if (inst == 1 || inst == 4 || inst == 8) return 2;
        if (inst == 2 || inst == 3 || inst == 6) return 3;
        if (inst == 7) return 4;
        return 0; // unknown
    };

    auto getSectionOrder = [](int inst) {
        // 낮을수록 왼쪽
        if (inst == 8) return 1;
        if (inst == 1) return 2;
        if (inst == 4) return 3;
        if (inst == 3) return 1;
        if (inst == 2) return 2;
        if (inst == 6) return 3;
        return 0;
    };

    int sec1 = getSection(inst1);
    int sec2 = getSection(inst2);

    // std::cout << "[섹션 손 분배 판단]\n";
    // std::cout << " - inst1: " << inst1 << " (섹션 " << sec1 << "), inst2: " << inst2 << " (섹션 " << sec2 << ")\n";

    if (sec1 < sec2) {
        // std::cout << " → 서로 다른 섹션: inst1이 더 왼쪽 → 왼손 = " << inst1 << ", 오른손 = " << inst2 << "\n";
        return {inst1, inst2};
    } else if (sec2 < sec1) {
        // std::cout << " → 서로 다른 섹션: inst2가 더 왼쪽 → 왼손 = " << inst2 << ", 오른손 = " << inst1 << "\n";
        return {inst2, inst1};
    } else {
        int order1 = getSectionOrder(inst1);
        int order2 = getSectionOrder(inst2);
        // std::cout << " → 같은 섹션 내 비교: order1 = " << order1 << ", order2 = " << order2 << "\n";
        if (order1 < order2) {
            // std::cout << "   → inst1이 더 왼쪽 → 왼손 = " << inst1 << ", 오른손 = " << inst2 << "\n";
            return {inst1, inst2};
        } else {
            // std::cout << "   → inst2가 더 왼쪽 → 왼손 = " << inst2 << ", 오른손 = " << inst1 << "\n";
            return {inst2, inst1};
        }
    }
}


// void Functions::assignHandsToEvents(const std::string& inputFilename, const std::string& outputFilename) {
//     std::ifstream input(inputFilename);
//     if (!input.is_open()) {
//         std::cerr << "입력 파일 열기 실패: " << inputFilename << "\n";
//         return;
//     }
//     std::ofstream output(outputFilename);
//     if (!output.is_open()) {
//         std::cerr << "출력 파일 생성 실패: " << outputFilename << "\n";
//         return;
//     }

//     struct FullEvent {
//         double time;
//         int inst1 = 0, inst2 = 0, bassHit = 0, hihat = 1;
//         int rightHand = 0, leftHand = 0;
//     };

//     std::string line;
//     std::vector<FullEvent> events;
//     int prevRight = 0, prevLeft = 0;
//     int prevRightNote = 1, prevLeftNote = 1;
//     double prevRightHit = 0, prevLeftHit = 0;

//     while (std::getline(input, line)) {
//         auto tokens = splitByWhitespace(line);
//         if (tokens.size() != 7) continue;

//         // inst1, inst2 가 칠 악기이며 그 전 단계에서 무조건 1부터 채운 후 2를 채우게 된다,
//         FullEvent e;
//         e.time = std::stod(tokens[0]);
//         e.inst1 = std::stoi(tokens[1]);
//         e.inst2 = std::stoi(tokens[2]);
//         e.bassHit = std::stoi(tokens[5]);
//         e.hihat = std::stoi(tokens[6]);

//         int inst1 = e.inst1, inst2 = e.inst2;
//         prevRightHit += e.time;
//         prevLeftHit += e.time;

//         //오른손으로 크러시 칠때 
//         //양손 크로스 될때 
//         //prevRight, prevLeft 전줄에 친 악기
//         //prevRightHit, prevLefttHit
//         //크러쉬 관련한 손 어사인
//         if (inst1 == 8 || inst2 == 8) {
//             if(prevLeft == 2 ||prevLeft == 3|| prevLeft == 6)
//             {
//                 e.rightHand = 7;
//                 e.leftHand = (inst1 == 8) ? inst2 : inst1;
//             }
//             else
//             {
//                 if (inst1 == 2 || inst1 == 3 || inst1 == 6 || inst2 == 2 || inst2 == 3 || inst2 == 6) {
//                     e.rightHand = 7;
//                     e.leftHand = (inst1 == 8) ? inst2 : inst1;
//                 }
//                 else{
//                     e.rightHand = 8;
//                     e.leftHand = (inst1 == 8) ? inst2 : inst1;
//                 }
//             }
//         } 
//         // 양손 다 칠 때 스네어가 있으면 무조건 왼손 스네어 스네어 없으면 올느쪽악기면 오른손 왼손 악기면 왼손
//         else if (inst1 != 0 && inst2 != 0) {
//             if (inst1 == 1 || inst2 == 1) {
//                 e.leftHand = (inst1 == 1) ? inst1 : inst2;
//                 e.rightHand = (inst1 == 1) ? inst2 : inst1;
//             } else {
//                 e.rightHand = (inst1 == 2 || inst1 == 3 || inst1 == 6 || inst1 == 7) ? inst1 : inst2;
//                 e.leftHand = (e.rightHand == inst1) ? inst2 : inst1;
//             }
//         } else if (inst1 != 0) {
//             if (inst1 == prevRight || inst1 == prevLeft) {
//                 if (e.time <= 0.1) {
//                     e.rightHand = (inst1 == prevRight) ? inst1 : 0;
//                     e.leftHand = (inst1 == prevLeft) ? inst1 : 0;
//                 } else {
//                     Hand preferred = Functions::getPreferredHandByDistance(inst1, prevRightNote, prevLeftNote, prevRightHit, prevLeftHit);
//                     if (preferred == RIGHT) e.rightHand = inst1;
//                     else if (preferred == LEFT) e.leftHand = inst1;
//                     else e.rightHand = inst1;
//                 }
//             } else {
//                 if (inst1 == 2 || inst1 == 3 || inst1 == 6 || inst1 == 7) e.rightHand = inst1;
//                 else e.leftHand = inst1;
//             }
//         }
//         prevRight = e.rightHand;
//         prevLeft = e.leftHand;
//         if (e.rightHand != 0) { prevRightNote = e.rightHand; prevRightHit = 0; }
//         if (e.leftHand != 0) { prevLeftNote = e.leftHand; prevLeftHit = 0; }

//         events.push_back(e);
//     }

//     for (const auto& e : events) {
//         int rightFlag = 0;
//         int leftFlag = 0;
//         if(e.rightHand != 0)    rightFlag = 5;
//         if(e.leftHand != 0)     leftFlag = 5;
//         output << std::fixed << std::setprecision(3)
//                << e.time
//                << std::setw(6) << e.rightHand
//                << std::setw(6) << e.leftHand
//                << std::setw(6) << rightFlag
//                << std::setw(6) << leftFlag
//                << std::setw(6) << e.bassHit
//                << std::setw(6) << e.hihat << "\n";
//     }

//     // std::cout << "손 어사인 포함 변환 완료! 저장 위치 → " << outputFilename << "\n";
// }

void Functions::assignHandsToEvents(const std::string& inputFilename, const std::string& outputFilename) {
    std::ifstream input(inputFilename);
    if (!input.is_open()) {
        std::cerr << "입력 파일 열기 실패: " << inputFilename << "\n";
        return;
    }
    std::ofstream output(outputFilename);
    if (!output.is_open()) {
        std::cerr << "출력 파일 생성 실패: " << outputFilename << "\n";
        return;
    }

    struct FullEvent {
        double time;
        int inst1 = 0, inst2 = 0, bassHit = 0, hihat = 1;
        int rightHand = 0, leftHand = 0;
    };

    std::string line;
    std::vector<FullEvent> events;
    int prevRight = 1, prevLeft = 1;
    int prevRightNote = 1, prevLeftNote = 1;
    double prevRightHit = 0, prevLeftHit = 0;

    while (std::getline(input, line)) {
        auto tokens = splitByWhitespace(line);
        if (tokens.size() != 7) continue;

        FullEvent e;
        e.time = std::stod(tokens[0]);
        e.inst1 = std::stoi(tokens[1]);
        e.inst2 = std::stoi(tokens[2]);
        e.bassHit = std::stoi(tokens[5]);
        e.hihat = std::stoi(tokens[6]);

        int inst1 = e.inst1, inst2 = e.inst2;
        prevRightHit += e.time;
        prevLeftHit += e.time;

        // std::cout << "[Time: " << e.time << "] inst1: " << inst1 << ", inst2: " << inst2
        //           << " | PrevR: " << prevRight << ", PrevL: " << prevLeft
        //           << " | RHit: " << prevRightHit << ", LHit: " << prevLeftHit << "\n";
        
        //step 1 크러시가 있는지 확인 크러쉬가 있다면 
        if (inst1 == 8 || inst2 == 8) {
            // std::cout << "→ 크러시 처리 진입\n";
            if(prevLeft == 2 || prevLeft == 3 || prevLeft == 6) {
                e.rightHand = 7;
                e.leftHand = (inst1 == 8) ? inst2 : inst1;
            } else {
                if (inst1 == 2 || inst1 == 3 || inst1 == 6 || inst2 == 2 || inst2 == 3 || inst2 == 6) {
                    e.rightHand = 7;
                    e.leftHand = (inst1 == 8) ? inst2 : inst1;
                } else {
                    e.rightHand = 8;
                    e.leftHand = (inst1 == 8) ? inst2 : inst1;
                }
            }
        }
        // step 2 양손 연주인지 한손인지 구분 
        else if (inst1 != 0 && inst2 != 0) {
            // std::cout << "→ 양손 처리 진입\n";
            // 1번 S와 5번 H-H 을 같이 치는 경우 오른손으로 H-H 왼손으로 S 치도록 설정
            if ((inst1 == 5 && inst2 == 1) || (inst1 == 1 && inst2 == 5)) 
            {
                e.leftHand = (inst1 == 5) ? inst2 : inst1;
                e.rightHand = (inst1 == 5) ? inst1 : inst2;
            }
            // 위의 경우를 제외한 모든 경우 악기 위치 기반으로 손 분배 
            else 
            {
                auto [left, right] = assignHandsByPosition(inst1,inst2);
                e.leftHand = left;
                e.rightHand = right;
            }
        }
        // step 3 한손 연주시 처리 
        else if (inst1 != 0) {
            // std::cout << "→ 한손 처리 진입\n";
            // 이전에 쳤던 악기와 같은 악기가 감지된다면 짧은 시간에 타격해야 할 시 같은 손 유지 시간차이가 크다면 거리 기반 판단
            if (inst1 == prevRight || inst1 == prevLeft) {
                // std::cout << "    - 이전 손과 같은 악기 감지\n";
                if (e.time <= 0.1) {
                    // std::cout << "    - 시간차 0.1 이하 → 같은 손 유지\n";
                    if(inst1 == prevRight)
                    {
                        e.rightHand = inst1;
                        e.leftHand = 0;
                    }
                    else
                    {
                        e.rightHand = 0;
                        e.leftHand = inst1;
                    }
                } else {
                    // std::cout << "    - 시간차 큼 → 거리 기반 판단\n";
                    Hand preferred = getPreferredHandByDistance(inst1, prevRightNote, prevLeftNote, prevRightHit, prevLeftHit);
                    if (preferred == RIGHT) {
                        // std::cout << "    → RIGHT 선택\n";
                        e.rightHand = inst1;
                        e.leftHand = 0;
                    } else if (preferred == LEFT) {
                        // std::cout << "    → LEFT 선택\n";
                        e.leftHand = inst1;
                        e.rightHand = 0;
                    }    
                    else {
                        // std::cout << "    → 점수 같고 시간 널널 오른손 우선권 선택\n";
                        // 여기는 한손 연주이면서 전에 쳤던 악기를 치는 것이지만 시간과 거리에 대한 점수도 모두 동일함 일단 오른손에 우선권을 주겠다.
                        e.rightHand = inst1;
                        e.leftHand = 0;
                    }
                }
            } else {
                // std::cout << "    - 이전 손과 다른 악기 → 거리 기반 판단\n";
                Hand preferred = getPreferredHandByDistance(inst1, prevRightNote, prevLeftNote, prevRightHit, prevLeftHit);
                    if (preferred == RIGHT) {
                        // std::cout << "    → RIGHT 선택\n";
                        e.rightHand = inst1;
                    } else if (preferred == LEFT) {
                        // std::cout << "    → LEFT 선택\n";
                        e.leftHand = inst1;
                    } 
                    //악기 위치 거리 기반으로 손 분배 이때 전에 친악기를 inst2로 사용해서 구함  
                    else {
                        // std::cout << "    → SAME 판단 → 섹션 기반 분배\n";
                        
                        int inst2 = (prevRightNote != 0) ? prevRightNote : prevLeftNote;
                        auto [left, right] = assignHandsByPosition(inst1, inst2);
                        if(left  == inst1)
                            e.leftHand  = (left  == inst1) ? inst1 : 0;
                        else
                            e.rightHand = (right == inst1) ? inst1 : 0;
                    
                        // std::cout << "      - inst1: " << inst1 << ", inst2: " << inst2
                        //             << " → 왼손 = " << e.leftHand << ", 오른손 = " << e.rightHand << "\n";
                    }
            }
        }

        // std::cout << "→ 결과: RH = " << e.rightHand << ", LH = " << e.leftHand << "\n\n";

        prevRight = e.rightHand;
        prevLeft = e.leftHand;
        if (e.rightHand != 0) { prevRightNote = e.rightHand; prevRightHit = 0; }
        if (e.leftHand != 0) { prevLeftNote = e.leftHand; prevLeftHit = 0; }

        events.push_back(e);
    }

    for (const auto& e : events) {
        int rightFlag = 0;
        int leftFlag = 0;
        if(e.rightHand != 0)    rightFlag = 5;
        if(e.leftHand != 0)     leftFlag = 5;
        output << std::fixed << std::setprecision(3)
               << e.time
               << std::setw(6) << e.rightHand
               << std::setw(6) << e.leftHand
               << std::setw(6) << rightFlag
               << std::setw(6) << leftFlag
               << std::setw(6) << e.bassHit
               << std::setw(6) << e.hihat << "\n";
    }
}


// 그루브 관련 처리
void Functions::addGroove(int bpm, const std::string& inputFilename, const std::string& outputFilename) {
    std::ifstream inFile(inputFilename);
    std::ofstream outFile(outputFilename);

    std::vector<std::vector<double>> values;
    std::vector<double> originalTimes;

        std::string line;
    // 1. 파일 읽기 및 파싱
    while (std::getline(inFile, line)) {
        std::istringstream ss(line);
        std::vector<double> row;
        double num;
        while (ss >> num) {
            row.push_back(num);
        }
        values.push_back(row);
        originalTimes.push_back(row[0]);  // 시간만 따로 저장
    }

    // 2. 누적합 기준 계산
    double threshold = (60.0 / bpm) * 2;
    double accTime = 0.0;

    // 3. 시간 조정 플래그 처리
    for (size_t i = 0; i < values.size(); ++i) {
        accTime += originalTimes[i];  // 항상 원본 기준 누적합 계산

        if (accTime >= threshold) {
            // 1. 현재 줄 시간값 -0.05
            values[i][0] -= 0.05;

            // 2. 다음 줄 존재하면 +0.05
            if (i + 1 < values.size()) {
                values[i + 1][0] += 0.05;
            }

            // 누적합 초기화
            accTime = 0.0;
        }
    }

    // 4. 결과 출력
    for (const auto& row : values) {

        outFile << std::fixed << std::setprecision(3) << row[0];
    

        outFile.unsetf(std::ios::fixed);
        outFile << std::setprecision(6);
    
        for (size_t i = 1; i < row.size(); ++i) {
            outFile << "\t" << row[i];
        }
        outFile << "\n";
    }

    
    inFile.close();
    outFile.close();
}

//세기 관련 처리

//악보에 적용시키는거
bool Functions::loadSegments(const string& intensityFile, vector<Functions::Seg>& segs) {
    ifstream fin(intensityFile);
    if (!fin) return false;

    string line;
    bool headerDone = false;

    while (getline(fin, line)) {
        // 1) 라인 트림
        auto trim = [](const string& s) -> string {
            auto issp = [](unsigned char c){ return std::isspace(c); };
            auto b = find_if_not(s.begin(), s.end(), issp);
            auto e = find_if_not(s.rbegin(), s.rend(), issp).base();
            return (b >= e) ? string() : string(b, e);
        };
        line = trim(line);
        if (line.empty()) continue;

        // 2) 콤마→공백 치환 + 공백 파싱 (토큰화)
        vector<string> toks;
        {
            string tmp = line;
            for (char& c : tmp) if (c == ',') c = ' ';
            istringstream iss(tmp);
            string tok;
            while (iss >> tok) toks.push_back(trim(tok)); // 토큰도 트림
        }
        if (toks.size() < 4) continue;

        // 3) BOM 제거(첫 토큰에만 필요 시)
        if (!toks[0].empty()
            && (unsigned char)toks[0][0] == 0xEF
            && toks[0].size() >= 3
            && (unsigned char)toks[0][1] == 0xBB
            && (unsigned char)toks[0][2] == 0xBF) {
            toks[0].erase(0, 3);
        }

        // 4) 헤더 스킵: 앞 두 칼럼이 숫자가 아니면 헤더로 간주
        auto looksNumber = [](const string& s) {
            if (s.empty()) return false;
            char* endp = nullptr;
            (void)strtod(s.c_str(), &endp);
            return endp != s.c_str(); // 앞에서 숫자를 하나라도 읽었는지
        };
        if (!headerDone && (!looksNumber(toks[0]) || !looksNumber(toks[1]))) {
            headerDone = true;
            continue;
        }

        // 5) 파싱 + 반올림(세기 0~3 가정)
        Seg s{};
        s.start      = stod(toks[0]);
        s.end        = stod(toks[1]);
        s.drum_avg   = (int)lround(stod(toks[2]));
        s.cymbal_avg = (int)lround(stod(toks[3]));
        segs.push_back(s);
        headerDone = true;
    }
    fin.close();

    // 6) 구간 정렬
    sort(segs.begin(), segs.end(), [](const Seg& a, const Seg& b){
        if (a.start == b.start) return a.end < b.end;
        return a.start < b.start;
    });

    return true;
}


bool Functions::applyIntensityToScore(const vector<Functions::Seg>& segs, const string& scoreIn, const string& scoreOut, bool mapTo357)
{
    ifstream sin(scoreIn);
    if (!sin) return false;
    ofstream sout(scoreOut);
    if (!sout) return false;

    // ---- 로컬 유틸(전부 이 함수 안에) ----
        auto trim = [](const string& s) -> string {
        auto issp = [](unsigned char c){ return std::isspace(c); };
        auto b = find_if_not(s.begin(), s.end(), issp);
        auto e = find_if_not(s.rbegin(), s.rend(), issp).base();
        return (b >= e) ? string() : string(b, e);
    };
    size_t cursor = 0; // 전진 포인터
    auto findSeg = [&](double t) -> const Seg* {
    while (cursor < segs.size()) {
        const Seg& s = segs[cursor];
        if (t < s.start) {
            if (cursor == 0) return nullptr;
            const Seg& p = segs[cursor - 1];
            if (t >= p.start && t < p.end) return &p;
            return nullptr;
        }
        if (t >= s.start && t < s.end) return &s;
        ++cursor;
    }
        if (!segs.empty()) {
            const Seg& last = segs.back();
            if (t >= last.start && t < last.end) return &last;
        }
        return nullptr;
    };
    auto mapIntensity = [&](int base) -> int {
        if (!mapTo357) return base;            // 원시 세기 사용 옵션
        if (base <= 1) return 3;               // 0/1 -> 3
        if (base == 2) return 5;               // 2     -> 5
        return 7;                               // 3+    -> 7
    };
    // --------------------------------------

    double accumTime = 0.0;
    string line;

    while (getline(sin, line)) {
        string raw = trim(line);
        if (raw.empty()) { sout << "\n"; continue; }

        // 콤마 → 공백 치환 + 공백/탭 파싱
        vector<string> toks;
        {
            string tmp = raw;
            for (char& c : tmp) if (c == ',') c = ' ';
            istringstream iss(tmp);
            string tok;
            while (iss >> tok) toks.push_back(tok);
        }

        // 최소 5컬럼(time R_inst L_inst R_vel L_vel) 아니면 원본 그대로 출력
        if (toks.size() < 5) { sout << raw << "\n"; continue; }

        // time은 Δt로 가정(절대시간이면 아래 한 줄을 accumTime = dt; 로 변경)
        double dt;
        try { dt = stod(toks[0]); }
        catch(...) { sout << raw << "\n"; continue; }
        accumTime += dt; // ← 절대시간이면: accumTime = dt;

        // 필드 파싱
        int R_inst, L_inst, R_vel, L_vel;
        try {
        R_inst = stoi(toks[1]);
        L_inst = stoi(toks[2]);
        R_vel  = stoi(toks[3]);
        L_vel  = stoi(toks[4]);
        } catch(...) { sout << raw << "\n"; continue; }

        // 구간 찾고 세기 적용
        if (!segs.empty()) {
            if (const Seg* seg = findSeg(accumTime)) {
                auto pick = [&](int inst, int oldv) -> int {
                    if (inst == 0) return oldv; // 무음은 유지
                    int base = (inst>=1 && inst<=4) ? seg->drum_avg
                        : (inst>=5 && inst<=8) ? seg->cymbal_avg : 0;
                    return mapIntensity(base);
                };
                R_vel = pick(R_inst, R_vel);
                L_vel = pick(L_inst, L_vel);
            }
        }

        // 갱신 후 탭 구분으로 출력(추가 컬럼 보존)
        toks[3] = to_string(R_vel);
        toks[4] = to_string(L_vel);
        for (size_t i = 0; i < toks.size(); ++i) {
            if (i) sout << "\t";
            sout << toks[i];
        }
        sout << "\n";
    }
    return true;
}

//세기 처리해서 파싱하는거 
void Functions::analyzeVelocityWithLowPassFilter(const std::string& velocityFile, const std::string& outputFile, double bpm)
{
    double windowSize = (60/bpm)*4;

    std::ifstream in(velocityFile);
    if (!in.is_open()) {
        std::cerr << "velocityFile 열기 실패: " << velocityFile << "\n";
        return;
    }

    std::vector<Functions::VelocityEntry> rawData;
    double t;
    int inst, vel;
    double maxTime = 0.0;

    // CSV 파일 읽기
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;

        // 콤마 → 공백 변환 (콤마/공백 혼합 지원)
        for (char &c : line) {
            if (c == ',') c = ' ';
        }

        std::stringstream ss(line);
        if (!(ss >> t >> inst >> vel)) continue; // 읽기 실패 시 건너뜀


        if (t == -1) break; // 종료 신호


        rawData.push_back({t, inst, vel});
        if (t > maxTime) maxTime = t;
    }

    int numWindows = static_cast<int>(std::ceil((maxTime + 0.001) / windowSize));

    std::vector<double> avgVelocityDrum(numWindows, 0.0);  // 1~4
    std::vector<int> countDrum(numWindows, 0);

    std::vector<double> avgVelocityCym(numWindows, 0.0);   // 5~8
    std::vector<int> countCym(numWindows, 0);

    // 시간 구간별 누적
    for (const auto& entry : rawData) {
        int bin = static_cast<int>(entry.time / windowSize);
        if (bin >= numWindows) continue;

        if (entry.instrument >= 1 && entry.instrument <= 4) {
            avgVelocityDrum[bin] += entry.velocity;
            countDrum[bin]++;
        }
        else if (entry.instrument >= 5 && entry.instrument <= 8) {
            avgVelocityCym[bin] += entry.velocity;
            countCym[bin]++;
        }
    }

    for (int i = 0; i < numWindows; ++i) {
        if (countDrum[i] > 0) {
            avgVelocityDrum[i] = static_cast<int>(std::round((avgVelocityDrum[i] / countDrum[i]) / 40.0));
        }
        if (countCym[i] > 0) {
            avgVelocityCym[i] = static_cast<int>(std::round((avgVelocityCym[i] / countCym[i]) / 40.0));
        }
    }

    // 결과 저장
    std::ofstream out(outputFile);
    if (!out.is_open()) {
        std::cerr << "outputFile 열기 실패: " << outputFile << "\n";
        return;
    }

    out << "start_time\tend_time\tdrum_avg\tcymbal_avg\n";
    for (int i = 0; i < numWindows; ++i) {
        double startT = i * windowSize;
        double endT = (i + 1) * windowSize;
        out << std::fixed << std::setprecision(3)
        << startT << "\t" << endT << "\t"
        << avgVelocityDrum[i] << "\t" << avgVelocityCym[i] << "\n";
    }

    std::cout << "[완료] 드럼/심벌 평균 벨로시티 저장 완료: " << outputFile << "\n";
}

// 박자 단위 분할 및 마디 번호 부여 함수
void Functions::convertToMeasureFile(const std::string& inputFilename, const std::string& outputFilename, bool endFlag) {
    struct DrumEvent {
        double time;
        int rightInstrument;
        int leftInstrument;
        int rightPower;
        int leftPower;
        int isBass;
        int hihatOpen;
    };

    std::ifstream input(inputFilename);
    if (!input.is_open()) {
        std::cerr << "입력 파일 열기 실패: " << inputFilename << "\n";
        return;
    }
    std::ofstream output(outputFilename);
    if (!output.is_open()) {
        std::cerr << "출력 파일 생성 실패: " << outputFilename << "\n";
        return;
    }

    std::string line;
    std::vector<DrumEvent> result;

    while (std::getline(input, line)) {
        std::stringstream ss(line);
        DrumEvent ev;
        ss >> ev.time >> ev.rightInstrument >> ev.leftInstrument
           >> ev.rightPower >> ev.leftPower >> ev.isBass >> ev.hihatOpen;

        int count = static_cast<int>(ev.time / 0.6);
        double leftover = ev.time - count * 0.6;

        for (int i = 0; i < count; ++i) {
            DrumEvent mid{0.6, 0, 0, 0, 0, 0, 0};
            result.push_back(mid);
        }
        if (leftover > 1e-6) {
            ev.time = leftover;
            result.push_back(ev);
        }
    }

    output << "1\t 0.600\t 0\t 0\t 0\t 0\t 0\t 0\n";

    double measureTime = 0.0;
    int measureNum = 1;
    // const double EPS = 1e-6;
    const double MEASURE_LIMIT = 2.4;

    for (const auto& ev : result) {
        measureTime += ev.time;
        // 마디 시간이 2.4초를 넘으면 새로운 마디로 시작
        if (measureTime >= MEASURE_LIMIT) {
            measureNum++;
            measureTime = ev.time;  // 새로운 마디 시간은 현재 이벤트의 시간으로 시작
        }
        output << measureNum << "\t "
               << std::fixed << std::setprecision(3) << ev.time << "\t "
               << ev.rightInstrument << "\t "
               << ev.leftInstrument << "\t "
               << ev.rightPower << "\t "
               << ev.leftPower << "\t "
               << ev.isBass << "\t "
               << ev.hihatOpen << "\n";
    }

    if (endFlag)
    {
        output << measureNum+1 << "\t 0.600\t 0\t 0\t 0\t 0\t 0\t 0\n";
        output << "-1" << "\t 0.600\t 1\t 1\t 1\t 1\t 1\t 1\n";
    }
}


void Functions::save_to_csv(const std::string& outputCsvPath, double &note_on_time, int drumNote) {
    std::ofstream file(outputCsvPath, std::ios::app);
    if (!file) {
        std::cerr << "Failed to open CSV file: " << outputCsvPath << std::endl;
        return;
    }
    int mappedDrumNote;
    switch (drumNote) {
        case 38: mappedDrumNote = 1; break;
        case 41: mappedDrumNote = 2; break;
        case 45: mappedDrumNote = 3; break;
        case 47: case 48: case 50: mappedDrumNote = 4; break;
        case 42: mappedDrumNote = 11; break;
        case 51: mappedDrumNote = 6; break;
        case 49: mappedDrumNote = 8; break;
        case 57: mappedDrumNote = 7; break;
        case 36: mappedDrumNote = 10; break;
        case 46: mappedDrumNote = 5; break;
        default: mappedDrumNote = 0; break;
    }
    file << note_on_time << "\t " << mappedDrumNote << "\n";
    file.close();
    note_on_time = 0;
}

// 폴더 내부 파일 지우는 함수
void Functions::clear_directory(const std::filesystem::path& dir_path) 
{
    namespace fs = std::filesystem;

    // 1. 해당 경로가 존재하는지, 그리고 디렉토리가 맞는지 확인합니다.
    if (!fs::exists(dir_path) || !fs::is_directory(dir_path)) {
        std::cerr << "오류: '" << dir_path << "' 경로가 존재하지 않거나 디렉토리가 아닙니다." << std::endl;
        return;
    }

    // 2. 디렉토리 내부를 순회하면서 각 항목을 삭제합니다.
    for (const auto& entry : fs::directory_iterator(dir_path)) {
        try {
            fs::remove(entry.path());
        } catch (const fs::filesystem_error& e) {
            std::cerr << "오류: " << entry.path() << " 삭제 실패. " << e.what() << std::endl;
        }
    }
}