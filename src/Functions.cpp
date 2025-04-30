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

    char can1_on[100], can2_on[100], can3_on[100], can1_off[100], can2_off[100], can3_off[100];

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

// 절대시간과 변수를 CSV 파일에 한 줄씩 저장하는 함수
void Functions::appendToCSV_DATA_absTime(const std::string& filename, float A_DATA, float B_DATA, float C_DATA) {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::ostringstream timestamp;
    timestamp << std::put_time(std::localtime(&now_c), "%H:%M:%S");
    timestamp << "." << std::setfill('0') << std::setw(3) << millis.count();  // .밀리초 붙이기

    std::ofstream file;
    std::string fullPath = basePath + filename + ".txt";

    bool fileExists = std::ifstream(fullPath).good();
    if (!fileExists) {
        file.open(fullPath, std::ios::out | std::ios::trunc);
    } else {
        file.open(fullPath, std::ios::app);
    }

    if (file.is_open()) {
        file << timestamp.str() << "," << A_DATA << "," << B_DATA << "," << C_DATA << "\n";
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

void Functions::handleMetaEvent(const std::vector<unsigned char>& data, size_t& pos, int &initial_setting_flag) {
    unsigned char metaType = data[pos++];
    int length = static_cast<int>(data[pos++]);
    size_t startPos = pos;
    if (metaType == 0x21 && length == 1) {
        initial_setting_flag = 1;
    } else if (metaType == 0x58 && length == 4) {
        unsigned char numerator = data[pos];
        unsigned char denominator = 1 << data[pos + 1];
        // std::cout << "  - Time Signature: " << (int)numerator << "/" << (int)denominator << "\n";
    } else if (metaType == 0x51 && length == 3) {
        int tempo = ((data[pos] & 0xFF) << 16) |
                    ((data[pos + 1] & 0xFF) << 8) |
                    (data[pos + 2] & 0xFF);
        int bpm = 60000000 / tempo;
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

        // ⏱️ 0.005 단위로 반올림
        double roundedDuration = std::round(duration / step) * step;

        outputFile << std::fixed << std::setprecision(3)
                   << roundedDuration << "\t" << note << std::endl;
    }

    inputFile.close();
    outputFile.close();

}

void Functions::handleNoteOn(const std::vector<unsigned char>& data, size_t& pos, double &note_on_time, int tpqn, const std::string& midiFilePath) {
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
        note_on_time = ((note_on_time * 60000) / (100 * tpqn)) / 1000;
        // std::cout << std::fixed << std::setprecision(1) << note_on_time << "s\t" << "Hit Drum: " << drumName << " -> " << (int)drumNote << "\n";
        this->save_to_csv(midiFilePath, note_on_time, drumNote);
    }
}

void Functions::analyzeMidiEvent(const std::vector<unsigned char>& data, size_t& pos, unsigned char& runningStatus, int &initial_setting_flag, double &note_on_time, int &tpqn, const std::string& midiFilePath) {
    if (pos >= data.size()) return;
    unsigned char eventType = data[pos];
    if (eventType == 0xFF || eventType == 0xB9 || eventType == 0xC9 || eventType == 0x99) {
        runningStatus = eventType;
        pos++;
    } else {
        eventType = runningStatus;
    }
    if (eventType == 0xFF) {
        handleMetaEvent(data, pos, initial_setting_flag);
    } else if (eventType == 0xB9 || eventType == 0xC9) {
        handleChannel10(data, pos, eventType);
    } else if (eventType == 0x99) {
        handleNoteOn(data, pos, note_on_time, tpqn, midiFilePath);
    } else {
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
                hihatState = 0;
            } else if (note == 5) {
                hihatState = 1;
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
    int prevRight = 0, prevLeft = 0;
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

        if (inst1 == 7 || inst2 == 7) {
            e.rightHand = (inst1 == 7) ? 7 : inst2;
            e.leftHand = (inst1 == 7) ? inst2 : inst1;
        } else if (inst1 != 0 && inst2 != 0) {
            if (inst1 == 1 || inst2 == 1) {
                e.leftHand = (inst1 == 1) ? inst1 : inst2;
                e.rightHand = (inst1 == 1) ? inst2 : inst1;
            } else {
                e.rightHand = (inst1 == 2 || inst1 == 3 || inst1 == 6 || inst1 == 7) ? inst1 : inst2;
                e.leftHand = (e.rightHand == inst1) ? inst2 : inst1;
            }
        } else if (inst1 != 0) {
            if (inst1 == prevRight || inst1 == prevLeft) {
                if (e.time <= 0.1) {
                    e.rightHand = (inst1 == prevRight) ? inst1 : 0;
                    e.leftHand = (inst1 == prevLeft) ? inst1 : 0;
                } else {
                    double dMax = 0.754;
                    double dRight = 1.0, dLeft = 1.0; // simplified for this context
                    double tRight = prevRightHit * 1.38;
                    double tLeft = prevLeftHit * 1.38;
                    double rScore = (tRight/0.6) * (1 - std::min(dRight/dMax, 1.0));
                    double lScore = (tLeft/0.6) * (1 - std::min(dLeft/dMax, 1.0));
                    if (lScore <= rScore) e.rightHand = inst1;
                    else e.leftHand = inst1;
                }
            } else {
                if (inst1 == 2 || inst1 == 3 || inst1 == 6 || inst1 == 7) e.rightHand = inst1;
                else e.leftHand = inst1;
            }
        }
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

    // std::cout << "손 어사인 포함 변환 완료! 저장 위치 → " << outputFilename << "\n";
}

// 박자 단위 분할 및 마디 번호 부여 함수
void Functions::convertToMeasureFile(const std::string& inputFilename, const std::string& outputFilename) {
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
    const double EPS = 1e-6;
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

    output << measureNum+1 << "\t 0.600\t 0\t 0\t 0\t 0\t 0\t 0\n";
    output << "-1" << "\t 0.600\t 1\t 1\t 1\t 1\t 1\t 1\n";
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
        case 42: mappedDrumNote = 5; break;
        case 51: mappedDrumNote = 6; break;
        case 49: mappedDrumNote = 7; break;
        case 57: mappedDrumNote = 8; break;
        case 36: mappedDrumNote = 10; break;
        case 46: mappedDrumNote = 11; break;
        default: mappedDrumNote = 0; break;
    }
    file << note_on_time << "\t " << mappedDrumNote << "\n";
    file.close();
    note_on_time = 0;
}
