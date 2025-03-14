#include "../include/managers/CanManager.hpp"

// For Qt
// #include "../managers/CanManager.hpp"
CanManager::CanManager(std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef, Functions &funRef)
    : motors(motorsRef), fun(funRef)
{
}
//hi i am taehwang
CanManager::~CanManager()
{
    // 모든 소켓 닫기
    for (const auto &socketPair : sockets)
    {
        if (socketPair.second >= 0)
        {
            flushCanBuffer(socketPair.second);
            close(socketPair.second);
        }
    }
    sockets.clear();
}

////////////////////////////////////////////////////////////////////////////////////////////////
/*                                Settign Functions [Public]                                 */
//////////////////////////////////////////////////////////////////////////////////////////////

void CanManager::initializeCAN()
{
    listAndActivateAvailableCANPorts();
    for (const auto &ifname : this->ifnames)
    {
        std::cout << "Processing interface: " << ifname << std::endl;
        int hsocket = createSocket(ifname);
        if (hsocket < 0)
        {
            std::cerr << "Socket creation error for interface: " << ifname << std::endl;
        }
        sockets[ifname] = hsocket;
        isConnected[ifname] = true;
        std::cout << "Socket created for " << ifname << ": " << hsocket << std::endl;
    }
}

void CanManager::flushCanBuffer(int socket)
{
    // 버퍼 초기화 (필터 설정으로 모든 패킷 필터링)
    struct can_filter filter = {0};
    setsockopt(socket, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));
}

void CanManager::resetCanFilter(int socket)
{
    // 기본 상태로 CAN 필터 재설정
    setsockopt(socket, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
}

void CanManager::setSocketsTimeout(int sec, int usec)
{
    for (const auto &socketPair : sockets)
    {
        int socket_fd = socketPair.second;
        if (setSocketTimeout(socket_fd, sec, usec) != 0)
        {
            std::cerr << "Failed to set socket timeout for " << socketPair.first << std::endl;
        }
    }
}

void CanManager::checkCanPortsStatus()
{

    for (const auto &ifname : this->ifnames)
    {
        isConnected[ifname] = getCanPortStatus(ifname.c_str());

        if (!isConnected[ifname])
        {
            std::cout << "Port " << ifname << " is NOT CONNECTED" << std::endl;
        }
    }

    // 모든 포트가 연결된 경우 1, 아니면 0 반환
}

////////////////////////////////////////////////////////////////////////////////////////////////
/*                                Settign Functions [Private]                                 */
//////////////////////////////////////////////////////////////////////////////////////////////

bool CanManager::getCanPortStatus(const char *port)
{
    char command[50];
    snprintf(command, sizeof(command), "ip link show %s", port);

    FILE *fp = popen(command, "r");
    if (fp == NULL)
    {
        perror("Error opening pipe");
        return false;
    }

    char output[1024];
    while (fgets(output, sizeof(output) - 1, fp) != NULL)
    {
        if (strstr(output, "DOWN") || strstr(output, "does not exist"))
        {
            pclose(fp);
            return false;
        }
        else if (strstr(output, "UP"))
        {
            pclose(fp);
            return true;
        }
    }

    perror("fgets failed");
    printf("Errno: %d\n", errno); // errno 값을 출력
    pclose(fp);
    return false;
}

int CanManager::createSocket(const std::string &ifname)
{
    int result;
    struct sockaddr_can addr;
    struct ifreq ifr;

    int localSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW); // 지역 변수로 소켓 생성
    if (localSocket < 0)
    {
        return ERR_SOCKET_CREATE_FAILURE;
    }

    memset(&ifr, 0, sizeof(struct ifreq));
    memset(&addr, 0, sizeof(struct sockaddr_can));

    strcpy(ifr.ifr_name, ifname.c_str());
    result = ioctl(localSocket, SIOCGIFINDEX, &ifr);
    if (result < 0)
    {
        close(localSocket);
        return ERR_SOCKET_CREATE_FAILURE;
    }

    addr.can_ifindex = ifr.ifr_ifindex;
    addr.can_family = AF_CAN;

    if (bind(localSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        close(localSocket);
        return ERR_SOCKET_CREATE_FAILURE;
    }

    return localSocket; // 생성된 소켓 디스크립터 반환
}

void CanManager::activateCanPort(const char *port)
{
    char command1[100], command2[100], command3[100], command4[100];



    snprintf(command4, sizeof(command4), "sudo ip link set %s down", port);
    snprintf(command1, sizeof(command1), "sudo ip link set %s type can bitrate 1000000 restart-ms 100", port);
    snprintf(command2, sizeof(command2), "sudo ip link set %s up", port);
    snprintf(command3, sizeof(command3), "sudo ifconfig %s txqueuelen 1000", port);

    int ret4 = system(command4);
    int ret1 = system(command1);
    int ret2 = system(command2);    // UP PORT 해줄 때 잔여 명령 실행
    int ret3 = system(command3);

    
    if (ret1 != 0 || ret2 != 0 || ret3 != 0 || ret4 != 0)
    {
        fprintf(stderr, "Failed to activate port: %s\n", port);
    }

    sleep(2);
}

void CanManager::deactivateCanPort(const char *port)
{
    char command[100];
    snprintf(command, sizeof(command), "sudo ip link set %s down", port);

    int ret = system(command);

    if (ret != 0)
    {
        fprintf(stderr, "Failed to deactivate port: %s\n", port);
    }
}

void CanManager::deactivateAllCanPorts()
{
    FILE *fp = popen("ip link show | grep can", "r");
    if (fp == nullptr)
    {
        perror("No available CAN port");
        return;
    }

    char output[1024];
    while (fgets(output, sizeof(output) - 1, fp) != nullptr)
    {
        std::string line(output);
        std::istringstream iss(line);
        std::string skip, port;
        iss >> skip >> port;

        // 콜론 제거
        if (!port.empty() && port.back() == ':')
        {
            port.pop_back();
        }

        // 포트 이름이 유효한지 확인
        if (!port.empty() && port.find("can") == 0)
        {
            deactivateCanPort(port.c_str());
        }
    }

    if (feof(fp) == 0)
    {
        perror("fgets failed");
        printf("Errno: %d\n", errno);
    }

    pclose(fp);
}

void CanManager::listAndActivateAvailableCANPorts()
{
    int portCount = 0; // CAN 포트 수를 세기 위한 변수

    FILE *fp = popen("ip link show | grep can", "r");
    if (fp == nullptr)
    {
        perror("No available CAN port");
    }

    char output[1024];
    while (fgets(output, sizeof(output) - 1, fp) != nullptr)
    {
        std::string line(output);
        std::istringstream iss(line);
        std::string skip, port;
        iss >> skip >> port;

        // 콜론 제거
        if (!port.empty() && port.back() == ':')
        {
            port.pop_back();
        }

        // 포트 이름이 유효한지 확인
        if (!port.empty() && port.find("can") == 0)
        {
            portCount++;
            if (!getCanPortStatus(port.c_str()))
            {
                printf("%s is DOWN, activating...\n", port.c_str());
                activateCanPort(port.c_str());
            }
            else
            {
                printf("%s is already UP\n", port.c_str());
            }

            this->ifnames.push_back(port); // 포트 이름을 ifnames 벡터에 추가
        }
    }

    if (feof(fp) == 0)
    {
        perror("fgets failed");
        printf("Errno: %d\n", errno);
    }

    pclose(fp);

    if (portCount == 0)
    {
        printf("No CAN port found. Exiting...\n");
    }
}

void CanManager::clearReadBuffers()
{
    for (const auto &socketPair : sockets)
    {
        int socket_fd = socketPair.second;
        clearCanBuffer(socket_fd);
    }
}

void CanManager::clearCanBuffer(int canSocket)
{
    struct can_frame frame;
    fd_set readSet;
    struct timeval timeout;

    // 수신 대기 시간 설정
    timeout.tv_sec = 0;
    timeout.tv_usec = 0; // 즉시 반환

    while (true)
    {
        FD_ZERO(&readSet);
        FD_SET(canSocket, &readSet);

        // 소켓에서 읽을 데이터가 있는지 확인
        int selectRes = select(canSocket + 1, &readSet, NULL, NULL, &timeout);
        // std::cout << "clearcanbuf" << std::endl;
        if (selectRes > 0)
        {
            // 수신 버퍼에서 데이터 읽기
            ssize_t nbytes = read(canSocket, &frame, sizeof(struct can_frame));
            if (nbytes < 0)  // 읽기 실패
            {
                if (errno == EWOULDBLOCK || errno == EAGAIN)
                {
                    // non-blocking 모드에서 읽을 데이터 없음 -> 루프 종료
                    break;
                }
                else
                {
                    perror("read() failed while clearing CAN buffer");
                    break;
                }
            }
            else if (nbytes == 0)
            {
                // CAN 소켓이 닫힘 -> 루프 종료
                break;
            }
        }
        else
        {
            // 읽을 데이터 없음
            break;
        }
    }
}

int CanManager::setSocketTimeout(int socket, int sec, int usec)
{
    struct timeval timeout;
    timeout.tv_sec = sec;
    timeout.tv_usec = usec;

    if (setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0)
    {
        perror("setsockopt failed");
        return -1;
    }

    return 0;
}

void CanManager::setSocketNonBlock()
{
    for (auto &socket : sockets)
    {                                                 // sockets는 std::map<std::string, int> 타입
        int flags = fcntl(socket.second, F_GETFL, 0); // 현재 플래그를 가져옴
        if (flags < 0)
            continue;                                      // 에러 체크
        fcntl(socket.second, F_SETFL, flags | O_NONBLOCK); // 논블록 플래그 추가
    }
}

void CanManager::setSocketBlock()
{
    for (auto &socket : sockets)
    {
        int flags = fcntl(socket.second, F_GETFL, 0);
        if (flags < 0)
            continue;
        fcntl(socket.second, F_SETFL, flags & ~O_NONBLOCK); // 논블록 플래그 제거
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////
/*                                Utility Functions                                          */
//////////////////////////////////////////////////////////////////////////////////////////////

bool CanManager::txFrame(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame)
{
    if (write(motor->socket, &frame, sizeof(frame)) != sizeof(frame))
    {
        perror("CAN write error");
        return false;
    }
    return true;
}

bool CanManager::rxFrame(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame)
{

    if (read(motor->socket, &frame, sizeof(frame)) != sizeof(frame))
    {
        std::cout << "CAN read error: " << motor->myName << "\n";
        return false;
    }
    return true;
}

bool CanManager::sendAndRecv(std::shared_ptr<GenericMotor> &motor, struct can_frame &frame)
{
    if (!txFrame(motor, frame) || !rxFrame(motor, frame))
    {
        perror("Send and receive error");
        return false;
    }
    return true;
}

bool CanManager::sendFromBuff(std::shared_ptr<GenericMotor> &motor)
{
    std::cout << "Erase this function\n";
    return true;
}

bool CanManager::sendMotorFrame(std::shared_ptr<GenericMotor> motor)
{
    struct can_frame frame;

    if (write(motor->socket, &motor->sendFrame, sizeof(frame)) != sizeof(frame))
    {
        errorCnt++;
        if (errorCnt > 5)
        {
            deactivateAllCanPorts();
            std::cout << "Go to Error state by CAN write error " << motor->myName << "\n";
            return false;
        }
    }
    return true;
}

bool CanManager::recvToBuff(std::shared_ptr<GenericMotor> &motor, int readCount)
{
    struct can_frame frame;
    for (int i = 0; i < readCount; i++)
    {
        if (rxFrame(motor, frame))
        {
            motor->recieveBuffer.push(frame);
        }
        else
        {
            return false;
        }
    }
    return true;
}

void CanManager::setMotorsSocket()
{
    struct can_frame frame;
    setSocketsTimeout(0, 10000);
    clearReadBuffers();
    std::map<int, int> localMotorsPerSocket;

    // 모든 소켓에 대해 Maxon 모터에 명령을 보내고 응답을 확인
    for (const auto &socketPair : sockets)
    {
        int socket_fd = socketPair.second;

        for (auto &motor_pair : motors)
        {
            auto &motor = motor_pair.second;

            if (!motor->isConected)
            {
                motor->socket = socket_fd;
                if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
                {
                    maxoncmd.getCheck(*maxonMotor, &frame);
                    txFrame(motor, frame);
                }

                usleep(50000);
            }
        }

        // 해당 소켓으로 프레임을 무작정 읽어서 버퍼에 넣음
        int readCount = 0;
        while (readCount < 10)
        {
            ssize_t result = read(socket_fd, &frame, sizeof(frame));

            if (result > 0)
            {
                tempFrames[socket_fd].push_back(frame);
            }
            readCount++;
        }

        // 버퍼에서 하나씩 읽어옴
        for (auto &motor_pair : motors)
        {
            auto &motor = motor_pair.second;
            if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
            {
                for (auto &frame : tempFrames[motor->socket])
                {
                    if ((frame.can_id & 0xFF) == tMotor->nodeId)
                    {
                        motor->isConected = true;
                        std::tuple<int, float, float, float, int8_t, int8_t> parsedData = tservocmd.motor_receive(&frame);
                        motor->motorPosition = std::get<1>(parsedData);
                    }
                }
            }
            else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
            {
                for (auto &frame : tempFrames[motor->socket])
                {
                    if (frame.can_id == maxonMotor->canReceiveId)
                    {
                        motor->isConected = true;
                        maxonCnt++;
                    }
                }
            }
        }

        tempFrames.clear();
    }

    // 모든 소켓에 대한 검사가 완료된 후, 모터 연결 상태 확인 및 연결 안된 모터 삭제
    for (uint32_t i = 0; i < 12; i++)
    {
        for (auto it = motors.begin(); it != motors.end(); it++)
        {
            std::string name = it->first;
            std::shared_ptr<GenericMotor> motor = it->second;
            if (motor->nodeId == i)
            {
                if (motor->isConected)
                {
                    std::cerr << "--------------> CAN NODE ID " << motor->nodeId << " Connected. " << "Motor [" << name << "]\n";
                }
                else
                {
                    std::cerr << "CAN NODE ID " << motor->nodeId << " Not Connected. " << "Motor [" << name << "]\n";
                    it = motors.erase(it);
                }

                break;
            }
        }
    }
}

// void CanManager::readFramesFromAllSockets()
// {
//     struct can_frame frame;
//     for (const auto &socketPair : sockets)
//     {
//         int socket_fd = socketPair.second;
//         tempFrames[socket_fd].clear();
        
//         while (read(socket_fd, &frame, sizeof(frame)) == sizeof(frame))
//         {
//             if (tempFrames[socket_fd].empty()) {
//                 tempFrames[socket_fd].push_back(frame);
//             } else {
//                 tempFrames[socket_fd][0] = frame;
//             }
//         }
//     }
// }

// void CanManager::readFramesFromAllSockets()
// {
//     struct can_frame frame;
//     for (const auto &socketPair : sockets)
//     {
//         int socket_fd = socketPair.second;
//         while (read(socket_fd, &frame, sizeof(frame)) == sizeof(frame))
//         {
//             tempFrames[socket_fd].push_back(frame);
//         }
//     }
// }

void CanManager::readFramesFromAllSockets()
{
    struct can_frame frame;

    for (const auto &socketPair : sockets)
    {
        int socket_fd = socketPair.second;

        // Non-blocking 모드 설정
        fcntl(socket_fd, F_SETFL, O_NONBLOCK);

        while (true)
        {
            ssize_t bytesRead = read(socket_fd, &frame, sizeof(frame));

            if (bytesRead == sizeof(frame))
            {
                tempFrames[socket_fd].push_back(frame); // 정상적으로 읽었으면 저장
            }
            else if (bytesRead == -1 && errno == EWOULDBLOCK)
            {
                // 읽을 데이터가 없으면 루프 종료 (non-blocking에서는 EWOULDBLOCK 발생 가능)
                break;
            }
            else
            {
                // 기타 오류 발생 시 처리
                perror("read error");
                break;
            }
        }
    }
}

bool CanManager::distributeFramesToMotors(bool setlimit)
{
    for (auto &motor_pair : motors)
    {
        auto &motor = motor_pair.second;

        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
        {
            // TMotor 처리
            for (auto &frame : tempFrames[motor->socket])
            {
                if ((frame.can_id & 0xFF) == tMotor->nodeId)
                {
                    std::tuple<int, float, float, float, int8_t, int8_t> parsedData = tservocmd.motor_receive(&frame);
                    
                    tMotor->motorPosition = std::get<1>(parsedData);
                    tMotor->motorVelocity = std::get<2>(parsedData);
                    tMotor->motorCurrent = std::get<3>(parsedData);

                    // tMotor->jointAngle = std::get<1>(parsedData) * tMotor->cwDir * tMotor->timingBeltRatio + tMotor->initialJointAngle;
                    tMotor->jointAngle = tMotor->motorPositionToJointAngle(std::get<1>(parsedData));
                    
                    // std::cout << tMotor->jointAngle << std::endl;
                    tMotor->recieveBuffer.push(frame);

                    fun.appendToCSV_DATA(fun.file_name, (float)tMotor->nodeId, tMotor->motorPosition, tMotor->motorCurrent);
                    
                    if (setlimit)
                    {
                        bool isSafe = safetyCheck_T(motor);
                        if (!isSafe)
                        {
                            return false;
                        }
                    }
                }
            }
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            // MaxonMotor 처리
            for (auto &frame : tempFrames[motor->socket])
            {
                // // Maxon모터에서 1ms마다 SDO 응답 확인
                // if (frame.can_id == (0x580 + maxonMotor->nodeId)) {
                //     if (frame.data[1] == 0x64 && frame.data[2] == 0x60 && frame.data[3] == 0x00) { 
                //         int32_t pos_enc = frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) | (frame.data[7] << 24);

                //         float pos_degrees = (static_cast<float>(pos_enc) * 360.0f) / (35.0f * 4096.0f);
                //         float pos_radians = pos_degrees * (M_PI / 180.0f);  
                //         maxonMotor->motorPosition = pos_radians;

                //         fun.appendToCSV_DATA("q8손목", (float)maxonMotor->nodeId, maxonMotor->motorPosition, 0);
                //         currentPosition = maxonMotor->motorPosition; // 현재 위치 값 해당 변수에 덮어쓰기
                //     }
                // }

                if (frame.can_id == maxonMotor->rxPdoIds[0])
                {
                    // getCheck(*maxonMotor ,&frame);
                    std::tuple<int, float, float, unsigned char> parsedData = maxoncmd.parseRecieveCommand(*maxonMotor, &frame);
                    
                    maxonMotor->motorPosition = std::get<1>(parsedData);
                    maxonMotor->motorTorque = std::get<2>(parsedData);
                    maxonMotor->statusBit = std::get<3>(parsedData);
                    maxonMotor->positionValues.push(std::get<1>(parsedData));
                    if ((maxonMotor->positionValues).size() >= maxonMotor->maxIndex)
                    {
                        maxonMotor->positionValues.pop();
                    }

                    // maxonMotor->jointAngle = std::get<1>(parsedData) * maxonMotor->cwDir + maxonMotor->initialJointAngle;
                    maxonMotor->jointAngle = maxonMotor->motorPositionToJointAngle(std::get<1>(parsedData));
                    maxonMotor->recieveBuffer.push(frame);
                    
                    fun.appendToCSV_DATA(fun.file_name, (float)maxonMotor->nodeId, maxonMotor->motorPosition, maxonMotor->motorTorque);

                    // fun.appendToCSV_DATA("손목데이터", (float)maxonMotor->nodeId, maxonMotor->motorPosition, maxonMotor->motorTorque);


                    if (setlimit)
                    {
                        bool isSafe = safetyCheck_M(motor);
                        if (!isSafe)
                        {
                            return false;
                        }
                    }
                }
            }
        }
    }
    tempFrames.clear(); // 프레임 분배 후 임시 배열 비우기

    return true;
}

bool CanManager::sendForCheck_Fixed(std::shared_ptr<GenericMotor> motor)
{
    clearReadBuffers();

    if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
    {
        if (motor->isfixed == false)
        {
            motor->fixedMotorPosition = tMotor->motorPosition;
            motor->isfixed = true;
        }

        fun.appendToCSV_DATA(fun.file_name, (float)tMotor->nodeId + SEND_SIGN, motor->fixedMotorPosition,  motor->fixedMotorPosition - tMotor->motorPosition);

        // safety check
        float diff_angle = motor->fixedMotorPosition - tMotor->motorPosition;
        if (abs(diff_angle) > POS_DIFF_LIMIT)
        {
            std::cout << "\nMotor Fixed Error : Safety Check (" << tMotor->myName << ")\n";
            std::cout << "Fixed Motor Position : " << motor->fixedMotorPosition << "rad\tCurrent Motor Position : " << tMotor->motorPosition << "rad\n";
            return false;
        }
        tservocmd.comm_can_set_pos(*tMotor, &tMotor->sendFrame, motor->fixedMotorPosition);

        if (!sendMotorFrame(tMotor))
        {
            return false;
        };
    }
    else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
    {
        if (motor->isfixed == false)
        {
            motor->fixedMotorPosition = maxonMotor->motorPosition;
            motor->isfixed = true;
        }
        maxoncmd.getTargetPosition(*maxonMotor, &maxonMotor->sendFrame, maxonMotor->fixedMotorPosition);

        if (!sendMotorFrame(maxonMotor))
        {
            return false;
        };

        maxoncmd.getSync(&maxonMotor->sendFrame);
        if (!sendMotorFrame(maxonMotor))
        {
            return false;
        };
        
        //maxoncmd.getTargetPosition(*maxonMotor, &maxonMotor->sendFrame, maxonMotor->fixedMotorPosition);

        fun.appendToCSV_DATA(fun.file_name, (float)maxonMotor->nodeId + SEND_SIGN, maxonMotor->fixedMotorPosition, maxonMotor->fixedMotorPosition - maxonMotor->motorPosition);
    }
    return true;
}

bool CanManager::checkMaxon()
{
    for (auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        auto &motor = motorPair.second;

        struct can_frame frame;
        clearReadBuffers();

        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            maxoncmd.getSync(&frame);
            txFrame(motor, frame);
        }
    }
    return true;
}

bool CanManager::checkAllMotors_Fixed()
{
    for (auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        auto &motor = motorPair.second;
        if (!motor->isError)
        {
            if (!sendForCheck_Fixed(motor))
            {
                return false;
            }
        }
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////
/*                                Functions for Thread Case                                      */
//////////////////////////////////////////////////////////////////////////////////////////////

bool CanManager::setCANFrame()
{
    static bool CSTModeR = false;
    static bool CSTModeL = false;

    vector<float> Pos(9);
    for (auto &motor_pair : motors)
    {
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
        {
            shared_ptr<GenericMotor> motor = motor_pair.second;

            MaxonData mData = maxonMotor->commandBuffer.front();
            float desiredPosition = maxonMotor->jointAngleToMotorPosition(mData.position);
            float desiredTorque = -400;
            
            maxonMotor->commandBuffer.pop();

            if (maxonMotor->myName == "R_wrist")
            {
                if (isCSTR && !CSTModeR)
                {
                    maxoncmd.getCSTMode(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);

                    maxoncmd.getShutdown(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);

                    maxoncmd.getEnable(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);
                    
                    CSTModeR = true;
                }
                else if (!isCSTR && CSTModeR)
                {
                    maxoncmd.getCSPMode(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);

                    maxoncmd.getShutdown(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);

                    maxoncmd.getEnable(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);
                    CSTModeR = false;
                }

                if(dct_fun(maxonMotor) && isHitR && maxonMotor->hitting == false)
                {
                    fun.appendToCSV_DATA("hittingDetectR", 1, 0, 0);

                    maxonMotor->hitting = true;
                    if (isHitR) isHitR = false;
                    maxonMotor->hittingPos = maxonMotor->positionValues.back() + maxonMotor->initialJointAngle - maxonMotor->hittingDrumAngle;
                    desiredPosition = maxonMotor->positionValues.back();
                    if (isCSTR) isCSTR = false;
                }
                else
                {
                    fun.appendToCSV_DATA("hittingDetectR", 0, 0, 0);
                }

                if(isCSTR && CSTModeR)
                {
                    maxoncmd.getTargetTorque(*maxonMotor, &maxonMotor->sendFrame, desiredTorque);
                }
                else
                {
                    maxoncmd.getTargetPosition(*maxonMotor, &maxonMotor->sendFrame, desiredPosition);
                }
            }
            else if (maxonMotor->myName == "L_wrist")
            {
                if (isCSTL && !CSTModeL)
                {
                    maxoncmd.getCSTMode(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);

                    maxoncmd.getShutdown(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);

                    maxoncmd.getEnable(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);
                    
                    CSTModeL = true;
                }
                else if (!isCSTL && CSTModeL)
                {
                    maxoncmd.getCSPMode(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);

                    maxoncmd.getShutdown(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);

                    maxoncmd.getEnable(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);
                    CSTModeL = false;
                }

                if(dct_fun(maxonMotor) && isHitL && maxonMotor->hitting == false)
                {
                    fun.appendToCSV_DATA("hittingDetectL", 1, 0, 0);
                    
                    maxonMotor->hitting = true;
                    if (isHitL) isHitL = false;
                    maxonMotor->hittingPos = maxonMotor->positionValues.back() + maxonMotor->initialJointAngle - maxonMotor->hittingDrumAngle;
                    desiredPosition = maxonMotor->positionValues.back();
                }
                else
                {
                    fun.appendToCSV_DATA("hittingDetectL", 0, 0, 0);
                }

                if(isCSTL && CSTModeL)
                {
                    maxoncmd.getTargetTorque(*maxonMotor, &maxonMotor->sendFrame, desiredTorque);
                }
                else
                {
                    maxoncmd.getTargetPosition(*maxonMotor, &maxonMotor->sendFrame, desiredPosition);
                }
            }

            fun.appendToCSV_DATA(fun.file_name, (float)maxonMotor->nodeId + SEND_SIGN, desiredPosition, desiredPosition - maxonMotor->motorPosition);
        }
        else if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
        {
            TMotorData tData = tMotor->commandBuffer.front();
            float desiredPosition;

            tMotor->commandBuffer.pop();
            // desiredPosition = (tData.position - tMotor->initialJointAngle) * tMotor->cwDir / tMotor->timingBeltRatio;
            desiredPosition = tMotor->jointAngleToMotorPosition(tData.position);

            if(!safetyCheck_Tmotor(tMotor, tData))
            {
                return false;
            }
            tservocmd.comm_can_set_pos(*tMotor, &tMotor->sendFrame, desiredPosition);
            tMotor->brakeState = tData.isBrake;

            fun.appendToCSV_DATA(fun.file_name, (float)tMotor->nodeId + SEND_SIGN, desiredPosition, desiredPosition - tMotor->motorPosition);
        }
    }

    return true;
}

bool CanManager::setCANFrame_TEST()
{
    static bool CSTModeR = false;
    static bool CSTModeL = false;
    static bool drumReachedR = false;
    static bool drumReachedL = false;
    float actualStayAngleR;
    float actualStayAngleL;

    vector<float> Pos(9);
    for (auto &motor_pair : motors)
    {
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
        {
            shared_ptr<GenericMotor> motor = motor_pair.second;

            MaxonData mData = maxonMotor->commandBuffer.front();
            float desiredPosition = maxonMotor->jointAngleToMotorPosition(mData.position);
            float desiredTorque = -500;
            
            maxonMotor->commandBuffer.pop();

            if (maxonMotor->myName == "R_wrist")
            {
                if (isCSTR && !CSTModeR)
                {
                    maxoncmd.getCSTMode(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);

                    maxoncmd.getShutdown(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);

                    maxoncmd.getEnable(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);
                    
                    CSTModeR = true;
                }
                else if (!isCSTR && CSTModeR)
                {
                    maxoncmd.getCSPMode(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);

                    maxoncmd.getShutdown(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);

                    maxoncmd.getEnable(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);
                    CSTModeR = false;
                }

                if(dct_fun(maxonMotor) && isHitR && maxonMotor->hitting == false)
                {
                    fun.appendToCSV_DATA("hittingDetectR", 1, 0, 0);
                    
                    if(isCSTR && CSTModeR && torqueOff)
                    {
                        drumReachedR = true;
                    }
                    else 
                    {
                        maxonMotor->hitting = true;
                        if (isHitR) isHitR = false;
                        maxonMotor->hittingPos = maxonMotor->positionValues.back() + maxonMotor->initialJointAngle - maxonMotor->hittingDrumAngle;
                        desiredPosition = maxonMotor->positionValues.back();
                    }
                }
                else
                {
                    fun.appendToCSV_DATA("hittingDetectR", 0, 0, 0);
                }

                if(isCSTR && CSTModeR)
                {
                    actualStayAngleR = wristStayAngle - maxonMotor->initialJointAngle + maxonMotor -> hittingDrumAngle;
                    if(maxonMotor->positionValues.back() <= actualStayAngleR && !drumReachedR)
                    {
                        maxoncmd.getTargetTorque(*maxonMotor, &maxonMotor->sendFrame, backTorque);
                    }
                    else if (maxonMotor->positionValues.back() >= (actualStayAngleR - wristStayAngle / 2) && drumReachedR)
                    {
                        maxonMotor->hitting = true;
                        if (isHitR) isHitR = false;
                        maxonMotor->hittingPos = maxonMotor->positionValues.back() + maxonMotor->initialJointAngle - maxonMotor->hittingDrumAngle;
                        desiredPosition = maxonMotor->positionValues.back();
                        drumReachedR = false;
                        maxoncmd.getTargetTorque(*maxonMotor, &maxonMotor->sendFrame, backTorque);
                    }
                    else if (maxonMotor->positionValues.back() < (actualStayAngleR - wristStayAngle / 2) && drumReachedR)
                    {
                        maxoncmd.getTargetTorque(*maxonMotor, &maxonMotor->sendFrame, backTorque);
                    }
                    else
                    {
                        maxoncmd.getTargetTorque(*maxonMotor, &maxonMotor->sendFrame, desiredTorque);
                    }
                }
                else
                {
                    maxoncmd.getTargetPosition(*maxonMotor, &maxonMotor->sendFrame, desiredPosition);
                }
            }
            else if (maxonMotor->myName == "L_wrist")
            {
                if (isCSTL && !CSTModeL)
                {
                    maxoncmd.getCSTMode(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);

                    maxoncmd.getShutdown(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);

                    maxoncmd.getEnable(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);
                    
                    CSTModeL = true;
                }
                else if (!isCSTL && CSTModeL)
                {
                    maxoncmd.getCSPMode(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);

                    maxoncmd.getShutdown(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);

                    maxoncmd.getEnable(*maxonMotor, &maxonMotor->sendFrame);
                    sendMotorFrame(maxonMotor);
                    CSTModeL = false;
                }

                if(dct_fun(maxonMotor) && isHitL && maxonMotor->hitting == false)
                {
                    fun.appendToCSV_DATA("hittingDetectL", 1, 0, 0);
                    
                    if(isCSTL && CSTModeL && torqueOff)
                    {
                        drumReachedL = true;
                    }
                    else 
                    {
                        maxonMotor->hitting = true;
                        if (isHitL) isHitL = false;
                        maxonMotor->hittingPos = maxonMotor->positionValues.back() + maxonMotor->initialJointAngle - maxonMotor->hittingDrumAngle;
                        desiredPosition = maxonMotor->positionValues.back();
                    }
                }
                else
                {
                    fun.appendToCSV_DATA("hittingDetectL", 0, 0, 0);
                }

                if(isCSTL && CSTModeL)
                {
                    actualStayAngleL = wristStayAngle - maxonMotor->initialJointAngle + maxonMotor -> hittingDrumAngle;
                    if(maxonMotor->positionValues.back() <= actualStayAngleL && !drumReachedL)
                    {
                        maxoncmd.getTargetTorque(*maxonMotor, &maxonMotor->sendFrame, backTorque);
                    }
                    else if (maxonMotor->positionValues.back() >= (actualStayAngleL - wristStayAngle / 2) && drumReachedL)
                    {
                        maxonMotor->hitting = true;
                        if (isHitL) isHitL = false;
                        maxonMotor->hittingPos = maxonMotor->positionValues.back() + maxonMotor->initialJointAngle - maxonMotor->hittingDrumAngle;
                        desiredPosition = maxonMotor->positionValues.back();
                        drumReachedL = false;
                        maxoncmd.getTargetTorque(*maxonMotor, &maxonMotor->sendFrame, backTorque);
                    }
                    else if (maxonMotor->positionValues.back() < (actualStayAngleL - wristStayAngle / 2) && drumReachedL)
                    {
                        maxoncmd.getTargetTorque(*maxonMotor, &maxonMotor->sendFrame, backTorque);
                    }
                    else
                    {
                        maxoncmd.getTargetTorque(*maxonMotor, &maxonMotor->sendFrame, desiredTorque);
                    }
                }
                else
                {
                    maxoncmd.getTargetPosition(*maxonMotor, &maxonMotor->sendFrame, desiredPosition);
                }
            }
            fun.appendToCSV_DATA(fun.file_name, (float)maxonMotor->nodeId + SEND_SIGN, desiredPosition, desiredPosition - maxonMotor->motorPosition);
        }
        else if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
        {
            TMotorData tData = tMotor->commandBuffer.front();
            float desiredPosition;

            tMotor->commandBuffer.pop();
            // desiredPosition = (tData.position - tMotor->initialJointAngle) * tMotor->cwDir / tMotor->timingBeltRatio;
            desiredPosition = tMotor->jointAngleToMotorPosition(tData.position);

            if(!safetyCheck_Tmotor(tMotor, tData))
            {
                return false;
            }
            tservocmd.comm_can_set_pos(*tMotor, &tMotor->sendFrame, desiredPosition);
            tMotor->brakeState = tData.isBrake;

            fun.appendToCSV_DATA(fun.file_name, (float)tMotor->nodeId + SEND_SIGN, desiredPosition, desiredPosition - tMotor->motorPosition);
        }
    }

    return true;
}

bool CanManager::safetyCheck_Tmotor(std::shared_ptr<TMotor> tMotor, TMotorData tData)
{
    bool isSafe = true;
    float diff_angle = tData.position - tMotor->jointAngle;

    if (abs(diff_angle) > POS_DIFF_LIMIT)
    {
        std::cout << "\nSet CAN Frame Error : Safety Check (" << tMotor->myName << ")" << endl;
        std::cout << "Desired Joint Angle : " << tData.position * 180.0 / M_PI << "deg\tCurrent Joint Angle : " << tMotor->jointAngle * 180.0 / M_PI << "deg\n";
        isSafe = false;
    }
    else if (tMotor->rMin > tData.position)
    {
        std::cout << "\nSet CAN Frame Error : Out of Min Range (" << tMotor->myName << ")" << endl;
        std::cout << "Desired Joint Angle : " << tData.position * 180.0 / M_PI << "deg\n";
        isSafe = false;
    }
    else if (tMotor->rMax < tData.position)
    {
        std::cout << "\nSet CAN Frame Error : Out of Max Range (" << tMotor->myName << ")" << endl;
        std::cout << "Desired Joint Angle : " << tData.position * 180.0 / M_PI << "deg\n";
        isSafe = false;
    }

    return isSafe;
}

bool CanManager::safetyCheck_T(std::shared_ptr<GenericMotor> &motor)
{
    bool isSafe = true;

    if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
    {
        if (tMotor->rMin > tMotor->jointAngle || tMotor->rMax < tMotor->jointAngle)
        {
            if (tMotor->rMin > tMotor->jointAngle)
            {
                std::cout << "\nRead CAN Frame Error : Out of Min Range (" << tMotor->myName << ")" << endl;
                std::cout << "Current Joint Angle : " << tMotor->jointAngle / M_PI * 180 << "deg\n";
            }
            else
            {
                std::cout << "\nRead CAN Frame Error : Out of Max Range (" << tMotor->myName << ")" << endl;
                std::cout << "Current Joint Angle : " << tMotor->jointAngle / M_PI * 180 << "deg\n";
            }

            isSafe = false;
            tMotor->isError = true;
        }
        else if (tMotor->motorCurrent > tMotor->currentLimit)
        {
            if (tMotor->currentErrorCnt > 10)
            {
                std::cout << "\nRead CAN Frame Error : Overcurrent (" << tMotor->myName << ")" << endl;
                std::cout << "current : " << tMotor->motorCurrent << "A\n";

                isSafe = false;
                tMotor->isError = true;
            }
            else
            {
                tMotor->currentErrorCnt++;
            }
        }
        else
        {
            tMotor->currentErrorCnt = 0;
        }
    }

    return isSafe;
}

bool CanManager::safetyCheck_M(std::shared_ptr<GenericMotor> &motor)
{
    bool isSafe = true;

    if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
    {
        if (maxonMotor->rMin > maxonMotor->jointAngle || maxonMotor->rMax < maxonMotor->jointAngle)
        {
            if (maxonMotor->rMin > maxonMotor->jointAngle)
            {
                std::cout << "\nRead CAN Frame Error : Out of Min Range (" << maxonMotor->myName << ")" << endl;
                std::cout << "Current Joint Angle : " << maxonMotor->jointAngle / M_PI * 180 << "deg\n";
            }
            else
            {
                std::cout << "\nRead CAN Frame Error : Out of Max Range (" << maxonMotor->myName << ")" << endl;
                std::cout << "Current Joint Angle : " << maxonMotor->jointAngle / M_PI * 180 << "deg\n";
            }

            maxoncmd.getQuickStop(*maxonMotor, &maxonMotor->sendFrame);
            sendMotorFrame(maxonMotor);
            usleep(10);
            maxoncmd.getSync(&maxonMotor->sendFrame);
            sendMotorFrame(maxonMotor);
        }
    }

    return isSafe;
}

bool CanManager::dct_fun(shared_ptr<MaxonMotor> maxonMotor)
{
    queue<double> positions = maxonMotor->positionValues;
    float hittingDrumAngle = maxonMotor->hittingDrumAngle;

    double the_k_3; // 가장 오래된 값
    double the_k_2;
    double the_k_1;
    double the_k;
    double threshold = hittingDrumAngle + wristStayAngle - maxonMotor->initialJointAngle; // 90 deg + 드럼 별 타격 각도 + 준비 각도

    if (positions.size() < 4)
    {
        return false;
    }
    else
    {
        the_k_3 = positions.front(); // 가장 오래된 값
        positions.pop();
        the_k_2 = positions.front();
        positions.pop();
        the_k_1 = positions.front();
        positions.pop();
        the_k = positions.front();
    }

    double vel_k = the_k - the_k_1;
    double vel_k_1 = the_k_1 - the_k_2;

    if (((vel_k > 0 && vel_k_1 < 0) || (abs(vel_k) * 2 < abs(vel_k_1))) && abs(vel_k_1) >= 0.01 && the_k <= threshold)
    {
        return true;
    }
    else
        return false;

    // if (abs(vel_k) * 2 < abs(vel_k_1) && vel_k_1 < 0 && abs(vel_k_1) >= 0.01 && the_k <= threshold)
    // {
    //     return true;
    // }
    // else
    //     return false;
}