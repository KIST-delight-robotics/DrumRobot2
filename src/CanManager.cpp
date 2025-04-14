#include "../include/managers/CanManager.hpp"

// For Qt
// #include "../managers/CanManager.hpp"
CanManager::CanManager(std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef, Functions &funRef, USBIO &usbioRef)
    : motors(motorsRef), fun(funRef), usbio(usbioRef)
{
}

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

////////////////////////////////////////////////////////////////////////////////
/*                          Initialize Functions                              */
////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////
/*                            Setting Functions                               */
////////////////////////////////////////////////////////////////////////////////

void CanManager::flushCanBuffer(int socket)
{
    // 버퍼 초기화 (필터 설정으로 모든 패킷 필터링)
    struct can_filter filter = {0};
    setsockopt(socket, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));
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

bool CanManager::setMotorsSocket()
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

    // 모터 없이 테스트하는 경우
    bool allMotorsUnConected = true;

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
                    allMotorsUnConected = false;
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

    return allMotorsUnConected;
}

////////////////////////////////////////////////////////////////////////////////
/*                             Send Functions                                 */
////////////////////////////////////////////////////////////////////////////////

void CanManager::setMaxonCANFrame(std::shared_ptr<MaxonMotor> maxonMotor, const MaxonData &mData)
{
    if (mData.mode == maxonMotor->CSP)
    {
        if (maxonMotor->controlMode != maxonMotor->CSP)
        {
            //멕슨모터 모드 변경시에는 모드 변경해주고 shutdown -> enable 해주기
            maxoncmd.getCSPMode(*maxonMotor, &maxonMotor->sendFrame);
            sendMotorFrame(maxonMotor);
            maxoncmd.getShutdown(*maxonMotor, &maxonMotor->sendFrame);
            sendMotorFrame(maxonMotor);
            maxoncmd.getEnable(*maxonMotor, &maxonMotor->sendFrame);
            sendMotorFrame(maxonMotor);

            maxonMotor->controlMode = maxonMotor->CSP;
        }
        maxoncmd.setPositionCANFrame(*maxonMotor, &maxonMotor->sendFrame, mData.position);

        // std::cout << "Maxon Motor : " << maxonMotor->myName << "\t" << mData.position << "\n";
        fun.appendToCSV_DATA(fun.file_name, (float)maxonMotor->nodeId + SEND_SIGN, mData.position, mData.position - maxonMotor->motorPosition);
    }
    else if (mData.mode == maxonMotor->CST)
    {
        if (maxonMotor->controlMode != maxonMotor->CST)
        {
            //멕슨모터 모드 변경시에는 모드 변경해주고 shutdown -> enable 해주기
            maxoncmd.getCSTMode(*maxonMotor, &maxonMotor->sendFrame);
            sendMotorFrame(maxonMotor);
            maxoncmd.getShutdown(*maxonMotor, &maxonMotor->sendFrame);
            sendMotorFrame(maxonMotor);
            maxoncmd.getEnable(*maxonMotor, &maxonMotor->sendFrame);
            sendMotorFrame(maxonMotor);

            maxonMotor->controlMode = maxonMotor->CST;
        }

        float targetTorquemNm = calTorque(maxonMotor,mData);
        int targetTorque = maxoncmd.setTorqueCANFrame(*maxonMotor, &maxonMotor->sendFrame, targetTorquemNm);

        fun.appendToCSV_DATA(fun.file_name, (float)maxonMotor->nodeId + SEND_SIGN, mData.position, targetTorque);
        fun.appendToCSV_DATA_absTime("wristTime", (float)maxonMotor->nodeId, mData.position, 0);
        // fun.appendToCSV_DATA("torque input", (float)maxonMotor->nodeId, targetTorquemNm, targetTorque);
    }
    else if (mData.mode == maxonMotor->CSV)
    {
        return;
    }
}

float CanManager::calTorque(std::shared_ptr<MaxonMotor> maxonMotor, const MaxonData &mData)
{       
        double frictionTorque = 0;

        if((mData.position - maxonMotor -> preMotorPosition)>(0.01*M_PI/180))
        {
            frictionTorque = 5;
            maxonMotor -> preMotorPosition = mData.position;
        }
        else if((mData.position - maxonMotor -> preMotorPosition)<(0.01*M_PI/180))
        {
            frictionTorque = -5;
            maxonMotor -> preMotorPosition = mData.position;
        }
        else
        {
            maxonMotor -> preMotorPosition = mData.position;
        }

        double err = mData.position - maxonMotor->motorPosition;
        double err_dot = (err - maxonMotor-> pre_err)/DTSECOND;
        double alpha = 0.2;  // 적절한 필터 계수

        double err_dot_filtered = alpha * ((err - maxonMotor-> pre_err) / DTSECOND) + (1 - alpha) * err_dot;
        float torquemNm = mData.kp * err + mData.kd * err_dot_filtered;

        // if (maxonMotor -> myName == "R_wrist")
        // {
        //     if (mData.isHitR)
        //     {
        //         if (torquemNm > 0)
        //         {
        //             torquemNm = maxonMotor->prevTorque; 
        //         }
        //         else if (torquemNm < 0)
        //         {
        //             maxonMotor -> prevTorque = torquemNm;
        //         }
        //     }
        // }
        
        // else if (maxonMotor -> myName == "L_wrist")
        // {
        //     if (mData.isHitL)
        //     {
        //         if (torquemNm > 0)
        //         {
        //             torquemNm = maxonMotor->prevTorque; 
        //         }
        //         else if (torquemNm < 0)
        //         {
        //             maxonMotor -> prevTorque = torquemNm;
        //         }
        //     }
        // }

        //fun.appendToCSV_DATA("isHitRL", (float)maxonMotor->nodeId, mData.isHitR, mData.isHitL);

        maxonMotor-> pre_err = err;

        //여기에서 보상을 해주고!!
        // 무게 중력 거리
        float gearRatio = 35.0;
        float stickLengthMeter = 0.17;
        float stickMassKg = 0.47;
        float div = 10.0;
        float gravity_angle = 0;
        for (auto &motor_pair : motors)
        {
            std::shared_ptr<GenericMotor> motor = motor_pair.second;
            std::string motorName = motor_pair.first;
            
            // TMotor
            if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
            {
                if (maxonMotor -> myName == "R_wrist")
                {
                    if(tMotor -> myName == "R_arm2" || tMotor -> myName == "R_arm3" )
                    {
                        gravity_angle += tMotor -> jointAngle;
                    }
                }
                else if (maxonMotor -> myName == "L_wrist")
                {
                    if(tMotor -> myName == "L_arm2" || tMotor -> myName == "L_arm3" )
                    {
                        gravity_angle += tMotor -> jointAngle;
                    }
                }
            }

        }

        
        gravity_angle += maxonMotor -> jointAngle;

        float gravityTorqueNm =  stickMassKg * 9.81 * stickLengthMeter * std::sin(gravity_angle) / gearRatio / div;

        torquemNm += gravityTorqueNm * 1000.0;  // N·m -> mN·m
        // torquemNm += frictionTorque; 

        return torquemNm;
}

void CanManager::setTMotorCANFrame(std::shared_ptr<TMotor> tMotor, const TMotorData &tData)
{
    if (tData.mode == tMotor->Position)
    {
        tservocmd.setPositionCANFrame(*tMotor, &tMotor->sendFrame, tData.position);

        fun.appendToCSV_DATA(fun.file_name, (float)tMotor->nodeId + SEND_SIGN, tData.position, tData.position - tMotor->motorPosition);
    }
    else if (tData.mode == tMotor->Idle)
    {
        return;
    }
}

bool CanManager::safetyCheckSendT(std::shared_ptr<TMotor> tMotor, TMotorData tData)
{
    bool isSafe = true;
    float desiredJointAngle = tMotor->motorPositionToJointAngle(tData.position);
    float diff_angle = desiredJointAngle - tMotor->jointAngle;

    if (abs(diff_angle) > POS_DIFF_LIMIT)
    {
        std::cout << "\nSet CAN Frame Error : Safety Check (" << tMotor->myName << ")" << endl;
        std::cout << "Desired Joint Angle : " << desiredJointAngle * 180.0 / M_PI << "deg\tCurrent Joint Angle : " << tMotor->jointAngle * 180.0 / M_PI << "deg\n";
        isSafe = false;
    }
    else if (tMotor->rMin > desiredJointAngle)
    {
        std::cout << "\nSet CAN Frame Error : Out of Min Range (" << tMotor->myName << ")" << endl;
        std::cout << "Desired Joint Angle : " << desiredJointAngle * 180.0 / M_PI << "deg\n";
        isSafe = false;
    }
    else if (tMotor->rMax < desiredJointAngle)
    {
        std::cout << "\nSet CAN Frame Error : Out of Max Range (" << tMotor->myName << ")" << endl;
        std::cout << "Desired Joint Angle : " << desiredJointAngle * 180.0 / M_PI << "deg\n";
        isSafe = false;
    }

    return isSafe;
}

bool CanManager::setCANFrame(std::map<std::string, bool>& fixFlags, int cycleCounter)
{
    for (auto &motor_pair : motors)
    {
        std::shared_ptr<GenericMotor> motor = motor_pair.second;
        std::string motorName = motor_pair.first;
        
        // TMotor
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
        {
            if (cycleCounter == 0)
            {
                if (tMotor->commandBuffer.empty()) continue;

                TMotorData tData = tMotor->commandBuffer.front();

                //버퍼 크기가 1이면 fixed 상태
                fixFlags[motorName] = (tMotor->commandBuffer.size() == 1);

                if (tMotor->commandBuffer.size() > 1)
                {
                    fixFlags[motorName] = false;
                    tMotor->commandBuffer.pop();
                }

                //isbrake가 1이면 브레이크 켜줌 0이면 꺼줌
                usbio.setUSBIO4761(0, tData.is_brake == 1); //세팅
                usbio.outputUSBIO4761();                    //실행

                setTMotorCANFrame(tMotor, tData);
                if(!safetyCheckSendT(tMotor, tData))
                {
                    return false;
                }
            }
        }
        // MaxonMotor
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            if (maxonMotor->commandBuffer.empty()) continue;

            MaxonData mData = maxonMotor->commandBuffer.front();

            // 버퍼 크기가 1이면 fixed 상태
            fixFlags[motorName] = (maxonMotor->commandBuffer.size() == 1);

            if (maxonMotor->commandBuffer.size() > 1)
            {
                fixFlags[motorName] = false;
                maxonMotor->commandBuffer.pop();
            }

            setMaxonCANFrame(maxonMotor, mData);
            
        }
    }
    return true;
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

bool CanManager::sendMotorFrame(std::shared_ptr<GenericMotor> motor)
{
    if (write(motor->socket, &motor->sendFrame, sizeof(motor->sendFrame)) != sizeof(motor->sendFrame))
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

////////////////////////////////////////////////////////////////////////////////
/*                            Receive Functions                               */
////////////////////////////////////////////////////////////////////////////////

bool CanManager::safetyCheckRecvT(std::shared_ptr<GenericMotor> &motor)
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
                std::cout << "Current Motor Position : " << tMotor->motorPosition / M_PI * 180 << "deg\n";
            }
            else
            {
                std::cout << "\nRead CAN Frame Error : Out of Max Range (" << tMotor->myName << ")" << endl;
                std::cout << "Current Joint Angle : " << tMotor->jointAngle / M_PI * 180 << "deg\n";
                std::cout << "Current Motor Position : " << tMotor->motorPosition / M_PI * 180 << "deg\n";
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

bool CanManager::safetyCheckRecvM(std::shared_ptr<GenericMotor> &motor)
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

                    tMotor->jointAngle = tMotor->motorPositionToJointAngle(std::get<1>(parsedData));
                    
                    // std::cout << tMotor->jointAngle << std::endl;
                    tMotor->recieveBuffer.push(frame);

                    fun.appendToCSV_DATA(fun.file_name, (float)tMotor->nodeId, tMotor->motorPosition, tMotor->motorCurrent);
                    
                    if (setlimit)
                    {
                        bool isSafe = safetyCheckRecvT(motor);
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

                    maxonMotor->jointAngle = maxonMotor->motorPositionToJointAngle(std::get<1>(parsedData));
                    maxonMotor->recieveBuffer.push(frame);
                    
                    fun.appendToCSV_DATA(fun.file_name, (float)maxonMotor->nodeId, maxonMotor->motorPosition, maxonMotor->motorTorque);

                    if (setlimit)
                    {
                        bool isSafe = safetyCheckRecvM(motor);
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

////////////////////////////////////////////////////////////////////////////////
/*                              Hit Detection                                 */
////////////////////////////////////////////////////////////////////////////////

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

    // if (((vel_k > 0 && vel_k_1 < 0) || (abs(vel_k) * 2 < abs(vel_k_1))) && abs(vel_k_1) >= 0.01 && the_k <= threshold)
    // {
    //     return true;
    // }
    // else
    //     return false;

    if ((vel_k > 0 && vel_k_1 < 0) && the_k <= threshold)
    {
        return true;
    }
    else
        return false;
}

void CanManager::detectHitting(shared_ptr<MaxonMotor> maxonMotor, float &desiredPosition)
{
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
}