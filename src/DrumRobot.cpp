#include "../include/tasks/DrumRobot.hpp"

// For Qt
// #include "../tasks/DrumRobot.hpp"

// DrumRobot 클래스의 생성자
DrumRobot::DrumRobot(State &stateRef,
                     CanManager &canManagerRef,
                     PathManager &pathManagerRef,
                     TestManager &testManagerRef,
                     std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef,
                     USBIO &usbioRef,
                     Functions &funRef)
    : state(stateRef),
      canManager(canManagerRef),
      pathManager(pathManagerRef),
      testManager(testManagerRef),
      motors(motorsRef),
      usbio(usbioRef),
      fun(funRef)
{
    sendLoopPeriod = std::chrono::steady_clock::now();
    recvLoopPeriod = std::chrono::steady_clock::now();
}

////////////////////////////////////////////////////////////////////////////////
/*                          Initialize DrumRobot                              */
////////////////////////////////////////////////////////////////////////////////

void DrumRobot::initializeMotors()
{
    motors["waist"] = make_shared<TMotor>(0x00, "AK10_9");
    motors["R_arm1"] = make_shared<TMotor>(0x01, "AK70_10");
    motors["L_arm1"] = make_shared<TMotor>(0x02, "AK70_10");
    motors["R_arm2"] = make_shared<TMotor>(0x03, "AK70_10");
    motors["R_arm3"] = make_shared<TMotor>(0x04, "AK70_10");
    motors["L_arm2"] = make_shared<TMotor>(0x05, "AK70_10");
    motors["L_arm3"] = make_shared<TMotor>(0x06, "AK70_10");
    motors["R_wrist"] = make_shared<MaxonMotor>(0x07);
    motors["L_wrist"] = make_shared<MaxonMotor>(0x08); 
    motors["maxonForTest"] = make_shared<MaxonMotor>(0x09);
    motors["R_foot"] = make_shared<MaxonMotor>(0x0A);
    motors["L_foot"] = make_shared<MaxonMotor>(0x0B);
    
    for (auto &motor_pair : motors)
    {
        auto &motor = motor_pair.second;
        int can_id = canManager.motorMapping[motor_pair.first];

        // 타입에 따라 적절한 캐스팅과 초기화 수행
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
        {
            // 각 모터 이름에 따른 멤버 변수 설정
            if (motor_pair.first == "waist")
            {
                tMotor->cwDir = 1.0f;
                tMotor->rMin = jointRangeMin[can_id] * M_PI / 180.0f; // -90deg
                tMotor->rMax = jointRangeMax[can_id] * M_PI / 180.0f;  // 90deg
                tMotor->myName = "waist";
                tMotor->initialJointAngle = initialJointAngles[can_id] * M_PI / 180.0f;
                tMotor->currentLimit = 29.8;  // [A]    // ak10-9
                tMotor->useFourBarLinkage = false;
            }
            else if (motor_pair.first == "R_arm1")
            {
                tMotor->cwDir = -1.0f;
                tMotor->rMin = jointRangeMin[can_id] * M_PI / 180.0f;   // 0deg
                tMotor->rMax = jointRangeMax[can_id] * M_PI / 180.0f; // 150deg
                tMotor->myName = "R_arm1";
                tMotor->initialJointAngle = initialJointAngles[can_id] * M_PI / 180.0f;
                tMotor->currentLimit = 23.2;  // [A]    // ak70-10
                tMotor->useFourBarLinkage = false;
            }
            else if (motor_pair.first == "L_arm1")
            {
                tMotor->cwDir = -1.0f;
                tMotor->rMin = jointRangeMin[can_id] * M_PI / 180.0f;  // 30deg
                tMotor->rMax = jointRangeMax[can_id] * M_PI / 180.0f; // 180deg
                tMotor->myName = "L_arm1";
                tMotor->initialJointAngle = initialJointAngles[can_id] * M_PI / 180.0f;
                tMotor->currentLimit = 23.2;  // [A]    // ak70-10
                tMotor->useFourBarLinkage = false;
            }
            else if (motor_pair.first == "R_arm2")
            {
                tMotor->cwDir = 1.0f;
                tMotor->rMin = jointRangeMin[can_id] * M_PI / 180.0f; // -60deg
                tMotor->rMax = jointRangeMax[can_id] * M_PI / 180.0f;  // 90deg
                tMotor->myName = "R_arm2";
                tMotor->initialJointAngle = initialJointAngles[can_id] * M_PI / 180.0f;
                tMotor->currentLimit = 23.2;  // [A]    // ak70-10
                tMotor->useFourBarLinkage = false;
            }
            else if (motor_pair.first == "R_arm3")
            {
                tMotor->cwDir = -1.0f;
                tMotor->rMin = jointRangeMin[can_id] * M_PI / 180.0f; // -30deg
                tMotor->rMax = jointRangeMax[can_id] * M_PI / 180.0f; // 130deg
                tMotor->myName = "R_arm3";
                tMotor->initialJointAngle = initialJointAngles[can_id] * M_PI / 180.0f;
                tMotor->currentLimit = 23.2;  // [A]    // ak70-10
                tMotor->useFourBarLinkage = false;
                // tMotor->setInitialMotorAngle(tMotor->initialJointAngle);     // 4-bar-linkage 사용하면 쓰는 함수
            }
            else if (motor_pair.first == "L_arm2")
            {
                tMotor->cwDir = -1.0f;
                tMotor->rMin = jointRangeMin[can_id] * M_PI / 180.0f; // -60deg
                tMotor->rMax = jointRangeMax[can_id] * M_PI / 180.0f;  // 90deg
                tMotor->myName = "L_arm2";
                tMotor->initialJointAngle = initialJointAngles[can_id] * M_PI / 180.0f;
                tMotor->currentLimit = 23.2;  // [A]    // ak70-10
                tMotor->useFourBarLinkage = false;
            }
            else if (motor_pair.first == "L_arm3")
            {
                tMotor->cwDir = 1.0f;
                tMotor->rMin = jointRangeMin[can_id] * M_PI / 180.0f; // -30 deg
                tMotor->rMax = jointRangeMax[can_id] * M_PI / 180.0f; // 130 deg
                tMotor->myName = "L_arm3";
                tMotor->initialJointAngle = initialJointAngles[can_id] * M_PI / 180.0f;
                tMotor->currentLimit = 23.2;  // [A]    // ak70-10
                tMotor->useFourBarLinkage = false;
                // tMotor->setInitialMotorAngle(tMotor->initialJointAngle);     // 4-bar-linkage 사용하면 쓰는 함수
            }
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            // 각 모터 이름에 따른 멤버 변수 설정
            if (motor_pair.first == "R_wrist")
            {
                maxonMotor->cwDir = -1.0f;
                maxonMotor->rMin = jointRangeMin[can_id] * M_PI / 180.0f; // -108deg
                maxonMotor->rMax = jointRangeMax[can_id] * M_PI / 180.0f;  // 135deg
                maxonMotor->txPdoIds[0] = 0x207; // Controlword
                maxonMotor->txPdoIds[1] = 0x307; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x407; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x507; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x187; // Statusword, ActualPosition, ActualTorque
                maxonMotor->rxPdoIds[1] = 0x287; // ActualPosition, ActualTorque
                maxonMotor->myName = "R_wrist";
                maxonMotor->initialJointAngle = initialJointAngles[can_id] * M_PI / 180.0f;
            }
            else if (motor_pair.first == "L_wrist")
            {
                maxonMotor->cwDir = -1.0f;
                maxonMotor->rMin = jointRangeMin[can_id] * M_PI / 180.0f; // -108deg
                maxonMotor->rMax = jointRangeMax[can_id] * M_PI / 180.0f;  // 135deg
                maxonMotor->txPdoIds[0] = 0x208; // Controlword
                maxonMotor->txPdoIds[1] = 0x308; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x408; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x508; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x188; // Statusword, ActualPosition, ActualTorque
                maxonMotor->rxPdoIds[1] = 0x288; // ActualPosition, ActualTorque
                maxonMotor->myName = "L_wrist";
                maxonMotor->initialJointAngle = initialJointAngles[can_id] * M_PI / 180.0f;
            }
            else if (motor_pair.first == "R_foot")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = jointRangeMin[can_id] * M_PI / 180.0f; // -90deg
                maxonMotor->rMax = jointRangeMax[can_id] * M_PI / 180.0f; // 135deg
                maxonMotor->txPdoIds[0] = 0x20A; // Controlword
                maxonMotor->txPdoIds[1] = 0x30A; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x40A; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x50A; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x18A; // Statusword, ActualPosition, ActualTorque
                maxonMotor->rxPdoIds[1] = 0x28A; // ActualPosition, ActualTorque
                maxonMotor->myName = "R_foot";
                maxonMotor->initialJointAngle = initialJointAngles[can_id] * M_PI / 180.0f;
            }
            else if (motor_pair.first == "L_foot")
            {
                maxonMotor->cwDir = -1.0f;
                maxonMotor->rMin = jointRangeMin[can_id] * M_PI / 180.0f; // -90deg
                maxonMotor->rMax = jointRangeMax[can_id] * M_PI / 180.0f; // 135deg
                maxonMotor->txPdoIds[0] = 0x20B; // Controlword
                maxonMotor->txPdoIds[1] = 0x30B; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x40B; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x50B; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x18B; // Statusword, ActualPosition, ActualTorque
                maxonMotor->rxPdoIds[1] = 0x28B; // ActualPosition, ActualTorque
                maxonMotor->myName = "L_foot";
                maxonMotor->initialJointAngle = initialJointAngles[can_id] * M_PI / 180.0f;
            }
            else if (motor_pair.first == "maxonForTest")
            {
                maxonMotor->cwDir = 1.0f;
                maxonMotor->rMin = jointRangeMin[can_id] * M_PI / 180.0f; // -108deg
                maxonMotor->rMax = jointRangeMax[can_id] * M_PI / 180.0f; // 135deg
                maxonMotor->txPdoIds[0] = 0x209; // Controlword
                maxonMotor->txPdoIds[1] = 0x309; // TargetPosition
                maxonMotor->txPdoIds[2] = 0x409; // TargetVelocity
                maxonMotor->txPdoIds[3] = 0x509; // TargetTorque
                maxonMotor->rxPdoIds[0] = 0x189; // Statusword, ActualPosition, ActualTorque
                maxonMotor->rxPdoIds[1] = 0x289; // ActualPosition, ActualTorque
                maxonMotor->myName = "maxonForTest";
                maxonMotor->initialJointAngle = initialJointAngles[can_id] * M_PI / 180.0f;
            }
        }
    }
}

void DrumRobot::initializeCanManager()
{
    canManager.initializeCAN();
    canManager.checkCanPortsStatus();
    //true 모터연결안된것 김태황
    allMotorsUnConected = canManager.setMotorsSocket();
}

void DrumRobot::motorSettingCmd()
{
    // Count Maxon Motors
    for (const auto &motor_pair : motors)
    {
        // 각 요소가 MaxonMotor 타입인지 확인
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
        {

            if (virtualMaxonMotor.size() == 0)
            {
                virtualMaxonMotor.push_back(maxonMotor);
            }
            else
            {
                bool otherSocket = true;
                int n = virtualMaxonMotor.size();
                for(int i = 0; i < n; i++)
                {
                    if (virtualMaxonMotor[i]->socket == maxonMotor->socket)
                    {
                        otherSocket = false;
                    }
                }

                if (otherSocket)
                {
                    virtualMaxonMotor.push_back(maxonMotor);
                }
            }
        }
    }

    struct can_frame frame;
    canManager.setSocketsTimeout(2, 0);

    for (const auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<GenericMotor> motor = motorPair.second;
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motorPair.second))
        {
            // CSP Settings
            maxoncmd.getCSPMode(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);

            maxoncmd.getPosOffset(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);

            maxoncmd.getTorqueOffset(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);

            // CSV Settings
            maxoncmd.getCSVMode(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);

            maxoncmd.getVelOffset(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);

            // CST Settings
            maxoncmd.getCSTMode(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);

            maxoncmd.getTorqueOffset(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);

            // HMM Settigns
            maxoncmd.getHomeMode(*maxonMotor, &frame);
            canManager.sendAndRecv(motor, frame);

            if(name == "L_wrist")
            {
                maxoncmd.getHomingMethodL(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomeoffsetDistance(*maxonMotor, &frame, 0);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomePosition(*maxonMotor, &frame, 0);
                canManager.sendAndRecv(motor, frame);

                // maxoncmd.getCurrentThresholdL(*maxonMotor, &frame);
                // canManager.sendAndRecv(motor, frame);
            }
            else if (name == "R_wrist")
            {
                maxoncmd.getHomingMethodR(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomeoffsetDistance(*maxonMotor, &frame, 0);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomePosition(*maxonMotor, &frame, 0);
                canManager.sendAndRecv(motor, frame);

                // maxoncmd.getCurrentThresholdR(*maxonMotor, &frame);
                // canManager.sendAndRecv(motor, frame);
            }
            else if (name == "R_foot")
            {
                maxoncmd.getHomingMethodR(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomeoffsetDistance(*maxonMotor, &frame, 0);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomePosition(*maxonMotor, &frame, 0);
                canManager.sendAndRecv(motor, frame);

                // maxoncmd.getCurrentThresholdR(*maxonMotor, &frame);
                // canManager.sendAndRecv(motor, frame);
            }
            else if (name == "L_foot")
            {
                maxoncmd.getHomingMethodR(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomeoffsetDistance(*maxonMotor, &frame, 0);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomePosition(*maxonMotor, &frame, 0);
                canManager.sendAndRecv(motor, frame);

                // maxoncmd.getCurrentThresholdR(*maxonMotor, &frame);
                // canManager.sendAndRecv(motor, frame);
            }
            else if (name == "maxonForTest")
            {
                maxoncmd.getHomingMethodTest(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomeoffsetDistance(*maxonMotor, &frame, 0);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getHomePosition(*maxonMotor, &frame, 0);
                canManager.sendAndRecv(motor, frame);

                maxoncmd.getCurrentThresholdL(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
        }
    }
}

bool DrumRobot::initializePos(const std::string &input)
{
    // set zero
    if (input == "o")
    {
        for (const auto &motorPair : motors)
        {
            if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motorPair.second))
            {
                tservocmd.comm_can_set_origin(*tMotor, &tMotor->sendFrame, 0);
                canManager.sendMotorFrame(tMotor);
                tMotor->finalMotorPosition = 0.0;

                usleep(1000*100);    // 100ms

                // std::cout << "Tmotor [" << tMotor->myName << "] set Zero \n";
                // std::cout << "Current Motor Position : " << tMotor->motorPosition / M_PI * 180 << "deg\n";
            }
            else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motorPair.second))
            {
                maxonMotor->finalMotorPosition = 0.0;
            }
        }
        std::cout << "set zero and offset setting ~ ~ ~\n";
        sleep(2);   // Set Zero 명령이 확실히 실행된 후

        maxonMotorEnable();
        setMaxonMotorMode("CSP");

        state.main = Main::AddStance;
        flagObj.setAddStanceFlag("isHome"); // 시작 자세는 Home 자세와 같음

        return true;
    }
    else if (input == "i")
    {
        // maxonMotorEnable();
        // setMaxonMotorMode("CSP");

        // state.main = Main::AddStance;
        // flagObj.setAddStanceFlag("isHome");
        
        return false;
    }
    else
    {
        std::cout << "Invalid command or not allowed in current state!\n";
        return false;
    }
}

void DrumRobot::initializeDrumRobot()
{
    std::string input;

    pathManager.initPathManager();
    initializeMotors();
    initializeCanManager();
    motorSettingCmd(); // Maxon
    canManager.setSocketNonBlock();

    usbio.initUSBIO4761();
    fun.openCSVFile();

    std::cout << "System Initialize Complete [ Press Commands ]\n";
    std::cout << "- o : Set Zero & Offset setting\n";
    // std::cout << "- i : Offset setting\n";

    do
    {
        std::cout << "Enter command: ";
        std::getline(std::cin, input);
    } while (!initializePos(input));
}

////////////////////////////////////////////////////////////////////////////////
/*                                    Exit                                    */
////////////////////////////////////////////////////////////////////////////////

void DrumRobot::deactivateControlTask()
{
    struct can_frame frame;

    canManager.setSocketsTimeout(0, 500000);

    for (auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        auto &motor = motorPair.second;

        // 타입에 따라 적절한 캐스팅과 초기화 수행
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
        {
            tservocmd.comm_can_set_cb(*tMotor, &tMotor->sendFrame, 0);
            canManager.sendMotorFrame(tMotor);
            std::cout << "Exiting for motor [" << name << "]" << std::endl;
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            maxoncmd.getQuickStop(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);
            std::cout << "Exiting for motor [" << name << "]" << std::endl;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/*                           Maxon Motor Function                             */
/////////////////////////////////////////////////////////////////////////////////

void DrumRobot::maxonMotorEnable()
{
    struct can_frame frame;
    canManager.setSocketsTimeout(2, 0);

    // 제어 모드 설정
    for (const auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<GenericMotor> motor = motorPair.second;
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            maxoncmd.getHomeMode(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getOperational(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            usleep(100000);

            maxoncmd.getShutdown(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);

            usleep(100000);
            
            maxoncmd.getEnable(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);
            
            std::cout << "Maxon Enabled\n";

            usleep(100000);

            // maxoncmd.getStartHoming(*maxonMotor, &frame);
            // canManager.txFrame(motor, frame);

            // maxoncmd.getSync(&frame);
            // canManager.txFrame(motor, frame);

            // usleep(100000);
            
            // std::cout << "Maxon Enabled(2) \n";
        }
    }
}

void DrumRobot::setMaxonMotorMode(std::string targetMode)
{
    struct can_frame frame;
    canManager.setSocketsTimeout(0, 10000);
    for (const auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<GenericMotor> motor = motorPair.second;
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motorPair.second))
        {
            if (targetMode == "CSV")    // Cyclic Sync Velocity Mode
            {
                maxoncmd.getCSVMode(*maxonMotor, &frame);
                canManager.txFrame(motor, frame);
            }
            else if (targetMode == "CST")   // Cyclic Sync Torque Mode
            {
                maxoncmd.getCSTMode(*maxonMotor, &frame);
                canManager.txFrame(motor, frame);
            }
            else if (targetMode == "HMM")   // Homming Mode
            {
                maxoncmd.getHomeMode(*maxonMotor, &frame);
                canManager.txFrame(motor, frame);
            }
            else if (targetMode == "CSP")   // Cyclic Sync Position Mode
            {
                maxoncmd.getCSPMode(*maxonMotor, &frame);
                canManager.txFrame(motor, frame);
            }

            // 모드 바꾸고 껐다 켜주기

            maxoncmd.getShutdown(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);

            usleep(100);

            maxoncmd.getEnable(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);

            usleep(100);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                                   THREAD                                   */
////////////////////////////////////////////////////////////////////////////////

void DrumRobot::stateMachine()
{
    sleep(1);   // read thead에서 clear 할 때까지 기다림

    while (state.main != Main::Shutdown)
    {
        switch (state.main.load())
        {
            case Main::Ideal:
            {
                idealStateRoutine();
                break;
            }
            case Main::AddStance:
            {
                flagObj.setFixationFlag("moving");
                sendAddStanceProcess();
                break;
            }
            case Main::Play:
            {
                flagObj.setFixationFlag("moving");
                sendPlayProcess();
                break;
            }
            case Main::Test:
            {
                testManager.SendTestProcess();  
                state.main = Main::Ideal;
                break;
            }
            case Main::Error:
            {
                state.main = Main::Shutdown;
                break;
            }
            case Main::Shutdown:
            {
                break;
            }
            case Main::Pause:
            {
                break;
            }
        }
    }

    // Exit
    if (usbio.useUSBIO)
    {
        usbio.exitUSBIO4761();
    }
    canManager.setSocketBlock();
    deactivateControlTask();
}

void DrumRobot::sendLoopForThread()
{
    sleep(2);   // read thead에서 clear / initial pos Path 생성 할 때까지 기다림

    bool wasFixed = false; // 이전 `fixed` 상태 추적
    int cycleCounter = 0; // 주기 조절을 위한 변수 (Tmotor : 5ms, Maxon : 1ms)
    sendLoopPeriod = std::chrono::steady_clock::now();
    while (state.main != Main::Shutdown)
    {
        sendLoopPeriod += std::chrono::microseconds(1000);  // 주기 : 1msec
        
        std::map<std::string, bool> fixFlags; // 각 모터의 고정 상태 저장
        
        if (!canManager.setCANFrame(fixFlags, cycleCounter))
        {   
            state.main = Main::Error;
            break;
        }

        static std::map<std::string, bool> prevFixFlags;
        bool newData = false; // 버퍼 크기가 증가했는지 여부 추적

        //  모든 모터가 고정 상태인지 체크
        bool allMotorsStagnant = !fixFlags.empty();
        for (const auto& flag : fixFlags)
        {
            if (!prevFixFlags[flag.first] && flag.second) // 이전 값이 false였다가 true가 된 경우
            {
                newData = true; // 버퍼가 다시 증가했음을 표시
            }
            prevFixFlags[flag.first] = flag.second; // 현재 상태를 prevFixFlags에 저장
        }

        if (allMotorsStagnant && !wasFixed)
        {
            flagObj.setFixationFlag("fixed");
            wasFixed = true;
        }
        else if (newData) // 새로운 데이터가 들어오면 wasFixed 해제
        {
            wasFixed = false;
        }

        //////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////보내기///////////////////////////////////
        //////////////////////////////////////////////////////////////////////////

        bool isWriteError = false; 
        
        for (auto &motor_pair : motors)
        {
            std::shared_ptr<GenericMotor> motor = motor_pair.second;
            // t모터는 5번 중 1번만 실행
            if (motor->isTMotor()) 
            {
                if (cycleCounter == 0) // 5ms마다 실행
                {
                    if (!canManager.sendMotorFrame(motor))
                    {
                        isWriteError = true;
                    }
                }
            }
            else if (motor->isMaxonMotor())
            {
                // Maxon모터는 1ms마다 실행
                if (!canManager.sendMotorFrame(motor))
                {
                    isWriteError = true;
                }
            }
        }

        int n = virtualMaxonMotor.size();
        for(int i = 0; i < n; i++)
        {
            maxoncmd.getSync(&virtualMaxonMotor[i]->sendFrame);

            if (!canManager.sendMotorFrame(virtualMaxonMotor[i]))
            {
                isWriteError = true;
            };
        }
        
        if (isWriteError)
        {
            state.main = Main::Error;
        }

        // 모든 모터가 연결 안된 경우 : 바로 fixed
        if(allMotorsUnConected)
        {
            flagObj.setFixationFlag("fixed");
        }

        cycleCounter = (cycleCounter + 1) % 5;

        std::this_thread::sleep_until(sendLoopPeriod);
    }
}

void DrumRobot::recvLoopForThread()
{
    canManager.clearReadBuffers();

    while (state.main != Main::Shutdown)
    {
        recvLoopPeriod = std::chrono::steady_clock::now();
        recvLoopPeriod += std::chrono::microseconds(100);  // 주기 : 100us

        canManager.readFramesFromAllSockets(); 
        bool isSafe = canManager.distributeFramesToMotors(true);
        if (!isSafe)
        {
            state.main = Main::Error;
        }

        std::this_thread::sleep_until(recvLoopPeriod);
    }
}

void DrumRobot::musicMachine()
{
    bool played = false;
    bool waiting = false;
    std::unique_ptr<sf::Music> music;

    while (state.main != Main::Shutdown)
    {
        if (state.main == Main::Play)
        {
            if (playMusic)
            {
                if (setWaitingTime)
                {
                    music = std::make_unique<sf::Music>();
                    if (!music->openFromFile(wavPath)) {
                        std::cerr << "음악 파일 열기 실패: " << wavPath << "\n";
                        music.reset(); // 파괴
                        continue;
                    }
                    std::cout << "음악 준비 완료. 동기화 타이밍 대기 중...\n";
                    setWaitingTime = false;
                    waiting = true;
                }

                if (waiting)
                {
                    // 재생
                    if (!played && std::chrono::system_clock::now() >= syncTime)
                    {
                        pathManager.startOfPlay = true;
                        music->play();
                        played = true;
                        std::cout << "음악 재생 시작 (동기화 완료)\n";
                    }

                    // 재생 종료
                    if (played && music->getStatus() != sf::Music::Playing)
                    {
                        std::cout << "음악 재생 완료\n";
                        played = false;
                        waiting = false;
                        music.reset();  // 안전하게 소멸
                    }
                }
            }
            else
            {
                if (setWaitingTime)
                {
                    setWaitingTime = false;
                    waiting = true;
                }

                if (waiting)
                {
                    if (std::chrono::system_clock::now() >= syncTime)
                    {
                        pathManager.startOfPlay = true;
                        waiting = false;
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

void DrumRobot::runPythonInThread()
{
    while (state.main != Main::Shutdown)
    {
        if (runPython)
        {
            if (pythonClass == 0)
            {
                // 시간 측정 + 마젠타
                if(repeatNum ==1)
                {
                    std::string pythonCmd = "/home/shy/DrumRobot/DrumSound/magenta-env/bin/python /home/shy/DrumRobot/DrumSound/getMIDITimeMagenta.py";

                    int ret = std::system(pythonCmd.c_str());
                    if (ret != 0)
                    {
                        std::cerr << "Python script failed to execute with code " << ret << std::endl;
                    }

                    getMagentaSheet("/home/shy/DrumRobot/DrumSound/output.mid", 0);
                }
                else
                {
                    std::string pythonArgs = "--rec_times ";
                        
                    for (int i = 0; i < repeatNum; ++i) {
                        float d = delayTime.front(); delayTime.pop();
                        float r = recordTime.front(); recordTime.pop();
                        float m = makeTime.front(); makeTime.pop();

                        pythonArgs += std::to_string(d) + " ";
                        pythonArgs += std::to_string(r) + " ";
                        pythonArgs += std::to_string(m) + " ";
                    }

                    std::string pythonCmd = "/home/shy/DrumRobot/DrumSound/magenta-env/bin/python "
                    "/home/shy/DrumRobot/DrumSound/getMIDI_rec_Mag.py " + pythonArgs + " &";

                    int ret = std::system(pythonCmd.c_str());  // 비동기 실행 (백그라운드 &)
                    if (ret != 0)
                    {
                        std::cerr << "Python script failed to execute with code " << ret << std::endl;
                    }

                    for (int i = 0; i < repeatNum; i++)
                    {
                        for (int j = 0; j < 2; j++)
                        {
                            std::string midiPath = "/home/shy/DrumRobot/DrumSound/record_output/output_" + std::to_string(i) + std::to_string(j) + ".mid";
                            std::string veloPath = "/home/shy/DrumRobot/DrumSound/record_velocity/drum_events_" + std::to_string(i) + std::to_string(j) + ".mid";

                            // 해당 MIDI 파일이 생성될 때까지 대기
                            while (!std::filesystem::exists(midiPath)) {
                                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                            }
                            
                            getMagentaSheet(midiPath, veloPath, j);  // 파이썬이 끝나지 않아도 즉시 실행
                        }
                    }

                    // getMIDI_rec_Mag.py의 출력이 담긴 폴더 주소. getMagnetaSheet 후에 파일 지우기 위함.
                    std::string velodir = "/home/shy/DrumRobot/DrumSound/record_velocity";
                    std::string inputdir = "/home/shy/DrumRobot/DrumSound/record_input";
                    std::string outputdir = "/home/shy/DrumRobot/DrumSound/record_output";

                    clear_directory(velodir);
                    clear_directory(inputdir);
                    clear_directory(outputdir);
                }
            }
            else
            {
                // 시간만 측정
                std::string pythonCmd = "/home/shy/DrumRobot/DrumSound/magenta-env/bin/python /home/shy/DrumRobot/DrumSound/getMIDITime.py";

                int ret = std::system(pythonCmd.c_str());
                if (ret != 0)
                {
                    std::cerr << "Python script failed to execute with code " << ret << std::endl;
                }
                
            }
            runPython = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                                Ideal State                                 */
////////////////////////////////////////////////////////////////////////////////

void DrumRobot::displayAvailableCommands(string flagName) const
{
    std::cout << "Available Commands:\n";

    if (state.main == Main::Ideal)
    {
        if (flagName == "isHome")
        {
            std::cout << "- r : Move to Ready Pos\n";
            std::cout << "- t : Start Test\n";
            std::cout << "- s : Shut down the system\n";
        }
        else if (flagName == "isReady")
        {
            std::cout << "- p : Play Drumming\n";
            std::cout << "- t : Start Test\n";
            std::cout << "- h : Move to Home Pos\n";
            std::cout << "- m : Run Python (Magenta)\n";
        }
    }
    else
    {
        std::cout << "- s : Shut down the system\n";
    }
}

void DrumRobot::processInput(const std::string &input, string flagName)
{
    if (input == "r" && flagName == "isHome")
    {
        flagObj.setAddStanceFlag("isReady");
        state.main = Main::AddStance;
    }
    else if (input == "p" && flagName == "isReady")
    {
        state.main = Main::Play;
    }
    else if (input == "m" && flagName == "isReady")
    {
        runPythonForMagenta();
    }
    else if (input == "h" && flagName == "isReady")
    {
        flagObj.setAddStanceFlag("isHome");
        state.main = Main::AddStance;
    }
    else if (input == "s" && flagName == "isHome")
    {
        flagObj.setAddStanceFlag("isShutDown");
        state.main = Main::AddStance;
    }
    else if (input == "t")
    {
        state.main = Main::Test;
    }
    else
    {
        std::cout << "Invalid command or not allowed in current state!\n";
    }
}

void DrumRobot::idealStateRoutine()
{
    std::string input;

    static bool isLockKeyRemoved = false;      // 키 제거했는지 확인

    if (!isLockKeyRemoved)
    {
        do
        {
            std::cout << "키 뽑았는지 확인(k) : ";
            std::getline(std::cin, input);
        } while (input != "k");

        isLockKeyRemoved = true;
    }

    if (flagObj.getFixationFlag())
    {
        string flag = flagObj.getAddStanceFlag();

        if (flag == "isShutDown")
        {
            state.main = Main::Shutdown;
            return;
        }

        if (repeatNum > currentIterations) // 반복 실행 중인 경우 바로 Play로 이동
        {
            state.main = Main::Play;
            return;
        }

        int ret = system("clear");
        if (ret == -1)
            std::cout << "system clear error" << endl;

        displayAvailableCommands(flag);

        std::cout << "Enter command: ";
        std::getline(std::cin, input);

        processInput(input, flag); 
    }
    else
    {
        usleep(100);
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                            AddStance State                                 */
////////////////////////////////////////////////////////////////////////////////

void DrumRobot::sendAddStanceProcess()
{
    string flag = flagObj.getAddStanceFlag();

    pathManager.pushAddStancePath(flag);

    state.main = Main::Ideal;
}

////////////////////////////////////////////////////////////////////////////////
/*                              Play State                                    */
////////////////////////////////////////////////////////////////////////////////

void DrumRobot::initializePlayState()
{
    measureMatrix.resize(1, 9);
    measureMatrix = MatrixXd::Zero(1, 9);

    endOfScore = false;
    measureTotalTime = 0.0;     ///< 악보 총 누적 시간. [s]

    pathManager.initPlayStateValue();
}

void DrumRobot::setSyncTime(int waitingTimeMillisecond)
{   
    // wait time + sync time 더해서 기다릴수 있도록 세팅 여기서 세팅된 시간이 pathManager 에서 기다리게 된다.   
    // 싱크 타임이 두개 거나 악보 읽는것도 두개거나
    // txt 파일 시간 읽어오기
    std::ifstream infile(syncPath);
    std::string time_str;
    if (!infile || !(infile >> time_str)) {
        std::cerr << "sync.txt 파일을 읽을 수 없습니다.\n" << syncPath;
        return;
    }

    // HH:MM:SS.mmm 파싱
    int hour, min, sec, millis;
    char sep1, sep2, dot;
    std::istringstream iss(time_str);
    if (!(iss >> hour >> sep1 >> min >> sep2 >> sec >> dot >> millis)) {
        std::cerr << "시간 형식 파싱 실패\n";
        return;
    }

    auto now = std::chrono::system_clock::now();
    time_t tt = std::chrono::system_clock::to_time_t(now);
    tm* local_tm = std::localtime(&tt);
    local_tm->tm_hour = hour;
    local_tm->tm_min = min;
    local_tm->tm_sec = sec;
    auto base_time = std::chrono::system_clock::from_time_t(std::mktime(local_tm)) + std::chrono::milliseconds(millis);

    std::remove(syncPath.c_str());      // syncTime 업데이트 하고 sync.txt 바로 지움
    
    syncTime = base_time + std::chrono::milliseconds(waitingTimeMillisecond);
    setWaitingTime = true;

    // fun.appendToCSV("sync_test.txt", false, -1, inputWaitMs);
    // fun.appendToCSV("sync_test.txt", false, syncTime);
    // fun.appendToCSV("sync_test.txt", false, base_time);
}

std::string DrumRobot::selectPlayMode()
{
    std::string txtPath;
    std::string txtFileName;
    bool useMagenta = false;
    int input;

    ////////////////////////////////////////////
    // 악보, bpm, 연주모드 입력받기

    std::cout << "\nEnter Music Code Name (Magenta : output5_final): ";
    std::cin >> txtFileName;

    if (txtFileName == "output5_final") // 마젠타 사용 시 최종 출력 파일 이름
    {
        // txtPath = magentaPath + txtFileName + ".txt";
        txtPath = magentaPath + txtFileName;
        useMagenta = true;
        //반복횟수에 따라 딜레이시간 , 만들시간 정의해주기 
        std::cout << "반복 횟수 : ";
        cin >> repeatNum;
        for(int i = 0; i < repeatNum; i++)
        {
            std::cout << i + 1 << "번째 delay time : ";
            cin >> delayTime_i;
            std::cout << i + 1 << "번째 record time : ";
            cin >> recordTime_i;
            std::cout << i + 1 << "번째 make time : ";
            cin >> makeTime_i;
            delayTime.push(delayTime_i);
            recordTime.push(recordTime_i);
            makeTime.push(makeTime_i);
        }
    }
    else
    {
        // txtPath = txtBasePath + txtFileName + std::to_string(fileIndex) + ".txt";
        txtPath = txtBasePath + txtFileName;
        useMagenta = false;
        repeatNum = 1;
    }

    std::cout << "Enter Initial BPM of Music: ";
    std::cin >> pathManager.bpmOfScore;

    std::cout << "Enter Maxon Control Mode (CSP : 1 / CST : 0): ";
    std::cin >> input;
    
    if (input == 0)
    {
        pathManager.MaxonMode = "CST";
        pathManager.Kp = 60;
        pathManager.Kd = 7;
    }
    else
    {
        pathManager.MaxonMode = "CSP";
    }

    ////////////////////////////////////////////
    // 1 음악 입력 2 그냥연주

    std::cout << "\nEnter (Drumming With Music : 1 / Just Drumming : 2): ";
    std::cin >> input;

    if (input == 1)
    {
        std::string wavFileName;
        std::cout << "Enter Music Name: ";
        std::cin >> wavFileName;
        wavPath = wavBasePath + wavFileName + ".wav";
        playMusic = true;
    }
    else //2
    {
        playMusic = false;
    }

    ////////////////////////////////////////////
    // 시작 트리거 정하기 1 -> 일정시간뒤에 2 -> 입력들어오고 난 후에 일정시간 뒤에 0 -> back

    if (useMagenta)
    {
        std::cout << "\nPlay When Receiving Input \n";
        input = 2;
    }
    else
    {
        std::cout << "\nEnter (Play After Waiting : 1 / Play When Receiving Input : 2 / Return to Ideal State : 0): ";
        std::cin >> input;
    }

    if (input == 1)
    {
        float inputWaitMs = 0.0;
        std::cout << "Enter Waiting Time: ";
        std::cin >> inputWaitMs;
        inputWaitMs *= 1000;

        syncTime = std::chrono::system_clock::now() + std::chrono::milliseconds((int)inputWaitMs);
        setWaitingTime = true;
    }
    else if (input == 2)
    {
        float inputWaitMs = 0.0;

        if (useMagenta)
        {
            //여기에서 musicMachine 마젠타 돌아간다
            if (repeatNum == 1)
            {
                std::cout << "Enter Waiting Time: ";
                std::cin >> inputWaitMs;
                inputWaitMs *= 1000;
            }
            else
            {
                for (int i = 0; i < repeatNum; i++)
                {
                    std::cout << i + 1 << "번째 wait time : ";
                    cin >> waitTime_i;
                    waitTime.push(waitTime_i);
                }
                inputWaitMs = waitTime.front() * 1000;
                waitTime.pop();
            }

            pythonClass = 0;
            runPython = true;

            std::string txtIndexPath = txtPath + "0.txt";
            while (!std::filesystem::exists(txtIndexPath)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 100ms 대기
            }
        }
        else
        {
            //그저 시간채크 하는 코드
            std::cout << "Enter Waiting Time: ";
            std::cin >> inputWaitMs;
            inputWaitMs *= 1000;
            
            pythonClass = 1;
            runPython = true;

            while (!std::filesystem::exists(syncPath)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 100ms 대기
            }
        }

        setSyncTime((int)inputWaitMs);
    }
    else
    {
        txtPath = magentaPath + "null"; // 존재하지 않는 악보
    }

    return txtPath;
}

string DrumRobot::trimWhitespace(const std::string &str)
{
    size_t first = str.find_first_not_of(" \t");
    if (std::string::npos == first)
    {
        return str;
    }
    size_t last = str.find_last_not_of(" \t");
    return str.substr(first, (last - first + 1));
}

bool DrumRobot::readMeasure(ifstream& inputFile)
{
    string row;
    double timeSum = 0.0;

    for (int i = 1; i < measureMatrix.rows(); i++)
    {
        timeSum += measureMatrix(i, 1);
    }

    // timeSum이 threshold를 넘으면 true 반환
    if (timeSum > measureThreshold)
    {
        return true;
    }

    while (getline(inputFile, row))
    {
        istringstream iss(row);
        string item;
        vector<string> items;

        while (getline(iss, item, '\t'))
        {
            item = trimWhitespace(item);
            items.push_back(item);
        }

        if (items[0] == "bpm")                          // bpm 변경 코드
        {
            // std::cout << "\n bpm : " << pathManager.bpmOfScore;
            pathManager.bpmOfScore = stod(items[1]);
            // std::cout << " -> " << pathManager.bpmOfScore << "\n";
        }
        else if (items[0] == "end")                     // 종료 코드
        {
            endOfScore = true;
            return false;
        }
        else if (stod(items[0]) < 0)                     // 종료 코드 (마디 번호가 음수)
        {
            endOfScore = true;
            return false;
        }
        else
        {
            measureMatrix.conservativeResize(measureMatrix.rows() + 1, measureMatrix.cols());
            for (int i = 0; i < 8; i++)
            {
                measureMatrix(measureMatrix.rows() - 1, i) = stod(items[i]);
            }

            // total time 누적
            measureTotalTime += measureMatrix(measureMatrix.rows() - 1, 1) * 100.0 / pathManager.bpmOfScore;
            measureMatrix(measureMatrix.rows() - 1, 8) = measureTotalTime;

            // timeSum 누적
            timeSum += measureMatrix(measureMatrix.rows() - 1, 1);

            // timeSum이 threshold를 넘으면 true 반환
            if (timeSum > measureThreshold)
            {
                return true;
            }
        }
    }
    return false;
}

void DrumRobot::sendPlayProcess()
{
    std::string txtPath;
    std::string txtIndexPath;
    int fileIndex = 0;
    std::ifstream inputFile;

    // 초기화
    initializePlayState();

    // 모드 세팅
    if (repeatNum == currentIterations)
    {
        currentIterations = 1;
        txtPath = selectPlayMode();
    }
    else
    {
        currentIterations++;
        txtPath = magentaPath + "output5_final";

        txtIndexPath = txtPath + std::to_string(fileIndex) + ".txt";
        while (!std::filesystem::exists(txtIndexPath)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 100ms 대기
        }

        float inputWaitMs = waitTime.front() * 1000;
        waitTime.pop();
        setSyncTime((int)inputWaitMs);
    }

    while (!endOfScore)
    {
        txtIndexPath = txtPath + std::to_string(fileIndex) + ".txt";
        inputFile.open(txtIndexPath); // 파일 열기

        inputFile.seekg(0, ios::beg); // 안전하게 파일 맨 처음으로 이동
        inputFile.clear();            // 상태 비트 초기화

        if (inputFile.is_open())     //////////////////////////////////////// 파일 열기 성공
        {
            while(readMeasure(inputFile))    // 한마디 분량 미만으로 남을 때까지 궤적/명령 생성
            {
                pathManager.processLine(measureMatrix);
            }

            inputFile.close(); // 파일 닫기
            fileIndex++;    // 다음 파일 열 준비
        }
        else     //////////////////////////////////////////////////////////// 파일 열기 실패
        {
            if (fileIndex == 0)                     ////////// 1. Play 시작도 못한 경우 (악보 입력 오타 등) -> Ideal 로 이동
            {
                std::cout << "not find " << txtIndexPath << "\n";
                flagObj.setFixationFlag("fixed");

                repeatNum = 1;
                currentIterations = 1;

                state.main = Main::Ideal;
                return;
            }
            else if (flagObj.getFixationFlag())     ////////// 2. 로봇 상태가 fixed 로 변경 (악보가 들어오기 전 명령 소진) -> 에러
            {
                std::cout << "Error : not find " << txtIndexPath << "\n";
                state.main = Main::Error;
                return;
            }
            else                                    ////////// 3. 다음 악보 생성될 때까지 대기
            {
                usleep(100);
            }
        }
    }

    // 종료 코드 (endOfScore) 확인됨 : 남은 궤적/명령 만들고 종료
    while (!pathManager.endOfPlayCommand)      // 명령 전부 생성할 때까지
    {
        pathManager.processLine(measureMatrix);
    }

    std::cout << "Play is Over\n";
    if (repeatNum == currentIterations)
    {
        flagObj.setAddStanceFlag("isHome"); // 연주 종료 후 Home 으로 이동
    }
    else
    {
        flagObj.setAddStanceFlag("isReady"); // Play 반복 시 Ready 으로 이동
        
        // 악보 파일 저장 후 삭제
        for (int i = 0; i < fileIndex; i++)
        {
            txtIndexPath = txtPath + std::to_string(i) + ".txt";
            std::string saveCode = txtPath +  std::to_string(currentIterations-1) + std::to_string(i) + "_save.txt";

            std::filesystem::rename(txtIndexPath.c_str(), saveCode.c_str());
            std::remove(txtIndexPath.c_str());
        }
    }
    state.main = Main::AddStance;
}

////////////////////////////////////////////////////////////////////////////////
/*                           Python (Magenta)                                 */
////////////////////////////////////////////////////////////////////////////////

void DrumRobot::runPythonForMagenta()
{
    filesystem::path midPath;

    filesystem::path outputPath1 = "/home/shy/DrumRobot/DrumSound/output1_drum_hits_time.csv"; 
    filesystem::path outputPath2 = "/home/shy/DrumRobot/DrumSound/output2_mc.csv";   
    filesystem::path outputPath3 = "/home/shy/DrumRobot/DrumSound/output3_mc2c.csv";    
    filesystem::path outputPath4 = "/home/shy/DrumRobot/DrumSound/output4_hand_assign.csv";
    filesystem::path outputPath5 = "/home/shy/DrumRobot/DrumSound/output5_add_groove.txt";
    filesystem::path outputPath6 = "/home/shy/DrumRobot/DrumSound/output6_final.txt";
    
    int userInput = 100;
    cout << "\n 1 - 녹음 \n 2 - 악보 생성\n";
    cout << "Enter Command: ";
    cin >> userInput;

    if (userInput == 1)
    {
        //이부분에 파이썬 파일 실행시키기 
        std::string pythonCmd = "/home/shy/DrumRobot/DrumSound/magenta-env/bin/python /home/shy/DrumRobot/DrumSound/getMIDI_input.py";

        int ret = std::system(pythonCmd.c_str());
        if (ret != 0) {
            std::cerr << "Python 스크립트 실행 실패!\n";
            return;
        }

        // mid 파일 들어올 때까지 대기
        //filesystem::path midPath = "/home/shy/DrumSound/output.mid";        // 파일 경로 + 이름
    }
    else if (userInput == 2)
    {
        // 2. 사용자에게 사용할 파일 선택
        std::string selected_input;
        std::cout << "\n원하는 리듬 스타일을 선택하세요:\n";
        std::cout << "1 - 안정적인 리듬 (temperature 0.3)\n";
        std::cout << "2 - 창의적인 리듬 (temperature 0.8)\n";
        std::cout << "입력: ";
        std::cin >> selected_input;

        if (selected_input == "1") {
            midPath = "/home/shy/DrumRobot/DrumSound/output_temp_03.mid";
        } else if (selected_input == "2") {
            midPath = "/home/shy/DrumRobot/DrumSound/output_temp_08.mid";
        } else {
            std::cerr << "잘못된 입력입니다. 1 또는 2를 입력하세요.\n";
            return;
        }

        while(!file_found) // ready 상태인지도 확인해주기
        {
            if (filesystem::exists(midPath) && flagObj.getAddStanceFlag() == "isReady")
            {
                file_found = true;          // 악보 끝나면 악보 지우고 false로
                break;
            } 
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 0.5초마다 체크
        }

        // mid 파일 받아서 악보 생성하기
        if(file_found)
        {
            size_t pos;
            unsigned char runningStatus;
            // int initial_setting_flag = 0;
            double note_on_time = 0;

            std::vector<unsigned char> midiData;

            if (filesystem::exists(midPath) && flagObj.getAddStanceFlag() == "isReady")
            {
                if (!fun.readMidiFile(midPath, midiData)) cout << "mid file error\n";
            } 
            // if (!fun.readMidiFile(targetPath, midiData)) cout << "mid file error\n";
            pos = 14;
            int tpqn = (midiData[12] << 8) | midiData[13];
            int bpm;

            while (pos + 8 <= midiData.size()) {
                if (!(midiData[pos] == 'M' && midiData[pos+1] == 'T' && midiData[pos+2] == 'r' && midiData[pos+3] == 'k')) {
                    // std::cerr << "MTrk expected at pos " << pos << "\n";
                    break;
                }
                size_t trackLength = (midiData[pos+4] << 24) |
                                (midiData[pos+5] << 16) |
                                (midiData[pos+6] << 8) |
                                midiData[pos+7];
                pos += 8;
                size_t trackEnd = pos + trackLength;

                note_on_time = 0;
                while (pos < trackEnd) {
                    size_t delta = fun.readTime(midiData, pos);
                    note_on_time += delta;
                    fun.analyzeMidiEvent(midiData, pos, runningStatus, note_on_time, tpqn, bpm, outputPath1);
                }
                pos = trackEnd;
            }

            fun.roundDurationsToStep(outputPath1, outputPath2); 
            fun.convertMcToC(outputPath2, outputPath3);
            fun.assignHandsToEvents(outputPath3, outputPath4);
            fun.addGroove(bpm, outputPath4, outputPath5);
            fun.convertToMeasureFile(outputPath5, outputPath6);

            file_found = false;
            // if(filesystem::exists(midPath))
            // {
            //     filesystem::remove(midPath);
            // }
        }
    }
}

void DrumRobot::getMagentaSheet(std::string midPath, std::string veloPath ,int recordingIndex)
{
    // filesystem::path midPath;

    filesystem::path outputPath1 = "/home/shy/DrumRobot/DrumSound/output1_drum_hits_time.csv"; 
    filesystem::path outputPath2 = "/home/shy/DrumRobot/DrumSound/output2_mc.csv";   
    filesystem::path outputPath3 = "/home/shy/DrumRobot/DrumSound/output3_mc2c.csv";    
    filesystem::path outputPath4 = "/home/shy/DrumRobot/DrumSound/output4_hand_assign.csv";

    filesystem::path outputPath5 = "/home/shy/DrumRobot/DrumSound/output5_vel.txt";
    filesystem::path outputPath6 = "/home/shy/DrumRobot/DrumSound/output6_add_groove.txt";

    filesystem::path outputPath7 = "/home/shy/DrumRobot/DrumSound/output7_final.txt";


    //루프돌때마다 이름 바꿔줘야함
    filesystem::path velocityFile = veloPath //"/home/shy/DrumRobot/DrumSound/record_velocity/drum_events_00.txt";
    filesystem::path outputVel = "/home/shy/DrumRobot/DrumSound/vel_output.txt";

    // midPath = "/home/shy/DrumRobot/DrumSound/output_0.mid";

    while(!file_found) // ready 상태인지도 확인해주기
    {
        if (filesystem::exists(midPath) && flagObj.getAddStanceFlag() == "isReady")
        {
            file_found = true;          // 악보 끝나면 악보 지우고 false로
            break;
        } 
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 0.5초마다 체크
    }

    // mid 파일 받아서 악보 생성하기
    if(file_found)
    {
        size_t pos;
        unsigned char runningStatus;
        // int initial_setting_flag = 0;
        double note_on_time = 0;

        std::vector<unsigned char> midiData;

        if (filesystem::exists(midPath) && flagObj.getAddStanceFlag() == "isReady")
        {
            if (!fun.readMidiFile(midPath, midiData)) cout << "mid file error\n";
        } 
        // if (!fun.readMidiFile(targetPath, midiData)) cout << "mid file error\n";
        pos = 14;
        int tpqn = (midiData[12] << 8) | midiData[13];
        int bpm;

        while (pos + 8 <= midiData.size()) {
            if (!(midiData[pos] == 'M' && midiData[pos+1] == 'T' && midiData[pos+2] == 'r' && midiData[pos+3] == 'k')) {
                // std::cerr << "MTrk expected at pos " << pos << "\n";
                break;
            }
            size_t trackLength = (midiData[pos+4] << 24) |
                            (midiData[pos+5] << 16) |
                            (midiData[pos+6] << 8) |
                            midiData[pos+7];
            pos += 8;
            size_t trackEnd = pos + trackLength;

            note_on_time = 0;
            while (pos < trackEnd) {
                size_t delta = fun.readTime(midiData, pos);
                note_on_time += delta;
                fun.analyzeMidiEvent(midiData, pos, runningStatus, note_on_time, tpqn, bpm, outputPath1);
            }
            pos = trackEnd;
        }

        //이거 세기 반영 시키는 변수 안하면 원본 그대로 
        bool mapTo357 = true;
        vector<Functions::Seg> segs;

        fun.roundDurationsToStep(outputPath1, outputPath2); 
        fun.convertMcToC(outputPath2, outputPath3);
        fun.assignHandsToEvents(outputPath3, outputPath4);

        //velocityFile 세기 파일 outputFile 우리가 쓸 아웃풋 파일
        fun.analyzeVelocityWithLowPassFilter(velocityFile, outputVel, bpm);

        //위에서 만든 아웃풋 파일 넣어주기 그럼 segs 에 필터씌운 정보 저장댐
        fun.loadSegments(outputVel, segs);

        //수정전 악보 scoreIn 최종 출력 파일 scoreOut
        fun.applyIntensityToScore(segs, outputPath4, outputPath5, mapTo357);

        //그루브 추가 
        fun.addGroove(bpm, outputPath5, outputPath6);

        fun.convertToMeasureFile(outputPath6, outputPath7);

        file_found = false;
        // if(filesystem::exists(midPath))
        // {
        //     filesystem::remove(midPath);
        // }

        std::remove(outputPath1.c_str());      // 중간 단계 txt 파일 삭제
        std::remove(outputPath2.c_str());
        std::remove(outputPath3.c_str());
        std::remove(outputPath4.c_str());

        std::remove(outputPath5.c_str());
        std::remove(outputPath6.c_str());
        std::remove(outputVel.c_str());
        
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                                  Flag                                      */
////////////////////////////////////////////////////////////////////////////////

FlagClass::FlagClass()
{

}

FlagClass::~FlagClass()
{

}

void FlagClass::setAddStanceFlag(string flagName)
{
    if (flagName == "isHome")
    {
        addStanceFlag = ISHOME;
    }
    else if (flagName == "isReady")
    {
        addStanceFlag = ISREADY;
    }
    else if (flagName == "isShutDown")
    {
        addStanceFlag = ISSHUTDOWN;
    }
    else
    {
        cout << "Invalid Flag Name\n";
    }
}

string FlagClass::getAddStanceFlag()
{
    if (addStanceFlag == ISHOME)
    {
        return "isHome";
    }
    else if (addStanceFlag == ISREADY)
    {
        return "isReady";
    }
    else if (addStanceFlag == ISSHUTDOWN)
    {
        return "isShutDown";
    }

    return "isError";
}

void FlagClass::setFixationFlag(string flagName)
{
    if (flagName == "moving")
    {
        isFixed = false;
    }
    else if (flagName == "fixed")
    {
        isFixed = true;
    }
    else
    {
        cout << "Invalid Flag Name\n";
    }
}

bool FlagClass::getFixationFlag()
{
    return isFixed;
}
