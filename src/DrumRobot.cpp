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

void DrumRobot::initializePathManager()
{
    pathManager.getDrumPositoin();
    pathManager.setReadyAngle();
}

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
                // tMotor->setInitialMotorAngle(tMotor->initialJointAngle);
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
                maxonMotor->cwDir = 1.0f;
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
            maxonMotorCount++;
            virtualMaxonMotor = maxonMotor;
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

            if (name == "L_wrist")
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

    initializePathManager();
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
            // case Main::FGPlay:
            // {
            //     flagObj.setFixationFlag("moving");
            //     sendFGProcess();
            //     break;
            // }
            // case Main::TFGPlay:
            // {
            //     flagObj.setFixationFlag("moving");
            //     sendTFGProcess();
            //     break;
            // }
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

        if (maxonMotorCount != 0)
        {
            maxoncmd.getSync(&virtualMaxonMotor->sendFrame);

            if (!canManager.sendMotorFrame(virtualMaxonMotor))
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
    std::unique_ptr<sf::Music> music;

    while (state.main != Main::Shutdown)
    {
        if (state.main == Main::Play && playMusic)
        {
            if (!music && setWaitingTime)
            {
                music = std::make_unique<sf::Music>();
                if (!music->openFromFile(wavPath)) {
                    std::cerr << "음악 파일 열기 실패: " << wavPath << "\n";
                    music.reset(); // 파괴
                    continue;
                }
                std::cout << "음악 준비 완료. 동기화 타이밍 대기 중...\n";    
            }

            // 재생
            if (music && !played && std::chrono::steady_clock::now() >= syncTime)
            {
                pathManager.startOfPlay = true;
                music->play();
                played = true;
                std::cout << "음악 재생 시작 (동기화 완료)\n";
            }

            // 재생 종료
            if (music && played && music->getStatus() != sf::Music::Playing)
            {
                std::cout << "음악 재생 완료\n";
                played = false;
                setWaitingTime = false;
                music.reset();  // 안전하게 소멸
            }
        }

        if (state.main == Main::Play && runPython)
        {
            if (pythonClass == 0)
            {
                // 마젠타
            }
            else
            {
                // 시간만 측정
                
            }
            runPython = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        //음악재생을 할지 파이썬을 할지 
        //1. 드럼로봇 1 이 치는거에 따라서 정해진 악보를 연주하는 드럼로봇2
        //2. 드럼 로봇 1 이 치는거에 따라서//bpm이런 정보들이 뭐가 필요한게 있으면 말해주세요 새로 생성한게 원곡 bpm 어울려야함 연주하는 드럼로봇 2
        //3. 음악에 따라서 정해진 악보를 연주하는 드럼로봇 2 
        // 파이썬 들어갈거고 
        //1. 버전 들머패드로 입력 받아서 첫타격입력이 들어오면 그 시간을 재서 써주는 코드
        //2. 첫타격입력이 들어오면 그시간 써서 써주고 녹음도 하고 끝나면 마젠타 돌려서 mid 아웃풋으로 내주는 코드
        // 음악 초기화
        //c++ 에서도 이제는 입력? 받은거에따라서 1,2,3 번루프중에 어떤 걸로 동작할지 정할수 있는 코드만들기 
        // if (state.main == Main::TFGPlay && !music && pathManager.startOfPlay)
        // {
        //     music = std::make_unique<sf::Music>();
        //     if (!music->openFromFile(pathManager.wavPath)) {
        //         std::cerr << "음악 파일 열기 실패: " << pathManager.wavPath << "\n";
        //         music.reset(); // 파괴
        //         continue;
        //     }
        //     std::cout << "음악 준비 완료. 동기화 타이밍 대기 중...\n";
        // }

        // // 재생
        // if (music && !played && std::chrono::steady_clock::now() >= pathManager.syncTime)
        // {
        //     string fileName = "Time";
        //     fun.appendToCSV_DATA(fileName, 0, 0, 0);
        //     pathManager.startOfPlay = false;
        //     music->play();
        //     played = true;
        //     std::cout << "음악 재생 시작 (동기화 완료)\n";
        // }

        // // 재생 종료
        // if (music && played && music->getStatus() != sf::Music::Playing)
        // {
        //     std::cout << "음악 재생 완료\n";
        //     played = false;
        //     music.reset();  // 안전하게 소멸
        // }

        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
        initializePlayState();
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
    // clearBufferforRecord();  // 아래 system 함수들
    // clearMotorsCommandBuffer();  // 이거 왜 하지??? 

    string flag = flagObj.getAddStanceFlag();

    pathManager.pushAddStancePath(flag);

    state.main = Main::Ideal;
}

////////////////////////////////////////////////////////////////////////////////
/*                           Python (Magenta)                                 */
////////////////////////////////////////////////////////////////////////////////

void DrumRobot::runPythonForMagenta()
{
    filesystem::path midPath;

    filesystem::path outputPath1 = "/home/shy/DrumSound/output1_drum_hits_time.csv"; 
    filesystem::path outputPath2 = "/home/shy/DrumSound/output2_mc.csv";   
    filesystem::path outputPath3 = "/home/shy/DrumSound/output3_mc2c.csv";    
    filesystem::path outputPath4 = "/home/shy/DrumSound/output4_hand_assign.csv";
    filesystem::path outputPath5 = "/home/shy/DrumSound/output5_final.txt";
    
    int userInput = 100;
    cout << "\n 1 - 녹음 \n 2 - 악보 생성\n";
    cout << "Enter Command: ";
    cin >> userInput;

    if (userInput == 1)
    {
        //이부분에 파이썬 파일 실행시키기 
        std::string pythonCmd = "/home/shy/DrumSound/magenta-env/bin/python /home/shy/DrumSound/getMIDI_input.py";

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
            midPath = "/home/shy/DrumSound/output_temp_03.mid";
        } else if (selected_input == "2") {
            midPath = "/home/shy/DrumSound/output_temp_08.mid";
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
                    fun.analyzeMidiEvent(midiData, pos, runningStatus, note_on_time, tpqn, outputPath1);
                }
                pos = trackEnd;
            }

            fun.roundDurationsToStep(outputPath1, outputPath2); 
            fun.convertMcToC(outputPath2, outputPath3);
            fun.assignHandsToEvents(outputPath3, outputPath4);
            fun.convertToMeasureFile(outputPath4, outputPath5);

            file_found = false;
            // if(filesystem::exists(midPath))
            // {
            //     filesystem::remove(midPath);
            // }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                              Play State                                    */
////////////////////////////////////////////////////////////////////////////////

void DrumRobot::initializePlayState()
{
    fileIndex = 0;

    measureMatrix.resize(1, 9);
    measureMatrix = MatrixXd::Zero(1, 9);

    endOfScore = false;
    lineOfScore = 0;        ///< 현재 악보 읽은 줄.
    measureTotalTime = 0.0;     ///< 악보를 읽는 동안 누적 시간. [s]

    pathManager.startOfPlay = false; // true 로 변경시키면 연주 시작
}

bool DrumRobot::selectPlayMode()
{
    bool useMagenta = false;
    std::string input;

    std::cout << "Drumming With Music (1) / Just Drumming (2)";
    std::cout << "\nEnter command: ";
    std::cin >> input;

    if (input == "1")
    {
        std::string wavFileName;
        std::cout << "enter music name : ";
        std::cin >> wavFileName;
        wavPath = wavBasePath + wavFileName + ".wav";
        playMusic = true;
    }
    else
    {
        playMusic = false;
    }

    std::cout << "enter music code name (Use Magenta : output5_final): ";
    std::cin >> txtFileName;

    if (txtFileName == "output5_final") // 마젠타 사용 시 최종 출력 파일 이름
    {
        txtPath = magentaPath + txtFileName + ".txt";
        useMagenta = true;
    }
    else
    {
        txtPath = txtBasePath + txtFileName + std::to_string(fileIndex) + ".txt";
        useMagenta = false;
    }

    std::cout << "enter BPM of music ";
    std::cin >> bpmOfScore;

    std::cout << "enter control mode (CSP : 1 / CST : 0): ";
    std::cin >> maxonMotorMode;
    
    if (maxonMotorMode == 0)
    {
        pathManager.MaxonMode = "CST";
        pathManager.Kp = 60;
        pathManager.Kd = 7;
        pathManager.Kppp = 0.0;
    }
    else
    {
        pathManager.MaxonMode = "CSP";
    }

    std::cout << "Play After Waiting (1) / Play When Receiving Input (2) / Return to Ideal State (0)";
    std::cout << "\nEnter command: ";
    std::cin >> input;

    if (input == "1")
    {
        int waitingTime = 1;
        std::cout << "Enter Waiting Time : ";
        std::cin >> waitingTime;
        
        if (useMagenta)
        {
            pythonClass = 0;
            runPython = true;
        }

        syncTime = std::chrono::steady_clock::now() + std::chrono::seconds(waitingTime);
        setWaitingTime = true;
    }
    else if (input == "2")
    {
        int waitingTime = 1;
        std::cout << "Enter Waiting Time : ";
        std::cin >> waitingTime;
        
        if (useMagenta)
        {
            pythonClass = 0;
            runPython = true;
        }
        else
        {
            pythonClass = 1;
            runPython = true;
        }

        syncTime = 111 + std::chrono::seconds(waitingTime);
        setWaitingTime = true;
    }
    else
    {
        flagObj.setFixationFlag("fixed");
        state.main = Main::Ideal;
    }

    return useMagenta;
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

double DrumRobot::readBpm(ifstream& inputFile)
{
    string row;
    getline(inputFile, row);
    istringstream iss(row);
    string item;
    vector<string> items;

    while (getline(iss, item, '\t'))
    {
        item = trimWhitespace(item);
        cout << item << "\n";
        items.push_back(item);
    }

    cout << items[0] << "\n";
    cout << items[0].substr(4) << "\n";

    return stod(items[0].substr(4));
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

        if (stod(items[0]) < 0)     // 종료 코드 확인 (마디 번호가 음수)
        {
            endOfScore = true;
            return false;
        }

        measureMatrix.conservativeResize(measureMatrix.rows() + 1, measureMatrix.cols());
        for (int i = 0; i < 8; i++)
        {
            measureMatrix(measureMatrix.rows() - 1, i) = stod(items[i]);
        }

        // total time 누적
        measureTotalTime += measureMatrix(measureMatrix.rows() - 1, 1);
        measureMatrix(measureMatrix.rows() - 1, 8) = measureTotalTime * 100.0 / bpmOfScore;

        // timeSum 누적
        timeSum += measureMatrix(measureMatrix.rows() - 1, 1);

        // timeSum이 threshold를 넘으면 true 반환
        if (timeSum > measureThreshold)
        {
            return true;
        }
    }
    return false;
}

void DrumRobot::processLine()
{
    lineOfScore++;
    // std::cout << "\n//////////////////////////////// Read Measure : " << lineOfScore << "\n";
    // std::cout << measureMatrix;
    // std::cout << "\n ////////////// \n";

    if (measureMatrix.rows() > 1)
    {
        // pathManager.avoidCollision(measureMatrix);      // 충돌 회피
        pathManager.generateTrajectory(measureMatrix);  // 궤적 생성
    }

    if (lineOfScore > preCreatedLine)
    {
        pathManager.solveIKandPushCommand();        // IK & 명령 생성
    }
}

void DrumRobot::sendPlayProcess()
{
    bool useMagenta = selectPlayMode();

    inputFile.open(txtPath); // 파일 열기

    inputFile.seekg(0, ios::beg); // 안전하게 파일 맨 처음으로 이동
    inputFile.clear();            // 상태 비트 초기화

    if (inputFile.is_open() && (!endOfScore))    //////////////////////////////////////// 파일 열기 성공
    {   
        while(readMeasure(inputFile))    // 한마디 분량 미만으로 남을 때까지 궤적/명령 생성
        {
            processLine();
        }

        inputFile.close(); // 파일 닫기
        fileIndex++;    // 다음 파일 열 준비
        if (useMagenta)
        {
            txtPath = magentaPath + txtFileName + std::to_string(fileIndex) + ".txt";
        }
        else
        {
            txtPath = txtBasePath + txtFileName + std::to_string(fileIndex) + ".txt";
        }
    }
    else    //////////////////////////////////////////////////////////// 파일 열기 실패
    {
        if (fileIndex == 0)                     ////////// 1. Play 시작도 못한 경우 (악보 입력 오타 등) -> Ideal 로 이동
        {
            std::cout << "not find " << txtPath << "\n";
            flagObj.setFixationFlag("fixed");
            state.main = Main::Ideal;
            return;
        }
        else if (endOfScore)                    ////////// 2. 종료 코드가 확인된 경우 : 남은 궤적/명령 만들고 종료
        {
            while (!pathManager.endOfPlayCommand)      // 명령 전부 생성할 때까지
            {
                processLine();
            }

            std::cout << "Play is Over\n";
            flagObj.setAddStanceFlag("isHome"); // 연주 종료 후 Home 으로 이동
            state.main = Main::AddStance;
        }
        else if (flagObj.getFixationFlag())     ////////// 3. 로봇 상태가 fixed 로 변경 (악보가 들어오기 전 명령 소진) -> 에러
        {
            std::cout << "Error : not find " << txtPath << "\n";
            state.main = Main::Error;
        }
        else                                    ////////// 4. 다음 악보 생성될 때까지 대기
        {
            usleep(100);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                              FG performance State                          */
////////////////////////////////////////////////////////////////////////////////

// void DrumRobot::sendFGProcess()
// {
//     filesystem::path magentaPath;

//     filesystem::path outputPath1 = "/home/shy/DrumSound/output1_drum_hits_time.csv"; 
//     filesystem::path outputPath2 = "/home/shy/DrumSound/output2_mc.csv";   
//     filesystem::path outputPath3 = "/home/shy/DrumSound/output3_mc2c.csv";    
//     filesystem::path outputPath4 = "/home/shy/DrumSound/output4_hand_assign.csv";
//     filesystem::path outputPath5 = "/home/shy/DrumSound/output5_final.txt";
    
//     if (!FG_start)
//     {
//         int userInput = 100;
//         cout << "\n 1 - 녹음 시작 \n 2 - 연주 시작\n";
//         cout << "Enter Command: ";
//         cin >> userInput;

//         if (userInput == 1)
//         {
//             //이부분에 파이썬 파일 실행시키기 
//             std::string pythonCmd = "/home/shy/DrumSound/magenta-env/bin/python /home/shy/DrumSound/getMIDI_input.py";

//             int ret = std::system(pythonCmd.c_str());
//             if (ret != 0) {
//                 std::cerr << "Python 스크립트 실행 실패!\n";
//                 return;
//             }

//             // mid 파일 들어올 때까지 대기
//             //filesystem::path magentaPath = "/home/shy/DrumSound/output.mid";        // 파일 경로 + 이름
//         }
//         else if (userInput == 2)
//         {
//             // 2. 사용자에게 사용할 파일 선택
//             std::string selected_input;
//             std::cout << "\n원하는 리듬 스타일을 선택하세요:\n";
//             std::cout << "1 - 안정적인 리듬 (temperature 0.3)\n";
//             std::cout << "2 - 창의적인 리듬 (temperature 0.8)\n";
//             std::cout << "입력: ";
//             std::cin >> selected_input;

//             if (selected_input == "1") {
//                 magentaPath = "/home/shy/DrumSound/output_temp_03.mid";
//             } else if (selected_input == "2") {
//                 magentaPath = "/home/shy/DrumSound/output_temp_08.mid";
//             } else {
//                 std::cerr << "잘못된 입력입니다. 1 또는 2를 입력하세요.\n";
//                 return;
//             }
//             while(!file_found && !FG_start) // ready 상태인지도 확인해주기
//             {
//                 if (filesystem::exists(magentaPath) && flagObj.getAddStanceFlag() == "isReady")
//                 {
//                     file_found = true;          // 악보 끝나면 악보 지우고 false로
//                     break;
//                 } 
//                 std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 0.5초마다 체크
//             }

//             // mid 파일 받아서 악보 생성하기
//             if(file_found)
//             {
//                 size_t pos;
//                 unsigned char runningStatus;
//                 // int initial_setting_flag = 0;
//                 double note_on_time = 0;

//                 std::vector<unsigned char> midiData;

//                 if (filesystem::exists(magentaPath) && flagObj.getAddStanceFlag() == "isReady")
//                 {
//                     if (!fun.readMidiFile(magentaPath, midiData)) cout << "mid file error\n";
//                 } 
//                 // if (!fun.readMidiFile(targetPath, midiData)) cout << "mid file error\n";
//                 pos = 14;
//                 int tpqn = (midiData[12] << 8) | midiData[13];

//                 while (pos + 8 <= midiData.size()) {
//                     if (!(midiData[pos] == 'M' && midiData[pos+1] == 'T' && midiData[pos+2] == 'r' && midiData[pos+3] == 'k')) {
//                         // std::cerr << "MTrk expected at pos " << pos << "\n";
//                         break;
//                     }
//                     size_t trackLength = (midiData[pos+4] << 24) |
//                                     (midiData[pos+5] << 16) |
//                                     (midiData[pos+6] << 8) |
//                                     midiData[pos+7];
//                     pos += 8;
//                     size_t trackEnd = pos + trackLength;

//                     note_on_time = 0;
//                     while (pos < trackEnd) {
//                         size_t delta = fun.readTime(midiData, pos);
//                         note_on_time += delta;
//                         fun.analyzeMidiEvent(midiData, pos, runningStatus, note_on_time, tpqn, outputPath1);
//                     }
//                     pos = trackEnd;
//                 }

//                 fun.roundDurationsToStep(outputPath1, outputPath2); 
//                 fun.convertMcToC(outputPath2, outputPath3);
//                 fun.assignHandsToEvents(outputPath3, outputPath4);
//                 fun.convertToMeasureFile(outputPath4, outputPath5);
                
//                 FG_start = true;

//                 file_found = false;
//                 // if(filesystem::exists(magentaPath))
//                 // {
//                 //     filesystem::remove(magentaPath);
//                 // }
//             }
//         }
//     }
//     else if (FG_start)
//     {
//         if (fileIndex == 0) // 처음 파일을 열 때
//         {
//             musicName = outputPath5;
//             maxonMotorMode = 0;
//             pathManager.Kp = 60;
//             pathManager.Kd= 7;
//             pathManager.Kppp = 0.0;
//         }

//         inputFile.open(musicName); // 파일 열기

//         if (inputFile.is_open() && (!endOfScore))    //////////////////////////////////////// 파일 열기 성공
//         {
//             if (fileIndex == 0) // 처음 파일을 열 때 -> bpm 확인
//             {
//                 bpmOfScore = 60.0;

//                 if (bpmOfScore > 0)
//                 {
//                     std::cout << "music bpm = " << bpmOfScore << "\n";
//                     pathManager.initializeValue(bpmOfScore);
//                 }
//                 else
//                 {
//                     std::cout << "\n bpm Read Error !!! \n";
//                     inputFile.close(); // 파일 닫기
//                     state.main = Main::Ideal;
//                     return;
//                 }
//             }
            
//             while(readMeasure(inputFile))    // 한마디 분량 미만으로 남을 때까지 궤적/명령 생성
//             {
//                 processLine();
//             }

//             inputFile.close(); // 파일 닫기
//             fileIndex++;
//         }
//         else    //////////////////////////////////////////////////////////// 파일 열기 실패
//         {
//             if (fileIndex == 0)                     ////////// 1. Play 시작도 못한 경우 (악보 입력 오타 등) -> Ideal 로 이동
//             {
//                 std::cout << "not find " << musicName << "\n";
//                 flagObj.setFixationFlag("fixed");
//                 state.main = Main::Ideal;
//                 return;
//             }
//             else if (endOfScore)                    ////////// 2. 종료 코드가 확인된 경우 : 남은 궤적/명령 만들고 종료
//             {
//                 while (!pathManager.endOfPlayCommand)      // 명령 전부 생성할 때까지
//                 {
//                     processLine();
//                 }

//                 FG_start = false;
//                 std::cout << "Play is Over\n";
//                 flagObj.setAddStanceFlag("isHome"); // 연주 종료 후 Home 으로 이동
//                 state.main = Main::AddStance;
//             }
//             else if (flagObj.getFixationFlag())     ////////// 3. 로봇 상태가 fixed 로 변경 (악보가 들어오기 전 명령 소진) -> 에러
//             {
//                 std::cout << "Error : not find " << musicName << "\n";
//                 state.main = Main::Error;
//             }
//             else                                    ////////// 4. 다음 악보 생성될 때까지 대기
//             {
//                 usleep(100);
//             }
//         }
//     }
// }

// void DrumRobot::initializeFGPlayState()
// {
//     fileIndex = 0;

//     measureMatrix.resize(1, 9);
//     measureMatrix = MatrixXd::Zero(1, 9);

//     endOfScore = false;
//     lineOfScore = 0;        ///< 현재 악보 읽은 줄.
//     measureTotalTime = 0.0;     ///< 악보를 읽는 동안 누적 시간. [s]
// }

// ////////////////////////////////////////////////////////////////////////////////
// /*                             TFG performance State                          */
// ////////////////////////////////////////////////////////////////////////////////

// void DrumRobot::sendTFGProcess()
// {

//     if (!FG_start)
//     {
//         int userInput = 100;
//         std::cout << "\n 1 - 연주 시작\n";
//         std::cout << "Enter Command: ";
//         std::cin >> userInput;
        
//         if (userInput == 1)
//         {
//             std::string baseName;
//             std::cout << "파일 이름을 입력하세요 : ";
//             std::cin >> baseName;
        
            
//             // pathManager.wavPath = "/home/taehwang/DrumRobot/DrumRobot2/include/music/" + baseName + ".wav";
//             // pathManager.txtPath = "/home/taehwang/DrumRobot/DrumRobot2/include/codes/" + baseName + ".txt";


//             pathManager.wavPath = "/home/shy/DrumRobot/include/music/" + baseName + ".wav";
//             pathManager.txtPath = "/home/shy/DrumRobot/include/codes/" + baseName + ".txt";
            
    
//             FG_start = true;
//             // 저장하거나 전역 상태로 넘겨야 함
//         }
//     }
//     else if (FG_start)
//     {
//         if (fileIndex == 0) // 처음 파일을 열 때
//         {
//             musicName = pathManager.txtPath;
//             maxonMotorMode = 0;
//             pathManager.Kp = 60;
//             pathManager.Kd= 7;
//             pathManager.Kppp = 0.0;
//             pathManager.MaxonMode = "CST";
//             syncTime = std::chrono::steady_clock::now() + std::chrono::seconds(3);
//             pathManager.startOfPlay = false;    // 악보를 읽고 처음 동작 시작
//         }

//         inputFile.open(musicName); // 파일 열기

//         inputFile.seekg(0, ios::beg); // 안전하게 파일 맨 처음으로 이동
//         inputFile.clear();            // 상태 비트 초기화

//         if (inputFile.is_open() && (!endOfScore))    //////////////////////////////////////// 파일 열기 성공
//         {
//             if (fileIndex == 0) // 처음 파일을 열 때 -> bpm 확인
//             {
//                 // bpmOfScore = 88.0;
//                 bpmOfScore = readBpm(inputFile);

//                 if (bpmOfScore > 0)
//                 {
//                     std::cout << "music bpm = " << bpmOfScore << "\n";
//                     pathManager.initializeValue(bpmOfScore);
//                 }
//                 else
//                 {
//                     std::cout << "\n bpm Read Error !!! \n";
//                     inputFile.close(); // 파일 닫기
//                     state.main = Main::Ideal;
//                     return;
//                 }
//             }
            
//             while(readMeasure(inputFile))    // 한마디 분량 미만으로 남을 때까지 궤적/명령 생성
//             {
//                 processLine();
//             }

//             inputFile.close(); // 파일 닫기
//             fileIndex++;
//         }
//         else    //////////////////////////////////////////////////////////// 파일 열기 실패
//         {
//             if (fileIndex == 0)                     ////////// 1. Play 시작도 못한 경우 (악보 입력 오타 등) -> Ideal 로 이동
//             {
//                 std::cout << "not find " << musicName << "\n";
//                 flagObj.setFixationFlag("fixed");
//                 state.main = Main::Ideal;
//                 return;
//             }
//             else if (endOfScore)                    ////////// 2. 종료 코드가 확인된 경우 : 남은 궤적/명령 만들고 종료
//             {
//                 while (!pathManager.endOfPlayCommand)      // 명령 전부 생성할 때까지
//                 {
//                     processLine();
//                 }

//                 FG_start = false;
//                 std::cout << "Play is Over\n";
//                 flagObj.setAddStanceFlag("isHome"); // 연주 종료 후 Home 으로 이동
//                 state.main = Main::AddStance;
//             }
//             else if (flagObj.getFixationFlag())     ////////// 3. 로봇 상태가 fixed 로 변경 (악보가 들어오기 전 명령 소진) -> 에러
//             {
//                 std::cout << "Error : not find " << musicName << "\n";
//                 state.main = Main::Error;
//             }
//             else                                    ////////// 4. 다음 악보 생성될 때까지 대기
//             {
//                 usleep(100);
//             }
//         }
//     }
// }

// void DrumRobot::initializeTFGPlayState()
// {
//     fileIndex = 0;

//     measureMatrix.resize(1, 9);
//     measureMatrix = MatrixXd::Zero(1, 9);

//     endOfScore = false;
//     lineOfScore = 0;        ///< 현재 악보 읽은 줄.
//     measureTotalTime = 0.0;     ///< 악보를 읽는 동안 누적 시간. [s]
// }

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

////////////////////////////////////////////////////////////////////////////////
/*                                 SYSTEM                                     */
////////////////////////////////////////////////////////////////////////////////
/*
void DrumRobot::clearBufferforRecord()
{
    for (auto &motor_pair : motors)
    {
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
        {
            tMotor->clearCommandBuffer();
            tMotor->clearReceiveBuffer();
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
        {
            maxonMotor->clearCommandBuffer();
            maxonMotor->clearReceiveBuffer();
        }
    }
}

void DrumRobot::clearMotorsCommandBuffer()
{
    for (const auto &motorPair : motors)
    {

        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motorPair.second))
        {
            maxonMotor->clearCommandBuffer();
        }
        else if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motorPair.second))
        {
            tMotor->clearCommandBuffer();
        }
    }
}
*/