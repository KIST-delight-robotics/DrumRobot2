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
    ReadStandard = chrono::system_clock::now();
    SendStandard = chrono::system_clock::now();
    addStandard = chrono::system_clock::now();

    sendLoopPeriod = std::chrono::steady_clock::now();
    recvLoopPeriod = std::chrono::steady_clock::now();
    stateMachinePeriod = std::chrono::steady_clock::now();

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
        int can_id = motorMapping[motor_pair.first];

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
                tMotor->useFourBarLinkage = true;
                tMotor->setInitialMotorAngle(tMotor->initialJointAngle);
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
                tMotor->useFourBarLinkage = true;
                tMotor->setInitialMotorAngle(tMotor->initialJointAngle);
            }
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            // 각 모터 이름에 따른 멤버 변수 설정
            if (motor_pair.first == "R_wrist")
            {
                maxonMotor->cwDir = 1.0f;
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
                maxonMotor->cwDir = 1.0f;
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
    canManager.setMotorsSocket();
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
                if (tMotor->myName == "waist")
                {
                    tservocmd.comm_can_set_origin(*tMotor, &tMotor->sendFrame, 0);
                    canManager.sendMotorFrame(tMotor);
                }
                else if (tMotor->myName == "R_arm1")
                {
                    tservocmd.comm_can_set_origin(*tMotor, &tMotor->sendFrame, 0);
                    canManager.sendMotorFrame(tMotor);
                }
                else if (tMotor->myName == "L_arm1")
                {
                    tservocmd.comm_can_set_origin(*tMotor, &tMotor->sendFrame, 0);
                    canManager.sendMotorFrame(tMotor);
                }
                else if (tMotor->myName == "R_arm2")
                {
                    tservocmd.comm_can_set_origin(*tMotor, &tMotor->sendFrame, 0);
                    canManager.sendMotorFrame(tMotor);
                }
                else if (tMotor->myName == "L_arm2")
                {
                    tservocmd.comm_can_set_origin(*tMotor, &tMotor->sendFrame, 0);
                    canManager.sendMotorFrame(tMotor);
                }
                else if (tMotor->myName == "R_arm3")
                {
                    tservocmd.comm_can_set_origin(*tMotor, &tMotor->sendFrame, 0);
                    canManager.sendMotorFrame(tMotor);
                }
                else if (tMotor->myName == "L_arm3")
                {
                    tservocmd.comm_can_set_origin(*tMotor, &tMotor->sendFrame, 0);
                    canManager.sendMotorFrame(tMotor);
                }
            }   
        }
        std::cout << "set zero and offset setting ~ ~ ~\n";
        sleep(2);   // Set Zero 명령이 확실히 실행된 후
    }

    if (input == "o" || input == "i")
    {
        maxonMotorEnable();
        setMaxonMotorMode("CSP");

        state.main = Main::AddStance;
        flagObj.setAddStanceFlag("isHome");
        flagObj.setFixationFlag("moving");

        return false;
    }
    else
    {
        std::cout << "Invalid command or not allowed in current state!\n";
        return true;
    }
}

void DrumRobot::initializeDrumRobot()
{
    std::string input;

    initializePathManager();
    initializeMotors(); // Tmotor
    initializeCanManager();
    motorSettingCmd(); // 손목
    canManager.setSocketNonBlock();

    usbio.initUSBIO4761();
    fun.openCSVFile();

    std::cout << "System Initialize Complete [ Press Commands ]\n";
    std::cout << "- o : Set Zero & Offset setting\n";
    std::cout << "- i : Offset setting\n";

    do
    {
        std::cout << "Enter command: ";
        std::getline(std::cin, input);
    } while (initializePos(input));
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
                canManager.sendAndRecv(motor, frame);
            }
            else if (targetMode == "CST")   // Cyclic Sync Torque Mode
            {
                maxoncmd.getCSTMode(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
            else if (targetMode == "HMM")   // Homming Mode
            {
                maxoncmd.getHomeMode(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
            else if (targetMode == "CSP")   // Cyclic Sync Position Mode
            {
                maxoncmd.getCSPMode(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                                  Exit                                      */
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

////////////////////////////////////////////////////////////////////////////////
/*                                   THREAD                                   */
////////////////////////////////////////////////////////////////////////////////

void DrumRobot::stateMachine()
{
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
                sendAddStanceProcess();
                break;
            }
            case Main::Play:
            {
                sendPlayProcess();
                break;
            }
            case Main::Test:
            {
                testManager.SendTestProcess(flagObj);  
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
    bool fixFlag = false;
    
    while (state.main != Main::Shutdown)
    {
        sendLoopPeriod = std::chrono::steady_clock::now();
        sendLoopPeriod += std::chrono::microseconds(5000);  // 주기 : 5msec
        
        std::map<std::string, bool> fixFlags; // 각 모터의 고정 상태 저장

        if (!canManager.setCANFrame(fixFlags))
        {
            state.main = Main::Error;
        }

        //  모든 모터가 고정 상태인지 체크
        bool allMotorsStagnant = !fixFlags.empty();
        for (const auto& flag : fixFlags)
        {
            if (!flag.second)  // 하나라도 false이면 전체가 고정된 것이 아님
            {
                allMotorsStagnant = false;
                break;
            }
        }
        if (allMotorsStagnant)
        {
            flagObj.setFixationFlag("fixed");
        }

        bool isWriteError = false;

        for (auto &motor_pair : motors)
        {
            shared_ptr<GenericMotor> motor = motor_pair.second;

            if (!canManager.sendMotorFrame(motor))
            {
                isWriteError = true;
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

        flagObj.setFixationFlag("fixed");
        
        std::this_thread::sleep_until(sendLoopPeriod);
    }
}

void DrumRobot::recvLoopForThread()
{
    while (state.main != Main::Shutdown)
    {
        recvLoopPeriod = std::chrono::steady_clock::now();
        recvLoopPeriod += std::chrono::microseconds(50000);  // 주기 : 100us

        canManager.readFramesFromAllSockets(); 
        bool isSafe = canManager.distributeFramesToMotors(true);
        if (!isSafe)
        {
            state.main = Main::Error;
        }

        std::this_thread::sleep_until(recvLoopPeriod);
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                              Ideal State                                  */
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
        flagObj.setFixationFlag("moving");
        state.main = Main::AddStance;
    }
    else if (input == "p" && flagName == "isReady")
    {
        flagObj.setAddStanceFlag("isHome"); // 연주가 끝난 후 Home 으로 돌아옴
        flagObj.setFixationFlag("moving");
        state.main = Main::Play;
    }
    else if (input == "h" && flagName == "isReady")
    {
        flagObj.setAddStanceFlag("isHome");
        flagObj.setFixationFlag("moving");
        state.main = Main::AddStance;
    }
    else if (input == "s" && flagName == "isHome")
    {
        flagObj.setAddStanceFlag("isShutDown");
        flagObj.setFixationFlag("moving");
        state.main = Main::AddStance;
    }
    else if (input == "t")
    {
        // 이거 어렵다...
        state.main = Main::Test;
    }
    else
    {
        std::cout << "Invalid command or not allowed in current state!\n";
    }
}

void DrumRobot::idealStateRoutine()
{
    if (flagObj.getFixationFlag())
    {
        int ret = system("clear");
        if (ret == -1)
            std::cout << "system clear error" << endl;

        string flag = flagObj.getAddStanceFlag();

        displayAvailableCommands(flag);

        std::string input;

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

    if (flag == "isHome")
    {
        pathManager.getArr(pathManager.homeArr);
    }
    else if (flag == "isReady")
    {
        pathManager.getArr(pathManager.readyArr);
    }
    else if (flag == "isShutDown")
    {
        pathManager.getArr(pathManager.backArr);
    }

    state.main = Main::Ideal;
}

////////////////////////////////////////////////////////////////////////////////
/*                              Play State                                    */
////////////////////////////////////////////////////////////////////////////////

double DrumRobot::readBpm(ifstream& inputFile)
{

}

bool DrumRobot::readMeasure(ifstream& inputFile)
{

}

void DrumRobot::sendPlayProcess()
{
    if (fileIndex == 0) // 처음 파일을 열 때
    {
        std::cout << "enter music name : ";
        std::getline(std::cin, musicName);
    }

    std::string currentFile = basePath + musicName + std::to_string(fileIndex) + ".txt";
    inputFile.open(currentFile); // 파일 열기

    if (inputFile.is_open())    // 파일 열기 성공
    {
        if (fileIndex == 0) // 처음 파일을 열 때
        {
            bpmOfScore = readBpm(inputFile);
            if (bpmOfScore <= 0)
            {
                std::cout << "\n BPM Read Error !!! \n";
                return;
            }
        }
        
        while(readMeasure(inputFile))    // 한마디 분량 미만으로 남을 때까지
        {
            // 충돌 회피 알고리즘 자리

            lineOfScore++;
            pathManager.generateTrajectory();

            if (lineOfScore > preCreatedLine)
            {
                pathManager.solveIKandPushConmmand();
            }
        }

        inputFile.close(); // 파일 닫기
        fileIndex++;    // 다음 파일 열 준비
    }
    else                        // 파일 열기 실패
    {
        {
            // 종료 코드 확인 -> 남은 궤적 생성

            while(1)    // 끝날 때까지
            {
                // 충돌 회피 알고리즘 자리

                lineOfScore++;
                pathManager.generateTrajectory();

                if (lineOfScore > preCreatedLine)
                {
                    pathManager.solveIKandPushConmmand();
                }
            }
            std::cout << "Play is Over\n";
            state.main = Main::Ideal;
        }
        {
            // fixed -> 비정상 종료
            std::cout << "Play is Over\n";
            state.main = Main::Ideal;
        }
        {
            // 다음 악보 생성될 때까지 대기
            usleep(100);
        }
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