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
    allMotorsUnConected = canManager.setMotorsSocket(); // 연결된 모터 없음 : 테스트 모드
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

void DrumRobot::initializeFolder()
{
    std::string syncDir = "../magenta/sync";
    std::string recordDir = "../magenta/record";
    std::string outputMidDir = "../magenta/generated";
    std::string outputVelDir = "../magenta/velocity";
    std::string outputCode = "../include/magenta";

    fun.clear_directory(syncDir);
    fun.clear_directory(recordDir);
    fun.clear_directory(outputMidDir);
    fun.clear_directory(outputVelDir);
    fun.clear_directory(outputCode);
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

        setHeadLED("POWER_ON"); // led 로딩바 차는 모션 실행

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
    dxl.initialize();

    usbio.initUSBIO4761();
    fun.openCSVFile();

    ArduinoConnect("/dev/ttyACM0");

    // 폴더 비우기
    initializeFolder();

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
                runAddStanceProcess();
                break;
            }
            case Main::Play:
            {
                runPlayProcess();
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

    setHeadLED("POWER_OFF"); // LED 프로그램 종료시 점등 -> Chzzk

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

        if (cycleCounter == 0) // 5ms마다 실행
        {
            // fixeFlags를 확인해서 1개라도 false면 무빙, 엘스 fixed
            bool isMoving = false;
            for (const auto &fixFlag : fixFlags)
            {
                if (!fixFlag.second)
                {
                    isMoving = true;
                    break;
                }
            }

            if (isMoving)
            {
                flagObj.setFixationFlag("moving");
            }
            else
            {
                flagObj.setFixationFlag("fixed");
            }
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

        // DXL
        if (cycleCounter == 0)
        {   
            if (!pathManager.dxlCommandBuffer.empty())
            {
                // 맨 앞 원소 꺼낸 값으로 SyncWrite 실행
                vector<vector<float>> dxlCommand = pathManager.dxlCommandBuffer.front();
                pathManager.dxlCommandBuffer.pop();
                dxl.syncWrite(dxlCommand);
            }
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
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void DrumRobot::runPythonInThread()
{
    int recordNum[100] = {0};
    int creationNum[100] = {0};

    while (state.main != Main::Shutdown)
    {
        if (runPython)
        {
            std::string pythonCmd = pythonScript + " " + pythonArgs;

            if (pythonArgs == "--sync")
            {
                pythonCmd += " --path ../magenta/ &";   // 경로 설정 & 백그라운드 실행

                int ret = std::system(pythonCmd.c_str());
                if (ret != 0)
                    std::cerr << "Python script failed to execute with code " << ret << std::endl;
                
                std::cout << "\nPython script is running... \n";
            }
            else if (pythonArgs == "--record")
            {
                pythonCmd += " --repeat " + std::to_string(repeatNum);

                pythonCmd += " --param";
                for (int i = 0; i < repeatNum; i++)
                {
                    int dT = delayTime.front(); delayTime.pop();
                    int rT = recordBarNum.front(); recordBarNum.pop();
                    int mT = makeBarNum.front(); makeBarNum.pop();

                    pythonCmd += " " + std::to_string(dT);
                    pythonCmd += " " + std::to_string(rT);
                    pythonCmd += " " + std::to_string(mT);

                    recordNum[i] = rT / 2;
                    creationNum[i] = mT / 2;
                }

                pythonCmd += " --bpm";
                pythonCmd += " " + std::to_string(pathManager.bpmOfScore);

                pythonCmd += " --path ../magenta/ &";   // 경로 설정 & 백그라운드 실행

                int ret = std::system(pythonCmd.c_str());
                if (ret != 0)
                    std::cerr << "Python script failed to execute with code " << ret << std::endl;

                std::cout << "\nPython script is running... \n";

                for (int i = 0; i < repeatNum; i++)
                {
                    for (int j = 0; j < creationNum[i]; j++)
                    {
                        std::string outputMid;
                        std::string outputVel = "null";
                        bool endFlag = (j == creationNum[i] - 1);

                        if (j < recordNum[i])
                        {
                            outputMid = "../magenta/generated/output" + std::to_string(i) + "_" + std::to_string(j + 1) + "3.mid";
                        }
                        else if (j < 2 * recordNum[i])
                        {
                            outputMid = "../magenta/generated/output" + std::to_string(i) + "_" + std::to_string(j - recordNum[i] + 1) + "4.mid";
                        }
                        else
                        {
                            outputMid = "../magenta/generated/output" + std::to_string(i) + "_" + std::to_string(j - 2 * recordNum[i] + 1) + "2.mid";
                        }

                        // 해당 미디 파일 생성될 때까지 대기
                        while(!std::filesystem::exists(outputMid))
                            std::this_thread::sleep_for(std::chrono::milliseconds(1));

                        usleep(100*1000);
                        generateCodeFromMIDI(outputMid, outputVel, j, endFlag);
                    }
                }

                // 폴더 비우기
                std::string recordDir = "../magenta/record";
                std::string outputMidDir = "../magenta/generated";
                std::string outputVelDir = "../magenta/velocity";
            
                fun.clear_directory(recordDir);
                fun.clear_directory(outputMidDir);
                fun.clear_directory(outputVelDir);
            }

            runPython = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
    // else if (input == "m" && flagName == "isReady")
    // {
        
    // }
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

        setHeadLED("IDLE"); // led 연주전 대기상태 -> 빨간 점등

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

void DrumRobot::runAddStanceProcess()
{
    string flag = flagObj.getAddStanceFlag();

    pathManager.genAndPushAddStance(flag);

    state.main = Main::Ideal;

    // send thread에서 읽기 전까지 대기
    while ((!allMotorsUnConected) && flagObj.getFixationFlag())
    {
       usleep(100); 
    }
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
}

void DrumRobot::displayPlayCommands(bool useMagenta, bool useDrumPad, float inputWaitMs, std::string txtFileName)
{
    int ret = system("clear");
    if (ret == -1) std::cout << "system clear error" << endl;

    // magenta or code name
    if(useMagenta)
    {
        std::cout << "magenta : On \n";
        std::cout << "trigger : Play When The Drum Pad is Struck. \n";
        std::cout << "args :\n";
        
        int argsSize = delayTime.size();
        if (repeatNum == argsSize)
        {
            std::cout << "\t- Repeat Num :" << repeatNum << "\n";
            for (int i = 0; i < repeatNum; i++)
            {
                float a, b, c, d;
                a = delayTime.front(); b = recordBarNum.front(); c = makeBarNum.front(); d = waitTime.front();
                delayTime.pop(); recordBarNum.pop(); makeBarNum.pop(); waitTime.pop();
                delayTime.push(a); recordBarNum.push(b); makeBarNum.push(c); waitTime.push(d);

                std::cout << "\t- " << i + 1 << "번째 Delay Time/Record Bar/Make Bar/Wait Time - (" << a << "/" << b << "/" << c << "/" << d << ") (s)\n";
            }
        }
        else
        {
            std::cout << "\t- Arguments Required\n";
        }
    }
    else
    {
        std::cout << "magenta : Off \n";
        std::cout << "code : " << txtFileName << "\n";

        if (useDrumPad)
        {
            std::cout << "trigger : Play When The Drum Pad is Struck (" << inputWaitMs/1000.0 << "s)\n";
        }
        else
        {
            std::cout << "trigger : Play After a Set Time Delay (" << inputWaitMs/1000.0 << "s)\n";
        }
    }

    // bpm
    std::cout << "bpm : " << pathManager.bpmOfScore << "\n";
    
    // maxon motor mode
    if (pathManager.maxonMode == "unknown")
        std::cout << "mode : unknown \n";
    else if (pathManager.maxonMode == "CST")
        std::cout << "mode : Maxon Motor Mode CST \n";
    else
        std::cout << "mode : Maxon Motor Mode CSP \n";
    
    // music
    if (playMusic)
    {
        std::cout << "music : Drumming With Music \n";
        std::cout << "\t- path :" << wavPath << "\n";
    }
    else
    {
        std::cout << "music : Just Drumming \n";
    }
    
    std::cout << "\nEnter Commad (magenta, ";
    if (useMagenta)
        std::cout << "args, ";
    else
        std::cout << "code, trigger, ";
    std::cout << "bpm, mode, music, run, exit): ";
}

void DrumRobot::setPythonArgs()
{
    std::cout << "\n반복 횟수 : ";
    cin >> repeatNum;

    // 큐 초기화 (모든 요소 삭제)
    while (!delayTime.empty()) {
        delayTime.pop();  // 큐에서 항목을 하나씩 제거
    }
    while (!recordBarNum.empty()) {
        recordBarNum.pop();
    }
    while (!makeBarNum.empty()) {
        makeBarNum.pop();
    }
    while (!waitTime.empty()) {
        waitTime.pop();
    }

    // 반복횟수에 따라 딜레이, 녹음, 생성 시간 입력
    for(int i = 0; i < repeatNum; i++)
    {
        int dT, rT, mT;
        float wT;
        
        std::cout << "\n" << i + 1 << "번째 delay time : ";
        cin >> dT;
        
        do{ 
            std::cout << i + 1 << "번째 record bar number : ";
            cin >> rT;
            if (rT % 2 == 1)
                std::cout << "녹음 마디 갯수는 2의 배수여야 합니다.\n";
        }while(rT % 2 == 1);
        
        do{
            std::cout << i + 1 << "번째 make bar number : ";
            cin >> mT;
            if(mT % 2 == 1)
                std::cout << "생성 마디 갯수는 2의 배수여야 합니다.\n";
            if(mT > 3*rT)
                std::cout << "생성 마디 갯수는 녹음 마디의 3배 이하여하 합니다.\n";
        }while(mT % 2 == 1 || mT > 3*rT);

        std::cout << i + 1 << "번째 wait time : ";
        cin >> wT;

        delayTime.push(dT);
        recordBarNum.push(rT);
        makeBarNum.push(mT);
        waitTime.push(wT);
    }
}

bool DrumRobot::checkPreconditions(bool useMagenta, std::string txtPath)
{
    if (useMagenta)
    {
        // args 확인
        int argsSize = delayTime.size();
        if ((repeatNum != argsSize))
        {
            return false;
        }

        if (pathManager.maxonMode == "unknown")
        {
            return false;
        }
    }
    else
    {
        // code 확인
        if (txtPath == "null")
        {
            return false;
        }

        if (pathManager.maxonMode == "unknown")
        {
            return false;
        }
    }

    return true;
}

std::string DrumRobot::selectPlayMode()
{
    std::string userInput;
    int cnt = 0;    // 입력 횟수 (일정 횟수 초과되면 오류)
    const int maxAttempts = 999;    // 최대 시도 횟수

    std::string errCode = "null"; // Ideal 로 이동

    bool useMagenta = false;
    bool useDrumPad = false;
    int mode = 1;
    int triggerMode = 1;
    float inputWaitMs = 3000.0; // 3s
    static std::string txtFileName = "null";
    std::string txtPath = txtBaseFolderPath + txtFileName;;
    std::string wavFileName = "null";

    while(cnt < maxAttempts)
    {
        displayPlayCommands(useMagenta, useDrumPad, inputWaitMs, txtFileName);
        std::cin >> userInput;

        if (userInput == "magenta")
        {
            if (useMagenta)
                useMagenta = false;
            else
                useMagenta = true;
        }
        else if (userInput == "args" && useMagenta)
        {
            txtPath = magentaCodePath;    // 마젠타 사용 시 최종 출력 파일 이름
            
            setPythonArgs();
        }
        else if (userInput == "code" && (!useMagenta))
        {
            std::cout << "\nEnter Music Code Name: ";
            std::cin >> txtFileName;

            txtPath = txtBaseFolderPath + txtFileName;
            repeatNum = 1;
        }
        else if (userInput == "trigger" && (!useMagenta))
        {
            std::cout << "\nEnter Trigger Mode (Play After a Set Time Delay : 1 / Play When The Drum Pad is Struck : 0): ";
            std::cin >> triggerMode;
            std::cout << "Enter Waiting Time: ";
            std::cin >> inputWaitMs;
            inputWaitMs *= 1000;

            if (triggerMode == 1)
                useDrumPad = false;
            else if (triggerMode == 0)
                useDrumPad = true;
        }
        else if (userInput == "bpm")
        {
            do
            {
                std::cout << "\nEnter Initial BPM of Music: ";
                std::cin >> pathManager.bpmOfScore;
                if (pathManager.bpmOfScore <= 0)
                {
                    std::cout << "\nInvalid Input (BPM <= 0)\n";
                }
            } while (pathManager.bpmOfScore <= 0);
        }
        else if (userInput == "mode")
        {
            do
            {
                std::cout << "\nEnter Maxon Control Mode (CSP : 1 / CST : 0): ";
                std::cin >> mode;

                if (mode == 0)
                {
                    pathManager.maxonMode = "CST";
                    pathManager.Kp = 60;
                    pathManager.Kd = 7;
                }
                else if (mode == 1)
                {
                    pathManager.maxonMode = "CSP";
                }
                else
                {
                    std::cout << "\nInvalid Input\n";
                    pathManager.maxonMode = "unknown";
                }
            } while (pathManager.maxonMode == "unknown");
        }
        else if (userInput == "music")
        {
            if (playMusic)
            {
                playMusic = false;
            }
            else
            {
                std::cout << "\nEnter Music Name: ";
                std::cin >> wavFileName;
                wavPath = wavBaseFolderPath + wavFileName + ".wav";
                playMusic = true;
            }
        }
        else if (userInput == "run")
        {
            if (checkPreconditions(useMagenta, txtPath))   // 오류 사전 점검
            {
                if (useMagenta)
                {
                    pythonArgs = "--record";
                    runPython = true;

                    std::string txtIndexPath = txtPath + "0.txt";
                    while (!std::filesystem::exists(txtIndexPath)) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1ms 대기
                    }

                    inputWaitMs = waitTime.front() * 1000;
                    waitTime.pop();
                    setSyncTime((int)inputWaitMs);
                }
                else if (useDrumPad)
                {
                    pythonArgs = "--sync";
                    runPython = true;

                    while (!std::filesystem::exists(syncPath)) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1ms 대기
                    }

                    setSyncTime((int)inputWaitMs);
                }
                else
                {
                    syncTime = std::chrono::system_clock::now() + std::chrono::milliseconds((int)inputWaitMs);
                    setWaitingTime = true;
                }
                
                return txtPath;
            }
            else
            {
                std::cout << "\nInvalid Command\n";
                sleep(1);
            }
        }
        else if (userInput == "exit")
        {
            return errCode;
        }
        else
        {
            std::cout << "\nInvalid Command\n";
            sleep(1);
        }

        cnt++;
    }

    std::cout << "\n입력을 시도한 횟수가 " << maxAttempts << " 이상입니다\n";
    sleep(1);

    return errCode;
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

    // // 루프가 끝난 후, 실패 원인 분석
    // if (inputFile.eof()) {
    //     std::cout << "getline()이 파일 끝(EOF)에 도달하여 종료되었습니다." << std::endl;
    // }
    // else if (inputFile.fail()) {
    //     std::cout << "getline()이 논리적 오류로 실패했습니다." << std::endl;
    // }
    // else if (inputFile.bad()) {
    //     std::cout << "getline()이 심각한 I/O 오류로 실패했습니다." << std::endl;
    // }
    // std::cout << measureMatrix;
    return false;
}

void DrumRobot::runPlayProcess()
{
    std::string txtPath;
    std::string txtIndexPath;
    int fileIndex = 0;

    // 초기화
    initializePlayState();

    // 모드 세팅
    if (repeatNum == currentIterations) // == 1
    {
        currentIterations = 1;
        txtPath = selectPlayMode();

        setHeadLED("PLAYING"); // led 연주중 점등 -> 아이보리 

        if (txtPath == "null")  // 잘못 입력한 경우 : Ideal 로 이동
        {
            repeatNum = 1;
            state.main = Main::Ideal;
            return;
        }
    }
    else
    {
        currentIterations++;
        txtPath = magentaCodePath;

        txtIndexPath = txtPath + std::to_string(fileIndex) + ".txt";
        while (!std::filesystem::exists(txtIndexPath)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 100ms 대기
        }

        float inputWaitMs = waitTime.front() * 1000;
        waitTime.pop();
        setSyncTime((int)inputWaitMs);
    }



    while (!endOfScore)
    {
        std::ifstream inputFile;
        txtIndexPath = txtPath + std::to_string(fileIndex) + ".txt";
        inputFile.open(txtIndexPath); // 파일 열기

        if (inputFile.is_open())     //////////////////////////////////////// 파일 열기 성공
        {
            if (inputFile.peek() == std::ifstream::traits_type::eof())
            {
                std::cout << "\n - The file exists, but it is empty.\n";
                inputFile.clear();          // 상태 비트 초기화
                usleep(100);                // 대기 : 다음 악보 작성 중 
            }
            else
            {
                while(readMeasure(inputFile))    // 한마디 분량 미만으로 남을 때까지 궤적/명령 생성
                {
                    pathManager.processLine(measureMatrix);
                }

                // send thread에서 읽기 전까지 대기
                if (fileIndex == 0)
                {
                    int sleepCnt = 0;
                    while (flagObj.getFixationFlag())
                    // while (!allMotorsUnConected && flagObj.getFixationFlag())
                    {
                        usleep(100);
                        sleepCnt ++;
                        if(sleepCnt == 50)
                            break;
                    }
                }

                inputFile.close(); // 파일 닫기
                fileIndex++;    // 다음 파일 열 준비

                // std::cout << "\nfileIndex : " << fileIndex << "\n";
            }
        }
        else     //////////////////////////////////////////////////////////// 파일 열기 실패
        {
            if (fileIndex == 0)                                             ////////// 1. Play 시작도 못한 경우 (악보 입력 오타 등) -> Ideal 로 이동
            {
                std::cout << "not find " << txtIndexPath << "\n";
                sleep(1);

                repeatNum = 1;
                currentIterations = 1;

                state.main = Main::Ideal;
                return;
            }
            else if (flagObj.getFixationFlag() && (!allMotorsUnConected))   ////////// 2. 로봇 상태가 fixed 로 변경 (악보가 들어오기 전 명령 소진) -> 에러
            {
                std::cout << "Error : not find " << txtIndexPath << "\n";
                state.main = Main::Error;
                return;
            }
            else                                                            ////////// 3. 다음 악보 생성될 때까지 대기
            {
                inputFile.clear();            // 상태 비트 초기화
                usleep(100);
            }
        }
    }

    // 종료 코드 (endOfScore) 확인됨 : 남은 궤적/명령 만들고 종료
    while (!pathManager.endOfPlayCommand)      // 명령 전부 생성할 때까지
    {
        pathManager.processLine(measureMatrix);
    }

    if(txtPath == magentaCodePath)
    {
        // 악보 파일 저장 후 삭제
        for (int i = 0; i < fileIndex; i++)
        {
            txtIndexPath = txtPath + std::to_string(i) + ".txt";

            // 현재 시간 가져오기
            auto now = std::chrono::system_clock::now();
            std::time_t t = std::chrono::system_clock::to_time_t(now);
            std::tm localTime = *std::localtime(&t);

            // 시간 문자열 생성 (MMDDHHMM)
            std::ostringstream timeStream;
            timeStream << std::setw(2) << std::setfill('0') << localTime.tm_mon + 1   // 월
                    << std::setw(2) << std::setfill('0') << localTime.tm_mday       // 일
                    << std::setw(2) << std::setfill('0') << localTime.tm_hour       // 시
                    << std::setw(2) << std::setfill('0') << localTime.tm_min;       // 분
            std::string timeStr = timeStream.str();
            
            std::string saveFolder = "../../DrumRobot_data/codes/";

            std::string saveCode = saveFolder + "save_code_" + timeStr + "_" + std::to_string(currentIterations-1) + std::to_string(i+1) + ".txt";

            std::filesystem::rename(txtIndexPath.c_str(), saveCode.c_str());
            std::remove(txtIndexPath.c_str());
        }
    }

    std::cout << "Play is Over\n";
    if (repeatNum == currentIterations)
    {
        flagObj.setAddStanceFlag("isHome"); // 연주 종료 후 Home 으로 이동
        repeatNum = 1;
        currentIterations = 1;
    }
    else
    {
        flagObj.setAddStanceFlag("isReady"); // Play 반복 시 Ready 으로 이동
    }
    
    state.main = Main::AddStance;
}

////////////////////////////////////////////////////////////////////////////////
/*                                                                            */
////////////////////////////////////////////////////////////////////////////////

void DrumRobot::generateCodeFromMIDI(std::string midPath, std::string veloPath, int recordingIndex, bool endFlag)
{
    // 경로 설정
    filesystem::path outputPath1 = "../include/magenta/output1_drum_hits_time.csv"; 
    filesystem::path outputPath2 = "../include/magenta/output2_mc.csv";   
    filesystem::path outputPath3 = "../include/magenta/output3_mc2c.csv";    
    filesystem::path outputPath4 = "../include/magenta/output4_hand_assign.csv";

    filesystem::path outputPath5 = "../include/magenta/output5_vel.txt";
    filesystem::path outputPath6 = "../include/magenta/output6_add_groove.txt";

    filesystem::path outputPath = magentaCodePath + std::to_string(recordingIndex) + ".txt";

    std::string outputVel = "../include/magenta/vel_output.txt";

    // mid 파일 받아서 악보 생성하기
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
    bool mapTo357 = false;
    vector<Functions::Seg> segs;

    fun.roundDurationsToStep(bpm, outputPath1, outputPath2); 
    fun.convertMcToC(outputPath2, outputPath3);
    fun.assignHandsToEvents(outputPath3, outputPath4);

    if (veloPath != "null")
    {
        // veloPath 세기 파일 outputFile 우리가 쓸 아웃풋 파일
        fun.analyzeVelocityWithLowPassFilter(veloPath, outputVel, bpm);

        // 위에서 만든 아웃풋 파일 넣어주기 그럼 segs 에 필터씌운 정보 저장댐
        fun.loadSegments(outputVel, segs);

        // 수정전 악보 scoreIn 최종 출력 파일 scoreOut
        fun.applyIntensityToScore(segs, outputPath4, outputPath5, mapTo357);

        // 그루브 추가 
        fun.addGroove(bpm, outputPath5, outputPath6);
        
        fun.convertToMeasureFile(outputPath6, outputPath, endFlag);

        std::remove(outputPath1.c_str());      // 중간 단계 txt 파일 삭제
        std::remove(outputPath2.c_str());
        std::remove(outputPath3.c_str());
        std::remove(outputPath4.c_str());

        std::remove(outputPath5.c_str());
        std::remove(outputPath6.c_str());
        std::remove(outputVel.c_str());
    }
    else
    {
        fun.convertToMeasureFile(outputPath4, outputPath, endFlag);

        std::remove(outputPath1.c_str());      // 중간 단계 txt 파일 삭제
        std::remove(outputPath2.c_str());
        std::remove(outputPath3.c_str());
        std::remove(outputPath4.c_str());
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

////////////////////////////////////////////////////////////////////////////////
/*                                  DXL                                       */
////////////////////////////////////////////////////////////////////////////////

DXL::DXL()
{
    port = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
    pkt = dynamixel::PacketHandler::getPacketHandler(2.0);
}

DXL::~DXL()
{
    
}

void DXL::initialize()
{
    // Open port
    if (port->openPort())
    {
        printf("[DXL] ------ Open Port");
    }
    else
    {
        printf("[DXL] Failed to open the port!\n");
        useDXL = false;
        return;
    }

    // Set port baudrate
    if (port->setBaudRate(4500000))
    {
        printf(" ------ change the baudrate!\n");
    }
    else
    {
        printf("\n[DXL] Failed to change the baudrate!\n");
        useDXL = false;
        return;
    }

    for (int id = 0; id < 3; id++) // Dynamixel ID 범위: 0~252
    {
        uint16_t dxl_model_number = 0;
        uint8_t dxl_error = 0;

        int dxl_comm_result = pkt->ping(port, id, &dxl_model_number, &dxl_error);

        if (dxl_comm_result == COMM_SUCCESS && dxl_error == 0)
        {
            printf("[ID:%03d] Found! Model number: %d\n", id, dxl_model_number);
            motorIDs.push_back(static_cast<uint8_t>(id));
        }
    }

    // DXL 토크 ON
    uint8_t err = 0;
    for (uint8_t id : motorIDs)
    {
        pkt->write1ByteTxRx(port, id, 64, 1, &err);
    }
}

void DXL::syncWrite(vector<vector<float>> command)
{
    if (useDXL)
    {
        sw = std::make_unique<dynamixel::GroupSyncWrite>(port, pkt, 108, 12);

        int numDxl = motorIDs.size();
        for (int i = 0; i < numDxl; i++)
        {
            uint8_t id = motorIDs[i];

            // 모터별 목표 값 배열 정의
            int32_t values_motor[3];
            uint8_t param_motor[12];

            commandToValues(values_motor, command[i]);
            // memcpy를 사용해 정수 배열의 내용을 바이트 배열로 복사
            memcpy(param_motor, values_motor, sizeof(values_motor));
            
            sw->addParam(id, param_motor);
        }

        sw->txPacket();
        sw->clearParam();
    }
}

vector<uint32_t> DXL::syncRead()
{
    vector<uint32_t> dxlPos;
    uint32_t errCode = 99;

    if (useDXL)
    {
        // GroupSyncRead 생성 (주소 132 = Present Position, 길이 4byte)
        dynamixel::GroupSyncRead groupSyncRead(port, pkt, 132, 4);

        // 모터 ID 등록 (예: 1, 2)
        groupSyncRead.addParam(1);
        groupSyncRead.addParam(2);

        // 데이터 요청
        int dxl_comm_result = groupSyncRead.txRxPacket();
        if (dxl_comm_result != COMM_SUCCESS)
        {
            std::cerr << "SyncRead failed: "
                    << pkt->getTxRxResult(dxl_comm_result) << std::endl;

            dxlPos.push_back(errCode);
            return dxlPos;
        }

        // 각 모터 값 읽어서 출력
        for (int id : {1, 2})
        {
            if (groupSyncRead.isAvailable(id, 132, 4))
            {
                uint32_t pos = groupSyncRead.getData(id, 132, 4);
                dxlPos.push_back(pos);
            }
            else
            {
                std::cerr << "[ID:" << id << "] data not available!" << std::endl;
            }
        }

        // 다음 사용 위해 clear
        groupSyncRead.clearParam();
    }

    return dxlPos;
}

int32_t DXL::angleToTick(float angle)
{
    float degree = angle * 180.0 / M_PI;
    degree = std::clamp(degree, -180.f, 180.f);
    const float ticks_per_degree = 4096.0 / 360.0;
    float ticks = 2048.0 - (degree * ticks_per_degree);

    return static_cast<int32_t>(std::round(ticks));
}

float DXL::tickToAngle(int32_t ticks)
{
    const float degrees_per_tick = 360.0 / 4096.0;
    float angle = (2048.0 - static_cast<double>(ticks)) * degrees_per_tick;
    return angle * M_PI / 180.0;
}

void DXL::commandToValues(int32_t values[], vector<float> command)
{
    // Profile Acceleration
    values[0] = static_cast<int32_t>(1000*command[0]);  // ms
    
    // Profile Velocity
    values[1] = static_cast<int32_t>(1000*command[1]);  // ms

    // Goal Position
    values[2] = angleToTick(command[2]);
}

////////////////////////////////////////////////////////////////////////////////
/*                                  Arduino                                   */
////////////////////////////////////////////////////////////////////////////////

// 연결 함수
bool DrumRobot::ArduinoConnect(const char* port_name) 
{
    // 이미 연결되어 있다면 아무것도 하지 않음
    if (is_connected) {
        std::cout << "이미 연결되어 있습니다." << std::endl;
        return true;
    }

    // 1. 시리얼 포트 열기
    arduino_port = open(port_name, O_RDWR);

    if (arduino_port < 0) {
        std::cerr << "에러: 시리얼 포트를 열 수 없습니다. (" << port_name << ")" << std::endl;
        std::cerr << strerror(errno) << std::endl;
        return false; // 실패
    }
    
    // 2. 시리얼 포트 설정 (termios 구조체 사용)
    struct termios tty;

    // 현재 포트 설정을 읽어옴
    if (tcgetattr(arduino_port, &tty) != 0) {
        std::cerr << "에러: 포트 설정을 읽어오는 데 실패했습니다: " << strerror(errno) << std::endl;
        close(arduino_port); // 포트를 열었으므로 닫아줘야 함
        return false;
    }

    // --- 통신 설정 시작 ---
    // (a) 통신 속도(Baud Rate) 설정: 9600 bps
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // (b) 표준 설정 (8N1: 8 데이터 비트, 패리티 없음, 1 스톱 비트)
    tty.c_cflag &= ~PARENB; // 패리티 비트 비활성화 (No Parity)
    tty.c_cflag &= ~CSTOPB; // 스톱 비트 1개로 설정 (1 Stop bit)
    tty.c_cflag &= ~CSIZE;  // 데이터 비트 크기 필드를 먼저 초기화
    tty.c_cflag |= CS8;     // 데이터 비트 8개로 설정 (8 Data bits)

    // (c) 하드웨어 흐름 제어(Flow Control) 비활성화
    tty.c_cflag &= ~CRTSCTS;

    // (d) 로컬 모드 및 수신 활성화 (필수)
    tty.c_cflag |= CREAD | CLOCAL;

    // (e) 로우(Raw) 모드 설정 (가장 중요)
    // 아두이노와 통신할 때는 운영체제가 데이터를 가공하지 않도록 설정해야 함
    tty.c_lflag &= ~ICANON; // Canonical 모드 비활성화
    tty.c_lflag &= ~ECHO;   // 입력된 문자를 다시 보내지 않음 (Echo off)
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;   // 제어 문자(Ctrl+C 등) 무시

    // (f) 소프트웨어 흐름 제어 비활성화
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    // (g) 출력 데이터 가공 비활성화
    tty.c_oflag &= ~OPOST;

    if (tcsetattr(arduino_port, TCSANOW, &tty) != 0) {
        std::cerr << "에러: 포트 설정을 적용하는 데 실패했습니다." << std::endl;
        close(arduino_port);
        return false; // 실패
    }

    std::cout << "시리얼 포트(" << port_name << ") 연결 및 설정 완료." << std::endl;
    is_connected = true;
    return true; // 성공
}

// 연결 해제 함수
void DrumRobot::ArduinoDisconnect() 
{
    if (is_connected) {
        close(arduino_port);
        is_connected = false;
        arduino_port = -1;
        std::cout << "시리얼 포트 연결을 해제했습니다." << std::endl;
    }
}

// 명령 전송 함수
bool DrumRobot::sendArduinoCommand(int command_num) 
{
    // 1 : yellow -> green / offset setting
    // 2 : green / ready
    // 3 : blue / play
    // 연결이 안 되어있으면 에러
    if (!is_connected) {
        std::cerr << "에러: 아두이노가 연결되지 않아 명령을 보낼 수 없습니다." << std::endl;
        return false;
    }

    // int를 char로 변환 (0~9 사이의 숫자만 가능)
    // 예: 숫자 1 -> 문자 '1' (ASCII 49)
    if (command_num < 0 || command_num > 9) {
        std::cerr << "에러: 0-9 사이의 숫자만 보낼 수 있습니다." << std::endl;
        return false;
    }
    char msg_to_send = command_num + '0';       // 문자 '0'은 숫자 48에 해당됨 

    std::cout << arduino_port << std::endl;
    // 변환된 문자를 아두이노로 전송
    int bytes_written = write(arduino_port, &msg_to_send, 1);

    if (bytes_written < 0) {
        std::cerr << "에러: 데이터 쓰기에 실패했습니다." << std::endl;
        return false;
    }

    // std::cout << "아두이노로 명령 '" << msg_to_send << "' 전송 완료." << std::endl;
    return true;
}

void DrumRobot::setHeadLED(std::string action)
{
    if (action == "POWER_ON")
    {
        sendArduinoCommand(1);
    }
    else if (action == "IDLE")
    {
        sendArduinoCommand(2);
    }
    else if (action == "PLAYING")
    {
        sendArduinoCommand(3);
    }
    else if (action == "POWER_OFF")
    {
        sendArduinoCommand(4);
    }
}