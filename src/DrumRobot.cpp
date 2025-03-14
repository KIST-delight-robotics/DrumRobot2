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
    // SendMaxon = chrono::system_clock::now();
    addStandard = chrono::system_clock::now();

    sendLoopPeriod = std::chrono::steady_clock::now();
    recvLoopPeriod = std::chrono::steady_clock::now();
    stateMachinePeriod = std::chrono::steady_clock::now();

}

////////////////////////////////////////////////////////////////////////////////
/*                                SYSTEM LOOPS                                */
////////////////////////////////////////////////////////////////////////////////

void DrumRobot::stateMachine()
{
    while (state.main != Main::Shutdown)
    {
        stateMachinePeriod = std::chrono::steady_clock::now();
        stateMachinePeriod += std::chrono::microseconds(5000);    // 주기 : 5ms

        switch (state.main.load())
        {
        case Main::SystemInit:
        {
            initializeMotors(); //팔 T모터
            initializecanManager();
            motorSettingCmd(); //손목
            canManager.setSocketNonBlock();
            usbio.initUSBIO4761();
            fun.openCSVFile();

            std::cout << "System Initialize Complete [ Press Enter ]\n";
            getchar();
            state.main = Main::Ideal;
            break;
        }
        case Main::Ideal:
        {
            clearBufferforRecord();
            idealStateRoutine();
            break;
        }
        case Main::Play:
        {
            checkUserInput();
            break;
        }
        case Main::Test:
        {
            bool isWriteError = false;
            if (state.test == TestSub::SelectParamByUser || state.test == TestSub::SetQValue || state.test == TestSub::SetXYZ || (state.test == TestSub::TestMaxon && !testManager.hitTest))
            {
                if (!canManager.checkAllMotors_Fixed()) // stateMachine() 주기가 5ms 라서 delay 필요 없음
                {
                    isWriteError = true;
                }
            }
            else
            {
                checkUserInput();
            }

            if (isWriteError)
            {
                state.main = Main::Error;
            }
            //checkUserInput();
            break;
        }
        case Main::Pause:
        {
            checkUserInput();
            break;
        }
        case Main::AddStance:
        {
            checkUserInput();
            break;
        }
        case Main::Error:
        {
            state.main = Main::Shutdown;
            break;
        }
        case Main::Shutdown:
            break;
        }
        
        std::this_thread::sleep_until(stateMachinePeriod);
    }

    if (usbio.useUSBIO)
    {
        usbio.exitUSBIO4761();
    }
    canManager.setSocketBlock();
    deactivateControlTask();
}

void DrumRobot::sendLoopForThread()
{
    initializePathManager();
    while (state.main != Main::Shutdown)
    {
        sendLoopPeriod = std::chrono::steady_clock::now();
        sendLoopPeriod += std::chrono::microseconds(100);  // 주기 : 100us

        switch (state.main.load())
        {
        case Main::SystemInit:
        {
            break;
        }
        case Main::Ideal:
        {
            bool isWriteError = false;
            if (settingInitPos)
            {
               if (!canManager.checkAllMotors_Fixed())
               {
                   isWriteError = true;
               }
            }
            // else
            // {
            //    canManager.checkMaxon();
            // }
            
            if (isWriteError)
            {
               state.main = Main::Error;
            }
            usleep(5000);   // sendLoopForThread() 주기가 100us 라서 delay 필요
            break;
        }
        case Main::Play:
        {
            unfixedMotor();
            sendPlayProcess(5000, musicName);
            break;
        }
        case Main::AddStance:
        {
            unfixedMotor();
            sendAddStanceProcess(5000);
            break;
        }
        case Main::Test:
        {
            //unfixedMotor();
            testManager.SendTestProcess(5000);
            break;
        }
        case Main::Pause:
        {
            bool isWriteError = false;
            if (!canManager.checkAllMotors_Fixed())
            {
                isWriteError = true;
            }
            if (isWriteError)
            {
                state.main = Main::Error;
            }
            usleep(5000);  // sendLoopForThread() 주기가 100us 라서 delay 필요
            break;
        }
        case Main::Error:
        {
            break;
        }
        case Main::Shutdown:
            break;
        }

        std::this_thread::sleep_until(sendLoopPeriod);
    }
}

void DrumRobot::recvLoopForThread()
{
    while (state.main != Main::Shutdown)
    {
        recvLoopPeriod = std::chrono::steady_clock::now();
        recvLoopPeriod += std::chrono::microseconds(100);  // 주기 : 100us

        switch (state.main.load())
        {
        case Main::SystemInit:
        {
            break;
        }
        case Main::Ideal:
        {
            readProcess(5000); /*1ms*/
            break;
        }
        case Main::Play:
        {
            readProcess(5000);
            break;
        }
        case Main::AddStance:
        {
            readProcess(5000);
            break;
        }
        case Main::Test:
        {
            readProcess(5000);
            break;
        }
        case Main::Pause:
        {
            readProcess(5000); // 1ms마다 실행
            break;
        }
        case Main::Error:
        {
            break;
        }
        case Main::Shutdown:
        {
            break;
        }
        }

        std::this_thread::sleep_until(recvLoopPeriod);
    }
}

void DrumRobot::readProcess(int periodMicroSec)
{
    // std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motors["L_wrist"]);
    // std::shared_ptr<GenericMotor> motor = motors["L_wrist"];

    // struct can_frame frame;

    auto currentTime = chrono::system_clock::now();
    auto elapsedTime = chrono::duration_cast<chrono::microseconds>(currentTime - ReadStandard);
    
    switch (state.read.load())
    {
    case ReadSub::TimeCheck:
        if (elapsedTime.count() >= periodMicroSec)
        {
            state.read = ReadSub::ReadCANFrame; // 주기가 되면 ReadCANFrame 상태로 진입
            ReadStandard = currentTime;         // 현재 시간으로 시간 객체 초기화

            // maxoncmd.getActualPos(*maxonMotor, &frame); //Maxon모터에게 1ms마다 신호
            // canManager.txFrame(motor, frame); // CAN을 통해 보내줌
        }
        break;
    case ReadSub::ReadCANFrame:
        canManager.readFramesFromAllSockets(); // CAN frame 읽기
        state.read = ReadSub::UpdateMotorInfo; // 다음 상태로 전환
        break;
    case ReadSub::UpdateMotorInfo:
    {
        // if ((!settingInitPos) || state.main == Main::Test) // test 모드에서 에러 검출 안함
        // {
        //     canManager.distributeFramesToMotors(false);
        // }
        if (!settingInitPos)    // test 모드에서 에러 검출함
        {
            canManager.distributeFramesToMotors(false);
        }
        else
        {
            bool isSafe = canManager.distributeFramesToMotors(true);
            if (!isSafe)
            {
                state.main = Main::Error;
            }
        }

        state.read = ReadSub::TimeCheck;
        break;
    }
    }
}

void DrumRobot::sendPlayProcess(int periodMicroSec, string musicName)
{
    auto currentTime = chrono::system_clock::now();
    auto elapsedTime = chrono::duration_cast<chrono::microseconds>(currentTime - SendStandard);
    // auto elapsedTimeMaxon = chrono::duration_cast<chrono::microseconds>(currentTime - SendMaxon);
    
    switch (state.play.load())
    {
    case PlaySub::ReadMusicSheet:
    {
        // 파일을 처음 열 때만
        if (openFlag == 1)
        {
            openFlag = 0; // 파일 열기 상태 초기화
            std::string currentFile = basePath + musicName + std::to_string(fileIndex) + ".txt";
            inputFile.open(currentFile); // 파일 열기
            
            if (!inputFile.is_open()) // 파일 열기 실패
            {
                if(pathManager.measureMatrix.rows() > 1)
                {
                    // 악보 남음
                    state.play = PlaySub::GenerateTrajectory;
                    break;
                }
                else
                {
                    if (pathManager.trajectoryQueue.empty())
                    {
                        // 연주 종료
                        std::cout << "Play is Over\n";
                        state.main = Main::AddStance;
                        state.play = PlaySub::ReadMusicSheet;
                        canManager.isPlay = false;
                        setAddStanceFlag("goToHome");
                        usleep(500*1000);     // 0.5s
                        break; // 파일 열지 못했으므로 상태 변경 후 종료
                    }
                    else
                    {
                        state.play = PlaySub::SolveIK;
                        break;
                    }
                }
            }
        }

        // 파일에서 읽고 measureMatrix가 2.4초 이상이 되도록 추가
        if (pathManager.readMeasure(inputFile, bpmFlag) == true)
        {
            state.play = PlaySub::GenerateTrajectory; // GenerateTrajectory 상태로 전환
            break;
        }
        else    // 파일 끝에 도달한 경우
        {
            inputFile.close(); // 파일 닫기
            fileIndex++;       // 다음 파일로 이동
            openFlag = 1;      // 파일 열 준비

            state.play = PlaySub::ReadMusicSheet;
            break;
        }

        state.play = PlaySub::SolveIK;

        break;
    
    }
    case PlaySub::GenerateTrajectory:
    {
        pathManager.lineOfScore++;
        pathManager.generateTrajectory();
        if (pathManager.lineOfScore > preCreatedLine)
        {
            state.play = PlaySub::SolveIK;
        }
        else
        {
            state.play = PlaySub::ReadMusicSheet;
        }

        break;
    }
    case PlaySub::SolveIK:
    {
        // 정해진 개수만큼 커맨드 생성
        if (pathManager.solveIKandPushConmmand())
        {
            state.play = PlaySub::TimeCheck;
        }
        else
        {
            state.play = PlaySub::ReadMusicSheet;
        }

        break;
    }
    case PlaySub::TimeCheck:
    {
        if (elapsedTime.count() >= periodMicroSec)  
        {
            state.play = PlaySub::SetCANFrame;  // 5ms마다 CAN Frame 설정
            SendStandard = currentTime;         // 시간 초기화
            SendMaxon = currentTime;             // 시간 초기화
        }
        break;
    }
    case PlaySub::SetCANFrame:
    {
        bool isSafe;
        isSafe = canManager.setCANFrame();
        if (!isSafe)
        {
            state.main = Main::Error;
        }
        state.play = PlaySub::SendCANFrame;
        break;
    }
    case PlaySub::SendCANFrame:
    {
        bool isWriteError = false;
        for (auto &motor_pair : motors)
        {
            shared_ptr<GenericMotor> motor = motor_pair.second;

            if (!canManager.sendMotorFrame(motor))
            {
                isWriteError = true;
            }

            // if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
            // {
            //     usbio.setUSBIO4761(motorMapping[motor_pair.first], tMotor->brakeState);
            // }
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
        else
        {
            state.play = PlaySub::SolveIK;
        }

        // brake
        if(!usbio.outputUSBIO4761())
        {
            cout << "brake Error\n";
        }
        break;
    }
    }
}

void DrumRobot::sendAddStanceProcess(int periodMicroSec)
{
    auto currentTime = chrono::system_clock::now();
    auto elapsedTime = chrono::duration_cast<chrono::microseconds>(currentTime - addStandard);

    switch (state.addstance.load())
    {
    case AddStanceSub::CheckCommand:
    {
        if (goToHome || goToReady || goToShutdown || goToSemiReady)
        {
            clearBufferforRecord();
            clearMotorsCommandBuffer();
            state.addstance = AddStanceSub::FillBuf;
        }
        else
        {
            state.main = Main::Ideal;
        }
        break;
    }
    case AddStanceSub::FillBuf:
    {
        if (goToReady)
        {
            std::cout << "Get Ready Pos Array ...\n";
            pathManager.getArr(pathManager.readyArr);
        }
        else if (goToSemiReady)
        {
            std::cout << "Get Semi Ready Pos Array ...\n";
            pathManager.getArr(pathManager.homeArr);
            sleep(1);
        }
        else if (goToHome)
        {
            hommingCnt++;
            vector<float> home_arr = pathManager.makeHomeArr(hommingCnt);
            std::cout << "Get Home Pos Array " << hommingCnt << "...\n";
            pathManager.getArr(home_arr);
        }
        else if (goToShutdown)
        {
            std::cout << "Get Back Pos Array ...\n";
            pathManager.getArr(pathManager.backArr);
        }

        state.addstance = AddStanceSub::TimeCheck;
        break;
    }
    case AddStanceSub::TimeCheck:
    {
        if (elapsedTime.count() >= periodMicroSec)  
        {
            state.addstance = AddStanceSub::CheckBuf;  // 5ms마다 SetCANFrame 실행
            addStandard = currentTime;
        }
        break;
    }
    case AddStanceSub::CheckBuf:
    {
        bool allBuffersEmpty = true;

        for (const auto &motor_pair : motors)
        {
            if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
            {
                if (!maxonMotor->commandBuffer.empty())
                {
                    allBuffersEmpty = false;
                    break;
                }
            }
            else if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
            {
                if (!tMotor->commandBuffer.empty())
                {
                    allBuffersEmpty = false;
                    break;
                }
            }
        }

        if (!allBuffersEmpty)
        {
            state.addstance = AddStanceSub::SetCANFrame;
        }
        else
        {
            if (goToShutdown)
            {
                state.main = Main::Shutdown;
            }
            else if (goToHome)
            {
                if (hommingCnt >= 2)
                {
                    state.addstance = AddStanceSub::CheckCommand;
                    state.main = Main::Ideal;
                    canManager.clearReadBuffers();
                    setRobotFlag("isHome");
                }
                else
                {
                    state.addstance = AddStanceSub::FillBuf;
                }
            }
            else if (goToReady)
            {
                state.addstance = AddStanceSub::CheckCommand;
                state.main = Main::Ideal;
                canManager.clearReadBuffers();
                setRobotFlag("isReady");
            }
            else if (goToSemiReady)
            {
                state.addstance = AddStanceSub::CheckCommand;
                state.main = Main::Ideal;
                canManager.clearReadBuffers();
                setRobotFlag("isHome");
            }
        }
        break;
    }
    case AddStanceSub::SetCANFrame:
    {
        bool isSafe;
        isSafe = canManager.setCANFrame();
        if (!isSafe)
        {
            state.main = Main::Error;
        }
        state.addstance = AddStanceSub::SendCANFrame;
        break;
    }
    case AddStanceSub::SendCANFrame:
    {
        bool isWriteError = false;
        for (auto &motor_pair : motors)
        {
            shared_ptr<GenericMotor> motor = motor_pair.second;

            if (!canManager.sendMotorFrame(motor))
            {
                isWriteError = true;
            }

            if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor_pair.second))
            {
                usbio.setUSBIO4761(motorMapping[motor_pair.first], tMotor->brakeState);
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
        else
        {
            state.addstance = AddStanceSub::TimeCheck;
        }

        // brake
        if(!usbio.outputUSBIO4761())
        {
            cout << "brake Error\n";
        }
        break;
    }
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                                STATE UTILITY                               */
////////////////////////////////////////////////////////////////////////////////

void DrumRobot::displayAvailableCommands() const
{
    std::cout << "Available Commands:\n";

    if (state.main == Main::Ideal)
    {
        if (!settingInitPos)
        {
            std::cout << "- o : Set Zero & Offset setting\n";
            std::cout << "- i : Offset setting\n";
        }
        else
        {
            if (isHome)
            {
                std::cout << "- r : Move to Ready Pos\n";
                std::cout << "- t : Start Test\n";
                std::cout << "- s : Shut down the system\n";
            }
            else if (isReady)
            {
                std::cout << "- p : Play Drum\n";
                std::cout << "- t : Start Test\n";
                std::cout << "- h : Move to Home Pos\n";
                std::cout << "- m : Mode Select\n";
                std::cout << "- v : Release Time Value\n";
            }
            else if (isRestart)
            {
                std::cout << "- h : Move to Home Pos\n";
                std::cout << "- t : Start Test\n";
            }
        }
    }
    else
    {
        std::cout << "- s : Shut down the system\n";
    }
}

bool DrumRobot::processInput(const std::string &input)
{
    if (state.main == Main::Ideal)
    {
        if (!settingInitPos)
        {
            if (input == "o")
            {
                // set zero
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
                sleep(2);   // setZero 명령이 확실히 실행된 후 fixed 함수 실행

                maxonMotorEnable();
                setMaxonMotorMode("CSP");

                settingInitPos = true;

                state.main = Main::AddStance;
                setRobotFlag("MOVING");
                setAddStanceFlag("goToSemiReady");

                return true;
            }
            else if (input == "i")
            {
                maxonMotorEnable();
                setMaxonMotorMode("CSP");

                settingInitPos = true;
                setRobotFlag("isRestart");

                return true;
            }
        }
        else
        {
            if (input == "r" && isHome)
            {
                state.main = Main::AddStance;
                setRobotFlag("MOVING");
                setAddStanceFlag("goToReady");

                return true;
            }
            else if (input == "v" && isReady)
            {
                std::cout << "Release Time Value : " << pathManager.releaseTimeVal;
                std::cout << "\nrange : 0.6 ~ 1.5 \n";
                std::cout << "enter Release TIme Value : ";
                cin >> pathManager.releaseTimeVal;
                if (pathManager.releaseTimeVal < 0.6 || pathManager.releaseTimeVal > 1.5)
                {
                    std::cout << "Release Time Value Error";
                }
            }
            else if (input == "m" && isReady)
            {
                int mode = 0;
                std::cout << "1 : CSP hitting \n";
                std::cout << "2 : CSP hitting Detect \n";
                std::cout << "3 : CSP hitting Detect (fast ver) \n";
                std::cout << "4 : CST hitting Detect \n";
                std::cout << "enter hitting mode : ";
                std::cin >> mode;

                if (mode == 1)
                {
                    pathManager.hitMode = 1;
                    canManager.isCST = false;
                }
                else if (mode == 2)
                {
                    pathManager.hitMode = 2;
                    canManager.isCST = false;
                }
                else if (mode == 3)
                {
                    pathManager.hitMode = 3;
                    canManager.isCST = false;  
                }
                else if (mode == 4)
                {
                    pathManager.hitMode = 4;
                }
            }
            else if (input == "p" && isReady)
            {
                std::cout << "enter music name : ";
                std::getline(std::cin, musicName);
                
                bpmFlag = 0;
                fileIndex = 0;
                openFlag = 1;
                state.main = Main::Play;
                canManager.isPlay = true;
                setRobotFlag("MOVING");
                return true;
            }
            else if (input == "h")
            {
                if (isReady || isRestart)
                {
                    state.main = Main::AddStance;
                    setRobotFlag("MOVING");
                    setAddStanceFlag("goToHome");

                    return true;

                }
            }
            else if (input == "s" && isHome)
            {
                state.main = Main::AddStance;
                setRobotFlag("MOVING");
                setAddStanceFlag("goToShutdown");

                return true;
            }
            else if (input == "t")
            {
                state.main = Main::Test;

                return true;
            }
        }
    }
    else
    {
        if (input == "s")
        {
            state.main = Main::Shutdown;
        }
    }

    return false;
}

void DrumRobot::idealStateRoutine()
{
    int ret = system("clear");
    if (ret == -1)
        std::cout << "system clear error" << endl;

    displayAvailableCommands();

    std::string input;

    std::cout << "Enter command: ";
    std::getline(std::cin, input);


    if (!processInput(input))
        std::cout << "Invalid command or not allowed in current state!\n";

    // usleep(2000);    // sleep_until()
}

void DrumRobot::checkUserInput()
{
    bool isWriteError = false;
    if (kbhit())
    {
        char input = getchar();
        if (state.main == Main::Play || state.main == Main::Pause)
        {
            if (input == 'q')
            {
                state.main = Main::Pause;

                if (!canManager.checkAllMotors_Fixed())
                {
                    isWriteError = true;
                }
            }
            else if(input == 's'){
                pathManager.lineOfScore = 0;
                state.main = Main::Shutdown;
            }
            else if (input == 'p') // restart의 의미, ready상태와 명령어 구분을 위함.
            {
                state.main = Main::Play;
            }
            else if (input == 'h')
            {
                state.play = PlaySub::ReadMusicSheet;
                inputFile.close(); // 파일 닫기
                while (!pathManager.trajectoryQueue.empty())
                {
                    pathManager.trajectoryQueue.pop();
                }

                state.main = Main::AddStance;
                setAddStanceFlag("goToHome");
            }
        }
        else if (state.main == Main::Error)
        {
            state.main = Main::Shutdown;
        }
    }
    if (isWriteError)
    {
        state.main = Main::Error;
    }
    // usleep(5000);    // sleep_until()
}

int DrumRobot::kbhit()
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
/*                                 SYSTEM                                     */
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
};

void DrumRobot::initializecanManager()
{
    canManager.initializeCAN();
    canManager.checkCanPortsStatus();
    canManager.setMotorsSocket();
}

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

void DrumRobot::printCurrentPositions()
{
    for (auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        auto &motor = motorPair.second;
        std::cout << "[" << std::hex << motor->nodeId << std::dec << "] ";
        std::cout << name << " Pos: " << motor->motorPosition << endl;
    }

    vector<float> P(6);
    P = pathManager.FK();

    std::cout << "Right Hand Position : { " << P[0] << " , " << P[1] << " , " << P[2] << " }\n";
    std::cout << "Left Hand Position : { " << P[3] << " , " << P[4] << " , " << P[5] << " }\n";
}

void DrumRobot::setMaxonMode(std::string targetMode)
{
    struct can_frame frame;
    canManager.setSocketsTimeout(0, 10000);
    for (const auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<GenericMotor> motor = motorPair.second;
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motorPair.second))
        {
            if (targetMode == "CSV")
            {
                maxoncmd.getCSVMode(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
            else if (targetMode == "CST")
            {
                maxoncmd.getCSTMode(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
            else if (targetMode == "HMM")
            {
                maxoncmd.getHomeMode(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
            else if (targetMode == "CSP")
            {
                maxoncmd.getCSPMode(*maxonMotor, &frame);
                canManager.sendAndRecv(motor, frame);
            }
        }
    }
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

////////////////////////////////////////////////////////////////////////////////
/*                                 Send Thread Loop                           */
////////////////////////////////////////////////////////////////////////////////

void DrumRobot::initializePathManager()
{
    pathManager.getDrumPositoin();
    pathManager.setReadyAngle();
}

void DrumRobot::clearMotorsSendBuffer()
{
    for (auto motor_pair : motors)
        motor_pair.second->clearSendBuffer();
}

void DrumRobot::clearMotorsCommandBuffer()
{
    for (const auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<GenericMotor> motor = motorPair.second;
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

void DrumRobot::unfixedMotor()
{
    for (auto motor_pair : motors)
        motor_pair.second->isfixed = false;
}

/////////////////////////////////////////////////////////////////////////////////
/*                            flag setting                                     */
/////////////////////////////////////////////////////////////////////////////////

void DrumRobot::setRobotFlag(string flag)
{

    if (flag == "MOVING")
    {
        isHome = false;
        isReady = false;
        isRestart = false;
    }
    else if (flag == "isHome")
    {
        isHome = true;
        isReady = false;
        isRestart = false;
    }
    else if (flag == "isReady")
    {
        isHome = false;
        isReady = true;
        isRestart = false;
    }
    else if (flag == "isRestart")
    {
        isHome = false;
        isReady = false;
        isRestart = true;
    }
    else
    {
        cout << "Invalid flag\n";
    }
}

void DrumRobot::setAddStanceFlag(string flag)
{
    if (flag == "goToReady")
    {
        goToReady = true;
        goToHome = false;
        goToSemiReady = false;
        goToShutdown = false;
    }
    else if (flag == "goToHome")
    {
        goToReady = false;
        goToHome = true;
        goToShutdown = false;
        goToSemiReady = false;

        hommingCnt = 0;
    }
    else if (flag == "goToSemiReady")
    {
        goToReady = false;
        goToHome = false;
        goToSemiReady = true;
        goToShutdown = false;
    }
    else if (flag == "goToShutdown")
    {
        goToReady = false;
        goToHome = false;
        goToShutdown = true;
        goToSemiReady = false;
    }
    else
    {
        cout << "Invalid flag\n";
    }
}

/////////////////////////////////////////////////////////////////////////////////
/*                         Maxon Motor Function                               */
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