#include "../include/managers/TestManager.hpp" // 적절한 경로로 변경하세요.

// For  Qt
// #include "../managers/TestManager.hpp"
using namespace std;

TestManager::TestManager(State &stateRef, CanManager &canManagerRef, std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef, USBIO &usbioRef, Functions &funRef)
    : state(stateRef), canManager(canManagerRef), motors(motorsRef), usbio(usbioRef), fun(funRef)
{

    standardTime = chrono::system_clock::now();
}

void TestManager::SendTestProcess(int periodMicroSec)
{
    auto currentTime = chrono::system_clock::now();
    auto elapsedTime = chrono::duration_cast<chrono::microseconds>(currentTime - standardTime);

    // 선택에 따라 testMode 설정
    switch (state.test.load())
    {
        case TestSub::SelectParamByUser:
        {
            int ret = system("clear");
            if (ret == -1)
                std::cout << "system clear error" << endl;

            float c_MotorAngle[10];
            getMotorPos(c_MotorAngle);

            std::cout << "[ Current Q Values (Radian / Degree) ]\n";
            for (int i = 0; i < 10; i++)
            {
                q[i] = c_MotorAngle[i];
                std::cout << "Q[" << i << "] : " << c_MotorAngle[i] << "\t\t" << c_MotorAngle[i] * 180.0 / M_PI << "\n";
            }
            FK(c_MotorAngle); // 현재 q값에 대한 FK 진행

            std::cout << "\nSelect Method (1 - 관절각도값 조절, 2 - 좌표값 조절, 3 - 손목 모터, 4 - TABlE TEST, -1 - 나가기) : ";
            std::cin >> method;
            
            if (method == 1)
            {
                state.test = TestSub::SetQValue;
            }
            else if (method == 2)
            {
                state.test = TestSub::SetXYZ;
            }
            else if (method == 3)
            {
                state.test = TestSub::TestMaxon;
            }
            else if (method == 4)
            {
                testTable();
            }
            else if (method == -1)
            {
                state.main = Main::Ideal;
            }

            break;
        }
        case TestSub::SetQValue:
        {
            int userInput = 100;
            int ret = system("clear");
            if (ret == -1)
                std::cout << "system clear error" << endl;

            float c_MotorAngle[10] = {0};
            getMotorPos(c_MotorAngle);

            if(sin_flag)
            {
                std::cout << "Mode : Sin Wave\n";
            }
            else
            {
                std::cout << "Mode : Go to target point\n";
            }
            std::cout << "\n[Current Q Values] [Target Q Values] (Radian)\n";
            for (int i = 0; i < 10; i++)
            {
                std::cout << "Q[" << i << "] : " << c_MotorAngle[i] << "\t<->\t" << q[i] << std::endl;
            }

            std::cout << "\ntime : " << t << "s";
            if(!sin_flag) std::cout << " + " << extra_time << "s";
            std::cout << "\nnumber of repeat : " << n_repeat << std::endl << std::endl;

            for (int i = 0; i < 7; i++)
            {
                if (brake_flag[i])
                {
                    std::cout << "Joint " << i << " brake on : " << brake_start_time[i] << "s ~ " << brake_end_time[i] << "s\n";
                }
                else
                {
                    std::cout << "Joint " << i << " brake off\n";
                }
            }
            
            std::cout << "\nSelect Motor to Change Value (0-8) / Run (9) / Time (10) / Extra Time (11) / Repeat(12) / Brake (13) / initialize test (14) / Sin Profile (15) / break on off (16) / Exit (-1): ";
            std::cin >> userInput;

            if (userInput == -1)
            {
                state.test = TestSub::SelectParamByUser;
            }
            else if (userInput < 9)
            {
                float degree_angle;
                
                std::cout << "\nRange : " << jointRangeMin[userInput] << "~" << jointRangeMax[userInput] << "(Degree)\n";
                std::cout << "Enter q[" << userInput << "] Values (Degree) : ";
                std::cin >> degree_angle;
                q[userInput] = degree_angle * M_PI / 180.0;
            }
            else if (userInput == 9)
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
                state.test = TestSub::FillBuf;
                usleep(5000);
                unfixedMotor();
            }
            else if (userInput == 10)
            {
                std::cout << "time : ";
                std::cin >> t;
            }
            else if (userInput == 11)
            {
                std::cout << "extra time : ";
                std::cin >> extra_time;
            }
            else if (userInput == 12)
            {
                std::cout << "number of repeat : ";
                std::cin >> n_repeat;
            }
            else if (userInput == 13)
            {
                int input_brake;
                std::cout << "Select joint : ";
                std::cin >> input_brake;

                if(input_brake < 7)
                {
                    if(brake_flag[input_brake])
                    {
                        brake_flag[input_brake] = false;
                    }
                    else
                    {
                        brake_flag[input_brake] = true;

                        std::cout << "brake start time (0~" << t+extra_time << ") : ";
                        std::cin >> brake_start_time[input_brake];

                        std::cout << "brake end time (" << brake_start_time[input_brake] << "~" << t+extra_time << ") : ";
                        std::cin >> brake_end_time[input_brake];
                    }
                }
            }
            else if (userInput == 14)
            {
                for (int i = 0; i <= 6; ++i)
                {
                    brake_flag[i] = false;
                    float degree_angle;
                    
                    // 1과 2는 90도, 나머지는 0도로 설정
                    if (i == 1 || i == 2) {
                        degree_angle = 90.0;
                    } else {
                        degree_angle = 0.0;
                    }
                    
                    std::cout << "\nRange : " << jointRangeMin[i] << "~" << jointRangeMax[i] << "(Degree)\n";
                    std::cout << "Enter q[" << i << "] Values (Degree) : " << degree_angle << "\n";
                    
                    // degree 값을 radian으로 변환하여 q 배열에 저장
                    q[i] = degree_angle * M_PI / 180.0;
                }
                t = 4.0;
                extra_time = 1.0;
                n_repeat = 1;
                sin_flag = false;

                state.test = TestSub::FillBuf;
                usleep(5000);
                unfixedMotor();

            }

            else if (userInput == 15)
            {
                if(sin_flag)
                {
                    sin_flag = false;
                    extra_time = 1.0;
                }
                else
                {
                    sin_flag = true;
                    extra_time = 0.0;
                }
            }

            // else if (userInput == 16)
            // {
            //     bool usbio_output = false;

            //     std::cout << "Current brake states:" << std::endl;
            //     for (int i = 0; i < 7; i++)
            //     {
            //         std::cout << "Brake " << i << ": " << (single_brake_flag[i] ? "Active" : "Inactive") << std::endl;
            //     }

            //     int break_num;
            //     cin >> break_num;

            //     if(!single_brake_flag[break_num])
            //     {
            //         usbio.setUSBIO4761(break_num % 7, true);
            //         usbio_output = usbio.outputUSBIO4761();
            //         single_brake_flag[break_num] = true;
            //     }
            //     else
            //     {
            //         usbio.setUSBIO4761(break_num % 7, false);
            //         usbio_output = usbio.outputUSBIO4761();
            //         single_brake_flag[break_num] = false;
            //     }

            //     if(!usbio_output)
            //     {
            //         std::cout << "OUTPUT Error" << endl;
            //         usleep(5000000);
            //         break;
            //     }

            // }
            break;
        }
        case TestSub::TestMaxon:
        {
            //std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motors["maxonForTest"]);
            int userInput = 100;
            int ret = system("clear");
            if (ret == -1)
                std::cout << "system clear error" << endl;
            
            float c_MotorAngle[10] = {0};
            getMotorPos(c_MotorAngle);

            curMode = "CSP";

            for (int i = 0; i < 10; i++)
            {
                q[i] = c_MotorAngle[i];
                std::cout << "Q[" << i << "] : " << c_MotorAngle[i] << "\t\t" << c_MotorAngle[i] * 180.0 / M_PI << "\n";
            }

            if(hitTest)
            {
                static int i = 0;
                static int repeatCnt = 0;
                int n = hit_time / dt;
                
                //canManager.isHit = true;

                for (auto &entry : motors)
                {
                    if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
                    {
                        MaxonData newData;
                        if (hitMode == 1)
                        {
                            if (repeatCnt == repeat)
                            {
                                q[8] = makeWristAngle(0, hit_time, i * dt, 1, intensity, maxonMotor->hitting, maxonMotor->hittingPos);
                            }
                            else if (repeatCnt == 0)
                            {
                                q[8] = makeWristAngle(0, hit_time, i * dt, 2, intensity, maxonMotor->hitting, maxonMotor->hittingPos);
                            }
                            else
                            {
                                q[8] = makeWristAngle(0, hit_time, i * dt, 3, intensity, maxonMotor->hitting, maxonMotor->hittingPos);
                            }
                        }
                        else if (hitMode == 2)
                        {
                            maxonMotor->hitting = false;
                            if (repeatCnt == repeat)
                            {
                                q[8] = makeWristAngle(0, hit_time, i * dt, 1, intensity, maxonMotor->hitting, maxonMotor->hittingPos);
                            }
                            else if (repeatCnt == 0)
                            {
                                q[8] = makeWristAngle(0, hit_time, i * dt, 2, intensity, maxonMotor->hitting, maxonMotor->hittingPos);
                            }
                            else
                            {
                                q[8] = makeWristAngle(0, hit_time, i * dt, 3, intensity, maxonMotor->hitting, maxonMotor->hittingPos);
                            }
                        }
                        else
                        {
                            if (repeatCnt == repeat)
                            {
                                q[8] = makeWristAngle_CST(0, hit_time, i * dt, 1, intensity, maxonMotor->hitting, maxonMotor->hittingPos);
                            }
                            else if (repeatCnt == 0)
                            {
                                q[8] = makeWristAngle_CST(0, hit_time, i * dt, 2, intensity, maxonMotor->hitting, maxonMotor->hittingPos);
                            }
                            else
                            {
                                q[8] = makeWristAngle_CST(0, hit_time, i * dt, 3, intensity, maxonMotor->hitting, maxonMotor->hittingPos);
                            }
                        }

                        newData.position = q[motorMapping[entry.first]];
                        maxonMotor->commandBuffer.push(newData);
                        //fun.appendToCSV_DATA("wristAngleData", (float)maxonMotor->nodeId , newData.position, 0);
                    }
                }
                if (i >= n)
                {
                    i = 0;
                    if (repeatCnt >= repeat)
                    {
                        repeatCnt = 0;
                        hitTest = false;
                        break;
                    }
                    else
                    {
                        repeatCnt++;
                    }
                }
                else
                {
                    i++;
                }
                state.test = TestSub::CheckBuf;
                unfixedMotor();
            }
            else
            {
                // canManager.isHit = false;
                cout << "\n Select Test Mode (1 - 타격 TEST, 2 - CST 모드 TEST, -1 - Back) : ";
                cin >> userInput;
            }

            if (userInput == -1)
            {
                state.test = TestSub::SelectParamByUser;
            }
            else if (userInput == 1)
            {
                params = CSTHitLoop();
                hit_time = get<0>(params);
                repeat = get<1>(params);
                intensity = get<2>(params);

                hitTest = 1;
            }
            else if (userInput == 2)
            {
                canManager.setSocketBlock();
                TestStickLoop();
                canManager.setSocketNonBlock();
                state.test = TestSub::SelectParamByUser;
            }
            break;
        }
        case TestSub::SetXYZ:
        {
            int userInput = 100;
            int ret = system("clear");
            if (ret == -1)
                std::cout << "system clear error" << endl;
            cout << "[ Current x, y, z (meter) ]\n";
            cout << "Right : ";
            for (int i = 0; i < 3; i++)
            {
                cout << R_xyz[i] << ", ";
            }
            cout << "\nLeft : ";
            for (int i = 0; i < 3; i++)
            {
                cout << L_xyz[i] << ", ";
            }

            cout << "\nSelect Motor to Change Value (1 - Right, 2 - Left) / Start Test (3) / Exit (-1) : ";
            cin >> userInput;

            if (userInput == -1)
            {
                state.test = TestSub::SelectParamByUser;
            }
            else if (userInput == 1)
            {
                cout << "Enter x, y, z Values (meter) : ";
                cin >> R_xyz[0] >> R_xyz[1] >> R_xyz[2];
            }
            else if (userInput == 2)
            {
                cout << "Enter x, y, z Values (meter) : ";
                cin >> L_xyz[0] >> L_xyz[1] >> L_xyz[2];
            }
            else if (userInput == 3)
            {
                unfixedMotor();
                state.test = TestSub::FillBuf;
            }
            break;
        }
        case TestSub::FillBuf:
        {
            // Fill motors command Buffer
            if (method == 1)
            {
                getArr(q);
            }
            else if (method == 2)
            {
                vector<float> Qf(7);
                Qf = ikfun_final(R_xyz, L_xyz, partLength, s, z0); // IK함수는 손목각도가 0일 때를 기준으로 풀림
                Qf.push_back(0.0);                                  // 오른쪽 손목 각도
                Qf.push_back(0.0);                                  // 왼쪽 손목 각도
                for (int i = 0; i < 9; i++)
                {
                    q[i] = Qf[i];
                    cout << Qf[i] << " ";
                }
                cout << "\n";
                sleep(1);
                getArr(q);
            }
            else if (method == 3)
            {
                getArr(q);
            }
            
            state.test = TestSub::CheckBuf;
            break;
        }
        case TestSub::CheckBuf:
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
                state.test = TestSub::TimeCheck;
            else
                state.test = TestSub::Done;
            break;
        }
        case TestSub::TimeCheck:
        {
            if (elapsedTime.count() >= periodMicroSec)  
            {
                // struct can_frame frame;
                // std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motors["L_wrist"]);
                // std::shared_ptr<GenericMotor> motor = motors["L_wrist"];
                // maxoncmd.getCheck(*maxonMotor, &frame);
                // canManager.sendAndRecv(motor, frame);

                state.test = TestSub::SetCANFrame;  // 5ms마다 CAN Frame 설정
                standardTime = currentTime;         // 시간 초기화
            }
            break;
        }
        case TestSub::SetCANFrame:
        {
            bool isSafe;
            isSafe = canManager.setCANFrame_TEST();
            if (!isSafe)
            {
                state.main = Main::Error;
            }
            state.test = TestSub::SendCANFrame;
            break;
        }
        case TestSub::SendCANFrame:
        {
            bool needSync = false;
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

                if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
                {
                    virtualMaxonMotor = maxonMotor;
                    needSync = true;
                }
            }

            if (needSync)
            {
                maxoncmd.getSync(&virtualMaxonMotor->sendFrame);
                if (!canManager.sendMotorFrame(virtualMaxonMotor))
                {
                    isWriteError = true;
                }
            }
            if (isWriteError)
            {
                state.main = Main::Error;
            }
            else
            {
                state.test = TestSub::CheckBuf;
            }

            // brake
            if(!usbio.outputUSBIO4761())
            {
                cout << "brake Error\n";
            }

            break;
        }
        case TestSub::Done:
        {
            //usleep(5000);
            
            if (method == 1)
            {
                state.test = TestSub::SetQValue;
                
                // std::ostringstream fileNameOut;
                // fileNameOut << std::fixed << std::setprecision(1); // 소숫점 1자리까지 표시
                // fileNameOut << "../../READ/Test_0704_P" << q[5]
                //             << "_spd" << speed_test
                //             << "_BrakeTime" << brake_start_time;
                // std::string fileName = fileNameOut.str();
                // parse_and_save_to_csv(fileName);
            }
            else if (method == 2)
            {
                state.test = TestSub::SetXYZ;
            }
            else if (method == 3)
            {
                state.test = TestSub::TestMaxon;
            }
            else
                state.test = TestSub::SelectParamByUser;

            if (error)
                state.main = Main::Error;
            break;
        }
    }
}

void TestManager::MaxonEnable()
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

            maxoncmd.getOperational(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            usleep(100000);
            maxoncmd.getEnable(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);
            std::cout << "Maxon Enabled(1) \n";

            usleep(100000);

            maxoncmd.getQuickStop(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);

            usleep(100000);
            maxoncmd.getEnable(*maxonMotor, &frame);
            canManager.txFrame(motor, frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motor, frame);

            std::cout << "Maxon Enabled(2) \n";
        }
    }
};

void TestManager::setMaxonMode(std::string targetMode)
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

void TestManager::FK(float theta[])
{
    vector<float> P;

    float r1 = partLength[0], r2 = partLength[1], l1 = partLength[2], l2 = partLength[3], stick = partLength[4];

    P.push_back(0.5 * s * cos(theta[0]) + r1 * sin(theta[3]) * cos(theta[0] + theta[1]) + r2 * sin(theta[3] + theta[4]) * cos(theta[0] + theta[1]) + stick * sin(theta[3] + theta[4] + theta[7]) * cos(theta[0] + theta[1]));
    P.push_back(0.5 * s * sin(theta[0]) + r1 * sin(theta[3]) * sin(theta[0] + theta[1]) + r2 * sin(theta[3] + theta[4]) * sin(theta[0] + theta[1]) + stick * sin(theta[3] + theta[4] + theta[7]) * sin(theta[0] + theta[1]));
    P.push_back(z0 - r1 * cos(theta[3]) - r2 * cos(theta[3] + theta[4]) - stick * cos(theta[3] + theta[4] + theta[7]));
    P.push_back(-0.5 * s * cos(theta[0]) + l1 * sin(theta[5]) * cos(theta[0] + theta[2]) + l2 * sin(theta[5] + theta[6]) * cos(theta[0] + theta[2]) + stick * sin(theta[5] + theta[6] + theta[8]) * cos(theta[0] + theta[2]));
    P.push_back(-0.5 * s * sin(theta[0]) + l1 * sin(theta[5]) * sin(theta[0] + theta[2]) + l2 * sin(theta[5] + theta[6]) * sin(theta[0] + theta[2]) + stick * sin(theta[5] + theta[6] + theta[8]) * sin(theta[0] + theta[2]));
    P.push_back(z0 - l1 * cos(theta[5]) - l2 * cos(theta[5] + theta[6]) - stick * cos(theta[5] + theta[6] + theta[8]));

    std::cout << "\nRight Hand Position : { " << P[0] << " , " << P[1] << " , " << P[2] << " }\n";
    std::cout << "Left Hand Position : { " << P[3] << " , " << P[4] << " , " << P[5] << " }\n";
}

/////////////////////////////////////////////////////////////////////////////////
/*                                 Values Test Mode                           */
///////////////////////////////////////////////////////////////////////////////

void TestManager::getMotorPos(float c_MotorAngle[])
{
    // 각 모터의 현재위치 값 불러오기 ** CheckMotorPosition 이후에 해야함(변수값을 불러오기만 해서 갱신 필요)
    for (auto &entry : motors)
    {
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        {
            c_MotorAngle[motorMapping[entry.first]] = tMotor->jointAngle;
        }
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
        {
            c_MotorAngle[motorMapping[entry.first]] = maxonMotor->jointAngle;
        }
    }
}

vector<float> TestManager::cal_Vmax(float q1[], float q2[],  float acc, float t2)
{
    vector<float> Vmax;

    for (long unsigned int i = 0; i < 10; i++)
    {
        float val;
        float S = q2[i] - q1[i];

        // 이동거리 양수로 변경
        if (S < 0)
        {
            S = -1 * S;
        }

        if (S > t2*t2*acc/4)
        {
            // 가속도로 도달 불가능
            // -1 반환
            val = -1;
        }
        else
        {
            // 2차 방정식 계수
            float A = 1/acc;
            float B = -1*t2;
            float C = S;
            float sol1 = (-B+sqrt(B*B-4*A*C))/2/A;
            float sol2 = (-B-sqrt(B*B-4*A*C))/2/A;
            if (sol1 >= 0 && sol1 <= acc*t2/2)
            {
                val = sol1;
            }
            else if (sol2 >= 0 && sol2 <= acc*t2/2)
            {
                val = sol2;
            }
            else
            {
                // 해가 범위 안에 없음
                // -2 반환
                val = -2;
            }
        }
        Vmax.push_back(val);
        cout << "Vmax_" << i << " : " << val << "rad/s\n";
    }

    return Vmax;
}

vector<float> TestManager::makeProfile(float q1[], float q2[], vector<float> &Vmax, float acc, float t, float t2)
{
    vector<float> Qi;
    for(long unsigned int i = 0; i < 9; i++)
    {
        float val, S;
        int sign;
        S = q2[i] - q1[i];   
        // 부호 확인
        if (S < 0)
        {
            S = -1 * S;
            sign = -1;
        }
        else
        {
            sign = 1;
        }
        // 궤적 생성
        if (S == 0)
        {
            // 정지
            val = q1[i];
        }
        else if (Vmax[i] < 0)
        {
            // Vmax 값을 구하지 못했을 때 삼각형 프로파일 생성
            float acc_tri = 4 * S / t2 / t2;
            if (t < t2/2)
            {
                val = q1[i] + sign * 0.5 * acc_tri * t * t;
            }
            else if (t < t2)
            {
                val = q2[i] - sign * 0.5 * acc_tri * (t2 - t) * (t2 - t);
            }
            else
            {
                val = q2[i];
            }
        }
        else
        {
            // 사다리꼴 프로파일
            if (t < Vmax[i] / acc)
            {
                // 가속
                val = q1[i] + sign * 0.5 * acc * t * t;
            }
            else if (t < S / Vmax[i])
            {
                // 등속
                val = q1[i] + (sign * 0.5 * Vmax[i] * Vmax[i] / acc) + (sign * Vmax[i] * (t - Vmax[i] / acc));          
            }
            else if (t < Vmax[i] / acc + S / Vmax[i])
            {
                // 감속
                val = q2[i] - sign * 0.5 * acc * (S / Vmax[i] + Vmax[i] / acc - t) * (S / Vmax[i] + Vmax[i] / acc - t);          
            }
            else 
            {
                val = q2[i];
            }
        }
        Qi.push_back(val);
    }
    return Qi;

    // vector<float> Qi;
    // float acceleration = 100; //320000 / 21 / 10 * 2 * M_PI / 60;  // rad/s^2 
    // int sign;
    // float Vmax = 0;
    // float S;
    // static int loop_count =0;

    // for (int i = 0; i < 9; i++)
    // {
    //     float val;

    //     S = Q2[i] - Q1[i];

    //     if (S < 0)
    //     {
    //         S = -1 * S;
    //         sign = -1;
    //     }
    //     else
    //     {
    //         sign = 1;
    //     }
    //     // 2차 방정식의 계수들
    //     float a = 1.0 / acceleration;
    //     float b = -n;
    //     float c = S;
    //     float discriminant = (b * b) - (4 * a * c);

    //     if (discriminant < 0)
    //     {
    //         // if(i ==4)
    //         // {
    //         // std::cout << "No real solution for Vmax." << std::endl;
    //         // sleep(1);
    //         // }
    //         val = -1;   //Qi.push_back(-1); // 실수 해가 없을 경우 -1 추가
    //         Qi.push_back(val);
    //         continue;
    //     }
    //     else
    //     {
    //         // 2차 방정식의 해 구하기
    //         float Vmax1 = (-b + std::sqrt(discriminant)) / (2 * a);
    //         float Vmax2 = (-b - std::sqrt(discriminant)) / (2 * a);
            
    //         // 두 해 중 양수인 해 선택
    //         if (Vmax1 > 0 && Vmax1 < 0.5*n*acceleration)
    //         {
    //             Vmax = Vmax1;

    //         }
    //         else if (Vmax2 > 0 && Vmax2 < 0.5*n*acceleration)
    //         {
    //             Vmax = Vmax2;

    //         }
    //         else
    //         {
    //             //std::cout << "No real solution for Vmax." << std::endl;
    //             Qi.push_back(Q1[i]); // 실수 해가 없을 경우
    //             continue;
    //         }
    //         //std::cout << "Calculated Vmax: " << Vmax << std::endl;
    //     }

    //     if (S == 0)
    //     {
    //         // 정지
    //         val = Q1[i];
    //     }
    //     else// if ((Vmax * Vmax / acceleration) < S)
    //     {
    //         // 가속
    //         if (k < Vmax / acceleration)
    //         {
    //             val = Q1[i] + sign * 0.5 * acceleration * k * k;
    //             // if(i==4)
    //             // {
    //             // std::cout <<"가속 : " <<val<< std::endl;
    //             // }
    //         }
    //         // 등속
    //         else if (k < S / Vmax)
    //         {
    //             val = Q1[i] + (sign * 0.5 * Vmax * Vmax / acceleration) + (sign * Vmax * (k - Vmax / acceleration)); 
    //         //    if(i==4)
    //         //     {
    //         //     std::cout <<"등속 : " <<val<< std::endl;
    //         //     }            
    //         }
    //         // 감속
    //         else if (k < Vmax / acceleration + S / Vmax)
    //         {
    //             val = Q2[i] - sign * 0.5 * acceleration * (S / Vmax + Vmax / acceleration - k) * (S / Vmax + Vmax / acceleration - k);
    //             // if(i ==4)
    //             // {
    //             // std::cout <<"감속 : " <<val<< std::endl;
    //             // }               
    //         }           
    //         else 
    //         {
    //             val = Q2[i];
    //             // if(i ==4)                
    //             // {
    //             // std::cout <<"else : " <<val<< std::endl;
    //             // }                   
    //         }
    //     }

    //     Qi.push_back(val);

    // }
    // loop_count ++;
    // // cout << " Qi[3] : "<< Qi[3] << " Qi[4] : "<< Qi[4] <<endl;
    // return Qi;
}

vector<float> TestManager::sinProfile(float q1[], float q2[], float t, float t2)
{
    vector<float> Qi;

    for (int i = 0; i < 9; i++)
    {
        float val;
        
        float A = q2[i] - q1[i];
        float w = 2.0 * M_PI/t2;
        
        val = -0.5*((A * cos(w*t)) - A)+ q1[i];

        Qi.push_back(val);
    }

    return Qi;
}

void TestManager::getArr(float arr[])
{
    const float acc_max = 100.0;    // rad/s^2
    vector<float> Qi;
    vector<float> Vmax;
    float Q1[10] = {0.0};
    float Q2[10] = {0.0};
    int n;
    int n_p;    // 목표위치까지 가기 위한 추가 시간
    int n_brake_start[7] = {0};
    int n_brake_end[7] = {0};

    n = (int)(t/canManager.DTSECOND);    // t초동안 이동
    n_p = (int)(extra_time/canManager.DTSECOND);  // 추가 시간
    
    std::cout << "Get Array...\n";

    getMotorPos(Q1);

    for (int i = 0; i < 10; i++)
    {
        Q2[i] = arr[i];
    }

    Vmax = cal_Vmax(Q1, Q2, acc_max, t);

    for (int i = 0; i < 7; i++)
    {
        if (brake_flag[i])
        {
            n_brake_start[i] = (int)(brake_start_time[i]/canManager.DTSECOND);
            n_brake_end[i] = (int)(brake_end_time[i]/canManager.DTSECOND);
        }
    }
    
    for (int i = 0; i < n_repeat; i++)
    {
        for (int k = 1; k <= n + n_p; ++k)
        {
            if(!sin_flag)
            {
                // Make Vector
                if ((i%2) == 0)
                {
                    Qi = makeProfile(Q1, Q2, Vmax, acc_max, t*k/n, t);
                }
                else
                {
                    Qi = makeProfile(Q2, Q1, Vmax, acc_max, t*k/n, t);
                }
            }
            else
            {
                Qi = sinProfile(Q1, Q2, t*k/n, t);
            }

            // Send to Buffer
            for (auto &entry : motors)
            {
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
                {
                    TMotorData newData;
                    newData.position = Qi[motorMapping[entry.first]];

                    tMotor->commandBuffer.push(newData);
                }
                else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
                {
                    MaxonData newData;
                    newData.position = Qi[motorMapping[entry.first]];
                    newData.torque = torque;
                    maxonMotor->commandBuffer.push(newData);
                }
            }
        }
    }
}

vector<float> TestManager::ikfun_final(float pR[], float pL[], float part_length[], float s, float z0)
{
    float direction = 0.0 * M_PI;

    float X1 = pR[0], Y1 = pR[1], z1 = pR[2];
    float X2 = pL[0], Y2 = pL[1], z2 = pL[2];
    float r1 = part_length[0];
    float r2 = part_length[1] + part_length[4];
    float L1 = part_length[2];
    float L2 = part_length[3] + part_length[5];

    int j = 0;
    float the3[1351];
    float zeta = z0 - z2;
    vector<float> Qf(7);
    float the0_f = 0;

    // the3 배열 초기화
    for (int i = 0; i < 1351; ++i)
        the3[i] = (-M_PI * 0.25) + (i / 1350.0 * M_PI * 0.75); // the3 범위 : -45deg ~ 90deg

    for (int i = 0; i < 1351; ++i)
    {
        float det_the4 = (z0 - z1 - r1 * cos(the3[i])) / r2;

        if (det_the4 < 1 && det_the4 > -1)
        {
            float the34 = acos((z0 - z1 - r1 * cos(the3[i])) / r2);
            float the4 = the34 - the3[i];

            if (the4 >= 0 && the4 < (M_PI * 0.75)) // the4 범위 : 0deg ~ 135deg
            {
                float r = r1 * sin(the3[i]) + r2 * sin(the34);
                float det_the1 = (X1 * X1 + Y1 * Y1 - r * r - s * s / 4.0) / (s * r);

                if (det_the1 < 1 && det_the1 > -1)
                {
                    float the1 = acos(det_the1);
                    if (the1 > 0 && the1 < (M_PI * 0.8)) // the1 범위 : 0deg ~ 144deg
                    {
                        float alpha = asin(X1 / sqrt(X1 * X1 + Y1 * Y1));
                        float det_the0 = (s / 4.0 + (X1 * X1 + Y1 * Y1 - r * r) / s) / sqrt(X1 * X1 + Y1 * Y1);

                        if (det_the0 < 1 && det_the0 > -1)
                        {
                            float the0 = asin(det_the0) - alpha;
                            if (the0 > (-M_PI / 2) && the0 < (M_PI / 2)) // the0 범위 : -90deg ~ 90deg
                            {
                                float L = sqrt((X2 - 0.5 * s * cos(the0 + M_PI)) * (X2 - 0.5 * s * cos(the0 + M_PI)) + (Y2 - 0.5 * s * sin(the0 + M_PI)) * (Y2 - 0.5 * s * sin(the0 + M_PI)));
                                float det_the2 = (X2 - 0.5 * s * cos(the0 + M_PI)) / L;

                                if (det_the2 < 1 && det_the2 > -1)
                                {
                                    float the2 = acos(det_the2) - the0;
                                    if (the2 > (M_PI * 0.2) && the2 < M_PI) // the2 범위 : 36deg ~ 180deg
                                    {
                                        float Lp = sqrt(L * L + zeta * zeta);
                                        float det_the6 = (Lp * Lp - L1 * L1 - L2 * L2) / (2 * L1 * L2);

                                        if (det_the6 < 1 && det_the6 > -1)
                                        {
                                            float the6 = acos(det_the6);
                                            if (the6 >= 0 && the6 < (M_PI * 0.75)) // the6 범위 : 0deg ~ 135deg
                                            {
                                                float T = (zeta * zeta + L * L + L1 * L1 - L2 * L2) / (L1 * 2);
                                                float det_the5 = L * L + zeta * zeta - T * T;

                                                if (det_the5 > 0)
                                                {
                                                    float sol = T * L - zeta * sqrt(L * L + zeta * zeta - T * T);
                                                    sol /= (L * L + zeta * zeta);
                                                    float the5 = asin(sol);
                                                    if (the5 > (-M_PI * 0.25) && the5 < (M_PI / 2)) // the5 범위 : -45deg ~ 90deg
                                                    {
                                                        if (j == 0 || fabs(the0 - direction) < fabs(the0_f - direction))
                                                        {
                                                            Qf[0] = the0;
                                                            Qf[1] = the1;
                                                            Qf[2] = the2;
                                                            Qf[3] = the3[i];
                                                            Qf[4] = the4;
                                                            Qf[5] = the5;
                                                            Qf[6] = the6;

                                                            the0_f = the0;
                                                            j = 1;
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if (j == 0)
    {
        cout << "IKFUN is not solved!!\n";
        state.main = Main::Error;
    }

    return Qf;
}

void TestManager::unfixedMotor()
{
    for (auto motor_pair : motors)
        motor_pair.second->isfixed = false;
}

/////////////////////////////////////////////////////////////////////////////////
/*                         Maxon Motor Function                               */
/////////////////////////////////////////////////////////////////////////////////

void TestManager::CSTLoop()
{
    std::shared_ptr<MaxonMotor> selectedMotor;

    for (auto &motor_pair : motors)
    {
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor_pair.second))
        {
            maxonMotor->clearCommandBuffer();
            maxonMotor->clearReceiveBuffer();
            if (motor_pair.first == "maxonForTest")
            {
                selectedMotor = maxonMotor;
                break;
            }
        }
    }

    // std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motors[selectedMotor]);

    chrono::system_clock::time_point external = std::chrono::system_clock::now();
    while(1)
    {
        chrono::system_clock::time_point internal = std::chrono::system_clock::now();
        chrono::microseconds elapsed_time = chrono::duration_cast<chrono::microseconds>(internal - external);
        if (elapsed_time.count() >= 5000)
        {
            maxoncmd.getTargetTorque(*selectedMotor, &frame, 1);
            canManager.txFrame(motors["maxonForTest"], frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motors["maxonForTest"], frame);
        }
    }
}

void TestManager::maxonMotorEnable()
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

void TestManager::setMaxonMotorMode(std::string targetMode, string motorName)
{
    struct can_frame frame;
    canManager.setSocketsTimeout(0, 10000);
    for (const auto &motorPair : motors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<GenericMotor> motor = motorPair.second;
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motorPair.second))
        {
            if (name == motorName)
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

                    usleep(10);

                    maxoncmd.getShutdown(*maxonMotor, &frame);
                    canManager.sendAndRecv(motor, frame);

                    usleep(10);

                    maxoncmd.getEnable(*maxonMotor, &frame);
                    canManager.sendAndRecv(motor, frame);

                    usleep(10);
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
}

void TestManager::TestStickLoop()
{

    std::string userInput;
    std::string selectedMotor = "maxonForTest";
    float des_tff = 0;
    float posThreshold = 1.57; // 위치 임계값 초기화
    float tffThreshold = 18;   // 토크 임계값 초기화
    int backTorqueUnit = 150;

    while (true)
    {

        int result = system("clear");
        if (result != 0)
        {
            std::cerr << "Error during clear screen" << std::endl;
        }

        std::cout << "================ Tuning Menu ================\n";
        std::cout << "Available Motors for Stick Mode:\n";
        for (const auto &motor_pair : motors)
        {
            if (motor_pair.first == "maxonForTest")
                std::cout << " - " << motor_pair.first << "\n";
        }

        bool isMaxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motors[selectedMotor]) != nullptr;
        if (!isMaxonMotor)
            break;

        std::cout << "---------------------------------------------\n";
        std::cout << "Selected Motor: " << selectedMotor << "\n";

        std::cout << "Des Torque: " << des_tff * 31.052 / 1000 << "[mNm]\n";
        std::cout << "Torque Threshold: " << tffThreshold << " [mNm]\n"; // 현재 토크 임계값 출력
        std::cout << "Position Threshold: " << posThreshold << " [rad]\n";
        std::cout << "Back Torque: " << backTorqueUnit * 31.052 / 1000 << " [mNm]\n";
        std::cout << "\nCommands:\n";
        std::cout << "[a]: des_tff | [b]: Direction | [c]: Back Torque\n";
        std::cout << "[f]: Run | [g]: Exit\n";
        std::cout << "=============================================\n";
        std::cout << "Enter Command: ";
        std::cin >> userInput;
        std::transform(userInput.begin(), userInput.end(), userInput.begin(), ::tolower);

        if (userInput[0] == 'g')
        {
            break;
        }

        else if (userInput == "a" && isMaxonMotor)
        {
            std::cout << "Enter Desired Torque In Unit: ";
            std::cout << "-100 [unit] = -3.1052 [mNm]\n";
            std::cin >> des_tff;
        }
        else if (userInput == "d" && isMaxonMotor)
        {
            std::cout << "Enter Desired Torque Threshold: ";
            std::cout << "-100 [unit] = -3.1052 [mNm]\n";
            std::cin >> tffThreshold;
        }
        else if (userInput == "e" && isMaxonMotor)
        {
            std::cout << "Enter Desired Position Threshold: ";
            std::cin >> posThreshold;
        }
        else if (userInput[0] == 'f' && isMaxonMotor)
        {
            std::cout << "\nRunning CST Mode";
            TestStick(selectedMotor, des_tff, tffThreshold, posThreshold, backTorqueUnit);
        }
    }
}

void TestManager::TestStick(const std::string selectedMotor, int des_tff, float tffThreshold, float posThreshold, int backTorqueUnit)
{
    canManager.setSocketsTimeout(0, 50000);

    struct can_frame frame;

    float p_act, tff_act;

    std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motors[selectedMotor]);

    bool motorFixed = false;

    chrono::system_clock::time_point external = std::chrono::system_clock::now();
    bool motorModeSet = false;
    int index = 0;

    while (1)
    {
        if (!motorModeSet)
        {
            maxoncmd.getCSTMode(*maxonMotor, &frame);
            canManager.sendAndRecv(motors[selectedMotor], frame);
            motorModeSet = true; // 모터 모드 설정 완료

            maxoncmd.getShutdown(*maxonMotor, &frame);
            canManager.sendAndRecv(motors[selectedMotor], frame);

            maxoncmd.getEnable(*maxonMotor, &frame);
            canManager.sendAndRecv(motors[selectedMotor], frame);

            usleep(10);
        }
        if (motorFixed)
        {
            break;
        }

        chrono::system_clock::time_point internal = std::chrono::system_clock::now();
        chrono::microseconds elapsed_time = chrono::duration_cast<chrono::microseconds>(internal - external);
        if (elapsed_time.count() >= 5000)
        {
            index++;
            maxoncmd.getTargetTorque(*maxonMotor, &frame, des_tff);
            canManager.txFrame(motors[selectedMotor], frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motors[selectedMotor], frame);

            if (canManager.recvToBuff(motors[selectedMotor], canManager.maxonCnt))
            {
                while (!motors[selectedMotor]->recieveBuffer.empty())
                {
                    frame = motors[selectedMotor]->recieveBuffer.front();
                    if (frame.can_id == maxonMotor->rxPdoIds[0])
                    {
                        std::tuple<int, float, float, int8_t> result = maxoncmd.parseRecieveCommand(*maxonMotor, &frame);

                        p_act = std::get<1>(result);
                        tff_act = std::get<2>(result);
                        fun.appendToCSV_DATA("CST_log", des_tff * 31.052 / 1000, tff_act, p_act);
                    }
                    if (!motors[selectedMotor]->recieveBuffer.empty())
                    {
                        motors[selectedMotor]->recieveBuffer.pop();
                    }
                }
            }
        }
        if (index >= 1000) 
        {
            maxoncmd.getTargetTorque(*maxonMotor, &frame, 0);
            canManager.txFrame(motors[selectedMotor], frame);

            maxoncmd.getSync(&frame);
            canManager.txFrame(motors[selectedMotor], frame);
            return;
        }
    }
}

float TestManager::makeWristAngle(float t1, float t2, float t, int state, int intensity, bool &hitting, float hittingPos)
{
    float wrist_q = 0.0;
    float t_press = std::min(0.1 * (t2 - t1), 0.05); // 0.08 -> 0.05
    float t_lift = std::max(0.8 * (t2 - t1), t2 - t1 - 0.1);
    float t_stay;
    t2 - t1 < 0.15 ? t_stay = 0.45 * (t2 - t1) : t_stay = 0.47 * (t2 - t1) - 0.05;
    float t_release = std::min(0.2 * (t2 - t1), 0.1);
    float t_contact = t2 - t1;
    float t_hitting = 0.0;
    float wristLiftAngle;
    t2 - t1 < 0.5 ? wristLiftAngle = (-100 * ((t2 - t1) - 0.5) * ((t2 - t1) - 0.5) + 40) * M_PI / 180.0 : wristLiftAngle = 40  * M_PI / 180.0;
    float wristStayAngle = 10.0 * M_PI / 180.0;
    float wristContactAngle = -1.0 * std::min((t2 - t1) * 5.0 * M_PI / 180.0 / 0.5, 5.0 * M_PI / 180.0);

    float intensityFactor = 0.4 * intensity + 0.2; // 1 : 약하게   2 : 기본    3 : 강하게
    wristLiftAngle = wristLiftAngle * intensityFactor;

    static bool hittingTimeCheck = true;

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd sol;

    if(hitting)
    {
        if (hittingTimeCheck)
        {
            //t_hitting = t + 0.005;
            t_hitting = t;
            hittingTimeCheck = false;
        }

        if (state == 1)
        {
            // Contact - Stay
            A.resize(4, 4);
            b.resize(4, 1);
            if (t <= t_press)
            {
                if (t_hitting > t_press)
                {
                    A << 1, 0, 0, 0,
                    1, t_press, t_press * t_press, t_press * t_press * t_press,
                    0, 1, 0, 0,
                    0, 1, 2 * t_press, 3 * t_press * t_press;
                    b << hittingPos, wristStayAngle, 0, 0;
                }
                else
                {
                    t_release = t_hitting + t_press;
                    A << 1, t_hitting, t_hitting * t_hitting, t_hitting * t_hitting * t_hitting,
                    1, t_release, t_release * t_release, t_release * t_release * t_release,
                    0, 1, 2 * t_hitting, 3 * t_hitting * t_hitting,
                    0, 1, 2 * t_release, 3 * t_release * t_release;
                    b << hittingPos, wristStayAngle, 0, 0;
                }
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else
            {
                hitting = false;
                hittingTimeCheck = true;
                wrist_q = wristStayAngle;
            }
        }
        else if (state == 2)
        {
            wrist_q = hittingPos;
        }
        else if (state == 3)
        {
            // Contact - Lift - Hit
            if (t < t_stay)
            {
                A.resize(4, 4);
                b.resize(4, 1);
                if (t_hitting > t_stay)
                {
                    A << 1, 0, 0, 0,
                    1, t_stay, t_stay * t_stay, t_stay * t_stay * t_stay,
                    0, 1, 0, 0,
                    0, 1, 2 * t_stay, 3 * t_stay * t_stay;
                    b << hittingPos, wristLiftAngle, 0, 0;
                }
                else
                {
                    A << 1, t_hitting, t_hitting * t_hitting, t_hitting * t_hitting * t_hitting,
                    1, t_stay, t_stay * t_stay, t_stay * t_stay * t_stay,
                    0, 1, 2 * t_hitting, 3 * t_hitting * t_hitting,
                    0, 1, 2 * t_stay, 3 * t_stay * t_stay;
                    b << hittingPos, wristLiftAngle, 0, 0;
                }
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else if (t < t_lift)
            {
                hitting = false;
                hittingTimeCheck = true;
                wrist_q = wristLiftAngle;
            }
            else if (t <= t_contact)
            {
                wrist_q = hittingPos;
            }
        }
    }
    else
    {
        if (state == 0)
        {
            // Stay
            wrist_q = wristStayAngle;
        }
        else if (state == 1)
        {
            // Contact - Stay
            A.resize(3, 3);
            b.resize(3, 1);
            if (t < t_press)
            {
                A << 1, 0, 0,
                1, t_press, t_press * t_press,
                0, 1, 2 * t_press;
                b << 0, wristContactAngle, 0;
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t;
                
            }
            else if (t <= t_release)
            {
                A.resize(4, 4);
                b.resize(4, 1);
                A << 1, t_press, t_press * t_press, t_press * t_press * t_press,
                    1, t_release, t_release * t_release, t_release * t_release * t_release,
                    0, 1, 2 * t_press, 3 * t_press * t_press,
                    0, 1, 2 * t_release, 3 * t_release * t_release;
                b << wristContactAngle, wristStayAngle, 0, 0;
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else
            {
                wrist_q = wristStayAngle;
            }
        }
        else if (state == 2)
        {
            
            // Stay - Lift - Hit
            if (t < t_stay)
            {
                // Stay
                wrist_q = wristStayAngle;
            }
            else if (t < t_lift)
            {
                A.resize(4, 4);
                b.resize(4, 1);
                A << 1, t_stay, t_stay * t_stay, t_stay * t_stay * t_stay,
                    1, t_lift, t_lift * t_lift, t_lift * t_lift * t_lift,
                    0, 1, 2 * t_stay, 3 * t_stay * t_stay,
                    0, 1, 2 * t_lift, 3 * t_lift * t_lift;
                b << wristStayAngle, wristLiftAngle, 0, 0;
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else if (t <= t_contact)
            {
                // canManager.isHit = true;
                A.resize(3, 3);
                b.resize(3, 1);
                A << 1, t_lift, t_lift * t_lift,
                    1, t_contact, t_contact * t_contact,
                    0, 1, 2 * t_lift;
                b << wristLiftAngle, 0, 0;
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t;
            }
            else
            {
                wrist_q = 0.0;
            }
            
        }
        else if (state == 3)
        {
            // Contact - Lift - Hit
            if (t < t_press)
            {
                A.resize(3, 3);
                b.resize(3, 1);
                A << 1, 0, 0,
                    1, t_press, t_press * t_press,
                    0, 1, 2 * t_press;
                b << 0, wristContactAngle, 0;
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t;
            }
            else if (t < t_stay)
            {
                A.resize(4, 4);
                b.resize(4, 1);
                A << 1, t_press, t_press * t_press, t_press * t_press * t_press,
                    1, t_stay, t_stay * t_stay, t_stay * t_stay * t_stay,
                    0, 1, 2 * t_press, 3 * t_press * t_press,
                    0, 1, 2 * t_stay, 3 * t_stay * t_stay;
                b << wristContactAngle, wristLiftAngle, 0, 0;
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else if (t < t_lift)
            {
                // Stay
                wrist_q = wristLiftAngle;
            }
            else if (t <= t_contact)
            {
                // canManager.isHit = true;
                A.resize(3, 3);
                b.resize(3, 1);
                A << 1, t_lift, t_lift * t_lift,
                    1, t_contact, t_contact * t_contact,
                    0, 1, 2 * t_lift;
                b << wristLiftAngle, 0, 0;
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t;
            }
            else
            {
                wrist_q = 0.0;
            }
        }
    }

    return wrist_q;

}

float TestManager::makeWristAngle_CST(float t1, float t2, float t, int state, int intensity, bool &hitting, float hittingPos)
{
    float wrist_q = 0.0;
    float t_press = std::min(0.1 * (t2 - t1), 0.05); // 0.08 -> 0.05
    float t_lift = std::max(0.8 * (t2 - t1), t2 - t1 - 0.1);
    float t_stay;
    t2 - t1 < 0.15 ? t_stay = 0.45 * (t2 - t1) : t_stay = 0.47 * (t2 - t1) - 0.05;
    static float t_release = std::min(0.2 * (t2 - t1), 0.1);
    float t_contact = t2 - t1;
    float t_hitting = 0.0;
    float wristLiftAngle;
    t2 - t1 < 0.5 ? wristLiftAngle = (-100 * ((t2 - t1) - 0.5) * ((t2 - t1) - 0.5) + 40) * M_PI / 180.0 : wristLiftAngle = 40  * M_PI / 180.0;
    float wristStayAngle = 15.0 * M_PI / 180.0;
    float wristContactAngle = -1.0 * std::min((t2 - t1) * 5.0 * M_PI / 180.0 / 0.5, 5.0 * M_PI / 180.0);

    float intensityFactor = 0.4 * intensity + 0.2; // 1 : 약하게   2 : 기본    3 : 강하게
    wristLiftAngle = wristLiftAngle * intensityFactor;

    static bool hittingTimeCheck = true;

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd sol;

    if(hitting)
    {
        if (hittingTimeCheck)
        {
            //t_hitting = t + 0.005;
            t_hitting = t;
            hittingTimeCheck = false;
            canManager.isCSTL = false;
        }

        if (state == 1)
        {
            // Contact - Stay
            A.resize(4, 4);
            b.resize(4, 1);
            if (t <= t_release)
            {
                if (t_hitting > t_press)
                {
                    A << 1, 0, 0, 0,
                    1, t_press, t_press * t_press, t_press * t_press * t_press,
                    0, 1, 0, 0,
                    0, 1, 2 * t_press, 3 * t_press * t_press;
                    b << hittingPos, wristStayAngle, 0, 0;
                }
                else
                {
                    t_release = t_hitting + t_press;
                    A << 1, t_hitting, t_hitting * t_hitting, t_hitting * t_hitting * t_hitting,
                    1, t_release, t_release * t_release, t_release * t_release * t_release,
                    0, 1, 2 * t_hitting, 3 * t_hitting * t_hitting,
                    0, 1, 2 * t_release, 3 * t_release * t_release;
                    b << hittingPos, wristStayAngle, 0, 0;
                }
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else
            {
                hitting = false;
                hittingTimeCheck = true;
                wrist_q = wristStayAngle;
            }
        }
        else if (state == 2)
        {
            wrist_q = hittingPos;
        }
        else if (state == 3)
        {
            // Contact - Lift - Hit
            if (t < t_stay)
            {
                A.resize(4, 4);
                b.resize(4, 1);
                if (t_hitting > t_stay)
                {
                    A << 1, 0, 0, 0,
                    1, t_stay, t_stay * t_stay, t_stay * t_stay * t_stay,
                    0, 1, 0, 0,
                    0, 1, 2 * t_stay, 3 * t_stay * t_stay;
                    b << hittingPos, wristLiftAngle, 0, 0;
                }
                else
                {
                    A << 1, t_hitting, t_hitting * t_hitting, t_hitting * t_hitting * t_hitting,
                    1, t_stay, t_stay * t_stay, t_stay * t_stay * t_stay,
                    0, 1, 2 * t_hitting, 3 * t_hitting * t_hitting,
                    0, 1, 2 * t_stay, 3 * t_stay * t_stay;
                    b << hittingPos, wristLiftAngle, 0, 0;
                }
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else if (t < t_lift)
            {
                hitting = false;
                hittingTimeCheck = true;
                wrist_q = wristLiftAngle;
            }
            else if (t <= t_contact)
            {
                wrist_q = hittingPos;
            }
        }
    }
    else if (!hitting && !canManager.isCSTL)
    {
        if (state == 0)
        {
            // Stay
            wrist_q = wristStayAngle;
        }
        else if (state == 1)
        {
            // Contact - Stay
            A.resize(3, 3);
            b.resize(3, 1);
            if (t < t_press)
            {
                A << 1, 0, 0,
                1, t_press, t_press * t_press,
                0, 1, 2 * t_press;
                b << 0, wristContactAngle, 0;
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t;
                
            }
            else if (t <= t_release)
            {
                A.resize(4, 4);
                b.resize(4, 1);
                A << 1, t_press, t_press * t_press, t_press * t_press * t_press,
                    1, t_release, t_release * t_release, t_release * t_release * t_release,
                    0, 1, 2 * t_press, 3 * t_press * t_press,
                    0, 1, 2 * t_release, 3 * t_release * t_release;
                b << wristContactAngle, wristStayAngle, 0, 0;
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else
            {
                wrist_q = wristStayAngle;
            }
        }
        else if (state == 2)
        {
            
            // Stay - Lift - Hit
            if (t < t_stay)
            {
                // Stay
                wrist_q = wristStayAngle;
            }
            else if (t < t_lift)
            {
                A.resize(4, 4);
                b.resize(4, 1);
                A << 1, t_stay, t_stay * t_stay, t_stay * t_stay * t_stay,
                    1, t_lift, t_lift * t_lift, t_lift * t_lift * t_lift,
                    0, 1, 2 * t_stay, 3 * t_stay * t_stay,
                    0, 1, 2 * t_lift, 3 * t_lift * t_lift;
                b << wristStayAngle, wristLiftAngle, 0, 0;
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else if (t <= t_contact)
            {
                canManager.isHitL = true;
                canManager.isCSTL = true;
            }
            
        }
        else if (state == 3)
        {
            // Contact - Lift - Hit
            if (t < t_press)
            {
                A.resize(3, 3);
                b.resize(3, 1);
                A << 1, 0, 0,
                    1, t_press, t_press * t_press,
                    0, 1, 2 * t_press;
                b << 0, wristContactAngle, 0;
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t;
            }
            else if (t < t_stay)
            {
                A.resize(4, 4);
                b.resize(4, 1);
                A << 1, t_press, t_press * t_press, t_press * t_press * t_press,
                    1, t_stay, t_stay * t_stay, t_stay * t_stay * t_stay,
                    0, 1, 2 * t_press, 3 * t_press * t_press,
                    0, 1, 2 * t_stay, 3 * t_stay * t_stay;
                b << wristContactAngle, wristLiftAngle, 0, 0;
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else if (t < t_lift)
            {
                // Stay
                wrist_q = wristLiftAngle;
            }
            else if (t <= t_contact)
            {
                canManager.isHitL = true;
                canManager.isCSTL = true;
            }
        }
    }

    return wrist_q;

}

tuple <double, int, int> TestManager::CSTHitLoop()
{
    string userInput;
    double t = 0.6;
    int repeat = 1;
    int state = 3;
    int intensity = 3;

    while(1)
    {
        int result = system("clear");
        if (result != 0)
        {
            std::cerr << "Error during clear screen" << std::endl;
        }

        cout << "========== Hit Test Mode ==========\n";
        cout << "Hit Time   : " << t;
        cout << "\nRepeat   : " << repeat;
        cout << "\nState    :" << state;
        cout << "\nIntensity    : " << intensity;
        cout << "\nBack Torque   : " << canManager.backTorque;
        if (hitMode == 1)
        {
            cout << "\nHitting Mode   : Detect Hitting";
        }
        else if (hitMode == 2)
        {
            cout << "\nHitting Mode   : 기존 방식";
        }
        else if (hitMode == 3)
        {
            cout << "\nHitting Mode   : CST Hitting";
        }
        cout << "\n-----------------------------------";
        cout << "\nSetting Parameters";
        cout << "\n[t] : Time / [r] Repeat / [i] intensity / [m] Mode / [b] back torque / [g] Run";
        cout << "\n Enter Command   : ";
        cin >> userInput;

        if (userInput == "t")
        {
            cout << "\nEnter Hit Time     : ";
            cin >> t;
        }
        else if (userInput == "b")
        {
            float temp;
            cout << "\nRange : 1 ~ 500";
            cout << "\nEnter Back Torque     : ";
            cin >> temp;
            canManager.backTorque = temp * -1;
        }
        else if(userInput == "r")
        {
            cout << "\nEnter Repeat Count     : ";
            cin >> repeat;
        }
        else if (userInput == "m")
        {
            cout << "\n[1] Detect Hitting / [2] 기존 방식 / [3] CST Hitting \nMode Select  : ";
            cin >> hitMode;
        }
        else if (userInput == "i")
        {
            cout << "\nEnter Intensity     : ";
            cin >> intensity;
        }
        else if(userInput == "g")
        {
            return make_tuple(t, repeat, intensity);
        }
    }

}

/////////////////////////////////////////////////////////////////////////////////
/*                              TEST Function                                  */
/////////////////////////////////////////////////////////////////////////////////

void TestManager::testTable()
{
    srand(time(NULL));
    std::ifstream tableFile;
    std::string tablePath = "/home/shy/DrumRobot2_TABLE/";    // 테이블 위치

    int n, sum = 0;
    double P[6] = {0};
    int P_index[6] = {0};
    double R[2][6] = {{-0.35, 0.45, 0.55, -0.35, 0.45, 0.55}, {0.8, 0.35, 0.65, 0.8, 0.35, 0.65}};
    double dx = 0.05;
    
    std::cout << "\n 반복 횟수 : ";
    std::cin >> n;

    std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
    for (int i = 0; i < n; i++)
    {
        // 랜덤 위치
        for (int j = 0; j < 6; j++)
        {
            // if (j == 0)
            // {
            //     std::cout << "\n 오른팔 좌표 :";
            // }
            // else if (j == 3)
            // {
            //     std::cout << "\n 왼팔 좌표 :";
            // }

            P[j] = R[0][j] + R[1][j]*((rand()%1000)/1000.0);
            // std::cout << "\t" << P[j];
        }

        // 인덱스 공간으로 변환
        for (int j = 0; j < 6; j++)
        {
            P_index[j] = floor((P[j] - R[0][j])/dx + 0.5);
        }

        std::string fileName = tablePath + "TABLE_" + std::to_string(P_index[0]+1) + "_" + std::to_string(P_index[1]+1) +".txt";
        tableFile.open(fileName); // 파일 열기
        
        if (tableFile.is_open())
        {
            string row;

            for (int j = 0; j < P_index[3]+1; j++)
            {
                getline(tableFile, row);
            }

            istringstream iss(row);
            string item;

            for (int j = 0; j < P_index[2]+1; j++)
            {
                getline(iss, item, '\t');
            }

            item = trimWhitespace(item);

            char hex1 = item.at(2*P_index[5] + 1);
            char hex2 = item.at(2*P_index[5]);

            if (hex2TableData(hex1, hex2, P_index[4]))
            {
                sum++;
            }

            // std::cout << "\n data : " << hex2TableData(hex1, hex2, P_index[4]) << "\n";
            // sleep(1);
        }
        else
        {
            std::cout << "\n table file open error \n";
        }

        tableFile.close(); // 파일 닫기
    }

    std::chrono::duration<double>sec = std::chrono::system_clock::now() - start;
    std::cout << "\n 실행 시간 : " << sec.count()*1000 << " ms\n";
    std::cout << "\n 충돌 횟수 : " << sum << "\n";
    sleep(10);
}

string TestManager::trimWhitespace(const std::string &str)
{
    size_t first = str.find_first_not_of(" \t");
    if (std::string::npos == first)
    {
        return str;
    }
    size_t last = str.find_last_not_of(" \t");
    return str.substr(first, (last - first + 1));
}

bool TestManager::hex2TableData(char hex1, char hex2, int index)
{
    char hex;
    bool bin[4];

    if (index < 4)
    {
        hex = hex1;
    }
    else
    {
        hex = hex2;
        index = index - 4;
    }

    switch(hex)
    {
    case '0':
        bin[0] = false;
        bin[1] = false;
        bin[2] = false;
        bin[3] = false;
    break;
    case '1':
        bin[0] = true;
        bin[1] = false;
        bin[2] = false;
        bin[3] = false;
    break;
    case '2':
        bin[0] = false;
        bin[1] = true;
        bin[2] = false;
        bin[3] = false;
    break;
    case '3':
        bin[0] = true;
        bin[1] = true;
        bin[2] = false;
        bin[3] = false;
    break;
    case '4':
        bin[0] = false;
        bin[1] = false;
        bin[2] = true;
        bin[3] = false;
    break;
    case '5':
        bin[0] = true;
        bin[1] = false;
        bin[2] = true;
        bin[3] = false;
    break;
    case '6':
        bin[0] = false;
        bin[1] = true;
        bin[2] = true;
        bin[3] = false;
    break;
    case '7':
        bin[0] = true;
        bin[1] = true;
        bin[2] = true;
        bin[3] = false;
    break;
    case '8':
        bin[0] = false;
        bin[1] = false;
        bin[2] = false;
        bin[3] = true;
    break;
    case '9':
        bin[0] = true;
        bin[1] = false;
        bin[2] = false;
        bin[3] = true;
    break;
    case 'A':
        bin[0] = false;
        bin[1] = true;
        bin[2] = false;
        bin[3] = true;
    break;
    case 'B':
        bin[0] = true;
        bin[1] = true;
        bin[2] = false;
        bin[3] = true;
    break;
    case 'C':
        bin[0] = false;
        bin[1] = false;
        bin[2] = true;
        bin[3] = true;
    break;
    case 'D':
        bin[0] = true;
        bin[1] = false;
        bin[2] = true;
        bin[3] = true;
    break;
    case 'E':
        bin[0] = false;
        bin[1] = true;
        bin[2] = true;
        bin[3] = true;
    break;
    case 'F':
        bin[0] = true;
        bin[1] = true;
        bin[2] = true;
        bin[3] = true;
    break;
    }

    return bin[index];
}
