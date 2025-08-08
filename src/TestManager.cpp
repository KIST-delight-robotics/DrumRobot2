#include "../include/managers/TestManager.hpp" // 적절한 경로로 변경하세요.

// For  Qt
// #include "../managers/TestManager.hpp"
using namespace std;

TestManager::TestManager(State &stateRef, CanManager &canManagerRef, std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef, USBIO &usbioRef, Functions &funRef)
    : state(stateRef), canManager(canManagerRef), motors(motorsRef), usbio(usbioRef), fun(funRef)
{
    standardTime = chrono::system_clock::now();
}

void TestManager::SendTestProcess()
{
    while(1)
    {    
        int ret = system("clear");
        if (ret == -1) std::cout << "system clear error" << endl;

        float c_MotorAngle[12];
        getMotorPos(c_MotorAngle);

        std::cout << "[ Current Q Values (Radian / Degree) ]\n";
        for (int i = 0; i < 12; i++)
        {
            q[i] = c_MotorAngle[i];
            std::cout << "Q[" << i << "] : " << c_MotorAngle[i] << "\t\t" << c_MotorAngle[i] * 180.0 / M_PI << "\n";
            }
        FK(c_MotorAngle); // 현재 q값에 대한 FK 진행
    
        std::cout << "\nSelect Method (1 - 관절각도값 조절, 2 - 좌표값 조절, 4 - 발 모터, -1 - 나가기) : ";
        std::cin >> method;

        if(method == 1)
        {
            while(1)
            {
                int userInput = 100;
                int ret = system("clear");
                if (ret == -1) std::cout << "system clear error" << endl;

                float c_MotorAngle[10] = {0};
                getMotorPos(c_MotorAngle);

                std::cout << "[ Current Q Values (Radian / Degree) ]\n";
                for (int i = 0; i < 10; i++)
                {
                    std::cout << "Q[" << i << "] : " << c_MotorAngle[i] << "\t\t" << c_MotorAngle[i] * 180.0 / M_PI << "\t\t" <<q[i]/ M_PI * 180.0 << "\n";
                    }
                FK(c_MotorAngle);

                std::cout << "\ntime : " << t << "s";
                std::cout << "\nnumber of repeat : " << n_repeat << std::endl << std::endl;


                std::cout << "\nSelect Motor to Change Value (0-8) / Run (9) / Time (10) / Extra Time (11) / Repeat(12) / break on off (13) / Exit (-1): ";
                std::cin >> userInput;

                if (userInput == -1)
                {
                    break;
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
                    getArr(q);
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
            }
        }
        else if (method == 2)
        {
            while(1)
            {
                int userInput = 100;
                int ret = system("clear");
                if (ret == -1)
                    std::cout << "system clear error" << endl;
                std::cout << "[ Current x, y, z (meter) ]\n";
                std::cout << "Right : ";
                for (int i = 0; i < 3; i++)
                {
                    std::cout << R_xyz[i] << ", ";
                }
                std::cout << "\nLeft : ";
                for (int i = 0; i < 3; i++)
                {
                    std::cout << L_xyz[i] << ", ";
                }

                std::cout << "\nSelect Motor to Change Value (1 - Right, 2 - Left) / Start Test (3) / Exit (-1) : ";
                std::cin >> userInput;

                if (userInput == -1)
                {
                    break;
                }
                else if (userInput == 1)
                {
                    std::cout << "Enter x, y, z Values (meter) : ";
                    std::cin >> R_xyz[0] >> R_xyz[1] >> R_xyz[2];
                }
                else if (userInput == 2)
                {
                    std::cout << "Enter x, y, z Values (meter) : ";
                    std::cin >> L_xyz[0] >> L_xyz[1] >> L_xyz[2];
                }
                else if (userInput == 3)
                {
                    VectorXd pR(3), pL(3);
                    for (int i = 0; i < 3; i++)
                    {
                        pR(i) = R_xyz[i];
                        pL(i) = L_xyz[i];
                    }
                    
                    VectorXd waistVector = calWaistAngle(pR, pL);
                    VectorXd qk = IKFixedWaist(pR, pL, 0.5 * (waistVector(0) + waistVector(1)));

                    int qn = qk.size();
                    for (int i = 0; i < qn && i < 10; ++i) {
                        q[i] = qk(i);
                    }
                    getArr(q);
                }
            }
           
        }               
        else if (method == 4)
        {
            while(1)
            {
                int userInput = 100;
                int ret = system("clear");
                if (ret == -1) std::cout << "system clear error" << endl;

                float c_MotorAngle[12] = {0};
                getMotorPos(c_MotorAngle);

                std::cout << "[ Current Q Values (Radian / Degree) ]\n";
                for (int i = 10; i < 12; i++)
                {
                    std::cout << "Q[" << i << "] : " << c_MotorAngle[i] << "\t\t" << c_MotorAngle[i] * 180.0 / M_PI << "\t\t" << q[i]/ M_PI * 180.0 << "\n";
                }

                std::cout << "\ntime : " << t << "s";
                std::cout << "\nextra time : " << extra_time << "s";
                std::cout << "\nnumber of repeat : " << n_repeat << std::endl << std::endl;


                std::cout << "\nSelect Motor to Change Value (R: 10, L: 11) / Run (1) / Time (2) / Extra Time (3) / Repeat(4) / 강시우(5) / Exit (-1): ";
                std::cin >> userInput;

                if (userInput == -1)
                {
                    break;
                }
                else if (userInput == 10 || userInput == 11)
                {
                    float degree_angle;

                    std::cout << "\nRange : " << jointRangeMin[userInput] << "~" << jointRangeMax[userInput] << "(Degree)\n";
                    std::cout << "Enter q[" << userInput << "] Values (Degree) : ";
                    std::cin >> degree_angle;
                    q[userInput] = degree_angle * M_PI / 180.0;
                }
                else if (userInput == 1)
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
                    getArr(q);
                }
                else if (userInput == 2)
                {
                    std::cout << "time : ";
                    std::cin >> t;
                }
                else if (userInput == 3)
                {
                    std::cout << "extra time : ";
                    std::cin >> extra_time;
                }
                else if (userInput == 4)
                {
                    std::cout << "number of repeat : ";
                    std::cin >> n_repeat;
                }
                else if (userInput == 5)
                {
                    float A = 90.0;
                    
                    std::cout << "A : ";
                    std::cin >> A;
                    
                    float t_now = 0.0;
                    float dt = 0.005;
                    
                    while(t_now<=t)
                    {   
                        vector<float> Qi(12);

                        for (int i = 0; i < 12; i++)
                        {
                            Qi[i] = A * sin (2 * M_PI * t_now / t) * M_PI / 180.0 + c_MotorAngle[i];
                            //Qi[i] = A * (1 - cos(2 * M_PI * t_now / t)) / 2.0 * M_PI / 180.0 + c_MotorAngle[i];

                            if (i == 11)
                            {
                                fun.appendToCSV_DATA("test", Qi[i], 0, 0);
                            }
                        }
                        // Send to Buffer
                        for (auto &entry : motors)
                        {
                            if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
                            {
                                TMotorData newData;
                                newData.position = tMotor->jointAngleToMotorPosition(Qi[canManager.motorMapping[entry.first]]);
                                newData.mode = tMotor->Position;
                                tMotor->commandBuffer.push(newData);
                                
                                tMotor->finalMotorPosition = newData.position;
                            }
                            else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
                            {
                                MaxonData newData;
                                newData.position = maxonMotor->jointAngleToMotorPosition(Qi[canManager.motorMapping[entry.first]]);
                                newData.mode = maxonMotor->CSP;
                                maxonMotor->commandBuffer.push(newData);

                                maxonMotor->finalMotorPosition = newData.position;
                            }
                        }
                        //std::this_thread::sleep_for(std::chrono::milliseconds(5));
                        t_now += dt;
                    }
                
                }
            
            }
        }
        else
        {
            break;
        }
    }    
}

void TestManager::FK(float theta[])
{
    vector<float> P;

    PartLength partLength;

    float r1 = partLength.upperArm;
    float r2 = partLength.lowerArm;
    float l1 = partLength.upperArm;
    float l2 = partLength.lowerArm;
    float stick = partLength.stick;
    float s = partLength.waist;
    float z0 = partLength.height;

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
/////////////////////////////////////////////////////////////////////////////////

void TestManager::getMotorPos(float c_MotorAngle[])
{
    // 각 모터의 현재위치 값 불러오기 ** CheckMotorPosition 이후에 해야함(변수값을 불러오기만 해서 갱신 필요)
    for (auto &entry : motors)
    {
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        {
            c_MotorAngle[canManager.motorMapping[entry.first]] = tMotor->jointAngle;
        }
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
        {
            c_MotorAngle[canManager.motorMapping[entry.first]] = maxonMotor->jointAngle;
        }
    }
}

vector<float> TestManager::cal_Vmax(float q1[], float q2[],  float acc, float t2)
{
    vector<float> Vmax;

    for (long unsigned int i = 0; i < 12; i++)
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
    for(long unsigned int i = 0; i < 12; i++)
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

void TestManager::getArr(float arr[])
{
    const float acc_max = 100.0;    // rad/s^2
    vector<float> Qi;
    vector<float> Vmax;
    float Q1[12] = {0.0};
    float Q2[12] = {0.0};
    int n;
    int n_p;    // 목표위치까지 가기 위한 추가 시간

    n = (int)(t/canManager.DTSECOND);    // t초동안 이동
    n_p = (int)(extra_time/canManager.DTSECOND);  // 추가 시간
    
    std::cout << "Get Array...\n";

    getMotorPos(Q1);

    for (int i = 0; i < 12; i++)
    {
        Q2[i] = arr[i];
    }

    Vmax = cal_Vmax(Q1, Q2, acc_max, t);
    
    for (int i = 0; i < n_repeat; i++)
    {
        for (int k = 1; k <= n + n_p; ++k)
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

            // Send to Buffer
            for (auto &entry : motors)
            {
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
                {
                    TMotorData newData;
                    newData.position = tMotor->jointAngleToMotorPosition(Qi[canManager.motorMapping[entry.first]]);
                    newData.mode = tMotor->Position;
                    tMotor->commandBuffer.push(newData);
                    
                    tMotor->finalMotorPosition = newData.position;
                }
                else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
                {
                    MaxonData newData;
                    newData.position = maxonMotor->jointAngleToMotorPosition(Qi[canManager.motorMapping[entry.first]]);
                    newData.mode = maxonMotor->CSP;
                    maxonMotor->commandBuffer.push(newData);

                    maxonMotor->finalMotorPosition = newData.position;
                }
            }
        }
    }
}

VectorXd TestManager::IKFixedWaist(VectorXd pR, VectorXd pL, double theta0)
{
    VectorXd Qf;
    PartLength partLength;

    float XR = pR(0), YR = pR(1), ZR = pR(2);
    float XL = pL(0), YL = pL(1), ZL = pL(2);
    float R1 = partLength.upperArm;
    float R2 = partLength.lowerArm + partLength.stick;
    float L1 = partLength.upperArm;
    float L2 = partLength.lowerArm + partLength.stick;
    float s = partLength.waist;
    float z0 = partLength.height;

    float shoulderXR = 0.5 * s * cos(theta0);
    float shoulderYR = 0.5 * s * sin(theta0);
    float shoulderXL = -0.5 * s * cos(theta0);
    float shoulderYL = -0.5 * s * sin(theta0);

    float theta01 = atan2(YR - shoulderYR, XR - shoulderXR);
    float theta1 = theta01 - theta0;

    if (theta1 < 0 || theta1 > 150.0 * M_PI / 180.0) // the1 범위 : 0deg ~ 150deg
    {
        std::cout << "IKFUN (q1) is not solved!!\n";
        state.main = Main::Error;
    }

    float theta02 = atan2(YL - shoulderYL, XL - shoulderXL);
    float theta2 = theta02 - theta0;

    if (theta2 < 30 * M_PI / 180.0 || theta2 > M_PI) // the2 범위 : 30deg ~ 180deg
    {
        std::cout << "IKFUN (q2) is not solved!!\n";
        state.main = Main::Error;
    }

    float zeta = z0 - ZR;
    float r2 = (YR - shoulderYR) * (YR - shoulderYR) + (XR - shoulderXR) * (XR - shoulderXR); // r^2

    float x = zeta * zeta + r2 - R1 * R1 - R2 * R2;
    float y = sqrt(4.0 * R1 * R1 * R2 * R2 - x * x);

    float theta4 = atan2(y, x);

    if (theta4 < 0 || theta4 > 140.0 * M_PI / 180.0) // the4 범위 : 0deg ~ 120deg
    {
        std::cout << "IKFUN (q4) is not solved!!\n";
        state.main = Main::Error;
    }

    float theta34 = atan2(sqrt(r2), zeta);
    float theta3 = theta34 - atan2(R2 * sin(theta4), R1 + R2 * cos(theta4));

    if (theta3 < -45.0 * M_PI / 180.0 || theta3 > 90.0 * M_PI / 180.0) // the3 범위 : -45deg ~ 90deg
    {
        std::cout << "IKFUN (q3) is not solved!!\n";
        state.main = Main::Error;
    }

    zeta = z0 - ZL;
    r2 = (YL - shoulderYL) * (YL - shoulderYL) + (XL - shoulderXL) * (XL - shoulderXL); // r^2

    x = zeta * zeta + r2 - L1 * L1 - L2 * L2;
    y = sqrt(4.0 * L1 * L1 * L2 * L2 - x * x);

    float theta6 = atan2(y, x);

    if (theta6 < 0 || theta6 > 140.0 * M_PI / 180.0) // the6 범위 : 0deg ~ 120deg
    {
        std::cout << "IKFUN (q6) is not solved!!\n";
        state.main = Main::Error;
    }

    float theta56 = atan2(sqrt(r2), zeta);
    float theta5 = theta56 - atan2(L2 * sin(theta6), L1 + L2 * cos(theta6));

    if (theta5 < -45.0 * M_PI / 180.0 || theta5 > 90.0 * M_PI / 180.0) // the5 범위 : -45deg ~ 90deg
    {
        std::cout << "IKFUN (q5) is not solved!!\n";
        state.main = Main::Error;
    }

    Qf.resize(9);
    Qf << theta0, theta1, theta2, theta3, theta4, theta5, theta6, 0, 0;

    return Qf;
}

VectorXd TestManager::calWaistAngle(VectorXd pR, VectorXd pL)
{
    PartLength partLength;

    float XR = pR(0), YR = pR(1), ZR = pR(2);
    float XL = pL(0), YL = pL(1), ZL = pL(2);
    float R1 = partLength.upperArm;
    float R2 = partLength.lowerArm + partLength.stick;
    float L1 = partLength.upperArm;
    float L2 = partLength.lowerArm + partLength.stick;
    float s = partLength.waist;
    float z0 = partLength.height;

    VectorXd W(2);
    W << 2, 1;
    double minCost = W.sum();
    double w = 0, cost = 0;
    int minIndex = 0;

    MatrixXd Qarr(7, 1);
    VectorXd output(3);
    int j = 0;

    for (int i = 0; i < 1801; i++)
    {
        double theta0 = -0.5 * M_PI + M_PI / 1800.0 * i; // the0 범위 : -90deg ~ 90deg

        float shoulderXR = 0.5 * s * cos(theta0);
        float shoulderYR = 0.5 * s * sin(theta0);
        float shoulderXL = -0.5 * s * cos(theta0);
        float shoulderYL = -0.5 * s * sin(theta0);

        float theta01 = atan2(YR - shoulderYR, XR - shoulderXR);
        float theta1 = theta01 - theta0;

        if (theta1 > 0 && theta1 < 150.0 * M_PI / 180.0) // the1 범위 : 0deg ~ 150deg
        {
            float theta02 = atan2(YL - shoulderYL, XL - shoulderXL);
            float theta2 = theta02 - theta0;

            if (theta2 > 30 * M_PI / 180.0 && theta2 < M_PI) // the2 범위 : 30deg ~ 180deg
            {
                float zeta = z0 - ZR;
                float r2 = (YR - shoulderYR) * (YR - shoulderYR) + (XR - shoulderXR) * (XR - shoulderXR); // r^2

                float x = zeta * zeta + r2 - R1 * R1 - R2 * R2;

                if (4.0 * R1 * R1 * R2 * R2 - x * x > 0)
                {
                    float y = sqrt(4.0 * R1 * R1 * R2 * R2 - x * x);

                    float theta4 = atan2(y, x);

                    if (theta4 > 0 && theta4 < 140.0 * M_PI / 180.0) // the4 범위 : 0deg ~ 120deg
                    {
                        float theta34 = atan2(sqrt(r2), zeta);
                        float theta3 = theta34 - atan2(R2 * sin(theta4), R1 + R2 * cos(theta4));

                        if (theta3 > -45.0 * M_PI / 180.0 && theta3 < 90.0 * M_PI / 180.0) // the3 범위 : -45deg ~ 90deg
                        {
                            zeta = z0 - ZL;
                            r2 = (YL - shoulderYL) * (YL - shoulderYL) + (XL - shoulderXL) * (XL - shoulderXL); // r^2

                            x = zeta * zeta + r2 - L1 * L1 - L2 * L2;

                            if (4.0 * L1 * L1 * L2 * L2 - x * x > 0)
                            {
                                y = sqrt(4.0 * L1 * L1 * L2 * L2 - x * x);

                                float theta6 = atan2(y, x);

                                if (theta6 > 0 && theta6 < 140.0 * M_PI / 180.0) // the6 범위 : 0deg ~ 120deg
                                {
                                    float theta56 = atan2(sqrt(r2), zeta);
                                    float theta5 = theta56 - atan2(L2 * sin(theta6), L1 + L2 * cos(theta6));

                                    if (theta5 > -45.0 * M_PI / 180.0 && theta5 < 90.0 * M_PI / 180.0) // the5 범위 : -45deg ~ 90deg
                                    {
                                        if (j == 0)
                                        {
                                            Qarr(0, 0) = theta0;
                                            Qarr(1, 0) = theta1;
                                            Qarr(2, 0) = theta2;
                                            Qarr(3, 0) = theta3;
                                            Qarr(4, 0) = theta4;
                                            Qarr(5, 0) = theta5;
                                            Qarr(6, 0) = theta6;

                                            j = 1;
                                        }
                                        else
                                        {
                                            Qarr.conservativeResize(Qarr.rows(), Qarr.cols() + 1);

                                            Qarr(0, j) = theta0;
                                            Qarr(1, j) = theta1;
                                            Qarr(2, j) = theta2;
                                            Qarr(3, j) = theta3;
                                            Qarr(4, j) = theta4;
                                            Qarr(5, j) = theta5;
                                            Qarr(6, j) = theta6;

                                            j++;
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
        cout << "IKFUN is not solved!! (Waist Range)\n";
        state.main = Main::Error;

        output(0) = 0;
        output(1) = 0;
    }
    else
    {
        output(0) = Qarr(0, 0);     // min
        output(1) = Qarr(0, j - 1); // max
    }

    w = 2.0 * M_PI / abs(Qarr(0, j - 1) - Qarr(0, 0));
    for (int i = 0; i < j; i++)
    {
        cost = W(0)*cos(Qarr(1, i) + Qarr(2, i)) + W(1)*cos(w*abs(Qarr(0, i) - Qarr(0, 0)));

        if (cost < minCost)
        {
            minCost = cost;
            minIndex = i;
        }
    }
    output(2) = Qarr(0, minIndex);

    return output;
}

