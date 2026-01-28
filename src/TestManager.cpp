#include "../include/managers/TestManager.hpp" // 적절한 경로로 변경하세요.
#include "TestManager.hpp"

// For  Qt
// #include "../managers/TestManager.hpp"
using namespace std;
using namespace cv;

TestManager::TestManager(State &stateRef, CanManager &canManagerRef, std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef, USBIO &usbioRef, Functions &funRef)
    : state(stateRef), canManager(canManagerRef), motors(motorsRef), usbio(usbioRef), func(funRef)
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
    
        std::cout << "\nSelect Method (1 - 관절각도값 조절, 2 - 좌표값 조절, 4 - 발 모터, 5 - 속도 제어 실험, 6 - 브레이크, 7 - 허리 모터, -1 - 나가기) : ";
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


                std::cout << "\nSelect Motor to Change Value (R: 10, L: 11) / Run (1) / Time (2) / Extra Time (3) / Repeat (4) / sine traj (5) / Exit (-1): ";
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
        else if(method == 5)
        {
            testTmotorVelocityMode();
        }
        else if(method == 6)
        {
            int state_brake;
            cin >> state_brake;
            //useBrake가 1이면 브레이크 켜줌 0이면 꺼줌
            usbio.setUSBIO4761(0, state_brake); //세팅
            usbio.outputUSBIO4761();                    //실행
        }
        else if(method == 7)
        {
            float t_now = 0.0f;
            float dt = 0.005f;

            float target_deg;
            float move_time;
            int wait_time = move_time;
            
            float c_MotorAngle[12] = {0};
            getMotorPos(c_MotorAngle);
            
            // float start_rad = c_MotorAngle[0];
            float start_rad = c_MotorAngle[6];

            std::cout << "========================================" << std::endl;
            std::cout << " [Waist Moving mode] " << std::endl;
            std::cout << " Current Waist Angle: " << start_rad * 180.0 / M_PI << " [deg]" << std::endl;
            std::cout << "----------------------------------------" << std::endl;
            std::cout << " Enter Goal Angle (deg): ";
            std::cin >> target_deg;
            std::cout << " Enter Moving Time (sec): ";
            std::cin >> move_time;

            // 예외 처리: 시간이 0 이하이면 실행 방지
            if (move_time <= 0) {
                std::cout << " [Error] Time must be > 0." << std::endl;
                return; 
            }

            float target_rad = target_deg * M_PI / 180.0f;

            std::cout << " Moving Start..." << std::endl;

            // 4. 제어 루프 시작
            while(t_now <= move_time)
            {   
                float Q = ((target_rad - start_rad) / 2.0f) * cos(M_PI * (t_now / move_time + 1.0f)) + ((target_rad + start_rad) / 2.0f);

                for (auto &entry : motors)
                {
                    if (entry.first == "Waist") 
                    {
                        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
                        {
                            TMotorData newData;
                            
                            newData.position = tMotor->jointAngleToMotorPosition(Q);
                            newData.mode = tMotor->Position; 
                            tMotor->commandBuffer.push(newData);
                            tMotor->finalMotorPosition = newData.position;
                        }
                    }
                }
                t_now += dt;
            }
            for(int i = 0; i<wait_time; i++)
            {
                std::cout << i << "\t";
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            std::cout << " Moving Complete." << std::endl;

            while (1)
            {
                std::cout << "mode 입력 (1: calibration, 2: check, -1: exit): ";
                int mode;
                std::cin >> mode;

                if (mode == 1)
                {
                    camera_calibration(target_deg);
                    std::cout << "Scanning Complete." << std::endl;
                }
                else if (mode == 2)
                {
                    std::cout << "Offset Matrix file name : ";
                    std::string offset_filename;
                    std::cin >> offset_filename;

                    std::cout << "current angle : ";
                    double angle;
                    std::cin >> angle;

                    measure_and_log(angle, offset_filename);
                    std::cout << "Checking calibration Complete." << std::endl;
                }
                else if (mode == -1)
                {
                    std::cout << "bye" << std::endl;
                    break;
                }
                else
                {
                    std::cout << "wrong command" << std::endl;
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

void TestManager::testTmotorVelocityMode()
{
    while(true)
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

        std::cout << "\ntime : " << t << "s";
        std::cout << "\nnumber of repeat : " << n_repeat << std::endl << std::endl;


        std::cout << "\nSelect Motor to Change Value (0-8) / Run (9) / Time (10) / Extra Time (11) / Repeat(12) / Exit (-1): ";
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
            pushVelCmd(q);
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

void TestManager::pushVelCmd(float arr[])
{
    const float acc_max = 100.0;    // rad/s^2
    vector<float> Qi;
    vector<float> Vi;
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
                Vi = makeVelProfile(Q1, Q2, Vmax, acc_max, t*k/n, t);
            }
            else
            {
                Qi = makeProfile(Q2, Q1, Vmax, acc_max, t*k/n, t);
                Vi = makeVelProfile(Q2, Q1, Vmax, acc_max, t*k/n, t);
            }

            // Send to Buffer
            for (auto &entry : motors)
            {
                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
                {
                    TMotorData newData;
                    newData.position = tMotor->jointAngleToMotorPosition(Qi[canManager.motorMapping[entry.first]]);
                    newData.mode = tMotor->Velocity;
                    newData.velocityERPM = Vi[canManager.motorMapping[entry.first]] * 60.0 * 21.0 * 10 / 2.0 / M_PI;
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

vector<float> TestManager::makeVelProfile(float q1[], float q2[], vector<float> &Vmax, float acc, float t, float t2)
{
    vector<float> Vi;
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
            val = 0;
        }
        else if (Vmax[i] < 0)
        {
            // Vmax 값을 구하지 못했을 때 삼각형 프로파일 생성
            float acc_tri = 4 * S / t2 / t2;
            if (t < t2/2)
            {
                val = sign * acc_tri * t;
            }
            else if (t < t2)
            {
                val = sign * acc_tri * (t2 - t);
            }
            else
            {
                val = 0;
            }
        }
        else
        {
            // 사다리꼴 프로파일
            if (t < Vmax[i] / acc)
            {
                // 가속
                val = sign * acc * t;
            }
            else if (t < S / Vmax[i])
            {
                // 등속
                val = sign * Vmax[i];          
            }
            else if (t < Vmax[i] / acc + S / Vmax[i])
            {
                // 감속
                val = sign * acc * (S / Vmax[i] + Vmax[i] / acc - t);          
            }
            else 
            {
                val = 0;
            }
        }
        Vi.push_back(val);
    }
    return Vi;
}

 /////////////////////////////////////////////////////////////////////////////////
/*                                  Drum Scan                                  */
/////////////////////////////////////////////////////////////////////////////////

void TestManager::DrumScan(float Waist_angle)
{
    try {
        // --- 초기 설정 ---
        auto dictionary_name = aruco::DICT_4X4_50;
        
        rs2::pipeline pipe;
        rs2::config cfg;

        ofstream outFile;

        int rgb_width = 1280;
        int rgb_height = 720;

        // USB3.0 사용 시 부활!
        // int rgb_fps = 30;
        // int depth_width = 848;
        // int depth_height = 480;
        // int depth_fps = 30;

        cfg.enable_stream(RS2_STREAM_COLOR, rgb_width, rgb_height, RS2_FORMAT_BGR8, 15);
        cfg.enable_stream(RS2_STREAM_DEPTH, 480, 270, RS2_FORMAT_Z16, 6);
        
        rs2::pipeline_profile profile = pipe.start(cfg);
        
        // Depth를 Color 시점으로 정렬
        rs2::align align_to_color(RS2_STREAM_COLOR);

        // [중요] Color 스트림의 내부 파라미터 가져오기 (영점 보정용)
        rs2_intrinsics intrin = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();

        // ArUco 설정
        Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(dictionary_name);
        Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();

        struct DrumTarget { Point3D left_hand; Point3D right_hand; };
        map<int, DrumTarget> drum_map;

        cout << "========== 드럼 스캔 (허리 각도: " << Waist_angle << ") ==========" << endl;
        cout << "종료하려면 창을 클릭하고 'q'를 누르세요." << endl;

        int frame_count = 0;

        while (true) {
            // 프레임 대기 (5초 타임아웃으로 멈춤 방지)
            rs2::frameset frames;
            try {
                frames = pipe.wait_for_frames(5000);
            } catch (...) {
                continue;
            }
            
            frames = align_to_color.process(frames);

            rs2::video_frame color_frame = frames.get_color_frame();
            rs2::depth_frame depth_frame = frames.get_depth_frame();
            if (!color_frame || !depth_frame) continue;

            Mat color_image(Size(rgb_width, rgb_height), CV_8UC3, (void*)color_frame.get_data());

            // 마커 감지
            vector<int> ids;
            vector<vector<Point2f>> corners;
            aruco::detectMarkers(color_image, dictionary, corners, ids, parameters);

            // [시각화 1] 화면 중앙 십자가 그리기
            float center_x = rgb_width / 2.0f;
            float center_y = rgb_height / 2.0f;
            line(color_image, Point(center_x, 0), Point(center_x, rgb_height), Scalar(0, 0, 255), 1);
            line(color_image, Point(0, center_y), Point(rgb_width, center_y), Scalar(0, 0, 255), 1);

            // [시각화 2] 중앙점 좌표 디버깅
            float center_dist = depth_frame.get_distance((int)center_x, (int)center_y);
            if (center_dist > 0) {
                float c_pixel[2] = { center_x, center_y };
                float c_point[3];
                rs2_deproject_pixel_to_point(c_point, &intrin, c_pixel, center_dist);

                string c_text = format("Center: (%.3f, %.3f, %.3f)", c_point[0], c_point[1], c_point[2]);
                putText(color_image, c_text, Point(center_x + 10, center_y + 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
            }

            // [마커 처리]
            if (ids.size() > 0) {
                aruco::drawDetectedMarkers(color_image, corners, ids);

                for (size_t i = 0; i < ids.size(); ++i) {
                    // 마커 중심 계산
                    Point2f center(0, 0);
                    for (int j = 0; j < 4; j++) center += corners[i][j];
                    center *= 0.25f;

                    float dist = depth_frame.get_distance((int)center.x, (int)center.y);
                    if (dist > 0) {
                        float pixel[2] = { center.x, center.y };
                        float marker_cam[3];
                        rs2_deproject_pixel_to_point(marker_cam, &intrin, pixel, dist);

                        // [시각화 3] 마커 좌표 출력
                        string label = format("X:%.2f Y:%.2f Z:%.2f", marker_cam[0], marker_cam[1], marker_cam[2]);
                        putText(color_image, label, center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 2);

                        // 손 오프셋 계산
                        Point3D left_cam_pt = { marker_cam[0] - HAND_OFFSET_X, marker_cam[1] + HAND_OFFSET_Y, marker_cam[2] };
                        Point3D right_cam_pt = { marker_cam[0] + HAND_OFFSET_X, marker_cam[1] + HAND_OFFSET_Y, marker_cam[2] };

                        // 월드 좌표 변환 및 저장
                        drum_map[ids[i]] = { 
                            transform_to_world(left_cam_pt, ROBOT_WAIST_ANGLE, CAMERA_TILT_ANGLE),
                            transform_to_world(right_cam_pt, ROBOT_WAIST_ANGLE, CAMERA_TILT_ANGLE)
                        };
                    }
                }
            }

            // 매 {frame_count}개의 프레임마다 저장
            frame_count++;
            if (frame_count % 15 == 0) {
                outFile.open("../drum_coordinates.txt");
                if (outFile.is_open()) {
                    outFile << fixed << setprecision(6); // 소수점 6자리 고정
                    // 1. 오른손 X
                    for (int id = 1; id < TOTAL_DRUMS; ++id) {
                        if (drum_map.count(id)) outFile << drum_map[id].right_hand.x << "\t";
                        else outFile << 0.0f << "\t";
                    }
                    outFile << endl;

                    // 2. 오른손 Y
                    for (int id = 1; id < TOTAL_DRUMS; ++id) {
                        if (drum_map.count(id)) outFile << drum_map[id].right_hand.y << "\t";
                        else outFile << 0.0f << "\t";
                    }
                    outFile << endl;

                    // 3. 오른손 Z
                    for (int id = 1; id < TOTAL_DRUMS; ++id) {
                        if (drum_map.count(id)) outFile << drum_map[id].right_hand.z << "\t";
                        else outFile << 0.0f << "\t";
                    }
                    outFile << endl;

                    // 4. 왼손 X
                    for (int id = 1; id < TOTAL_DRUMS; ++id) {
                        if (drum_map.count(id)) outFile << drum_map[id].left_hand.x << "\t";
                        else outFile << 0.0f << "\t";
                    }
                    outFile << endl;

                    // 5. 왼손 Y
                    for (int id = 1; id < TOTAL_DRUMS; ++id) {
                        if (drum_map.count(id)) outFile << drum_map[id].left_hand.y << "\t";
                        else outFile << 0.0f << "\t";
                    }
                    outFile << endl;

                    // 6. 왼손 Z
                    for (int id = 1; id < TOTAL_DRUMS; ++id) {
                        if (drum_map.count(id)) outFile << drum_map[id].left_hand.z << "\t";
                        else outFile << 0.0f << "\t";
                    }
                    outFile << endl;
                }
                else{ std::cout << " file isn't open " << endl;cerr << "\n[에러] 파일 생성 실패! 경로 권한을 확인하세요." << endl;}
                cout << "\r[저장완료] 감지된 드럼 수: " << drum_map.size() << " / " << TOTAL_DRUMS << "   " << flush;
            }

            imshow("Robot Eye View", color_image);
            if (waitKey(1) == 'q') break;
        }
        cv::destroyAllWindows(); // 열린 창 닫기
        pipe.stop();             // 카메라 스트리밍 중지
        return;
    }
    catch (const rs2::error & e) {
        cerr << "RealSense Error: " << e.what() << endl;
        return;
    }
    catch (const exception & e) {
        cerr << "Error: " << e.what() << endl;
        return;
    }
}

TestManager::Point3D TestManager::transform_to_world(Point3D cam_pt, float waist_deg, float tilt_deg)
{
    const float PI = 3.14159265f;
    float theta = tilt_deg * PI / 180.0f;
    float psi = waist_deg * PI / 180.0f;

    // 0. cam_pt: 카메라 좌표축 기준 좌표값

    // 1. 아래로 숙인 카메라 각도 보정
    float y_untilted = cam_pt.y * cos(theta) - cam_pt.z * sin(theta);
    float z_untilted = cam_pt.y * sin(theta) + cam_pt.z * cos(theta);
    float x_untilted = cam_pt.x;

    // 2. world 좌표로 변환하기 위해 좌표축 변환(카메라 좌표는 화면 좌상단이 원점, 오른쪽 가로방향: +x, 아래 세로방향: +y, 깊이: +z)
    float x_body = x_untilted;
    float y_body = z_untilted;
    float z_body = -y_untilted;

    // 3. 허리 회전 각도 보정
    float x_world = x_body * cos(psi) - (y_body + CAMERA_OFFSET_FWD) * sin(psi);
    float y_world = x_body * sin(psi) + (y_body + CAMERA_OFFSET_FWD) * cos(psi);
    float z_world = z_body + CAMERA_HEIGHT;

    // cout << "------------------------------------------------" << endl;
    // cout << "[1.Input Cam] X:" << cam_pt.x << "\t Y:" << cam_pt.y << "\t Z:" << cam_pt.z << endl;
    // cout << "[2.Untilted] X:" << x_untilted << "\t Y:" << y_untilted << "\t Z:" << z_untilted << endl;
    // cout << "[3.Body Frame] X:" << x_body << "\t Y:" << y_body << "\t Z:" << z_body << endl;
    // cout << "[4.Final World] X:" << x_world << "\t Y:" << y_world << "\t Z:" << z_world << endl;
    // cout << "------------------------------------------------" << endl;

    return {x_world, y_world, z_world};
}

cv::Mat TestManager::getIdentity() 
{
    return cv::Mat::eye(4, 4, CV_64F);
}

// rvec, tvec를 4x4 변환 행렬로 변환
cv::Mat TestManager::getTransformMatrix(const cv::Vec3d& rvec, const cv::Vec3d& tvec) 
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    T.at<double>(0, 3) = tvec[0];
    T.at<double>(1, 3) = tvec[1];
    T.at<double>(2, 3) = tvec[2];
    
    return T;
}

cv::Mat TestManager::getMarkerWorldPose(double x, double y, double z) 
{
    // 마커 좌표계: 빨간색(x) 오른쪽, 초록색(y) 앞쪽, 파란색(z) 위쪽
    // 월드 좌표계와 방향이 일치한다고 가정 (Identity Rotation)
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    T.at<double>(0, 3) = x;
    T.at<double>(1, 3) = y;
    T.at<double>(2, 3) = z;
    return T;
}

void TestManager::printMatrix(const std::string& name, const cv::Mat& M) 
{
    std::cout << "[" << name << "]" << std::endl;
    for (int i = 0; i < 4; i++) {
        std::cout << "  ";
        for (int j = 0; j < 4; j++) {
            printf("%7.3f ", M.at<double>(i, j));
        }
        std::cout << std::endl;
    }
}

cv::Vec3d TestManager::getEulerAngles(cv::Mat R_in) 
{
    // 3x3 행렬이 아니면 예외 처리 필요하지만 여기선 생략
    cv::Mat mtxR, mtxQ;
    // RQ 분해를 이용해 Euler Angle (x, y, z 축 회전) 추출
    cv::Vec3d angles = cv::RQDecomp3x3(R_in, mtxR, mtxQ);
    return angles;
}

void TestManager::saveCornersToCSV(const std::string& filename, const std::vector<int>& ids, const std::vector<std::vector<cv::Point2f>>& corners) 
{
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "파일을 열 수 없습니다: " << filename << std::endl;
        return;
    }

    // 헤더 작성 (순서: ID, 좌상단, 우상단, 좌하단, 우하단)
    file << "id,tl_x,tl_y,tr_x,tr_y,bl_x,bl_y,br_x,br_y\n";

    for (size_t i = 0; i < ids.size(); ++i) {
        // ArUco 기본 corner 순서: 0=TL(좌상단), 1=TR(우상단), 2=BR(우하단), 3=BL(좌하단)
        // 사용자 요청 순서: TL -> TR -> BL -> BR
        
        const auto& c = corners[i]; // 현재 마커의 코너들

        file << ids[i] << ","
             << c[0].x << "," << c[0].y << ","  // 좌상단 (Top-Left)
             << c[1].x << "," << c[1].y << ","  // 우상단 (Top-Right)
             << c[3].x << "," << c[3].y << ","  // 좌하단 (Bottom-Left) -> 인덱스 3
             << c[2].x << "," << c[2].y         // 우하단 (Bottom-Right) -> 인덱스 2
             << "\n";
    }

    file.close();
    // std::cout << filename << " 저장 완료 (" << ids.size() << "개 마커)" << std::endl;
}

void TestManager::savePoseToCSV(const std::string& filename, const std::vector<int>& ids, const std::vector<cv::Vec3d>& rvecs, const std::vector<cv::Vec3d>& tvecs) 
{
    // 데이터 개수 정합성 확인
    if (ids.size() != rvecs.size() || ids.size() != tvecs.size()) {
        std::cerr << "오류: ID와 포즈 데이터(rvecs, tvecs)의 개수가 맞지 않습니다." << std::endl;
        return;
    }

    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "파일을 열 수 없습니다: " << filename << std::endl;
        return;
    }

    // 헤더 작성 (ID, 회전 벡터 x,y,z, 이동 벡터 x,y,z)
    file << "id,rvec_x,rvec_y,rvec_z,tvec_x,tvec_y,tvec_z\n";

    for (size_t i = 0; i < ids.size(); ++i) {
        const auto& r = rvecs[i]; // 회전 벡터
        const auto& t = tvecs[i]; // 이동 벡터 (카메라 기준 마커 위치)

        file << ids[i] << ","
             << r[0] << "," << r[1] << "," << r[2] << ","
             << t[0] << "," << t[1] << "," << t[2]
             << "\n";
    }

    file.close();
    // std::cout << filename << " 저장 완료 (" << ids.size() << "개 데이터)" << std::endl;
}

void TestManager::saveMatrixToCSV(const std::string& filename, const cv::Mat& M) 
{
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "[Error] 파일을 열 수 없습니다: " << filename << std::endl;
        return;
    }

    // 소수점 6자리까지 정밀하게 저장 (캘리브레이션 데이터는 정밀도 중요)
    file << std::fixed << std::setprecision(6);

    for (int i = 0; i < M.rows; ++i) {
        for (int j = 0; j < M.cols; ++j) {
            // double형으로 접근하여 값 쓰기
            file << M.at<double>(i, j);
            
            // 마지막 열이 아니면 콤마(,) 추가
            if (j < M.cols - 1) {
                file << ",";
            }
        }
        // 행이 바뀌면 줄바꿈
        file << "\n";
    }

    file.close();
    std::cout << "[Saved] 캘리브레이션 데이터 저장 완료: " << filename << std::endl;
}

// CSV 파일에서 4x4 행렬 불러오기 (프로그램 시작 시 사용)
cv::Mat TestManager::loadMatrixFromCSV(const std::string& filename) 
{
    cv::Mat M = cv::Mat::eye(4, 4, CV_64F); // 기본 단위 행렬로 초기화
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "[Warning] 파일을 찾을 수 없습니다 (기본값 사용): " << filename << std::endl;
        return M;
    }

    std::string line;
    int row = 0;

    // 한 줄씩 읽기
    while (std::getline(file, line) && row < 4) {
        std::stringstream ss(line);
        std::string cell;
        int col = 0;

        // 콤마(,)를 기준으로 값 분리
        while (std::getline(ss, cell, ',') && col < 4) {
            try {
                // 문자열을 double로 변환하여 행렬에 대입
                M.at<double>(row, col) = std::stod(cell);
            } catch (...) {
                std::cerr << "[Error] 데이터 파싱 실패: " << cell << std::endl;
            }
            col++;
        }
        row++;
    }

    file.close();
    // std::cout << "[Loaded] 캘리브레이션 데이터 로드 완료" << std::endl;
    return M;
}

void TestManager::saveAnalysisLog(const std::string& filename, 
                     double waist_angle, 
                     const std::string& data_type, // "INDIVIDUAL" or "AVERAGE"
                     int marker_id,                // 평균일 경우 -1 등 표기
                     const cv::Vec3d& tvec_raw,    // 원본 tvec (x, y)
                     double z_rgb,                 // RGB 추정 깊이
                     double z_depth,               // Depth 센서 깊이
                     const cv::Mat& T_World_Cam,   // (4 or 5) 카메라 월드 행렬
                     const cv::Mat& T_Offset)      // (6) 최종 오프셋 행렬
{
    static bool headers_written = false;
    std::ofstream file;

    if (!headers_written) {
        file.open(filename); // 새 파일 생성
        file << "timestamp,waist_angle,data_type,marker_id,"
             << "raw_x,raw_y,z_rgb,z_depth,"
             << "world_cam_x,world_cam_y,world_cam_z,"
             << "offset_x,offset_y,offset_z\n";
        headers_written = true;
    } else {
        file.open(filename, std::ios::app); // 이어쓰기
    }

    if (!file.is_open()) return;

    // 타임스탬프 (현재 시간)
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    char time_str[100];
    std::strftime(time_str, sizeof(time_str), "%H:%M:%S", std::localtime(&now_time));

    file << std::fixed << std::setprecision(4);

    // 1. 기본 정보
    file << time_str << "," << waist_angle << "," << data_type << "," << marker_id << ",";

    // 2. 원본 측정값 (Average일 경우 tvec_raw 등은 0이나 평균값 기록)
    file << tvec_raw[0] << "," << tvec_raw[1] << "," << z_rgb << "," << z_depth << ",";

    // 3. (4 or 5번) 카메라 월드 위치 (T_World_Cam의 x,y,z)
    file << T_World_Cam.at<double>(0, 3) << "," 
         << T_World_Cam.at<double>(1, 3) << "," 
         << T_World_Cam.at<double>(2, 3) << ",";

    // 4. (6번) 최종 오프셋 위치 (T_Offset의 x,y,z)
    file << T_Offset.at<double>(0, 3) << "," 
         << T_Offset.at<double>(1, 3) << "," 
         << T_Offset.at<double>(2, 3) << "\n";

    file.close();
}

/*
void TestManager::camera_calibration_ex(double CURRENT_WAIST_ANGLE_DEG)
{
    try {
        // 0. 파일명 동적 생성 (예: calibration_log_45deg.csv)
        // 각도를 정수형(int)으로 변환하여 파일명에 포함
        int angle_int = (int)std::round(CURRENT_WAIST_ANGLE_DEG);
        std::string suffix = "_" + std::to_string(angle_int) + "deg.csv";
        std::string angle_suffix = "_" + std::to_string(angle_int) + "deg.csv";

        std::string file_corners = "corners" + suffix;
        std::string file_pose    = "pose_data" + suffix;
        std::string file_log     = "calibration_log" + suffix;

        // 알고 있는 마커 좌표
        struct MarkerPos { double x, y, z; };
        std::map<int, MarkerPos> KNOWN_MARKERS = {
            // {0, {-0.773, 0.00, 0.08}}
            // ,{1, {-0.683, 0.00, 0.08}}
            // ,{2, {-0.593, 0.00, 0.08}}

            {0, {-0.773, 0.00, 0.08}}
            ,{1, {-0.713, 0.00, 0.08}}
            ,{2, {-0.653, 0.00, 0.08}}
            ,{3, {-0.593, 0.00, 0.08}}
        };

        // 1. RealSense 설정
        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
        
        rs2::pipeline_profile profile = pipe.start(cfg);
        rs2::align align_to_color(RS2_STREAM_COLOR);

        // Intrinsic
        auto stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        rs2_intrinsics intr = stream.get_intrinsics();
        cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        cameraMatrix.at<double>(0, 0) = intr.fx; cameraMatrix.at<double>(1, 1) = intr.fy;
        cameraMatrix.at<double>(0, 2) = intr.ppx; cameraMatrix.at<double>(1, 2) = intr.ppy;
        cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
        for(int i=0; i<5; i++) distCoeffs.at<double>(i) = intr.coeffs[i];

        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

        std::cout << "=== Calibration Start: " << angle_int << " deg ===" << std::endl;
        std::cout << "Waiting for stable detection (approx 1 sec)..." << std::endl;

        // [핵심 변수] 안정화 카운트
        int stable_frame_count = 0; 
        const int REQUIRED_STABLE_FRAMES = 90; // 30프레임(약 1초) 동안 마커가 보여야 저장함

        while (true) {
            rs2::frameset frames = pipe.wait_for_frames();
            frames = align_to_color.process(frames); 
            
            rs2::frame color_frame = frames.get_color_frame();
            rs2::depth_frame depth_frame = frames.get_depth_frame();
            if (!color_frame || !depth_frame) continue;

            cv::Mat image(cv::Size(848, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters);

            if (ids.size() > 0) {
                // 마커가 잡히면 카운트 증가
                stable_frame_count++;

                // 화면에 진행 상황 표시
                std::string status = "Stabilizing: " + std::to_string(stable_frame_count) + "/" + std::to_string(REQUIRED_STABLE_FRAMES);
                cv::putText(image, status, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

                // [조건 충족] 30프레임 이상 안정적으로 잡혔을 때 -> 딱 한 번 저장하고 종료
                if (stable_frame_count >= REQUIRED_STABLE_FRAMES) {
                    
                    std::cout << ">>> Capturing Data for " << angle_int << " deg..." << std::endl;

                    // 1. Pose 추정
                    std::vector<cv::Vec3d> rvecs, tvecs;
                    cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

                    // 2. 허리 역행렬 준비
                    double rad = CURRENT_WAIST_ANGLE_DEG * CV_PI / 180.0;
                    cv::Mat T_Waist_Rot = cv::Mat::eye(4, 4, CV_64F);
                    T_Waist_Rot.at<double>(0, 0) = cos(rad); T_Waist_Rot.at<double>(0, 1) = -sin(rad);
                    T_Waist_Rot.at<double>(1, 0) = sin(rad); T_Waist_Rot.at<double>(1, 1) = cos(rad);
                    cv::Mat T_Waist_Inv = T_Waist_Rot.inv();

                    // 평균 계산용 변수
                    cv::Mat T_World_Camera_Sum = cv::Mat::zeros(4, 4, CV_64F);
                    int valid_count = 0;

                    // 3. 개별 마커 처리 및 저장
                    for (size_t i = 0; i < ids.size(); ++i) {
                        int id = ids[i];
                        
                        // Depth 보정
                        double z_rgb = tvecs[i][2];
                        double z_depth = 0.0;
                        
                        cv::Point2f center = (corners[i][0] + corners[i][1] + corners[i][2] + corners[i][3]) * 0.25f;
                        int cx = std::round(center.x), cy = std::round(center.y);
                        double d_sum = 0; int d_cnt = 0;
                        for(int dy=-2; dy<=2; dy++) {
                            for(int dx=-2; dx<=2; dx++) {
                                int u=cx+dx, v=cy+dy;
                                if(u>=0 && u<848 && v>=0 && v<480) {
                                    double d = depth_frame.get_distance(u, v);
                                    if(d>0.1 && d<5.0) { d_sum+=d; d_cnt++; }
                                }
                            }
                        }
                        if(d_cnt > 0) {
                            z_depth = d_sum / d_cnt;
                            tvecs[i][2] = z_depth; // Z값 교체
                        } else {
                            z_depth = z_rgb;
                        }

                        if (KNOWN_MARKERS.find(id) != KNOWN_MARKERS.end()) {
                            cv::Mat T_Cam_Marker = getTransformMatrix(rvecs[i], tvecs[i]);
                            MarkerPos pos = KNOWN_MARKERS[id];
                            cv::Mat T_World_Marker = getMarkerWorldPose(pos.x, pos.y, pos.z);
                            cv::Mat T_World_Cam_Indiv = T_World_Marker * T_Cam_Marker.inv();
                            cv::Mat T_Offset_Indiv = T_Waist_Inv * T_World_Cam_Indiv;

                            // --- [저장 A] 개별 행렬 저장 (Individual) ---
                            // 파일명 예시: Matrix_World_Indiv_ID0_30deg.csv
                            std::string name_world_indiv = "Matrix_World_Indiv_ID" + std::to_string(id) + angle_suffix;
                            std::string name_offset_indiv = "Matrix_Offset_Indiv_ID" + std::to_string(id) + angle_suffix;

                            saveMatrixToCSV(name_world_indiv, T_World_Cam_Indiv);
                            saveMatrixToCSV(name_offset_indiv, T_Offset_Indiv);

                            // [저장 A] 개별 데이터
                            saveAnalysisLog(file_log, CURRENT_WAIST_ANGLE_DEG, "INDIVIDUAL", id, 
                                            tvecs[i], z_rgb, z_depth, T_World_Cam_Indiv, T_Offset_Indiv);

                            T_World_Camera_Sum += T_World_Cam_Indiv;
                            valid_count++;
                        }
                    }

                    // 4. 평균 데이터 처리 및 저장
                    if (valid_count > 0) {
                        cv::Mat T_World_Cam_Avg = T_World_Camera_Sum / valid_count;
                        cv::Mat T_Offset_Avg = T_Waist_Inv * T_World_Cam_Avg;

                        std::string name_world_avg = "Matrix_World_Avg" + angle_suffix;
                        std::string name_offset_avg = "Matrix_Offset_Avg" + angle_suffix;

                        saveMatrixToCSV(name_world_avg, T_World_Cam_Avg);
                        saveMatrixToCSV(name_offset_avg, T_Offset_Avg);

                        // [저장 B] 평균 데이터
                        saveAnalysisLog(file_log, CURRENT_WAIST_ANGLE_DEG, "AVERAGE", -1, 
                                        cv::Vec3d(0,0,0), 0, 0, T_World_Cam_Avg, T_Offset_Avg);
                        
                        // [저장 C] 원본 코너 및 포즈 데이터 (필요시)
                        saveCornersToCSV(file_corners, ids, corners);
                        savePoseToCSV(file_pose, ids, rvecs, tvecs);

                        std::cout << ">>> [Complete] Data Saved to " << file_log << std::endl;
                        
                        // [종료] 저장 완료했으므로 루프 탈출 (함수 종료)
                        break; 
                    }
                }
            } else {
                // 마커가 안 보이면 카운트 초기화 (연속 30프레임 조건)
                stable_frame_count = 0;
                cv::putText(image, "Looking for markers...", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            }

            // 모니터링용 화면 출력
            cv::imshow("Auto Calibration", image);
            if (cv::waitKey(1) == 27) break; // ESC 누르면 강제 취소
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    cv::destroyAllWindows(); 
    cv::waitKey(1); // 창 닫힘 대기
}

void TestManager::measure_and_log_ex(double current_waist_angle, const std::string& offset_filename)
{
    // 1. Offset 로드
    cv::Mat T_Offset = loadMatrixFromCSV(offset_filename);

    // 2. 허리 회전 행렬 계산
    double rad = current_waist_angle * CV_PI / 180.0;
    cv::Mat T_Waist = cv::Mat::eye(4, 4, CV_64F);
    T_Waist.at<double>(0, 0) = cos(rad); T_Waist.at<double>(0, 1) = -sin(rad);
    T_Waist.at<double>(1, 0) = sin(rad); T_Waist.at<double>(1, 1) = cos(rad);

    cv::Mat T_World_Camera = T_Waist * T_Offset;

    // --- CSV 파일 준비 ---
    std::string log_filename = "marker_world_log.csv";
    std::ofstream log_file(log_filename);
    if (log_file.is_open()) {
        log_file << "frame_idx,marker_id,world_x,world_y,world_z,c_x,c_y\n";
    }
    
    // [핵심 변수] 프레임 카운터
    int total_frames_passed = 0;    // 카메라 켜진 후 총 프레임 수
    int recorded_frames = 0;        // 실제 CSV에 저장된 프레임 수
    const int WARMUP_FRAMES = 60;   // 대기할 프레임 수 (약 2초)
    const int TARGET_FRAMES = 60;   // 저장할 프레임 수
    bool is_logging_complete = false;

    // --- RealSense & Aruco 설정 ---
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
    rs2::pipeline_profile profile = pipe.start(cfg);
    rs2::align align_to_color(RS2_STREAM_COLOR);

    auto stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    rs2_intrinsics intr = stream.get_intrinsics();
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = intr.fx; cameraMatrix.at<double>(1, 1) = intr.fy;
    cameraMatrix.at<double>(0, 2) = intr.ppx; cameraMatrix.at<double>(1, 2) = intr.ppy;
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    for(int i=0; i<5; i++) distCoeffs.at<double>(i) = intr.coeffs[i];

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

    std::cout << "=== Camera Started. Waiting for Warm-up (" << WARMUP_FRAMES << " frames)... ===" << std::endl;

    while (true) {
        rs2::frameset frames = pipe.wait_for_frames();
        frames = align_to_color.process(frames);
        
        rs2::frame color_frame = frames.get_color_frame();
        rs2::depth_frame depth_frame = frames.get_depth_frame();
        if (!color_frame || !depth_frame) continue;

        cv::Mat image(cv::Size(848, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        
        // 프레임 카운트 증가
        total_frames_passed++;

        // 마커 감지
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters);

        bool frame_has_valid_data = false; // 현재 프레임 저장 여부 플래그

        if (ids.size() > 0) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

            for (size_t i = 0; i < ids.size(); ++i) {
                // 1. Depth 보정
                cv::Point2f center = (corners[i][0] + corners[i][1] + corners[i][2] + corners[i][3]) * 0.25f;
                int cx = std::round(center.x);
                int cy = std::round(center.y);
                double d_sum = 0; int d_cnt = 0;
                for(int dy=-2; dy<=2; dy++){ for(int dx=-2; dx<=2; dx++){
                    int u=cx+dx, v=cy+dy;
                    if(u>=0 && u<848 && v>=0 && v<480){
                        double d = depth_frame.get_distance(u, v);
                        if(d>0.1 && d<5.0){ d_sum+=d; d_cnt++; }
                    }
                }}
                if(d_cnt>0) tvecs[i][2] = d_sum / d_cnt;

                // 2. 월드 좌표 계산
                cv::Mat T_Cam_Marker = getTransformMatrix(rvecs[i], tvecs[i]);
                // cv::Mat T_temp = T_World_Camera * T_Cam_Marker;
                cv::Mat T_World_Marker = T_World_Camera * T_Cam_Marker;
                // cv::Mat T_World_Marker = T_temp.inv();

                double wx = T_World_Marker.at<double>(0, 3);
                double wy = T_World_Marker.at<double>(1, 3);
                double wz = T_World_Marker.at<double>(2, 3);

                // [핵심 로직] Warm-up이 끝났고, 아직 기록이 완료되지 않았다면 CSV 저장
                if (total_frames_passed > WARMUP_FRAMES && !is_logging_complete && log_file.is_open()) {
                    log_file << recorded_frames << "," 
                             << ids[i] << "," 
                             << wx << "," << wy << "," << wz << cx << cy << "\n";
                    frame_has_valid_data = true;
                }

                // 시각화
                std::string pos_text = cv::format("ID %d: (%.3f, %.3f, %.3f)", ids[i], wx, wy, wz);
                cv::putText(image, pos_text, cv::Point(10, 30 + i * 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.04);
            }
        }

        // --- 상태 관리 및 UI 표시 ---
        if (total_frames_passed <= WARMUP_FRAMES) {
            // [Warm-up 상태]
            std::string status = cv::format("Warming up... %d/%d", total_frames_passed, WARMUP_FRAMES);
            cv::putText(image, status, cv::Point(550, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2); // 노란색
        } 
        else if (!is_logging_complete) {
            // [Logging 상태]
            if (frame_has_valid_data) {
                recorded_frames++; // 데이터가 유효하게 저장되었을 때만 카운트
            }

            std::string status = cv::format("LOGGING: %d / %d", recorded_frames, TARGET_FRAMES);
            cv::putText(image, status, cv::Point(550, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2); // 빨간색

            // 목표 달성 확인
            if (recorded_frames >= TARGET_FRAMES) {
                is_logging_complete = true;
                log_file.close();
                std::cout << ">>> [Complete] 60 Data Frames Saved." << std::endl;
            }
        } 
        else {
            // [Complete 상태]
            cv::putText(image, "Logging Complete!", cv::Point(550, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2); // 파란색
        }

        cv::imshow("Measurement with Warmup", image);
        if (cv::waitKey(1) == 27) break;
    }
    
    if (log_file.is_open()) log_file.close();
    cv::destroyAllWindows();
}
*/

void TestManager::camera_calibration(double CURRENT_WAIST_ANGLE_DEG) // camera_calibration
{
    try {
        /* =========================
         * 0. 파일명 설정
         * ========================= */
        int angle_int = (int)std::round(CURRENT_WAIST_ANGLE_DEG);
        std::string suffix = "_" + std::to_string(angle_int) + "deg.csv";
        std::string file_log = "calibration_log" + suffix;

        /* =========================
         * 1. 월드 기준 마커 좌표
         * ========================= */
        struct MarkerPos { double x, y, z; };
        std::map<int, MarkerPos> KNOWN_MARKERS = {
            {0, {-0.583,  0.055, 0.087}},
            {1, {-0.473,  0.055, 0.087}},
            {2, {-0.583, -0.055, 0.087}},
            {3, {-0.473, -0.055, 0.087}}
        };

        /* =========================
         * 2. RealSense 설정
         * ========================= */
        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);

        rs2::pipeline_profile profile = pipe.start(cfg);
        rs2::align align_to_color(RS2_STREAM_COLOR);

        auto stream = profile.get_stream(RS2_STREAM_COLOR)
                          .as<rs2::video_stream_profile>();
        rs2_intrinsics intr = stream.get_intrinsics();

        cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        cameraMatrix.at<double>(0, 0) = intr.fx;
        cameraMatrix.at<double>(1, 1) = intr.fy;
        cameraMatrix.at<double>(0, 2) = intr.ppx;
        cameraMatrix.at<double>(1, 2) = intr.ppy;

        cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
        for (int i = 0; i < 5; i++)
            distCoeffs.at<double>(i) = intr.coeffs[i];

        cv::Ptr<cv::aruco::Dictionary> dictionary =
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        cv::Ptr<cv::aruco::DetectorParameters> parameters =
            cv::aruco::DetectorParameters::create();

        /* =========================
         * 3. 안정화 설정
         * ========================= */
        int stable_frame_count = 0;
        const int REQUIRED_STABLE_FRAMES = 90;

        std::cout << "=== Calibration Start: "
                  << angle_int << " deg ===" << std::endl;

        while (true) {
            rs2::frameset frames = pipe.wait_for_frames();
            frames = align_to_color.process(frames);

            rs2::frame color_frame = frames.get_color_frame();
            rs2::depth_frame depth_frame = frames.get_depth_frame();
            if (!color_frame || !depth_frame) continue;

            cv::Mat image(cv::Size(848, 480), CV_8UC3,
                          (void*)color_frame.get_data(),
                          cv::Mat::AUTO_STEP);

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters);

            if (!ids.empty()) {
                stable_frame_count++;

                if (stable_frame_count >= REQUIRED_STABLE_FRAMES) {

                    /* =========================
                     * 4. Pose 추정 (위치만 사용)
                     * ========================= */
                    std::vector<cv::Vec3d> rvecs, tvecs;
                    cv::aruco::estimatePoseSingleMarkers(
                        corners, 0.08, cameraMatrix, distCoeffs, rvecs, tvecs
                    );

                    /* =========================
                     * 5. 점 행렬 구성
                     * ========================= */
                    cv::Mat P_Cam   = cv::Mat::zeros(4, 4, CV_64F);
                    cv::Mat P_World = cv::Mat::zeros(4, 4, CV_64F);
                    int col = 0;

                    for (size_t i = 0; i < ids.size(); i++) {
                        int id = ids[i];
                        if (!KNOWN_MARKERS.count(id)) continue;
                        if (col >= 4) break;

                        // 카메라 좌표계 점
                        P_Cam.at<double>(0, col) = tvecs[i][0];
                        P_Cam.at<double>(1, col) = tvecs[i][1];
                        P_Cam.at<double>(2, col) = tvecs[i][2];
                        P_Cam.at<double>(3, col) = 1.0;

                        // 월드 좌표계 점
                        auto& m = KNOWN_MARKERS[id];
                        P_World.at<double>(0, col) = m.x;
                        P_World.at<double>(1, col) = m.y;
                        P_World.at<double>(2, col) = m.z;
                        P_World.at<double>(3, col) = 1.0;

                        col++;
                    }

                    saveMatrixToCSV("P_Cam_marker", P_Cam);
                    saveMatrixToCSV("P_World_marker", P_World);

                    if (col < 3) {
                        std::cerr << "Not enough markers for inverse method." << std::endl;
                        break;
                    }

                    /* =========================
                     * 6. World ← Camera (역행렬)
                     * ========================= */
                    cv::Mat T_World_Cam = P_World * P_Cam.inv();

                    /* =========================
                     * 7. 허리 보정 (passive)
                     * ========================= */
                    double rad = CURRENT_WAIST_ANGLE_DEG * CV_PI / 180.0;
                    cv::Mat T_World_Waist = cv::Mat::eye(4, 4, CV_64F);
                    T_World_Waist.at<double>(0,0) =  cos(rad);
                    T_World_Waist.at<double>(0,1) =  sin(rad);
                    T_World_Waist.at<double>(1,0) = -sin(rad);
                    T_World_Waist.at<double>(1,1) =  cos(rad);
                    T_World_Waist.at<double>(2,3) =  -1.129;

                    saveMatrixToCSV("T_World_Waist", T_World_Waist);
                    cv::Mat T_World_Waist_inv = T_World_Waist.inv();
                    cv::Mat T_Final = T_World_Waist_inv * T_World_Cam;

                    /* =========================
                     * 8. 저장 (C만 저장)
                     * ========================= */
                    saveMatrixToCSV(
                        "Matrix_C_WorldFromCamera" + suffix,
                        T_Final
                    );

                    std::cout << ">>> Calibration Complete." << std::endl;
                    break;
                }
            } else {
                stable_frame_count = 0;
            }

            cv::imshow("Auto Calibration", image);
            if (cv::waitKey(1) == 27) break;
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    cv::destroyAllWindows();
    cv::waitKey(1);
}

void TestManager::measure_and_log(double current_waist_angle, const std::string& C_filename) // 행렬 로드하여 마커 월드 좌표 측정
{
    // 측정된 T_World->Waist로 마커의 월드 좌표 측정
    /* =========================
     * 1. C 로드 (World ← Camera)
     * ========================= */
    cv::Mat C = loadMatrixFromCSV(C_filename);
    saveMatrixToCSV("load_matrix", C);
    /* =========================
     * 2. 허리 보정 (passive)
     * ========================= */
    double rad = current_waist_angle * CV_PI / 180.0;
    cv::Mat T_Waist = cv::Mat::eye(4, 4, CV_64F);
    T_Waist.at<double>(0,0) =  cos(rad);
    T_Waist.at<double>(0,1) =  sin(rad);
    T_Waist.at<double>(1,0) = -sin(rad);
    T_Waist.at<double>(1,1) =  cos(rad);
    T_Waist.at<double>(2,3) =  -1.129;

    cv::Mat T_World_Camera = T_Waist * C;

    /* =========================
     * 3. RealSense + ArUco
     * ========================= */
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
    rs2::pipeline_profile profile = pipe.start(cfg);
    rs2::align align_to_color(RS2_STREAM_COLOR);

    auto stream = profile.get_stream(RS2_STREAM_COLOR)
                      .as<rs2::video_stream_profile>();
    rs2_intrinsics intr = stream.get_intrinsics();

    cv::Mat cameraMatrix = cv::Mat::eye(3,3,CV_64F);
    cameraMatrix.at<double>(0,0) = intr.fx;
    cameraMatrix.at<double>(1,1) = intr.fy;
    cameraMatrix.at<double>(0,2) = intr.ppx;
    cameraMatrix.at<double>(1,2) = intr.ppy;

    cv::Mat distCoeffs = cv::Mat::zeros(5,1,CV_64F);
    for (int i = 0; i < 5; i++) distCoeffs.at<double>(i) = intr.coeffs[i];

    auto dictionary = cv::aruco::getPredefinedDictionary(
                        cv::aruco::DICT_4X4_50);
    auto parameters = cv::aruco::DetectorParameters::create();

    /* =========================
    * 4. Loop
    * ========================= */
    while (true) {
        rs2::frameset frames = pipe.wait_for_frames();
        frames = align_to_color.process(frames);

        auto color = frames.get_color_frame();
        if (!color) continue;

        cv::Mat image(cv::Size(848,480), CV_8UC3,
                    (void*)color.get_data(), cv::Mat::AUTO_STEP);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters);

        if (ids.empty()) {
            cv::imshow("World position (tvec)", image);
            if (cv::waitKey(1) == 27) break;
            continue;
        }

        /* =========================
        * ArUco Pose Estimation
        * ========================= */
        std::vector<cv::Vec3d> rvecs, tvecs;

        cv::aruco::estimatePoseSingleMarkers(corners, 0.08, cameraMatrix, distCoeffs, rvecs, tvecs);

        for (size_t i = 0; i < ids.size(); i++) {

            /* =========================
            * 5. Camera 좌표 (tvec 직접 사용)
            * ========================= */
            const cv::Vec3d& t = tvecs[i];

            cv::Mat p_cam = (cv::Mat_<double>(4,1)
                << t[0], t[1], t[2], 1.0);

            /* =========================
            * 6. World 좌표
            * ========================= */
            cv::Mat p_world = T_World_Camera * p_cam;

            double wx = p_world.at<double>(0);
            double wy = p_world.at<double>(1);
            double wz = p_world.at<double>(2);

            cv::putText(image,
                cv::format("ID %d : %.3f %.3f %.3f",
                        ids[i], wx, wy, wz),
                cv::Point(10, 30 + 30*i),
                cv::FONT_HERSHEY_SIMPLEX, 0.6,
                {0,255,0}, 2);

            cv::aruco::drawAxis(
                image, cameraMatrix, distCoeffs,
                rvecs[i], tvecs[i], 0.08 * 0.5
            );
        }

        cv::imshow("World position (tvec)", image);
        if (cv::waitKey(1) == 27) break;
    }
    cv::destroyAllWindows();
}