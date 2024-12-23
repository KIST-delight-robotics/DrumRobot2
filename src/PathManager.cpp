#include "../include/managers/PathManager.hpp" // 적절한 경로로 변경하세요.

// For Qt
// #include "../managers/PathManager.hpp"
PathManager::PathManager(State &stateRef,
                         CanManager &canManagerRef,
                         std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef,
                         Functions &funRef)
    : state(stateRef), canManager(canManagerRef), motors(motorsRef), fun(funRef)
{
}

////////////////////////////////////////////////////////////////////////////////
/*                              Initialization                                */
////////////////////////////////////////////////////////////////////////////////

void PathManager::GetDrumPositoin()
{
    ifstream inputFile("../include/managers/rT.txt");

    if (!inputFile.is_open())
    {
        cerr << "Failed to open the file."
             << "\n";
    }

    // Read data into a 2D vector
    MatrixXd inst_xyz(6, 8);

    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 8; ++j)
        {
            inputFile >> inst_xyz(i, j);
            if (i == 1 || i == 4)
                inst_xyz(i, j) *= 1.0;
        }
    }

    right_drum_position.resize(3, 9);
    left_drum_position.resize(3, 9);

    // Extract the desired elements
    Vector3d right_S;
    Vector3d right_FT;
    Vector3d right_MT;
    Vector3d right_HT;
    Vector3d right_HH;
    Vector3d right_R;
    Vector3d right_RC;
    Vector3d right_LC;

    Vector3d left_S;
    Vector3d left_FT;
    Vector3d left_MT;
    Vector3d left_HT;
    Vector3d left_HH;
    Vector3d left_R;
    Vector3d left_RC;
    Vector3d left_LC;

    for (int i = 0; i < 3; ++i)
    {
        right_S(i) = inst_xyz(i, 0);
        right_FT(i) = inst_xyz(i, 1);
        right_MT(i) = inst_xyz(i, 2);
        right_HT(i) = inst_xyz(i, 3);
        right_HH(i) = inst_xyz(i, 4);
        right_R(i) = inst_xyz(i, 5);
        right_RC(i) = inst_xyz(i, 6);
        right_LC(i) = inst_xyz(i, 7);

        left_S(i) = inst_xyz(i + 3, 0);
        left_FT(i) = inst_xyz(i + 3, 1);
        left_MT(i) = inst_xyz(i + 3, 2);
        left_HT(i) = inst_xyz(i + 3, 3);
        left_HH(i) = inst_xyz(i + 3, 4);
        left_R(i) = inst_xyz(i + 3, 5);
        left_RC(i) = inst_xyz(i + 3, 6);
        left_LC(i) = inst_xyz(i + 3, 7);
    }

    right_drum_position << right_RC, right_R, right_S, right_HH, right_HH, right_FT, right_MT, right_LC, right_HT;
    left_drum_position << left_RC, left_R, left_S, left_HH, left_HH, left_FT, left_MT, left_LC, left_HT;
}

void PathManager::SetReadyAngle()
{
    VectorXd inst_p(18);

    default_right.resize(9);
    default_left.resize(9);
    default_right << 0, 0, 1, 0, 0, 0, 0, 0, 0;
    default_left << 0, 0, 1, 0, 0, 0, 0, 0, 0;

    inst_p << default_right,
        default_left;

    MatrixXd combined(6, 18);
    combined << right_drum_position, MatrixXd::Zero(3, 9), MatrixXd::Zero(3, 9), left_drum_position;
    MatrixXd p = combined * inst_p;

    VectorXd pR = VectorXd::Map(p.data(), 3, 1);
    VectorXd pL = VectorXd::Map(p.data() + 3, 3, 1);
    VectorXd qk = ikfun_final(pR, pL);

    for (int i = 0; i < qk.size(); ++i)
    {
        readyArr[i] = qk(i);
    }

    HitParameter param;
    readyArr[4] += param.elbowStayAngle;
    readyArr[6] += param.elbowStayAngle;
    readyArr[7] = param.wristStayAngle;
    readyArr[8] = param.wristStayAngle;
}

void PathManager::InitVal()
{
    threshold = 2.4;
    total_time = 0.0;
    totalTime = 0.0;
    detect_time_R = 0;
    detect_time_L = 0;
    current_time = 0;
    moving_start_R = 0;
    moving_start_L = 0;
    line_n = 0; 
    round_sum = 0;
    vector<string> prev_col = { "0","0","1","1","0","0","0","0" };  // default 악기 위치로 맞추기
    state___ = MatrixXd::Zero(2, 3);
}

////////////////////////////////////////////////////////////////////////////////    
/*                                  Play                                      */
////////////////////////////////////////////////////////////////////////////////

void PathManager::generateTrajectory()
{   
    // position
    VectorXd Pi(6), Pf(6);
    VectorXd Pi_R(3);
    VectorXd Pi_L(3);
    VectorXd Pf_R(3);
    VectorXd Pf_L(3);

    float n, s_R, s_L;
    float delta_t_measure_R;
    float delta_t_measure_L;
    float dt = canManager.deltaT;

    // waist
    float q0_t1 = 0.0, q0_t2 = 0.0;

    // state
    getInstrument();

    // time
    delta_t_measure_R = t_f_R - t_i_R;
    delta_t_measure_L = t_f_L - t_i_L;

    // position
    Pi = getTargetPosition(inst_i);
    Pf = getTargetPosition(inst_f);

    Pi_R << Pi(0), Pi(1), Pi(2);
    Pi_L << Pi(3), Pi(4), Pi(5);
    Pf_R << Pf(0), Pf(1), Pf(2);
    Pf_L << Pf(3), Pf(4), Pf(5);

    std::cout << "\nPi_R\n" << Pi_R
    << "\nPi_L\n" << Pi_L
    << "\nPf_R\n" << Pf_R
    << "\nPf_L\n" << Pf_L << std::endl;

    // trajectory
    n = (t2 - t1) / dt;


    for (int i = 0; i < n; i++)
    {
        Position Pt;
        float t_R = dt * i + t1 - t_i_R;
        float t_L = dt * i + t1 - t_i_L;
        
        s_R = timeScaling(0.0f, delta_t_measure_R, t_R);
        s_L = timeScaling(0.0f, delta_t_measure_L, t_L);

        Pt.pR = makePath(Pi_R, Pf_R, s_R);
        Pt.pL = makePath(Pi_L, Pf_L, s_L);
        
        P_buffer.push(Pt);

        std::string fileName;
        fileName = "Trajectory_R";
        fun.appendToCSV_DATA(fileName, Pt.pR[0], Pt.pR[1], Pt.pR[2]);
        fileName = "Trajectory_L";
        fun.appendToCSV_DATA(fileName, Pt.pL[0], Pt.pL[1], Pt.pL[2]);
        fileName = "S_R";
        fun.appendToCSV_DATA(fileName, t_R, s_R, delta_t_measure_R);
        fileName = "S_L";
        fun.appendToCSV_DATA(fileName, t_L, s_L, delta_t_measure_L);

        // waist
        if (i == 0)
        {
            VectorXd q_t1 = ikfun_final(Pt.pR, Pt.pL);
            q0_t1 = q_t1(0);
        }
        else if (i + 1 >= n)
        {
            VectorXd q_t2 = ikfun_final(Pt.pR, Pt.pL);
            q0_t2 = q_t2(0);
        }
    }

    // waist, wrist, elbow & brake
    for (int i = 0; i < n; i++)
    {
        AddAngle qt;
        Brake brake_t;
        float t = dt * i;
        
        // waist
        MatrixXd A;
        MatrixXd b;
        MatrixXd A_1;
        MatrixXd sol;

        float t21 = t2 - t1;

        A.resize(4,4);
        b.resize(4,1);

        A << 1, 0, 0, 0,
            1, t21, t21*t21, t21*t21*t21,
            0, 1, 0, 0,
            0, 1, 2*t21, 3*t21*t21;

        b << q0_t1, q0_t2, 0, 0;

        A_1 = A.inverse();
        sol = A_1 * b;

        qt.q0 = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t + sol(3,0) * t * t * t;
        
        // wrist & elbow
        HitParameter param;
        qt.add_qR = makeHitTrajetory(t1, t2, t, hit_state_R, param);
        qt.add_qL = makeHitTrajetory(t1, t2, t, hit_state_L, param);

        // brake
        for (int j = 0; j < 8; j++)
        {
            brake_t.state[j] = false;
        }

        q_buffer.push(qt);
        brake_buffer.push(brake_t);
    }
}

void PathManager::generateTrajectory___()
{
    // position
    VectorXd Pi(6), Pf(6);
    VectorXd Pi_R(3);
    VectorXd Pi_L(3);
    VectorXd Pf_R(3);
    VectorXd Pf_L(3);

    float n, s_R, s_L;
    float delta_t_measure_R;
    float delta_t_measure_L;
    float dt = canManager.deltaT;

    // waist
    float q0_t1 = 0.0, q0_t2 = 0.0;

    // parse
    parseMeasure___(measureMatrix);

    // time
    delta_t_measure_R = t_f_R - t_i_R;
    delta_t_measure_L = t_f_L - t_i_L;

    // position
    Pi = getTargetPosition(inst_i);
    Pf = getTargetPosition(inst_f);

    Pi_R << Pi(0), Pi(1), Pi(2);
    Pi_L << Pi(3), Pi(4), Pi(5);
    Pf_R << Pf(0), Pf(1), Pf(2);
    Pf_L << Pf(3), Pf(4), Pf(5);

    // std::cout << "\nPi_R\n" << Pi_R.transpose()
    // << "\nPi_L\n" << Pi_L.transpose()
    // << "\nPf_R\n" << Pf_R.transpose()
    // << "\nPf_L\n" << Pf_L.transpose() << std::endl;

    // trajectory
    n = (t2 - t1) / dt;

    round_sum += (int)(n * 1000) % 1000;
    if (round_sum >= 1000)
    {
        round_sum -= 1000;
        n++;
    }
    n = (int)n;

    // cout << "\n---------------------" << n << '\n';

    for (int i = 0; i < n; i++)
    {
        Position Pt;
        float t_R = dt * i + t1 - t_i_R;
        float t_L = dt * i + t1 - t_i_L;
        
        s_R = timeScaling(0.0f, delta_t_measure_R, t_R);
        s_L = timeScaling(0.0f, delta_t_measure_L, t_L);

        Pt.pR = makePath(Pi_R, Pf_R, s_R);
        Pt.pL = makePath(Pi_L, Pf_L, s_L);
        
        P_buffer.push(Pt);

        std::string fileName;
        fileName = "Trajectory_R";
        fun.appendToCSV_DATA(fileName, Pt.pR[0], Pt.pR[1], Pt.pR[2]);
        fileName = "Trajectory_L";
        fun.appendToCSV_DATA(fileName, Pt.pL[0], Pt.pL[1], Pt.pL[2]);
        fileName = "S_R";
        fun.appendToCSV_DATA(fileName, t_R, s_R, delta_t_measure_R);
        fileName = "S_L";
        fun.appendToCSV_DATA(fileName, t_L, s_L, delta_t_measure_L);

        // waist
        if (i == 0)
        {
            VectorXd q_t1 = ikfun_final(Pt.pR, Pt.pL);
            q0_t1 = q_t1(0);
        }
        else if (i + 1 >= n)
        {
            VectorXd q_t2 = ikfun_final(Pt.pR, Pt.pL);
            q0_t2 = q_t2(0);
        }
    }

    // waist, wrist & elbow
    for (int i = 0; i < n; i++)
    {
        AddAngle qt;
        float t = dt * i;
        
        // waist
        MatrixXd A;
        MatrixXd b;
        MatrixXd A_1;
        MatrixXd sol;

        float t21 = t2 - t1;

        A.resize(4,4);
        b.resize(4,1);

        A << 1, 0, 0, 0,
            1, t21, t21*t21, t21*t21*t21,
            0, 1, 0, 0,
            0, 1, 2*t21, 3*t21*t21;

        b << q0_t1, q0_t2, 0, 0;

        A_1 = A.inverse();
        sol = A_1 * b;

        qt.q0 = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t + sol(3,0) * t * t * t;
        
        // wrist & elbow
        HitParameter param;
        qt.add_qR = makeHitTrajetory(t1, t2, t, hit_state_R, param);
        qt.add_qL = makeHitTrajetory(t1, t2, t, hit_state_L, param);

        q_buffer.push(qt);
    }

    // brake
    for (int i = 0; i < n; i++)
    {
        Brake brake_t;
        

        for (int j = 0; j < 8; j++)
        {
            brake_t.state[j] = false;
        }

        brake_buffer.push(brake_t);
    }
}

void PathManager::solveIK()
{
    VectorXd q;
    Position nextP;
    AddAngle nextQ;

    nextP = P_buffer.front();
    P_buffer.pop();

    nextQ = q_buffer.front();
    q_buffer.pop();

    q = ikFixedWaist(nextP.pR, nextP.pL, nextQ.q0);

    q(4) += nextQ.add_qR(1);
    q(6) += nextQ.add_qL(1);

    q(7) = nextQ.add_qR(0);
    q(8) = nextQ.add_qL(0);

    pushConmmandBuffer(q);

    // 데이터 기록
    for (int m = 0; m < 9; m++)
    {
        std::string fileName = "solveIK_q" + to_string(m);
        fun.appendToCSV_DATA(fileName, m, q(m), 0);
    }
}

bool PathManager::readMeasure(ifstream& inputFile, bool &BPMFlag, double &timeSum)
{
    string line;

    while(getline(inputFile, line))
    {
        istringstream iss(line);
        string item;
        int cnt = 0;

        vector<string> columns;
        while (getline(iss, item, '\t'))
        {
            if(cnt >= 8) break;
            item = trimWhitespace(item);
            columns.push_back(item);
            cnt++;

        }

        if (!BPMFlag)
        { // 첫번째 행엔 bpm에 대한 정보
            cout << "music";
            bpm = stod(columns[0].substr(4));
            cout << " bpm = " << bpm << "\n";
            BPMFlag = 1;

            pre_inst_R << default_right;
            pre_inst_L << default_left;
        }
        else
        {
            

            // total_time을 columns의 맨 끝에 추가
            columns.emplace_back(to_string(total_time)); // total_time을 columns 끝에 추가

            // timeSum 누적
            timeSum += stod(columns[1]);

            // total_time 갱신
            total_time += stod(columns[1]);

            // 큐에 저장
            Q.push(columns);

            // timeSum이 threshold를 넘으면 출력
            if(timeSum >= threshold)
            {
                queue<vector<string>> tempQ = Q; // 큐 복사본 사용
                while (!tempQ.empty())
                {
                    vector<string> current = tempQ.front();
                    tempQ.pop();

                    // 큐의 각 요소 출력 (탭으로 구분)
                    for (size_t i = 0; i < current.size(); ++i)
                    {
                        cout << current[i];
                        if (i != current.size() - 1) // 마지막 요소가 아니라면 탭 추가
                            cout << '\t';
                    }
                    cout << '\n';
                }
                cout << '\n';

                return true;
            }
        }
    }
    // sleep(5);
    return false;
}

void PathManager::parseMeasure(double &timeSum)
{
    map<string, int> instrument_mapping = {
    {"1", 2}, {"2", 5}, {"3", 6}, {"4", 8}, {"5", 3}, {"6", 1}, {"7", 0}, {"8", 7}, {"11", 2}, {"51", 2}, {"61", 2}, {"71", 2}, {"81", 2}, {"91", 2}};
    // S        FT          MT       HT        HH        R         RC        LC         S          S          S          S          S           S
    
    // 지금 들어온 Q 맨 앞에 값이 현재 시간임
    vector<string> curLine = Q.front(); 
    current_time = stod(curLine[8]);  
    
    // 이전 위치에 대한 업데이트
    if(prev_col[2] != "0" || prev_col[3] != "0")
    {
        VectorXd inst_R_prev = VectorXd::Zero(9), inst_L_prev = VectorXd::Zero(9);
        if (prev_col[2] != "0")
        {
            inst_R_prev(instrument_mapping[prev_col[2]]) = 1.0; // 해당 악기 상태 활성화
        }

        if (prev_col[3] != "0")
        {
            inst_L_prev(instrument_mapping[prev_col[3]]) = 1.0; // 해당 악기 상태 활성화
        }

        inst_i << inst_R_prev, inst_L_prev;
    }

    // 들어왔을 때 현재 시간이 detect_timeR이나 detect_timeL보다 크거나 같으면 움직이기 시작하는 시간을 현재 시간으로 설정
    if (std::round(detect_time_R * 100000) / 100000 <= std::round(current_time * 100000) / 100000)
    {
        moving_start_R = current_time;
    }

    if (std::round(detect_time_L * 100000) / 100000 <= std::round(current_time * 100000) / 100000)
    {
        moving_start_L = current_time;
    }

    float make_time = 0;
    //threshold/2
    VectorXd inst_R = VectorXd::Zero(9), inst_L = VectorXd::Zero(9);
    // VectorXd inst_next = VectorXd::Zero(18);

    // 큐를 전부 순회
    for (size_t i = 0; i < Q.size(); ++i)
    {
        curLine = Q.front();
        Q.pop();
        Q.push(curLine); // 현재 데이터를 다시 큐 끝에 삽입

        // 오른손 타격 감지
        if (curLine[2] != "0" && !(inst_R.array() != 0).any())
        {
            inst_R(instrument_mapping[curLine[2]]) = 1.0; // 해당 악기 상태 활성화
            detect_time_R = stod(curLine[1]) + stod(curLine[8]); // 오른손 타격 시간 갱신
        }

        // 왼손 타격 감지
        if (curLine[3] != "0" && !(inst_L.array() != 0).any())
        {
            inst_L(instrument_mapping[curLine[3]]) = 1.0; // 해당 악기 상태 활성화
            detect_time_L = stod(curLine[1]) + stod(curLine[8]); // 왼손 타격 시간 갱신
        }

        // 양손 모두 타격 감지
        if ((inst_R.array() != 0).any() && (inst_L.array() != 0).any())
        {
            continue; // 둘 다 타격이 감지되면 다음 루프로 넘어감
        }
    }

    //결론적으로 움직일 위치와 현재 오른손 시간 현재 왼손 시간 타격할 오른손 시간 타격할 왼손 시간 움직이기 시작한 왼손 시간 움직이긴 시작한 오른손 시간 정보를 다음 함수에 넘겨주는 구조가 될 예정
    inst_f << inst_R, inst_L;

    prev_col = Q.front();
    timeSum -= stod(prev_col[1]);

    make_time = current_time + stod(prev_col[1]);

    t_i_R = moving_start_R;
    t_i_L = moving_start_L;
    t_f_R = detect_time_R;
    t_f_L = detect_time_L;
    t1 = current_time;
    t2 = make_time;

    t_i_R = t_i_R*100/bpm;
    t_i_L = t_i_L*100/bpm;
    t_f_R = t_f_R*100/bpm;
    t_f_L = t_f_L*100/bpm;
    t1 = t1*100/bpm;
    t2 = t2*100/bpm;


    Q.pop();

    hit_state_R << stod(prev_col[2]), stod(Q.front()[2]);
    hit_state_L << stod(prev_col[3]), stod(Q.front()[3]);
    
    // cout << "Right : " << hit_state_R.transpose() << "\tLeft : " << hit_state_L.transpose() << "\n";

    line_n++;
    std::cout << "-----------------------------------------------------현재라인 : " << line_n << "----------------------------------------------------" << std::endl;
    std::cout << "// 오른손 타격할 악기 --> \t" << inst_R.transpose() << "    움직임 시작 시간 --> " << moving_start_R << " \t현재시간 --> "  << current_time <<  " \t궤적시간 --> "  << make_time << " \t타격감지시간 --> " << detect_time_R << " \t //" << std::endl;
    std::cout << "// 왼손 타격할 악기 --> \t" << inst_L.transpose() << "    움직임 시작 시간 --> " << moving_start_L << " \t현재시간 --> "  << current_time <<  " \t궤적시간 --> "  << make_time << " \t타격감지시간 --> " << detect_time_L << " \t //" <<  std::endl;
    std::cout << "// 이전 악기 --> \t" << inst_i.transpose() << "    다음 악기 --> " << inst_f.transpose() << " \t\t //" << std::endl;
    std::cout << "--------------------------------------------------------------------------------------------------------------------" << std::endl << std::endl;

}

////////////////////////////////////////////////////////////////////////////////
/*                                AddStance                                   */
////////////////////////////////////////////////////////////////////////////////

void PathManager::GetArr(vector<float> &arr)
{
    const float acc_max = 100.0;    // rad/s^2
    //vector<float> Qi;
    //vector<float> Vmax;
    VectorXd Q1 = VectorXd::Zero(9);
    VectorXd Q2 = VectorXd::Zero(9);
    VectorXd Qi = VectorXd::Zero(9);
    VectorXd Vmax = VectorXd::Zero(9);

    float dt = canManager.deltaT;   // 0.005
    float t = 2.0;                  // 3초동안 실행
    float extra_time = 1.0;         // 추가 시간 1초
    int n = (int)(t / dt);   
    int n_p = (int)(extra_time / dt); 

    getMotorPos(); 

    for (int i = 0; i < 9; i++)
    {
        Q1(i) = c_MotorAngle[i];
        Q2(i) = arr[i];
    }

    Vmax = calVmax(Q1, Q2, acc_max, t);

    for (int k = 0; k < 9; k++)
    {
        cout << "Q1[" << k << "] : " << Q1[k]*180.0/M_PI <<  " [deg] -> Q2[" << k << "] : " << Q2[k]*180.0/M_PI << " [deg]" << endl;
        cout << "Vmax[" << k << "] : " << Vmax(k) << "[rad/s]\n\n";
    }

    for (int k = 1; k <= n + n_p; ++k)
    {
        // Make Array
        Qi = makeProfile(Q1, Q2, Vmax, acc_max, t*k/n, t);

        // Send to Buffer
        for (auto &entry : motors)
        {
            if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
            {
                TMotorData newData;
                newData.position = Qi[motor_mapping[entry.first]];
                newData.spd = 0;
                newData.acl = 0;
                newData.isBrake = false;
                tMotor->commandBuffer.push(newData);
            }
            else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
            {
                MaxonData newData;
                newData.position = Qi[motor_mapping[entry.first]];
                newData.WristState = 0.5;
                maxonMotor->commandBuffer.push(newData);
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                         Read & Parse Measure                               */
////////////////////////////////////////////////////////////////////////////////

string PathManager::trimWhitespace(const std::string &str)
{
    size_t first = str.find_first_not_of(" \t");
    if (std::string::npos == first)
    {
        return str;
    }
    size_t last = str.find_last_not_of(" \t");
    return str.substr(first, (last - first + 1));
}

////////////////////////////////////////////////////////////////////////////////
/*                            Make Trajectory                                 */
////////////////////////////////////////////////////////////////////////////////

void PathManager::getInstrument()
{
    float norm_R_i = inst_i.block(0, 0, 9, 1).norm();
    float norm_L_i = inst_i.block(9, 0, 9, 1).norm();
    float norm_R_f = inst_f.block(0, 0, 9, 1).norm();
    float norm_L_f = inst_f.block(9, 0, 9, 1).norm();

    if (norm_R_f == 0 && norm_R_i == 0)
    {
        inst_i.block(0, 0, 9, 1) = pre_inst_R;
        inst_f.block(0, 0, 9, 1) = pre_inst_R;
        t_i_R = t1;
        t_f_R = t2;
    }
    else if (norm_R_f == 0 && norm_R_i == 1)
    {
        inst_f.block(0, 0, 9, 1) = inst_i.block(0, 0, 9, 1);
        pre_inst_R = inst_i.block(0, 0, 9, 1);
        t_i_R = t1;
        t_f_R = t2;
    }
    else if (norm_R_f == 1 && norm_R_i == 0)
    {
        inst_i.block(0, 0, 9, 1) = pre_inst_R;
        pre_inst_R = inst_i.block(0, 0, 9, 1);
    }
    else if (norm_R_f == 1 && norm_R_i == 1)
    {
        pre_inst_R = inst_i.block(0, 0, 9, 1);
    }

    if (norm_L_f == 0 && norm_L_i == 0)
    {
        inst_i.block(9, 0, 9, 1) = pre_inst_L;
        inst_f.block(9, 0, 9, 1) = pre_inst_L;
        t_i_L = t1;
        t_f_L = t2;
    }
    else if (norm_L_f == 0 && norm_L_i == 1)
    {
        inst_f.block(9, 0, 9, 1) = inst_i.block(9, 0, 9, 1);
        pre_inst_L = inst_i.block(9, 0, 9, 1);
        t_i_L = t1;
        t_f_L = t2;
    }
    else if (norm_L_f == 1 && norm_L_i == 0)
    {
        inst_i.block(9, 0, 9, 1) = pre_inst_L;
        pre_inst_L = inst_i.block(9, 0, 9, 1);
    }
    else if (norm_L_f == 1 && norm_L_i == 1)
    {
        pre_inst_L = inst_i.block(9, 0, 9, 1);
    }
}

VectorXd PathManager::getTargetPosition(VectorXd &inst_vector)
{
    VectorXd inst_right = inst_vector.segment(0, 9);
    VectorXd inst_left = inst_vector.segment(9, 9);

    if (inst_right.sum() == 0)
    {
        std::cout << "Right Instrument Vector Error!!\n";
    }

    if (inst_left.sum() == 0)
    {
        std::cout << "Left Instrument Vector Error!!\n";
    }

    VectorXd inst_p(18);
    inst_p << inst_right,
        inst_left;

    MatrixXd combined(6, 18);
    combined << right_drum_position, MatrixXd::Zero(3, 9), MatrixXd::Zero(3, 9), left_drum_position;
    MatrixXd p = combined * inst_p;

    return p;
}

float PathManager::timeScaling(float ti, float tf, float t)
{
    float s;

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd sol;

    A.resize(4,4);
    b.resize(4,1);

    A << 1, ti, ti*ti, ti*ti*ti,
    1, tf, tf*tf, tf*tf*tf,
    0, 1, 2*ti, 3*ti*ti,
    0, 1, 2*tf, 3*tf*tf;

    b << 0, 1, 0, 0;

    A_1 = A.inverse();
    sol = A_1 * b;

    s = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t + sol(3,0) * t * t * t;

    return s;
}

VectorXd PathManager::makePath(VectorXd Pi, VectorXd Pf, float s)
{
    float degree = 2.0;

    float xi = Pi(0), xf = Pf(0);
    float yi = Pi(1), yf = Pf(1);
    float zi = Pi(2), zf = Pf(2);

    VectorXd Ps;
    Ps.resize(3);

    if (Pi == Pf)
    {
        Ps(0) = xi;
        Ps(1) = yi;
        Ps(2) = zi;
    }
    else
    {
        Ps(0) = xi + s * (xf - xi);
        Ps(1) = yi + s * (yf - yi);

        if (zi > zf)
        {
            float a = zf - zi;
            float b = zi;

            Ps(2) = a*std::pow(s, degree) + b;
        }
        else
        {
            float a = (zi - zf) * std::pow(-1, degree);
            float b = zf;

            Ps(2) = a*std::pow(s-1, degree) + b;
        }
    }

    return Ps;
}

VectorXd PathManager::makeHitTrajetory(float t1, float t2, float t, VectorXd hitState, HitParameter param)
{
    VectorXd addAngle;
    int state;

    if (hitState(0) == 0 && hitState(1) == 0)
    {
        // Stay
        state = 0;
    }
    else if (hitState(1) == 0)
    {
        // Contact - Stay
        state = 1;
    }
    else if (hitState(0) == 0)
    {
        // Stay - Lift - Hit
        state = 2;
    }
    else
    {
        // Contact - Lift - Hit
        state = 3;
    }

    addAngle.resize(2);    // wrist, elbow
    addAngle(0) = makeWristAngle(t1, t2, t, state, param);
    addAngle(1) = makeElbowAngle(t1, t2, t, state, param);
   
    return addAngle;
}

float PathManager::makeWristAngle(float t1, float t2, float t, int state, HitParameter param)
{
    float wrist_q = 0.0;

    float t_contact = 0.2 * (t2 - t1);
    float t_lift = 0.6 * (t2 - t1);
    float t_stay = 0.5 * (t2 - t1);
    float t_hit = t2 - t1;

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd sol;

    if (state == 0)
    {
        // Stay
        wrist_q = param.wristStayAngle;
    }
    else if (state == 1)
    {
        // Contact - Stay
        if (t < t_contact)
        {
            A.resize(3,3);
            b.resize(3,1);

            A << 1, 0, 0,
                1, t_contact, t_contact*t_contact,
                0, 1, 2*t_contact;

            b << 0, param.wristContactAngle, 0;

            A_1 = A.inverse();
            sol = A_1 * b;

            wrist_q = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t;
        }
        else if (t < t_stay)
        {
            A.resize(4,4);
            b.resize(4,1);

            A << 1, t_contact, t_contact*t_contact, t_contact*t_contact*t_contact,
                1, t_stay, t_stay*t_stay, t_stay*t_stay*t_stay,
                0, 1, 2*t_contact, 3*t_contact*t_contact,
                0, 1, 2*t_stay, 3*t_stay*t_stay;

            b << param.wristContactAngle, param.wristStayAngle, 0, 0;

            A_1 = A.inverse();
            sol = A_1 * b;

            wrist_q = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t + sol(3,0) * t * t * t;
        }
        else
        {
            wrist_q = param.wristStayAngle;
        }
    }
    else if (state == 2)
    {
        // Stay - Lift - Hit
        if (t < t_lift)
        {
            A.resize(4,4);
            b.resize(4,1);

            A << 1, 0, 0, 0,
                1, t_lift, t_lift*t_lift, t_lift*t_lift*t_lift,
                0, 1, 0, 0,
                0, 1, 2*t_lift, 3*t_lift*t_lift;

            b << param.wristStayAngle, param.wristLiftAngle, 0, 0;

            A_1 = A.inverse();
            sol = A_1 * b;

            wrist_q = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t + sol(3,0) * t * t * t;
        }
        else if (t < t_hit)
        {
            A.resize(3,3);
            b.resize(3,1);

            A << 1, t_lift, t_lift*t_lift,
                1, t_hit, t_hit*t_hit,
                0, 1, 2*t_lift;

            b << param.wristLiftAngle, 0, 0;

            A_1 = A.inverse();
            sol = A_1 * b;

            wrist_q = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t;
        }
        else
        {
            wrist_q = 0.0;
        }
    }
    else if (state == 3)
    {
        // Contact - Lift - Hit
        if (t < t_contact)
        {
            A.resize(3,3);
            b.resize(3,1);

            A << 1, 0, 0,
                1, t_contact, t_contact*t_contact,
                0, 1, 2*t_contact;

            b << 0, param.wristContactAngle, 0;

            A_1 = A.inverse();
            sol = A_1 * b;

            wrist_q = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t;
        }
        else if (t < t_lift)
        {
            A.resize(4,4);
            b.resize(4,1);

            A << 1, t_contact, t_contact*t_contact, t_contact*t_contact*t_contact,
                1, t_lift, t_lift*t_lift, t_lift*t_lift*t_lift,
                0, 1, 2*t_contact, 3*t_contact*t_contact,
                0, 1, 2*t_lift, 3*t_lift*t_lift;

            b << param.wristContactAngle, param.wristLiftAngle, 0, 0;

            A_1 = A.inverse();
            sol = A_1 * b;

            wrist_q = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t + sol(3,0) * t * t * t;
        }
        else if (t < t_hit)
        {
            A.resize(3,3);
            b.resize(3,1);

            A << 1, t_lift, t_lift*t_lift,
                1, t_hit, t_hit*t_hit,
                0, 1, 2*t_lift;

            b << param.wristLiftAngle, 0, 0;

            A_1 = A.inverse();
            sol = A_1 * b;

            wrist_q = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t;
        }
        else
        {
            wrist_q = 0.0;
        }
    }

    return wrist_q;
}

float PathManager::makeElbowAngle(float t1, float t2, float t, int state, HitParameter param)
{
    float elbow_q = 0.0;

    float t_lift = 0.5 * (t2 - t1);
    float t_stay = 0.5 * (t2 - t1);
    float t_hit = t2 - t1;

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd sol;

    if (state == 0)
    {
        // Stay
        elbow_q = param.elbowStayAngle;
    }
    else if (state == 1)
    {
        // Contact - Stay
        if (t < t_stay)
        {
            A.resize(4,4);
            b.resize(4,1);

            A << 1, 0, 0, 0,
                1, t_stay, t_stay*t_stay, t_stay*t_stay*t_stay,
                0, 1, 0, 0,
                0, 1, 2*t_stay, 3*t_stay*t_stay;

            b << 0, param.elbowStayAngle, 0, 0;

            A_1 = A.inverse();
            sol = A_1 * b;

            elbow_q = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t + sol(3,0) * t * t * t;
        }
        else
        {
            elbow_q = param.elbowStayAngle;
        }
    }
    else if (state == 2)
    {
        // Stay - Lift - Hit
        if (t < t_lift)
        {
            A.resize(4,4);
            b.resize(4,1);

            A << 1, 0, 0, 0,
                1, t_lift, t_lift*t_lift, t_lift*t_lift*t_lift,
                0, 1, 0, 0,
                0, 1, 2*t_lift, 3*t_lift*t_lift;

            b << param.elbowStayAngle, param.elbowLiftAngle, 0, 0;

            A_1 = A.inverse();
            sol = A_1 * b;

            elbow_q = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t + sol(3,0) * t * t * t;
        }
        else if (t < t_hit)
        {
            A.resize(4,4);
            b.resize(4,1);

            A << 1, t_lift, t_lift*t_lift, t_lift*t_lift*t_lift,
                1, t_hit, t_hit*t_hit, t_hit*t_hit*t_hit,
                0, 1, 2*t_lift, 3*t_lift*t_lift,
                0, 1, 2*t_hit, 3*t_hit*t_hit;

            b << param.elbowLiftAngle, 0, 0, 0;

            A_1 = A.inverse();
            sol = A_1 * b;

            elbow_q = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t + sol(3,0) * t * t * t;
        }
        else
        {
            elbow_q = 0.0;
        }
    }
    else if (state == 3)
    {
        // Contact - Lift - Hit
        if (t < t_lift)
        {
            A.resize(4,4);
            b.resize(4,1);

            A << 1, 0, 0, 0,
                1, t_lift, t_lift*t_lift, t_lift*t_lift*t_lift,
                0, 1, 0, 0,
                0, 1, 2*t_lift, 3*t_lift*t_lift;

            b << 0, param.elbowLiftAngle, 0, 0;

            A_1 = A.inverse();
            sol = A_1 * b;

            elbow_q = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t + sol(3,0) * t * t * t;
        }
        else if (t < t_hit)
        {
            A.resize(4,4);
            b.resize(4,1);

            A << 1, t_lift, t_lift*t_lift, t_lift*t_lift*t_lift,
                1, t_hit, t_hit*t_hit, t_hit*t_hit*t_hit,
                0, 1, 2*t_lift, 3*t_lift*t_lift,
                0, 1, 2*t_hit, 3*t_hit*t_hit;

            b << param.elbowLiftAngle, 0, 0, 0;

            A_1 = A.inverse();
            sol = A_1 * b;

            elbow_q = sol(0,0) + sol(1,0) * t + sol(2,0) * t * t + sol(3,0) * t * t * t;
        }
        else
        {
            elbow_q = 0.0;
        }
    }

    return elbow_q;
}

////////////////////////////////////////////////////////////////////////////////
/*                                Solve IK                                    */
////////////////////////////////////////////////////////////////////////////////

VectorXd PathManager::ikFixedWaist(VectorXd &pR, VectorXd &pL, float theta0)
{
    VectorXd Qf;
    PartLength part_length;

    float XR = pR(0), YR = pR(1), ZR = pR(2);
    float XL = pL(0), YL = pL(1), ZL = pL(2);
    float R1 = part_length.upperArm;
    float R2 = part_length.lowerArm + part_length.stick;
    float L1 = part_length.upperArm;
    float L2 = part_length.lowerArm + part_length.stick;
    float s = part_length.waist;
    float z0 = part_length.height;

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
    float r2 = (YR - shoulderYR)*(YR - shoulderYR) + (XR - shoulderXR)*(XR - shoulderXR); // r^2

    float x = zeta*zeta + r2 - R1*R1 - R2*R2;
    float y = sqrt(4.0*R1*R1*R2*R2 - x*x);

    float theta4 = atan2(y,x);

    if (theta4 < 0 || theta4 > 140.0 * M_PI / 180.0) // the4 범위 : 0deg ~ 120deg
    {
        std::cout << "IKFUN (q4) is not solved!!\n";
        state.main = Main::Error;
    }

    float theta34 = atan2(sqrt(r2), zeta);
    float theta3 = theta34 - atan2(R2*sin(theta4), R1 + R2*cos(theta4));

    if (theta3 < -45.0 * M_PI / 180.0 || theta3 > 90.0 * M_PI / 180.0) // the3 범위 : -45deg ~ 90deg
    {
        std::cout << "IKFUN (q3) is not solved!!\n";
        std::cout << "q3 : " << theta3 * 180.0 / M_PI << "\n";
        std::cout << "pR : \n" << pR << "\n";
        std::cout << "pL : \n" << pL << "\n";
        state.main = Main::Error;
    }

    zeta = z0 - ZL;
    r2 = (YL - shoulderYL)*(YL - shoulderYL) + (XL - shoulderXL)*(XL - shoulderXL); // r^2

    x = zeta*zeta + r2 - L1*L1 - L2*L2;
    y = sqrt(4.0*L1*L1*L2*L2 - x*x);

    float theta6 = atan2(y,x);

    if (theta6 < 0 || theta6 > 140.0 * M_PI / 180.0) // the6 범위 : 0deg ~ 120deg
    {
        std::cout << "IKFUN (q6) is not solved!!\n";
        state.main = Main::Error;
    }

    float theta56 = atan2(sqrt(r2), zeta);
    float theta5 = theta56 - atan2(L2*sin(theta6), L1 + L2*cos(theta6));

    if (theta5 < -45.0 * M_PI / 180.0 || theta5 > 90.0 * M_PI / 180.0) // the5 범위 : -45deg ~ 90deg
    {
        std::cout << "IKFUN (q5) is not solved!!\n";
        state.main = Main::Error;
    }

    Qf.resize(9);
    Qf << theta0, theta1, theta2, theta3, theta4, theta5, theta6, 0.0, 0.0;

    return Qf;
}

void PathManager::pushConmmandBuffer(VectorXd &Qi)
{
    for (auto &entry : motors)
    {
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        {
            TMotorData newData;
            newData.position = Qi[motor_mapping[entry.first]];
            newData.spd = 0;
            newData.acl = 0;
            tMotor->commandBuffer.push(newData);
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
        {
            MaxonData newData;
            newData.position = Qi[motor_mapping[entry.first]];
            newData.WristState = 0; // 토크 제어 시 WristState 사용
            maxonMotor->commandBuffer.push(newData);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                           AddStance FUNCTION                               */
////////////////////////////////////////////////////////////////////////////////

VectorXd PathManager::calVmax(VectorXd &q1, VectorXd &q2, float acc, float t2)
{
    VectorXd Vmax = VectorXd::Zero(9);

    for (int i = 0; i < 9; i++)
    {
        float val;
        float S = q2(i) - q1(i);

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

            // 2차 방정식 해
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

        Vmax(i) = val;
    }

    return Vmax;
}

VectorXd PathManager::makeProfile(VectorXd &q1, VectorXd &q2, VectorXd &Vmax, float acc, float t, float t2)
{
    VectorXd Qi = VectorXd::Zero(9);

    for(int i = 0; i < 9; i++)
    {
        float val, S;
        int sign;

        S = q2(i) - q1(i);
        
        // 부호 확인, 이동거리 양수로 변경
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
            val = q1(i);
        }
        else if (Vmax(i) < 0)
        {
            // Vmax 값을 구하지 못했을 때 삼각형 프로파일 생성
            float acc_tri = 4 * S / t2 / t2;

            if (t < t2/2)
            {
                val = q1(i) + sign * 0.5 * acc_tri * t * t;
            }
            else if (t < t2)
            {
                val = q2(i) - sign * 0.5 * acc_tri * (t2 - t) * (t2 - t);
            }
            else
            {
                val = q2(i);
            }
        }
        else
        {
            // 사다리꼴 프로파일
            if (t < Vmax(i) / acc)
            {
                // 가속
                val = q1(i) + sign * 0.5 * acc * t * t;
            }
            else if (t < S / Vmax(i))
            {
                // 등속
                val = q1(i) + (sign * 0.5 * Vmax(i) * Vmax(i) / acc) + (sign * Vmax(i) * (t - Vmax(i) / acc));          
            }
            else if (t < Vmax(i) / acc + S / Vmax(i))
            {
                // 감속
                val = q2(i) - sign * 0.5 * acc * (S / Vmax(i) + Vmax(i) / acc - t) * (S / Vmax(i) + Vmax(i) / acc - t);              
            }
            else 
            {
                val = q2(i);              
            }
        }

        Qi(i) = val;
    }

    return  Qi;
}

void PathManager::getMotorPos()
{
    // 각 모터의 현재위치 값 불러오기 ** CheckMotorPosition 이후에 해야함(변수값을 불러오기만 해서 갱신 필요)
    for (auto &entry : motors)
    {
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        {
            c_MotorAngle[motor_mapping[entry.first]] = tMotor->jointAngle;
        }
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
        {
            c_MotorAngle[motor_mapping[entry.first]] = maxonMotor->jointAngle;
        }
    }
}

vector<float> PathManager::makeHomeArr(int cnt)
{
    vector<float> home_arr = {0, 0, 0, 0, 0, 0, 0, 0, 0};

    if (cnt == 1)
    {
        getMotorPos();

        for (int i = 0; i < 9; i++)
        {
            home_arr[i] = c_MotorAngle[i];
        }
        home_arr[1] = 135 * M_PI / 180.0;
        home_arr[2] = 45 * M_PI / 180.0;
        home_arr[7] = 90 * M_PI / 180.0;
        home_arr[8] = 90 * M_PI / 180.0;
    }
    // else if (cnt == 2)
    // {
    //     getMotorPos();

    //     for (int i = 0; i < 9; i++)
    //     {
    //         home_arr[i] = c_MotorAngle[i];
    //     }
    //     home_arr[1] = 135 * M_PI / 180.0;
    //     home_arr[2] = 45 * M_PI / 180.0;
    //     home_arr[7] = 90 * M_PI / 180.0;
    //     home_arr[8] = 90 * M_PI / 180.0;
    // }
    else if (cnt == 2)
    {
        for (int i = 0; i < 9; i++)
        {
            home_arr[i] = homeArr[i];
        }
    }
    else
    {
        std::cout << "Invalid Home Cnt";
    }

    return home_arr;
}

////////////////////////////////////////////////////////////////////////////////
/*                            SYSTEM FUNCTION                                 */
////////////////////////////////////////////////////////////////////////////////

VectorXd PathManager::ikfun_final(VectorXd &pR, VectorXd &pL)
{
    // float direction = 0.0 * M_PI;
    PartLength part_length;

    float X1 = pR(0), Y1 = pR(1), z1 = pR(2);
    float X2 = pL(0), Y2 = pL(1), z2 = pL(2);
    float r1 = part_length.upperArm;
    float r2 = part_length.lowerArm + part_length.stick;
    float L1 = part_length.upperArm;
    float L2 = part_length.lowerArm + part_length.stick;
    float s = part_length.waist;
    float z0 = part_length.height;

    int j = 0, m = 0;
    float the3[1351];
    float zeta = z0 - z2;
    VectorXd Qf(9);
    MatrixXd Q_arr(7,1);
    // float the0_f = 0;

    // the3 배열 초기화
    for (int i = 0; i < 1351; ++i)
        the3[i] = -M_PI / 4.0 + i * M_PI / 1350.0 * (3.0 / 4.0); // the3 범위 : -45deg ~ 90deg

    for (int i = 0; i < 1351; ++i)
    {
        float det_the4 = (z0 - z1 - r1 * cos(the3[i])) / r2;

        if (det_the4 < 1 && det_the4 > -1)
        {
            float the34 = acos((z0 - z1 - r1 * cos(the3[i])) / r2);
            float the4 = the34 - the3[i];

            if (the4 >= 0 && the4 < 120.0 * M_PI / 180.0) // the4 범위 : 0deg ~ 120deg
            {
                float r = r1 * sin(the3[i]) + r2 * sin(the34);
                float det_the1 = (X1 * X1 + Y1 * Y1 - r * r - s * s / 4.0) / (s * r);

                if (det_the1 < 1 && det_the1 > -1)
                {
                    float the1 = acos(det_the1);
                    if (the1 > 0 && the1 < 150.0 * M_PI / 180.0) // the1 범위 : 0deg ~ 150deg
                    {
                        float alpha = asin(X1 / sqrt(X1 * X1 + Y1 * Y1));
                        float det_the0 = (s / 4.0 + (X1 * X1 + Y1 * Y1 - r * r) / s) / sqrt(X1 * X1 + Y1 * Y1);

                        if (det_the0 < 1 && det_the0 > -1)
                        {
                            float the0 = asin(det_the0) - alpha;
                            if (the0 > -M_PI / 3.0 && the0 < M_PI / 3.0) // the0 범위 : -60deg ~ 60deg
                            {
                                float L = sqrt((X2 - 0.5 * s * cos(the0 + M_PI)) * (X2 - 0.5 * s * cos(the0 + M_PI)) + (Y2 - 0.5 * s * sin(the0 + M_PI)) * (Y2 - 0.5 * s * sin(the0 + M_PI)));
                                float det_the2 = (X2 - 0.5 * s * cos(the0 + M_PI)) / L;

                                if (det_the2 < 1 && det_the2 > -1)
                                {
                                    float the2 = acos(det_the2) - the0;
                                    if (the2 > 30 * M_PI / 180.0 && the2 < M_PI) // the2 범위 : 30deg ~ 180deg
                                    {
                                        float Lp = sqrt(L * L + zeta * zeta);
                                        float det_the6 = (Lp * Lp - L1 * L1 - L2 * L2) / (2 * L1 * L2);

                                        if (det_the6 < 1 && det_the6 > -1)
                                        {
                                            float the6 = acos(det_the6);
                                            if (the6 >= 0 && the6 < 120.0 * M_PI / 180.0) // the6 범위 : 0deg ~ 120deg
                                            {
                                                float T = (zeta * zeta + L * L + L1 * L1 - L2 * L2) / (L1 * 2);
                                                float det_the5 = L * L + zeta * zeta - T * T;

                                                if (det_the5 > 0)
                                                {
                                                    float sol = T * L - zeta * sqrt(L * L + zeta * zeta - T * T);
                                                    sol /= (L * L + zeta * zeta);
                                                    float the5 = asin(sol);
                                                    if (the5 > -M_PI / 4 && the5 < M_PI / 2) // the5 범위 : -45deg ~ 90deg
                                                    {
                                                        // if (j == 0 || fabs(the0 - direction) < fabs(the0_f - direction))
                                                        // {
                                                        //     Qf(0) = the0;
                                                        //     Qf(1) = the1;
                                                        //     Qf(2) = the2;
                                                        //     Qf(3) = the3[i];
                                                        //     Qf(4) = the4;
                                                        //     Qf(5) = the5;
                                                        //     Qf(6) = the6;
                                                        //     the0_f = the0;
                                                        //     j = 1;
                                                        // }
                                                        if (j == 0)
                                                        {
                                                            Q_arr(0,0) = the0;
                                                            Q_arr(1,0) = the1;
                                                            Q_arr(2,0) = the2;
                                                            Q_arr(3,0) = the3[i];
                                                            Q_arr(4,0) = the4;
                                                            Q_arr(5,0) = the5;
                                                            Q_arr(6,0) = the6;

                                                            j = 1;
                                                        }
                                                        else
                                                        {
                                                            Q_arr.conservativeResize(Q_arr.rows(), Q_arr.cols() + 1);

                                                            Q_arr(0,j) = the0;
                                                            Q_arr(1,j) = the1;
                                                            Q_arr(2,j) = the2;
                                                            Q_arr(3,j) = the3[i];
                                                            Q_arr(4,j) = the4;
                                                            Q_arr(5,j) = the5;
                                                            Q_arr(6,j) = the6;

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
                }
            }
        }
    }

    if (j == 0)
    {
        cout << "IKFUN is not solved!!\n";
        state.main = Main::Error;
    }
    else
    {
        m = j/2;
        // std::cout << "j = " << j << ", m = " << m << std::endl;
        for (int i = 0; i < 7; i++)
        {
            Qf(i) = Q_arr(i,m);

            // std::cout << "Q(" << i << ") = " << Qf(i) << std::endl;
        }
    }

    Qf(7) = 0.0;
    Qf(8) = 0.0;

    return Qf;
}

vector<float> PathManager::fkfun()
{
    getMotorPos();

    PartLength part_length;
    vector<float> P;
    vector<float> theta(9);
    for (auto &motorPair : motors)
    {
        auto &name = motorPair.first;
        auto &motor = motorPair.second;
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
        {
            theta[motor_mapping[name]] = tMotor->jointAngle;
            cout << name << " : " << theta[motor_mapping[name]] << "\n";
        }
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            theta[motor_mapping[name]] = maxonMotor->jointAngle;
            cout << name << " : " << theta[motor_mapping[name]] << "\n";
        }
    }
    float r1 = part_length.upperArm, r2 = part_length.lowerArm, l1 = part_length.upperArm, l2 = part_length.lowerArm, stick = part_length.stick;
    float s = part_length.waist, z0 = part_length.height;

    P.push_back(0.5 * s * cos(theta[0]) + r1 * sin(theta[3]) * cos(theta[0] + theta[1]) + r2 * sin(theta[3] + theta[4]) * cos(theta[0] + theta[1]) + stick * sin(theta[3] + theta[4] + theta[7]) * cos(theta[0] + theta[1]));
    P.push_back(0.5 * s * sin(theta[0]) + r1 * sin(theta[3]) * sin(theta[0] + theta[1]) + r2 * sin(theta[3] + theta[4]) * sin(theta[0] + theta[1]) + stick * sin(theta[3] + theta[4] + theta[7]) * sin(theta[0] + theta[1]));
    P.push_back(z0 - r1 * cos(theta[3]) - r2 * cos(theta[3] + theta[4]) - stick * cos(theta[3] + theta[4] + theta[7]));
    P.push_back(-0.5 * s * cos(theta[0]) + l1 * sin(theta[5]) * cos(theta[0] + theta[2]) + l2 * sin(theta[5] + theta[6]) * cos(theta[0] + theta[2]) + stick * sin(theta[5] + theta[6] + theta[8]) * cos(theta[0] + theta[2]));
    P.push_back(-0.5 * s * sin(theta[0]) + l1 * sin(theta[5]) * sin(theta[0] + theta[2]) + l2 * sin(theta[5] + theta[6]) * sin(theta[0] + theta[2]) + stick * sin(theta[5] + theta[6] + theta[8]) * sin(theta[0] + theta[2]));
    P.push_back(z0 - l1 * cos(theta[5]) - l2 * cos(theta[5] + theta[6]) - stick * cos(theta[5] + theta[6] + theta[8]));

    return P;
}

bool PathManager::readMeasure___(ifstream& inputFile, bool &BPMFlag)
{
    string row;
    double timeSum = 0.0;

    for (int i = 1; i < measureMatrix.rows(); i++)
    {
        timeSum += measureMatrix(i, 1);
    }

    while(getline(inputFile, row))
    {
        istringstream iss(row);
        string item;
        vector<string> items;

        while (getline(iss, item, '\t'))
        {
            item = trimWhitespace(item);
            items.push_back(item);
        }

        if (!BPMFlag)
        {
            cout << "music";
            bpm = stod(items[0].substr(4));
            cout << " bpm = " << bpm << "\n";
            BPMFlag = 1;

            pre_inst_R << default_right;
            pre_inst_L << default_left;

            measureMatrix.resize(1, 9);
            measureMatrix = MatrixXd::Zero(1, 9);

            state___.resize(2, 3);
            state___ = MatrixXd::Zero(2, 3);
            state___(0, 1) = 1.0;
            state___(1, 1) = 1.0;
        }
        else
        {
            measureMatrix.conservativeResize(measureMatrix.rows() + 1, measureMatrix.cols());
            for (int i = 0; i < 8; i++)
            {
                measureMatrix(measureMatrix.rows() - 1, i) = stod(items[i]);
            }

            // total time 누적
            totalTime += measureMatrix(measureMatrix.rows() - 1, 1);
            measureMatrix(measureMatrix.rows() - 1, 8) = totalTime * 100.0 / bpm;

            // timeSum 누적
            timeSum += measureMatrix(measureMatrix.rows() - 1, 1);

            // timeSum이 threshold를 넘으면 true 반환
            if (timeSum >= threshold)
            {
                std::cout << measureMatrix;
                std::cout << "\n ////////////// time sum : " << timeSum << "\n";

                return true;
            }
        }
    }
    return false;
}

void PathManager::parseMeasure___(MatrixXd &measureMatrix)
{
    VectorXd Measure_time = measureMatrix.col(8);
    VectorXd Measure_R = measureMatrix.col(2);
    VectorXd Measure_L = measureMatrix.col(3);

    pair<VectorXd, VectorXd> R = parseOneArm___(Measure_time, Measure_R, state___.row(0));
    pair<VectorXd, VectorXd> L = parseOneArm___(Measure_time, Measure_L, state___.row(1));

    // 데이터 저장
    inst_i << R.first.block(1,0,9,1), L.first.block(1,0,9,1);
    inst_f << R.first.block(11,0,9,1), L.first.block(11,0,9,1);

    t_i_R = R.first(0);
    t_i_L = L.first(0);
    t_f_R = R.first(10);
    t_f_L = L.first(10);

    t1 = measureMatrix(0, 8);
    t2 = measureMatrix(1, 8);

    hit_state_R.resize(2);
    hit_state_R << measureMatrix(0,2), measureMatrix(1,2);
    hit_state_L.resize(2);
    hit_state_L << measureMatrix(0,3), measureMatrix(1,3);

    state___.block(0,0,1,3) = R.second.transpose();
    state___.block(1,0,1,3) = L.second.transpose();

    std::cout << "\n ////////////// R\n";
    std::cout << inst_i.block(0,0,9,1).transpose() << " -> " << inst_f.block(0,0,9,1).transpose();
    std::cout << "\n /// ti -> tf : " << t_i_R << " -> " << t_f_R;
    
    std::cout << "\n ////////////// L\n";
    std::cout << inst_i.block(9,0,9,1).transpose() << " -> " << inst_f.block(9,0,9,1).transpose();
    std::cout << "\n /// ti -> tf : " << t_i_L << " -> " << t_f_L;

    std::cout << "\n ////////////// t1 -> t2\n";
    std::cout << t1 << " -> " << t2;

    std::cout << "\n ////////////// state\n";
    std::cout << state___;

    // 읽은 줄 삭제
    MatrixXd tmp_matrix(measureMatrix.rows() - 1, measureMatrix.cols());
    tmp_matrix = measureMatrix.block(1, 0, tmp_matrix.rows(), tmp_matrix.cols());
    measureMatrix.resize(tmp_matrix.rows(), tmp_matrix.cols());
    measureMatrix = tmp_matrix;
}

pair<VectorXd, VectorXd> PathManager::parseOneArm___(VectorXd t, VectorXd inst, VectorXd stateVector)
{
    map<int, int> instrument_mapping = {
    {1, 2}, {2, 5}, {3, 6}, {4, 8}, {5, 3}, {6, 1}, {7, 0}, {8, 7}, {11, 2}, {51, 2}, {61, 2}, {71, 2}, {81, 2}, {91, 2}};
    // S      FT      MT      HT      HH      R       RC      LC       S        S        S        S        S        S

    VectorXd inst_i = VectorXd::Zero(9), inst_f = VectorXd::Zero(9);
    VectorXd outputVector = VectorXd::Zero(20);

    VectorXd nextStateVector;

    bool detectHit = false;
    double detectTime = 0, t_i, t_f;
    int detectInst = 0, instNum_i, instNum_f;
    int preState, nextState;
    double threshold = 1.2*100.0/bpm;   // 일단 이렇게 하면 1줄만 읽는 일 없음
    
    // 타격 감지
    for (int i = 1; i < t.rows(); i++)
    {
        if (round(10000*threshold) < round(10000*(t(i) - t(0))))
        {
            break;
        }

        if (inst(i) != 0)
        {
            detectHit = true;
            detectTime = t(i);
            detectInst = inst(i);

            break;
        }
    }

    // inst
    preState = stateVector(2);

    // 타격으로 끝나지 않음
    if (inst(0) == 0)
    {
        // 궤적 생성 중
        if (preState == 2 || preState == 3)
        {
            nextState = preState;

            instNum_i = stateVector(1);
            instNum_f = detectInst;

            t_i = stateVector(0);
            t_f = detectTime;
        }
        else
        {
            // 다음 타격 감지
            if (detectHit)
            {
                nextState = 2;

                instNum_i = stateVector(1);
                instNum_f = detectInst;

                t_i = t(0);
                t_f = detectTime;
            }
            // 다음 타격 감지 못함
            else
            {
                nextState = 0;

                instNum_i = stateVector(1);
                instNum_f = stateVector(1);

                t_i = t(0);
                t_f = t(1);
            }
        }
    }
    // 타격으로 끝남
    else
    {
        // 다음 타격 감지
        if (detectHit)
        {
            nextState = 3;

            instNum_i = inst(0);
            instNum_f = detectInst;

            t_i = t(0);
            t_f = detectTime;
        }
        // 다음 타격 감지 못함
        else
        {
            nextState = 1;

            instNum_i = inst(0);
            instNum_f = inst(0);
            
            t_i = t(0);
            t_f = t(1);
        }
    }

    inst_i(instrument_mapping[instNum_i]) = 1.0;
    inst_f(instrument_mapping[instNum_f]) = 1.0;
    outputVector << t_i, inst_i, t_f, inst_f;

    nextStateVector.resize(3);
    nextStateVector << t_i, instNum_i, nextState;

    return std::make_pair(outputVector, nextStateVector);
}
