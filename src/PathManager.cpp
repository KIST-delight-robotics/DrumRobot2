#include "../include/managers/PathManager.hpp" // 적절한 경로로 변경하세요.


PathManager::PathManager(State &stateRef,
                         CanManager &canManagerRef,
                         std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef,
                         USBIO &usbioRef,
                         Functions &funRef)
    : state(stateRef), canManager(canManagerRef), motors(motorsRef), usbio(usbioRef), fun(funRef)
{
}

////////////////////////////////////////////////////////////////////////////////
/*                              Initialization                                */
////////////////////////////////////////////////////////////////////////////////

void PathManager::getDrumPositoin()
{
    ifstream inputFile("../include/managers/rT.txt");

    if (!inputFile.is_open())
    {
        cerr << "Failed to open the file."
             << "\n";
    }

    // Read data into a 2D vector
    MatrixXd instXYZ(6, 8);

    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 8; ++j)
        {
            inputFile >> instXYZ(i, j);
            if (i == 1 || i == 4)
                instXYZ(i, j) *= 1.0;
        }
    }

    drumCoordinateR.resize(3, 9);
    drumCoordinateL.resize(3, 9);

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
        right_S(i) = instXYZ(i, 0);
        right_FT(i) = instXYZ(i, 1);
        right_MT(i) = instXYZ(i, 2);
        right_HT(i) = instXYZ(i, 3);
        right_HH(i) = instXYZ(i, 4);
        right_R(i) = instXYZ(i, 5);
        right_RC(i) = instXYZ(i, 6);
        right_LC(i) = instXYZ(i, 7);

        left_S(i) = instXYZ(i + 3, 0);
        left_FT(i) = instXYZ(i + 3, 1);
        left_MT(i) = instXYZ(i + 3, 2);
        left_HT(i) = instXYZ(i + 3, 3);
        left_HH(i) = instXYZ(i + 3, 4);
        left_R(i) = instXYZ(i + 3, 5);
        left_RC(i) = instXYZ(i + 3, 6);
        left_LC(i) = instXYZ(i + 3, 7);
    }

    drumCoordinateR << right_S, right_FT, right_MT, right_HT, right_HH, right_R, right_RC, right_LC, right_LC;
    drumCoordinateL << left_S, left_FT, left_MT, left_HT, left_HH, left_R, left_RC, left_LC, left_LC;

    // 악기 별 타격 시 손목 각도
    wristAnglesR.resize(1, 9);
    wristAnglesL.resize(1, 9);

    //              S                  FT                  MT                  HT                  HH                  R                   RC                 LC
    wristAnglesR << 25.0*M_PI/180.0,   25.0*M_PI/180.0,     0.0*M_PI/180.0,     0.0*M_PI/180.0,    10.0*M_PI/180.0,    15.0*M_PI/180.0,    10.0*M_PI/180.0,    10.0*M_PI/180.0, 0;
    wristAnglesL << 25.0*M_PI/180.0,   25.0*M_PI/180.0,     0.0*M_PI/180.0,     0.0*M_PI/180.0,    10.0*M_PI/180.0,    15.0*M_PI/180.0,    10.0*M_PI/180.0,    10.0*M_PI/180.0, 0;
    // wristAnglesL << 0, 0, 0, 0, 0, 0, 0, 0, 0;
}

void PathManager::setReadyAngle()
{
    //////////////////////////////////////// Ready Angle
    readyAngle.resize(12);

    VectorXd defaultInstrumentR;    /// 오른팔 시작 위치
    VectorXd defaultInstrumentL;    /// 왼팔 시작 위치

    VectorXd instrumentVector(18);

    defaultInstrumentR.resize(9);
    defaultInstrumentL.resize(9);
    defaultInstrumentR << 1, 0, 0, 0, 0, 0, 0, 0, 0;    // S
    defaultInstrumentL << 1, 0, 0, 0, 0, 0, 0, 0, 0;    // S

    instrumentVector << defaultInstrumentR,
        defaultInstrumentL;

    MatrixXd combined(6, 18);
    combined << drumCoordinateR, MatrixXd::Zero(3, 9), MatrixXd::Zero(3, 9), drumCoordinateL;
    MatrixXd p = combined * instrumentVector;

    VectorXd pR = VectorXd::Map(p.data(), 3, 1);
    VectorXd pL = VectorXd::Map(p.data() + 3, 3, 1);

    combined.resize(2, 18);
    combined << wristAnglesR, MatrixXd::Zero(1, 9), MatrixXd::Zero(1, 9), wristAnglesL;
    MatrixXd defaultWristAngle = combined * instrumentVector;

    VectorXd waistVector = calWaistAngle(pR, pL);
    VectorXd qk = IKFixedWaist(pR, pL, 0.5 * (waistVector(0) + waistVector(1)), defaultWristAngle(0), defaultWristAngle(1));

    for (int i = 0; i < qk.size(); ++i)
    {
        readyAngle(i) = qk(i);
    }

    elbowAngle eA;
    wristAngle wA;
    readyAngle(4) += eA.stayAngle;
    readyAngle(6) += eA.stayAngle;
    readyAngle(7) += wA.stayAngle;
    readyAngle(8) += wA.stayAngle;

    readyAngle(9) = 0;
    readyAngle(10) = 0;
    readyAngle(11) = 0;

    //////////////////////////////////////// Home Angle
    homeAngle.resize(12);
    //              waist          R_arm1         L_arm1
    homeAngle << 10*M_PI/180.0,  90*M_PI/180.0,  90*M_PI/180.0,
    //              R_arm2         R_arm3         L_arm2
                0*M_PI/180.0,  135*M_PI/180.0,  0*M_PI/180.0,
    //              L_arm3         R_wrist        L_wrist
                135*M_PI/180.0, 60*M_PI/180.0, 60*M_PI/180.0,
    //          Test               R_foot         L_foot            
                0*M_PI/180.0,   90*M_PI/180.0, 90*M_PI/180.0;

    //////////////////////////////////////// Shutdown Angle
    shutdownAngle.resize(12);
        //              waist          R_arm1         L_arm1
    shutdownAngle << 0*M_PI/180.0, 135*M_PI/180.0, 45*M_PI/180.0,
    //                  R_arm2         R_arm3         L_arm2
                    0*M_PI/180.0,  0*M_PI/180.0,   0*M_PI/180.0,
    //                  L_arm3         R_wrist        L_wrist
                    0*M_PI/180.0,  90*M_PI/180.0,  90*M_PI/180.0,
    //          Test               R_foot         L_foot            
                    0,                 0,              0;
}

////////////////////////////////////////////////////////////////////////////////
/*                            Public FUNCTION                                 */
////////////////////////////////////////////////////////////////////////////////

void PathManager::pushAddStancePath(string flagName)
{
    VectorXd Q1 = VectorXd::Zero(12);
    VectorXd Q2 = VectorXd::Zero(12);
    VectorXd Qt = VectorXd::Zero(12);
    float dt = canManager.DTSECOND; // 0.005
    double T = 2.0;                // 2초동안 실행
    double stayTime = 1.0;       // 이전 대기 시간 1초
    int n = (int)(T / dt);
    int stayN = (int)(stayTime / dt);

    // finalMotorPosition -> 마지막 명령값에서 이어서 생성
    Q1 = getMotorPos();

    if (flagName == "isHome")
    {
        for (int i = 0; i < 12; i++)
        {
            Q2(i) = homeAngle(i);
        }
    }
    else if (flagName == "isReady")
    {
        for (int i = 0; i < 12; i++)
        {
            Q2(i) = readyAngle(i);
        }
    }
    else if (flagName == "isShutDown")
    {
        for (int i = 0; i < 12; i++)
        {
            Q2(i) = shutdownAngle(i);
        }
    }

    const float accMax = 100.0; // rad/s^2
    
    VectorXd Vmax = VectorXd::Zero(12);

    Vmax = calVmax(Q1, Q2, accMax, T);

    for (int k = 0; k < 12; k++)
    {
        cout << "Q1[" << k << "] : " << Q1[k] * 180.0 / M_PI << " [deg] -> Q2[" << k << "] : " << Q2[k] * 180.0 / M_PI << " [deg]" << endl;
    }

    for (int k = 1; k <= n + stayN; ++k)
    {
        if (k > stayN)
        {
            // 이동
            float t = (k - stayN) * T / n;

            // Make Array
            Qt = makeProfile(Q1, Q2, Vmax, accMax, t, T);

            // Send to Buffer
            for (auto &entry : motors)
            {
                int can_id = canManager.motorMapping[entry.first];

                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
                {
                    TMotorData newData;
                    newData.position = tMotor->jointAngleToMotorPosition(Qt[can_id]);
                    newData.mode = tMotor->Position;
                    newData.is_brake = 0;
                    tMotor->commandBuffer.push(newData);

                    tMotor->finalMotorPosition = newData.position;
                }
                else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
                {
                    // 1ms 로 동작 (이인우)
                    for (int i = 0; i < 5; i++)
                    {
                        MaxonData newData;
                        newData.position = maxonMotor->jointAngleToMotorPosition(Qt[can_id]);
                        newData.mode = maxonMotor->CSP;
                        maxonMotor->commandBuffer.push(newData);

                        maxonMotor->finalMotorPosition = newData.position;
                    }
                    maxonMotor->pre_q = Qt[can_id];
                }
            }
        }
        else
        {
            // 현재 위치 유지
            for (auto &entry : motors)
            {
                int can_id = canManager.motorMapping[entry.first];

                if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
                {
                    TMotorData newData;
                    newData.position = tMotor->jointAngleToMotorPosition(Q1[can_id]);
                    newData.mode = tMotor->Position;
                    newData.is_brake = 0;
                    tMotor->commandBuffer.push(newData);
                }
                else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
                {
                    // 1ms 로 동작 (이인우)
                    for (int i = 0; i < 5; i++)
                    {
                        MaxonData newData;
                        newData.position = maxonMotor->jointAngleToMotorPosition(Q1[can_id]);
                        newData.mode = maxonMotor->CSP;
                        maxonMotor->commandBuffer.push(newData);
                    }
                    maxonMotor->pre_q = Qt[can_id];
                }
            }
        }
    }
}

void PathManager::initializeValue(int bpm)
{
    endOfPlayCommand = false;
    bpmOfScore = bpm;

    measureState.resize(2, 3);
    measureState = MatrixXd::Zero(2, 3);
    measureState(0, 1) = 1.0;
    measureState(1, 1) = 1.0;

    roundSum = 0.0;
    roundSumHit = 0.0;

    lineData.resize(1, 4);
    lineData = MatrixXd::Zero(1, 4);
    lineData(0, 0) = -1;

    hitState.resize(2, 3);
    hitState = MatrixXd::Zero(2, 3);

    prevLine = VectorXd::Zero(9);

    q0_t1 = readyAngle(0);
    q0_t0 = readyAngle(0);
    t0 = -1;
}

void PathManager::avoidCollision(MatrixXd &measureMatrix)
{
    if (predictCollision(measureMatrix))    // 충돌 예측
    {
        // for (int priority = 0; priority < 5; priority++)    // 수정방법 중 우선순위 높은 것부터 시도
        // {
        //     if (modifyMeasure(measureMatrix, priority))     // 주어진 방법으로 회피되면 measureMatrix를 바꾸고 True 반환
        //     {
        //         colli_debug = 0;
        //         break;
        //     }
        // }
    }
    else
    {
        // std::cout << "\n 충돌 안함 \n";
    }
}

void PathManager::generateTrajectory(MatrixXd &measureMatrix)
{
    ///////////////////////////////////////////////////////////// 궤적
    // position
    pair<VectorXd, VectorXd> initialPosition, finalPosition;
    VectorXd initialPositionR(3);
    VectorXd initialPositionL(3);
    VectorXd finalPositionR(3);
    VectorXd finalPositionL(3);
    VectorXd initialWristAngle(2);
    VectorXd finalWristAngle(2);
    VectorXd waistVector;

    double n, sR, sL;
    double dt = canManager.DTSECOND;

    // parse
    parseMeasure(measureMatrix);

    // 한 줄의 데이터 개수 (5ms 단위)
    n = (line_t2 - line_t1) / dt;
    roundSum += (int)(n * 10000) % 10000;
    if (roundSum >= 10000)
    {
        roundSum -= 10000;
        n++;
    }
    n = floor(n);

    // position
    initialPosition = getTargetPosition(initialInstrument);
    finalPosition = getTargetPosition(finalInstrument);

    initialPositionR << initialPosition.first(0), initialPosition.first(1), initialPosition.first(2);
    initialPositionL << initialPosition.first(3), initialPosition.first(4), initialPosition.first(5);
    finalPositionR << finalPosition.first(0), finalPosition.first(1), finalPosition.first(2);
    finalPositionL << finalPosition.first(3), finalPosition.first(4), finalPosition.first(5);

    // 타격 시 손목 각도
    initialWristAngle = initialPosition.second;
    finalWristAngle = finalPosition.second;

    for (int i = 0; i < n; i++)
    {
        Position Pt;
        double tR = dt * i + line_t1 - initialTimeR;
        double tL = dt * i + line_t1 - initialTimeL;

        sR = timeScaling(0.0, finalTimeR - initialTimeR, tR);
        sL = timeScaling(0.0, finalTimeL - initialTimeL, tL);
        
        // task space 경로
        Pt.trajectoryR = makePath(initialPositionR, finalPositionR, sR);
        Pt.trajectoryL = makePath(initialPositionL, finalPositionL, sL);

        // IK 풀기 위한 손목 각도
        Pt.wristAngleR = tR*(finalWristAngle(0) - initialWristAngle(0))/(finalTimeR - initialTimeR) + initialWristAngle(0);
        Pt.wristAngleL = tL*(finalWristAngle(1) - initialWristAngle(1))/(finalTimeL - initialTimeL) + initialWristAngle(1);

        trajectoryQueue.push(Pt);

        // 데이터 저장
        std::string fileName;
        fileName = "Trajectory_R";
        fun.appendToCSV_DATA(fileName, Pt.trajectoryR[0], Pt.trajectoryR[1], Pt.trajectoryR[2]);
        fileName = "Trajectory_L";
        fun.appendToCSV_DATA(fileName, Pt.trajectoryL[0], Pt.trajectoryL[1], Pt.trajectoryL[2]);
        // fileName = "S";
        // fun.appendToCSV_DATA(fileName, sR, sL, 0);
        // fileName = "T";
        // fun.appendToCSV_DATA(fileName, tR + initialTimeR, tL + initialTimeL, 0);

        if (i == 0)
        {
            // 허리 범위, 및 최적화 각도 계산
            waistVector = calWaistAngle(Pt.trajectoryR, Pt.trajectoryL);
        }
    }

    saveLineData(n, waistVector);

    ///////////////////////////////////////////////////////////// 손목

    MatrixXd dividedMatrix;

    double timeStep = 0.05;
    double lineT = measureMatrix(1,1);
    int repeat = static_cast<int>(round(lineT / timeStep)); // 한 줄 궤적 생성을 위한 반복 횟수
    int k = 0;
    int stateR = 0;
    int stateL = 0;

    dividedMatrix = divideMatrix(measureMatrix);

    for (int i = 0; i < n; i++)
    {
        if (i >= k*n/repeat)
        {
            k++;
            if(i != 0)
            {
                // 읽은 줄 삭제
                MatrixXd tmpMatrix(dividedMatrix.rows() - 1, dividedMatrix.cols());
                tmpMatrix = dividedMatrix.block(1, 0, tmpMatrix.rows(), tmpMatrix.cols());
                dividedMatrix.resize(tmpMatrix.rows(), tmpMatrix.cols());
                dividedMatrix = tmpMatrix;
            }

            // parse
            parseHitData(dividedMatrix);

            // 타격 궤적 만들기
            makeHitCoefficient();

            stateR = hitState(0, 1);
            stateL = hitState(1, 1);
        }
        
        HitAngle Ht;
        float tHitR = dt * (i - (k-1)*n/repeat) + line_t1 - hitR_t1;
        float tHitL = dt * (i - (k-1)*n/repeat) + line_t1 - hitL_t1;

        // 각 관절에 더해줄 각도
        Ht = generateHit(tHitR, tHitL, Ht);

        getHitTime(Ht, stateR, stateL, tHitR, tHitL);

        hitAngleQueue.push(Ht);

        // // 데이터 저장
        // std::string fileName;
        // fileName = "Wrist";
        // fun.appendToCSV_DATA(fileName, Ht.wristR, Ht.wristL, 0);
        // fileName = "HtR";
        // fun.appendToCSV_DATA(fileName, tHitR, hitR_t2 - hitR_t1, 0);
        // fileName = "HtL";
        // fun.appendToCSV_DATA(fileName, tHitL, hitL_t2 - hitL_t1, 0);
    }

    ///////////////////////////////////////////////////////////// 읽은 줄 삭제
    MatrixXd tmpMatrix(measureMatrix.rows() - 1, measureMatrix.cols());
    tmpMatrix = measureMatrix.block(1, 0, tmpMatrix.rows(), tmpMatrix.cols());
    measureMatrix.resize(tmpMatrix.rows(), tmpMatrix.cols());
    measureMatrix = tmpMatrix;
}

void PathManager::solveIKandPushCommand()
{
    int n = lineData(0, 0); // 명령 개수

    makeWaistCoefficient();

    for (int i = 0; i < n; i++)
    {
        // waist angle
        double q0 = getWaistAngle(i);

        // solve IK
        VectorXd q;
        Position Pt = solveIK(q, q0);

        HitAngle nextP;

        nextP = hitAngleQueue.front();
        hitAngleQueue.pop();

        q(4) += nextP.elbowR;
        q(6) += nextP.elbowL;
        q(7) += nextP.wristR;
        q(8) += nextP.wristL;

        // push command buffer
        pushCommandBuffer(q, nextP.Kpp, nextP.isHitR, nextP.isHitL);

        // 데이터 기록
        for (int i = 0; i < 9; i++)
        {
            std::string fileName = "solveIK_q" + to_string(i);
            fun.appendToCSV_DATA(fileName, i, q(i), 0);
        }
    }

    // std::cout << "\n/////////////// Line Data \n";
    // std::cout << lineData;
    // std::cout << "\n ////////////// \n";

    // 커맨드 생성 후 lineData 첫 줄 삭제
    if (lineData.rows() > 1)
    {
        MatrixXd tmpMatrix(lineData.rows() - 1, lineData.cols());
        tmpMatrix = lineData.block(1, 0, tmpMatrix.rows(), tmpMatrix.cols());
        lineData.resize(tmpMatrix.rows(), tmpMatrix.cols());
        lineData = tmpMatrix;
    }
    else if (lineData.rows() == 1)  // 마지막 줄에서 모든 브레이크 정리
    {
        endOfPlayCommand = true;   // DrumRobot 에게 끝났음 알리기
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                           AddStance FUNCTION                               */
////////////////////////////////////////////////////////////////////////////////

VectorXd PathManager::calVmax(VectorXd &q1, VectorXd &q2, float acc, float t2)
{
    VectorXd Vmax = VectorXd::Zero(12);

    for (int i = 0; i < 12; i++)
    {
        double val;
        double S = abs(q2(i) - q1(i)); // 수정됨, overflow방지

        if (S > t2 * t2 * acc / 4)
        {
            // 가속도로 도달 불가능
            // -1 반환
            val = -1;
        }
        else
        {
            // 2차 방정식 계수
            double A = 1 / acc;
            double B = -1 * t2;
            double C = S;

            // 2차 방정식 해
            double sol1 = (-B + sqrt(B * B - 4 * A * C)) / 2 / A;
            double sol2 = (-B - sqrt(B * B - 4 * A * C)) / 2 / A;

            if (sol1 >= 0 && sol1 <= acc * t2 / 2)
            {
                val = sol1;
            }
            else if (sol2 >= 0 && sol2 <= acc * t2 / 2)
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
    VectorXd Qi = VectorXd::Zero(12);

    for (int i = 0; i < 12; i++)
    {
        double val, S;
        int sign;

        S = q2(i) - q1(i);

        // 부호 확인, 이동거리 양수로 변경
        if (S < 0)
        {
            S = abs(S);
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
            double acc_tri = 4 * S / t2 / t2;

            if (t < t2 / 2)
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

    return Qi;
}

VectorXd PathManager::getMotorPos()
{
    VectorXd Qf = VectorXd::Zero(12);

    // finalMotorPosition 가져오기
    // 마지막 명령값에서 이어서 생성
    for (auto &entry : motors)
    {
        int can_id = canManager.motorMapping[entry.first];

        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        {
            // currentMotorAngle[can_id] = tMotor->motorPositionToJointAngle(tMotor->finalMotorPosition);
            Qf(can_id) = tMotor->motorPositionToJointAngle(tMotor->finalMotorPosition);
        }
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
        {
            // currentMotorAngle[can_id] = maxonMotor->motorPositionToJointAngle(maxonMotor->finalMotorPosition);
            Qf(can_id) = maxonMotor->motorPositionToJointAngle(maxonMotor->finalMotorPosition);
        }
    }

    return Qf;
}

void PathManager::getAddStanceCoefficient(VectorXd Q1, VectorXd Q2, double t)
{
    // 이인우
}

////////////////////////////////////////////////////////////////////////////////
/*                       Make Task Space Trajectory                           */
////////////////////////////////////////////////////////////////////////////////

void PathManager::parseMeasure(MatrixXd &measureMatrix)
{
    VectorXd measureTime = measureMatrix.col(8);
    VectorXd measureInstrumentR = measureMatrix.col(2);
    VectorXd measureInstrumentL = measureMatrix.col(3);
    VectorXd measureIntensityR = measureMatrix.col(4);
    VectorXd measureIntensityL = measureMatrix.col(5);

    // parsing data
    line_t1 = measureMatrix(0, 8);
    line_t2 = measureMatrix(1, 8);

    // std::cout << "\n /// t1 -> t2 : " << line_t1 << " -> " << line_t2 << " = " << line_t2 - line_t1 <<  "\n";

    // std::cout << "\n /// R ///";
    pair<VectorXd, VectorXd> dataR = parseOneArm(measureTime, measureInstrumentR, measureState.row(0));
    // std::cout << "\n /// L ///";
    pair<VectorXd, VectorXd> dataL = parseOneArm(measureTime, measureInstrumentL, measureState.row(1));

    // measureState 저장
    measureState.block(0, 0, 1, 3) = dataR.second.transpose();
    measureState.block(1, 0, 1, 3) = dataL.second.transpose();

    // 악기
    initialInstrument << dataR.first.block(1, 0, 9, 1), dataL.first.block(1, 0, 9, 1);
    finalInstrument << dataR.first.block(11, 0, 9, 1), dataL.first.block(11, 0, 9, 1);
    
    // 궤적 시간
    initialTimeR = dataR.first(0);
    initialTimeL = dataL.first(0);
    finalTimeR = dataR.first(10);
    finalTimeL = dataL.first(10);
}

pair<VectorXd, VectorXd> PathManager::parseOneArm(VectorXd t, VectorXd inst, VectorXd stateVector)
{
    map<int, int> instrumentMapping = {
        {1, 0}, {2, 1}, {3, 2}, {4, 3}, {5, 4}, {6, 5}, {7, 6}, {8, 7}, {11, 0}, {51, 0}, {61, 0}, {71, 0}, {81, 0}, {91, 0}};
    //    S       FT      MT      HT      HH       R      RC      LC       S        S        S        S        S        S

    VectorXd initialInstrument = VectorXd::Zero(9), finalInstrument = VectorXd::Zero(9);
    VectorXd outputVector = VectorXd::Zero(20);

    VectorXd nextStateVector;

    bool detectHit = false;
    double detectTime = 0, initialT, finalT;
    int detectInst = 0, initialInstNum, finalInstNum;
    int preState, nextState;
    double hitDetectionThreshold = 1.2 * 100.0 / bpmOfScore; // 일단 이렇게 하면 1줄만 읽는 일 없음

    // 타격 감지
    for (int i = 1; i < t.rows(); i++)
    {
        if (round(10000 * hitDetectionThreshold) < round(10000 * (t(i) - t(0))))
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

            initialInstNum = stateVector(1);
            finalInstNum = detectInst;

            initialT = stateVector(0);
            finalT = detectTime;
        }
        else
        {
            // 다음 타격 감지
            if (detectHit)
            {
                nextState = 2;

                initialInstNum = stateVector(1);
                finalInstNum = detectInst;

                initialT = t(0);
                finalT = detectTime;
            }
            // 다음 타격 감지 못함
            else
            {
                nextState = 0;

                initialInstNum = stateVector(1);
                finalInstNum = stateVector(1);

                initialT = t(0);
                finalT = t(1);
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

            initialInstNum = inst(0);
            finalInstNum = detectInst;

            initialT = t(0);
            finalT = detectTime;
        }
        // 다음 타격 감지 못함
        else
        {
            nextState = 1;

            initialInstNum = inst(0);
            finalInstNum = inst(0);

            initialT = t(0);
            finalT = t(1);
        }
    }

    initialInstrument(instrumentMapping[initialInstNum]) = 1.0;
    finalInstrument(instrumentMapping[finalInstNum]) = 1.0;
    outputVector << initialT, initialInstrument, finalT, finalInstrument;

    nextStateVector.resize(3);
    nextStateVector << initialT, initialInstNum, nextState;

    // std::cout << "\n insti -> instf : " << initialInstNum << " -> " << finalInstNum;
    // std::cout << "\n ti -> tf : " << initialT << " -> " << finalT;
    // std::cout << "\n state : " << nextStateVector.transpose();
    // std::cout << "\n";

    return std::make_pair(outputVector, nextStateVector);
}

pair<VectorXd, VectorXd> PathManager::getTargetPosition(VectorXd inst)
{
    VectorXd instrumentR = inst.segment(0, 9);
    VectorXd instrumentL = inst.segment(9, 9);

    if (instrumentR.sum() == 0)
    {
        std::cout << "Right Instrument Vector Error!!\n";
    }

    if (instrumentL.sum() == 0)
    {
        std::cout << "Left Instrument Vector Error!!\n";
    }

    VectorXd instrumentVector(18);
    instrumentVector << instrumentR, instrumentL;

    MatrixXd combined(6, 18);
    combined << drumCoordinateR, MatrixXd::Zero(3, 9), MatrixXd::Zero(3, 9), drumCoordinateL;
    MatrixXd p = combined * instrumentVector;
    
    combined.resize(2, 18);
    combined << wristAnglesR, MatrixXd::Zero(1, 9), MatrixXd::Zero(1, 9), wristAnglesL;
    MatrixXd angle = combined * instrumentVector;

    return std::make_pair(p, angle);
}

double PathManager::timeScaling(double ti, double tf, double t)
{
    float s;

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd sol;

    A.resize(4, 4);
    b.resize(4, 1);

    A << 1, ti, ti * ti, ti * ti * ti,
        1, tf, tf * tf, tf * tf * tf,
        0, 1, 2 * ti, 3 * ti * ti,
        0, 1, 2 * tf, 3 * tf * tf;

    b << 0, 1, 0, 0;

    A_1 = A.inverse();
    sol = A_1 * b;

    s = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;

    return s;
}

VectorXd PathManager::makePath(VectorXd Pi, VectorXd Pf, double s)
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

            Ps(2) = a * std::pow(s, degree) + b;
        }
        else
        {
            float amp = abs(zi - zf) / 2;    // sin 함수 amplitude
            float a = (zi - zf) * std::pow(-1, degree);
            float b = zf;

            Ps(2) = a * std::pow(s - 1, degree) + b + (amp * sin(M_PI * s));    // sin 궤적 추가
            // Ps(2) = a * std::pow(s - 1, degree) + b;
        }
    }

    return Ps;
}

VectorXd PathManager::calWaistAngle(VectorXd pR, VectorXd pL)
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

void PathManager::saveLineData(int n, VectorXd waistVector)
{
    if (lineData(0, 0) == -1)   // 첫 줄
    {
        lineData(0, 0) = n;                         // 명령 개수
        lineData(0, 1) = waistVector(2);            // q0 최적값
        lineData(0, 2) = waistVector(0);            // q0 min
        lineData(0, 3) = waistVector(1);            // q0 max
    }
    else
    {
        lineData.conservativeResize(lineData.rows() + 1, lineData.cols());
        lineData(lineData.rows() - 1, 0) = n;                       // 명령 개수
        lineData(lineData.rows() - 1, 1) = waistVector(2);          // q0 최적값
        lineData(lineData.rows() - 1, 2) = waistVector(0);          // q0 min
        lineData(lineData.rows() - 1, 3) = waistVector(1);          // q0 max
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                           Make Hit Trajectory                              */
////////////////////////////////////////////////////////////////////////////////

MatrixXd PathManager::divideMatrix(MatrixXd &measureMatrix)
{
    double timeStep = 0.05;
    int numCols = measureMatrix.cols();
    std::vector<Eigen::VectorXd> newRows;
    newRows.push_back(prevLine);

    double prevTotalTime = prevLine(8);

    for (int i = 0; i < measureMatrix.rows() - 1; ++i)
    {
        double duration = measureMatrix(i + 1, 1);
        double endTime = measureMatrix(i + 1, 8);
        int steps = static_cast<int>(round(duration / timeStep));
        double sumStep = (endTime-prevTotalTime) / steps;

        for (int s = 0; s < steps; ++s)
        {
            Eigen::VectorXd row(numCols);
            row.setZero();

            row(0) = measureMatrix(i + 1, 0);                 
            row(1) = timeStep;                              
            row(8) = prevTotalTime + (s + 1) * sumStep;          

            if (s + 1 == steps)
            {
                for (int j = 2; j <= 7; ++j)
                {
                    row(j) = measureMatrix(i + 1, j);
                }
            }

            newRows.push_back(row);
        }

        prevTotalTime = endTime;

    }

    Eigen::MatrixXd parsedMatrix(newRows.size(), numCols);
    for (size_t i = 0; i < newRows.size(); ++i)
    {
        parsedMatrix.row(i) = newRows[i];
    }

    prevLine = measureMatrix.row(1);

    return parsedMatrix;
}

void PathManager::parseHitData(MatrixXd &measureMatrix)
{
    VectorXd t = measureMatrix.col(8);
    VectorXd hitR = measureMatrix.col(4);
    VectorXd hitL = measureMatrix.col(5);

    // parsing data
    line_t1 = measureMatrix(0, 8);
    line_t2 = measureMatrix(1, 8);
    double hitDetectionThreshold = 0.5;

    //////////////////////////////////////// R

    bool detectHit = false;
    double hitTime = t(1);
    int intensity = 0;
    // 다음 타격 찾기
    for (int i = 1; i < t.rows(); i++)
    {
        if (round(10000 * hitDetectionThreshold) < round(10000 * (t(i) - t(0))))
        {
            if (i != 1)     // 이인우 : 첫 줄은 무조건 읽도록 (임시) 
            {
                break;
            }
        }

        if (hitR(i) != 0)
        {
            detectHit = true;
            hitTime = t(i);
            intensity = hitR(i);
            break;
        }
    }

    if (hitR(0) == 0)  // 현재 줄이 타격이 아님
    {
        if (hitState(0,1) == 2 || hitState(0,1) == 3) // 이전에 타격 O
        {
            hitR_t1 = hitState(0,0);
            hitR_t2 = hitTime;
        }
        else    // 이전에 타격 X
        {
            if (detectHit)
            {
                hitState(0,1) = 2;
                hitR_t1 = t(0);
                hitR_t2 = hitTime;
                hitState(0,0) = t(0);
                hitState(0,2) = intensity;
            }
            else
            {
                hitState(0,1) = 0;
                hitR_t1 = t(0);
                hitR_t2 = t(1);
            }
        }
    }
    else
    {
        // 다음 타격 찾기
        if (detectHit)
        {
            hitState(0,1) = 3;
            hitR_t1 = t(0);
            hitR_t2 = hitTime;
            hitState(0,0) = t(0);
            hitState(0,2) = intensity;
        }
        else
        {
            hitState(0,1) = 1;
            hitR_t1 = t(0);
            hitR_t2 = t(1);
        }
    }

    //////////////////////////////////////// L

    detectHit = false;
    hitTime = t(1);
    intensity = 0;
    // 다음 타격 찾기
    for (int i = 1; i < t.rows(); i++)
    {
        if (round(10000 * hitDetectionThreshold) < round(10000 * (t(i) - t(0))))
        {
            if (i != 1)     // 이인우 : 첫 줄은 무조건 읽도록 (임시) 
            {
                break;
            }
        }

        if (hitL(i) != 0)
        {
            detectHit = true;
            hitTime = t(i);
            intensity = hitL(i);
            break;
        }
    }

    if (hitL(0) == 0)  // 현재 줄이 타격이 아님
    {
        if (hitState(1,1) == 2 || hitState(1,1) == 3) // 이전에 타격 O
        {
            hitL_t1 = hitState(1,0);
            hitL_t2 = hitTime;
        }
        else    // 이전에 타격 X
        {
            if (detectHit)
            {
                hitState(1,1) = 2;
                hitL_t1 = t(0);
                hitL_t2 = hitTime;
                hitState(1,0) = t(0);
                hitState(1,2) = intensity;
            }
            else
            {
                hitState(1,1) = 0;
                hitL_t1 = t(0);
                hitL_t2 = t(1);
            }
        }
    }
    else
    {
        // 다음 타격 찾기
        if (detectHit)
        {
            hitState(1,1) = 3;
            hitL_t1 = t(0);
            hitL_t2 = hitTime;
            hitState(1,0) = t(0);
            hitState(1,2) = intensity;
        }
        else
        {
            hitState(1,1) = 1;
            hitL_t1 = t(0);
            hitL_t2 = t(1);
        }
    }
}

void PathManager::makeHitCoefficient()
{
    int stateR = hitState(0, 1);
    int stateL = hitState(1, 1);
    int intensityR = hitState(0, 2);
    int intensityL = hitState(1, 2);

    // 타격 관련 파라미터
    elbowAngle elbowAngleR, elbowAngleL;
    wristAngle wristAngleR, wristAngleL;

    elbowTimeR = getElbowTime(hitR_t1, hitR_t2, intensityR);
    elbowTimeL = getElbowTime(hitL_t1, hitL_t2, intensityL);

    wristTimeR = getWristTime(hitR_t1, hitR_t2, intensityR);
    wristTimeL = getWristTime(hitL_t1, hitL_t2, intensityL);

    elbowAngleR = getElbowAngle(hitR_t1, hitR_t2, intensityR);
    elbowAngleL = getElbowAngle(hitL_t1, hitL_t2, intensityL);

    wristAngleR = getWristAngle(hitR_t1, hitR_t2, intensityR);
    wristAngleL = getWristAngle(hitL_t1, hitL_t2, intensityL);

    // 계수 행렬 구하기
    elbowCoefficientR = makeElbowCoefficient(stateR, elbowTimeR, elbowAngleR);
    elbowCoefficientL = makeElbowCoefficient(stateL, elbowTimeL, elbowAngleL);

    wristCoefficientR = makeWristCoefficient(stateR, wristTimeR, wristAngleR);
    wristCoefficientL = makeWristCoefficient(stateL, wristTimeL, wristAngleL);
}

PathManager::elbowTime PathManager::getElbowTime(float t1, float t2, int intensity)
{
    float T = t2 - t1;
    elbowTime elbowTime;

    elbowTime.stayTime = std::max(0.5 * (T), T - 0.2);
    elbowTime.liftTime = std::max(0.5 * (T), T - 0.2);
    elbowTime.hitTime = T;
    
    return elbowTime;
}

PathManager::wristTime PathManager::getWristTime(float t1, float t2, int intensity)
{
    float T = t2 - t1;
    wristTime wristTime;

    wristTime.releaseTime = std::min(0.2 * (T), 0.1);

    t2 - t1 < 0.15 ? wristTime.stayTime = 0.45 * (T) : wristTime.stayTime = 0.47 * (T) - 0.05;

    if (intensity < 4)
        wristTime.liftTime = std::max(0.5 * (T), T - 0.25);
    else if (intensity == 4)
        wristTime.liftTime = std::max(0.6 * (T), T - 0.2);
    else
        wristTime.liftTime = std::max(0.7 * (T), T - 0.15);
    
    wristTime.hitTime = T;
    
    return wristTime;
}

PathManager::elbowAngle PathManager::getElbowAngle(float t1, float t2, int intensity)
{
    float T = t2 - t1;
    elbowAngle elbowAngle;
    int temp = intensity;
    // float intensityFactor = 0.4 * intensity + 0.2; // 1 : 약하게   2 : 기본    3 : 강하게
    // double intensityFactor = 0.0179 * intensity * intensity + 0.1464 * intensity + 0.1286;  // 1 : 30%, 2: 50%, 3: 70%, 4: 100%, 5: 130%, 6: 170%, 7: 200%
    double intensityFactor = 0.01786 * temp * temp + 0.11071 * temp;  // 1: 0%, 2: 30%, 3: 50%, 4: 70%, 5: 100%, 6: 130%, 7: 170%, 8: 200%  

    if (T < 0.2)
    {
        elbowAngle.liftAngle = 0;
    }
    else if (T <= 0.5)
    {
        elbowAngle.liftAngle = (T) * (10 * M_PI / 180.0) / 0.5;
    }
    else
    {
        elbowAngle.liftAngle = 10 * M_PI / 180.0;
    }

    // elbowAngle.liftAngle = std::min((T) * (10 * M_PI / 180.0) / 0.5, (10 * M_PI / 180.0));

    elbowAngle.liftAngle = elbowAngle.stayAngle + elbowAngle.liftAngle * intensityFactor;

    return elbowAngle;
}

PathManager::wristAngle PathManager::getWristAngle(float t1, float t2, int intensity)
{
    float T = t2 - t1;
    wristAngle wristAngle;
    int temp = intensity;
    // float intensityFactor = 0.4 * intensity + 0.2; // 1 : 약하게    2 : 기본    3 : 강하게
    // double intensityFactor = 0.0179 * intensity * intensity + 0.1464 * intensity + 0.1286;  // 1 : 30%, 2: 50%, 3: 70%, 4: 100%, 5: 130%, 6: 170%, 7: 200%
    double intensityFactor = 0.01786 * temp * temp + 0.11071 * temp;  // 1: 0%, 2: 30%, 3: 50%, 4: 70%, 5: 100%, 6: 130%, 7: 170%, 8: 200%  

    wristAngle.stayAngle = 10 * M_PI / 180.0;
    t2 - t1 < 0.5 ? wristAngle.liftAngle = (-100 * ((T) - 0.5) * ((T) - 0.5) + 30) * M_PI / 180.0 : wristAngle.liftAngle = 30  * M_PI / 180.0;
    wristAngle.pressAngle = -1.0 * std::min((T) * (5 * M_PI / 180.0)/ 0.5, (5 * M_PI / 180.0));
    if (intensity != 1)
    {
        wristAngle.liftAngle = wristAngle.stayAngle + wristAngle.liftAngle * intensityFactor;
    }
    else
    {
        wristAngle.liftAngle = wristAngle.stayAngle;
    }
    

    return wristAngle;
}

MatrixXd PathManager::makeElbowCoefficient(int state, elbowTime eT, elbowAngle eA)
{
    MatrixXd elbowCoefficient;
    
    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd sol, sol2;

    if (state == 0)
    {
        // Stay
        elbowCoefficient.resize(2, 4);
        elbowCoefficient << eA.stayAngle, 0, 0, 0,  // Stay
                            eA.stayAngle, 0, 0, 0;  // Stay
    }
    else if (state == 1)
    {
        // Release - Stay

        // // Release
        // A.resize(4, 4);
        // b.resize(4, 1);

        // A << 1, 0, 0, 0,
        //     1, eT.stayTime, eT.stayTime * eT.stayTime, eT.stayTime * eT.stayTime * eT.stayTime,
        //     0, 1, 0, 0,
        //     0, 1, 2 * eT.stayTime, 3 * eT.stayTime * eT.stayTime;

        // b << 0, eA.stayAngle, 0, 0;

        // A_1 = A.inverse();
        // sol = A_1 * b;

        // elbowCoefficient.resize(2, 4);
        // elbowCoefficient << sol(0), sol(1), sol(2), sol(3), // Release
        //                     eA.stayAngle, 0, 0, 0;          // Stay

        // Release
        if (eA.stayAngle == 0)
        {
            sol.resize(4, 1);
            sol << 0, 0, 0, 0;
        }
        else
        {
            A.resize(4, 4);
            b.resize(4, 1);

            A << 1, 0, 0, 0,
                1, eT.hitTime, eT.hitTime * eT.hitTime, eT.hitTime * eT.hitTime * eT.hitTime,
                0, 1, 0, 0,
                0, 1, 2 * eT.hitTime, 3 * eT.hitTime * eT.hitTime;

            b << 0, eA.stayAngle, 0, 0;

            A_1 = A.inverse();
            sol = A_1 * b;
        }

        elbowCoefficient.resize(2, 4);
        elbowCoefficient << sol(0), sol(1), sol(2), sol(3), // Release
                            sol(0), sol(1), sol(2), sol(3); // Release
    }
    else if (state == 2)
    {
        // Lift - Hit

        // Lift
        A.resize(4, 4);
        b.resize(4, 1);

        if(eA.stayAngle == eA.liftAngle)
        {
            sol.resize(4,1);
            sol << eA.stayAngle, 0, 0, 0;
        }
        else
        {
            A << 1, 0, 0, 0,
            1, eT.liftTime, eT.liftTime * eT.liftTime, eT.liftTime * eT.liftTime * eT.liftTime,
            0, 1, 0, 0,
            0, 1, 2 * eT.liftTime, 3 * eT.liftTime * eT.liftTime;

            b << eA.stayAngle, eA.liftAngle, 0, 0;

            A_1 = A.inverse();
            sol = A_1 * b;
        }

        // Hit
        if (eA.liftAngle == 0)
        {
            sol.resize(4, 1);
            sol << 0, 0, 0, 0;
        }
        else
        {
            A.resize(4, 4);
            b.resize(4, 1);

            A << 1, eT.liftTime, eT.liftTime * eT.liftTime, eT.liftTime * eT.liftTime * eT.liftTime,
                1, eT.hitTime, eT.hitTime * eT.hitTime, eT.hitTime * eT.hitTime * eT.hitTime,
                0, 1, 2 * eT.liftTime, 3 * eT.liftTime * eT.liftTime,
                0, 1, 2 * eT.hitTime, 3 * eT.hitTime * eT.hitTime;

            b << eA.liftAngle, 0, 0, 0;

            A_1 = A.inverse();
            sol2 = A_1 * b;
        }

        elbowCoefficient.resize(2, 4);
        elbowCoefficient << sol(0), sol(1), sol(2), sol(3),     // Lift
                            sol2(0), sol2(1), sol2(2), sol2(3); // Hit
    }
    else if (state == 3)
    {
        // Lift - Hit

        // Lift
        if (eA.liftAngle == 0)
        {
            sol.resize(4, 1);
            sol << 0, 0, 0, 0;

            sol2.resize(4, 1);
            sol2 << 0, 0, 0, 0;
        }
        else
        {
            A.resize(4, 4);
            b.resize(4, 1);

            A << 1, 0, 0, 0,
                1, eT.liftTime, eT.liftTime * eT.liftTime, eT.liftTime * eT.liftTime * eT.liftTime,
                0, 1, 0, 0,
                0, 1, 2 * eT.liftTime, 3 * eT.liftTime * eT.liftTime;

            b << 0, eA.liftAngle, 0, 0;

            A_1 = A.inverse();
            sol = A_1 * b;

            // Hit
            A.resize(4, 4);
            b.resize(4, 1);

            A << 1, eT.liftTime, eT.liftTime * eT.liftTime, eT.liftTime * eT.liftTime * eT.liftTime,
                1, eT.hitTime, eT.hitTime * eT.hitTime, eT.hitTime * eT.hitTime * eT.hitTime,
                0, 1, 2 * eT.liftTime, 3 * eT.liftTime * eT.liftTime,
                0, 1, 2 * eT.hitTime, 3 * eT.hitTime * eT.hitTime;

            b << eA.liftAngle, 0, 0, 0;

            A_1 = A.inverse();
            sol2 = A_1 * b;
        }

        elbowCoefficient.resize(2, 4);
        elbowCoefficient << sol(0), sol(1), sol(2), sol(3),     // Lift
                            sol2(0), sol2(1), sol2(2), sol2(3); // Hit
    }

    return elbowCoefficient;
}

MatrixXd PathManager::makeWristCoefficient(int state, wristTime wT, wristAngle wA)
{
    MatrixXd wristCoefficient;
    
    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd sol, sol2;

    if (state == 0)
    {
        // Stay
        wristCoefficient.resize(4, 4);
        wristCoefficient << wA.stayAngle, 0, 0, 0,  // Stay
                            wA.stayAngle, 0, 0, 0,  // Stay
                            wA.stayAngle, 0, 0, 0,  // Stay
                            wA.stayAngle, 0, 0, 0;  // Stay
    }
    else if (state == 1)
    {
        // Release - Stay

        // // Release
        // A.resize(3, 3);
        // b.resize(3, 1);

        // A << 1, 0, 0,
        //     1, wT.releaseTime, wT.releaseTime * wT.releaseTime,
        //     0, 1, 2 * wT.releaseTime;

        // b << wA.pressAngle, wA.stayAngle, 0;

        // A_1 = A.inverse();
        // sol = A_1 * b;

        // wristCoefficient.resize(4, 4);
        // wristCoefficient << sol(0), sol(1), sol(2), 0,  // Release
        //                     wA.stayAngle, 0, 0, 0,      // Stay
        //                     wA.stayAngle, 0, 0, 0,      // Stay
        //                     wA.stayAngle, 0, 0, 0;      // Stay

        // Release
        if (wA.pressAngle == wA.stayAngle)
        {
            sol.resize(4, 1);
            sol << wA.stayAngle, 0, 0, 0;
        }
        else
        {
            A.resize(3, 3);
            b.resize(3, 1);

            A << 1, 0, 0,
                1, wT.hitTime, wT.hitTime * wT.hitTime,
                0, 1, 2 * wT.hitTime;

            b << wA.pressAngle, wA.stayAngle, 0;

            A_1 = A.inverse();
            sol = A_1 * b;
        }

        wristCoefficient.resize(4, 4);
        wristCoefficient << sol(0), sol(1), sol(2), 0,  // Release
                            sol(0), sol(1), sol(2), 0,  // Release
                            sol(0), sol(1), sol(2), 0,  // Release
                            sol(0), sol(1), sol(2), 0;  // Release
    }
    else if (state == 2)
    {
        // Stay - Lift - Hit

        // Lift
        A.resize(4, 4);
        b.resize(4, 1);

        // Lift Angle = Stay Angle일 때 -> 아주 작게 칠 때
        if (wA.stayAngle == wA.liftAngle)
        {
            sol.resize(4, 1);
            sol << wA.stayAngle, 0, 0, 0;
        }
        else
        {
             A << 1, wT.stayTime, wT.stayTime * wT.stayTime, wT.stayTime * wT.stayTime * wT.stayTime,
                1, wT.liftTime, wT.liftTime * wT.liftTime, wT.liftTime * wT.liftTime * wT.liftTime,
                0, 1, 2 * wT.stayTime, 3 * wT.stayTime * wT.stayTime,
                0, 1, 2 * wT.liftTime, 3 * wT.liftTime * wT.liftTime;

            b << wA.stayAngle, wA.liftAngle, 0, 0;

            A_1 = A.inverse();
            sol = A_1 * b;
        }

        // Hit
        if (wA.liftAngle == wA.pressAngle)
        {
            sol2.resize(4, 1);
            sol2 << wA.liftAngle, 0, 0, 0;
        }
        else
        {
            A.resize(3, 3);
            b.resize(3, 1);

            A << 1, wT.liftTime, wT.liftTime * wT.liftTime,
                1, wT.hitTime, wT.hitTime * wT.hitTime,
                0, 1, 2 * wT.liftTime;

            b << wA.liftAngle, wA.pressAngle, 0;

            A_1 = A.inverse();
            sol2 = A_1 * b;
        }

        wristCoefficient.resize(4, 4);
        wristCoefficient << wA.stayAngle, 0, 0, 0,          // Stay
                            wA.stayAngle, 0, 0, 0,          // Stay
                            sol(0), sol(1), sol(2), sol(3), // Lift
                            sol2(0), sol2(1), sol2(2), 0;   // Hit
    }
    else if (state == 3)
    {
        // Lift - Hit

        // Lift
        if (wA.pressAngle == wA.liftAngle)
        {
            sol.resize(4, 1);
            sol << wA.liftAngle, 0, 0, 0;
        }
        else
        {
            A.resize(3, 3);
            b.resize(3, 1);

            A << 1, 0, 0,
                1, wT.stayTime, wT.stayTime * wT.stayTime,
                0, 1, 2 * wT.stayTime;

            b << wA.pressAngle, wA.liftAngle, 0;

            A_1 = A.inverse();
            sol = A_1 * b;
        }

        // Hit
        if (wA.liftAngle == wA.pressAngle)
        {
            sol2.resize(4, 1);
            sol2 << wA.pressAngle, 0, 0, 0;
        }
        else
        {
            A.resize(3, 3);
            b.resize(3, 1);

            A << 1, wT.liftTime, wT.liftTime * wT.liftTime,
                1, wT.hitTime, wT.hitTime * wT.hitTime,
                0, 1, 2 * wT.liftTime;

            b << wA.liftAngle, wA.pressAngle, 0;

            A_1 = A.inverse();
            sol2 = A_1 * b;
        }

        wristCoefficient.resize(4, 4);
        wristCoefficient << sol(0), sol(1), sol(2), 0,      // Lift
                            sol(0), sol(1), sol(2), 0,      // Lift
                            wA.liftAngle, 0, 0, 0,          // Stay (Lift Angle)
                            sol2(0), sol2(1), sol2(2), 0;   // Hit
    }

    return wristCoefficient;
}

double PathManager::makeElbowAngle(double t, elbowTime eT, MatrixXd coefficientMatrix)
{
    MatrixXd tMatrix;
    tMatrix.resize(4, 1);
    tMatrix << 1, t, t*t, t*t*t;

    MatrixXd elbowAngle = coefficientMatrix * tMatrix;

    if (t < eT.liftTime)
    {
        return elbowAngle(0);
    }
    else
    {
        return elbowAngle(1);
    }
}

double PathManager::makeWristAngle(double t, wristTime wT, MatrixXd coefficientMatrix)
{
    MatrixXd tMatrix;
    tMatrix.resize(4, 1);
    tMatrix << 1, t, t*t, t*t*t;

    MatrixXd wristAngle = coefficientMatrix * tMatrix;

    if (t < wT.releaseTime)
    {
        return wristAngle(0);
    }
    else if (t < wT.stayTime)
    {
        return wristAngle(1);
    }
    else if (t < wT.liftTime)
    {
        return wristAngle(2);
    }
    else
    {
        return wristAngle(3);
    }
}

PathManager::HitAngle PathManager::generateHit(float tHitR, float tHitL, HitAngle &Pt)
{
    Pt.Kpp = VectorXd::Zero(9);   // Kp 에 곱해지는 값

    // 각 관절에 더해줌
    Pt.elbowR = makeElbowAngle(tHitR, elbowTimeR, elbowCoefficientR);
    Pt.elbowL = makeElbowAngle(tHitL, elbowTimeL, elbowCoefficientL);
    Pt.wristR = makeWristAngle(tHitR, wristTimeR, wristCoefficientR);
    Pt.wristL = makeWristAngle(tHitL, wristTimeL, wristCoefficientL);

    // Kp : 위치에 따라 감소
    wristAngle wA;
    if (Pt.wristR >= wA.stayAngle)
    {
        Pt.Kpp(7) = 1;
    }
    else
    {
        if (Kppp < 1.0)
        {
            Pt.Kpp(7) = 1 - Kppp * (wA.stayAngle - Pt.wristR) / (wA.stayAngle + 5.0 * M_PI / 180.0);
        }
        else
        {
            double s = (wA.stayAngle - Pt.wristR) / (wA.stayAngle + 5.0 * M_PI / 180.0);   // Pt.wristAngleR : stayAngle -> -5
                                                                                                // s :0 -> 1
            Pt.Kpp(7) = exp(-1.0*s*Kppp);
        }
    }

    if (Pt.wristL >= wA.stayAngle)
    {
       Pt.Kpp(8) = 1;
    }
    else
    {
        if (Kppp < 1.0)
        {
            Pt.Kpp(8) = 1 - Kppp * (wA.stayAngle - Pt.wristL) / (wA.stayAngle + 5.0 * M_PI / 180.0);
        }
        else
        {
            double s = (wA.stayAngle - Pt.wristL) / (wA.stayAngle + 5.0 * M_PI / 180.0);   // Pt.wristAngleL : stayAngle -> -5
                                                                                                // s :0 -> 1
            Pt.Kpp(8) = exp(-1.0*s*Kppp);
        }
    }

    return Pt;
}

void PathManager::getHitTime(HitAngle &Pt, int stateR, int stateL, float tHitR, float tHitL)
{
    //////////////////////////////////////////// R

    if (stateR == 2 || stateR == 3)
    {
        if (tHitR <= wristTimeR.liftTime)
        {
            Pt.isHitR = false;
        }
        else
        {
            Pt.isHitR = true;
        }
    }
    else
    {
        Pt.isHitR = false;
    }

    ////////////////////////////////////////// L

    if (stateL == 2 || stateL == 3)
    {
        if (tHitL <= wristTimeL.liftTime)
        {
            Pt.isHitL = false;
        }
        else
        {
            Pt.isHitL = true;
        }
    }
    else
    {
        Pt.isHitL = false;
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                                Waist                                       */
////////////////////////////////////////////////////////////////////////////////

vector<double> PathManager::cubicInterpolation(const vector<double> &q, const vector<double> &t)
{
    vector<double> m = {0.0, 0.0};
    vector<double> a(3, 0.0);

    for (int i = 0; i < 3; i++)
    {
        a[i] = (q[i + 1] - q[i]) / (t[i + 1] - t[i]);
    }
    double m1 = 0.5 * (a[0] + a[1]);
    double m2 = 0.5 * (a[1] + a[2]);

    // Monotone Cubic Interpolation 적용
    double alph, bet;
    if (q[1] == q[2])
    {
        m1 = 0;
        m2 = 0;
    }
    else
    {
        if((q[0] == q[1]) || (a[0] * a[1] < 0)){
            m1 = 0;
        }
        else if((q[2] == q[3]) || (a[1] * a[2] < 0)){
            m2 = 0;
        }
        alph = m1 / (q[2] - q[1]);
        bet = m2 / (q[2] - q[1]);

        double e = std::sqrt(std::pow(alph, 2) + std::pow(bet, 2));
        if (e > 3.0)
        {
            m1 = (3 * m1) / e;
            m2 = (3 * m2) / e;
        }
    }
    
    m[0] = m1;
    m[1] = m2;
    
    return m;
}

std::pair<double, vector<double>> PathManager::getNextQ0()
{
    VectorXd t_getNextQ0;     // getNextQ0() 함수 안에서 사용할 시간 벡터
    double dt = canManager.DTSECOND;
    vector<double> m_interpolation = {0.0, 0.0};
    double q0_t2 = 0.0, q0_t3;;

    VectorXd a(3);  // 기울기
    double avg_a;   // 기울기 평균
    double q0Min, q0Max;

    // 기울기 평균 + interpolation
    t_getNextQ0.resize(6);
    for (int i = 0; i < 6; i++)
    {
        if (i == 0)
        {
            t_getNextQ0(i) = t0;
        }
        else if (i == 1)
        {
            t_getNextQ0(i) = 0;
        }
        else if (lineData.rows() >= i-1)
        {
            t_getNextQ0(i) = t_getNextQ0(i-1) + lineData(i-2,0)*dt;
        }
        else
        {
            t_getNextQ0(i) = t_getNextQ0(i-1) + 1;
        }
    }

    // t1 -> t2
    for (int i = 0; i < 3; i++)
    {
        if (lineData.rows() > i+1)
        {
            a(i) = (lineData(i+1,1) - q0_t1) / (t_getNextQ0(i+2)-t_getNextQ0(1));
        }
        else
        {
            a(i) = (t_getNextQ0(i+1)-t_getNextQ0(1)) / (t_getNextQ0(i+2)-t_getNextQ0(1)) * a(i-1);
        }
    }

    avg_a = a.sum()/3.0;
    q0_t2 = avg_a*(t_getNextQ0(2)-t_getNextQ0(1)) + q0_t1;

    // 허리 범위 확인
    q0Min = lineData(1,2);
    q0Max = lineData(1,3);
    if (q0_t2 <= q0Min || q0_t2 >= q0Max)
    {
        q0_t2 = 0.5*q0Min + 0.5*q0Max;
    }

    // t2 -> t3
    if (lineData.rows() == 2)
    {
        q0_t3 = q0_t2;
    }
    else
    {
        for (int i = 0; i < 3; i++)
        {
            if (lineData.rows() > i+2)
            {
                a(i) = (lineData(i+2,1) - q0_t1) / (t_getNextQ0(i+3)-t_getNextQ0(2));
            }
            else
            {
                a(i) = (t_getNextQ0(i+2)-t_getNextQ0(2)) / (t_getNextQ0(i+3)-t_getNextQ0(2)) * a(i-1);
            }
        }

        // 허리 범위 확인
        avg_a = a.sum()/3.0;
        q0_t3 = avg_a*(t_getNextQ0(3)-t_getNextQ0(2)) + q0_t2;

        q0Min = lineData(2,2);
        q0Max = lineData(2,3);
        if (q0_t3 <= q0Min || q0_t3 >= q0Max)
        {
            q0_t3 = 0.5*q0Min + 0.5*q0Max;
        }
    }

    // 3차 보간법
    vector<double> y = {q0_t0, q0_t1, q0_t2, q0_t3};
    vector<double> x = {t_getNextQ0(0), t_getNextQ0(1), t_getNextQ0(2), t_getNextQ0(3)};
    m_interpolation = cubicInterpolation(y, x);

    return std::make_pair(q0_t2, m_interpolation);
}

void PathManager::makeWaistCoefficient()
{
    vector<double> m = {0.0, 0.0};
    double q0_t2;

    int n = lineData(0, 0); // 명령 개수
    double dt = canManager.DTSECOND;
    double t21 = n * dt;    // 전체 시간

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;

    if (lineData.rows() == 1)
    {
        // 마지막 줄 -> 허리 각 유지
        q0_t2 = q0_t1;
    }
    else
    {
        std::pair<double, vector<double>> output = getNextQ0();
        q0_t2 = output.first;
        m = output.second; 
    }

    A.resize(4, 4);
    b.resize(4, 1);

    A << 1, 0, 0, 0,
        1, t21, t21 * t21, t21 * t21 * t21,
        0, 1, 0, 0,
        0, 1, 2 * t21, 3 * t21 * t21;
    b << q0_t1, q0_t2, m[0], m[1];

    A_1 = A.inverse();
    waistCoefficient = A_1 * b;

    // 값 저장
    q0_t0 = q0_t1;
    q0_t1 = q0_t2;
    t0 = -1*t21;
}

double PathManager::getWaistAngle(int index)
{
    double dt = canManager.DTSECOND;
    double t = dt * index;

    return waistCoefficient(0, 0) + waistCoefficient(1, 0) * t + waistCoefficient(2, 0) * t * t + waistCoefficient(3, 0) * t * t * t;
}

////////////////////////////////////////////////////////////////////////////////
/*                                Solve IK                                    */
////////////////////////////////////////////////////////////////////////////////

float PathManager::getLength(double theta)
{
    PartLength partLength;
    double l1 = partLength.lowerArm;
    double l2 = partLength.stick;

    double l3 = l1 + l2 * cos(theta);

    double l4 = sqrt(l3 * l3 + ((l2 * sin(theta)) * (l2 * sin(theta))));

    return l4;
}

double PathManager::getTheta(float l1, double theta)
{
    PartLength partLength;

    float l2 = partLength.lowerArm + partLength.stick * cos(theta);

    double theta_m = acos(l2 / l1);

    return theta_m;
}

PathManager::Position PathManager::solveIK(VectorXd &q, double q0)
{
    Position nextP;

    nextP = trajectoryQueue.front();
    trajectoryQueue.pop();

    q.resize(9);
    q = IKFixedWaist(nextP.trajectoryR, nextP.trajectoryL, q0, nextP.wristAngleR, nextP.wristAngleL);
    
    return nextP;
}

VectorXd PathManager::IKFixedWaist(VectorXd pR, VectorXd pL, double theta0, double theta7, double theta8)
{
    VectorXd Qf;
    PartLength partLength;

    float XR = pR(0), YR = pR(1), ZR = pR(2);
    float XL = pL(0), YL = pL(1), ZL = pL(2);
    // float R1 = part_length.upperArm;
    // float R2 = part_length.lowerArm + part_length.stick;
    // float L1 = part_length.upperArm;
    // float L2 = part_length.lowerArm + part_length.stick;
    float R1 = partLength.upperArm;
    float R2 = getLength(theta7);
    float L1 = partLength.upperArm;
    float L2 = getLength(theta8);
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
        std::cout << "PR \n" << pR << "\n";
        std::cout << "PL \n" << pL << "\n";
        std::cout << "theta0 : " << theta0 * 180.0 / M_PI << "\n theta7 : " << theta7 * 180.0 / M_PI << "\n theta8 : " << theta8 * 180.0 / M_PI << "\n";
        std::cout << "IKFUN (q1) is not solved!!\n";
        std::cout << "theta1 : " << theta1 * 180.0 / M_PI << "\n";
        // state.main = Main::Error;
    }

    float theta02 = atan2(YL - shoulderYL, XL - shoulderXL);
    float theta2 = theta02 - theta0;

    if (theta2 < 30 * M_PI / 180.0 || theta2 > M_PI) // the2 범위 : 30deg ~ 180deg
    {
        std::cout << "PR \n" << pR << "\n";
        std::cout << "PL \n" << pL << "\n";
        std::cout << "theta0 : " << theta0 * 180.0 / M_PI << "\n theta7 : " << theta7 * 180.0 / M_PI << "\n theta8 : " << theta8 * 180.0 / M_PI << "\n";
        std::cout << "IKFUN (q2) is not solved!!\n";
        std::cout << "theta2 : " << theta2 * 180.0 / M_PI << "\n";
        // state.main = Main::Error;
    }

    float zeta = z0 - ZR;
    float r2 = (YR - shoulderYR) * (YR - shoulderYR) + (XR - shoulderXR) * (XR - shoulderXR); // r^2

    float x = zeta * zeta + r2 - R1 * R1 - R2 * R2;
    float y = sqrt(4.0 * R1 * R1 * R2 * R2 - x * x);

    float theta4 = atan2(y, x);

    if (theta4 < 0 || theta4 > 140.0 * M_PI / 180.0) // the4 범위 : 0deg ~ 120deg
    {
        std::cout << "PR \n" << pR << "\n";
        std::cout << "PL \n" << pL << "\n";
        std::cout << "theta0 : " << theta0 * 180.0 / M_PI << "\n theta7 : " << theta7 * 180.0 / M_PI << "\n theta8 : " << theta8 * 180.0 / M_PI << "\n";
        std::cout << "IKFUN (q4) is not solved!!\n";
        std::cout << "theta4 : " << theta4 * 180.0 / M_PI << "\n";
        // state.main = Main::Error;
    }

    float theta34 = atan2(sqrt(r2), zeta);
    float theta3 = theta34 - atan2(R2 * sin(theta4), R1 + R2 * cos(theta4));

    if (theta3 < -45.0 * M_PI / 180.0 || theta3 > 90.0 * M_PI / 180.0) // the3 범위 : -45deg ~ 90deg
    {
        std::cout << "PR \n" << pR << "\n";
        std::cout << "PL \n" << pL << "\n";
        std::cout << "theta0 : " << theta0 * 180.0 / M_PI << "\n theta7 : " << theta7 * 180.0 / M_PI << "\n theta8 : " << theta8 * 180.0 / M_PI << "\n";
        std::cout << "IKFUN (q3) is not solved!!\n";
        std::cout << "theta3 : " << theta3 * 180.0 / M_PI << "\n";
        // state.main = Main::Error;
    }

    zeta = z0 - ZL;
    r2 = (YL - shoulderYL) * (YL - shoulderYL) + (XL - shoulderXL) * (XL - shoulderXL); // r^2

    x = zeta * zeta + r2 - L1 * L1 - L2 * L2;
    y = sqrt(4.0 * L1 * L1 * L2 * L2 - x * x);

    float theta6 = atan2(y, x);

    if (theta6 < 0 || theta6 > 140.0 * M_PI / 180.0) // the6 범위 : 0deg ~ 120deg
    {
        std::cout << "PR \n" << pR << "\n";
        std::cout << "PL \n" << pL << "\n";
        std::cout << "theta0 : " << theta0 * 180.0 / M_PI << "\n theta7 : " << theta7 * 180.0 / M_PI << "\n theta8 : " << theta8 * 180.0 / M_PI << "\n";
        std::cout << "IKFUN (q6) is not solved!!\n";
        std::cout << "theta6 : " << theta6 * 180.0 / M_PI << "\n";
        // state.main = Main::Error;
    }

    float theta56 = atan2(sqrt(r2), zeta);
    float theta5 = theta56 - atan2(L2 * sin(theta6), L1 + L2 * cos(theta6));

    if (theta5 < -45.0 * M_PI / 180.0 || theta5 > 90.0 * M_PI / 180.0) // the5 범위 : -45deg ~ 90deg
    {
        std::cout << "PR \n" << pR << "\n";
        std::cout << "PL \n" << pL << "\n";
        std::cout << "theta0 : " << theta0 * 180.0 / M_PI << "\n theta7 : " << theta7 * 180.0 / M_PI << "\n theta8 : " << theta8 * 180.0 / M_PI << "\n";
        std::cout << "IKFUN (q5) is not solved!!\n";
        std::cout << "theta5 : " << theta5 * 180.0 / M_PI << "\n";
        // state.main = Main::Error;
    }

    theta4 -= getTheta(R2, theta7);
    theta6 -= getTheta(L2, theta8);

    Qf.resize(9);
    Qf << theta0, theta1, theta2, theta3, theta4, theta5, theta6, theta7, theta8;

    return Qf;
}

////////////////////////////////////////////////////////////////////////////////
/*                           Push Command Buffer                              */
////////////////////////////////////////////////////////////////////////////////

void PathManager::pushCommandBuffer(VectorXd Qi, VectorXd Kpp, bool isHitR, bool isHitL)
{
    for (auto &entry : motors)
    {
        int can_id = canManager.motorMapping[entry.first];

        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        {
            TMotorData newData;
            newData.position = tMotor->jointAngleToMotorPosition(Qi[can_id]);
            newData.mode = tMotor->Position;

            if (can_id == 0)
            {
                float alpha = 0.7;
                float diff = alpha*preDiff + (1 - alpha)*std::abs(newData.position - prevWaistPos);
                prevWaistPos = newData.position; 
                newData.is_brake = (diff < 0.01 * M_PI / 180.0) ? 1 : 0;
                preDiff = diff;

                fun.appendToCSV_DATA("brake input", newData.position, newData.is_brake, diff);
            }
            else
            {
                newData.is_brake = 0;
            }

            tMotor->commandBuffer.push(newData);

            tMotor->finalMotorPosition = newData.position;
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
        {
            // 1ms 로 동작 (이인우)
            for (int i = 0; i < 5; i++)
            {
                MaxonData newData;
                float Qi1ms = ((i+1)*Qi[can_id] + (4-i)*maxonMotor->pre_q)/5.0;
                newData.position = maxonMotor->jointAngleToMotorPosition(Qi1ms);
                if (MaxonMode == "CST")
                {
                    newData.mode = maxonMotor->CST;
                    newData.kp = Kpp[can_id] * Kp;
                    newData.kd = Kd;
                }
                else
                {
                    newData.mode = maxonMotor->CSP;
                    newData.kp = 0;
                    newData.kd = 0;
                }

                newData.isHitR = isHitR;
                newData.isHitL = isHitL;

                // fun.appendToCSV_DATA("isHit", isHitR, isHitL, 0);
                
                maxonMotor->commandBuffer.push(newData);

                maxonMotor->finalMotorPosition = newData.position;
            }
            maxonMotor->pre_q = Qi[can_id];
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                           Predict Collision                                */
////////////////////////////////////////////////////////////////////////////////

bool PathManager::predictCollision(MatrixXd measureMatrix)
{
    VectorXd measureTime = measureMatrix.col(8);
    VectorXd measureIntensityR = measureMatrix.col(4);
    VectorXd measureIntensityL = measureMatrix.col(5);

    MatrixXd state = measureState;

    // 충돌 예측을 위한 악보 (뒤쪽 목표위치 없는 부분 짜르기)
    int endIndex = findDetectionRange(measureMatrix);

    // 충돌 예측
    bool isColli = false;
    double stepSize = 5;
    for (int i = 0; i < endIndex-1; i++)
    {
        MatrixXd tmpMatrix = measureMatrix.block(i,0,measureMatrix.rows()-i,measureMatrix.cols());

        // position
        pair<VectorXd, VectorXd> initialPosition, finalPosition;
        VectorXd initialPositionR(3);
        VectorXd initialPositionL(3);
        VectorXd finalPositionR(3);
        VectorXd finalPositionL(3);
        // VectorXd initialWristAngle(2);
        // VectorXd finalWristAngle(2);

        // parse
        MatrixXd nextState = parseMeasurePC(tmpMatrix, state);
        state = nextState;

        // position
        initialPosition = getTargetPosition(initialInstrumentPC);
        finalPosition = getTargetPosition(finalInstrumentPC);

        initialPositionR << initialPosition.first(0), initialPosition.first(1), initialPosition.first(2);
        initialPositionL << initialPosition.first(3), initialPosition.first(4), initialPosition.first(5);
        finalPositionR << finalPosition.first(0), finalPosition.first(1), finalPosition.first(2);
        finalPositionL << finalPosition.first(3), finalPosition.first(4), finalPosition.first(5);

        // // 타격 시 손목 각도
        // initialWristAngle = initialPosition.second;
        // finalWristAngle = finalPosition.second;

        double dt = (line_t2PC - line_t1PC)/stepSize;

        for (int j = 0; j < stepSize+1; j++)
        {
            double tR = dt * j + line_t1PC - initialTimeRPC;
            double tL = dt * j + line_t1PC - initialTimeLPC;

            double sR = timeScaling(0.0, finalTimeRPC - initialTimeRPC, tR);
            double sL = timeScaling(0.0, finalTimeLPC - initialTimeLPC, tL);
            
            // task space 경로
            VectorXd PR = makePath(initialPositionR, finalPositionR, sR);
            VectorXd PL = makePath(initialPositionL, finalPositionL, sL);

            // // IK 풀기 위한 손목 각도
            // double wristAngleR = tR*(finalWristAngle(0) - initialWristAngle(0))/(finalTimeR - initialTimeR) + initialWristAngle(0);
            // double wristAngleL = tL*(finalWristAngle(1) - initialWristAngle(1))/(finalTimeL - initialTimeL) + initialWristAngle(1);

            double Tr = 1.0, hitR, hitL;
            if (measureTime(i+1) - measureTime(i) < 0.5)
            {
                Tr = (measureTime(i+1) - measureTime(i))/0.5;
            }
            
            if (measureIntensityR(i+1) == 0)
            {
                hitR = 10.0 * M_PI / 180.0;
            }
            else
            {
                hitR = measureIntensityR(i+1)*Tr*15.0*sin(M_PI*j/stepSize) * M_PI / 180.0;
            }

            if (measureIntensityL(i+1) == 0)
            {
                hitL = 10.0 * M_PI / 180.0;
            }
            else
            {
                hitL = measureIntensityL(i+1)*Tr*15.0*sin(M_PI*j/stepSize) * M_PI / 180.0;
            }

            if (checkTable(PR, PL, hitR, hitL))
            {
                // std::cout << "\n 충돌이 예측됨 \n";
                // std::cout << "\n R :" << PR.transpose() << ", L :" << PL.transpose() << "\n";
                // std::cout << "\n t :" << (measureTime(i+1) - measureTime(i))*j/stepSize + measureTime(i) << "\n";

                // return true;
                isColli = true;

                if (i == 0)
                {
                    colli_debug[0] = 1;
                    std::string fileName = "colliC";
                    fun.appendToCSV_DATA(fileName, colli_debug[0], hitR, hitL);
                    fileName = "colliR";
                    fun.appendToCSV_DATA(fileName, PR(0), PR(1), PR(2));
                    fileName = "colliL";
                    fun.appendToCSV_DATA(fileName, PL(0), PL(1), PL(2));
                }
            }
            else
            {
                if (i == 0)
                {
                    colli_debug[0] = 0;
                    std::string fileName = "colliC";
                    fun.appendToCSV_DATA(fileName, colli_debug[0], hitR, hitL);
                    fileName = "colliR";
                    fun.appendToCSV_DATA(fileName, PR(0), PR(1), PR(2));
                    fileName = "colliL";
                    fun.appendToCSV_DATA(fileName, PL(0), PL(1), PL(2));
                }
            }
        }

        if (isColli)
        {
            return true;
        }
    }

    return false;
}

int PathManager::findDetectionRange(MatrixXd measureMatrix)
{
    // 뒤쪽 목표위치 없는 부분 짜르기
    // endIndex 까지 탐색
    VectorXd measureTime = measureMatrix.col(8);
    VectorXd measureInstrumentR = measureMatrix.col(2);
    VectorXd measureInstrumentL = measureMatrix.col(3);

    // 충돌 예측을 위한 악보 (뒤쪽 목표위치 없는 부분 짜르기)
    bool endR = false, endL = false;
    int endIndex = measureTime.rows();
    double hitDetectionThreshold = 1.2 * 100.0 / bpmOfScore; // 일단 이렇게 하면 1줄만 읽는 일 없음

    for (int i = 0; i < measureTime.rows(); i++)
    {
        if (measureInstrumentR(measureTime.rows() - 1 - i) != 0)
        {
            endR = true;
        }

        if (!endR)
        {
            if (round(10000 * hitDetectionThreshold) < round(10000 * (measureTime(measureTime.rows() - 1) - measureTime(measureTime.rows() - 1 - i))))
            {
                endR = true;
            }
        }

        if (measureInstrumentL(measureTime.rows() - 1 - i) != 0)
        {
            endL = true;
        }

        if (!endL)
        {
            if (round(10000 * hitDetectionThreshold) < round(10000 * (measureTime(measureTime.rows() - 1) - measureTime(measureTime.rows() - 1 - i))))
            {
                endL = true;
            }
        }

        if (endR && endL)
        {
            endIndex = measureTime.rows() - i;
            break;
        }
    }

    return endIndex;
}

MatrixXd PathManager::parseMeasurePC(MatrixXd &measureMatrix, MatrixXd &state)
{
    VectorXd measureTime = measureMatrix.col(8);
    VectorXd measureInstrumentR = measureMatrix.col(2);
    VectorXd measureInstrumentL = measureMatrix.col(3);

    MatrixXd nextState(2, 3);

    // parsing data
    line_t1PC = measureMatrix(0, 8);
    line_t2PC = measureMatrix(1, 8);

    // std::cout << "\n /// t1 -> t2 : " << line_t1 << " -> " << line_t2 << " = " << line_t2 - line_t1 <<  "\n";

    // std::cout << "\n /// R ///";
    pair<VectorXd, VectorXd> dataR = parseOneArm(measureTime, measureInstrumentR, state.row(0));
    // std::cout << "\n /// L ///";
    pair<VectorXd, VectorXd> dataL = parseOneArm(measureTime, measureInstrumentL, state.row(1));

    // nextState 저장
    nextState.block(0, 0, 1, 3) = dataR.second.transpose();
    nextState.block(1, 0, 1, 3) = dataL.second.transpose();

    // 악기
    initialInstrumentPC << dataR.first.block(1, 0, 9, 1), dataL.first.block(1, 0, 9, 1);
    finalInstrumentPC << dataR.first.block(11, 0, 9, 1), dataL.first.block(11, 0, 9, 1);
    
    // 궤적 시간
    initialTimeRPC = dataR.first(0);
    initialTimeLPC = dataL.first(0);
    finalTimeRPC = dataR.first(10);
    finalTimeLPC = dataL.first(10);

    return nextState;
}

bool PathManager::checkTable(VectorXd PR, VectorXd PL, double hitR, double hitL)
{
    std::ifstream tableFile;
    std::string tablePath = "/home/shy/DrumRobot/include/table/";    // 테이블 위치

    int PR_index[3] = {0};
    int PL_index[3] = {0};
    int WR_index = 0, WL_index = 0;
    double range[2][4] = {{-0.35, 0.50, 0.60, 0.0}, {0.50, 0.75, 1.10, 30.0*M_PI/180.0}};
    double resolution[4] = {14, 4, 8, 10};

    // 인덱스 공간으로 변환
    for (int i = 0; i < 4; i++)
    {
        if (i < 3)
        {
            PR_index[i] = round(resolution[i]*(PR(i) - range[0][i])/(range[1][i] - range[0][i]));
            PL_index[i] = round(resolution[i]*(PL(i) - range[0][i])/(range[1][i] - range[0][i]));

            if (PR_index[i] > resolution[i])
            {
                PR_index[i] = (int)(resolution[i]);
            }

            if (PL_index[i] > resolution[i])
            {
                PL_index[i] = (int)(resolution[i]);
            }
        }
        else
        {
            WR_index = round(resolution[i]*(hitR - range[0][i])/(range[1][i] - range[0][i]));
            WL_index = round(resolution[i]*(hitL - range[0][i])/(range[1][i] - range[0][i]));

            if (WR_index > resolution[i])
            {
                WR_index = (int)(resolution[i]);
            }

            if (WL_index > resolution[i])
            {
                WL_index = (int)(resolution[i]);
            }
        }
    }

    // std::cout << "\n W index : " << WR_index << ", " << WL_index << "\n";
    // std::cout << "\n X index : " << PR_index[0] << ", " << PL_index[0] << "\n";
    // std::cout << "\n Y index : " << PR_index[1] << ", " << PL_index[1] << "\n";
    // std::cout << "\n Z index : " << PR_index[2] << ", " << PL_index[2] << "\n";

    // 테이블 확인
    std::string fileName = tablePath + "TABLE_" + std::to_string(WR_index) + "_" + std::to_string(WL_index) +".txt";
    tableFile.open(fileName); // 파일 열기

    if (tableFile.is_open())
    {
        string row;
        int readingLine = (resolution[2] * (resolution[0] + 1) + 1) * PR_index[1] + (resolution[0] + 1) * PR_index[2] + PR_index[0] + 1;    // 읽어야 할 줄 수

        // std::cout << "\n readingLine : " << readingLine << "\n";

        for (int j = 0; j < readingLine; j++)
        {
            getline(tableFile, row);
        }

        istringstream iss(row);
        string item;

        for (int j = 0; j < PL_index[2]+1; j++)
        {
            getline(iss, item, '\t');
        }

        item = trimWhitespace(item);

        char hex1 = item.at(2*PL_index[0] + 1);
        char hex2 = item.at(2*PL_index[0]);

        tableFile.close(); // 파일 닫기

        return hex2TableData(hex1, hex2, PL_index[1]);
    }
    else
    {
        std::cout << "\n table file open error \n";
        std::cout << fileName;
    }

    return false;
}

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

bool PathManager::hex2TableData(char hex1, char hex2, int index)
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

////////////////////////////////////////////////////////////////////////////////
/*                            Avoid Collision                                 */
////////////////////////////////////////////////////////////////////////////////

bool PathManager::modifyMeasure(MatrixXd &measureMatrix, int priority)
{
    // 주어진 방법으로 회피되면 measureMatrix를 바꾸고 True 반환

    std::string method = modificationMethods[priority];
    MatrixXd modifedMatrix = measureMatrix;
    bool modificationSuccess = true;
    int nModification = 0;  // 주어진 방법을 사용한 횟수

    while (modificationSuccess)
    {
        if (method == "Crash")
        {
            modificationSuccess = modifyCrash(modifedMatrix, nModification);     // 주어진 방법으로 수정하면 True 반환
        }
        else if (method == "Arm")
        {
            modificationSuccess = modifyArm(modifedMatrix, nModification);     // 주어진 방법으로 수정하면 True 반환
        }
        else if (method == "WaitAndMove")
        {
            modificationSuccess = waitAndMove(modifedMatrix, nModification);     // 주어진 방법으로 수정하면 True 반환
        }
        else if (method == "MoveAndWait")
        {
            modificationSuccess = moveAndWait(modifedMatrix, nModification);     // 주어진 방법으로 수정하면 True 반환
        }
        else if (method == "Delete")
        {
            modificationSuccess = deleteInst(modifedMatrix, nModification);     // 주어진 방법으로 수정하면 True 반환
        }
        else
        {
            modificationSuccess = false;
        }

        // std::cout << "\n ////////////// modify Measure : " << method << "\n";
        // std::cout << modifedMatrix;
        // std::cout << "\n ////////////// \n";

        if (!predictCollision(modifedMatrix))   // 충돌 예측
        {
            // std::cout << "\n 성공 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
            
            measureMatrix = modifedMatrix;
            return true;
        }
        else
        {
            // std::cout << "\n 실패 ㅜㅜ \n";
            modifedMatrix = measureMatrix;
        }

        nModification++;
    }
    return false;
}

pair<int, int> PathManager::findModificationRange(VectorXd t, VectorXd instR, VectorXd instL)
{
    // 수정하면 안되는 부분 제외
    // detectLine 부터 수정 가능
    double hitDetectionThreshold = 1.2 * 100.0 / bpmOfScore; // 일단 이렇게 하면 1줄만 읽는 일 없음

    int detectLineR = 1;
    for (int i = 1; i < t.rows(); i++)
    {
        if (round(10000 * hitDetectionThreshold) < round(10000 * (t(i) - t(0))))
        {
            break;
        }

        if (instR(i) != 0)
        {
            detectLineR = i + 1;
            break;
        }
    }

    int detectLineL = 1;
    for (int i = 1; i < t.rows(); i++)
    {
        if (round(10000 * hitDetectionThreshold) < round(10000 * (t(i) - t(0))))
        {
            break;
        }

        if (instL(i) != 0)
        {
            detectLineL = i + 1;
            break;
        }
    }

    return make_pair(detectLineR, detectLineL);
}

bool PathManager::modifyCrash(MatrixXd &measureMatrix, int num)
{
    // 주어진 방법으로 수정하면 True 반환
    VectorXd t = measureMatrix.col(8);
    VectorXd instR = measureMatrix.col(2);
    VectorXd instL = measureMatrix.col(3);

    // 수정하면 안되는 부분 제외
    pair<int, int> detectLine = findModificationRange(t, instR, instL);

    int detectLineR = detectLine.first;
    int detectLineL = detectLine.second;

    // Modify Crash
    int cnt = 0;

    for (int i = detectLineR; i < t.rows(); i++)
    {
        if (instR(i) == 7)
        {
            if (cnt == num)
            {
                measureMatrix(i, 2) = 8;
                return true;
            }
            cnt++;
        }
        else if (instR(i) == 8)
        {
            if (cnt == num)
            {
                measureMatrix(i, 2) = 7;
                return true;
            }
            cnt++;
        }
    }

    for (int i = detectLineL; i < t.rows(); i++)
    {
        if (instL(i) == 7)
        {
            if (cnt == num)
            {
                measureMatrix(i, 3) = 8;
                return true;
            }
            cnt++;
        }
        else if (instL(i) == 8)
        {
            if (cnt == num)
            {
                measureMatrix(i, 3) = 7;
                return true;
            }
            cnt++;
        }
    }

    return false;
}

bool PathManager::modifyArm(MatrixXd &measureMatrix, int num)
{
    // 주어진 방법으로 수정하면 True 반환
    VectorXd t = measureMatrix.col(8);
    VectorXd instR = measureMatrix.col(2);
    VectorXd instL = measureMatrix.col(3);

    // 수정하면 안되는 부분 제외
    pair<int, int> detectLine = findModificationRange(t, instR, instL);

    int detectLineR = detectLine.first;
    int detectLineL = detectLine.second;

    // Modify Arm
    int cnt = 0;
    int maxDetectLine = 1;

    if (detectLineR > detectLineL)
    {
        maxDetectLine = detectLineR;
    }
    else
    {
        maxDetectLine = detectLineL;
    }

    for (int i = maxDetectLine; i < t.rows(); i++)
    {
        if ((instR(i) != 0) && (instL(i) != 0))
        {
            if (cnt == num)
            {
                int tmp = measureMatrix(i, 2);
                measureMatrix(i, 2) = measureMatrix(i, 3);
                measureMatrix(i, 3) = tmp;
                
                tmp = measureMatrix(i, 4);
                measureMatrix(i, 4) = measureMatrix(i, 5);
                measureMatrix(i, 5) = tmp;
                
                return true;
            }
            cnt++;
        }
    }

    return false;
}

bool PathManager::waitAndMove(MatrixXd &measureMatrix, int num)
{
    // 주어진 방법으로 수정하면 True 반환
    VectorXd t = measureMatrix.col(8);
    VectorXd instR = measureMatrix.col(2);
    VectorXd instL = measureMatrix.col(3);

    // 수정하면 안되는 부분 제외
    pair<int, int> detectLine = findModificationRange(t, instR, instL);

    int detectLineR = detectLine.first;
    int detectLineL = detectLine.second;

    // Wait and Move
    int cnt = 0;

    bool isStart = false;
    int startInst, startIndex;
    for (int i = detectLineL-1; i < t.rows(); i++)
    {
        if (instL(i) != 0)
        {
            if (isStart)
            {
                if (startInst != instL(i))
                {
                    for (int j = 1; j < i-startIndex; j++)
                    {
                        if (cnt == num)
                        {
                            measureMatrix(startIndex+j, 3) = startInst;
                            return true;
                        }
                        cnt++;
                    }
                }
                
                startIndex = i;
                startInst = instL(i);
            }
            else
            {
                isStart = true;
                startIndex = i;
                startInst = instL(i);
            }
        }
    }

    isStart = false;
    for (int i = detectLineR-1; i < t.rows(); i++)
    {
        if (instR(i) != 0)
        {
            if (isStart)
            {
                if (startInst != instR(i))
                {
                    for (int j = 1; j < i-startIndex; j++)
                    {
                        if (cnt == num)
                        {
                            measureMatrix(startIndex+j, 2) = startInst;
                            return true;
                        }
                        cnt++;
                    }
                }

                startIndex = i;
                startInst = instR(i);
            }
            else
            {
                isStart = true;
                startIndex = i;
                startInst = instR(i);
            }
        }
    }

    return false;
}

bool PathManager::moveAndWait(MatrixXd &measureMatrix, int num)
{
    // 주어진 방법으로 수정하면 True 반환
    VectorXd t = measureMatrix.col(8);
    VectorXd instR = measureMatrix.col(2);
    VectorXd instL = measureMatrix.col(3);

    // 수정하면 안되는 부분 제외
    pair<int, int> detectLine = findModificationRange(t, instR, instL);

    int detectLineR = detectLine.first;
    int detectLineL = detectLine.second;

    // Move and Wait
    int cnt = 0;

    bool isStart = false;
    int startInst, endInst, startIndex;
    for (int i = detectLineL-1; i < t.rows(); i++)
    {
        if (instL(i) != 0)
        {
            if (isStart)
            {
                endInst = instL(i);
                if (endInst != startInst)
                {
                    for (int j = 1; j < i-startIndex; j++)
                    {
                        if (cnt == num)
                        {
                            measureMatrix(i-j, 3) = endInst;
                            return true;
                        }
                        cnt++;
                    }
                }
                
                startIndex = i;
                startInst = instL(i);
            }
            else
            {
                isStart = true;
                startIndex = i;
                startInst = instL(i);
            }
        }
    }

    isStart = false;
    for (int i = detectLineR-1; i < t.rows(); i++)
    {
        if (instR(i) != 0)
        {
            if (isStart)
            {
                endInst = instR(i);
                if (endInst != startInst)
                {
                    for (int j = 1; j < i-startIndex; j++)
                    {
                        if (cnt == num)
                        {
                            measureMatrix(i-j, 2) = endInst;
                            return true;
                        }
                        cnt++;
                    }
                }
                
                startIndex = i;
                startInst = instR(i);
            }
            else
            {
                isStart = true;
                startIndex = i;
                startInst = instR(i);
            }
        }
    }

    return false;
}

bool PathManager::deleteInst(MatrixXd &measureMatrix, int num)
{
    // 주어진 방법으로 수정하면 True 반환
    VectorXd t = measureMatrix.col(8);
    VectorXd instR = measureMatrix.col(2);
    VectorXd instL = measureMatrix.col(3);

    // 수정하면 안되는 부분 제외
    pair<int, int> detectLine = findModificationRange(t, instR, instL);

    int detectLineR = detectLine.first;
    int detectLineL = detectLine.second;

    // Move and Wait
    int cnt = 0;

    for (int i = detectLineR; i < t.rows(); i++)
    {
        if (instR(i) != 0)
        {
            if (cnt == num)
            {
                measureMatrix(i, 2) = 0;
                measureMatrix(i, 4) = 0;
                return true;
            }
            cnt++;
        }
    }

    for (int i = detectLineL; i < t.rows(); i++)
    {
        if (instL(i) != 0)
        {
            if (cnt == num)
            {
                measureMatrix(i, 3) = 0;
                measureMatrix(i, 5) = 0;
                return true;
            }
            cnt++;
        }
    }

    return false;
}
