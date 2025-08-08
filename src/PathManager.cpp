#include "../include/managers/PathManager.hpp" // 적절한 경로로 변경하세요.


PathManager::PathManager(State &stateRef,
                         CanManager &canManagerRef,
                         std::map<std::string, std::shared_ptr<GenericMotor>> &motorsRef,
                         USBIO &usbioRef,
                         Functions &funRef)
    : state(stateRef), canManager(canManagerRef), motors(motorsRef), usbio(usbioRef), fun(funRef)
{
}

// Public

void PathManager::initPathManager()
{
    setDrumCoordinate();
    setWristAngleOnImpact();
    setAddStanceAngle();
}

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

    // Q1 : finalMotorPosition -> 마지막 명령값에서 이어서 생성
    Q1 = getFinalMotorPosition();

    // Q2 : 정해진 위치
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
    else
    {
        std::cout << "Flag Error !\n";
        return;
    }

    // 사다리꼴 속도 프로파일 생성을 위한 가속도
    const float accMax = 100.0; // rad/s^2
    
    // 사다리꼴 속도 프로파일 생성을 위한 최고속도
    VectorXd Vmax = VectorXd::Zero(12);
    Vmax = calVmax(Q1, Q2, accMax, T);

    // 출력
    for (int k = 0; k < 12; k++)
    {
        std::cout << "Q1[" << k << "] : " << Q1[k] * 180.0 / M_PI << " [deg] -> Q2[" << k << "] : " << Q2[k] * 180.0 / M_PI << " [deg]" << endl;
    }

    // 궤적 생성
    for (int k = 1; k <= n + stayN; ++k)
    {
        if (k > stayN)  // 이동 궤적 생성
        {
            float t = (k - stayN) * T / n;

            Qt = makeProfile(Q1, Q2, Vmax, accMax, t, T);

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
                    // 1ms 로 동작
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
        else    // 현재 위치 대기
        {
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
                    // 1ms 로 동작
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

void PathManager::initPlayStateValue()
{
    startOfPlay = false; // true 로 변경시키면 연주 시작
    endOfPlayCommand = false;

    lineOfScore = 0;        ///< 현재 악보 읽은 줄.

    measureStateR.resize(3);
    measureStateR = VectorXd::Zero(3);
    measureStateR(1) = 1.0; // 시작 위치 SN

    measureStateL.resize(3);
    measureStateL = VectorXd::Zero(3);
    measureStateL(1) = 1.0; // 시작 위치 SN

    roundSum = 0.0;
    roundSumHit = 0.0;

    waistParameterQueue = std::queue<waistParameter>();   // 큐 재선언해서 비우기

    hitState.resize(2, 3);
    hitState = MatrixXd::Zero(2, 3);

    prevLine = VectorXd::Zero(9);

    q0_t1 = readyAngle(0);
    q0_t0 = readyAngle(0);
    t0 = -1;
}

void PathManager::processLine(MatrixXd &measureMatrix)
{
    lineOfScore++;
    std::cout << "\n//////////////////////////////// Read Measure : " << lineOfScore << "\n";
    // std::cout << measureMatrix;
    // std::cout << "\n ////////////// \n";

    if (measureMatrix.rows() > 1)
    {
        // avoidCollision(measureMatrix);      // 충돌 회피
        genTrajectory(measureMatrix);  // 궤적 생성
    }

    if (lineOfScore > preCreatedLine)
    {
        solveIKandPushCommand();        // IK & 명령 생성
    }
}

// Private

////////////////////////////////////////////////////////////////////////////////
/*                               Initialization                               */
////////////////////////////////////////////////////////////////////////////////

void PathManager::setDrumCoordinate()
{
    ifstream inputFile("../include/managers/rT.txt");

    if (!inputFile.is_open())
    {
        cerr << "Failed to open the file."
             << "\n";
    }

    // Read data into a 2D vector
    MatrixXd instXYZ(6, 9);

    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 9; ++j)
        {
            inputFile >> instXYZ(i, j);
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
    Vector3d right_OHH;

    Vector3d left_S;
    Vector3d left_FT;
    Vector3d left_MT;
    Vector3d left_HT;
    Vector3d left_HH;
    Vector3d left_R;
    Vector3d left_RC;
    Vector3d left_LC;
    Vector3d left_OHH;

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
        right_OHH(i) = instXYZ(i, 8);

        left_S(i) = instXYZ(i + 3, 0);
        left_FT(i) = instXYZ(i + 3, 1);
        left_MT(i) = instXYZ(i + 3, 2);
        left_HT(i) = instXYZ(i + 3, 3);
        left_HH(i) = instXYZ(i + 3, 4);
        left_R(i) = instXYZ(i + 3, 5);
        left_RC(i) = instXYZ(i + 3, 6);
        left_LC(i) = instXYZ(i + 3, 7);
        left_OHH(i) = instXYZ(i + 3, 8);
    }

    drumCoordinateR << right_S, right_FT, right_MT, right_HT, right_HH, right_R, right_RC, right_LC, right_OHH;
    drumCoordinateL << left_S, left_FT, left_MT, left_HT, left_HH, left_R, left_RC, left_LC, left_OHH;
}

void PathManager::setWristAngleOnImpact()
{
    // 악기 별 타격 시 손목 각도
    wristAngleOnImpactR.resize(1, 9);
    wristAngleOnImpactL.resize(1, 9);

    //                          S                  FT                  MT                  HT                  HH                  R                   RC                 LC                Open HH
    wristAngleOnImpactR << 10.0*M_PI/180.0,   10.0*M_PI/180.0,     5.0*M_PI/180.0,     5.0*M_PI/180.0,    10.0*M_PI/180.0,    15.0*M_PI/180.0,    10.0*M_PI/180.0,   10.0*M_PI/180.0,  10.0*M_PI/180.0;
    wristAngleOnImpactL << 10.0*M_PI/180.0,   10.0*M_PI/180.0,     5.0*M_PI/180.0,     5.0*M_PI/180.0,    10.0*M_PI/180.0,    15.0*M_PI/180.0,    10.0*M_PI/180.0,   10.0*M_PI/180.0,  10.0*M_PI/180.0;
}

void PathManager::setAddStanceAngle()
{
    //////////////////////////////////////// Ready Angle
    setReadyAngle();

    //////////////////////////////////////// Home Angle
    homeAngle.resize(12);
    //              waist          R_arm1         L_arm1
    homeAngle << 10*M_PI/180.0,  90*M_PI/180.0,  90*M_PI/180.0,
    //              R_arm2         R_arm3         L_arm2
                0*M_PI/180.0,   90*M_PI/180.0,  0*M_PI/180.0,
    //              L_arm3         R_wrist        L_wrist
                90*M_PI/180.0, 75*M_PI/180.0, 75*M_PI/180.0,
    //          Test               R_foot         L_foot            
                0*M_PI/180.0,   0*M_PI/180.0,   0*M_PI/180.0;

    //////////////////////////////////////// Shutdown Angle
    shutdownAngle.resize(12);
        //              waist          R_arm1         L_arm1
    shutdownAngle << 0*M_PI/180.0, 135*M_PI/180.0, 45*M_PI/180.0,
    //                  R_arm2         R_arm3         L_arm2
                    0*M_PI/180.0,  20*M_PI/180.0,   0*M_PI/180.0,
    //                  L_arm3         R_wrist        L_wrist
                    20*M_PI/180.0,  90*M_PI/180.0,  90*M_PI/180.0,
    //          Test               R_foot         L_foot            
                    0,                 0,              0;
}

void PathManager::setReadyAngle()
{
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
    combined << wristAngleOnImpactR, MatrixXd::Zero(1, 9), MatrixXd::Zero(1, 9), wristAngleOnImpactL;
    MatrixXd defaultWristAngle = combined * instrumentVector;

    VectorXd waistParams = getWaistParams(pR, pL);
    bool printError = false;
    VectorXd sol = solveGeometricIK(pR, pL, 0.5 * (waistParams(0) + waistParams(1)), defaultWristAngle(0), defaultWristAngle(1), printError);

    for (int i = 0; i < 9; i++)
    {
        readyAngle(i) = sol(i);
    }

    elbowAngle eA;
    wristAngle wA;
    bassAngle bA;
    HHAngle hA;
    readyAngle(4) += eA.stayAngle;
    readyAngle(6) += eA.stayAngle;
    readyAngle(7) += wA.stayAngle;
    readyAngle(8) += wA.stayAngle;

    readyAngle(9) = 0;
    readyAngle(10) = bA.stayAngle;
    readyAngle(11) = hA.openAngle;
}

////////////////////////////////////////////////////////////////////////////////
/*                                 AddStance                                  */
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

VectorXd PathManager::getFinalMotorPosition()
{
    VectorXd Qf = VectorXd::Zero(12);

    // finalMotorPosition 가져오기
    // 마지막 명령값에서 이어서 생성
    for (auto &entry : motors)
    {
        int can_id = canManager.motorMapping[entry.first];

        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        {
            Qf(can_id) = tMotor->motorPositionToJointAngle(tMotor->finalMotorPosition);
        }
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
        {
            Qf(can_id) = maxonMotor->motorPositionToJointAngle(maxonMotor->finalMotorPosition);
        }
    }

    return Qf;
}

////////////////////////////////////////////////////////////////////////////////
/*                                    Play                                    */
////////////////////////////////////////////////////////////////////////////////

void PathManager::avoidCollision(MatrixXd &measureMatrix)
{
    if (detectCollision(measureMatrix))    // 충돌 예측
    {
        for (int priority = 0; priority < 5; priority++)    // 수정방법 중 우선순위 높은 것부터 시도
        {
            if (modifyMeasure(measureMatrix, priority))     // 주어진 방법으로 회피되면 measureMatrix를 바꾸고 True 반환
            {
                std::cout << measureMatrix;
                std::cout << "\n 충돌 회피 성공 \n";
                break;
            }
        }
    }
    else
    {
        // std::cout << "\n 충돌 안함 \n";
    }
}

void PathManager::genTrajectory(MatrixXd &measureMatrix)
{
    int n = genTaskSpaceTrajectory(measureMatrix);

    genHitTrajectory(measureMatrix, n);

    ///////////////////////////////////////////////////////////// 읽은 줄 삭제
    MatrixXd tmpMatrix(measureMatrix.rows() - 1, measureMatrix.cols());
    tmpMatrix = measureMatrix.block(1, 0, tmpMatrix.rows(), tmpMatrix.cols());
    measureMatrix.resize(tmpMatrix.rows(), tmpMatrix.cols());
    measureMatrix = tmpMatrix;
}

void PathManager::solveIKandPushCommand()
{
    std::vector<waistParameter> wPs = waistParamsQueueToVector();
    waistParameterQueue.pop();

    int n = wPs[0].n;   // 명령 개수

    MatrixXd waistCoefficient = makeWaistCoefficient(wPs);

    // 여기서 첫 접근 때 정지하기

    while(!startOfPlay) // 시작 신호 받을 때까지 대기
    {
        usleep(500);
    }

    for (int i = 0; i < n; i++)
    {
        double q0 = getWaistAngle(waistCoefficient, i);

        VectorXd q = getJointAngles(q0);

        pushCommandBuffer(q);

        // 데이터 기록
        for (int i = 0; i < 9; i++)
        {
            std::string fileName = "solveIK_q" + to_string(i);
            fun.appendToCSV_DATA(fileName, i, q(i), 0);
        }
    }

    if (waistParameterQueue.empty())
    {
        endOfPlayCommand = true;   // DrumRobot 에게 끝났음 알리기
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                       Play : Trajectory (Task Space)                       */
////////////////////////////////////////////////////////////////////////////////

int PathManager::genTaskSpaceTrajectory(MatrixXd &measureMatrix)
{
    double sR, sL, n;
    double dt = canManager.DTSECOND;

    // parse
    parsedData data = parseMeasure(measureMatrix, measureStateR, measureStateL);
    measureStateR = data.nextStateR;
    measureStateL = data.nextStateL;

    // 한 줄의 데이터 개수 (5ms 단위)
    n = (data.t2 - data.t1) / dt;
    roundSum += (int)(n * 10000) % 10000;
    if (roundSum >= 10000)
    {
        roundSum -= 10000;
        n++;
    }
    n = floor(n);

    for (int i = 0; i < n; i++)
    {
        Position Pt;
        double tR = dt * i + data.t1 - data.initialTimeR;
        double tL = dt * i + data.t1 - data.initialTimeL;

        sR = timeScaling(0.0, data.finalTimeR - data.initialTimeR, tR);
        sL = timeScaling(0.0, data.finalTimeL - data.initialTimeL, tL);
        
        // task space 경로
        Pt.trajectoryR = makePath(data.initialPositionR, data.finalPositionR, sR);
        Pt.trajectoryL = makePath(data.initialPositionL, data.finalPositionL, sL);

        // IK 풀기 위한 손목 각도
        Pt.wristAngleR = sR*(data.finalWristAngleR - data.initialWristAngleR) + data.initialWristAngleR;
        Pt.wristAngleL = sL*(data.finalWristAngleL - data.initialWristAngleL) + data.initialWristAngleL;

        trajectoryQueue.push(Pt);

        // // 데이터 저장
        // std::string fileName;
        // fileName = "Trajectory_R";
        // fun.appendToCSV_DATA(fileName, Pt.trajectoryR[0], Pt.trajectoryR[1], Pt.trajectoryR[2]);
        // fileName = "Trajectory_L";
        // fun.appendToCSV_DATA(fileName, Pt.trajectoryL[0], Pt.trajectoryL[1], Pt.trajectoryL[2]);
        // fileName = "S";
        // fun.appendToCSV_DATA(fileName, sR, sL, 0);

        if (i == 0)
        {
            // 명령 개수, 허리 범위, 최적화 각도 계산 및 저장
            VectorXd waistParams = getWaistParams(Pt.trajectoryR, Pt.trajectoryL);
            storeWaistParams(n, waistParams);
        }
    }

    return n;
}

PathManager::parsedData PathManager::parseMeasure(MatrixXd &measureMatrix, VectorXd &stateR, VectorXd &stateL)
{
    parsedData data;

    VectorXd measureTime = measureMatrix.col(8);
    VectorXd measureInstrumentR = measureMatrix.col(2);
    VectorXd measureInstrumentL = measureMatrix.col(3);
    VectorXd measureHihat = measureMatrix.col(7);

    data.t1 = measureMatrix(0, 8);
    data.t2 = measureMatrix(1, 8);

    // std::cout << "\n /// t1 -> t2 : " << data.t1 << " -> " << data.t2 << " : " << data.t2 - data.t1 <<  "\n";

    // std::cout << "\n /// R ///";
    pair<VectorXd, VectorXd> dataR = parseOneArm(measureTime, measureInstrumentR, measureHihat, stateR);
    // std::cout << "\n /// L ///";
    pair<VectorXd, VectorXd> dataL = parseOneArm(measureTime, measureInstrumentL, measureHihat, stateL);

    // state 업데이트
    data.nextStateR = dataR.second;
    data.nextStateL = dataL.second;

    // 시간
    data.initialTimeR = dataR.first(0);
    data.initialTimeL = dataL.first(0);

    data.finalTimeR = dataR.first(10);
    data.finalTimeL = dataL.first(10);

    // 악기
    VectorXd initialInstrumentR = dataR.first.block(1, 0, 9, 1);
    VectorXd initialInstrumentL = dataL.first.block(1, 0, 9, 1);

    VectorXd finalInstrumentR = dataR.first.block(11, 0, 9, 1);
    VectorXd finalInstrumentL = dataL.first.block(11, 0, 9, 1);

    pair<VectorXd, double> initialTagetR = getTargetPosition(initialInstrumentR, 'R');
    pair<VectorXd, double> initialTagetL = getTargetPosition(initialInstrumentL, 'L');

    pair<VectorXd, double> finalTagetR = getTargetPosition(finalInstrumentR, 'R');
    pair<VectorXd, double> finalTagetL = getTargetPosition(finalInstrumentL, 'L');

    // position
    data.initialPositionR = initialTagetR.first;
    data.initialPositionL = initialTagetL.first;

    data.finalPositionR = finalTagetR.first;
    data.finalPositionL = finalTagetL.first;

    // 타격 시 손목 각도
    data.initialWristAngleR = initialTagetR.second;
    data.initialWristAngleL = initialTagetL.second;
    
    data.finalWristAngleR = finalTagetR.second;
    data.finalWristAngleL = finalTagetL.second;

    return data;
}

pair<VectorXd, VectorXd> PathManager::parseOneArm(VectorXd &t, VectorXd &inst, VectorXd &hh, VectorXd &stateVector)
{
    map<int, int> instrumentMapping = {
        {1, 0}, {2, 1}, {3, 2}, {4, 3}, {5, 4}, {6, 5}, {7, 6}, {8, 7}, {11, 0}, {51, 0}, {61, 0}, {71, 0}, {81, 0}, {91, 0}, {9, 8}};
    //    S       FT      MT      HT      HH       R      RC      LC       S        S        S        S        S        S     Open HH

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
            detectInst = checkOpenHihat(inst(i), hh(i));

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

            initialInstNum = checkOpenHihat(inst(0), hh(0));
            finalInstNum = detectInst;

            initialT = t(0);
            finalT = detectTime;
        }
        // 다음 타격 감지 못함
        else
        {
            nextState = 1;

            initialInstNum = checkOpenHihat(inst(0), hh(0));
            finalInstNum = checkOpenHihat(inst(0), hh(0));

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

int PathManager::checkOpenHihat(int instNum, int isHH)
{
    if (instNum == 5)       // 하이햇인 경우
    {
        if (isHH == 0)  // 오픈
        {
            return 9;
        }
        else            // 클로즈
        {
            return instNum;
        }
    }
    else                // 하이햇이 아니면 그냥 반환
    {
        return instNum;
    }
}

pair<VectorXd, double> PathManager::getTargetPosition(VectorXd &inst, char RL)
{
    double angle = 0.0;
    VectorXd p(3);

    if (inst.sum() == 0)
    {
        std::cout << "Instrument Vector Error!! : " << inst << "\n";
    }

    if (RL == 'R' || RL == 'r')
    {
        MatrixXd productP = drumCoordinateR * inst;
        p = productP.block(0, 0, 3, 1);

        MatrixXd productA = wristAngleOnImpactR * inst;
        angle = productA(0, 0);
    }
    else if (RL == 'L' || RL == 'l')
    {
        MatrixXd productP = drumCoordinateL * inst;
        p = productP.block(0, 0, 3, 1);

        MatrixXd productA = wristAngleOnImpactL * inst;
        angle = productA(0, 0);
    }
    else
    {
        std::cout << "RL Error!! : " << RL << "\n";
    }

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

VectorXd PathManager::makePath(VectorXd &Pi, VectorXd &Pf, double s)
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

VectorXd PathManager::getWaistParams(VectorXd &pR, VectorXd &pL)
{
    std::vector<VectorXd> Qarr;
    int j = 0;      // 솔루션 개수
    VectorXd output(3);

    VectorXd W(2);
    W << 2, 1;
    double minCost = W.sum();
    double w = 0, cost = 0;
    int minIndex = 0;
    
    for (int i = 0; i < 1801; i++)
    {
        double theta0 = -0.5 * M_PI + M_PI / 1800.0 * i; // the0 범위 : -90deg ~ 90deg
        double theta7 = 0.0;
        double theta8 = 0.0;
        bool printError = false;

        VectorXd sol = solveGeometricIK(pR, pL, theta0, theta7, theta8, printError);

        if (sol(9) == 0.0)      // 해가 구해진 경우 (err == 0)
        {
            Qarr.push_back(sol);
            j++;
        }
    }

    if (j == 0)
    {
        std::cout << "IKFUN is not solved!! (Waist Range)\n";
        state.main = Main::Error;

        output(0) = 0;
        output(1) = 0;
        output(2) = 0;
    }
    else
    {
        VectorXd minSol = Qarr[0];
        VectorXd maxSol = Qarr[j-1];
        
        w = 2.0 * M_PI / abs(maxSol[0] - minSol[0]);

        for (int i = 0; i < j; i++)
        {
            VectorXd sol = Qarr[i];
            cost = W(0)*cos(sol[1] + sol[2]) + W(1)*cos(w*abs(sol[0] - minSol[0]));

            if (cost < minCost)
            {
                minCost = cost;
                minIndex = i;
            }
        }

        VectorXd optSol = Qarr[minIndex];
        
        output(0) = minSol[0];
        output(1) = maxSol[0];
        output(2) = optSol[0];
    }

    return output;
}

void PathManager::storeWaistParams(int n, VectorXd &waistParams)
{
    waistParameter wP;
    wP.n = n;
    wP.min_q0 = waistParams(0);
    wP.max_q0 = waistParams(1);
    wP.optimized_q0 = waistParams(2);

    waistParameterQueue.push(wP);
}

////////////////////////////////////////////////////////////////////////////////
/*                        Play : Trajectory (Hitting)                         */
////////////////////////////////////////////////////////////////////////////////

void PathManager::genHitTrajectory(MatrixXd &measureMatrix, int n)
{
    MatrixXd dividedMatrix;

    double dt = canManager.DTSECOND;
    double timeStep = 0.05;
    double lineT = measureMatrix(1,1);
    int repeat = static_cast<int>(round(lineT / timeStep)); // 한 줄 궤적 생성을 위한 반복 횟수
    int k = 0;

    dividedMatrix = divideMatrix(measureMatrix);

    double line_t1 = measureMatrix(0, 8);
    double line_t2 = measureMatrix(1, 8);

    // Bass 관련 변수
    bool bassHit = measureMatrix(0, 6);
    bool nextBaseHit = measureMatrix(1, 6);
    int bassState = getBassState(bassHit, nextBaseHit);
    bassTimeR = getBassTime(line_t1, line_t2);

    // Hihat 관련 변수
    bool HHclosed = measureMatrix(0, 7);
    int nextHHclosed = measureMatrix(1, 7);
    int HHstate = getHHstate(HHclosed, nextHHclosed);
    HHTimeL = getHHTime(line_t1, line_t2);
    
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
        }
        
        HitAngle Ht;
        float tHitR = dt * (i - (k-1)*n/repeat) + line_t1 - hitR_t1;
        float tHitL = dt * (i - (k-1)*n/repeat) + line_t1 - hitL_t1;

        // 각 관절에 더해줄 각도
        generateHit(tHitR, tHitL, Ht);

        // 양 발 모터 각도
        Ht.bass = makeBassAngle(i * dt, bassTimeR, bassState);
        Ht.hihat = makeHHAngle(i * dt, HHTimeL, HHstate, nextHHclosed);
        
        hitAngleQueue.push(Ht);

        // // 데이터 저장
        // std::string fileName;
        // fileName = "hitAngle";
        // fun.appendToCSV_DATA(fileName, Ht.bass, Ht.wristL, Ht.wristR);
        // fileName = "HtR";
        // fun.appendToCSV_DATA(fileName, tHitR, hitR_t2 - hitR_t1, 0);
        // fileName = "HtL";
        // fun.appendToCSV_DATA(fileName, tHitL, hitL_t2 - hitL_t1, 0);
        // fileName = "HHAngle.csv";
        // fun.appendToCSV_DATA(fileName, Ht.hihat, Ht.bass, 0);
    }
}

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
            if (i != 1)     // 첫 줄은 무조건 읽도록
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
            if (i != 1)     // 첫 줄은 무조건 읽도록
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

int PathManager::getBassState(bool bassHit, bool nextBaseHit)
{
    int state = 0;

    if(!bassHit && !nextBaseHit)
    {
        state = 0;
    }
    else if (bassHit && !nextBaseHit)
    {
        state = 1;
    }
    else if (!bassHit && nextBaseHit)
    {
        state = 2;
    }
    else if (bassHit && nextBaseHit)
    {
        state = 3;
    }

    return state;
}

int PathManager::getHHstate(bool HHclosed, bool nextHHclosed)
{
    int state = 0;

    if(!HHclosed && !nextHHclosed)
    {
        state = 0;
    }
    else if (HHclosed && !nextHHclosed)
    {
        state = 1;
    }
    else if (!HHclosed && nextHHclosed)
    {
        state = 2;
    }
    else if (HHclosed && nextHHclosed)
    {
        state = 3;
    }

    return state;
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

    wristTimeR = getWristTime(hitR_t1, hitR_t2, intensityR, stateR);
    wristTimeL = getWristTime(hitL_t1, hitL_t2, intensityL, stateL);

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
    elbowTime elbowTime;
    float T = t2 - t1; // 전체 타격 시간

    elbowTime.liftTime = std::max(0.5 * (T), T - 0.2);
    elbowTime.hitTime = T;
    
    return elbowTime;
}

PathManager::wristTime PathManager::getWristTime(float t1, float t2, int intensity, int state)
{
    wristTime wristTime;
    float T = t2 - t1;  // 전체 타격 시간

    // 타격 후 복귀 시간
    wristTime.releaseTime = std::min(0.2 * (T), 0.1);

    if (state == 2)
    {
        wristTime.liftTime = std::max(0.6 * (T), T - 0.2);      // 최고점에 도달하는 시간 
        wristTime.stayTime = 0.45 * T;                          // 상승하기 시작하는 시간
    }
    else
    {
        // state 1 or 3일 때 (복귀 모션 필요)
        // state 3일 때 시간이 0.3초 이하이면 전체 타격 시간의 절반을 기준으로 들고 내리는 궤적(stay 없음)
        if (T <= 0.3)
        {
            wristTime.liftTime = 0.5 * (T);
        }
        else
        {
            wristTime.liftTime = std::max(0.6 * (T), T - 0.2);
        }
        wristTime.stayTime = 0.5 * (T);
    }

    wristTime.hitTime = T;
    
    return wristTime;
}

PathManager::bassTime PathManager::getBassTime(float t1, float t2)
{
    bassTime bassTime;
    float T = t2 - t1;      // 전체 타격 시간
    
    bassTime.liftTime = std::max(0.6 * (T), T - 0.2);   // 최고점 시간, 전체 타격 시간의 60% , 타격 시간의 최대값은 0.2초
    bassTime.stayTime = 0.45 * (T);     // 복귀 시간, 전체 타격 시간의 45%
    
    bassTime.hitTime = T;

    return bassTime;
}

PathManager::HHTime PathManager::getHHTime(float t1, float t2)
{
    float T = t2 - t1;
    HHTime HHTime;

    HHTime.liftTime = 0.1 * T;
    HHTime.settlingTime = 0.9 * T;
    HHTime.hitTime = T;
    HHTime.splashTime = std::max(0.5*T, T - 0.1);       // 0.1초에 20도 이동이 안전. 그 이하는 모터 멈출 수 있음.

    return HHTime;
}

PathManager::elbowAngle PathManager::getElbowAngle(float t1, float t2, int intensity)
{
    elbowAngle elbowAngle;
    float T = (t2 - t1);        // 전체 타격 시간

    double intensityFactor;  // 1: 0%, 2: 0%, 3: 0%, 4: 90%, 5: 100%, 6: 110%, 7: 120%  

    if (intensity <= 3)
    {
        intensityFactor = 0;
    }
    else
    {
        intensityFactor = 0.1 * intensity + 0.5;  
    }

    if (T < 0.2)
    {
        // 0.2초보다 짧을 땐 안 들게
        elbowAngle.liftAngle = 0;
    }
    else if (T <= 0.5)
    {
        // 0.2초 ~ 0.5초에선 시간에 따라 선형적으로 줄여서 사용
        elbowAngle.liftAngle = (T) * (15 * M_PI / 180.0) / 0.5;
    }
    else
    {
        elbowAngle.liftAngle = 15 * M_PI / 180.0;
    }

    elbowAngle.liftAngle = elbowAngle.stayAngle + elbowAngle.liftAngle * intensityFactor;

    return elbowAngle;
}

PathManager::wristAngle PathManager::getWristAngle(float t1, float t2, int intensity)
{
    wristAngle wristAngle;
    float T = (t2 - t1);        // 타격 전체 시간
    double intensityFactor;
   
    if (intensity < 5)
    {
        intensityFactor = 0.25 * intensity - 0.25;  // 1: 0%, 2: 25%, 3: 50%, 4: 75%
    }
    else
    {
        intensityFactor = 0.1 * intensity + 0.5;  // 5: 100%, 6: 110%, 7: 120% 
    }

    // Lift Angle (최고점 각도) 계산, 최대 40도 * 세기
    T < 0.5 ? wristAngle.liftAngle = (80 * T) * M_PI / 180.0 : wristAngle.liftAngle = 40  * M_PI / 180.0;

    if (intensity == 1)
    {
        // intensity 1일 땐 아예 안들도록
        wristAngle.liftAngle = wristAngle.stayAngle;
    }
    else
    {
        wristAngle.liftAngle = wristAngle.stayAngle + wristAngle.liftAngle * intensityFactor;
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
        // Lift
        A.resize(4, 4);
        b.resize(4, 1);

        if(eA.liftAngle == eA.stayAngle)
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
        if (eA.liftAngle == eA.stayAngle)
        {
            sol2.resize(4, 1);
            sol2 << 0, 0, 0, 0;
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
        // Lift
        if (eA.liftAngle == eA.stayAngle)
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

        // Lift Angle = Stay Angle일 때(아주 작게 칠 때) -> 드는 궤적 없이 내리기만
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

double PathManager::makeBassAngle(double t, bassTime bt, int bassState)
{
    bassAngle bA;
    // Xl : lift 궤적 , Xh : hit 궤적
    double X0 = 0.0, Xp = 0.0, Xl = 0.0, Xh = 0.0;

    X0 = bA.stayAngle;          // 준비 각도
    Xp = bA.pressAngle;         // 최저점 각도

    // 타격 시간이 0.2초 이하일 때
    if (bt.hitTime <= 0.2)
    {
        if (bassState == 3)
        {
            // 짧은 시간에 연속으로 타격하는 경우 덜 들도록, 올라오는 궤적과 내려가는 궤적을 합쳐서 사용 (- 영역이라 가능)
            // 조정수 박사님 자료 BassAngle 부분 참고
            double temp_liftTime = bt.hitTime / 2;
            Xl = -0.5 * (Xp - X0) * (cos(M_PI * (t / bt.hitTime + 1)) - 1);

            if (t < temp_liftTime)
            {
                Xh = 0;
            }
            else 
            {
                Xh = -0.5 * (Xp - X0) * (cos(M_PI * (t - temp_liftTime) / (bt.hitTime - temp_liftTime)) - 1);
            }
        }

        // 시간이 짧을 땐 전체 시간을 다 써서 궤적 생성
        else if (bassState == 1)
        {
            // 복귀하는 궤적
            Xl = -0.5 * (Xp - X0) * (cos(M_PI * (t / bt.hitTime + 1)) - 1);
            Xh = 0;
        }
        else if (bassState == 2)
        {
            // 타격하는 궤적
            Xl = 0;
            Xh = -0.5 * (Xp - X0) * (cos(M_PI * t / bt.hitTime) - 1);
        }
        else
        {
            Xl = 0;
            Xh = 0;
        } 
    }

    // 0.2초 이상일 때 
    // StayTime 이전 -> 타격 후 들어올림
    else if (t < bt.stayTime)
    {
        if(bassState == 1 || bassState == 3)    
        {
            // 복귀하는 궤적
            Xl = -0.5 * (Xp - X0) * (cos(M_PI * (t / bt.stayTime + 1)) - 1);
            Xh = 0;
        }
        else
        {
            Xl = 0;
            Xh = 0;
        }
    }

    // LiftTime부터 HitTime까지 -> 타격
    else if (t > bt.liftTime && t <= bt.hitTime)
    {
        if (bassState == 2 || bassState == 3)
        {
            // 타격하는 궤적
            Xl = 0;
            Xh = -0.5 * (Xp - X0) * (cos(M_PI * (t - bt.liftTime) / (bt.hitTime - bt.liftTime)) - 1);
        }
        else
        {
            Xl = 0;
            Xh = 0;
        }
    }
    
    return X0 + Xl + Xh;        // 준비 각도 + 하강 각도 + 상승 각도
}

double PathManager::makeHHAngle(double t, HHTime ht, int HHstate, int nextHHclosed)
{
    // 1.open/closed, 2.splash 두 개의 하이햇 연주법을 구현함.
    // o/c 상태에 따라 악기 소리가 다름. 소리나는 도중에 상태가 변해도 소리가 변함. 
    // 정확한 소리를 내기 위해 타격 이전(settlingTime)에 상태를 바꾸고 타격 이후(liftTime)에도 그 상태를 유지함.
    // splash는 페달을 밟았다가 떼며 두 심벌이 부딪힘으로 소리내는 방식임.
    // nextHHclosed == 2 이면 splash 연주법임.

    HHAngle HA;

    // 각도 변경 원할 시 헤더파일에서 해당 각도 수정
    double X0 = HA.openAngle;       // Open Hihat : -3도
    double Xp = HA.closedAngle;     // Closed Hihat : -13도 

    double Xl = 0.0;

    if(HHstate == 0)
    {
        Xl = X0;
    }
    else if(ht.hitTime <= 0.2)       // 한 박의 시간이 0.2초 이하인 경우
    {
        if(HHstate == 1)
        {
            Xl = makecosineprofile(Xp, X0, 0, ht.hitTime, t);       // 전체 시간동안 궤적 생성
            /*Xl = makecosineprofile(Xp, X0, 0, 0.9*ht.hitTime, t);     // 타격 이전에 open/closed가 되도록 0.9T
            if(t >= 0.8*ht.hitTime)
            {
                Xl = X0;
            }*/
        }
        else if(nextHHclosed == 2)      // Hihat splash 궤적
        {
            if(t < ht.splashTime)
            {
                if(HHstate == 2)
                {
                    Xl = makecosineprofile(X0, Xp, 0, ht.hitTime, t);
                }
                else if(HHstate == 3)
                {
                    Xl = makecosineprofile(Xp, Xp - (Xp - X0) * ht.hitTime / 0.2, 0, ht.splashTime, t);     // 20도/0.1초 의 속도를 유지하기 위해서 시간에 따라 이동량 감소시킴.
                }
            }
            else
            {
                if(HHstate == 2)
                {
                    Xl = makecosineprofile(X0, Xp, 0, ht.hitTime, t);
                }
                else if(HHstate == 3)
                {
                    Xl = makecosineprofile(Xp - (Xp - X0) * ht.hitTime / 0.2, Xp, ht.splashTime, ht.hitTime, t);
                }
            }
        }
        else        // open/closed Hihat 궤적
        {
            if(HHstate == 2)
            {
                Xl = makecosineprofile(X0, Xp, 0, ht.hitTime, t);
                /*Xl = makecosineprofile(X0, Xp, 0, 0.9*ht.hitTime, t);
                if(t >= 0.9*ht.hitTime)
                {
                    Xl = Xp;
                }*/
            }
            else if(HHstate == 3)
            {                
                Xl = Xp;
            }
        }
    }
    // 한 박의 시간이 0.2초 초과인 경우
    else if(nextHHclosed == 2)      // Hi-Hat splash 궤적
    {
        if(t < ht.splashTime)
        {
            if(HHstate == 2)
            {
                Xl = X0;
            }
            else if(HHstate == 3)
            {
                Xl = makecosineprofile(Xp, X0, 0, ht.splashTime, t);
            }
        }
        else if(t >= ht.splashTime)
        {
            if(HHstate == 2)
            {
                Xl = makecosineprofile(X0, Xp, ht.splashTime, ht.hitTime, t);
            }
            else if(HHstate == 3)
            {
                Xl = makecosineprofile(X0, Xp, ht.splashTime, ht.hitTime, t);
            }
        }
    }
    else        // open/closed Hi-Hat 궤적
    {
        if(t < ht.liftTime)
        {
            if(HHstate == 1)
            {
                Xl = Xp;
            }
            else if(HHstate == 2)
            {
                Xl = X0;
            }
            else if(HHstate == 3)
            {                
                Xl = Xp;
            }
        }
        else if(t >= ht.liftTime && t < ht.settlingTime)
        {
            if(HHstate == 1)
            {
                Xl = makecosineprofile(Xp, X0, ht.liftTime, ht.settlingTime, t);
            }
            else if(HHstate == 2)
            {      
                Xl = makecosineprofile(X0, Xp, ht.liftTime, ht.settlingTime, t);
            }
            else if(HHstate == 3)
            {
                Xl = Xp;
            }
        }
        else if(t >= ht.settlingTime)
        {
            if(HHstate == 1)
            {
                Xl = X0;
            }
            else if(HHstate == 2)
            {
                Xl = Xp;
            }
            else if(HHstate == 3)
            {
                Xl = Xp;
            }
        }
    }

    return Xl;
}

double PathManager::makecosineprofile(double qi, double qf, double ti, double tf, double t)
{
    // ti <= t <= tf
    double A, B, w;

    A = (qi - qf) / 2;
    B = (qi + qf) / 2;
    w = M_PI / (tf - ti);

    return A * cos (w*(t - ti)) + B;
}

void PathManager::generateHit(float tHitR, float tHitL, HitAngle &Pt)
{
    // 각 관절에 더해줌
    Pt.elbowR = makeElbowAngle(tHitR, elbowTimeR, elbowCoefficientR);
    Pt.elbowL = makeElbowAngle(tHitL, elbowTimeL, elbowCoefficientL);
    Pt.wristR = makeWristAngle(tHitR, wristTimeR, wristCoefficientR);
    Pt.wristL = makeWristAngle(tHitL, wristTimeL, wristCoefficientL);
}

////////////////////////////////////////////////////////////////////////////////
/*                      Play : Solve IK and Push Command                      */
////////////////////////////////////////////////////////////////////////////////

std::vector<PathManager::waistParameter> PathManager::waistParamsQueueToVector()
{
    std::vector<waistParameter> wPs;
    int size = waistParameterQueue.size();

    for (int i = 0; i < size; i++)
    {
        waistParameter temp = waistParameterQueue.front();
        waistParameterQueue.pop();
        waistParameterQueue.push(temp);
        wPs.push_back(temp);
    }

    return wPs;
}

MatrixXd PathManager::makeWaistCoefficient(std::vector<waistParameter> &wPs)
{
    vector<double> m = {0.0, 0.0};
    double q0_t2;

    int n = wPs[0].n;           // 명령 개수
    double dt = canManager.DTSECOND;
    double t21 = n * dt;    // 전체 시간

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd waistCoefficient;

    if (wPs.size() == 1)
    {
        // 마지막 줄 -> 허리 각 유지
        q0_t2 = q0_t1;
    }
    else
    {
        std::pair<double, vector<double>> output = getNextQ0(wPs);
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

    return waistCoefficient;
}

std::pair<double, vector<double>> PathManager::getNextQ0(std::vector<waistParameter> &wPs)
{
    VectorXd t_getNextQ0;     // getNextQ0() 함수 안에서 사용할 시간 벡터
    double dt = canManager.DTSECOND;
    int wPsSize = wPs.size();
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
        else if (wPsSize >= i-1)
        {
            t_getNextQ0(i) = t_getNextQ0(i-1) + wPs[i-2].n*dt;
        }
        else
        {
            t_getNextQ0(i) = t_getNextQ0(i-1) + 1;
        }
    }

    // t1 -> t2
    for (int i = 0; i < 3; i++)
    {
        if (wPsSize > i+1)
        {
            a(i) = (wPs[1].optimized_q0 - q0_t1) / (t_getNextQ0(i+2)-t_getNextQ0(1));
        }
        else
        {
            a(i) = (t_getNextQ0(i+1)-t_getNextQ0(1)) / (t_getNextQ0(i+2)-t_getNextQ0(1)) * a(i-1);
        }
    }

    avg_a = a.sum()/3.0;
    q0_t2 = avg_a*(t_getNextQ0(2)-t_getNextQ0(1)) + q0_t1;

    // 허리 범위 확인
    q0Min = wPs[1].min_q0;
    q0Max = wPs[1].max_q0;
    if (q0_t2 <= q0Min || q0_t2 >= q0Max)
    {
        q0_t2 = 0.5*q0Min + 0.5*q0Max;
    }

    // t2 -> t3
    if (wPsSize == 2)
    {
        q0_t3 = q0_t2;
    }
    else
    {
        for (int i = 0; i < 3; i++)
        {
            if (wPsSize > i+2)
            {
                a(i) = (wPs[1].optimized_q0 - q0_t1) / (t_getNextQ0(i+3)-t_getNextQ0(2));
            }
            else
            {
                a(i) = (t_getNextQ0(i+2)-t_getNextQ0(2)) / (t_getNextQ0(i+3)-t_getNextQ0(2)) * a(i-1);
            }
        }

        // 허리 범위 확인
        avg_a = a.sum()/3.0;
        q0_t3 = avg_a*(t_getNextQ0(3)-t_getNextQ0(2)) + q0_t2;

        q0Min = wPs[2].min_q0;
        q0Max = wPs[2].max_q0;
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

double PathManager::getWaistAngle(MatrixXd &waistCoefficient, int index)
{
    double dt = canManager.DTSECOND;
    double t = dt * index;

    return waistCoefficient(0, 0) + waistCoefficient(1, 0) * t + waistCoefficient(2, 0) * t * t + waistCoefficient(3, 0) * t * t * t;
}

VectorXd PathManager::getJointAngles(double q0)
{
    VectorXd q(12);

    Position nextP;
    nextP = trajectoryQueue.front();
    trajectoryQueue.pop();

    bool printError = true;
    VectorXd sol = solveGeometricIK(nextP.trajectoryR, nextP.trajectoryL, q0, nextP.wristAngleR, nextP.wristAngleL, printError);

    for (int i = 0; i < 9; i++)
    {
        q(i) = sol(i);
    }

    HitAngle nextH;
    nextH = hitAngleQueue.front();
    hitAngleQueue.pop();

    q(3) += nextH.elbowR / 3.0;
    q(4) += nextH.elbowR;
    q(5) += nextH.elbowL / 3.0;
    q(6) += nextH.elbowL;
    q(7) += nextH.wristR;
    q(8) += nextH.wristL;
    q(9) = 0.0; // test maxon motor
    q(10) = nextH.bass;
    q(11) = nextH.hihat;

    return q;
}

void PathManager::pushCommandBuffer(VectorXd &Qi)
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

                // fun.appendToCSV_DATA("brake input", newData.position, newData.is_brake, diff);
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
            for (int i = 0; i < 5; i++)
            {
                MaxonData newData;
                float Qi1ms = ((i+1)*Qi[can_id] + (4-i)*maxonMotor->pre_q)/5.0;
                newData.position = maxonMotor->jointAngleToMotorPosition(Qi1ms);
                if (can_id == 10 || can_id == 11)
                {
                    // 발 모터는 항상 CSP mode
                    newData.mode = maxonMotor->CSP;
                    newData.kp = 0;
                    newData.kd = 0;
                }
                else if (MaxonMode == "CST")
                {
                    newData.mode = maxonMotor->CST;
                    newData.kp = Kp;
                    newData.kd = Kd;
                }
                else
                {
                    newData.mode = maxonMotor->CSP;
                    newData.kp = 0;
                    newData.kd = 0;
                }
                
                maxonMotor->commandBuffer.push(newData);

                maxonMotor->finalMotorPosition = newData.position;
            }
            maxonMotor->pre_q = Qi[can_id];
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                              Detect Collision                              */
////////////////////////////////////////////////////////////////////////////////

bool PathManager::detectCollision(MatrixXd &measureMatrix)
{
    VectorXd measureTime = measureMatrix.col(8);
    VectorXd measureIntensityR = measureMatrix.col(4);
    VectorXd measureIntensityL = measureMatrix.col(5);

    VectorXd stateDCR = measureStateR;
    VectorXd stateDCL = measureStateL;

    // 충돌 예측을 위한 악보 (뒤쪽 목표위치 없는 부분 짜르기)
    int endIndex = findDetectionRange(measureMatrix);

    // 충돌 예측
    bool isColli = false;
    double stepSize = 5;
    for (int i = 0; i < endIndex-1; i++)
    {
        MatrixXd tmpMatrix = measureMatrix.block(i,0,measureMatrix.rows()-i,measureMatrix.cols());

        // parse
        parsedData data = parseMeasure(tmpMatrix, stateDCR, stateDCL);
        stateDCR = data.nextStateR;
        stateDCL = data.nextStateL;

        double dt = (data.t2 - data.t1)/stepSize;

        for (int j = 0; j < stepSize+1; j++)
        {
            double tR = dt * j + data.t1 - data.initialTimeR;
            double tL = dt * j + data.t1 - data.initialTimeL;

            double sR = timeScaling(0.0, data.finalTimeR - data.initialTimeR, tR);
            double sL = timeScaling(0.0, data.finalTimeL - data.initialTimeL, tL);
            
            // task space 경로
            VectorXd PR = makePath(data.initialPositionR, data.finalPositionR, sR);
            VectorXd PL = makePath(data.initialPositionL, data.finalPositionL, sL);

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
            }
        }

        if (isColli)
        {
            return true;
        }
    }

    return false;
}

int PathManager::findDetectionRange(MatrixXd &measureMatrix)
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

bool PathManager::checkTable(VectorXd PR, VectorXd PL, double hitR, double hitL)
{
    double rangeMin[8] = {0.0, 0.0, -0.3, 0.3, 0.5, -0.3, 0.3, 0.5};
    double rangeMax[8] = {50.0*M_PI/180.0, 50.0*M_PI/180.0, 0.5, 0.7, 1.0, 0.5, 0.7, 1.0};

    std::vector<double> target = {hitR, hitL, PR(0), PR(1), PR(2), PL(0), PL(1), PL(2)};
    
    std::vector<size_t> dims = {11, 11, 19, 10, 12, 19, 10, 12};    // Rw Lw Rx Ry Rz Lx Ly Lz
    std::vector<size_t> targetIndex;

    // 인덱스 공간으로 변환
    for (int i = 0; i < 8; i++)
    {
        size_t index = round((dims[i]-1)*(target[i] - rangeMin[i])/(rangeMax[i] - rangeMin[i]));

        if (index > dims[i]-1)
        {
            index = (size_t)(dims[i]-1);
        }
        else if (index < 0)
        {
            index = 0;
        }

        targetIndex.push_back(index);
    }

    // 테이블 확인
    std::ifstream tableFile(tablePath, std::ifstream::binary);
    
    if (tableFile)
    {
        std::size_t offsetIndex = getFlattenIndex(targetIndex, dims);

        std::pair<size_t, size_t> bitIndex = getBitIndex(offsetIndex);
        
        tableFile.seekg(bitIndex.first, std::ios::beg);
        
        char byte;
        tableFile.read(&byte, 1);
        if (!tableFile) {
            std::cerr << " 바이트 읽기 실패 \n";
            return false;
        }

        tableFile.close();

        // byte에서 원하는 2비트 추출 (LSB 기준)
        uint8_t value = (byte >> bitIndex.second) & 0b11;

        if (value == 0b00)
        {
            // fun.appendToCSV_DATA("CC1", targetIndex[0], targetIndex[1], targetIndex[2]);
            // fun.appendToCSV_DATA("CC2", targetIndex[3], targetIndex[4], targetIndex[5]);
            // fun.appendToCSV_DATA("CC3", targetIndex[6], targetIndex[7], 0);
            return false;   // 충돌 안함
        }
        else
        {
            // fun.appendToCSV_DATA("CC1", targetIndex[0], targetIndex[1], targetIndex[2]);
            // fun.appendToCSV_DATA("CC2", targetIndex[3], targetIndex[4], targetIndex[5]);
            // fun.appendToCSV_DATA("CC3", targetIndex[6], targetIndex[7], 1);
            return true;    // 충돌 위험 or IK 안풀림
        }
    }
    else
    {
        std::cout << "\n 테이블 열기 실패 \n";
        std::cout << tablePath;
        return false;
    }
}

size_t PathManager::getFlattenIndex(const std::vector<size_t>& indices, const std::vector<size_t>& dims)
{
    size_t flatIndex = 0;
    size_t multiplier = 1;

    for (int i = 0; i < 8; i++)
    {
        flatIndex += indices[7-i] * multiplier;
        multiplier *= dims[7-i];
    }

    return flatIndex;
}

std::pair<size_t, size_t> PathManager::getBitIndex(size_t offsetIndex)
{
    size_t bitNum = offsetIndex * 2;
    size_t byteNum = bitNum / 8;
    size_t bitIndex = bitNum - 8 * byteNum;

    return std::make_pair(byteNum, bitIndex);
}

////////////////////////////////////////////////////////////////////////////////
/*                              Avoid Collision                               */
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
        if (method == modificationMethods[0])
        {
            modificationSuccess = modifyCrash(modifedMatrix, nModification);     // 주어진 방법으로 수정하면 True 반환
        }
        else if (method == modificationMethods[1])
        {
            modificationSuccess = waitAndMove(modifedMatrix, nModification);     // 주어진 방법으로 수정하면 True 반환   
        }
        else if (method == modificationMethods[2])
        {
            modificationSuccess = moveAndWait(modifedMatrix, nModification);     // 주어진 방법으로 수정하면 True 반환
        }
        else if (method == modificationMethods[3])
        {
            modificationSuccess = switchHands(modifedMatrix, nModification);     // 주어진 방법으로 수정하면 True 반환
        }
        else if (method == modificationMethods[4])
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

        if (!detectCollision(modifedMatrix))   // 충돌 예측
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

bool PathManager::switchHands(MatrixXd &measureMatrix, int num)
{
    // 주어진 방법으로 수정하면 True 반환
    VectorXd t = measureMatrix.col(8);
    VectorXd instR = measureMatrix.col(2);
    VectorXd instL = measureMatrix.col(3);

    // 수정하면 안되는 부분 제외
    pair<int, int> detectLine = findModificationRange(t, instR, instL);

    // 뒤쪽 목표위치 없는 부분 수정하면 안됨
    int endIndex = findDetectionRange(measureMatrix);

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

    for (int i = detectLineR; i < endIndex; i++)
    {
        if (instR(i) != 0)
        {
            if (instL(i) == 0)
            {
                if (cnt == num)
                {
                    measureMatrix(i, 3) = measureMatrix(i, 2);
                    measureMatrix(i, 5) = measureMatrix(i, 4);

                    measureMatrix(i, 2) = 0;
                    measureMatrix(i, 4) = 0;
                    return true;
                }
                cnt++;
            }
        }
    }

    for (int i = detectLineL; i < endIndex; i++)
    {
        if (instL(i) != 0)
        {
            if (instR(i) == 0)
            {
                if (cnt == num)
                {
                    measureMatrix(i, 2) = measureMatrix(i, 3);
                    measureMatrix(i, 4) = measureMatrix(i, 5);

                    measureMatrix(i, 3) = 0;
                    measureMatrix(i, 5) = 0;
                    return true;
                }
                cnt++;
            }
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

    // 뒤쪽 목표위치 없는 부분 수정하면 안됨
    int endIndex = findDetectionRange(measureMatrix);

    int detectLineR = detectLine.first;
    int detectLineL = detectLine.second;

    // Move and Wait
    int cnt = 0;

    for (int i = detectLineR; i < endIndex; i++)
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

    for (int i = detectLineL; i < endIndex; i++)
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

////////////////////////////////////////////////////////////////////////////////
/*                                   IK                                       */
////////////////////////////////////////////////////////////////////////////////

VectorXd PathManager::solveGeometricIK(VectorXd &pR, VectorXd &pL, double theta0, double theta7, double theta8, bool printError)
{
    VectorXd output;
    PartLength partLength;

    double XR = pR(0), YR = pR(1), ZR = pR(2);
    double XL = pL(0), YL = pL(1), ZL = pL(2);
    double R1 = partLength.upperArm;
    double R2 = getLength(theta7);
    double L1 = partLength.upperArm;
    double L2 = getLength(theta8);
    double s = partLength.waist;
    double z0 = partLength.height;

    double shoulderXR = 0.5 * s * cos(theta0);
    double shoulderYR = 0.5 * s * sin(theta0);
    double shoulderXL = -0.5 * s * cos(theta0);
    double shoulderYL = -0.5 * s * sin(theta0);

    double theta01 = atan2(YR - shoulderYR, XR - shoulderXR);
    double theta1 = theta01 - theta0;

    double err = 0.0;  // IK 풀림

    if (theta1 < 0 || theta1 > 150.0 * M_PI / 180.0) // the1 범위 : 0deg ~ 150deg
    {
        if (printError)
        {
            std::cout << "PR \n" << pR << "\n";
            std::cout << "PL \n" << pL << "\n";
            std::cout << "theta0 : " << theta0 * 180.0 / M_PI << "\n theta7 : " << theta7 * 180.0 / M_PI << "\n theta8 : " << theta8 * 180.0 / M_PI << "\n";
            std::cout << "IKFUN (q1) is not solved!!\n";
            std::cout << "theta1 : " << theta1 * 180.0 / M_PI << "\n";
        }
        err = 1.0;  // IK 안풀림
    }

    double theta02 = atan2(YL - shoulderYL, XL - shoulderXL);
    double theta2 = theta02 - theta0;

    if (theta2 < 30 * M_PI / 180.0 || theta2 > M_PI) // the2 범위 : 30deg ~ 180deg
    {
        if (printError)
        {
            std::cout << "PR \n" << pR << "\n";
            std::cout << "PL \n" << pL << "\n";
            std::cout << "theta0 : " << theta0 * 180.0 / M_PI << "\n theta7 : " << theta7 * 180.0 / M_PI << "\n theta8 : " << theta8 * 180.0 / M_PI << "\n";
            std::cout << "IKFUN (q2) is not solved!!\n";
            std::cout << "theta2 : " << theta2 * 180.0 / M_PI << "\n";
        }
        err = 1.0;  // IK 안풀림
    }

    double zeta = z0 - ZR;
    double r2 = (YR - shoulderYR) * (YR - shoulderYR) + (XR - shoulderXR) * (XR - shoulderXR); // r^2

    double x = zeta * zeta + r2 - R1 * R1 - R2 * R2;
    double y = sqrt(4.0 * R1 * R1 * R2 * R2 - x * x);

    double theta4 = atan2(y, x);

    if (theta4 < 0 || theta4 > 140.0 * M_PI / 180.0) // the4 범위 : 0deg ~ 120deg
    {
        if (printError)
        {
            std::cout << "PR \n" << pR << "\n";
            std::cout << "PL \n" << pL << "\n";
            std::cout << "theta0 : " << theta0 * 180.0 / M_PI << "\n theta7 : " << theta7 * 180.0 / M_PI << "\n theta8 : " << theta8 * 180.0 / M_PI << "\n";
            std::cout << "IKFUN (q4) is not solved!!\n";
            std::cout << "theta4 : " << theta4 * 180.0 / M_PI << "\n";
        }
        err = 1.0;  // IK 안풀림
    }

    double theta34 = atan2(sqrt(r2), zeta);
    double theta3 = theta34 - atan2(R2 * sin(theta4), R1 + R2 * cos(theta4));

    if (theta3 < -45.0 * M_PI / 180.0 || theta3 > 90) // the3 범위 : -45deg ~ 90deg
    {
        if (printError)
        {
            std::cout << "PR \n" << pR << "\n";
            std::cout << "PL \n" << pL << "\n";
            std::cout << "theta0 : " << theta0 * 180.0 / M_PI << "\n theta7 : " << theta7 * 180.0 / M_PI << "\n theta8 : " << theta8 * 180.0 / M_PI << "\n";
            std::cout << "IKFUN (q3) is not solved!!\n";
            std::cout << "theta3 : " << theta3 * 180.0 / M_PI << "\n";
        }
        err = 1.0;  // IK 안풀림
    }

    zeta = z0 - ZL;
    r2 = (YL - shoulderYL) * (YL - shoulderYL) + (XL - shoulderXL) * (XL - shoulderXL); // r^2

    x = zeta * zeta + r2 - L1 * L1 - L2 * L2;
    y = sqrt(4.0 * L1 * L1 * L2 * L2 - x * x);

    double theta6 = atan2(y, x);

    if (theta6 < 0 || theta6 > 140.0 * M_PI / 180.0) // the6 범위 : 0deg ~ 120deg
    {
        if (printError)
        {
            std::cout << "PR \n" << pR << "\n";
            std::cout << "PL \n" << pL << "\n";
            std::cout << "theta0 : " << theta0 * 180.0 / M_PI << "\n theta7 : " << theta7 * 180.0 / M_PI << "\n theta8 : " << theta8 * 180.0 / M_PI << "\n";
            std::cout << "IKFUN (q6) is not solved!!\n";
            std::cout << "theta6 : " << theta6 * 180.0 / M_PI << "\n";
        }
        err = 1.0;  // IK 안풀림
    }

    double theta56 = atan2(sqrt(r2), zeta);
    double theta5 = theta56 - atan2(L2 * sin(theta6), L1 + L2 * cos(theta6));

    if (theta5 < -45.0 * M_PI / 180.0 || theta5 > 90) // the5 범위 : -45deg ~ 90deg
    {
        if (printError)
        {
            std::cout << "PR \n" << pR << "\n";
            std::cout << "PL \n" << pL << "\n";
            std::cout << "theta0 : " << theta0 * 180.0 / M_PI << "\n theta7 : " << theta7 * 180.0 / M_PI << "\n theta8 : " << theta8 * 180.0 / M_PI << "\n";
            std::cout << "IKFUN (q5) is not solved!!\n";
            std::cout << "theta5 : " << theta5 * 180.0 / M_PI << "\n";
        }
        err = 1.0;  // IK 안풀림
    }

    theta4 -= getTheta(R2, theta7);
    theta6 -= getTheta(L2, theta8);

    output.resize(10);
    output << theta0, theta1, theta2, theta3, theta4, theta5, theta6, theta7, theta8, err;

    return output;
}

double PathManager::getLength(double theta)
{
    PartLength partLength;
    double l1 = partLength.lowerArm;
    double l2 = partLength.stick;

    double l3 = l1 + l2 * cos(theta);

    double l4 = sqrt(l3 * l3 + ((l2 * sin(theta)) * (l2 * sin(theta))));

    return l4;
}

double PathManager::getTheta(double l1, double theta)
{
    PartLength partLength;

    double l2 = partLength.lowerArm + partLength.stick * cos(theta);

    double theta_m = acos(l2 / l1);

    return theta_m;
}
