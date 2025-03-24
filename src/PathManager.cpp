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

    // 타격 시 손목 각도
    wristAnglesR.resize(1, 9);
    wristAnglesL.resize(1, 9);

    //              S                  FT                  MT                  HT                  HH                  R                   RC                 LC
    wristAnglesR << 25.0*M_PI/180.0,   25.0*M_PI/180.0,    15.0*M_PI/180.0,    15.0*M_PI/180.0,    10.0*M_PI/180.0,    15.0*M_PI/180.0,    0.0*M_PI/180.0,    10.0*M_PI/180.0, 0;
    wristAnglesL << 25.0*M_PI/180.0,   25.0*M_PI/180.0,    15.0*M_PI/180.0,    15.0*M_PI/180.0,    10.0*M_PI/180.0,    15.0*M_PI/180.0,    0.0*M_PI/180.0,    10.0*M_PI/180.0, 0;
    // wristAnglesL << 0, 0, 0, 0, 0, 0, 0, 0, 0;

    canManager.wristAnglesR.resize(1, 9);
    canManager.wristAnglesL.resize(1, 9);

    canManager.wristAnglesR << 25.0*M_PI/180.0,   25.0*M_PI/180.0,    15.0*M_PI/180.0,    15.0*M_PI/180.0,    10.0*M_PI/180.0,    15.0*M_PI/180.0,    0.0*M_PI/180.0,    10.0*M_PI/180.0, 0;
    canManager.wristAnglesL << 25.0*M_PI/180.0,   25.0*M_PI/180.0,    15.0*M_PI/180.0,    15.0*M_PI/180.0,    10.0*M_PI/180.0,    15.0*M_PI/180.0,    0.0*M_PI/180.0,    10.0*M_PI/180.0, 0;
}

void PathManager::setReadyAngle()
{
    //////////////////////////////////////// Ready Angle
    readyAngle.resize(9);

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

    //////////////////////////////////////// Home Angle
    homeAngle.resize(9);
    //              waist          R_arm1         L_arm1
    homeAngle << 10*M_PI/180.0,  90*M_PI/180.0,  90*M_PI/180.0,
    //              R_arm2         R_arm3         L_arm2
                0*M_PI/180.0,  135*M_PI/180.0,  0*M_PI/180.0,
    //              L_arm3         R_wrist        L_wrist
                135*M_PI/180.0, 60*M_PI/180.0, 60*M_PI/180.0;

    //////////////////////////////////////// Shutdown Angle
    shutdownAngle.resize(9);
        //              waist          R_arm1         L_arm1
    shutdownAngle << 0*M_PI/180.0, 135*M_PI/180.0, 45*M_PI/180.0,
    //                  R_arm2         R_arm3         L_arm2
                    0*M_PI/180.0,  0*M_PI/180.0,   0*M_PI/180.0,
    //                  L_arm3         R_wrist        L_wrist
                    0*M_PI/180.0,  90*M_PI/180.0,  90*M_PI/180.0;
}

////////////////////////////////////////////////////////////////////////////////
/*                            Public FUNCTION                                 */
////////////////////////////////////////////////////////////////////////////////

void PathManager::pushAddStancePath(string flagName)
{
    VectorXd Q1 = VectorXd::Zero(9);
    VectorXd Q2 = VectorXd::Zero(9);
    VectorXd Qt = VectorXd::Zero(9);
    float dt = canManager.DTSECOND; // 0.005

    // finalMotorPosition -> 마지막 명령값에서 이어서 생성
    Q1 = getMotorPos();

    if (flagName == "isHome")
    {
        for (int i = 0; i < 9; i++)
        {
            Q2(i) = homeAngle(i);
        }
    }
    else if (flagName == "isReady")
    {
        for (int i = 0; i < 9; i++)
        {
            Q2(i) = readyAngle(i);
        }
    }
    else if (flagName == "isShutDown")
    {
        for (int i = 0; i < 9; i++)
        {
            Q2(i) = shutdownAngle(i);
        }
    }

    const float accMax = 100.0; // rad/s^2
    
    VectorXd Vmax = VectorXd::Zero(10);
    
    double T = 2.0;                // 2초동안 실행
    double extraTime = 1.0;       // 이전 시간 1초
    int n = (int)(T / dt);
    int extraN = (int)(extraTime / dt);

    Vmax = calVmax(Q1, Q2, accMax, T);

    for (int k = 0; k < 9; k++)
    {
        cout << "Q1[" << k << "] : " << Q1[k] * 180.0 / M_PI << " [deg] -> Q2[" << k << "] : " << Q2[k] * 180.0 / M_PI << " [deg]" << endl;
        cout << "Vmax[" << k << "] : " << Vmax(k) << "[rad/s]\n\n";
    }

    for (int k = 1; k <= n + extraN; ++k)
    {
        if (k > extraN)
        {
            // 이동
            float t = (k - extraN) * T / n;

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
                    tMotor->commandBuffer.push(newData);

                    tMotor->finalMotorPosition = newData.position;
                }
                else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
                {
                    // 1ms 로 동작 (임시)
                    for (int i = 0; i < 5; i++)
                    {
                        MaxonData newData;
                        newData.position = maxonMotor->jointAngleToMotorPosition(Qt[can_id]);
                        newData.mode = maxonMotor->CSP;
                        maxonMotor->commandBuffer.push(newData);

                        maxonMotor->finalMotorPosition = newData.position;
                    }
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
                    tMotor->commandBuffer.push(newData);
                }
                else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
                {
                    // 1ms 로 동작 (임시)
                    for (int i = 0; i < 5; i++)
                    {
                        MaxonData newData;
                        newData.position = maxonMotor->jointAngleToMotorPosition(Q1[can_id]);
                        newData.mode = maxonMotor->CSP;
                        maxonMotor->commandBuffer.push(newData);
                    }
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
    lineData.resize(1, 10);
    lineData = MatrixXd::Zero(1, 10);
    lineData(0, 0) = -1;

    q0_t1 = readyAngle(0);
    q0_t0 = readyAngle(0);
    nextq0_t1 = readyAngle(0);
    clearBrake();
}

void PathManager::generateTrajectory(MatrixXd &measureMatrix)
{
    // position
    pair<VectorXd, VectorXd> initialPosition, finalPosition;
    VectorXd initialPositionR(3);
    VectorXd initialPositionL(3);
    VectorXd finalPositionR(3);
    VectorXd finalPositionL(3);
    VectorXd initialWristAngle(2);
    VectorXd finalWristAngle(2);
    VectorXd waistVector;

    float n, sR, sL;
    float dt = canManager.DTSECOND;

    // parse
    parseMeasure(measureMatrix);

    // 한 줄의 데이터 개수
    n = (t2 - t1) / dt;
    roundSum += (int)(n * 1000) % 1000;
    if (roundSum >= 1000)
    {
        roundSum -= 1000;
        n++;
    }
    n = (int)n;

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
        float tR = dt * i + t1 - initialTimeR;
        float tL = dt * i + t1 - initialTimeL;

        sR = timeScaling(0.0f, finalTimeR - initialTimeR, tR);
        sL = timeScaling(0.0f, finalTimeL - initialTimeL, tL);
        
        Pt.trajectoryR = makePath(initialPositionR, finalPositionR, sR);
        Pt.trajectoryL = makePath(initialPositionL, finalPositionL, sL);

        // 타격 시 손목 각도
        Pt.wristAngleR = tR*(finalWristAngle(0) - initialWristAngle(0))/(finalTimeR - initialTimeR) + initialWristAngle(0);
        Pt.wristAngleL = tL*(finalWristAngle(1) - initialWristAngle(1))/(finalTimeL - initialTimeL) + initialWristAngle(1);

        trajectoryQueue.push(Pt);

        // 데이터 저장
        std::string fileName;
        fileName = "Trajectory_R";
        fun.appendToCSV_DATA(fileName, Pt.trajectoryR[0], Pt.trajectoryR[1], Pt.trajectoryR[2]);
        fileName = "Trajectory_L";
        fun.appendToCSV_DATA(fileName, Pt.trajectoryL[0], Pt.trajectoryL[1], Pt.trajectoryL[2]);
        fileName = "Wrist";
        fun.appendToCSV_DATA(fileName, Pt.wristAngleR, Pt.wristAngleL, 0.0);

        if (i == 0)
        {
            // 허리 범위, 및 최적화 각도 계산
            waistVector = calWaistAngle(Pt.trajectoryR, Pt.trajectoryL);
        }
    }

    saveLineData(n, waistVector);
}

void PathManager::solveIKandPushCommand()
{
    int n = lineData(0, 0); // 명령 개수

    makeWaistCoefficient();

    makeHitCoefficient();

    for (int i = 0; i < n; i++)
    {
        // waist angle
        double q0 = getWaistAngle(i);

        // solve IK
        VectorXd q = solveIK(q0);

        // wrist (hit)
        generateHit(q, i);

        // push command buffer
        pushCommandBuffer(q);

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
    if (lineData.rows() >= 1)
    {
        MatrixXd tmpMatrix(lineData.rows() - 1, lineData.cols());
        tmpMatrix = lineData.block(1, 0, tmpMatrix.rows(), tmpMatrix.cols());
        lineData.resize(tmpMatrix.rows(), tmpMatrix.cols());
        lineData = tmpMatrix;
    }

    // 마지막 줄에서 모든 브레이크 정리
    if(lineData.rows() == 1)
    {
        clearBrake();
        endOfPlayCommand = true;   // DrumRobot 에게 끝났음 알리기
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                           AddStance FUNCTION                               */
////////////////////////////////////////////////////////////////////////////////

VectorXd PathManager::calVmax(VectorXd &q1, VectorXd &q2, float acc, float t2)
{
    VectorXd Vmax = VectorXd::Zero(10);

    for (int i = 0; i < 9; i++)
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
    VectorXd Qi = VectorXd::Zero(10);

    for (int i = 0; i < 9; i++)
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
    VectorXd Qf = VectorXd::Zero(9);

    // finalMotorPosition 가져오기
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
/*                              Parse Measure                                 */
////////////////////////////////////////////////////////////////////////////////

void PathManager::parseMeasure(MatrixXd &measureMatrix)
{
    VectorXd measureTime = measureMatrix.col(8);
    VectorXd measureInstrumentR = measureMatrix.col(2);
    VectorXd measureInstrumentL = measureMatrix.col(3);
    VectorXd measureIntensityR = measureMatrix.col(4);
    VectorXd measureIntensityL = measureMatrix.col(5);

    MatrixXd StatesNTimeR;
    MatrixXd StatesNTimeL;

    pair<VectorXd, VectorXd> dataR = parseOneArm(measureTime, measureInstrumentR, measureState.row(0));
    pair<VectorXd, VectorXd> dataL = parseOneArm(measureTime, measureInstrumentL, measureState.row(1));

    // 데이터 저장
    initialInstrument << dataR.first.block(1, 0, 9, 1), dataL.first.block(1, 0, 9, 1);
    finalInstrument << dataR.first.block(11, 0, 9, 1), dataL.first.block(11, 0, 9, 1);

    initialTimeR = dataR.first(0);
    initialTimeL = dataL.first(0);
    finalTimeR = dataR.first(10);
    finalTimeL = dataL.first(10);

    t1 = measureMatrix(0, 8);
    t2 = measureMatrix(1, 8);

    // StatesNTimeR = makeState(measureInstrumentR, measureTime);
    // StatesNTimeL = makeState(measureInstrumentL, measureTime);

    hitState.resize(4); 
    hitState.head(2) = makeState(measureMatrix);
    // hitState(0) = StatesNTimeR(0,0);
    // hitState(1) = StatesNTimeL(0,0);
    hitState(2) = dataR.first(20);
    hitState(3) = dataL.first(20);

    measureState.block(0, 0, 1, 3) = dataR.second.transpose();
    measureState.block(1, 0, 1, 3) = dataL.second.transpose();

    // 다음 타격 세기
    intensity.resize(2);
    intensity(0) = measureMatrix(1, 4);
    intensity(1) = measureMatrix(1, 5);

    // std::cout << "\n /// t1 -> t2 : " << t1 << " -> " << t2 << "\n";

    // std::cout << "\nR : " << initialInstrument.block(0,0,9,1).transpose() << " -> " << finalInstrument.block(0,0,9,1).transpose();
    // std::cout << "\n /// ti -> tf : " << initialTimeR << " -> " << finalTimeR;

    // std::cout << "\nL : " << initialInstrument.block(9,0,9,1).transpose() << " -> " << finalInstrument.block(9,0,9,1).transpose();
    // std::cout << "\n /// ti -> tf : " << initialTimeL << " -> " << finalTimeL << std::endl;

    // std::cout << "\n ////////////// state\n";
    // std::cout << measureState << std::endl;

    // 읽은 줄 삭제
    MatrixXd tmpMatrix(measureMatrix.rows() - 1, measureMatrix.cols());
    tmpMatrix = measureMatrix.block(1, 0, tmpMatrix.rows(), tmpMatrix.cols());
    measureMatrix.resize(tmpMatrix.rows(), tmpMatrix.cols());
    measureMatrix = tmpMatrix;
}

pair<VectorXd, VectorXd> PathManager::parseOneArm(VectorXd t, VectorXd inst, VectorXd stateVector)
{
    map<int, int> instrumentMapping = {
        {1, 0}, {2, 1}, {3, 2}, {4, 3}, {5, 4}, {6, 5}, {7, 6}, {8, 7}, {11, 0}, {51, 0}, {61, 0}, {71, 0}, {81, 0}, {91, 0}};
    //    S       FT      MT      HT      HH       R      RC      LC       S        S        S        S        S        S

    VectorXd initialInstrument = VectorXd::Zero(9), finalInstrument = VectorXd::Zero(9);
    VectorXd outputVector = VectorXd::Zero(21); // 20 > 40 으로 변경

    VectorXd nextStateVector;

    bool detectHit = false;
    double detectTime = 0, initialT, finalT;
    int detectInst = 0, initialInstNum, finalInstNum;
    int preState, nextState;
    double hitDetectionThreshold = 1.2 * 100.0 / bpmOfScore; // 일단 이렇게 하면 1줄만 읽는 일 없음
    bool targetChangeFlag = 0;

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

    if ((initialInstNum - 1 / 2) < (finalInstNum - 1 / 2))
    {
        targetChangeFlag = 1;
    }

    initialInstrument(instrumentMapping[initialInstNum]) = 1.0;
    finalInstrument(instrumentMapping[finalInstNum]) = 1.0;
    outputVector << initialT, initialInstrument, finalT, finalInstrument, targetChangeFlag;

    nextStateVector.resize(3);
    nextStateVector << initialT, initialInstNum, nextState;

    return std::make_pair(outputVector, nextStateVector);
}

VectorXd PathManager::makeState(MatrixXd measureMatrix)
{
    VectorXd state(2);
    for (int i = 0; i < 2; i++)
    {
        if (measureMatrix(0, i + 4) == 0 && measureMatrix(1, i + 4) == 0)
        {
            // Stay
            state(i) = 0;
        }
        else if (measureMatrix(1, i + 4) == 0)
        {
            // Contact - Stay
            state(i) = 1;
        }
        else if (measureMatrix(0, i + 4) == 0)
        {
            // Stay - Lift - Hit
            state(i) = 2;
        }
        else
        {
            // Contact - Lift - Hit
            state(i) = 3;
        }
    }
    return state;
}

////////////////////////////////////////////////////////////////////////////////
/*                       Make Task Space Trajectory                           */
////////////////////////////////////////////////////////////////////////////////

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

float PathManager::timeScaling(float ti, float tf, float t)
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

VectorXd PathManager::makePath(VectorXd Pi, VectorXd Pf, float s)
{
    float degree = 2.0;

    float xi = Pi(0), xf = Pf(0);
    float yi = Pi(1), yf = Pf(1);
    float zi = Pi(2), zf = Pf(2);
    // vector<Pos_i> pos(3); // 미래상태 3개를 저장

    // for(int i = 0; i < pos.size(); i++){
    //     for(int dim = 0; dim < 3; dim++){
    //         pos[i].x = Pf(dim);
    //         pos[i].y = Pf(dim);
    //         pos[i].z = Pf(dim);
    //     }
    // }
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
            float a = (zi - zf) * std::pow(-1, degree);
            float b = zf;

            Ps(2) = a * std::pow(s - 1, degree) + b;
        }
    }

    return Ps;
}

void PathManager::saveLineData(int n, VectorXd waistVector)
{
    if (lineData(0, 0) == -1)   // 첫 줄
    {
        lineData(0, 0) = n;                         // 명령 개수
        lineData(0, 1) = waistVector(2);            // q0 최적값
        lineData(0, 2) = waistVector(0);            // q0 min
        lineData(0, 3) = waistVector(1);            // q0 max
        lineData(0, 4) = t1;                        // t1
        lineData(0, 5) = t2;                        // t2
        lineData(0, 6) = hitState(0);               // state R
        lineData(0, 7) = hitState(1);               // state L
        lineData(0, 8) = intensity(0);              // intensity R
        lineData(0, 9) = intensity(1);              // intensity L
    }
    else
    {
        lineData.conservativeResize(lineData.rows() + 1, lineData.cols());
        lineData(lineData.rows() - 1, 0) = n;                       // 명령 개수
        lineData(lineData.rows() - 1, 1) = waistVector(2);          // q0 최적값
        lineData(lineData.rows() - 1, 2) = waistVector(0);          // q0 min
        lineData(lineData.rows() - 1, 3) = waistVector(1);          // q0 max
        lineData(lineData.rows() - 1, 4) = t1;                      // t1
        lineData(lineData.rows() - 1, 5) = t2;                      // t2
        lineData(lineData.rows() - 1, 6) = hitState(0);             // state R
        lineData(lineData.rows() - 1, 7) = hitState(1);             // state L
        lineData(lineData.rows() - 1, 8) = intensity(0);            // intensity R
        lineData(lineData.rows() - 1, 9) = intensity(1);            // intensity L
    }
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

VectorXd PathManager::solveIK(double q0)
{
    VectorXd q;
    Position nextP;

    nextP = trajectoryQueue.front();
    trajectoryQueue.pop();

    q = IKFixedWaist(nextP.trajectoryR, nextP.trajectoryL, q0, nextP.wristAngleR, nextP.wristAngleL);
    
    return q;
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

    theta4 -= getTheta(R2, theta7);
    theta6 -= getTheta(L2, theta8);

    Qf.resize(9);
    Qf << theta0, theta1, theta2, theta3, theta4, theta5, theta6, theta7, theta8;

    return Qf;
}

////////////////////////////////////////////////////////////////////////////////
/*                           Make Hit Trajectory                              */
////////////////////////////////////////////////////////////////////////////////

void PathManager::makeHitCoefficient()
{
    float t1 = lineData(0, 4);
    float t2 = lineData(0, 5);
    int stateR = lineData(0, 6);
    int stateL = lineData(0, 7);
    int intensityR = lineData(0, 8);
    int intensityL = lineData(0, 9);

    elbowAngle elbowAngleR, elbowAngleL;
    wristAngle wristAngleR, wristAngleL;

    elbowTimeR = getElbowTime(t1, t2, intensityR);
    elbowTimeL = getElbowTime(t1, t2, intensityL);

    wristTimeR = getWristTime(t1, t2, intensityR);
    wristTimeL = getWristTime(t1, t2, intensityL);

    elbowAngleR = getElbowAngle(t1, t2, intensityR);
    elbowAngleL = getElbowAngle(t1, t2, intensityL);

    wristAngleR = getWristAngle(t1, t2, intensityR);
    wristAngleL = getWristAngle(t1, t2, intensityL);

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

    if (intensity == 1)
        wristTime.liftTime = std::max(0.5 * (T), T - 0.25);
    else if (intensity == 2)
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

    elbowAngle.liftAngle = std::max(0.5 * (T), T - 0.2);

    return elbowAngle;
}

PathManager::wristAngle PathManager::getWristAngle(float t1, float t2, int intensity)
{
    float T = t2 - t1;
    wristAngle wristAngle;

    wristAngle.stayAngle = 10 * M_PI / 180.0;
    t2 - t1 < 0.5 ? wristAngle.liftAngle = (-100 * ((T) - 0.5) * ((T) - 0.5) + 30) * M_PI / 180.0 : wristAngle.liftAngle = 30  * M_PI / 180.0;
    wristAngle.pressAngle = -1.0 * std::min((T) * (5 * M_PI / 180.0)/ 0.5, (5 * M_PI / 180.0));

    return wristAngle;
}

MatrixXd PathManager::makeElbowCoefficient(int state, elbowTime eT, elbowAngle eA)
{
    MatrixXd elbowCoefficient;
    
    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd sol, sol2;

    // elbow
    if (state == 0)
    {
        // Stay
        elbowCoefficient.resize(2, 4);
        elbowCoefficient << eA.stayAngle, 0, 0, 0,
                            eA.stayAngle, 0, 0, 0;
    }
    else if (state == 1)
    {
        // Release - Stay
        A.resize(4, 4);
        b.resize(4, 1);

        A << 1, 0, 0, 0,
            1, eT.stayTime, eT.stayTime * eT.stayTime, eT.stayTime * eT.stayTime * eT.stayTime,
            0, 1, 0, 0,
            0, 1, 2 * eT.stayTime, 3 * eT.stayTime * eT.stayTime;

        b << 0, eA.stayAngle, 0, 0;

        A_1 = A.inverse();
        sol = A_1 * b;

        elbowCoefficient.resize(2, 4);
        elbowCoefficient << sol(0), sol(1), sol(2), sol(3),
                            eA.stayAngle, 0, 0, 0;
    }
    else if (state == 2)
    {
        // Stay - Lift - Hit
        A.resize(4, 4);
        b.resize(4, 1);

        A << 1, 0, 0, 0,
            1, eT.liftTime, eT.liftTime * eT.liftTime, eT.liftTime * eT.liftTime * eT.liftTime,
            0, 1, 0, 0,
            0, 1, 2 * eT.liftTime, 3 * eT.liftTime * eT.liftTime;

        b << eA.stayAngle, eA.liftAngle, 0, 0;

        A_1 = A.inverse();
        sol = A_1 * b;

        A.resize(4, 4);
        b.resize(4, 1);

        A << 1, eT.liftTime, eT.liftTime * eT.liftTime, eT.liftTime * eT.liftTime * eT.liftTime,
            1, eT.hitTime, eT.hitTime * eT.hitTime, eT.hitTime * eT.hitTime * eT.hitTime,
            0, 1, 2 * eT.liftTime, 3 * eT.liftTime * eT.liftTime,
            0, 1, 2 * eT.hitTime, 3 * eT.hitTime * eT.hitTime;

        b << eA.liftAngle, 0, 0, 0;

        A_1 = A.inverse();
        sol2 = A_1 * b;

        elbowCoefficient.resize(2, 4);
        elbowCoefficient << sol(0), sol(1), sol(2), sol(3),
                            sol2(0), sol2(1), sol2(2), sol2(3);
    }
    else if (state == 3)
    {
        // Lift - Hit
        A.resize(4, 4);
        b.resize(4, 1);

        A << 1, 0, 0, 0,
            1, eT.liftTime, eT.liftTime * eT.liftTime, eT.liftTime * eT.liftTime * eT.liftTime,
            0, 1, 0, 0,
            0, 1, 2 * eT.liftTime, 3 * eT.liftTime * eT.liftTime;

        b << 0, eA.liftAngle, 0, 0;

        A_1 = A.inverse();
        sol = A_1 * b;

        A.resize(4, 4);
        b.resize(4, 1);

        A << 1, eT.liftTime, eT.liftTime * eT.liftTime, eT.liftTime * eT.liftTime * eT.liftTime,
            1, eT.hitTime, eT.hitTime * eT.hitTime, eT.hitTime * eT.hitTime * eT.hitTime,
            0, 1, 2 * eT.liftTime, 3 * eT.liftTime * eT.liftTime,
            0, 1, 2 * eT.hitTime, 3 * eT.hitTime * eT.hitTime;

        b << eA.liftAngle, 0, 0, 0;

        A_1 = A.inverse();
        sol2 = A_1 * b;

        elbowCoefficient.resize(2, 4);
        elbowCoefficient << sol(0), sol(1), sol(2), sol(3),
                            sol2(0), sol2(1), sol2(2), sol2(3);
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

    // wrist
    if (state == 0)
    {
        // Stay
        wristCoefficient.resize(4, 4);
        wristCoefficient << wA.stayAngle, 0, 0, 0,
                            wA.stayAngle, 0, 0, 0,
                            wA.stayAngle, 0, 0, 0,
                            wA.stayAngle, 0, 0, 0;
    }
    else if (state == 1)
    {
        // Release - Stay
        A.resize(3, 3);
        b.resize(3, 1);

        A << 1, 0, 0,
            1, wT.releaseTime, wT.releaseTime * wT.releaseTime,
            0, 1, 2 * wT.releaseTime;

        b << wA.pressAngle, wA.stayAngle, 0;

        A_1 = A.inverse();
        sol = A_1 * b;

        wristCoefficient.resize(4, 4);
        wristCoefficient << sol(0), sol(1), sol(2), 0,
                            wA.stayAngle, 0, 0, 0,
                            wA.stayAngle, 0, 0, 0,
                            wA.stayAngle, 0, 0, 0;
    }
    else if (state == 2)
    {
        // Stay - Lift - Hit
        A.resize(4, 4);
        b.resize(4, 1);

        A << 1, wT.stayTime, wT.stayTime * wT.stayTime, wT.stayTime * wT.stayTime * wT.stayTime,
            1, wT.liftTime, wT.liftTime * wT.liftTime, wT.liftTime * wT.liftTime * wT.liftTime,
            0, 1, 2 * wT.stayTime, 3 * wT.stayTime * wT.stayTime,
            0, 1, 2 * wT.liftTime, 3 * wT.liftTime * wT.liftTime;

        b << wA.stayAngle, wA.liftAngle, 0, 0;

        A_1 = A.inverse();
        sol = A_1 * b;

        A.resize(3, 3);
        b.resize(3, 1);

        A << 1, wT.liftTime, wT.liftTime * wT.liftTime,
            1, wT.hitTime, wT.hitTime * wT.hitTime,
            0, 1, 2 * wT.liftTime;

        b << wA.liftAngle, wA.pressAngle, 0;

        A_1 = A.inverse();
        sol2 = A_1 * b;

        wristCoefficient.resize(4, 4);
        wristCoefficient << wA.stayAngle, 0, 0, 0,
                            wA.stayAngle, 0, 0, 0,
                            sol(0), sol(1), sol(2), sol(3),
                            sol2(0), sol2(1), sol2(2), 0;
    }
    else if (state == 3)
    {
        // Lift - Hit
        A.resize(3, 3);
        b.resize(3, 1);

        A << 1, 0, 0,
            1, wT.stayTime, wT.stayTime * wT.stayTime,
            0, 1, 2 * wT.stayTime;

        b << wA.pressAngle, wA.liftAngle, 0;

        A_1 = A.inverse();
        sol = A_1 * b;

        A.resize(3, 3);
        b.resize(3, 1);

        A << 1, wT.liftTime, wT.liftTime * wT.liftTime,
            1, wT.hitTime, wT.hitTime * wT.hitTime,
            0, 1, 2 * wT.liftTime;

        b << wA.liftAngle, wA.pressAngle, 0;

        A_1 = A.inverse();
        sol2 = A_1 * b;

        wristCoefficient.resize(4, 4);
        wristCoefficient << sol(0), sol(1), sol(2), 0,
                            sol(0), sol(1), sol(2), 0,
                            wA.liftAngle, 0, 0, 0,
                            sol2(0), sol2(1), sol2(2), 0;
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

void PathManager::generateHit(VectorXd &q, int index)
{
    double dt = canManager.DTSECOND;
    double t = dt * index;

    double elbowAngleR = makeElbowAngle(t, elbowTimeR, elbowCoefficientR);
    double elbowAngleL = makeElbowAngle(t, elbowTimeL, elbowCoefficientL);
    double wristAngleR = makeWristAngle(t, wristTimeR, wristCoefficientR);
    double wristAngleL = makeWristAngle(t, wristTimeL, wristCoefficientL);

    q(4) += elbowAngleR;
    q(6) += elbowAngleL;
    q(7) += wristAngleR;
    q(8) += wristAngleL;
}

// MatrixXd PathManager::makeState(VectorXd drums, VectorXd time)
// {
//     float threshold = 0.2;

//     VectorXd tempStates;
//     MatrixXd statesNTime;

//     tempStates.resize(drums.size());
//     statesNTime.resize(2, drums.size());

//     tempStates = makeTempState(drums);                              // 악보만 보고 temp state 생성

//     statesNTime = makeArrangedState(tempStates, time, threshold);   // 시간이 짧은 부분 state 수정

//     return statesNTime;
// }

// VectorXd PathManager::makeTempState(VectorXd drums)
// {
//     VectorXd states;
//     states.resize(drums.size());

//     for (int i = 0; i < drums.size(); i++)
//     {
//         if (i == 0)                                 // 첫 줄에 드럼을 치면 state 2 아니면 0
//         {
//             if (drums(i) == 0) states(0) = 0;
//             else states(0) = 2;
//         }
//         else
//         {
//             if(drums(i) != 0)                           // 지금 줄에 타격이 있을 때
//             {
//                 if (drums(i-1) != 0) states(i) = 3;     // 이전 줄에 타격 o
//                 else states(i) = 2;                     // 이전 줄에 타격 x
//             }
//             else                                        // 지금 줄에 타격이 없을 때
//             {
//                 if (drums(i-1) != 0) states(i) = 1;     // 이전 줄에 타격 o
//                 else states(i) = 0;                     // 이전 줄에 타격 x
//             }
//         }
//     }
//     return states;
// }

// MatrixXd PathManager::makeArrangedState(VectorXd states, VectorXd time, float threshold)
// {
//     MatrixXd result;
//     result.resize(states.size(), 2);

//     for (int k = 0; k < states.size() - 1; k++)
//     {
//         if (time(k) <= threshold)
//         {
//             if(states(k) == 2)
//             {
//                 if (states(k-1) == 0)
//                 {
//                     time(k) += time(k-1);
//                     time(k-1) = 0;
//                 } 
//                 else if (states(k-1) == 1)
//                 {
//                     time(k) += time(k-1);
//                     time(k-1) = 0;
//                     states(k) = 3;
//                 }
//             }
//             else if (states(k) == 1)
//             {
//                 if (states(k+1) == 0)
//                 {
//                     time(k) += time(k+1);
//                     time(k+1) = 0;
//                 }
//                 else if (states(k+1) == 2)
//                 {
//                     time(k) += time(k+1);
//                     time(k+1) = 0;
//                     states(k) = 3;
//                 }
//             }
//         }
//     }

//     result.col(0) = states;
//     result.col(1) = time;

//     return result;
// }

////////////////////////////////////////////////////////////////////////////////
/*                           Push Command Buffer                              */
////////////////////////////////////////////////////////////////////////////////

void PathManager::pushCommandBuffer(VectorXd Qi)
{
    for (auto &entry : motors)
    {
        int can_id = canManager.motorMapping[entry.first];

        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        {
            TMotorData newData;
            newData.position = tMotor->jointAngleToMotorPosition(Qi[can_id]);
            newData.mode = tMotor->Position;
            tMotor->commandBuffer.push(newData);

            tMotor->finalMotorPosition = newData.position;
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
        {
            // 1ms 로 동작 (임시)
            for (int i = 0; i < 5; i++)
            {
                MaxonData newData;
                newData.position = maxonMotor->jointAngleToMotorPosition(Qi[can_id]);
                if (MaxonMode == "CST")
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
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                                Brake                                       */
////////////////////////////////////////////////////////////////////////////////

void PathManager::clearBrake()  // 모든 brake끄기
{
    vector<int> brakeArr = {0, 0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 8; i++)
    {
        usbio.setUSBIO4761(i, brakeArr[i]);
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                              Collision                                     */
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

int PathManager::predictCollision(MatrixXd measureMatrix)
{
    VectorXd measureTime = measureMatrix.col(8);
    VectorXd measureInstrumentR = measureMatrix.col(2);
    VectorXd measureInstrumentL = measureMatrix.col(3);
    VectorXd measureIntensityR = measureMatrix.col(4);
    VectorXd measureIntensityL = measureMatrix.col(5);

    // 악보 중 충돌을 예측할 범위
    double tStartR = measureTime(0), tStartL = measureTime(0);
    bool startR = false, startL = false;
    bool endR = false, endL = false;
    int endIndex = measureTime.rows();
    double hitDetectionThreshold = 1.2 * 100.0 / bpmOfScore;
    for (int i = 0; i < measureTime.rows(); i++)
    {
        if (measureInstrumentR(i) == 0)
        {
            if (round(10000 * hitDetectionThreshold) < round(10000 * (measureTime(i) - tStartR)))
            {
                endR = true;
            }
        }
        else
        {
            if (startR)
            {
                endR = true;
            }
            else
            {
                startR = true;
                tStartR = measureTime(i);
            }
        }

        if (measureInstrumentL(i) == 0)
        {
            if (round(10000 * hitDetectionThreshold) < round(10000 * (measureTime(i) - tStartL)))
            {
                endL = true;
            }
        }
        else
        {
            if (startL)
            {
                endL = true;
            }
            else
            {
                startL = true;
                tStartL = measureTime(i);
            }
        }

        if (endR && endL)
        {
            endIndex = i + 1;
            break;
        }
    }

    // std::cout << "\n end Index \n" << endIndex;

    // 파싱
    MatrixXd linePositionR = parseAllLine(measureTime, measureInstrumentR, measureState.row(0), 'R');
    MatrixXd linePositionL = parseAllLine(measureTime, measureInstrumentL, measureState.row(1), 'L');

    // 충돌 예측
    double stepSize = 5;
    int C = 0;
    for (int i = 0; i < endIndex; i++)
    {
        if (i == measureTime.rows()-1)
        {
            break;
        }

        for (int j = 0; j < stepSize+1; j++)
        {
            VectorXd PR(3), PL(3), preR(3), nextR(3), preL(3), nextL(3);
            double hitR, hitL;

            preR << linePositionR.block(i,1,1,3).transpose();
            nextR << linePositionR.block(i+1,1,1,3).transpose();
            preL << linePositionL.block(i,1,1,3).transpose();
            nextL << linePositionL.block(i+1,1,1,3).transpose();

            PR = preR + (nextR - preR)*j/stepSize;
            PL = preL + (nextL - preL)*j/stepSize;

            if (measureIntensityR(i+1) == 0)
            {
                hitR = 0.07;
            }
            else
            {
                hitR = 0.07 + 0.07*sin(M_PI*j/stepSize);
            }

            if (measureIntensityL(i+1) == 0)
            {
                hitL = 0.07;
            }
            else
            {
                hitL = 0.07 + 0.07*sin(M_PI*j/stepSize);
            }

            bool CT = checkTable(PR, PL, hitR, hitL);
            if (CT && i == 0)    // 해당 줄만 체크
            {
                C = 1;
                aNumOfLine++;
                break;
            }
        }
        aNumOfLine++;
    }

    return C;
}

MatrixXd PathManager::parseAllLine(VectorXd t, VectorXd inst, VectorXd stateVector, char RL)
{
    map<int, int> instrumentMapping = {
        {1, 0}, {2, 1}, {3, 2}, {4, 3}, {5, 4}, {6, 5}, {7, 6}, {8, 7}, {11, 0}, {51, 0}, {61, 0}, {71, 0}, {81, 0}, {91, 0}};
    //    S       FT      MT      HT      HH       R      RC      LC       S        S        S        S        S        S

    MatrixXd linePosition = MatrixXd::Zero(t.rows() + 1, 4);
    int preState = stateVector(2), preInst = stateVector(1);
    double preT = stateVector(0);
    double hitDetectionThreshold = 1.2 * 100.0 / bpmOfScore; // 일단 이렇게 하면 1줄만 읽는 일 없음

    linePosition(0,0) = preT;
    linePosition.block(0,1,1,3) = getOneDrumPosition(preInst, RL);

    for (int i = 0; i < t.rows(); i++)
    {
        linePosition(i+1,0) = t(i);

        if (inst(i) == 0)
        {
            if (preState == 2 || preState == 3) // 궤적 생성 중
            {
                MatrixXd prePosition = getOneDrumPosition(preInst, RL);
                MatrixXd nextPosition;
                double nextT = t(i);

                for (int j = 1; j < t.rows()-i; j++)    // 다음 타격 찾기
                {
                    if (inst(i+j) != 0)
                    {
                        nextPosition = getOneDrumPosition(inst(i+j), RL);
                        nextT = t(i+j);
                        break;
                    }
                }

                linePosition.block(i+1,1,1,3) = prePosition + (nextPosition - prePosition)*(t(i) - preT)/(nextT - preT);
            }
            else    // 대기 중
            {
                linePosition.block(i+1,1,1,3) = getOneDrumPosition(preInst, RL);
                preT = t(i);

                for (int j = 1; j < t.rows()-i; j++)    // 다음 타격 찾기
                {
                    preState = 0;
                    if (round(10000 * hitDetectionThreshold) < round(10000 * (t(i+j) - t(i))))
                    {
                        break;
                    }

                    if (inst(i+j) != 0)
                    {
                        preState = 2;
                        break;
                    }
                }
            }
        }
        else
        {
            linePosition.block(i+1,1,1,3) = getOneDrumPosition(inst(i), RL);
            preT = t(i);
            preInst = inst(i);

            for (int j = 1; j < t.rows()-i; j++)    // 다음 타격 찾기
            {
                preState = 0;
                if (round(10000 * hitDetectionThreshold) < round(10000 * (t(i+j) - t(i))))
                {
                    break;
                }

                if (inst(i+j) != 0)
                {
                    preState = 2;
                    break;
                }
            }
        }
    }

    MatrixXd tmpMatrix(linePosition.rows() - 1, linePosition.cols());
    tmpMatrix = linePosition.block(1, 0, tmpMatrix.rows(), tmpMatrix.cols());

    return tmpMatrix;
}

MatrixXd PathManager::getOneDrumPosition(int InstNum, char RL)
{
    VectorXd instrument = VectorXd::Zero(9);
    MatrixXd p;
    instrument(InstNum-1) = 1;

    if (RL == 'R')
    {
        p = drumCoordinateR * instrument;
    }
    else
    {
        p = drumCoordinateL * instrument;
    }

    return p.transpose();
}

bool PathManager::checkTable(VectorXd PR, VectorXd PL, double hitR, double hitL)
{
    std::ifstream tableFile;
    std::string tablePath = "/home/shy/DrumRobot/collision_table/";    // 테이블 위치

    int PR_index[3] = {0};
    int PL_index[3] = {0};
    double range[2][3] = {{-0.35, 0.45, 0.55}, {0.8, 0.35, 0.65}};
    double dx = 0.05;

    // 타격 궤적 보정
    PR(2) = PR(2) + hitR;
    PL(2) = PL(2) + hitL;

    // 인덱스 공간으로 변환
    for (int i = 0; i < 3; i++)
    {
        PR_index[i] = floor((PR(i) - range[0][i])/dx + 0.5);
        PL_index[i] = floor((PL(i) - range[0][i])/dx + 0.5);

        if (PR_index[i] > range[1][i]/dx)
        {
            PR_index[i] = (int)(range[1][i]/dx);
        }

        if (PL_index[i] > range[1][i]/dx)
        {
            PL_index[i] = (int)(range[1][i]/dx);
        }
    }

    std::string fileName = tablePath + "TABLE_" + std::to_string(PR_index[0]+1) + "_" + std::to_string(PR_index[1]+1) +".txt";
    tableFile.open(fileName); // 파일 열기
        
    if (tableFile.is_open())
    {
        string row;

        for (int j = 0; j < PL_index[0]+1; j++)
        {
            getline(tableFile, row);
        }

        istringstream iss(row);
        string item;

        for (int j = 0; j < PR_index[2]+1; j++)
        {
            getline(iss, item, '\t');
        }

        item = trimWhitespace(item);

        char hex1 = item.at(2*PL_index[2] + 1);
        char hex2 = item.at(2*PL_index[2]);

        return hex2TableData(hex1, hex2, PL_index[1]);
    }
    else
    {
        std::cout << "\n table file open error \n";
        std::cout << fileName;
    }

    tableFile.close(); // 파일 닫기

    return false;
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
