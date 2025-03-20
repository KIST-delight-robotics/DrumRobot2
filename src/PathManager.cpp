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
    // wristAnglesL << 25.0*M_PI/180.0,   25.0*M_PI/180.0,    15.0*M_PI/180.0,    15.0*M_PI/180.0,    10.0*M_PI/180.0,    15.0*M_PI/180.0,    0.0*M_PI/180.0,    10.0*M_PI/180.0, 0;
    wristAnglesL << 0, 0, 0, 0, 0, 0, 0, 0, 0;

    canManager.wristAnglesR.resize(1, 9);
    canManager.wristAnglesL.resize(1, 9);

    canManager.wristAnglesR << 25.0*M_PI/180.0,   25.0*M_PI/180.0,    15.0*M_PI/180.0,    15.0*M_PI/180.0,    10.0*M_PI/180.0,    15.0*M_PI/180.0,    0.0*M_PI/180.0,    10.0*M_PI/180.0, 0;
    canManager.wristAnglesL << 25.0*M_PI/180.0,   25.0*M_PI/180.0,    15.0*M_PI/180.0,    15.0*M_PI/180.0,    10.0*M_PI/180.0,    15.0*M_PI/180.0,    0.0*M_PI/180.0,    10.0*M_PI/180.0, 0;
}

void PathManager::setReadyAngle()
{
    VectorXd defaultInstrumentR; /// 오른팔 시작 위치
    VectorXd defaultInstrumentL;  /// 왼팔 시작 위치

    VectorXd instrumentVector(18);

    defaultInstrumentR.resize(9);
    defaultInstrumentL.resize(9);
    defaultInstrumentR << 1, 0, 0, 0, 0, 0, 0, 0, 0; // S
    defaultInstrumentL << 1, 0, 0, 0, 0, 0, 0, 0, 0;  // S

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

    VectorXd waistMinMax = waistRange(pR, pL);
    VectorXd qk = IKFixedWaist(pR, pL, 0.5 * (waistMinMax(0) + waistMinMax(1)), defaultWristAngle(0), defaultWristAngle(1));

    for (int i = 0; i < qk.size(); ++i)
    {
        readyArr[i] = qk(i);
    }

    HitParameter param;
    readyArr[4] += param.elbowStayAngle;
    readyArr[6] += param.elbowStayAngle;
    readyArr[7] += param.wristStayAngle;
    readyArr[8] += param.wristStayAngle;
    // readyArr[8] = param.wristStayAngle;     // for Test
    // readyArr[9] = param.wristStayAngle;
    
}

////////////////////////////////////////////////////////////////////////////////
/*                                  Play                                      */
////////////////////////////////////////////////////////////////////////////////

bool PathManager::readMeasure(ifstream &inputFile, bool &bpmFlag)
{
    string row;
    double timeSum = 0.0;

    for (int i = 1; i < measureMatrix.rows(); i++)
    {
        timeSum += measureMatrix(i, 1);
    }

    // timeSum이 threshold를 넘으면 true 반환
    if (timeSum >= threshold)
    {
        std::cout << "\n//////////////////////////////// Read Measure : " << lineOfScore + 1 << "\n";
        // std::cout << measureMatrix;
        // std::cout << "\n ////////////// time sum : " << timeSum << "\n";

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

        if (!bpmFlag)
        {
            cout << "music";
            bpmOfScore = stod(items[0].substr(4));
            cout << " bpm = " << bpmOfScore << "\n";
            bpmFlag = 1;

            initVal();
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
            measureMatrix(measureMatrix.rows() - 1, 8) = totalTime * 100.0 / bpmOfScore;

            // timeSum 누적
            timeSum += measureMatrix(measureMatrix.rows() - 1, 1);

            // timeSum이 threshold를 넘으면 true 반환
            if (timeSum >= threshold)
            {
                std::cout << "\n//////////////////////////////// Read Measure : " << lineOfScore + 1 << "\n";
                // std::cout << measureMatrix;
                // std::cout << "\n ////////////// time sum : " << timeSum << "\n";

                return true;
            }
        }
    }
    return false;
}

void PathManager::generateTrajectory()
{
    // Predict Collision
    aNumOfLine = 0;
    int C = predictCollision(measureMatrix);

    // std::string fileName = "CollisionDetection";
    // fun.appendToCSV_DATA(fileName, C, aNumOfLine, 0.0);  // 충돌 OX, 탐색시간(us), 확인 줄

    // position
    pair<VectorXd, VectorXd> initialPosition, finalPosition;
    VectorXd initialPositionR(3);
    VectorXd initialPositionL(3);
    VectorXd finalPositionR(3);
    VectorXd finalPositionL(3);
    VectorXd initialWristAngle(2);
    VectorXd finalWristAngle(2);
    VectorXd waistMinMax;
    VectorXd intensity(2);
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
        
        Pt.endEffectorR = makePath(initialPositionR, finalPositionR, sR);
        Pt.endEffectorL = makePath(initialPositionL, finalPositionL, sL);

        // 타격 시 손목 각도
        Pt.wristAngleR = tR*(finalWristAngle(0) - initialWristAngle(0))/(finalTimeR - initialTimeR) + initialWristAngle(0);
        Pt.wristAngleL = tL*(finalWristAngle(1) - initialWristAngle(1))/(finalTimeL - initialTimeL) + initialWristAngle(1);

        trajectoryQueue.push(Pt);

        // std::string fileName;
        // fileName = "Trajectory_R";
        // fun.appendToCSV_DATA(fileName, Pt.endEffectorR[0], Pt.endEffectorR[1], Pt.endEffectorR[2]);
        // fileName = "Trajectory_L";
        // fun.appendToCSV_DATA(fileName, Pt.endEffectorL[0], Pt.endEffectorL[1], Pt.endEffectorL[2]);
        // fileName = "Wrist";
        // fun.appendToCSV_DATA(fileName, Pt.wristAngleR, Pt.wristAngleL, 0.0);

        if (i == 0)
        {
            waistMinMax = waistRange(Pt.endEffectorR, Pt.endEffectorL);
        }
    }

    // 다음 타격 세기 = 한 줄 삭제한 후 첫 줄
    intensity(0) = measureMatrix(0, 4);
    intensity(1) = measureMatrix(0, 5);

    saveLineData(n, waistMinMax, intensity, finalWristAngle);
}

bool PathManager::solveIKandPushConmmand()
{
    VectorXd q;
    // 정해진 개수만큼 커맨드 생성
    if (indexSolveIK >= lineData(0, 0))
    {
        if (shadow_flag == 1) shadow_flag = 0;
        indexSolveIK = 0;

        // 커맨드 생성 후 삭제
        if (lineData.rows() >= 1)
        {
            MatrixXd tmpMatrix(lineData.rows() - 1, lineData.cols());
            tmpMatrix = lineData.block(1, 0, tmpMatrix.rows(), tmpMatrix.cols());
            lineData.resize(tmpMatrix.rows(), tmpMatrix.cols());
            lineData = tmpMatrix;
        }

        // std::cout << "\n lineDate Over \n";

        return false;
    }
    else
    {
        if (indexSolveIK == 0)
        {
            // 허리 계수 구하기
            getWaistCoefficient();

            // std::cout << "\n lineDate Start : \n";
            // std::cout << lineData;
        }
        indexSolveIK++;
    }

    // waist
    double q0 = getWaistAngle(indexSolveIK);

    // solve IK
    solveIK(q, q0);

    // wrist, elbow
    getHitAngle(q, indexSolveIK);

    // push motor obj
    pushConmmandBuffer(q);

    // 데이터 기록
    // for (int i = 0; i < 9; i++)
    // {
    //     std::string fileName = "solveIK_q" + to_string(i);
    //     fun.appendToCSV_DATA(fileName, i, q(i), 0);
    // }

    // brake (허리)
    toBrake(0, q0_t0, q0_t1, q0_threshold);

    // 마지막 줄에서 모든 브레이크 정리
    if(lineData.rows() == 1){
        clearBrake();
    }
    
    // // 데이터 기록
    // for (int m = 0; m < 12; m++)
    // {
    //     if(m == 9){
    //         fun.appendToCSV_DATA("q0_brake_status", m, q0_b, 0); // 허리 브레이크 결과 출력
    //     }
    //     else if(m == 10){
    //         fun.appendToCSV_DATA("q1_brake_status", m, q1_b, 0); // 1번 어깨 브레이크 결과 출력
    //     }
    //     else if(m == 11){
    //         fun.appendToCSV_DATA("q2_brake_status", m, q2_b, 0); // 2번 어깨 브레이크 결과 출력
    //     }
    //     else{
    //         std::string fileName = "solveIK_q" + to_string(m);
    //         fun.appendToCSV_DATA(fileName, m, q(m), 0);
    //     }
    // }

    return true;
}

////////////////////////////////////////////////////////////////////////////////
/*                                AddStance                                   */
////////////////////////////////////////////////////////////////////////////////

void PathManager::getArr(vector<float> &arr)
{
    const float accMax = 100.0; // rad/s^2
    VectorXd Q1 = VectorXd::Zero(10);
    VectorXd Q2 = VectorXd::Zero(10);
    VectorXd Qt = VectorXd::Zero(10);
    VectorXd Vmax = VectorXd::Zero(10);

    float dt = canManager.DTSECOND; // 0.005
    float t = 2.0;                // 3초동안 실행
    float extraTime = 1.0;       // 추가 시간 1초
    int n = (int)(t / dt);
    int extraN = (int)(extraTime / dt);

    getMotorPos();

    for (int i = 0; i < 10; i++)
    {
        Q1(i) = currentMotorAngle[i];
        Q2(i) = arr[i];
    }

    Vmax = calVmax(Q1, Q2, accMax, t);

    for (int k = 0; k < 10; k++)
    {
        cout << "Q1[" << k << "] : " << Q1[k] * 180.0 / M_PI << " [deg] -> Q2[" << k << "] : " << Q2[k] * 180.0 / M_PI << " [deg]" << endl;
        cout << "Vmax[" << k << "] : " << Vmax(k) << "[rad/s]\n\n";
    }

    for (int k = 1; k <= n + extraN; ++k)
    {
        // Make Array
        Qt = makeProfile(Q1, Q2, Vmax, accMax, t * k / n, t);

        // Send to Buffer
        for (auto &entry : motors)
        {
            if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
            {
                TMotorData newData;
                newData.position = Qt[motorMapping[entry.first]];
                newData.spd = 0;
                newData.acl = 0;
                newData.isBrake = false;
                tMotor->commandBuffer.push(newData);
            }
            else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
            {
                MaxonData newData;
                newData.position = Qt[motorMapping[entry.first]];
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

void PathManager::initVal()
{
    measureMatrix.resize(1, 9);
    measureMatrix = MatrixXd::Zero(1, 9);

    measureState.resize(2, 3);
    measureState = MatrixXd::Zero(2, 3);
    measureState(0, 1) = 1.0;
    measureState(1, 1) = 1.0;

    lineData.resize(1, 12);
    lineData = MatrixXd::Zero(1, 12);

    lineOfScore = 0;
    threshold = 2.4;
    roundSum = 0.0;
    totalTime = 0.0;

    indexSolveIK = 0;
    q0_t1 = readyArr[0];
    q0_t0 = readyArr[0];
    nextq0_t1 = readyArr[0];
    clearBrake();

}

void PathManager::parseMeasure(MatrixXd &measureMatrix)
{
    VectorXd measureTime = measureMatrix.col(8);
    VectorXd measureInstrumentR = measureMatrix.col(2);
    VectorXd measureInstrumentL = measureMatrix.col(3);
    VectorXd measureIntensityR = measureMatrix.col(4);
    VectorXd measureIntensityL = measureMatrix.col(5);

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

    hitState.resize(4); 
    hitState.head(2) = makeState(measureMatrix);
    hitState(2) = dataR.first(20);
    hitState(3) = dataL.first(20);

    measureState.block(0, 0, 1, 3) = dataR.second.transpose();
    measureState.block(1, 0, 1, 3) = dataL.second.transpose();

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
/*                            Make Trajectory                                 */
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

VectorXd PathManager::getTargetWristAngle(VectorXd &inst)
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

    MatrixXd combined(2, 18);
    combined << wristAnglesR, MatrixXd::Zero(1, 9), MatrixXd::Zero(1, 9), wristAnglesL;
    MatrixXd angle = combined * instrumentVector;

    return angle;
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

void PathManager::saveLineData(int n, VectorXd waistMinMax, VectorXd intensity, VectorXd finalWristAngle)
{
    if (lineOfScore == 1)
    {
        lineData(0, 0) = n;
        lineData(0, 1) = waistMinMax(0);
        lineData(0, 2) = waistMinMax(1);
        lineData(0, 3) = hitState(0);
        lineData(0, 4) = hitState(1);
        lineData(0, 5) = intensity(0);
        lineData(0, 6) = intensity(1);
        lineData(0, 7) = waistMinMax(2);
        lineData(0, 8) = hitState(2);
        lineData(0, 9) = hitState(3);
        lineData(0, 10) = finalWristAngle(0);
        lineData(0, 11) = finalWristAngle(1);
    }
    else
    {
        lineData.conservativeResize(lineData.rows() + 1, lineData.cols());
        lineData(lineData.rows() - 1, 0) = n;
        lineData(lineData.rows() - 1, 1) = waistMinMax(0);
        lineData(lineData.rows() - 1, 2) = waistMinMax(1);
        lineData(lineData.rows() - 1, 3) = hitState(0);
        lineData(lineData.rows() - 1, 4) = hitState(1);
        lineData(lineData.rows() - 1, 5) = intensity(0);
        lineData(lineData.rows() - 1, 6) = intensity(1);
        lineData(lineData.rows() - 1, 7) = waistMinMax(2);
        lineData(lineData.rows() - 1, 8) = hitState(2);
        lineData(lineData.rows() - 1, 9) = hitState(3);
        lineData(lineData.rows() - 1, 10) = finalWristAngle(0);
        lineData(lineData.rows() - 1, 11) = finalWristAngle(1);
    }
}

VectorXd PathManager::waistRange(VectorXd &pR, VectorXd &pL)
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
/*                              Wrist & Elbow                                 */
////////////////////////////////////////////////////////////////////////////////

VectorXd PathManager::makeHitTrajetory(float t1, float t2, float t, int state, int wristIntensity, bool targetChangeFlag, bool wristDir)
{
    VectorXd addAngle;

    HitParameter param = getHitParameter(t1, t2, state, preParametersTmp, wristIntensity);
    preParametersTmp = param;

    addAngle.resize(2); // wrist, elbow
    
    if (hitMode == 2)
    {
        for (auto &entry : motors)
        {
            if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
            {
                if (maxonMotor->myName == "R_wrist"  && wristDir == 0)
                {
                    maxonMotor->hittingDrumAngle = lineData(0, 10);
                    addAngle(0) = makeWristAngle_TEST_R(t1, t2, t, state, param, wristIntensity, targetChangeFlag, maxonMotor->hitting, maxonMotor->hittingPos, wristDir);
                }
                else if (maxonMotor->myName == "L_wrist"  && wristDir == 1)
                {
                    maxonMotor->hittingDrumAngle = lineData(0, 11);
                    addAngle(0) = makeWristAngle_TEST_L(t1, t2, t, state, param, wristIntensity, targetChangeFlag, maxonMotor->hitting, maxonMotor->hittingPos, wristDir);
                }
            }
        }
    }
    else if (hitMode == 3)
    {
        for (auto &entry : motors)
        {
            if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
            {
                if (maxonMotor->myName == "R_wrist"  && wristDir == 0)
                {
                    maxonMotor->hittingDrumAngle = lineData(0, 10);
                    addAngle(0) = makeWristAngle_TEST_R_Fast(t1, t2, t, state, param, wristIntensity, targetChangeFlag, maxonMotor->hitting, maxonMotor->hittingPos, wristDir);
                }
                else if (maxonMotor->myName == "L_wrist"  && wristDir == 1)
                {
                    maxonMotor->hittingDrumAngle = lineData(0, 11);
                    addAngle(0) = makeWristAngle_TEST_L_Fast(t1, t2, t, state, param, wristIntensity, targetChangeFlag, maxonMotor->hitting, maxonMotor->hittingPos, wristDir);
                }
            }
        }
    }
    else if (hitMode == 4)
    {
        for (auto &entry : motors)
        {
            if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
            {
                if (maxonMotor->myName == "R_wrist"  && wristDir == 0)
                {
                    maxonMotor->hittingDrumAngle = lineData(0, 10);
                    addAngle(0) = makeWristAngle_TEST_R_CST(t1, t2, t, state, param, wristIntensity, targetChangeFlag, maxonMotor->hitting, maxonMotor->hittingPos, wristDir);
                }
                else if (maxonMotor->myName == "L_wrist"  && wristDir == 1)
                {
                    maxonMotor->hittingDrumAngle = lineData(0, 11);
                    addAngle(0) = makeWristAngle_TEST_L_CST(t1, t2, t, state, param, wristIntensity, targetChangeFlag, maxonMotor->hitting, maxonMotor->hittingPos, wristDir);
                }
            }
        }
    }
    else if (hitMode == 1)
    {
        addAngle(0) = makeWristAngle(t1, t2, t, state, param, wristIntensity, targetChangeFlag);
    }

    // addAngle(0) = makeWristAngle(t1, t2, t, state, param, wristIntensity, targetChangeFlag);
    addAngle(1) = makeElbowAngle(t1, t2, t, state, param, wristIntensity, targetChangeFlag);

    return addAngle;
}

PathManager::HitParameter PathManager::getHitParameter(float t1, float t2, int hitState, HitParameter preParam, int intensity)
{
    HitParameter param;

    param.elbowStayAngle = preParam.elbowStayAngle;
    param.wristStayAngle = preParam.wristStayAngle;
    canManager.wristStayAngle = param.wristStayAngle;

    param.elbowLiftAngle = std::min((t2 - t1) * elbowLiftBaseAngle / baseTime, elbowLiftBaseAngle);
    param.wristContactAngle = -1.0 * std::min((t2 - t1) * wristContactBaseAngle / baseTime, wristContactBaseAngle);
    // param.wristLiftAngle = std::min((t2-t1)*wristLiftBaseAngle/baseTime, wristLiftBaseAngle);

    t2 - t1 < 0.5 ? param.wristLiftAngle = (-100 * ((t2 - t1) - 0.5) * ((t2 - t1) - 0.5) + 30) * M_PI / 180.0 : param.wristLiftAngle = 30  * M_PI / 180.0;

    param.elbowStayTime = std::max(0.5 * (t2 - t1), t2 - t1 - 0.2);
    param.elbowLiftTime = std::max(0.5 * (t2 - t1), t2 - t1 - 0.2);

    t2 - t1 < 0.15 ? param.wristStayTime = 0.45 * (t2 - t1) : param.wristStayTime = 0.47 * (t2 - t1) - 0.05;
    if (intensity == 1)
        param.wristLiftTime = std::max(0.5 * (t2 - t1), t2 - t1 - 0.25);
    else if (intensity == 2)
        param.wristLiftTime = std::max(0.6 * (t2 - t1), t2 - t1 - 0.2);
    else
        param.wristLiftTime = std::max(0.7 * (t2 - t1), t2 - t1 - 0.15);
    param.wristContactTime = std::min(0.1 * (t2 - t1), 0.05); // 0.05 -> 0.025
    param.wristReleaseTime = std::min(0.2 * (t2 - t1), 0.1);

    return param;
}

float PathManager::makeWristAngle(float t1, float t2, float t, int state, HitParameter param, int intensity, bool targetChangeFlag)
{
    float wrist_q = 0.0;
    float t_contact = param.wristContactTime;
    float t_lift = param.wristLiftTime;
    float t_stay = param.wristStayTime;
    float t_release = param.wristReleaseTime;
    float t_hit = t2 - t1;
    float intensityFactor = 0.4 * intensity + 0.2; // 1 : 약하게   2 : 기본    3 : 강하게
    float wristLiftAngle = param.wristLiftAngle * intensityFactor + (param.wristLiftAngle * 0.2  * targetChangeFlag);

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
            A.resize(3, 3);
            b.resize(3, 1);
            A << 1, 0, 0,
                1, t_contact, t_contact * t_contact,
                0, 1, 2 * t_contact;
            b << 0, param.wristContactAngle, 0;
            A_1 = A.inverse();
            sol = A_1 * b;
            wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t;
        }
        else if (t <= t_release)
        {
            A.resize(4, 4);
            b.resize(4, 1);
            A << 1, t_contact, t_contact * t_contact, t_contact * t_contact * t_contact,
                1, t_release, t_release * t_release, t_release * t_release * t_release,
                0, 1, 2 * t_contact, 3 * t_contact * t_contact,
                0, 1, 2 * t_release, 3 * t_release * t_release;
            b << param.wristContactAngle, param.wristStayAngle, 0, 0;
            A_1 = A.inverse();
            sol = A_1 * b;
            wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
        }
        else
        {
            wrist_q = param.wristStayAngle;
        }
    }
    else if (state == 2)
    {
        
        // Stay - Lift - Hit
        if (t < t_stay)
        {
            // Stay
            wrist_q = param.wristStayAngle;
        }
        else if (t < t_lift)
        {
            A.resize(4, 4);
            b.resize(4, 1);
            A << 1, t_stay, t_stay * t_stay, t_stay * t_stay * t_stay,
                1, t_lift, t_lift * t_lift, t_lift * t_lift * t_lift,
                0, 1, 2 * t_stay, 3 * t_stay * t_stay,
                0, 1, 2 * t_lift, 3 * t_lift * t_lift;
            b << param.wristStayAngle, wristLiftAngle, 0, 0;
            A_1 = A.inverse();
            sol = A_1 * b;
            wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
        }
        else if (t <= t_hit)
        {
            A.resize(3, 3);
            b.resize(3, 1);
            A << 1, t_lift, t_lift * t_lift,
                1, t_hit, t_hit * t_hit,
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
        if (t < t_contact)
        {
            A.resize(3, 3);
            b.resize(3, 1);
            A << 1, 0, 0,
                1, t_contact, t_contact * t_contact,
                0, 1, 2 * t_contact;
            b << 0, param.wristContactAngle, 0;
            A_1 = A.inverse();
            sol = A_1 * b;
            wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t;
        }
        else if (t < t_stay)
        {
            A.resize(4, 4);
            b.resize(4, 1);
            A << 1, t_contact, t_contact * t_contact, t_contact * t_contact * t_contact,
                1, t_stay, t_stay * t_stay, t_stay * t_stay * t_stay,
                0, 1, 2 * t_contact, 3 * t_contact * t_contact,
                0, 1, 2 * t_stay, 3 * t_stay * t_stay;
            b << param.wristContactAngle, wristLiftAngle, 0, 0;
            A_1 = A.inverse();
            sol = A_1 * b;
            wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
        }
        else if (t < t_lift)
        {
            // Stay
            wrist_q = wristLiftAngle;
        }
        else if (t <= t_hit)
        {
            A.resize(3, 3);
            b.resize(3, 1);
            A << 1, t_lift, t_lift * t_lift,
                1, t_hit, t_hit * t_hit,
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
    return wrist_q;
}

float PathManager::makeWristAngle_TEST_R(float t1, float t2, float t, int state, HitParameter param, int intensity, bool targetChangeFlag, bool &hitting, float hittingPos, bool wristDir)
{
    static float t_hitting = 0.0;
    float wrist_q = 0.0;
    float t_press = param.wristContactTime;
    float t_lift = param.wristLiftTime;
    float t_stay = param.wristStayTime;
    float t_release = param.wristReleaseTime;
    float t_contact = t2 - t1;
    float intensityFactor = 0.4 * intensity + 0.2; // 1 : 약하게   2 : 기본    3 : 강하게
    float wristLiftAngle = param.wristLiftAngle * intensityFactor + (param.wristLiftAngle * 0.2  * targetChangeFlag);

    static bool hittingTimeCheck = true;

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd sol;

    if(hitting)
    {
        if (hittingTimeCheck)
        {
            t_hitting = t;
            hittingTimeCheck = false;
        }

        if (state == 1)
        {
            // Contact - Stay
            A.resize(4, 4);
            b.resize(4, 1);
            if (t <= t_release)
            {
                if (t_hitting > t_release)
                {
                    A << 1, 0, 0, 0,
                    1, t_release, t_release * t_release, t_release * t_release * t_release,
                    0, 1, 0, 0,
                    0, 1, 2 * t_release, 3 * t_release * t_release;
                    b << hittingPos, param.wristStayAngle, 0, 0;
                }
                else
                {
                    A << 1, t_hitting, t_hitting * t_hitting, t_hitting * t_hitting * t_hitting,
                    1, t_release, t_release * t_release, t_release * t_release * t_release,
                    0, 1, 2 * t_hitting, 3 * t_hitting * t_hitting,
                    0, 1, 2 * t_release, 3 * t_release * t_release;
                    b << hittingPos, param.wristStayAngle, 0, 0;
                }
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else
            {
                hitting = false;
                hittingTimeCheck = true;
                wrist_q = param.wristStayAngle;
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
            wrist_q = param.wristStayAngle;
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
                b << 0, param.wristContactAngle, 0;
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
                b << param.wristContactAngle, param.wristStayAngle, 0, 0;
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else
            {
                wrist_q = param.wristStayAngle;
            }
        }
        else if (state == 2)
        {
            
            // Stay - Lift - Hit
            if (t < t_stay)
            {
                // Stay
                wrist_q = param.wristStayAngle;
            }
            else if (t < t_lift)
            {
                A.resize(4, 4);
                b.resize(4, 1);
                A << 1, t_stay, t_stay * t_stay, t_stay * t_stay * t_stay,
                    1, t_lift, t_lift * t_lift, t_lift * t_lift * t_lift,
                    0, 1, 2 * t_stay, 3 * t_stay * t_stay,
                    0, 1, 2 * t_lift, 3 * t_lift * t_lift;
                b << param.wristStayAngle, wristLiftAngle, 0, 0;
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else if (t <= t_contact)
            {
                canManager.isHitR = true;
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
                b << 0, param.wristContactAngle, 0;
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
                b << param.wristContactAngle, wristLiftAngle, 0, 0;
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
                canManager.isHitR = true;
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

float PathManager::makeWristAngle_TEST_R_Fast(float t1, float t2, float t, int state, HitParameter param, int intensity, bool targetChangeFlag, bool &hitting, float hittingPos, bool wristDir)
{
    static float t_hitting = 0.0;
    float wrist_q = 0.0;
    float t_press = param.wristContactTime * releaseTimeVal;
    float t_lift = param.wristLiftTime;
    float t_stay = param.wristStayTime;
    static float t_release = param.wristReleaseTime;
    float t_contact = t2 - t1;
    float intensityFactor = 0.4 * intensity + 0.2; // 1 : 약하게   2 : 기본    3 : 강하게
    float wristLiftAngle = param.wristLiftAngle * intensityFactor + (param.wristLiftAngle * 0.2  * targetChangeFlag);

    static bool hittingTimeCheck = true;

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd sol;

    if(hitting)
    {
        if (hittingTimeCheck)
        {
            t_hitting = t;
            hittingTimeCheck = false;
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
                    b << hittingPos, param.wristStayAngle, 0, 0;
                }
                else
                {
                    t_release = t_hitting + t_press;
                    A << 1, t_hitting, t_hitting * t_hitting, t_hitting * t_hitting * t_hitting,
                    1, t_release, t_release * t_release, t_release * t_release * t_release,
                    0, 1, 2 * t_hitting, 3 * t_hitting * t_hitting,
                    0, 1, 2 * t_release, 3 * t_release * t_release;
                    b << hittingPos, param.wristStayAngle, 0, 0;
                }
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else
            {
                hitting = false;
                hittingTimeCheck = true;
                wrist_q = param.wristStayAngle;
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
            wrist_q = param.wristStayAngle;
        }
        else if (state == 1)
        {
            t_release = t_hitting + t_press;
            // Contact - Stay
            A.resize(3, 3);
            b.resize(3, 1);
            if (t < t_press)
            {
                A << 1, 0, 0,
                1, t_press, t_press * t_press,
                0, 1, 2 * t_press;
                b << 0, param.wristContactAngle, 0;
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
                b << param.wristContactAngle, param.wristStayAngle, 0, 0;
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else
            {
                wrist_q = param.wristStayAngle;
            }
        }
        else if (state == 2)
        {
            
            // Stay - Lift - Hit
            if (t < t_stay)
            {
                // Stay
                wrist_q = param.wristStayAngle;
            }
            else if (t < t_lift)
            {
                A.resize(4, 4);
                b.resize(4, 1);
                A << 1, t_stay, t_stay * t_stay, t_stay * t_stay * t_stay,
                    1, t_lift, t_lift * t_lift, t_lift * t_lift * t_lift,
                    0, 1, 2 * t_stay, 3 * t_stay * t_stay,
                    0, 1, 2 * t_lift, 3 * t_lift * t_lift;
                b << param.wristStayAngle, wristLiftAngle, 0, 0;
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else if (t <= t_contact)
            {
                canManager.isHitR = true;
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
                b << 0, param.wristContactAngle, 0;
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
                b << param.wristContactAngle, wristLiftAngle, 0, 0;
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
                canManager.isHitR = true;
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

float PathManager::makeWristAngle_TEST_R_CST(float t1, float t2, float t, int state, HitParameter param, int intensity, bool targetChangeFlag, bool &hitting, float hittingPos, bool wristDir)
{
    static float t_hitting = 0.0;
    float wrist_q = 0.0;
    float t_press = param.wristContactTime * releaseTimeVal;
    float t_lift = param.wristLiftTime;
    float t_stay = param.wristStayTime;
    static float t_release = param.wristReleaseTime;
    float t_contact = t2 - t1;
    float intensityFactor = 0.4 * intensity + 0.2; // 1 : 약하게   2 : 기본    3 : 강하게
    float wristLiftAngle = param.wristLiftAngle * intensityFactor + (param.wristLiftAngle * 0.2  * targetChangeFlag);

    static bool hittingTimeCheck = true;

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd sol;

    if(hitting)
    {
        if (hittingTimeCheck)
        {
            t_hitting = t;
            hittingTimeCheck = false;
            canManager.isCSTR = false;
        }

        if (state == 1)
        {
            // Contact - Stay
            A.resize(3, 3);
            b.resize(3, 1);
            if (t <= t_release)
            {
                if (t_hitting > t_press)
                {
                    wrist_q = param.wristStayAngle;
                }
                else
                {
                    t_release = t_hitting + t_press;
                    A << 1, t_hitting, t_hitting * t_hitting,
                    1, t_release, t_release * t_release,
                    0, 1, 2 * t_release;
                    b << hittingPos, param.wristStayAngle, 0;

                    A_1 = A.inverse();
                    sol = A_1 * b;
                    wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t;

                }
            }
            else
            {
                hitting = false;
                hittingTimeCheck = true;
                wrist_q = param.wristStayAngle;
            }
        }
        else if (state == 2)
        {
            if(t_hitting == t_contact)
            {
                wrist_q = hittingPos;
            }
            else
            {
                A.resize(3, 3);
                b.resize(3, 1);
                A << 1, t_hitting, t_hitting * t_hitting,
                1, t_contact, t_contact * t_contact,
                0, 1, 2 * t_contact;
                b << hittingPos, param.wristStayAngle, 0;
                   
            }
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
    else if (!hitting && !canManager.isCSTR)
    {
        if (state == 0)
        {
            // Stay
            wrist_q = param.wristStayAngle;
        }
        else if (state == 1)
        {
            wrist_q = param.wristStayAngle;
        }
        else if (state == 2)
        {
            
            // Stay - Lift - Hit
            if (t < t_stay)
            {
                // Stay
                wrist_q = param.wristStayAngle;
            }
            else if (t < t_lift)
            {
                A.resize(4, 4);
                b.resize(4, 1);
                A << 1, t_stay, t_stay * t_stay, t_stay * t_stay * t_stay,
                    1, t_lift, t_lift * t_lift, t_lift * t_lift * t_lift,
                    0, 1, 2 * t_stay, 3 * t_stay * t_stay,
                    0, 1, 2 * t_lift, 3 * t_lift * t_lift;
                b << param.wristStayAngle, wristLiftAngle, 0, 0;
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else if (t <= t_contact)
            {
                canManager.isHitR = true;
                canManager.isCSTR = true;
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
                b << 0, param.wristContactAngle, 0;
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
                b << param.wristContactAngle, wristLiftAngle, 0, 0;
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
                canManager.isHitR = true;
                canManager.isCSTR = true;
            }
        }
    }

    return wrist_q;
}

float PathManager::makeWristAngle_TEST_L(float t1, float t2, float t, int state, HitParameter param, int intensity, bool targetChangeFlag, bool &hitting, float hittingPos, bool wristDir)
{
    static float t_hitting = 0.0;
    float wrist_q = 0.0;
    float t_press = param.wristContactTime;
    float t_lift = param.wristLiftTime;
    float t_stay = param.wristStayTime;
    float t_release = param.wristReleaseTime;
    float t_contact = t2 - t1;
    float intensityFactor = 0.4 * intensity + 0.2; // 1 : 약하게   2 : 기본    3 : 강하게
    float wristLiftAngle = param.wristLiftAngle * intensityFactor + (param.wristLiftAngle * 0.2  * targetChangeFlag);

    static bool hittingTimeCheck = true;

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd sol;

    if(hitting)
    {
        if (hittingTimeCheck)
        {
            t_hitting = t;
            hittingTimeCheck = false;
        }

        if (state == 1)
        {
            // Contact - Stay
            A.resize(4, 4);
            b.resize(4, 1);
            if (t <= t_release)
            {
                if (t_hitting > t_release)
                {
                    A << 1, 0, 0, 0,
                    1, t_release, t_release * t_release, t_release * t_release * t_release,
                    0, 1, 0, 0,
                    0, 1, 2 * t_release, 3 * t_release * t_release;
                    b << hittingPos, param.wristStayAngle, 0, 0;
                }
                else
                {
                    A << 1, t_hitting, t_hitting * t_hitting, t_hitting * t_hitting * t_hitting,
                    1, t_release, t_release * t_release, t_release * t_release * t_release,
                    0, 1, 2 * t_hitting, 3 * t_hitting * t_hitting,
                    0, 1, 2 * t_release, 3 * t_release * t_release;
                    b << hittingPos, param.wristStayAngle, 0, 0;
                }
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else
            {
                hitting = false;
                hittingTimeCheck = true;
                wrist_q = param.wristStayAngle;
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
            wrist_q = param.wristStayAngle;
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
                b << 0, param.wristContactAngle, 0;
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
                b << param.wristContactAngle, param.wristStayAngle, 0, 0;
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else
            {
                wrist_q = param.wristStayAngle;
            }
        }
        else if (state == 2)
        {
            
            // Stay - Lift - Hit
            if (t < t_stay)
            {
                // Stay
                wrist_q = param.wristStayAngle;
            }
            else if (t < t_lift)
            {
                A.resize(4, 4);
                b.resize(4, 1);
                A << 1, t_stay, t_stay * t_stay, t_stay * t_stay * t_stay,
                    1, t_lift, t_lift * t_lift, t_lift * t_lift * t_lift,
                    0, 1, 2 * t_stay, 3 * t_stay * t_stay,
                    0, 1, 2 * t_lift, 3 * t_lift * t_lift;
                b << param.wristStayAngle, wristLiftAngle, 0, 0;
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else if (t <= t_contact)
            {
                canManager.isHitL = true;
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
                b << 0, param.wristContactAngle, 0;
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
                b << param.wristContactAngle, wristLiftAngle, 0, 0;
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

float PathManager::makeWristAngle_TEST_L_Fast(float t1, float t2, float t, int state, HitParameter param, int intensity, bool targetChangeFlag, bool &hitting, float hittingPos, bool wristDir)
{
    static float t_hitting = 0.0;
    float wrist_q = 0.0;
    float t_press = param.wristContactTime * releaseTimeVal;
    float t_lift = param.wristLiftTime;
    float t_stay = param.wristStayTime;
    static float t_release = param.wristReleaseTime;
    float t_contact = t2 - t1;
    float intensityFactor = 0.4 * intensity + 0.2; // 1 : 약하게   2 : 기본    3 : 강하게
    float wristLiftAngle = param.wristLiftAngle * intensityFactor + (param.wristLiftAngle * 0.2  * targetChangeFlag);

    static bool hittingTimeCheck = true;

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd sol;

    if(hitting)
    {
        if (hittingTimeCheck)
        {
            t_hitting = t;
            hittingTimeCheck = false;
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
                    b << hittingPos, param.wristStayAngle, 0, 0;
                }
                else
                {
                    t_release = t_hitting + t_press;
                    A << 1, t_hitting, t_hitting * t_hitting, t_hitting * t_hitting * t_hitting,
                    1, t_release, t_release * t_release, t_release * t_release * t_release,
                    0, 1, 2 * t_hitting, 3 * t_hitting * t_hitting,
                    0, 1, 2 * t_release, 3 * t_release * t_release;
                    b << hittingPos, param.wristStayAngle, 0, 0;
                }
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else
            {
                hitting = false;
                hittingTimeCheck = true;
                wrist_q = param.wristStayAngle;
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
            wrist_q = param.wristStayAngle;
        }
        else if (state == 1)
        {
            t_release = t_hitting + t_press;
            // Contact - Stay
            A.resize(3, 3);
            b.resize(3, 1);
            if (t < t_press)
            {
                A << 1, 0, 0,
                1, t_press, t_press * t_press,
                0, 1, 2 * t_press;
                b << 0, param.wristContactAngle, 0;
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
                b << param.wristContactAngle, param.wristStayAngle, 0, 0;
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else
            {
                wrist_q = param.wristStayAngle;
            }
        }
        else if (state == 2)
        {
            
            // Stay - Lift - Hit
            if (t < t_stay)
            {
                // Stay
                wrist_q = param.wristStayAngle;
            }
            else if (t < t_lift)
            {
                A.resize(4, 4);
                b.resize(4, 1);
                A << 1, t_stay, t_stay * t_stay, t_stay * t_stay * t_stay,
                    1, t_lift, t_lift * t_lift, t_lift * t_lift * t_lift,
                    0, 1, 2 * t_stay, 3 * t_stay * t_stay,
                    0, 1, 2 * t_lift, 3 * t_lift * t_lift;
                b << param.wristStayAngle, wristLiftAngle, 0, 0;
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
            }
            else if (t <= t_contact)
            {
                canManager.isHitL = true;
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
                b << 0, param.wristContactAngle, 0;
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
                b << param.wristContactAngle, wristLiftAngle, 0, 0;
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

float PathManager::makeWristAngle_TEST_L_CST(float t1, float t2, float t, int state, HitParameter param, int intensity, bool targetChangeFlag, bool &hitting, float hittingPos, bool wristDir)
{
    static float t_hitting = 0.0;
    float wrist_q = 0.0;
    float t_press = param.wristContactTime * releaseTimeVal;
    float t_lift = param.wristLiftTime;
    float t_stay = param.wristStayTime;
    static float t_release = param.wristReleaseTime;
    float t_contact = t2 - t1;
    float intensityFactor = 0.4 * intensity + 0.2; // 1 : 약하게   2 : 기본    3 : 강하게
    float wristLiftAngle = param.wristLiftAngle * intensityFactor + (param.wristLiftAngle * 0.2  * targetChangeFlag);

    static bool hittingTimeCheck = true;

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd sol;

    if(hitting)
    {
        if (hittingTimeCheck)
        {
            t_hitting = t;
            hittingTimeCheck = false;
            canManager.isCSTL = false;
        }

        if (state == 1)
        {
            A.resize(3, 3);
            b.resize(3, 1);
            // Contact - Stay
            if (t <= t_release)
            {
                if (t_hitting > t_press)
                {
                    wrist_q = param.wristStayAngle;
                }
                else
                {
                    t_release = t_hitting + t_press;
                    A << 1, t_hitting, t_hitting * t_hitting,
                    1, t_release, t_release * t_release,
                    0, 1, 2 * t_release;
                    b << hittingPos, param.wristStayAngle, 0;

                    A_1 = A.inverse();
                    sol = A_1 * b;
                    wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t;
                }
            }
            else
            {
                hitting = false;
                hittingTimeCheck = true;
                wrist_q = param.wristStayAngle;
            }
        }
        else if (state == 2)
        {
            if(t_hitting == t_contact)
            {
                wrist_q = hittingPos;
            }
            else
            {
                A.resize(3, 3);
                b.resize(3, 1);
                A << 1, t_hitting, t_hitting * t_hitting,
                1, t_contact, t_contact * t_contact,
                0, 1, 2 * t_contact;
                b << hittingPos, param.wristStayAngle, 0;
                
                A_1 = A.inverse();
                sol = A_1 * b;
                wrist_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t;
                cout << state << "\n";
                cout << t_hitting << "\n";
                cout << A << "\n";
                cout << b << "\n";
                cout << A_1 << "\n";
                cout << sol << "\n";
                cout << "\n" << "\n";

                if (A.determinant() == 0) {
                std::cerr << "Error: Matrix A is singular!" << std::endl;
                sleep(5);
                return -1;
                }       
            }
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
            else if (t <= t_contact)    // 여기 나중에 수정하기
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
            wrist_q = param.wristStayAngle;
        }
        else if (state == 1)
        {
            wrist_q = param.wristStayAngle;
        }
        else if (state == 2)
        {
            
            // Stay - Lift - Hit
            if (t < t_stay)
            {
                // Stay
                wrist_q = param.wristStayAngle;
            }
            else if (t < t_lift)
            {
                A.resize(4, 4);
                b.resize(4, 1);
                A << 1, t_stay, t_stay * t_stay, t_stay * t_stay * t_stay,
                    1, t_lift, t_lift * t_lift, t_lift * t_lift * t_lift,
                    0, 1, 2 * t_stay, 3 * t_stay * t_stay,
                    0, 1, 2 * t_lift, 3 * t_lift * t_lift;
                b << param.wristStayAngle, wristLiftAngle, 0, 0;
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
                b << 0, param.wristContactAngle, 0;
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
                b << param.wristContactAngle, wristLiftAngle, 0, 0;
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

float PathManager::makeElbowAngle(float t1, float t2, float t, int state, HitParameter param, int intensity, bool targetChangeFlag)
{
    float elbow_q = 0.0;

    float t_lift = param.elbowLiftTime;
    float t_stay = param.elbowStayTime;
    float t_hit = t2 - t1;
    float intensityFactor = 0.4 * intensity + 0.2; // 1 : 약하게   2 : 기본    3 : 강하게
    float elbowLiftAngle = param.elbowLiftAngle * intensityFactor + (param.elbowLiftAngle * 0.2 * targetChangeFlag);

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
            A.resize(4, 4);
            b.resize(4, 1);

            A << 1, 0, 0, 0,
                1, t_stay, t_stay * t_stay, t_stay * t_stay * t_stay,
                0, 1, 0, 0,
                0, 1, 2 * t_stay, 3 * t_stay * t_stay;

            b << 0, param.elbowStayAngle, 0, 0;

            A_1 = A.inverse();
            sol = A_1 * b;

            elbow_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
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
            A.resize(4, 4);
            b.resize(4, 1);

            A << 1, 0, 0, 0,
                1, t_lift, t_lift * t_lift, t_lift * t_lift * t_lift,
                0, 1, 0, 0,
                0, 1, 2 * t_lift, 3 * t_lift * t_lift;

            b << param.elbowStayAngle, elbowLiftAngle, 0, 0;

            A_1 = A.inverse();
            sol = A_1 * b;

            elbow_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
        }
        else if (t < t_hit)
        {
            A.resize(4, 4);
            b.resize(4, 1);

            A << 1, t_lift, t_lift * t_lift, t_lift * t_lift * t_lift,
                1, t_hit, t_hit * t_hit, t_hit * t_hit * t_hit,
                0, 1, 2 * t_lift, 3 * t_lift * t_lift,
                0, 1, 2 * t_hit, 3 * t_hit * t_hit;

            b << elbowLiftAngle, 0, 0, 0;

            A_1 = A.inverse();
            sol = A_1 * b;

            elbow_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
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
            A.resize(4, 4);
            b.resize(4, 1);

            A << 1, 0, 0, 0,
                1, t_lift, t_lift * t_lift, t_lift * t_lift * t_lift,
                0, 1, 0, 0,
                0, 1, 2 * t_lift, 3 * t_lift * t_lift;

            b << 0, elbowLiftAngle, 0, 0;

            A_1 = A.inverse();
            sol = A_1 * b;

            elbow_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
        }
        else if (t < t_hit)
        {
            A.resize(4, 4);
            b.resize(4, 1);

            A << 1, t_lift, t_lift * t_lift, t_lift * t_lift * t_lift,
                1, t_hit, t_hit * t_hit, t_hit * t_hit * t_hit,
                0, 1, 2 * t_lift, 3 * t_lift * t_lift,
                0, 1, 2 * t_hit, 3 * t_hit * t_hit;

            b << elbowLiftAngle, 0, 0, 0;

            A_1 = A.inverse();
            sol = A_1 * b;

            elbow_q = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;
        }
        else
        {
            elbow_q = 0.0;
        }
    }

    return elbow_q;
}

void PathManager::getHitAngle(VectorXd &q, int index)
{
    VectorXd add_qR;
    VectorXd add_qL;
    int stateR = lineData(0, 3);
    int stateL = lineData(0, 4);
    float n = lineData(0, 0);
    float next_n = 0;
    float dt = canManager.DTSECOND;
    float t = n * dt;

    if (i_wristR >= nnR)
    {
        i_wristR = 0;
        if (readyRflag)
            readyRflag = 0;
    }
    else if (readyRflag)
    {
        i_wristR++;
    }
    if (i_wristL >= nnL)
    {
        i_wristL = 0;
        if (readyLflag)
            readyLflag = 0;
    }
    else if (readyLflag)
    {
        i_wristL++;
    }

    if (lineData.rows() > 2)
    {
        next_n = lineData(1, 0);

        if (n * dt <= 0.2)
        {
            if (lineData(0, 3) == 1 && !readyRflag)
            {
                nnR = n + next_n;
                ntR = nnR * dt;
                readyRflag = 1;
                if (lineData(1, 3) == 0)
                {
                    next_stateR = lineData(0, 3);
                    next_intensityR = lineData(0, 5);
                }
                else if (lineData(1, 3) == 2)
                {
                    next_stateR = 3;
                    next_intensityR = lineData(1, 5);
                }
            }
            if (lineData(0, 4) == 1 && !readyLflag)
            {
                nnL = n + next_n;
                ntL = nnL * dt;
                readyLflag = 1;
                if (lineData(1, 4) == 0)
                {
                    next_stateL = lineData(0, 4);
                    next_intensityL = lineData(0, 6);
                }
                else if (lineData(1, 4) == 2)
                {
                    next_stateL = 3;
                    next_intensityL = lineData(1, 6);
                }
            }
        }

        if (next_n * dt <= 0.2)
        {
            if (lineData(1, 3) == 2 && !readyRflag)
            {
                nnR = n + next_n;
                ntR = nnR * dt;
                readyRflag = 1;
                if (lineData(0, 3) == 0)
                {
                    next_stateR = lineData(1, 3);
                    next_intensityR = lineData(1, 5);
                }
                else if (lineData(0, 3) == 1)
                {
                    next_stateR = 3;
                    next_intensityR = lineData(1, 5);
                }
            }

            if (lineData(1, 4) == 2 && !readyLflag)
            {
                nnL = n + next_n;
                ntL = nnL * dt;
                readyLflag = 1;
                if (lineData(0, 4) == 0)
                {
                    next_stateL = lineData(1, 4);
                    next_intensityL = lineData(1, 6);
                }
                else if (lineData(0, 4) == 1)
                {
                    next_stateL = 3;
                    next_intensityL = lineData(1, 6);
                }
            }
        }
    }

    if (readyRflag)
    {
        preParametersTmp = preParametersR;
        add_qR = makeHitTrajetory(0, ntR, i_wristR * dt, next_stateR, next_intensityR, lineData(0, 8), 0);
        preParametersR = preParametersTmp;
    }
    else
    {
        preParametersTmp = preParametersR;
        add_qR = makeHitTrajetory(0, t, index * dt, stateR, lineData(0, 5), lineData(0, 8), 0);
        preParametersR = preParametersTmp;
    }

    if (readyLflag)
    {
        preParametersTmp = preParametersL;
        add_qL = makeHitTrajetory(0, ntL, i_wristL * dt, next_stateL, next_intensityL, lineData(0, 9), 1);
        preParametersL = preParametersTmp;
    }
    else
    {
        preParametersTmp = preParametersL;
        add_qL = makeHitTrajetory(0, t, index * dt, stateL, lineData(0, 6), lineData(0, 9), 1);
        preParametersL = preParametersTmp;
    }

    q(4) += add_qR(1);
    q(6) += add_qL(1);
    q(7) += add_qR(0);
    q(8) += add_qL(0);
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

void PathManager::solveIK(VectorXd &q, double q0)
{
    Position nextP;

    nextP = trajectoryQueue.front();
    trajectoryQueue.pop();

    q = IKFixedWaist(nextP.endEffectorR, nextP.endEffectorL, q0, nextP.wristAngleR, nextP.wristAngleL);
}

VectorXd PathManager::IKFixedWaist(VectorXd &pR, VectorXd &pL, double theta0, double theta7, double theta8)
{
    VectorXd Qf;
    PartLength partLength;
    q1_state[0] = q1_state[1];
    q2_state[0] = q2_state[1];

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

    // cout << "\ntheta1: " << theta1 << "\ttheta2: " << theta2 << endl;
    q1_state[1] = Qf(1);
    q2_state[1] = Qf(2);

    return Qf;
}

void PathManager::pushConmmandBuffer(VectorXd &Qi)
{
    for (auto &entry : motors)
    {
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        {
            TMotorData newData;
            newData.position = Qi[motorMapping[entry.first]];
            newData.spd = 0;
            newData.acl = 0;
            tMotor->commandBuffer.push(newData);
        }
        else if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
        {
            MaxonData newData;
            newData.position = Qi[motorMapping[entry.first]];
            newData.WristState = 0; // 토크 제어 시 WristState 사용
            maxonMotor->commandBuffer.push(newData);
        }
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
    double alph, bet;
    if (q[1] == q[2])
    {
        m1 = 0;
        m2 = 0;
    }
    else{
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
    
    // for (int i = 0; i < 4; i++)
    // {
    //     std::cout << "\n q[" << i << "] = " << q[i] << ", t[" << i << "] = " << t[i] << endl;
    // }
    // std::cout << "\n m1 : " << m1 << "\tm2 : " << m2 << endl;
    
    return m;
}

std::pair<double, vector<double>> PathManager::getQ0t2(int mode)
{
    VectorXd t_getQ0t2;     // getQ0t2() 함수 안에서 사용할 시간 벡터
    double dt = canManager.DTSECOND;
    vector<double> m_interpolation = {0.0, 0.0};
    double q0_t2 = 0.0;

    switch (mode)
    {
        case 0:
        {
            // 중앙값
            q0_t2 = 0.5*(lineData(1,1) + lineData(1,2));
            break;
        }
        case 1:
        {
            // 다익스트라
            double dt = canManager.DTSECOND;
            t_getQ0t2.resize(6);
            for (int i = 0; i < 6; i++)
            {
                if (i == 0)
                {
                    t_getQ0t2(i) = t0;
                }
                else if (i == 1)
                {
                    t_getQ0t2(i) = 0;
                }
                else if (lineData.rows() >= i-1)
                {
                    t_getQ0t2(i) = t_getQ0t2(i-1) + lineData(i-2,0)*dt;
                }
                else
                {
                    t_getQ0t2(i) = t_getQ0t2(i-1) + 1;
                }
            }
            vector<double> x_values = {t_getQ0t2(0), t_getQ0t2(1)}; // 현재 x값과 다음 x값
            vector<pair<double, double>> y_ranges = {{lineData(0,1), lineData(0,2)}, {lineData(1,1), lineData(1,2)}};
            
            try {
                if(status == 1){
                    q0_t1 = nextq0_t1;
                }
                q0_t2 = dijkstra_top10_with_median(x_values, y_ranges, q0_t1);
                
                if(abs(q0_t2 - q0_t1) <= q0_threshold){ // qthreshold 이하라면 안 움직이고 이보다 큰 것들만 움직이게 함.
                    if(q0_t1 >= lineData(1,1) && q0_t1 <= lineData(1,2)){
                        nextq0_t1 = q0_t1;
                        q0_t2 = q0_t1;
                        status = 1;
                    }
                    else{
                        status = 0;
                    }
                }
                else{
                    status = 0;
                }

            } catch (const exception& e) {
                cerr << e.what() << endl;
            }
            break;
        }
        case 2:
        {
            // 기울기 평균
            t_getQ0t2.resize(4);
            for (int i = 0; i < 4; i++)
            {
                if (i == 0)
                {
                    t_getQ0t2(i) = 0;
                }
                else if (lineData.rows() >= i)
                {
                    t_getQ0t2(i) = t_getQ0t2(i-1) + lineData(i-1,0)*dt;
                }
                else
                {
                    t_getQ0t2(i) = t_getQ0t2(i-1) + 1;
                }
            }

            VectorXd a(3);
            for (int i = 0; i < 3; i++)
            {
                if (lineData.rows() > i+1)
                {
                    a(i) = (lineData(i+1,7) - q0_t1) / (t_getQ0t2(i+1)-t_getQ0t2(0));
                }
                else
                {
                    a(i) = (t_getQ0t2(i)-t_getQ0t2(0)) / (t_getQ0t2(i+1)-t_getQ0t2(0)) * a(i-1);
                }
            }

            q0_t2 = a.sum()/3.0*(t_getQ0t2(1)-t_getQ0t2(0)) + q0_t1;

            if (q0_t2 <= lineData(1,1) || q0_t2 >= lineData(1,2))
            {
                q0_t2 = 0.5*lineData(1,1) + 0.5*lineData(1,2);
            }

            break;
        }
        case 3:
        {
            // 다익스트라 + interpolation
            double dt = canManager.DTSECOND;
            t_getQ0t2.resize(6);
            for (int i = 0; i < 6; i++)
            {
                if (i == 0)
                {
                    t_getQ0t2(i) = t0;
                }
                else if (i == 1)
                {
                    t_getQ0t2(i) = 0;
                }
                else if (lineData.rows() >= i-1)
                {
                    t_getQ0t2(i) = t_getQ0t2(i-1) + lineData(i-2,0)*dt;
                }
                else
                {
                    t_getQ0t2(i) = t_getQ0t2(i-1) + 1;
                }
            }

            // 다익스트라 평균 (다음, 다다음, 다다다음 값까지 봄)
            try {
                vector<double> x_values1 = {t_getQ0t2(1), t_getQ0t2(2)}; // 현재 x값과 다음 x값
                vector<pair<double, double>> y_ranges1 = {{lineData(0,1), lineData(0,2)}, {lineData(1,1), lineData(1,2)}};
                q0_t2 = dijkstra_top10_with_median(x_values1, y_ranges1, q0_t1);

                vector<double> x_values2 = {t_getQ0t2(2), t_getQ0t2(3)}; // 다음 x값과 다다음 x값
                vector<pair<double, double>> y_ranges2 = {{lineData(1,1), lineData(1,2)}, {lineData(2,1), lineData(2,2)}};
                double q0_t3 = dijkstra_top10_with_median(x_values2, y_ranges2, q0_t2);

                // vector<double> x_values3 = {t_getQ0t2(3), t_getQ0t2(4)}; // 다다음 x값과 다다다음 x값
                // vector<pair<double, double>> y_ranges3 = {{lineData(2,1), lineData(2,2)}, {lineData(3,1), lineData(3,2)}};
                // double q0_t4 = dijkstra_top10_with_median(x_values3, y_ranges3, q0_t3);
                
                // Interpolation
                vector<double> y = {q0_t0, q0_t1, q0_t2, q0_t3};
                vector<double> x = {t_getQ0t2(0), t_getQ0t2(1), t_getQ0t2(2), t_getQ0t2(3)};
                m_interpolation = cubicInterpolation(y, x);

            } catch (const exception& e) {
                cerr << e.what() << endl;
            }
            break;
        }
        case 4:
        {
            // 기울기 평균 + interpolation
            double dt = canManager.DTSECOND;
            t_getQ0t2.resize(6);
            for (int i = 0; i < 6; i++)
            {
                if (i == 0)
                {
                    t_getQ0t2(i) = t0;
                }
                else if (i == 1)
                {
                    t_getQ0t2(i) = 0;
                }
                else if (lineData.rows() >= i-1)
                {
                    t_getQ0t2(i) = t_getQ0t2(i-1) + lineData(i-2,0)*dt;
                }
                else
                {
                    t_getQ0t2(i) = t_getQ0t2(i-1) + 1;
                }
            }

            // t1 -> t2
            VectorXd a(3);
            for (int i = 0; i < 3; i++)
            {
                if (lineData.rows() > i+1)
                {
                    a(i) = (lineData(i+1,7) - q0_t1) / (t_getQ0t2(i+2)-t_getQ0t2(1));
                }
                else
                {
                    a(i) = (t_getQ0t2(i+1)-t_getQ0t2(1)) / (t_getQ0t2(i+2)-t_getQ0t2(1)) * a(i-1);
                }
            }

            q0_t2 = a.sum()/3.0*(t_getQ0t2(2)-t_getQ0t2(1)) + q0_t1;

            if (q0_t2 <= lineData(1,1) || q0_t2 >= lineData(1,2))
            {
                q0_t2 = 0.5*lineData(1,1) + 0.5*lineData(1,2);
            }

            // t2 -> t3
            double q0_t3;
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
                        a(i) = (lineData(i+2,7) - q0_t1) / (t_getQ0t2(i+3)-t_getQ0t2(2));
                    }
                    else
                    {
                        a(i) = (t_getQ0t2(i+2)-t_getQ0t2(2)) / (t_getQ0t2(i+3)-t_getQ0t2(2)) * a(i-1);
                    }
                }

                q0_t3 = a.sum()/3.0*(t_getQ0t2(3)-t_getQ0t2(2)) + q0_t2;

                if (q0_t3 <= lineData(2,1) || q0_t3 >= lineData(2,2))
                {
                    q0_t3 = 0.5*lineData(2,1) + 0.5*lineData(2,2);
                }
            }

            vector<double> y = {q0_t0, q0_t1, q0_t2, q0_t3};
            vector<double> x = {t_getQ0t2(0), t_getQ0t2(1), t_getQ0t2(2), t_getQ0t2(3)};
            m_interpolation = cubicInterpolation(y, x);
        }
    }
    return {q0_t2, m_interpolation};
}

void PathManager::getWaistCoefficient()
{
    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    vector<double> m = {0.0, 0.0};
    double q0_t2;
    double dt = canManager.DTSECOND;
    double t21 = lineData(0, 0) * dt;

    if (lineData.rows() == 1)
    {
        q0_t2 = q0_t1;
    }
    else
    {
        std::pair<double, vector<double>> output = getQ0t2(4); // 0: 중앙값, 1: 다익스트라, 2: 기울기평균, 3: 다익스트라 + 보간법, 4: 기울기평균 + 보간법
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
/*                           AddStance FUNCTION                               */
////////////////////////////////////////////////////////////////////////////////

VectorXd PathManager::calVmax(VectorXd &q1, VectorXd &q2, float acc, float t2)
{
    VectorXd Vmax = VectorXd::Zero(10);

    for (int i = 0; i < 10; i++)
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

    for (int i = 0; i < 10; i++)
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

void PathManager::getMotorPos()
{
    // 각 모터의 현재위치 값 불러오기 ** CheckMotorPosition 이후에 해야함(변수값을 불러오기만 해서 갱신 필요)
    for (auto &entry : motors)
    {
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(entry.second))
        {
            currentMotorAngle[motorMapping[entry.first]] = tMotor->jointAngle;
        }
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(entry.second))
        {
            currentMotorAngle[motorMapping[entry.first]] = maxonMotor->jointAngle;
        }
    }
}

vector<float> PathManager::makeHomeArr(int cnt)
{
    vector<float> homeArrOneStep = {0, 0, 0, 0, 0, 0, 0, 0, 0};

    if (cnt == 1)
    {
        getMotorPos();

        for (int i = 0; i < 10; i++)
        {
            homeArrOneStep[i] = currentMotorAngle[i];
        }
        homeArrOneStep[1] = 135 * M_PI / 180.0;
        homeArrOneStep[2] = 45 * M_PI / 180.0;
        homeArrOneStep[7] = 90 * M_PI / 180.0;
        homeArrOneStep[8] = 90 * M_PI / 180.0;
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
            homeArrOneStep[i] = homeArr[i];
        }
    }
    else
    {
        std::cout << "Invalid Home Cnt";
    }

    return homeArrOneStep;
}

////////////////////////////////////////////////////////////////////////////////
/*                            SYSTEM FUNCTION                                 */
////////////////////////////////////////////////////////////////////////////////

vector<float> PathManager::FK()
{
    getMotorPos();

    PartLength partLength;
    vector<float> P;
    vector<float> theta(9);
    for (auto &motorPair : motors)
    {
        auto &name = motorPair.first;
        auto &motor = motorPair.second;
        if (std::shared_ptr<TMotor> tMotor = std::dynamic_pointer_cast<TMotor>(motor))
        {
            theta[motorMapping[name]] = tMotor->jointAngle;
            cout << name << " : " << theta[motorMapping[name]] << "\n";
        }
        if (std::shared_ptr<MaxonMotor> maxonMotor = std::dynamic_pointer_cast<MaxonMotor>(motor))
        {
            theta[motorMapping[name]] = maxonMotor->jointAngle;
            cout << name << " : " << theta[motorMapping[name]] << "\n";
        }
    }
    float r1 = partLength.upperArm, r2 = partLength.lowerArm, l1 = partLength.upperArm, l2 = partLength.lowerArm, stick = partLength.stick;
    float s = partLength.waist, z0 = partLength.height;

    P.push_back(0.5 * s * cos(theta[0]) + r1 * sin(theta[3]) * cos(theta[0] + theta[1]) + r2 * sin(theta[3] + theta[4]) * cos(theta[0] + theta[1]) + stick * sin(theta[3] + theta[4] + theta[7]) * cos(theta[0] + theta[1]));
    P.push_back(0.5 * s * sin(theta[0]) + r1 * sin(theta[3]) * sin(theta[0] + theta[1]) + r2 * sin(theta[3] + theta[4]) * sin(theta[0] + theta[1]) + stick * sin(theta[3] + theta[4] + theta[7]) * sin(theta[0] + theta[1]));
    P.push_back(z0 - r1 * cos(theta[3]) - r2 * cos(theta[3] + theta[4]) - stick * cos(theta[3] + theta[4] + theta[7]));
    P.push_back(-0.5 * s * cos(theta[0]) + l1 * sin(theta[5]) * cos(theta[0] + theta[2]) + l2 * sin(theta[5] + theta[6]) * cos(theta[0] + theta[2]) + stick * sin(theta[5] + theta[6] + theta[8]) * cos(theta[0] + theta[2]));
    P.push_back(-0.5 * s * sin(theta[0]) + l1 * sin(theta[5]) * sin(theta[0] + theta[2]) + l2 * sin(theta[5] + theta[6]) * sin(theta[0] + theta[2]) + stick * sin(theta[5] + theta[6] + theta[8]) * sin(theta[0] + theta[2]));
    P.push_back(z0 - l1 * cos(theta[5]) - l2 * cos(theta[5] + theta[6]) - stick * cos(theta[5] + theta[6] + theta[8]));

    return P;
}

////////////////////////////////////////////////////////////////////////////////
/*                                DIJKSTRA                                    */
////////////////////////////////////////////////////////////////////////////////

int PathManager::y_to_index(double y, double global_y_min, double step_size)
{
    return static_cast<int>(round((y - global_y_min) / step_size));
}

double PathManager::select_top10_with_median(const vector<double> &y_vals, double current_y, double y_min, double y_max)
{
    vector<double> distances;
    for (double y : y_vals)
    {
        distances.push_back(abs(y - current_y));
    }

    // 거리 정렬 및 인덱스 추적
    vector<int> sorted_idx(y_vals.size());
    iota(sorted_idx.begin(), sorted_idx.end(), 0);
    sort(sorted_idx.begin(), sorted_idx.end(), [&](int i, int j)
         { return distances[i] < distances[j]; });

    // 상위 n% 거리 추출
    int top_10_limit = max(1, static_cast<int>(ceil(sorted_idx.size() * 0.1)));
    vector<double> top_10_y_vals;
    for (int i = 0; i < top_10_limit; ++i)
    {
        top_10_y_vals.push_back(y_vals[sorted_idx[i]]);
    }

    // y 범위 중앙값 계산
    double y_mid = (y_min + y_max) / 2;

    // 중앙값과 가장 가까운 값을 선택
    auto closest = min_element(top_10_y_vals.begin(), top_10_y_vals.end(), [&](double a, double b)
                               { return abs(a - y_mid) < abs(b - y_mid); });

    return *closest;
}

double PathManager::dijkstra_top10_with_median(const vector<double> &x_values, const vector<pair<double, double>> &y_ranges, double start_y)
{
    int n = x_values.size(); // x 값의 개수
    double step_size = 0.01; // y 값 간격

    // y 범위의 전역 최소 및 최대값
    double global_y_min = y_ranges[0].first;
    double global_y_max = y_ranges[0].second;
    for (const auto &range : y_ranges)
    {
        global_y_min = min(global_y_min, range.first);
        global_y_max = max(global_y_max, range.second);
    }

    int max_steps = ceil((global_y_max - global_y_min) / step_size) + 1;

    // 초기값 유효성 확인
    if (start_y < y_ranges[0].first || start_y > y_ranges[0].second)
    {
        throw runtime_error("초기값이 유효하지 않습니다. 시작 범위는 [" + to_string(y_ranges[0].first) + ", " + to_string(y_ranges[0].second) + "]입니다.");
    }

    // 거리 및 이전 노드 저장
    vector<vector<double>> dist(n, vector<double>(max_steps, INFINITY));
    vector<vector<pair<int, double>>> prev(n, vector<pair<int, double>>(max_steps, {-1, -1}));

    // 시작 노드 초기화
    dist[0][y_to_index(start_y, global_y_min, step_size)] = 0;

    // 우선순위 큐
    priority_queue<Node, vector<Node>, greater<Node>> pq;
    pq.push(Node{0, start_y, 0});

    // 다익스트라 알고리즘 실행
    while (!pq.empty())
    {
        Node current = pq.top();
        pq.pop();

        int x_idx = current.x_idx;
        double y_val = current.y_val;
        double current_cost = current.cost;

        if (x_idx == n - 1)
        {
            continue; // 마지막 x 값에서는 경로 갱신만 수행
        }

        // 다음 x 값의 y 범위 확인
        int next_x_idx = x_idx + 1;
        double y_min_next = y_ranges[next_x_idx].first;
        double y_max_next = y_ranges[next_x_idx].second;

        vector<double> next_y_vals;
        for (double next_y = y_min_next; next_y <= y_max_next; next_y += step_size)
        {
            next_y_vals.push_back(next_y);
        }

        // 상위 10% 거리와 중앙값 근처의 y 값을 선택
        double selected_y = select_top10_with_median(next_y_vals, y_val, y_min_next, y_max_next);
        double next_cost = current_cost + abs(selected_y - y_val);
        int next_y_idx = y_to_index(selected_y, global_y_min, step_size);

        if (dist[next_x_idx][next_y_idx] > next_cost)
        {
            dist[next_x_idx][next_y_idx] = next_cost;
            prev[next_x_idx][next_y_idx] = {x_idx, y_val};
            pq.push(Node{next_x_idx, selected_y, next_cost});
        }
    }

    // 최적 경로 역추적
    vector<pair<double, double>> optimal_path;
    int best_y_idx = min_element(dist[n - 1].begin(), dist[n - 1].end()) - dist[n - 1].begin();
    double current_y = global_y_min + (best_y_idx * step_size);
    int current_x = n - 1;

    while (current_x >= 0)
    {
        optimal_path.push_back({x_values[current_x], current_y});
        auto prev_node = prev[current_x][y_to_index(current_y, global_y_min, step_size)];
        if (prev_node.first == -1)
        {
            break;
        }
        current_x = prev_node.first;
        current_y = prev_node.second;
    }

    reverse(optimal_path.begin(), optimal_path.end());
    return optimal_path.back().second;
}

void PathManager::updateRange(const VectorXd &output, double &min, double &max)
{
    min = output(1);
    max = output(0);
}

////////////////////////////////////////////////////////////////////////////////
/*                                Brake                                       */
////////////////////////////////////////////////////////////////////////////////

void PathManager::toBrake(double motornum, double nowval, double nextval, double threshold)
{
    if(abs(nowval - nextval) <= threshold){
        brakeArr[motornum] = 1;
    }
    else{
        brakeArr[motornum] = 0;
    }
    usbio.setUSBIO4761(motornum, brakeArr[motornum]);
}

void PathManager::clearBrake()  // 모든 brake끄기
{
    brakeArr = {0, 0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 8; i++)
    {
        usbio.setUSBIO4761(i, brakeArr[i]);
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                              Collision                                     */
////////////////////////////////////////////////////////////////////////////////

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
