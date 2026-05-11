#include "../include/tasks/DrumDetector.hpp"

DrumDetector::DrumDetector(PathManager &pathManagerRef, TestManager &testManagerRef)
    : pathManager(pathManagerRef), testManager(testManagerRef),
      align_to_color(RS2_STREAM_COLOR),
      depth_to_disparity(true),
      disparity_to_depth(false)
{
    initCamera();
}
DrumDetector::~DrumDetector()
{
    try
    {
        pipe.stop();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Destructor error: " << e.what() << std::endl;
    }
}

void DrumDetector::initCamera()
{
    int width = 848; int height = 480; int fps = 30;
    // D455 권장 해상도(848x480) 사용. 해상도가 높을수록 근거리 뎁스 품질에 유리합니다.
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);

    // 1. 파이프라인 시작 및 프로필 획득
    rs2::pipeline_profile profile = pipe.start(cfg);

    // --- 2. 하드웨어 센서 옵션 강제 변경 (High Accuracy 프리셋) ---
    rs2::device dev = profile.get_device();
    rs2::depth_sensor depth_sensor = dev.first<rs2::depth_sensor>();

    if (depth_sensor.supports(RS2_OPTION_VISUAL_PRESET)) {
        // RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY 적용 (노이즈 억제력 최대화)
        depth_sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_DENSITY);
        std::cout << "카메라에 High Accuracy 프리셋이 성공적으로 적용되었습니다." << std::endl;
    }

    if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, 150.0f);
        std::cout << "레이저 파워가 150으로 설정되었습니다." << std::endl;
    }

    // 드럼이 0.4~0.6m 사이만 남깁니다. (단위: 미터)
    thresh_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.4f);
    thresh_filter.set_option(RS2_OPTION_MAX_DISTANCE, 0.6f);
    
    // Decimation Filter: 해상도를 낮춰 연산량 감소 및 기본 노이즈 완화
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.0f); 
    // 기본값: 2.0 (해상도를 1/2로 줄임, 848x480 -> 424x240)

    // Spatial Filter: 평면(드럼 헤드)을 펴주고 엣지를 보존
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 1.0f); 
    // 기본값: 0.5 (1.0이면 필터링 없음, 낮을수록 스무딩 강함)
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f); 
    // 기본값: 20.0 (엣지를 판단하는 임계값. 드럼 곡면을 펴기 위해 유지)
    spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 5.0f);    
    // 기본값: 2.0 (필터 반복 횟수)

    // Temporal Filter: 프레임 누적을 통한 시간적 노이즈(깜빡임) 제거
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.2f); 
    // 기본값: 0.4 (낮을수록 이전 프레임의 가중치가 높아짐. 드럼이 정지 상태라면 0.1~0.2도 좋음)
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f); 
    // 기본값: 20.0
    temp_filter.set_option(RS2_OPTION_HOLES_FILL, 8.0f);          
    // 기본값: 3.0 (과거 데이터를 기반으로 구멍을 유지하는 정도)

    // // Hole Filling Filter: 누락된 뎁스 픽셀 메우기
    // hole_filter.set_option(RS2_OPTION_HOLES_FILL, 1.0f); 
    // // 기본값: 1.0 (0=왼쪽 픽셀 사용, 1=주변 픽셀 중 가장 먼 값, 2=주변 픽셀 중 가장 가까운 값)

}

pcl::PointCloud<pcl::PointXYZ>::Ptr DrumDetector::convertRs2PointsToPcl(const rs2::points& points, const rs2::video_stream_profile& profile)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    const size_t n = points.size();
    cloud->points.reserve(n);
    auto ptr = points.get_vertices();
    for (size_t i = 0; i < n; ++i) {
        const float x = ptr[i].x, y = ptr[i].y, z = ptr[i].z;
        // 무효 픽셀(깊이 0) 및 NaN 제거 — RANSAC 동일점 클러스터 방지
        if (z <= 0.0f || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            continue;
        cloud->points.emplace_back(x, y, z);
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

rs2::depth_frame DrumDetector::applyFilters(const rs2::frameset& frames)
{
    rs2::frame filtered = frames.get_depth_frame();

    filtered = thresh_filter.process(filtered);
    // filtered = dec_filter.process(filtered);           // 1. 다운샘플링
    filtered = depth_to_disparity.process(filtered);   // 2. Depth -> Disparity 변환 (필수)
    filtered = spat_filter.process(filtered);          // 3. 공간 필터
    filtered = temp_filter.process(filtered);          // 4. 시간 필터
    filtered = disparity_to_depth.process(filtered);   // 5. Disparity -> Depth 복구 (필수)
    // filtered = hole_filter.process(filtered);          // 6. 구멍 메우기 

    return filtered.as<rs2::depth_frame>();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DrumDetector::removeOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(pointcloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.7);
    sor.filter(*filtered);

    return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DrumDetector::downSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(pointcloud);
    vg.setLeafSize(0.005f, 0.005f, 0.005f);
    vg.filter(*filtered);

    return filtered;
}

std::vector<pcl::PointIndices> DrumDetector::extractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud)
{
    std::vector<pcl::PointIndices> cluster_indices;

    if (!pointcloud || pointcloud->empty())
    {
        return cluster_indices;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(pointcloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.007);
    ec.setMinClusterSize(300);
    ec.setMaxClusterSize(50000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(pointcloud);
    ec.extract(cluster_indices);

    return cluster_indices;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DrumDetector::transform2RobotFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, float waist_angle_rad)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Matrix4f T_cam2robot = makeCam2RobotMatrix(waist_angle_rad);
    pcl::transformPointCloud(*pointcloud, *transformed, T_cam2robot);
    
    return transformed;
}

Eigen::Matrix4f DrumDetector::makeCam2RobotMatrix(float waist_angle_rad)
{
    // 카메라로 얻은 점군을 로봇 프레임으로 변환하기 위한 동차 변환 행렬 계산
    // rot은 지면으로부터 카메라 설치 높이 및 허리 회전 각도에 따른 동적 좌표계
    Eigen::Matrix4f T_cam2rot, T_rot2robot, T_cam2robot;

    T_cam2rot << -0.01219f,  0.999661f, -0.02301f,  0.004527f,
                  0.767679f, 0.024104f,  0.640381f, 0.15327f,
                  0.640719f,-0.00986f,  -0.76771f, -0.08879f,
                  0.0f,      0.0f,       0.0f,      1.0f;

    T_rot2robot << cos(waist_angle_rad), -sin(waist_angle_rad), 0, 0,
                   sin(waist_angle_rad), cos(waist_angle_rad), 0, 0,
                   0, 0, 1, 1.129,
                   0, 0, 0, 1;

    T_cam2robot = T_rot2robot * T_cam2rot;

    return T_cam2robot;
}

pcl::PointCloud<pcl::PointNormal>::Ptr DrumDetector::addNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setInputCloud(pointcloud);
    pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(20);
    ne.compute(*normal);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normal(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*pointcloud, *normal, *cloud_with_normal);

    return cloud_with_normal;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DrumDetector::registration(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pointclouds)
{
    const size_t N = pointclouds.size();
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);

    if (N == 0)
        return merged;
    if (N == 1)
    {
        *merged = *pointclouds[0];
        return merged;
    }

    // 1) 노멀 부여
    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> world_with_normals(N);
    for (size_t i = 0; i < N; ++i)
        world_with_normals[i] = addNormals(pointclouds[i]);

    // 2) 중간 프레임(center)을 기준으로 앞/뒤 방향으로 순차 ICP 누적
    const size_t center = N / 2;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
        icp_poses(N, Eigen::Matrix4f::Identity());

    auto run_icp = [](const pcl::PointCloud<pcl::PointNormal>::Ptr& src, const pcl::PointCloud<pcl::PointNormal>::Ptr& tgt) -> Eigen::Matrix4f
    {
        pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
        icp.setTransformationEstimation(
            pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal>::Ptr(
                new pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal>()));
        icp.setInputSource(src);
        icp.setInputTarget(tgt);
        icp.setMaxCorrespondenceDistance(0.05);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-9);
        icp.setEuclideanFitnessEpsilon(1e-8);

        pcl::PointCloud<pcl::PointNormal>::Ptr aligned(new pcl::PointCloud<pcl::PointNormal>);
        icp.align(*aligned);

        if (!icp.hasConverged())
        {
            std::cerr << "  [Registration] ICP 수렴 실패. Identity 사용" << std::endl;
            return Eigen::Matrix4f::Identity();
        }
        std::cout << "  [Registration] fitness=" << icp.getFitnessScore() << std::endl;
        return icp.getFinalTransformation();
    };

    // center -> center+1, center+2, ...
    for (size_t i = center; i + 1 < N; ++i)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr tgt(new pcl::PointCloud<pcl::PointNormal>);
        pcl::transformPointCloud(*world_with_normals[i], *tgt, icp_poses[i]);
        Eigen::Matrix4f T_icp = run_icp(world_with_normals[i + 1], tgt);
        icp_poses[i + 1] = T_icp;
    }

    // center -> center-1, center-2, ...
    for (size_t i = center; i > 0; --i)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr tgt(new pcl::PointCloud<pcl::PointNormal>);
        pcl::transformPointCloud(*world_with_normals[i], *tgt, icp_poses[i]);
        Eigen::Matrix4f T_icp = run_icp(world_with_normals[i - 1], tgt);
        icp_poses[i - 1] = T_icp;
    }

    // 3) 보정치를 적용해 모든 점군을 center 좌표계로 병합
    for (size_t i = 0; i < N; ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*pointclouds[i], *transformed, icp_poses[i]);
        *merged += *transformed;
    }

    std::cout << "[Registration] 정합 후 병합 점군 크기: " << merged->size() << " 점" << std::endl;
    return merged;
}

void DrumDetector::detectCircles(pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud,
                                std::vector<pcl::ModelCoefficients::Ptr>& drum_coeffs, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& drum_clouds)
{
    std::vector<Eigen::Vector4f> found_centers;

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.03);
    seg.setMaxIterations(5000);
    seg.setRadiusLimits(0.100, 0.205); // 드럼 반경 제약

    for (int pass = 0; pass < 2; ++pass)
    {
        std::cout << "  [pass= " << pass << "]" << std::endl;
        if (!pointcloud || pointcloud->empty())
            return;

        pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        seg.setInputCloud(pointcloud);
        seg.segment(*inliers, *coeffs);


        if (inliers->indices.size() <= 200 || coeffs->values.size() < 7)
        // continue;
            return;

        std::cout << " i found!! \n";
        std::cout << "   inliers=" << inliers->indices.size() << " coeffs=" << coeffs->values.size() << std::endl;

        Eigen::Vector4f center(coeffs->values[0], coeffs->values[1], coeffs->values[2], 0.0f);
        std::cout << "   center ok" << std::endl;
        std::cout << "   x = " << coeffs->values[0] << " / y = " << coeffs->values[1] << " / z = " << coeffs->values[2] << "\n"
                    << " / r = " << coeffs->values[3] << "\n"
                    << " / nx = " << coeffs->values[4] << " / ny = " << coeffs->values[5] << " / nz = " << coeffs->values[6] << std::endl;

        bool far_enough = true;

        for (const auto& prev : found_centers)
        {
            float dist = (center - prev).head<3>().norm();
            if (dist < 0.15f)
            {
                far_enough = false;
                break;
            }
        }

        if (!far_enough)
            // continue;
            return;

        found_centers.push_back(center);
        drum_coeffs.push_back(coeffs);

        pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*pointcloud, *inliers, *inlier_cloud);

        drum_clouds.push_back(inlier_cloud);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(pointcloud);
        extract.setIndices(inliers);
        extract.setNegative(true);

        pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*remaining);
        pointcloud.swap(remaining);
    }
    std::cout << "   detectCircles exiting" << std::endl;
}

std::vector<int> DrumDetector::indexCircles(std::vector<pcl::ModelCoefficients::Ptr>& drum_coeffs, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& drum_clouds)
{
    const std::vector<int> pos_to_id = {5, 8, 1, 4, 2, 3, 6, 7};
    const float x_group_threshold = 0.1f;
    const size_t n = drum_coeffs.size();

    // 1단계: x 오름차순 인덱스 정렬
    std::vector<size_t> order(n);
    for (size_t i = 0; i < n; ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return drum_coeffs[a]->values[0] < drum_coeffs[b]->values[0];
    });

    // 2단계: x가 가까운 그룹 내에서 z 오름차순 정렬
    size_t i = 0;
    while (i < n) {
        size_t j = i + 1;
        while (j < n &&
               std::abs(drum_coeffs[order[j]]->values[0] -
                        drum_coeffs[order[i]]->values[0]) < x_group_threshold) {
            ++j;
        }
        std::sort(order.begin() + i, order.begin() + j, [&](size_t a, size_t b) {
            return drum_coeffs[a]->values[2] < drum_coeffs[b]->values[2];
        });
        i = j;
    }

    // 3단계: pos_to_id 역매핑 — drum_id k가 x/z 정렬에서 몇 번째 위치인지
    std::vector<size_t> drum_id_order(n);
    for (size_t idx = 0; idx < n; ++idx) {
        int target_id = static_cast<int>(idx) + 1;
        for (size_t pos = 0; pos < n; ++pos) {
            if (pos < pos_to_id.size() && pos_to_id[pos] == target_id) {
                drum_id_order[idx] = pos;
                break;
            }
        }
    }

    // 4단계: drum_id 오름차순으로 1회 복사
    std::vector<pcl::ModelCoefficients::Ptr> sorted_coeffs(n);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> sorted_clouds(n);
    std::vector<int> drum_ids(n);
    for (size_t idx = 0; idx < n; ++idx) {
        sorted_coeffs[idx] = drum_coeffs[order[drum_id_order[idx]]];
        sorted_clouds[idx] = drum_clouds[order[drum_id_order[idx]]];
        drum_ids[idx] = static_cast<int>(idx) + 1;
        std::cout << "[indexCircles] drum_id=" << drum_ids[idx]
                  << "  x=" << sorted_coeffs[idx]->values[0]
                  << "  z=" << sorted_coeffs[idx]->values[2] << std::endl;
    }
    drum_coeffs = sorted_coeffs;
    drum_clouds = sorted_clouds;
    return drum_ids;
}

std::vector<Eigen::VectorXd> DrumDetector::selectCandidateForOneCircle(const pcl::ModelCoefficients::Ptr& coeffs, char DB)
{
    std::vector<Eigen::VectorXd> candidate;

    Eigen::Vector3f center(coeffs->values[0], coeffs->values[1], coeffs->values[2]);
    float radius = coeffs->values[3];
    Eigen::Vector3f normal(coeffs->values[4], coeffs->values[5], coeffs->values[6]);
    normal.normalize();

    Eigen::Vector3f seed(1.0f, 0.0f, 0.0f);
    if (std::abs(normal.dot(seed)) > 0.9f)
        seed = Eigen::Vector3f(0.0f, 1.0f, 0.0f);

    Eigen::Vector3f u = seed.cross(normal).normalized();
    Eigen::Vector3f v = normal.cross(u).normalized();

    if (DB == 'D')
    {
        int numRings = 2;
        int numAngles = 4;
        candidate.reserve(1 + numRings * numAngles);

        candidate.push_back(center.cast<double>());

        for (int k = 1; k <= numRings; ++k)
        {
            float rk = radius * 0.4f * static_cast<float>(k);
            for (int j = 0; j < numAngles; ++j)
            {
                float theta = 2.0f * M_PI * static_cast<float>(j) / static_cast<float>(numAngles);
                Eigen::Vector3f point = center + rk * (std::cos(theta) * u + std::sin(theta) * v);
                candidate.push_back(point.cast<double>());
            }
        }
    }
    else
    {
        int numRings = 3;
        int numAngles = 3;
        candidate.reserve(numRings * numAngles);

        Eigen::Vector2f dirToCenter(center.x(), center.y());

        for (int k = 1; k <= numRings; ++k)
        {
            float rk = radius * static_cast<float>(k) / static_cast<float>(numRings + 1);
            for (int j = 0; j < numAngles; ++j)
            {
                float theta = 2.0f * M_PI * static_cast<float>(j) / static_cast<float>(numAngles);
                Eigen::Vector3f point = center + rk * (std::cos(theta) * u + std::sin(theta) * v);

                Eigen::Vector2f offset(point.x() - center.x(), point.y() - center.y());
                if (offset.dot(dirToCenter) >= 0)
                    continue;

                candidate.push_back(point.cast<double>());
            }
        }
    }

    return candidate;
}

// 벨류 악기들은 후보군 선정 방식 변경해야함.
// 타격 후보군들 시각화.
std::vector<std::vector<Eigen::VectorXd>> DrumDetector::selectCandidates(const std::vector<pcl::ModelCoefficients::Ptr>& drum_coeffs)
{
    std::vector<std::vector<Eigen::VectorXd>> all_candidates;
    all_candidates.reserve(drum_coeffs.size());

    for (size_t i = 0; i < drum_coeffs.size(); ++i)
    {
        char DB = (i < 5) ? 'D' : 'B'; // D: 드럼, B: 벨류
        std::vector<Eigen::VectorXd> candidates = selectCandidateForOneCircle(drum_coeffs[i], DB);
        std::cout << "[HitCandidates] drum " << i << ": " << candidates.size() << " candidates generated" << std::endl;
        all_candidates.push_back(candidates);
    }

    return all_candidates;
}

void DrumDetector::visualizeDrums(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& drum_clouds,
                                  const std::vector<std::vector<Eigen::VectorXd>>& drum_candidates)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("Drum Detector"));
    viewer->setBackgroundColor(0.1, 0.1, 0.1);
    viewer->addCoordinateSystem(0.1);
    viewer->initCameraParameters();

    // 악기마다 구분되는 색상 팔레트
    const int palette[][3] = {
        {255, 80,  80 },  // red
        {80,  255, 80 },  // green
        {80,  140, 255},  // blue
        {255, 220, 60 },  // yellow
        {255, 120, 255},  // magenta
        {80,  255, 240},  // cyan
        {255, 170, 70 },  // orange
        {180, 110, 255},  // purple
    };
    const int palette_size = sizeof(palette) / sizeof(palette[0]);

    for (size_t i = 0; i < drum_clouds.size(); ++i)
    {
        if (!drum_clouds[i] || drum_clouds[i]->empty())
            continue;

        const int* c = palette[i % palette_size];
        const std::string id = "drum_" + std::to_string(i);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            color_handler(drum_clouds[i], c[0], c[1], c[2]);
        viewer->addPointCloud<pcl::PointXYZ>(drum_clouds[i], color_handler, id);
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id);
    }

    for (size_t i = 0; i < drum_candidates.size(); ++i)
    {
        for (size_t j = 0; j < drum_candidates[i].size(); ++j)
        {
            const Eigen::VectorXd& pt = drum_candidates[i][j];
            pcl::PointXYZ p;
            p.x = static_cast<float>(pt(0));
            p.y = static_cast<float>(pt(1));
            p.z = static_cast<float>(pt(2));

            std::string sphere_id = "cand_" + std::to_string(i) + "_" + std::to_string(j);
            viewer->addSphere(p, 0.005, 255.0, 255.0, 255.0, sphere_id);
        }
    }

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 완전 종료 — VTK 리소스까지 정리
    // viewer->removeAllPointClouds();
    // viewer->removeAllShapes();
    // viewer->getRenderWindow()->Finalize();
    viewer->close();
    viewer.reset();
}

void DrumDetector::detectDrums()
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_clouds;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // std::vector<float> angles = {30.0f, 20.0f, 10.0f, 0.0f, -10.0f, -20.0f, -30.0f};
    std::vector<float> angles = {0.0f};
    for (float angle : angles) {
        std::cout << angle << "도 위치로 로봇 이동..." << std::endl;
        testManager.move_waist(angle);

        float c_MotorAngle[12] = {0};
        testManager.getMotorPos(c_MotorAngle);

        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        int warmup_frames = 30;
        for (int i = 0; i < warmup_frames; i++)
        {
            rs2::frameset frames = pipe.wait_for_frames();
            frames = align_to_color.process(frames);
            rs2::depth_frame depth_frame = applyFilters(frames);
        }
        rs2::frameset frames = pipe.wait_for_frames();
        frames = align_to_color.process(frames);
        rs2::depth_frame final_frame = applyFilters(frames);

        rs2::pointcloud pc;
        rs2::points points = pc.calculate(final_frame);
        auto depth_stream = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        temp_cloud = convertRs2PointsToPcl(points, depth_stream);

        filtered_cloud = removeOutliers(temp_cloud);
        filtered_cloud = downSampling(filtered_cloud);
        filtered_cloud = transform2RobotFrame(filtered_cloud, c_MotorAngle[0]);

        accumulated_clouds.push_back(filtered_cloud);
    }

    bool use_registration = false;
    pcl::PointCloud<pcl::PointXYZ>::Ptr full_clouds(new pcl::PointCloud<pcl::PointXYZ>);
    if (use_registration)
    {
        full_clouds = registration(accumulated_clouds);
    }
    else
    {
        for (const auto& c : accumulated_clouds)
            *full_clouds += *c;
    }

    savePointsToCSV("test", full_clouds);
    visualizeDrums({full_clouds});

    std::vector<pcl::PointIndices> cluster_indices;
    cluster_indices = extractClusters(full_clouds);

    std::vector<pcl::ModelCoefficients::Ptr> drum_coeffs;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> drum_clouds;

    int iter = 0;
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*full_clouds, indices, *temp_cluster);
        iter++;
        std::cout << iter << "번째 클러스터" << std::endl;
        detectCircles(temp_cluster, drum_coeffs, drum_clouds);
    }
    
    std::vector<int> drum_ids = indexCircles(drum_coeffs, drum_clouds);

    std::vector<std::vector<Eigen::VectorXd>> drum_candidates = selectCandidates(drum_coeffs);
    pathManager.hit_Candidates = drum_candidates;

    savePointsToCSV("test_final", drum_clouds[0]);
    saveCandidatesToCSV("drum_candidates", drum_candidates);
    visualizeDrums(drum_clouds, drum_candidates);
}

// utils
void DrumDetector::savePointsToCSV(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // std::ios::out은 기본적으로 파일을 새로 생성하거나 기존 내용을 지우고 새로 씁니다.
    
    std::string fullPath = "../../DepthCamera/pcd/" + filename + ".txt";
    std::ofstream file(fullPath);

    if (!file.is_open())
    {
        std::cerr << "파일을 열 수 없습니다: " << fullPath << std::endl;
        return;
    }

    file << std::fixed << std::setprecision(6);     // 소수점 정밀도 설정

    for (const auto& point : cloud->points)
    {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            file << point.x << "," << point.y << "," << point.z << "\n";
        }
    }

    file.close();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DrumDetector::loadPointsFromCSV(const std::string& filename)
{    
    std::string fullPath = "../../DepthCamera/pcd/" + filename + ".txt";
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::ifstream file(fullPath);
    
    if (!file.is_open())
    {
        std::cerr << "파일을 열 수 없습니다: " << fullPath << std::endl;
        return nullptr;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string x_str, y_str, z_str;
        
        // 쉼표로 구분된 x, y, z 파싱
        if (std::getline(ss, x_str, ',') && 
            std::getline(ss, y_str, ',') && 
            std::getline(ss, z_str, ',')) {
            
            pcl::PointXYZ point;
            point.x = std::stof(x_str);
            point.y = std::stof(y_str);
            point.z = std::stof(z_str);
            cloud->points.push_back(point);
        }
    }
    
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    return cloud;
}

void DrumDetector::saveCandidatesToCSV(const std::string& filename, const std::vector<std::vector<Eigen::VectorXd>>& drum_candidates)
{
    std::string fullPath = "../../DepthCamera/pcd/" + filename + ".txt";
    std::ofstream file(fullPath);

    if (!file.is_open())
    {
        std::cerr << "파일을 열 수 없습니다: " << fullPath << std::endl;
        return;
    }

    file << std::fixed << std::setprecision(6);

    for (size_t i = 0; i < drum_candidates.size(); ++i)
    {
        int drum_id = static_cast<int>(i) + 1;
        for (size_t j = 0; j < drum_candidates[i].size(); ++j)
        {
            const Eigen::VectorXd& pt = drum_candidates[i][j];
            file << drum_id << "," << pt(0) << "," << pt(1) << "," << pt(2) << "\n";
        }
    }

    file.close();
    std::cout << "[saveCandidatesToCSV] " << fullPath << " 저장 완료" << std::endl;
}
