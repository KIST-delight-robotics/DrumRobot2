#pragma once

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <string>
#include <cmath>
#include <Eigen/Dense>

#include "../include/managers/TestManager.hpp"

// realsense 헤더
#include <librealsense2/rs.hpp>
// Point Cloud Libarary 헤더
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>

class DrumDetector
{
public:

    DrumDetector(PathManager &pathManagerRef, TestManager &testManagerRef);
    ~DrumDetector();

    void detectDrums();

    // utils
    void savePointsToCSV(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointsFromCSV(const std::string& filename);
    void saveCandidatesToCSV(const std::string& filename, const std::vector<std::vector<Eigen::VectorXd>>& drum_candidates);

private:
    PathManager &pathManager;
    TestManager &testManager;

    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::align align_to_color;
    rs2::threshold_filter thresh_filter;
    rs2::decimation_filter dec_filter;
    rs2::spatial_filter spat_filter;
    rs2::temporal_filter temp_filter;
    rs2::hole_filling_filter hole_filter;
    rs2::disparity_transform depth_to_disparity;  // 생성자에서 true로 초기화
    rs2::disparity_transform disparity_to_depth;  // 생성자에서 false로 초기화

    void initCamera();
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertRs2PointsToPcl(const rs2::points& points, const rs2::video_stream_profile& profile);
    rs2::depth_frame applyFilters(const rs2::frameset& frames);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud);
    std::vector<pcl::PointIndices> extractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transform2RobotFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, float waist_angle_rad);
    Eigen::Matrix4f makeCam2RobotMatrix(float waist_angle_rad);
    pcl::PointCloud<pcl::PointNormal>::Ptr addNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr registration(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& world_clouds);
    void detectCircles(pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud,
                      std::vector<pcl::ModelCoefficients::Ptr>& drum_coeffs,
                      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& drum_clouds);
    std::vector<int> indexCircles(std::vector<pcl::ModelCoefficients::Ptr>& drum_coeffs, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& drum_clouds);
    std::vector<Eigen::VectorXd> selectCandidateForOneCircle(const pcl::ModelCoefficients::Ptr& coeffs, char DB);
    std::vector<std::vector<Eigen::VectorXd>> selectCandidates(const std::vector<pcl::ModelCoefficients::Ptr>& drum_coeffs);
    void visualizeDrums(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& drum_clouds,
                        const std::vector<std::vector<Eigen::VectorXd>>& drum_candidates = {});
};
