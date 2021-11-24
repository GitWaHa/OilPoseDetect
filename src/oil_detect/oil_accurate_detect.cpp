#include <fstream>

#include "oil_detect/oil_accurate_detect.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h> //体素滤波相关

OilAccurateDetect::OilAccurateDetect()
    : cloud_(new PCLPointCloud), cloud_voxel_(new PCLPointCloud), oil_cloud_(new PCLPointCloud),
      plane_cloud_(new PCLPointCloud), not_plane_cloud_(new PCLPointCloud)
{
}

OilAccurateDetect::~OilAccurateDetect()
{
}

int OilAccurateDetect::detect_once(const PCLPointCloud::Ptr cloud)
{
    cloud_ = cloud;
    if (poseDetect() != 0)
        return -1;

    return 0;
}

void OilAccurateDetect::saveDataFrame(const std::string save_folder, const std::string save_num)
{
    std::string cloud_path = save_folder + "/frame_" + save_num + "_reconstruct_cloud" + ".pcd";
    pcl::io::savePCDFile(cloud_path, *cloud_);

    std::string oil_cloud_path = save_folder + "/frame_" + save_num + "_oil_cloud" + ".pcd";
    pcl::io::savePCDFile(oil_cloud_path, *oil_cloud_);

    std::string plane_cloud_path = save_folder + "/frame_" + save_num + "_plane_cloud" + ".pcd";
    pcl::io::savePCDFile(plane_cloud_path, *plane_cloud_);

    std::string notplane_cloud_path = save_folder + "/frame_" + save_num + "_notplane_cloud" + ".pcd";
    pcl::io::savePCDFile(notplane_cloud_path, *not_plane_cloud_);
}

void OilAccurateDetect::planeToQuat(pcl::ModelCoefficients coef, float *quat)
{
    // 根据平面法线（z轴），求x，y轴
    Eigen::Vector3d z(coef.values[0], coef.values[1], coef.values[2]);
    Eigen::Vector3d v(0, 0, -1.0);
    auto x = z.cross(v);
    auto y = z.cross(x);

    // 原始坐标系分别绕x轴和y轴旋转后, 使z轴与平面法向量平行, 作为中心点处坐标系
    Eigen::Matrix3d rot_matrix(3, 3);
    rot_matrix.col(0) << x;
    rot_matrix.col(1) << y;
    rot_matrix.col(2) << z;

    Eigen::Quaterniond quat_eigen(rot_matrix);
    quat[0] = quat_eigen.x();
    quat[1] = quat_eigen.y();
    quat[2] = quat_eigen.z();
    quat[3] = quat_eigen.w();
}

int OilAccurateDetect::poseDetect()
{
    // 体素化点云
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_); //给滤波对象设置需过滤的点云
    float grid_size = 0.001f;
    sor.setLeafSize(grid_size, grid_size, grid_size); //设置滤波时创建的体素大小
    sor.filter(*cloud_voxel_);                        //执行滤波处理

    /// 平面拟合 // 平面方程: ax+by+cz+d = 0
    std::vector<int> indices(0);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setInputCloud(cloud_voxel_);
    // seg.setAxis();

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.005);
    seg.setMaxIterations(10000);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() < 100)
    {
        std::cout << "[OilAccurateDetect] "
                  << "plane point nuw is too less!" << std::endl;
        return -1;
    }
    std::cout << "[OilAccurateDetect] "
              << "平面局内点数：" << inliers->indices.size() << std::endl;

    // oil 旋转四元数
    planeToQuat(*coefficients, oil_quat_);
    std::cout << "[OilAccurateDetect] "
              << "oil_quat_: " << oil_quat_[0] << "," << oil_quat_[1] << "," << oil_quat_[2] << "," << oil_quat_[3] << std::endl;

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_voxel_);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(indices);

    extract.setInputCloud(cloud_voxel_);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane_cloud_);

    auto not_plane_idx = extractAbovePlane(cloud_voxel_,
                                           coefficients->values[0], coefficients->values[1],
                                           coefficients->values[2], coefficients->values[3],
                                           0.01, 0.1);
    inliers->indices = not_plane_idx;

    extract.setInputCloud(cloud_voxel_);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*not_plane_cloud_);

    if (inliers->indices.size() > 100)
    {
        // *********** 圆模型
        pcl::SACSegmentation<pcl::PointXYZ> seg_circle;
        pcl::ModelCoefficients coefficients_circle;
        seg_circle.setInputCloud(not_plane_cloud_);
        // seg_circle.setIndices(inliers);
        seg_circle.setOptimizeCoefficients(true);
        seg_circle.setModelType(pcl::SACMODEL_CIRCLE3D);
        seg_circle.setRadiusLimits(0.04, 0.05);
        seg_circle.setMethodType(pcl::SAC_RANSAC);
        seg_circle.setDistanceThreshold(0.01);
        seg_circle.setMaxIterations(1000);
        seg_circle.segment(*inliers, coefficients_circle);

        extract.setInputCloud(not_plane_cloud_);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*oil_cloud_);

        // oil 平移
        oil_trans_[0] = coefficients_circle.values[0];
        oil_trans_[1] = coefficients_circle.values[1];
        oil_trans_[2] = coefficients_circle.values[2];
        std::cout << "[OilAccurateDetect] "
                  << "oil_trans_: " << oil_trans_[0] << "," << oil_trans_[1] << "," << oil_trans_[2] << std::endl;
    }

    return 0;
}

std::vector<int> OilAccurateDetect::extractAbovePlane(PCLPointCloud::Ptr cloud, float A, float B, float C, float D, float min_dis, float max_dis)
{
    std::vector<int> idx;

    auto down = std::sqrt(std::pow(A, 2) + std::pow(B, 2) + std::pow(C, 2));

    for (int i = 0; i < cloud->size(); i++)
    {
        auto point = cloud->at(i);
        // 计算距离
        auto dis = abs(A * point.x + B * point.y + C * point.z + D) / down;
        if (dis >= min_dis && dis <= max_dis)
        {
            idx.push_back(i);
        }
    }

    return idx;
}
