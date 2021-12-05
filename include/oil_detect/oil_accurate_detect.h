#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

class OilAccurateDetect
{
public:
    using PCLPointRGB = pcl::PointXYZRGBA;
    using PCLPoint = pcl::PointXYZ;
    using PCLPointCloudRGB = pcl::PointCloud<PCLPointRGB>;
    using PCLPointCloud = pcl::PointCloud<PCLPoint>;

public:
    OilAccurateDetect(/* args */);
    ~OilAccurateDetect();

    int detect_once(const PCLPointCloud::Ptr cloud);

    void saveDataFrame(const std::string save_folder, const std::string save_num);

    const float *getPosition()
    {
        return oil_trans_;
    }

    const float *getQuaternion()
    {
        return oil_quat_;
    }

    const PCLPointCloud::Ptr getOriginalCloud()
    {
        return cloud_;
    }

    const PCLPointCloud::Ptr getVoxelCloud()
    {
        return cloud_voxel_;
    }

    const PCLPointCloud::Ptr getPlaneCloud()
    {
        return plane_cloud_;
    }

    const PCLPointCloud::Ptr getNotPlaneCloud()
    {
        return not_plane_cloud_;
    }

    const PCLPointCloud::Ptr getOilCloud()
    {
        return oil_cloud_;
    }

private:
    int poseDetect();

    void planeToQuat(pcl::ModelCoefficients coef, float *quat);

    std::vector<int> extractAbovePlane(PCLPointCloud::Ptr cloud, float A, float B, float C, float D, float min_dis, float max_dis);

    std::vector<float> correctionNormalDirection(PCLPointCloud::Ptr cloud, float A, float B, float C, float D);

private:
    /* data */
    float oil_trans_[3];
    float oil_quat_[4];

    PCLPointCloud::Ptr cloud_;
    PCLPointCloud::Ptr cloud_voxel_;
    PCLPointCloud::Ptr oil_cloud_;
    PCLPointCloud::Ptr plane_cloud_;
    PCLPointCloud::Ptr not_plane_cloud_;
};
