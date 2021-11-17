#pragma once

#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

class OilRoughDetect
{
public:
    using PCLPointRGB = pcl::PointXYZRGBA;
    using PCLPoint = pcl::PointXYZ;
    using PCLPointCloudRGB = pcl::PointCloud<PCLPointRGB>;
    using PCLPointCloud = pcl::PointCloud<PCLPoint>;

public:
    OilRoughDetect(std::string color_frame);
    ~OilRoughDetect();

    int detect_once(const cv::Mat &color, const cv::Mat &depth, const PCLPointCloudRGB::Ptr cloud, float *oil_pose);

    void saveDataFrame(const std::string save_folder, const std::string save_num);

    float *getPositionInCamera()
    {
        return oil_pos_in_camera_;
    }

    float *getPositionInWorld()
    {
        return oil_pos_in_world_;
    }

private:
    int roiDetect();
    int getPosFromRoi();

    void computerMeanValue(PCLPointCloud::Ptr cloud, std::vector<int> indices, float *pos);
    void computerMeanValue(PCLPointCloud::Ptr cloud, float *pos);

    void getCameraPose(std::string source_frame, std::string target_frame);
    void saveCameraPose(std::string save_path);

    void convertPosToWorld(float *oil_pos_in_camera, float *oil_pos_in_world);

private:
    /* data */
    float oil_pos_in_camera_[3];
    float oil_pos_in_world_[3];
    std::string color_frame_;

    tf::StampedTransform camera_pose_;

    cv::Mat color_;
    cv::Mat color_draw_;
    cv::Mat depth_;

    PCLPointCloudRGB::Ptr cloud_;

    cv::Rect oil_roi_;
};
