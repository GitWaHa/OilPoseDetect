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
    OilRoughDetect(/* args */);
    ~OilRoughDetect();

    int detect_once(const cv::Mat &color, const cv::Mat &depth, const PCLPointCloudRGB::Ptr cloud, float *oil_pose);

    void saveDataFrame(const std::string save_folder, const std::string save_num);

    float *getPosition()
    {
        return oil_pos_;
    }

private:
    int roiDetect();
    int getPosFromRoi();
    void getCameraPose(std::string source_frame, std::string target_frame, tf::StampedTransform &transform, std::string save_path = "");

    void computerMeanValue(PCLPointCloud::Ptr cloud, float *pos);

private:
    /* data */
    float oil_pos_[3];

    cv::Mat color_;
    cv::Mat color_draw_;
    cv::Mat depth_;

    PCLPointCloudRGB::Ptr cloud_;

    cv::Rect oil_roi_;
};
