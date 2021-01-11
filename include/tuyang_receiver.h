#include <utility>

#pragma once
#include <string>
#include <utility>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "camera_receiver.h"

#define R2_DEFAULT_NS "camera"

#define R2_TOPIC_IMAGE_COLOR "/rgb/image_rect_color"
#define R2_TOPIC_IMAGE_DEPTH "/depth/image_raw"
#define R2_TOPIC_ALIGNED_DEPTH "/depth_registered/image_raw"

#define R2_TOPIC_COMPRESSED "/compressed"
#define R2_TOPIC_INFO "/camera_info"

class TuYangCameraReceiver : public CameraReceiver
{
public:
    TuYangCameraReceiver(const ros::NodeHandle &node, std::string topicColor, std::string topicDepth, const bool useExact, const bool useCompressed, int rate)
        : CameraReceiver(node, topicColor, topicDepth, useExact, useCompressed, rate)
    {
        std::cout << "TuYangCameraReceiver init" << std::endl;
    };

protected:
    virtual void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
    {
        const float badPoint = std::numeric_limits<float>::quiet_NaN();

#pragma omp parallel for
        for (int r = 0; r < depth.rows; ++r)
        {
            pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
            const uint16_t *itD = depth.ptr<uint16_t>(r);
            const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
            const float y = getLookupX().at<float>(0, r);
            const float *itX = getLookupY().ptr<float>();

            for (size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
            {
                register const float depthValue = *itD / 4000.0f;
                // Check for invalid measurements
                if (*itD == 0)
                {
                    // not valid
                    itP->x = itP->y = itP->z = badPoint;
                    itP->rgba = 0;
                    continue;
                }
                itP->z = depthValue;
                itP->x = *itX * depthValue;
                itP->y = y * depthValue;
                itP->b = 255;
                itP->g = 255;
                itP->r = 255;
                itP->a = 255;
            }
        }
    }

private:
};
