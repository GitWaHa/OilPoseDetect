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
#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "camera_receiver.h"

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
        // alignDepth2RGB(depth, depth_align_rgb_);
        // std::cout << depth_align_rgb_.rows << "," << depth_align_rgb_.cols << endl;
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
                // std::cout << depthValue << endl;
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
                // itP->b = itC->val[0];
                // itP->g = itC->val[1];
                // itP->r = itC->val[2];
                // itP->a = 255;
            }
        }
    }
    cv::Mat getDepth2RGB()
    {
        return depth_align_rgb_;
    }

private:
    void alignDepth2RGB(const cv::Mat &src, cv::Mat &dst);
    void tfTrans(std::string source_frame, std::string target_grame, tf::StampedTransform &transform);
    void depth2RGB(const cv::Mat &src, cv::Mat &dst, const float *W);

    void StampedTfToArray(const tf::StampedTransform &transform, float *W);

    cv::Mat depth_align_rgb_;
};

void TuYangCameraReceiver::alignDepth2RGB(const cv::Mat &src, cv::Mat &dst)
{
    tf::Matrix3x3 roat;
    tf::Quaternion quat;

    // 深度坐标系转彩色坐标系
    tf::StampedTransform transform_m;
    tfTrans("camera_rgb_optical_frame", "camera_depth_optical_frame", transform_m);
    transform_m.setOrigin(tf::Vector3(50, 0, 0));

    // rgb内参
    tf::StampedTransform transform_krgb;
    roat.setValue(cameraMatrixColor.at<double>(0, 0), cameraMatrixColor.at<double>(0, 1), cameraMatrixColor.at<double>(0, 2),
                  cameraMatrixColor.at<double>(1, 0), cameraMatrixColor.at<double>(1, 1), cameraMatrixColor.at<double>(1, 2),
                  cameraMatrixColor.at<double>(2, 0), cameraMatrixColor.at<double>(2, 1), cameraMatrixColor.at<double>(2, 2));
    roat.getRotation(quat);
    transform_krgb.setOrigin(tf::Vector3(0, 0, 0));
    transform_krgb.setRotation(quat);
    std::cout << cameraMatrixColor.at<double>(0, 0) << " " << cameraMatrixColor.at<double>(0, 1) << " " << cameraMatrixColor.at<double>(0, 2) << endl;
    std::cout << cameraMatrixColor.at<double>(1, 0) << " " << cameraMatrixColor.at<double>(1, 1) << " " << cameraMatrixColor.at<double>(1, 2) << endl;
    std::cout << cameraMatrixColor.at<double>(2, 0) << " " << cameraMatrixColor.at<double>(2, 1) << " " << cameraMatrixColor.at<double>(2, 2) << endl;

    // depth内参
    tf::StampedTransform transform_kdepth;
    roat.setValue(cameraMatrixDepth.at<double>(0, 0), cameraMatrixDepth.at<double>(0, 1), cameraMatrixDepth.at<double>(0, 2),
                  cameraMatrixDepth.at<double>(1, 0), cameraMatrixDepth.at<double>(1, 1), cameraMatrixDepth.at<double>(1, 2),
                  cameraMatrixDepth.at<double>(2, 0), cameraMatrixDepth.at<double>(2, 1), cameraMatrixDepth.at<double>(2, 2));
    roat.getRotation(quat);
    transform_kdepth.setOrigin(tf::Vector3(0, 0, 0));
    transform_kdepth.setRotation(quat);
    std::cout << cameraMatrixDepth.at<double>(0, 0) << " " << cameraMatrixDepth.at<double>(0, 1) << " " << cameraMatrixDepth.at<double>(0, 2) << endl;
    std::cout << cameraMatrixDepth.at<double>(1, 0) << " " << cameraMatrixDepth.at<double>(1, 1) << " " << cameraMatrixDepth.at<double>(1, 2) << endl;
    std::cout << cameraMatrixDepth.at<double>(2, 0) << " " << cameraMatrixDepth.at<double>(2, 1) << " " << cameraMatrixDepth.at<double>(2, 2) << endl;

    tf::StampedTransform transform_tmp1, transform_tmp2;
    transform_tmp1.mult(transform_m, transform_kdepth.inverse());
    transform_tmp2.mult(transform_krgb, transform_tmp1);
    // transform_krgb *transform_m *transform_kdepth.inverse();

    std::cout
        << "transform" << transform_tmp2.getOrigin().getX() << endl;
    float W[4 * 4];
    StampedTfToArray(transform_tmp2, W);
    std::cout << W[0 * 4 + 0] << " " << W[0 * 4 + 1] << " " << W[0 * 4 + 2] << " " << W[0 * 4 + 3] << " " << endl;
    std::cout << W[1 * 4 + 0] << " " << W[1 * 4 + 1] << " " << W[1 * 4 + 2] << " " << W[1 * 4 + 3] << " " << endl;
    std::cout << W[2 * 4 + 0] << " " << W[2 * 4 + 1] << " " << W[2 * 4 + 2] << " " << W[2 * 4 + 3] << " " << endl;
    std::cout << W[3 * 4 + 0] << " " << W[3 * 4 + 1] << " " << W[3 * 4 + 2] << " " << W[3 * 4 + 3] << " " << endl;

    depth2RGB(src, dst, W);
}

void TuYangCameraReceiver::tfTrans(std::string source_frame, std::string target_grame, tf::StampedTransform &transform)
{
    tf::TransformListener listener;

    listener.waitForTransform(target_grame, source_frame, ros::Time(0), ros::Duration(3));
    listener.lookupTransform(target_grame, source_frame, ros::Time(0), transform);
}

void TuYangCameraReceiver::StampedTfToArray(const tf::StampedTransform &transform, float *W)
{
    tf::Matrix3x3 roat(transform.getRotation());
    W[0 * 4 + 0] = roat.getRow(0).getX();
    W[0 * 4 + 1] = roat.getRow(0).getY();
    W[0 * 4 + 2] = roat.getRow(0).getZ();
    W[0 * 4 + 3] = transform.getOrigin().getX();

    W[1 * 4 + 0] = roat.getRow(1).getX();
    W[1 * 4 + 1] = roat.getRow(1).getY();
    W[1 * 4 + 2] = roat.getRow(1).getZ();
    W[1 * 4 + 3] = transform.getOrigin().getY();

    W[2 * 4 + 0] = roat.getRow(2).getX();
    W[2 * 4 + 1] = roat.getRow(2).getY();
    W[2 * 4 + 2] = roat.getRow(2).getZ();
    W[2 * 4 + 3] = transform.getOrigin().getZ();

    W[3 * 4 + 0] = 0;
    W[3 * 4 + 1] = 0;
    W[3 * 4 + 2] = 0;
    W[3 * 4 + 3] = 1;
}

void TuYangCameraReceiver::depth2RGB(const cv::Mat &src, cv::Mat &dst, const float *W)
{
    double z;
    uint16_t u, v, d;
    uint16_t u_rgb, v_rgb;
    cv::Mat newdepth(src.rows, src.cols, CV_16UC1, cv::Scalar(0));
    for (v = 0; v < src.rows; v++)
    {
        for (u = 0; u < src.cols; u++)
        {
            d = src.at<uint16_t>(v, u);
            z = (double)d / 4000;
            u_rgb = (uint16_t)((W[0] * (double)u + W[1] * (double)v + W[2] + W[3] / z)); //参照上述公式（5）
            v_rgb = (uint16_t)((W[4] * (double)u + W[5] * (double)v + W[6] + W[7] / z)); //参照上述公式（6）
            // double z_rgb = (double)((W[3 * 4 + 0] * (double)u + W[3 * 4 + 1] * (double)v + W[3 * 4 + 2] + W[3 * 4 + 0] / z));
            // std::cout << u_rgb << "," << v_rgb << "std::endl";
            // if (u_rgb < 0 && u_rgb >= newdepth.cols && v_rgb < 0 && v_rgb >= newdepth.rows)
            if (u_rgb >= 0 && u_rgb < newdepth.cols && v_rgb >= 0 && v_rgb < newdepth.rows)
            {
                uint16_t *val = (uint16_t *)newdepth.ptr<uint16_t>(v_rgb) + u_rgb;
                *val = d;
                // std::cout << u_rgb << "," << v_rgb << std::endl;
            }
        }
    }
    dst = newdepth;
}