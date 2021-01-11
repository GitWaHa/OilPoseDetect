#pragma onece

// system
#include <memory>
#include <vector>
#include <iostream>
#include <algorithm>

// ROS
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

// Opencv
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"

#include "camera_receiver.h"

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudPointNormal;

class OilFillerPose
{
public:
    /**
     * \brief Constructor.
     * \param node the ROS node
    */
    // OilFillerPose(ros::NodeHandle &node, std::string camera_frame, int rate);

    OilFillerPose(ros::NodeHandle &node, std::shared_ptr<CameraReceiver> camera_receiver, std::string camera_frame, int rate);

    /**
     * \brief Run the ROS node. Loops while waiting for incoming ROS messages.
    */
    void run(int loop_rate); // 仅进行姿态检测

    void runShow(int loop_rate); // 姿态检测加显示

    void imageViewer();

    void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *viewer_void);

    void saveCloudAndImages();

    bool ofDetect(); // 加油口检测

    bool ofPlaneCal(); // 加油口平面拟合

    void ofCenterCal(); // 加油口中心世界坐标计算

    void ofPoseCal(); // 加油口姿态解算

    void ofPoseShow(pcl::visualization::PCLVisualizer::Ptr &visualizer); // 显示加油口相关信息

    void publishTF(); // 发布加油口姿态

private:
    // 加油口相关参数
    cv::Rect of_rect;                                   // 加油口外接矩形
    cv::Point of_center;                                // 加油口中心点
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;      // 原始点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_of;       // 加油口点云
    Eigen::VectorXf coef = Eigen::VectorXf::Zero(4, 1); // 平面参数
    Eigen::Matrix3Xd trans_buff_;
    const int buff_size_;

    Eigen::Vector3d trans;      // 加油口坐标系平移矩阵
    Eigen::Matrix3d rot_matrix; // 加油口坐标系旋转矩阵

    bool running = false;

    cv::Mat color_draw;

    size_t frame = 0;
    int frame_rate_ = 0;
    int rate_;
    bool save = false;
    bool update = false;
    std::ostringstream oss;
    pcl::PCDWriter writer;
    std::vector<int> params;

    std::thread imageViewerThread;

    std::shared_ptr<CameraReceiver> receiver;

    std::string camera_frame_;
    tf::TransformBroadcaster broadcaster;
};