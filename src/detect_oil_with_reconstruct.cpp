#include <iostream>

#include "oil_detect/oil_detect_tsdf.h"
#include "camera/tuyang_receiver.h"
#include "tsdf_fusion/topics_capture.h"

#include <oil_pose_detector/OilPoseDetector.h>
#include <oil_pose_detector/OilPoseDetectorRequest.h>
#include <oil_pose_detector/OilPoseDetectorResponse.h>

using namespace std;

class DetectOilWithReconstructServer
{

public:
    DetectOilWithReconstructServer() = delete;
    DetectOilWithReconstructServer(std::string service_name, ros::NodeHandle &nh);
    ~DetectOilWithReconstructServer(){};

    bool serverCallBack(oil_pose_detector::OilPoseDetector::Request &req, oil_pose_detector::OilPoseDetector::Response &res);

private:
    void init();
    /* data */
    std::unique_ptr<OilDetectTsdf> oil_detecter;

    ros::NodeHandle nh_;
    ros::ServiceServer server_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "oil_filler_pose");
    ros::NodeHandle node("~");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    DetectOilWithReconstructServer server("get_pose", node);

    ros::waitForShutdown();
    return 0;
}

DetectOilWithReconstructServer::DetectOilWithReconstructServer(std::string service_name, ros::NodeHandle &nh)
    : nh_(nh)
{
    // 初始化相关变量
    init();

    // 发布服务
    server_ = nh_.advertiseService(service_name, &DetectOilWithReconstructServer::serverCallBack, this);

    ROS_INFO("server is start....");
}

bool DetectOilWithReconstructServer::serverCallBack(oil_pose_detector::OilPoseDetector::Request &req,
                                                    oil_pose_detector::OilPoseDetector::Response &res)
{
    std::cout << "[DetectOilWithReconstructServer] server start ..." << std::endl
              << std::endl;
    int flag = oil_detecter->run();

    res.is_valid = flag == 0 ? true : false;

    if (res.is_valid)
    {
        res.is_valid = true;
        res.trans.push_back(oil_detecter->getOilTrans()[0]);
        res.trans.push_back(oil_detecter->getOilTrans()[1]);
        res.trans.push_back(oil_detecter->getOilTrans()[2]);

        res.quat.push_back(oil_detecter->getOilQuat()[0]);
        res.quat.push_back(oil_detecter->getOilQuat()[1]);
        res.quat.push_back(oil_detecter->getOilQuat()[2]);
        res.quat.push_back(oil_detecter->getOilQuat()[3]);
    }

    std::cout << std::endl
              << "[DetectOilWithReconstructServer] server end" << std::endl;

    return true;
}

void DetectOilWithReconstructServer::init()
{
    // 参数解析
    bool show;
    std::string camera;
    std::string oil_frame_reference;
    std::string topicColor;
    std::string topicDepth;
    bool useExact = false;
    bool useCompressed = false;

    nh_.param("show", show, true);
    nh_.param("camera", camera, std::string("realsense"));
    nh_.param("oil_frame_reference", oil_frame_reference, std::string("camera_color_optical_frame"));
    nh_.param("topicColor", topicColor, std::string("/camera/color/image_raw"));
    nh_.param("topicDepth", topicDepth, std::string("/camera/depth/image_raw"));
    nh_.param("useExact", useExact, false);
    nh_.param("useCompressed", useCompressed, false);

    /// Realsense cloud and image receiver
    std::shared_ptr<CameraReceiver> camera_receiver;
    if (camera == "tuyang")
        camera_receiver = std::make_shared<TuYangCameraReceiver>(nh_, topicColor, topicDepth, useExact, useCompressed, 30);
    else
        camera_receiver = std::make_shared<CameraReceiver>(nh_, topicColor, topicDepth, useExact, useCompressed, 30);

    // tsdf相关话题的捕获，保存到某个文件夹下，方便tsdf调用
    std::string data_folder = "/home/waha/Desktop/test_data";
    auto topic_receiver = std::make_shared<TopicsCapture>(topicDepth, "/camera/pose", data_folder + "/reconstruct_data");
    oil_detecter = std::make_unique<OilDetectTsdf>(camera_receiver, topic_receiver, oil_frame_reference, data_folder);

    if (show)
        oil_detecter->show(15);
};