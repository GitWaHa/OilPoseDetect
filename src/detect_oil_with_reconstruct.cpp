#include <iostream>

#include "oil_detect/oil_detect_tsdf.h"
#include "camera/tuyang_receiver.h"
#include "fusion/topics_capture.h"

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

bool checkFiles(string data_folder, std::vector<std::string> camera_files)
{
    if (!boost::filesystem::exists(data_folder))
    {
        ROS_INFO_STREAM("[main] mkdir :" << data_folder);
        boost::filesystem::create_directories(data_folder);
        return false;
    }

    for (auto name : camera_files)
    {
        if (!boost::filesystem::exists(data_folder + "/" + name))
            return false;
    }

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

    std::string data_folder = "/home/waha/Desktop/oil_reconstruct_data";
    std::vector<string> camera_params_files = {"adjust_hand_eye.txt", "camera-intrinsics.txt"};
    if (!checkFiles(data_folder, camera_params_files))
    {
        ROS_ERROR_STREAM("[main] checkFiles failed!!!");
        return;
    }

    // 按时间创建文件夹
    time_t t = time(0);
    char time[32];
    strftime(time, sizeof(time), "%Y-%m-%d-%H-%M-%S", localtime(&t));
    string data_time_folder = data_folder + "/" + string(time);
    boost::filesystem::create_directories(data_time_folder);
    for (auto name : camera_params_files)
    {
        ROS_INFO_STREAM("[main] copy file " << data_folder + "/" + name << " to " << data_time_folder + "/" + name);
        boost::filesystem::copy_file(data_folder + "/" + name, data_time_folder + "/" + name);
    }

    /// Realsense cloud and image receiver
    std::shared_ptr<CameraReceiver> camera_receiver;
    if (camera == "tuyang")
        camera_receiver = std::make_shared<TuYangCameraReceiver>(nh_, topicColor, topicDepth, useExact, useCompressed, 30);
    else
        camera_receiver = std::make_shared<CameraReceiver>(nh_, topicColor, topicDepth, useExact, useCompressed, 30);

    // tsdf相关话题的捕获，保存到某个文件夹下，方便tsdf调用
    auto topic_receiver = std::make_shared<TopicsCapture>(topicDepth, topicColor, "/camera/pose", data_time_folder + "/reconstruct_data");
    oil_detecter = std::make_unique<OilDetectTsdf>(camera_receiver, topic_receiver, oil_frame_reference, data_time_folder);

    if (show)
        oil_detecter->show(15);
};