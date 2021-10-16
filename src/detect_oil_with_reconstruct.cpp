#include <iostream>

#include "oil_detect/oil_detect_tsdf.h"
#include "camera/tuyang_receiver.h"

#include <oil_pose_detect/DetectOilWithReconstruct.h>
#include <oil_pose_detect/DetectOilWithReconstructRequest.h>
#include <oil_pose_detect/DetectOilWithReconstructResponse.h>
// #include <std_srvs/SetBool.h>

using namespace std;

class DetectOilWithReconstructServer
{

public:
    DetectOilWithReconstructServer() = delete;
    DetectOilWithReconstructServer(std::string service_name, ros::NodeHandle &nh, std::shared_ptr<CameraReceiver> img_receiver);
    ~DetectOilWithReconstructServer();

    bool serverCallBack(oil_pose_detect::DetectOilWithReconstruct::Request &req, oil_pose_detect::DetectOilWithReconstruct::Response &res);

private:
    /* data */
    oilDetectTsdf oil_detecter;

    ros::NodeHandle nh_;
    ros::ServiceServer server_;
};

DetectOilWithReconstructServer::DetectOilWithReconstructServer(std::string service_name, ros::NodeHandle &nh, std::shared_ptr<CameraReceiver> img_receiver)
    : nh_(nh), oil_detecter(img_receiver)
{
    oil_detecter.setArmNames({"tuyang_rebuild_1", "tuyang_rebuild_2", "tuyang_rebuild_3"});
    server_ = nh_.advertiseService(service_name, &DetectOilWithReconstructServer::serverCallBack, this);
}

DetectOilWithReconstructServer::~DetectOilWithReconstructServer()
{
}

bool DetectOilWithReconstructServer::serverCallBack(oil_pose_detect::DetectOilWithReconstruct::Request &req, oil_pose_detect::DetectOilWithReconstruct::Response &res)
{
    std::cout << "[DetectOilWithReconstructServer] server start ..." << std::endl
              << std::endl;
    oil_detecter.run(15, req.flag);

    res.trans.push_back(oil_detecter.getOilTrans()[0]);
    res.trans.push_back(oil_detecter.getOilTrans()[1]);
    res.trans.push_back(oil_detecter.getOilTrans()[2]);

    res.quat.push_back(oil_detecter.getOilQuat()[0]);
    res.quat.push_back(oil_detecter.getOilQuat()[1]);
    res.quat.push_back(oil_detecter.getOilQuat()[2]);
    res.quat.push_back(oil_detecter.getOilQuat()[3]);

    std::cout << std::endl
              << "[DetectOilWithReconstructServer] server end" << std::endl;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "oil_filler_pose");
    ros::NodeHandle node("~");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    // 参数解析
    bool show;
    std::string camera;
    std::string oil_frame_reference;
    std::string topicColor;
    std::string topicDepth;
    bool useExact = false;
    bool useCompressed = false;

    node.param("show", show, true);
    node.param("camera", camera, std::string("realsense"));
    node.param("oil_frame_reference", oil_frame_reference, std::string("camera_color_optical_frame"));
    node.param("topicColor", topicColor, std::string("/camera/color/image_raw"));
    node.param("topicDepth", topicDepth, std::string("/camera/depth/image_raw"));
    node.param("useExact", useExact, false);
    node.param("useCompressed", useCompressed, false);

    int loop_rate = 15; // 与采集频率接近即可

    /// Realsense cloud and image receiver
    std::shared_ptr<CameraReceiver> camera_receiver;
    if (camera == "tuyang")
        camera_receiver = std::make_shared<TuYangCameraReceiver>(node, topicColor, topicDepth, useExact, useCompressed, loop_rate);
    else
        camera_receiver = std::make_shared<CameraReceiver>(node, topicColor, topicDepth, useExact, useCompressed, loop_rate);

    DetectOilWithReconstructServer server("oil_detect_with_reconstruct", node, camera_receiver);

    ros::waitForShutdown();
    return 0;
}