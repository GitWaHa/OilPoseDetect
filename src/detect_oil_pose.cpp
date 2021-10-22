#include <utility>

#include "oil_detect/oil_detect.h"
#include "camera/tuyang_receiver.h"

#include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "oil_filler_pose");
    ros::NodeHandle node("~");

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

    if (!ros::ok())
    {
        return 1;
    }

    int loop_rate = 15; // 与采集频率接近即可

    /// Realsense cloud and image receiver
    std::shared_ptr<CameraReceiver> camera_receiver;
    if (camera == "tuyang")
        camera_receiver = std::make_shared<TuYangCameraReceiver>(node, topicColor, topicDepth, useExact, useCompressed, loop_rate);
    else
        camera_receiver = std::make_shared<CameraReceiver>(node, topicColor, topicDepth, useExact, useCompressed, loop_rate);

    OilFillerPose of_pose(node, camera_receiver, oil_frame_reference, loop_rate);
    if (!show)
    {
        of_pose.run(loop_rate);
    }
    else
    {
        of_pose.runShow(loop_rate);
    }

    ros::waitForShutdown();
    return 0;
}