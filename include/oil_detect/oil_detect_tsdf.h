#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "camera/camera_receiver.h"
#include "oil_detect/oil_rough_detect.h"
#include "oil_detect/oil_accurate_detect.h"
#include "fusion/tsdf_fusion.h"
#include "fusion/fusion.h"
#include "fusion/topics_capture.h"

class OilDetectTsdf
{
public:
    OilDetectTsdf(std::shared_ptr<CameraReceiver> img_receiver, std::shared_ptr<TopicsCapture> topic_capture, std::string color_frame, std::string root_floder);
    ~OilDetectTsdf();

    int run();

    const float *getOilTrans()
    {
        return oil_pos_;
    }

    const float *getOilQuat()
    {
        return oil_quat_;
    }

    void imageViewer(int loop_rate);
    void cloudViewer(int loop_rate);

    void show(int loop_rate);

private:
    int multiViewDataCollect(float *oil_position, std::string output_folder);

private:
    /* data */
    std::shared_ptr<CameraReceiver> img_receiver_;
    std::shared_ptr<TopicsCapture> topic_capture_;

    OilRoughDetect oil_rough_detecter_;
    OilAccurateDetect oil_accurate_detecter_;

    std::string tsdf_data_floder_;
    Fusion *fusion_;

    float init_target_x_;
    float init_target_y_;
    float init_target_z_;
    std::vector<geometry_msgs::Pose> init_waypoints_;
    std::vector<std::string> name_targets_;

    const float *oil_pos_;
    const float *oil_quat_;

    // 机械臂运动规划相关
    // 运动规划接口
    moveit::planning_interface::MoveGroupInterface move_group_;
    // 规划环境接口，如障碍物设置
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};
