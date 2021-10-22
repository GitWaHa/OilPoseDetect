#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <thread>

#include "camera/camera_receiver.h"
#include "oil_detect/oil_rough_detect.h"
#include "oil_detect/oil_accurate_detect.h"
#include "tsdf_fusion/tsdf_fusion.h"
#include "tsdf_fusion/topics_capture.h"

class OilDetectTsdf
{
public:
    OilDetectTsdf(std::shared_ptr<CameraReceiver> img_receiver, std::shared_ptr<TopicsCapture> topic_capture);
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
    TsdfFusion tsdf_fusion_;

    float init_target_x_;
    float init_target_y_;
    float init_target_z_;
    std::vector<geometry_msgs::Pose> init_waypoints_;

    const float *oil_pos_;
    const float *oil_quat_;
};
