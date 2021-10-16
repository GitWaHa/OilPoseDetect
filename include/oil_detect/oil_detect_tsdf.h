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

#include <tf/transform_broadcaster.h>

class oilDetectTsdf
{
public:
    oilDetectTsdf(std::shared_ptr<CameraReceiver> img_receiver);
    ~oilDetectTsdf();

    void run(int loop_rate, int flag);

    void setArmNames(std::vector<std::string> names)
    {
        multi_arm_name_ = std::move(names);
    }

    const float *getOilTrans()
    {
        return oil_pos_;
    }

    const float *getOilQuat()
    {
        return oil_quat_;
    }

private:
    int multiViewDataCollect(std::string output_folder);
    void publishTF();

private:
    /* data */
    std::shared_ptr<CameraReceiver> img_receiver_;
    OilRoughDetect oil_rough_detecter_;
    OilAccurateDetect oil_accurate_detecter_;
    TsdfFusion tsdf_fusion_;

    std::vector<std::string> multi_arm_name_;

    const float *oil_pos_;
    const float *oil_quat_;
    bool oil_pose_valid_;

    tf::TransformBroadcaster broadcaster_;
};
