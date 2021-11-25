#pragma once

#include <iostream>
#include <ros/ros.h>

#include "fusion/fusion.h"
#include "fusion/tsdf_cuda.cuh"

class TsdfFusion : public Fusion
{
public:
    TsdfFusion();
    ~TsdfFusion();

    void fusion(std::string img_folder, int num, const float *target_pos, std::string save_ply_path);

private:
    /* data */
};