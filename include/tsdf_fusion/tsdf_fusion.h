#pragma once

#include <iostream>
#include <ros/ros.h>

#include "tsdf_fusion/tsdf_cuda.cuh"

class TsdfFusion
{
public:
    TsdfFusion();
    ~TsdfFusion();

    void Fusion(std::string img_folder, int num, const float *target_pos, std::string save_ply_path);

private:
    /* data */
};