#include <iostream>
#include "tsdf_fusion/tsdf_fusion.h"

int main()
{
    std::string tsdf_folder = "/home/waha/Desktop/test_data/";
    std::string ply_path = tsdf_folder + "tsdf_cloud.ply";

    TsdfFusion tsdf_fusion_;

    float target_pos[3] = {-0.0364852, 0.492246, 1.16771};
    tsdf_fusion_.Fusion(tsdf_folder, 34, target_pos, ply_path);

    return 0;
}