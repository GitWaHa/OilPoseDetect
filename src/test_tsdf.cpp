#include <iostream>
#include "tsdf_fusion/tsdf_fusion.h"

int main()
{
    std::string tsdf_folder = "/home/waha/Desktop/test_data/";
    std::string ply_path = tsdf_folder + "tsdf_cloud.ply";

    TsdfFusion tsdf_fusion_;

    float target_pos[3] = {-0.0101787, 0.4173, 1.16656};
    tsdf_fusion_.Fusion(tsdf_folder, 84, target_pos, ply_path);

    return 0;
}