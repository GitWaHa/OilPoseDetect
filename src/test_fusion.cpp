#include <iostream>
#include "fusion/fusion.h"
#include "fusion/tsdf_fusion.h"
#include "fusion/simple_fusion.h"

int main()
{
    std::string tsdf_folder = "/home/waha/Desktop/test_data/";
    std::string ply_path = tsdf_folder + "fusion_cloud_test.ply";

    Fusion *fusion_ = new TsdfFusion();
    // Fusion *fusion_ = new SimpleFusion();

    float target_pos[3] = {-0.0364852, 0.492246, 1.16771};
    fusion_->fusion(tsdf_folder, 34, target_pos, ply_path);

    return 0;
}