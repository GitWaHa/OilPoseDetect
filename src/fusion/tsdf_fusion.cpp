#include "fusion/tsdf_fusion.h"

TsdfFusion::TsdfFusion()
{
}
TsdfFusion::~TsdfFusion()
{
}

void TsdfFusion::fusion(std::string img_folder, int num, const float *target_pos, std::string save_ply_path)
{
    std::cout << "[TsdfFusion] tsdf fusion start ...." << std::endl;
    TSDF_Fusion(img_folder.c_str(), num, target_pos, save_ply_path.c_str());
    std::cout << "[TsdfFusion] tsdf fusion complete" << std::endl;
}