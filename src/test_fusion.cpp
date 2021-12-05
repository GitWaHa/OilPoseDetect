#include <iostream>
#include "fusion/fusion.h"
#include "fusion/tsdf_fusion.h"
#include "fusion/simple_fusion.h"

int main(int arvn, char **argv)
{
    ros::init(arvn, argv, "test_fusion");
    ros::NodeHandle nh("~");

    bool tsdf = true;
    ROS_INFO_STREAM("nh.hasParam('tsdf'):" << nh.hasParam("tsdf"));
    if (nh.hasParam("tsdf"))
        nh.getParam("tsdf", tsdf);
    ROS_INFO_STREAM("is tsdf: " << tsdf);

    std::string tsdf_folder = "/home/waha/Desktop/test_data/";
    std::string ply_path = tsdf_folder + "fusion_cloud_test.ply";
    Fusion *fusion;
    if (tsdf)
        fusion = new TsdfFusion();
    else
        fusion = new SimpleFusion();

    float target_pos[3] = {-0.0364852, 0.492246, 1.16771};
    fusion->fusion(tsdf_folder, 34, target_pos, ply_path);

    return 0;
}