#pragma once

#include <iostream>

#include "fusion/fusion.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class SimpleFusion : public Fusion
{
public:
    SimpleFusion();
    ~SimpleFusion();

    void fusion(std::string img_folder, int num, const float *target_pos, std::string save_ply_path);

private:
    void simpleFusion_(std::string data_folder, int frame_nums, const float *target_pos, std::string save_path);

    void fusionOnce_(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud,
                     const float *depth_img, const int width, const int height,
                     const float *cam_k, const float *cam2base,
                     const float *origin_pos, const float length, const float *base2world);

private:
    /* data */
};
