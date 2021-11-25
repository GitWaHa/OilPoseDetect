#include "fusion/simple_fusion.h"
#include "fusion/utils.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

SimpleFusion::SimpleFusion()
{
}
SimpleFusion::~SimpleFusion()
{
}

void SimpleFusion::fusion(std::string img_folder, int num, const float *target_pos, std::string save_ply_path)
{
    std::cout << "[SimpleFusion] simple fusion start ...." << std::endl;
    simpleFusion_(img_folder, num, target_pos, save_ply_path);
    std::cout << "[SimpleFusion] simple fusion complete" << std::endl;
}

void SimpleFusion::simpleFusion_(std::string data_folder, int frame_nums, const float *target_pos, std::string save_path)
{
    std::string ply_save_path = save_path; // 重建的ply文件保存位置
    std::string data_path = data_folder;
    std::string reconstruct_data_folder = data_path + "/reconstruct_data"; // 重建需要的数据(相机位姿、深度图)
    std::string base2world_file = data_path + "/rough_detecter" + "/frame_0_camerapose.txt";
    std::string cam_K_file = data_path + "/camera-intrinsics.txt";         // 相机内参
    std::string adjust_hand_eye_file = data_path + "/adjust_hand_eye.txt"; // 相机内参

    // Location of folder containing RGB-D frames and camera pose files
    int base_frame_idx = 0;
    int first_frame_idx = 0;
    float num_frames = frame_nums;

    float cam_K[3 * 3];
    float cam2tmp[4 * 4];
    float tmp2world[4 * 4];
    float base2world[4 * 4];
    float cam2base[4 * 4];
    float cam2world[4 * 4];
    int im_width = 640;
    int im_height = 480;
    float depth_im[im_height * im_width];
    //[-0.145, 0.414, 1.162]

    // Voxel grid parameters (change these to change voxel grid resolution, etc.)
    // 加油口初始位置（相机坐标系），此坐标由world_voxel_grid_origin变换计算得到
    float voxel_grid_origin_x = 0.f;
    float voxel_grid_origin_y = 0.f;
    float voxel_grid_origin_z = 0.f;
    // 单个网格边长
    float voxel_size = 0.0005f;
    // 截断距离
    float trunc_margin = voxel_size * 10;
    // 网格数量，总数=voxel_grid_dim_x*voxel_grid_dim_y*voxel_grid_dim_z
    int voxel_grid_dim_x = 400;
    int voxel_grid_dim_y = 400;
    int voxel_grid_dim_z = 400;

    // TODO: 加油口初始位置（世界坐标系）
    float world_voxel_grid_origin_x = target_pos[0];
    float world_voxel_grid_origin_y = target_pos[1];
    float world_voxel_grid_origin_z = target_pos[2];

    // Read camera intrinsics
    std::cout << "Read camera intrinsics\n";
    std::vector<float> cam_K_vec = LoadMatrixFromFile(cam_K_file, 3, 3);
    std::copy(cam_K_vec.begin(), cam_K_vec.end(), cam_K);

    // Read base frame camera pose
    std::cout << "Read base frame camera pose\n";
    std::ostringstream base_frame_prefix;
    base_frame_prefix << std::setw(2) << std::setfill('0') << base_frame_idx;
    std::vector<float> tmp2world_vec = LoadMatrixFromFile(base2world_file, 4, 4);
    std::copy(tmp2world_vec.begin(), tmp2world_vec.end(), tmp2world);
    std::vector<float> cam2tmp_vec = LoadMatrixFromFile(adjust_hand_eye_file, 4, 4);
    std::copy(cam2tmp_vec.begin(), cam2tmp_vec.end(), cam2tmp);
    multiply_matrix(tmp2world, cam2tmp, base2world);
    printArray(tmp2world, 4, 4);
    printArray(cam2tmp, 4, 4);
    printArray(base2world, 4, 4);

    // Invert base frame camera pose to get world-to-base frame transform
    float base2world_inv[16] = {0};
    invert_matrix(base2world, base2world_inv);

    // 目标位置变换（世界坐标系 -> 相机坐标系)
    float in_pt[3] = {world_voxel_grid_origin_x, world_voxel_grid_origin_y, world_voxel_grid_origin_z};
    float out_pt[3] = {0};
    transform_point(base2world_inv, in_pt, out_pt);
    voxel_grid_origin_x = out_pt[0] - voxel_grid_dim_x * voxel_size / 2;
    voxel_grid_origin_y = out_pt[1] - voxel_grid_dim_y * voxel_size / 2;
    voxel_grid_origin_z = out_pt[2] - voxel_grid_dim_z * voxel_size / 2;
    std::cout << "world_voxel_grid_origin_x: " << std::endl;
    printArray(in_pt, 1, 3);
    std::cout << "voxel_grid_origin: " << std::endl;
    printArray(out_pt, 1, 3);
    std::cout << "voxel_grid_origin(move to origin): " << std::endl;
    std::cout << voxel_grid_origin_x << "," << voxel_grid_origin_y << "," << voxel_grid_origin_z << "\n";

    // Initialize voxel grid
    std::cout << "Initialize voxel grid\n";
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Loop through each depth frame and integrate TSDF voxel grid
    for (int frame_idx = first_frame_idx; frame_idx < first_frame_idx + (int)num_frames; ++frame_idx)
    {

        std::ostringstream curr_frame_prefix;
        curr_frame_prefix << std::setw(2) << std::setfill('0') << frame_idx;

        // Read current frame depth
        std::string depth_im_file = reconstruct_data_folder + "/frame_" + curr_frame_prefix.str() + "_depth.png";
        std::cout << "Read current frame dept: " << depth_im_file << std::endl;
        ReadDepth(depth_im_file, im_height, im_width, depth_im);

        // Read base frame camera pose
        std::string cam2world_file = reconstruct_data_folder + "/frame_" + curr_frame_prefix.str() + "_pose.txt";
        std::cout << "Read base frame camera pose: " << cam2world_file << std::endl;
        std::vector<float> tmp2world_vec = LoadMatrixFromFile(cam2world_file, 4, 4);
        std::copy(tmp2world_vec.begin(), tmp2world_vec.end(), tmp2world);
        multiply_matrix(tmp2world, cam2tmp, cam2world);

        // Compute relative camera pose (camera-to-base frame)
        multiply_matrix(base2world_inv, cam2world, cam2base);

        std::cout << "Fusing: " << depth_im_file << std::endl;
        float origin_pos[3] = {voxel_grid_origin_x, voxel_grid_origin_y, voxel_grid_origin_z};
        fusionOnce_(pointcloud, depth_im, im_width, im_height,
                    cam_K, cam2base,
                    origin_pos, voxel_size * voxel_grid_dim_x,
                    base2world);
    }

    // Compute surface points from TSDF voxel grid and save to point cloud .ply file
    std::cout << "Saving surface point cloud : " << ply_save_path << std::endl;
    pcl::PLYWriter writer;
    writer.write(ply_save_path, *pointcloud, true);
}

void SimpleFusion::fusionOnce_(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud,
                               const float *depth_img, const int width, const int height,
                               const float *cam_k, const float *cam2base,
                               const float *origin_pos, const float length, const float *base2world)
{
    double constant_x = 1. / cam_k[0 * 3 + 0];
    double constant_y = 1. / cam_k[1 * 3 + 1];
    double center_x = cam_k[0 * 3 + 2];
    double center_y = cam_k[1 * 3 + 2];
    for (int r = 0; r < height; r++)
    {
        for (int c = 0; c < width; c++)
        {
            // 像素坐标系转相机坐标系
            float z = depth_img[r * width + c];
            float x = (c - center_x) * z * constant_x;
            float y = (r - center_y) * z * constant_y;
            // 相机坐标系转base相机坐标系
            float in[3] = {x, y, z};
            float out[3] = {x, y, z};
            transform_point(cam2base, in, out);

            // 是否在区域内
            if ((out[0] >= origin_pos[0] && out[0] <= origin_pos[0] + length) &&
                (out[1] >= origin_pos[1] && out[1] <= origin_pos[1] + length) &&
                (out[2] >= origin_pos[2] && out[2] <= origin_pos[2] + length))
            {
                // 转换到世界坐标系
                float tmp[3];
                transform_point(base2world, out, tmp);
                pointcloud->push_back({tmp[0], tmp[1], tmp[2]});
            }
        }
    }
}