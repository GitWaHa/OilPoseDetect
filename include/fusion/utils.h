// ---------------------------------------------------------
// Author: Andy Zeng, Princeton University, 2016
// ---------------------------------------------------------

#include <vector>
#include <opencv2/opencv.hpp>

void printArray(float *array, int r, int c);

void transform_point(const float *trans, const float *tmp_pt, float *out_pt);
// Compute surface points from TSDF voxel grid and save points to point cloud file
void SaveVoxelGrid2SurfacePointCloud(const std::string &file_name, int voxel_grid_dim_x, int voxel_grid_dim_y, int voxel_grid_dim_z,
                                     float voxel_size, float voxel_grid_origin_x, float voxel_grid_origin_y, float voxel_grid_origin_z,
                                     float *voxel_grid_TSDF, float *voxel_grid_weight,
                                     float tsdf_thresh, float weight_thresh, const float cam2world[16]);

// Load an M x N matrix from a text file (numbers delimited by spaces/tabs)
// Return the matrix as a float vector of the matrix in row-major order
std::vector<float> LoadMatrixFromFile(std::string filename, int M, int N);

// Read a depth image with size H x W and save the depth values (in meters) into a float array (in row-major order)
// The depth image file is assumed to be in 16-bit PNG format, depth in millimeters
void ReadDepth(std::string filename, int H, int W, float *depth);

// 点变换
void multiply_point(const float m[16], const float in[3], float out[3]);

// 4x4 matrix multiplication (matrices are stored as float arrays in row-major order)
void multiply_matrix(const float m1[16], const float m2[16], float mOut[16]);

// 4x4 matrix inversion (matrices are stored as float arrays in row-major order)
bool invert_matrix(const float m[16], float invOut[16]);