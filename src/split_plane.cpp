#include <ros/ros.h>
#include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argn, char **argv)
{
    ros::init(argn, argv, "split_plane");

    // 读取点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile("/home/waha/Workspace/Git/tsdf-fusion/tsdf1.ply", *raw_cloud);
    // pcl::io::loadPCDFile("/home/waha/Desktop/rgbdCloud.pcd", *cloud_of);

    // PCLVisualizer初始化
    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer2"));

    // 显示
    int v1(0);
    visualizer->createViewPort(0, 0, 0.33, 1.0, v1);
    visualizer->addPointCloud(raw_cloud, "raw_pcd", v1);

    std::vector<int> indices(0);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setInputCloud(raw_cloud);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.015);
    seg.setMaxIterations(100);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() < 100)
    {
        std::cout << "extract plane point nums is too less" << std::endl;
        return 0;
    }
    std::cout << "extract plane point nums " << inliers->indices.size() << endl;

    //extract plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ModelCoefficients::Ptr coefficients_circle(new pcl::ModelCoefficients);
    extract.setInputCloud(raw_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(indices);
    extract.filter(*plane_cloud);

    // 显示
    int v2(1);
    visualizer->createViewPort(0.33, 0, 0.66, 1.0, v2);
    visualizer->addPointCloud(plane_cloud, "plane_cloud", v2);

    // extract oil
    pcl::PointCloud<pcl::PointXYZ>::Ptr oil_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setNegative(true);
    extract.filter(indices);
    extract.filter(*oil_cloud);

    // 显示
    int v3(2);
    visualizer->createViewPort(0.66, 0, 1.0, 1.0, v3);
    visualizer->addPointCloud(oil_cloud, "oil_cloud", v3);

    // save cloud
    std::string save_path = "/home/waha/catkin_ws/src/oil_robot/oil_pose_detect/data/";
    std::cout << "save path is " << save_path << std::endl;
    pcl::io::savePCDFile(save_path + "raw_cloud.pcd", *raw_cloud);
    pcl::io::savePCDFile(save_path + "plane_cloud.pcd", *plane_cloud);
    pcl::io::savePCDFile(save_path + "oil_cloud.pcd", *oil_cloud);

    while (ros::ok())
    {
        visualizer->spinOnce(10);
    }

    return 0;
}