#include <fstream>

#include "oil_detect/oil_rough_detect.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/keypoints/uniform_sampling.h>

#include <tf/transform_listener.h>

OilRoughDetect::OilRoughDetect() : cloud_(new PCLPointCloudRGB)
{
}

OilRoughDetect::~OilRoughDetect()
{
}

int OilRoughDetect::detect_once(const cv::Mat &color, const cv::Mat &depth, const PCLPointCloudRGB::Ptr cloud, float *oil_pose)
{
    color_ = color;
    depth_ = depth;
    cloud_ = cloud;

    if (roiDetect() != 0 || getPosFromRoi() != 0)
        return -1;

    oil_pose[0] = getPosition()[0];
    oil_pose[1] = getPosition()[1];
    oil_pose[2] = getPosition()[2];

    return 0;
}

void OilRoughDetect::saveDataFrame(const std::string save_folder, const std::string save_num)
{
    std::string pos_path = save_folder + "frame_" + save_num + "_pos" + ".pcd";
    ofstream pose_f(pos_path);
    pose_f << getPosition()[0] << " " << getPosition()[1] << " " << getPosition()[2];
    pose_f.close();

    std::string cloud_path = save_folder + "frame_" + save_num + "_cloud" + ".pcd";
    pcl::io::savePCDFile(cloud_path, *cloud_);

    std::string color_path = save_folder + "frame_" + save_num + "_color" + ".jpg";
    cv::imwrite(color_path, color_);

    std::string color_draw_path = save_folder + "frame_" + save_num + "_color_draw" + ".jpg";
    cv::imwrite(color_draw_path, color_draw_);

    std::string depth_path = save_folder + "frame_" + save_num + "_depth" + ".png";
    cv::imwrite(depth_path, depth_);

    tf::StampedTransform transform;
    std::string camera_pose = save_folder + "frame_" + save_num + "_pose" + ".txt";
    getCameraPose("camera_rgb_optical_frame", "base_link", transform, camera_pose);
}

int OilRoughDetect::roiDetect()
{
    // 均值滤波
    cv::Mat img_blur;
    blur(color_, img_blur, cv::Size(10, 10));

    // 灰度转换
    cv::Mat img_gray;
    cvtColor(img_blur, img_gray, cv::COLOR_BGR2GRAY);

    // 霍夫变换圆检测
    // 参数:      默认, 最小间距, canny上限, 阈值(越大越圆), 最小半径, 最大半径
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(img_gray, circles, cv::HOUGH_GRADIENT, 1, 1, 80, 30, 40, 200); // 这里的canny算法下限自动设置为上限一半

    if (circles.empty())
    {
        std::cout << "[OilRoughDetect] Detect 0 circles!" << std::endl;
        return -1;
    }

    // 查找最大圆
    std::vector<double> radius_vec;
    int max_r_index = 0;
    double max_r = 0;
    for (int i = 0; i < circles.size(); i++)
    {
        double radius = cvRound(circles[i][2]);
        radius_vec.push_back(radius);

        if (radius > max_r)
        {
            max_r = radius;
            max_r_index = i;
        }
    }

    std::cout << "[OilRoughDetect] 最大圆半径是 " << max_r << ", 索引是 " << max_r_index << std::endl;

    cv::Point center(cvRound(circles[max_r_index][0]), cvRound(circles[max_r_index][1]));
    double radius = cvRound(circles[max_r_index][2]);

    // 绘制圆心与轮廓
    circle(color_draw_, center, 4, cv::Scalar(0, 255, 0), -1, 8, 0);
    circle(color_draw_, center, (int)radius, cv::Scalar(0, 0, 255), 2, 8, 0);

    // 放大圆形区域为矩形框
    int radius_zoom = (int)(radius * 2);
    int x = std::max(center.x - radius_zoom, 0);
    int y = std::max(center.y - radius_zoom, 0);
    int w = std::min(2 * radius_zoom, 640 - x);
    int h = std::min(2 * radius_zoom, 480 - y);
    cv::Rect rect(x, y, w, h);
    cv::rectangle(color_draw_, rect, cvScalar(0, 255, 255), 2, 8, 0);

    if (rect.x < 0 || rect.x > color_.cols || rect.y < 0 || rect.y > color_.rows)
    {
        std::cout << "[OilRoughDetect] [error] Bad rect!" << std::endl;
        return -1;
    }

    oil_roi_ = rect;

    return 0;
}

void OilRoughDetect::getCameraPose(std::string source_frame, std::string target_frame, tf::StampedTransform &transform, std::string save_path)
{
    tf::TransformListener listener;

    listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(3));
    listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);

    tf::Matrix3x3 roat(transform.getRotation());

    if (!save_path.empty())
    {
        ofstream OutFile(save_path);
        OutFile << roat.getRow(0).getX() << " " << roat.getRow(0).getY() << " " << roat.getRow(0).getZ() << " " << transform.getOrigin().getX() << std::endl;
        OutFile << roat.getRow(1).getX() << " " << roat.getRow(1).getY() << " " << roat.getRow(1).getZ() << " " << transform.getOrigin().getY() << std::endl;
        OutFile << roat.getRow(2).getX() << " " << roat.getRow(2).getY() << " " << roat.getRow(2).getZ() << " " << transform.getOrigin().getZ() << std::endl;
        OutFile << 0 << " " << 0 << " " << 0 << " " << 1;

        OutFile.close();
    }
}

int OilRoughDetect::getPosFromRoi()
{
    /// 获取加油口无组织无色彩点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
    for (int row = oil_roi_.y; row < oil_roi_.y + oil_roi_.height; row++)
    {
        for (int col = oil_roi_.x; col < oil_roi_.x + oil_roi_.width; col++)
        {
            pcl::PointXYZRGBA p_in = cloud_->points[row * cloud_->width + col];
            pcl::PointXYZ p_out;
            p_out.x = p_in.x;
            p_out.y = p_in.y;
            p_out.z = p_in.z;
            cloud_tmp->points.push_back(p_out);
        }
    }

    if (cloud_tmp->points.empty())
    {
        std::cout << "[OilRoughDetect] "
                  << "[Erro] Cloud_tmp is empty! no point in selected area" << std::endl;
        return -1;
    }

    std::cout << "[OilRoughDetect] "
              << "cloud_tmp size: " << cloud_tmp->points.size() << std::endl;

    /// 平面拟合 // 平面方程: ax+by+cz+d = 0
    std::vector<int> indices(0);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setInputCloud(cloud_tmp);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setMaxIterations(100);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() < 100)
    {
        std::cout << "[OilRoughDetect] "
                  << "plane point nuw is too less!" << std::endl;
        return -1;
    }
    std::cout << "[OilRoughDetect] "
              << "平面局内点数：" << inliers->indices.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_tmp);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(indices);
    // extract.filter(*plane);

    // 随机选择点
    pcl::PointCloud<pcl::PointXYZ>::Ptr ran_plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::UniformSampling<pcl::PointXYZ> filter;
    filter.setInputCloud(plane);
    filter.setRadiusSearch(0.005f);
    // filter.filter(*ran_plane);

    computerMeanValue(ran_plane, oil_pos_);

    return 0;
}

void OilRoughDetect::computerMeanValue(PCLPointCloud::Ptr cloud, float *pos)
{
    float XData = 0.0, YData = 0.0, ZData = 0.0;
    for (auto iter = cloud->begin(); iter != cloud->end(); iter++)
    {
        XData += (*iter).x;
        YData += (*iter).y;
        ZData += (*iter).z;
    }
    pos[0] = XData / cloud->size();
    pos[1] = YData / cloud->size();
    pos[2] = ZData / cloud->size();
}