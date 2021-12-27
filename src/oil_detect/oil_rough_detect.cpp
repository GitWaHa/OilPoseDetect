#include <fstream>

#include "oil_detect/oil_rough_detect.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/keypoints/uniform_sampling.h>

#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>

OilRoughDetect::OilRoughDetect(std::string color_frame) : color_frame_(color_frame), cloud_(new PCLPointCloudRGB)
{
}

OilRoughDetect::~OilRoughDetect()
{
}

int OilRoughDetect::detect_once(const cv::Mat &color, const cv::Mat &depth, const PCLPointCloudRGB::Ptr cloud, float *oil_pose)
{
    color_ = color;
    color_draw_ = color_.clone();
    depth_ = depth;
    cloud_ = cloud;

    if (roiDetect() != 0 || getPosFromRoi() != 0)
        return -1;

    oil_pose[0] = getPositionInWorld()[0];
    oil_pose[1] = getPositionInWorld()[1];
    oil_pose[2] = getPositionInWorld()[2];

    cout << "[info]"
         << "getPositionInCamera:" << getPositionInCamera()[0] << "," << getPositionInCamera()[1] << "," << getPositionInCamera()[2] << endl;
    cout << "[info]"
         << "getPositionInWorld:" << getPositionInWorld()[0] << "," << getPositionInWorld()[1] << "," << getPositionInWorld()[2] << endl;

    return 0;
}

void OilRoughDetect::saveDataFrame(const std::string save_folder, const std::string save_num)
{
    if (!boost::filesystem::exists(save_folder))
    {
        ROS_INFO_STREAM("[main] mkdir :" << save_folder);
        boost::filesystem::create_directories(save_folder);
    }

    cout << "[info] start save data ....." << endl;
    std::string pos_path = save_folder + "/frame_" + save_num + "_oil_pos" + ".txt";
    ofstream pose_f(pos_path);
    pose_f << "in camera: ";
    pose_f << getPositionInCamera()[0] << " " << getPositionInCamera()[1] << " " << getPositionInCamera()[2] << endl;
    pose_f << "in world: ";
    pose_f << getPositionInWorld()[0] << " " << getPositionInWorld()[1] << " " << getPositionInWorld()[2] << endl;
    pose_f.close();
    cout << "[info]"
         << "save pose to " << pos_path << endl;

    std::string cloud_path = save_folder + "/frame_" + save_num + "_cloud" + ".pcd";
    pcl::io::savePCDFile(cloud_path, *cloud_);
    cout << "[info]"
         << "save cloud_ to " << cloud_path << endl;

    std::string color_path = save_folder + "/frame_" + save_num + "_color" + ".jpg";
    cv::imwrite(color_path, color_);
    cout << "[info]"
         << "save color_ to " << color_path << endl;

    std::string color_draw_path = save_folder + "/frame_" + save_num + "_color_draw" + ".jpg";
    cv::imwrite(color_draw_path, color_draw_);
    cout << "[info]"
         << "save color_draw_ to " << color_draw_path << endl;

    std::string depth_path = save_folder + "/frame_" + save_num + "_depth" + ".png";
    cv::imwrite(depth_path, depth_);
    cout << "[info]"
         << "save depth_ to " << depth_path << endl;

    tf::StampedTransform transform;
    std::string camera_pose_path = save_folder + "/frame_" + save_num + "_camerapose" + ".txt";
    saveCameraPose(camera_pose_path);
    cout << "[info]"
         << "save camera_pose to " << camera_pose_path << endl;
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
    std::vector<int> indices(0);
    extract.setInputCloud(cloud_tmp);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(indices);
    // FIXME: 造成程序自动退出，只能使用 extract.filter(indices);方式
    // extract.filter(*plane);

    // 随机选择点
    // pcl::PointCloud<pcl::PointXYZ>::Ptr ran_plane(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::UniformSampling<pcl::PointXYZ> filter;
    // filter.setInputCloud(plane);
    // filter.setRadiusSearch(0.005f);
    // filter.filter(*ran_plane);

    // 计算平面内点的均值坐标，相对于像极坐标系
    computerMeanValue(cloud_tmp, indices, oil_pos_in_camera_);
    // computerMeanValue(plane, oil_pos_in_camera_);  // FIXME:

    //转换到世界坐标系
    getCameraPose(color_frame_, "base_link");
    convertPosToWorld(oil_pos_in_camera_, oil_pos_in_world_);

    return 0;
}

void OilRoughDetect::computerMeanValue(PCLPointCloud::Ptr cloud, std::vector<int> indices, float *pos)
{
    float XData = 0.0, YData = 0.0, ZData = 0.0;
    for (auto idx : indices)
    {
        auto point = cloud->at(idx);
        XData += point.x;
        YData += point.y;
        ZData += point.z;
    }
    int size = indices.size();
    pos[0] = XData / size;
    pos[1] = YData / size;
    pos[2] = ZData / size;
}

void OilRoughDetect::computerMeanValue(PCLPointCloud::Ptr cloud, float *pos)
{
    float XData = 0.0, YData = 0.0, ZData = 0.0;
    for (auto start = cloud->begin(); start != cloud->end(); start++)
    {
        XData += (*start).x;
        YData += (*start).y;
        ZData += (*start).z;
    }
    int size = cloud->size();
    pos[0] = XData / size;
    pos[1] = YData / size;
    pos[2] = ZData / size;
}

void OilRoughDetect::saveCameraPose(std::string save_path)
{
    tf::Matrix3x3 roat(camera_pose_.getRotation());

    if (!save_path.empty())
    {
        ofstream OutFile(save_path);
        OutFile << roat.getRow(0).getX() << " " << roat.getRow(0).getY() << " " << roat.getRow(0).getZ() << " " << camera_pose_.getOrigin().getX() << std::endl;
        OutFile << roat.getRow(1).getX() << " " << roat.getRow(1).getY() << " " << roat.getRow(1).getZ() << " " << camera_pose_.getOrigin().getY() << std::endl;
        OutFile << roat.getRow(2).getX() << " " << roat.getRow(2).getY() << " " << roat.getRow(2).getZ() << " " << camera_pose_.getOrigin().getZ() << std::endl;
        OutFile << 0 << " " << 0 << " " << 0 << " " << 1;

        OutFile.close();
    }
}

void OilRoughDetect::getCameraPose(std::string source_frame, std::string target_frame)
{
    tf::TransformListener listener;

    listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(3));
    listener.lookupTransform(target_frame, source_frame, ros::Time(0), camera_pose_);
}

void OilRoughDetect::convertPosToWorld(float *oil_pos_in_camera, float *oil_pos_in_world)
{
    geometry_msgs::Point point1;
    point1.x = oil_pos_in_camera[0];
    point1.y = oil_pos_in_camera[1];
    point1.z = oil_pos_in_camera[2];

    geometry_msgs::Point point2;

    geometry_msgs::TransformStamped transform;
    transform.transform.translation.x = camera_pose_.getOrigin().getX();
    transform.transform.translation.y = camera_pose_.getOrigin().getY();
    transform.transform.translation.z = camera_pose_.getOrigin().getZ();
    transform.transform.rotation.x = camera_pose_.getRotation().getX();
    transform.transform.rotation.y = camera_pose_.getRotation().getY();
    transform.transform.rotation.z = camera_pose_.getRotation().getZ();
    transform.transform.rotation.w = camera_pose_.getRotation().getW();

    tf2::doTransform(point1, point2, transform);
    oil_pos_in_world[0] = point2.x;
    oil_pos_in_world[1] = point2.y;
    oil_pos_in_world[2] = point2.z;
}