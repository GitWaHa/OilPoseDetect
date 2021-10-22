#include "oil_detect/oil_detect_tsdf.h"

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <pcl/io/ply_io.h>
#include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

OilDetectTsdf::OilDetectTsdf(std::shared_ptr<CameraReceiver> img_receiver, std::shared_ptr<TopicsCapture> topic_capture)
    : img_receiver_(img_receiver), topic_capture_(topic_capture)
{
    img_receiver->run();

    //TODO: 初始化路径点及目标位置
}

OilDetectTsdf::~OilDetectTsdf()
{
}

int OilDetectTsdf::run()
{
    std::string tsdf_folder = "/home/waha/Desktop/test_data/";
    std::string ply_path = tsdf_folder + "tsdf_cloud.ply";
    int is_ok = -1;

    // 初步定位
    cout << "[info] "
         << "初步定位...！" << endl;
    float rough_pos[3] = {init_target_x_, init_target_y_, init_target_z_};
    // is_ok = oil_rough_detecter_.detect_once(img_receiver_->getColor(),
    //                                         img_receiver_->getDepth(),
    //                                         img_receiver_->getCloud(), rough_pos);
    // if (is_ok != 0)
    // {
    //     cout << "[error] "
    //          << "初步定位失败！" << endl;
    //     return 1;
    // }

    // 多视角采集
    cout << "[info] "
         << "多视角采集...！" << endl;
    int multi_num = multiViewDataCollect(rough_pos, tsdf_folder);
    if (multi_num == 0)
    {
        cout << "[error] "
             << "多视角采集失败！" << endl;
        return 2;
    }

    // 三维重建
    cout << "[info] "
         << "三维重建...！" << endl;
    tsdf_fusion_.Fusion(tsdf_folder, multi_num, rough_pos, ply_path);

    // 精定位
    cout << "[info] "
         << "精定位...！" << endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tsdf_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile(ply_path, *tsdf_cloud);
    is_ok = oil_accurate_detecter_.detect_once(tsdf_cloud);
    oil_accurate_detecter_.saveDataFrame(tsdf_folder, "0");

    if (is_ok != 0)
    {
        cout << "[error] "
             << "精定位失败！" << endl;
        return 3;
    }

    oil_pos_ = oil_accurate_detecter_.getPosition();
    oil_quat_ = oil_accurate_detecter_.getQuaternion();

    tf::Quaternion quat(oil_quat_[0], oil_quat_[1], oil_quat_[2], oil_quat_[3]);
    tf::Matrix3x3 rota(quat);
    double roll, pitch, yaw;
    rota.getRPY(roll, pitch, yaw);
    std::cout << "[OilDetectTsdf] eula angle: " << roll / M_PI * 180 << "," << pitch / M_PI * 180 << "," << yaw / M_PI * 180 << std::endl;
    std::cout << "[OilDetectTsdf] oil_pos_: " << oil_pos_[0] << "," << oil_pos_[1] << "," << oil_pos_[2] << std::endl;

    std::cout << "[INFO] Exit oil filter detector...\n";

    return 0;
}

int OilDetectTsdf::multiViewDataCollect(float *oil_position, std::string output_folder)
{
    // static const std::string PLANNING_GROUP = "arm";
    static const std::string PLANNING_GROUP = "panda_arm";

    //运动规划接口
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    //规划环境接口，如障碍物设置
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // TODO: 测试
    init_target_x_ = 0;
    init_target_y_ = 0;
    init_target_z_ = 0;
    geometry_msgs::Pose pose = move_group.getCurrentPose().pose;
    pose.position.z -= 1 * 0.1;
    pose.position.y += 1 * 0.2;
    init_waypoints_.push_back(pose);
    pose.position.x += 1 * 0.1;
    init_waypoints_.push_back(pose);
    pose.position.y -= 1 * 0.1;
    init_waypoints_.push_back(pose);

    move_group.setMaxVelocityScalingFactor(0.4);

    // 根据设定的路径及对应的目标位置和检测的目标位置，调整设定的路径
    std::vector<geometry_msgs::Pose> waypoints;
    for (auto pose : init_waypoints_)
    {
        geometry_msgs::Pose tmp;
        tmp.orientation = pose.orientation;
        tmp.position.x = pose.position.x + (oil_position[0] - init_target_x_);
        tmp.position.y = pose.position.y + (oil_position[1] - init_target_y_);
        tmp.position.z = pose.position.z + (oil_position[2] - init_target_z_);
        waypoints.push_back(tmp);
    }

    // 运动到起始位置
    move_group.setPoseTarget(*waypoints.begin());
    auto is_ok = move_group.move();
    if (is_ok == false)
        return -1;

    // 通过缩放因子降低关节最大速度，笛卡尔路径规划经常会将机械臂运动速度设置慢一些
    move_group.setMaxVelocityScalingFactor(0.3);

    // 根据路径点计算插补路径
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    cout << "[info]"
         << "fraction: " << fraction << endl;

    //　运行
    if (fraction * 100.0 < 99)
    {
        return 0;
    }

    topic_capture_->start();
    move_group.execute(trajectory);
    topic_capture_->stop();

    return topic_capture_->getFrameNums();
}

void OilDetectTsdf::imageViewer(int loop_rate)
{
    static int frame_rate_ = 0;
    std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
    double fps = 0;

    std::ostringstream oss;
    const cv::Point pos(5, 15);
    const cv::Scalar colorText = CV_RGB(255, 255, 255);
    const double sizeText = 0.5;
    const int lineText = 1;
    const int font = cv::FONT_HERSHEY_SIMPLEX;

    cv::namedWindow("Image Viewer");
    oss << "starting...";

    start = std::chrono::high_resolution_clock::now();

    ros::Rate rate(loop_rate);
    bool running = true;
    size_t frameCount = 0;
    for (; running && ros::ok();)
    {
        ++frameCount;
        now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
        if (elapsed >= 1.0)
        {
            fps = int(frameCount / elapsed);
            oss.str("");
            oss << "fps: " << fps << " ( " << int(1000.0 / fps) << " ms)";
            start = now;
            frameCount = 0;
        }
        auto color_draw = img_receiver_->getColor().clone();
        if (!color_draw.empty())
        {
            cv::Mat color_show(color_draw);
            if (color_draw.cols > 800)
            {
                resize(color_show, color_show, cv::Size(0, 0), 0.7, 0.7, cv::INTER_LINEAR);
            }
            cv::putText(color_show, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);
            cv::imshow("Image Viewer", color_show);
        }

        int key = cv::waitKey(1);
        switch (key & 0xFF)
        {
        case 27:
        case 'q':
            running = false;
            break;
        }

        rate.sleep();
    }
    cv::destroyAllWindows();
    cv::waitKey(100);
}

void OilDetectTsdf::cloudViewer(int loop_rate)
{
    // PCLVisualizer初始化
    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    const std::string cloudName = "rendered";
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());

    visualizer->addPointCloud(cloud, cloudName);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    visualizer->initCameraParameters();
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->setShowFPS(true);
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);

    ros::Rate rate(loop_rate); // 与采集频率接近即可
    while (ros::ok() && !visualizer->wasStopped())
    {
        visualizer->removeAllShapes();

        pcl::copyPointCloud(*img_receiver_->getCloud(), *cloud); // copy原始点云

        // 更新点云显示
        visualizer->updatePointCloud(cloud, cloudName);
        visualizer->spinOnce(10);
        rate.sleep();
    }
}

void OilDetectTsdf::show(int loop_rate)
{
    // 启动图像显示线程
    std::thread image_viewer_thread = std::thread(&OilDetectTsdf::imageViewer, this, loop_rate);
    image_viewer_thread.detach(); // 将子线程从主线程里分离

    // 启动点云显示线程
    std::thread cloud_viewer_thread = std::thread(&OilDetectTsdf::cloudViewer, this, loop_rate);
    cloud_viewer_thread.detach(); // 将子线程从主线程里分离

    printf("[INFO] show image and cloud...\n");
}