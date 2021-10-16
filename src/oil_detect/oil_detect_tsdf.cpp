#include "oil_detect/oil_detect_tsdf.h"

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <pcl/io/ply_io.h>

oilDetectTsdf::oilDetectTsdf(std::shared_ptr<CameraReceiver> img_receiver) : img_receiver_(img_receiver)
{
    img_receiver->run();
}

oilDetectTsdf::~oilDetectTsdf()
{
}

void oilDetectTsdf::run(int loop_rate, int flag)
{
    ros::Rate rate(loop_rate); // 与采集频率接近即可
    std::string tsdf_folder = "/home/waha/Desktop/test_data/";
    std::string ply_path = tsdf_folder + "tsdf_cloud.ply";
    int is_ok = -1;
    while (ros::ok())
    {
        oil_pose_valid_ = false;
        // // 初步定位
        // float rough_pos[3];
        // flag = oil_rough_detecter_.detect_once(img_receiver_->getColor(),
        //                                        img_receiver_->getDepth(),
        //                                        img_receiver_->getCloud(), rough_pos);
        // if (flag != 0)
        //     continue;

        int multi_num = 3;
        if ((flag & 1) != 0)
        {
            multi_num = multiViewDataCollect(tsdf_folder);
            if (multi_num == 0)
                break;
        }

        // 三维重建
        tsdf_fusion_.Fusion(tsdf_folder, multi_num, ply_path);

        // 精定位
        pcl::PointCloud<pcl::PointXYZ>::Ptr tsdf_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPLYFile(ply_path, *tsdf_cloud);
        is_ok = oil_accurate_detecter_.detect_once(tsdf_cloud);
        oil_accurate_detecter_.saveDataFrame(tsdf_folder, "0");

        // if (is_ok != 0)
        //     break;

        oil_pos_ = oil_accurate_detecter_.getPosition();
        oil_quat_ = oil_accurate_detecter_.getQuaternion();
        oil_pose_valid_ = true;

        tf::Quaternion quat(oil_quat_[0], oil_quat_[1], oil_quat_[2], oil_quat_[3]);
        tf::Matrix3x3 rota(quat);
        double roll, pitch, yaw;
        rota.getRPY(roll, pitch, yaw);
        std::cout << "[oilDetectTsdf] eula angle: " << roll / M_PI * 180 << "," << pitch / M_PI * 180 << "," << yaw / M_PI * 180 << std::endl;
        std::cout << "[oilDetectTsdf] oil_pos_: " << oil_pos_[0] << "," << oil_pos_[1] << "," << oil_pos_[2] << std::endl;

        // std::thread pub_tf_(publishTF);
        // pub_tf_.join();

        rate.sleep();

        break;
    }

    std::cout << "[INFO] Exit oil filter detector...\n";
    // img_receiver_->stop();
}

int oilDetectTsdf::multiViewDataCollect(std::string output_folder)
{
    static const std::string PLANNING_GROUP = "arm";

    //运动规划接口
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    //规划环境接口，如障碍物设置
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    move_group.setMaxVelocityScalingFactor(0.4);

    std::stringstream oss;
    int count = 0;
    for (std::string name_target : multi_arm_name_)
    {
        std::cout << "[oilDetectTsdf] set arm name target: " << name_target << std::endl;
        move_group.setNamedTarget(name_target);
        auto is_ok = move_group.move();
        if (is_ok == false)
            break;

        sleep(1);

        float rough_pos[3];
        int flag = oil_rough_detecter_.detect_once(img_receiver_->getColor(),
                                                   img_receiver_->getDepth(),
                                                   img_receiver_->getCloud(), rough_pos);

        oss.str("");
        oss << std::setfill('0') << std::setw(2) << count++;
        oil_rough_detecter_.saveDataFrame(output_folder, oss.str());

        std::cout << "[oilDetectTsdf] save frame " << oss.str() << " data to " << output_folder << std::endl;
    }
    if (count != multi_arm_name_.size())
        std::cout << "[oilDetectTsdf] The collected data does not meet the requirements "
                  << count << "/" << multi_arm_name_.size() << std::endl;

    return count;
}

void oilDetectTsdf::publishTF()
{
    ros::Rate rate(15);

    while (oil_pose_valid_ && ros::ok())
    {
        tf::Quaternion tf_quat(oil_quat_[0], oil_quat_[1], oil_quat_[2], oil_quat_[3]);
        tf::Transform oil_tf = tf::Transform(tf_quat, tf::Vector3(oil_pos_[0], oil_pos_[1], oil_pos_[2]));
        tf::StampedTransform oil_stf(oil_tf, ros::Time::now(), "base_link", "oil_filler");

        broadcaster_.sendTransform(oil_stf);

        rate.sleep();
    }
}