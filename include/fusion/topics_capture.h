#pragma once

#include <ros/ros.h>

#include <std_srvs/SetBool.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

class TopicsCapture
{
public:
    using CameraPoseMsg = geometry_msgs::PoseStamped;
    // using CameraPoseMsg = sensor_msgs::Image;
    using DepthImageMsg = sensor_msgs::Image;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<CameraPoseMsg, DepthImageMsg>;
    // using SyncPolicy = message_filters::sync_policies::ExactTime<CameraPoseMsg, DepthImageMsg>;

public:
    TopicsCapture(const std::string depth_img_topic_name, const std::string camera_pose_topic_name, const std::string save_folder);
    ~TopicsCapture();

    void subCallBack(const CameraPoseMsg::ConstPtr &camera_pose, const DepthImageMsg::ConstPtr &depth_img);

    void start()
    {
        frame_nums_ = 0;
        std::cout << "[info]"
                  << "start topic capture...." << std::endl;
        depth_img_sub_->subscribe();
        camera_pose_sub_->subscribe();
    }

    void stop()
    {
        depth_img_sub_->unsubscribe();
        camera_pose_sub_->unsubscribe();
        // sync_->disconnectAll();
        std::cout << "[info]"
                  << "stop topic capture...." << std::endl;
    }

    int getFrameNums()
    {
        return frame_nums_;
    }

private:
    /* data */
    ros::NodeHandle nh_;

    message_filters::Subscriber<DepthImageMsg> *depth_img_sub_;
    message_filters::Subscriber<CameraPoseMsg> *camera_pose_sub_;
    message_filters::Synchronizer<SyncPolicy> *sync_;

    std::string depth_name_;
    std::string pose_name_;
    std::string save_folder_;

    int frame_nums_;
};
