#include "tsdf_fusion/topics_capture.h"
#include <ostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

void saveCameraPose(const TopicsCapture::CameraPoseMsg camera_pose, std::string save_path = "")
{
    tf::StampedTransform transform;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(camera_pose.pose.orientation, quat);

    tf::Matrix3x3 roat(quat);

    if (!save_path.empty())
    {
        std::ofstream OutFile(save_path);
        OutFile << roat.getRow(0).getX() << " " << roat.getRow(0).getY() << " " << roat.getRow(0).getZ() << " " << camera_pose.pose.position.x << std::endl;
        OutFile << roat.getRow(1).getX() << " " << roat.getRow(1).getY() << " " << roat.getRow(1).getZ() << " " << camera_pose.pose.position.y << std::endl;
        OutFile << roat.getRow(2).getX() << " " << roat.getRow(2).getY() << " " << roat.getRow(2).getZ() << " " << camera_pose.pose.position.z << std::endl;
        OutFile << 0 << " " << 0 << " " << 0 << " " << 1;

        OutFile.close();
    }
}

TopicsCapture::TopicsCapture(const std::string depth_img_topic_name, const std::string camera_pose_topic_name, const std::string save_folder)
    : depth_name_(depth_img_topic_name), pose_name_(camera_pose_topic_name), save_folder_(save_folder)
{
    frame_nums_ = 0;

    camera_pose_sub_ = new message_filters::Subscriber<CameraPoseMsg>(nh_, pose_name_, 1);
    depth_img_sub_ = new message_filters::Subscriber<DepthImageMsg>(nh_, depth_name_, 1);

    stop();

    sync_ = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *camera_pose_sub_, *depth_img_sub_);
    sync_->registerCallback(boost::bind(&TopicsCapture::subCallBack, this, _1, _2));
}
TopicsCapture::~TopicsCapture()
{
}

void TopicsCapture::subCallBack(const CameraPoseMsg::ConstPtr &camera_pose, const DepthImageMsg::ConstPtr &depth_img)
{
    //TODO: 保存信息
    std::stringstream oss;
    oss.str("");
    oss << std::setfill('0') << std::setw(2) << frame_nums_++;

    std::string camera_pose_path = save_folder_ + "/frame_" + oss.str() + "_pose" + ".txt";
    saveCameraPose(*camera_pose, camera_pose_path);

    std::string depth_path = save_folder_ + "/frame_" + oss.str() + "_depth" + ".png";
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(depth_img, depth_img->encoding);
    cv::imwrite(depth_path, pCvImage->image);

    std::cout << "[OilDetectTsdf] save frame " << oss.str() << " data to " << save_folder_ << std::endl;
}