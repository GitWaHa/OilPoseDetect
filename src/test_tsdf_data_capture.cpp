#include <iostream>
#include "fusion/topics_capture.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_tsdf_data_capture");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    // TopicsCapture topic_capture("/camera/aligned_depth_to_color/image_raw", "/camera/pose", "/home/waha/Desktop/test/reconstruct_data");
    TopicsCapture topic_capture("/camera/depth/image_align", "/camera/rgb/image_rect_color", "/camera/pose", "/home/waha/Desktop/test/reconstruct_data");
    topic_capture.start();

    ros::Rate loop(1);
    int pre_count = topic_capture.getFrameNums();
    while (!ros::isShuttingDown())
    {
        /* code */
        int cur_count = topic_capture.getFrameNums();
        std::cout << "[info] fps: " << cur_count - pre_count << std::endl;
        pre_count = cur_count;
        loop.sleep();
    }
    topic_capture.stop();

    return 0;
}