#!/usr/bin/env python
#coding=utf-8

import rospy
import geometry_msgs.msg as geometry_msgs

import tf
import sys
import time
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped


class Publisher(object):
    def __init__(self, base_frame_name, camera_frame_name):
        super(Publisher, self).__init__()
        self.base_frame_name_ = base_frame_name
        self.camera_frame_name_ = camera_frame_name
        self.listener_ = tf.TransformListener()
        self.pub_ = rospy.Publisher("/camera/pose", PoseStamped, queue_size=10)

    def listen(self):

        self.listener_.waitForTransform(self.base_frame_name_,
                                        self.camera_frame_name_, rospy.Time(0),
                                        rospy.Duration(1))
        '''
        返回　源坐标系相对与目标坐标系的位置，姿态
        target_frame, 
        source_frame, 
        time
        '''
        (position, orientation) = self.listener_.lookupTransform(
            self.base_frame_name_, self.camera_frame_name_,
            rospy.Time(0))  # frame_name_相对于world的位置姿态
        camera_pose = PoseStamped()
        time_now = time.time()
        camera_pose.header.stamp.set(int(time_now),
                                     (time_now - int(time_now)) / 1000000000.)
        camera_pose.pose.position.x = position[0]
        camera_pose.pose.position.y = position[1]
        camera_pose.pose.position.z = position[2]
        camera_pose.pose.orientation.x = orientation[0]
        camera_pose.pose.orientation.y = orientation[1]
        camera_pose.pose.orientation.z = orientation[2]
        camera_pose.pose.orientation.w = orientation[3]
        self.pub_.publish(camera_pose)


if __name__ == "__main__":
    rospy.init_node("camera_pose_publish", anonymous=True)

    rospy.loginfo("camera_pose_publish is start....")
    print("[info] base_frame:{}, camera_frame:{}".format(
        rospy.get_param("~base_frame", "world"),
        rospy.get_param("~camera_frame", "panda_grasp_frame")))

    pub = Publisher(rospy.get_param("~base_frame", "world"),
                    rospy.get_param("~camera_frame", "panda_grasp_frame"))

    pose = tf2_geometry_msgs.PoseStamped()
    transform = tf2_ros.TransformStamped()
    tf2_geometry_msgs
    tf2_geometry_msgs.do_transform_pose(pose, transform)

    rate = rospy.Rate(60)
    while (not rospy.is_shutdown()):
        pub.listen()
        rate.sleep()
