#!/usr/bin/env python
#coding=utf-8

import os
import sys
import copy
import math

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import tf
import tf.transformations as tf_trans

import tf2_ros as tf2
import tf2_geometry_msgs as tf2_gm

from oil_pose_detect.srv import DetectOilWithReconstruct, DetectOilWithReconstructRequest, DetectOilWithReconstructResponse


class OilPoseServer(object):
    def __init__(self, target_frame, source_frame):
        super(OilPoseServer, self).__init__()
        self.listener_ = tf.TransformListener()
        self.target_frame_ = target_frame
        self.source_frame_ = source_frame
        self.max_time_ = 5

    def find(self):
        is_find = False
        position = [0, 0, 0]
        orientation = [0, 0, 0, 1]

        try:
            self.listener_.waitForTransform(self.target_frame_,
                                            self.source_frame_, rospy.Time(0),
                                            rospy.Duration(self.max_time_))

            (position, orientation) = self.listener_.lookupTransform(
                self.target_frame_, self.source_frame_, rospy.Time(0))
            is_find = True
        except:
            is_find = False

        print((position, orientation))
        return (is_find, position, orientation)

    def setMaxTime(self, max_time):
        self.max_time_ = max_time


class OilPoseReconstructServer(object):
    def __init__(
        self,
        service_name="/detect_oil_with_reconstruct/oil_detect_with_reconstruct"
    ):
        super(OilPoseReconstructServer, self).__init__()

        self.client_ = rospy.ServiceProxy(service_name,
                                          DetectOilWithReconstruct)

    def find(self, flag):
        is_find = True
        position = [0, 0, 0]
        orientation = [0, 0, 0, 1]

        position, orientation = self.clientCall(flag)

        print((position, orientation))
        return (is_find, position, orientation)

    def clientCall(self, flag):
        print("---------- client ---------")
        # 设置request
        request = DetectOilWithReconstructRequest()
        request.flag = flag

        response = DetectOilWithReconstructResponse()

        response = self.client_.call(request)

        return response.trans, response.quat
