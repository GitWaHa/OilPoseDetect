#!/usr/bin/env python
#coding=utf-8

import os
import sys
import copy
import math
import threading
import numpy as np

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import tf
import tf.transformations as tf_trans

import tf2_ros as tf2
import tf2_geometry_msgs as tf2_gm

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class OilApp(object):
    X = 0
    Y = 1
    Z = 2

    def __init__(self, server, server_name):
        super(OilApp, self).__init__()
        self.oil_track_ = server
        self.server_name_ = server_name

    def findOilHole(self, flag=None):
        if flag == None:
            (is_find, position, orientation) = self.oil_track_.find()
        else:
            (is_find, position, orientation) = self.oil_track_.find(flag)

        pose_ar_tag = tf2_gm.PoseStamped()

        # # 实际使用
        pose_ar_tag.pose.position.x = position[0]
        pose_ar_tag.pose.position.y = position[1]
        pose_ar_tag.pose.position.z = position[2]

        pose_ar_tag.pose.orientation.x = orientation[0]
        pose_ar_tag.pose.orientation.y = orientation[1]
        pose_ar_tag.pose.orientation.z = orientation[2]
        pose_ar_tag.pose.orientation.w = orientation[3]

        # # # 测试使用
        #       position:
        #     x: -0.0225854032456
        #     y: 0.397178437781
        #     z: 1.16692480216
        # orientation:
        #     x: 0.457271815237
        #     y: 0.00695638337351
        #     z: -0.0385852559266
        #     w: -0.888462308568
        is_find = True
        pose_ar_tag.pose.position.x = -0.02624163531422455
        pose_ar_tag.pose.position.y = 0.42376444923208506
        pose_ar_tag.pose.position.z = 1.180509593819837

        pose_ar_tag.pose.orientation.x = 0.457271815237
        pose_ar_tag.pose.orientation.y = 0.00695638337351
        pose_ar_tag.pose.orientation.z = -0.0385852559266
        pose_ar_tag.pose.orientation.w = -0.888462308568

        pose_ar_tag.header.frame_id = "base_link"

        # if (self.server_name_ == "pcd"):
        #     return is_find, pose_ar_tag
        if (self.server_name_ == "aruco"):
            # 将二维码坐标系转换坐标到加油口
            pose_oil = self.translationPoseFromEnd(pose_ar_tag, OilApp.Y,
                                                   -0.147)
            pose_oil = self.translationPoseFromEnd(pose_oil, OilApp.X, 0.04)
            print("[transformPoseFromEnd] pose_oil", pose_oil)

            # 翻转使得z轴朝外,即末端夹爪坐标系z轴
            pose_out_turn = self.rotationFrameAxis(pose_oil, OilApp.Y, math.pi)

            # 旋转z轴
            pose_out_turn = self.rotationFrameAxis(pose_out_turn, OilApp.Z,
                                                   -math.pi / 2)

            return is_find, pose_out_turn

        return is_find, pose_ar_tag

    def translationPoseFromEnd(self, pose, direction, distance):
        '''
        # 以末端坐标系为基坐标系，沿着设定方向（x/y/z）移动一定距离
        # pose： 机械臂末端当前pose
        # direction: 沿着机械臂当前末端坐标系移动的方向
        # distance: 移动的距离
        '''
        transform2 = tf2.TransformStamped()
        transform2.transform.translation.x = pose.pose.position.x
        transform2.transform.translation.y = pose.pose.position.y
        transform2.transform.translation.z = pose.pose.position.z

        transform2.transform.rotation.x = pose.pose.orientation.x
        transform2.transform.rotation.y = pose.pose.orientation.y
        transform2.transform.rotation.z = pose.pose.orientation.z
        transform2.transform.rotation.w = pose.pose.orientation.w

        pose1 = tf2_gm.PoseStamped()
        if (direction == OilApp.X):
            pose1.pose.position.x = distance
        elif (direction == OilApp.Y):
            pose1.pose.position.y = distance
        elif (direction == OilApp.Z):
            pose1.pose.position.z = distance

        pose1.pose.orientation.w = 1

        pose2 = tf2_gm.do_transform_pose(pose1, transform2)

        pose2.header.frame_id = "base_link"

        return pose2

    def oilCartesianPath(self,
                         move_group,
                         direction=2,
                         distance_in=0.09,
                         max_num=200,
                         debug=False):
        ## 笛卡尔路径规划，加油动作
        # 设置路径点
        waypoints = []
        wpose = move_group.get_current_pose()

        step = 1

        for i in range(step + 1):
            # print(i)
            pose_in = self.translationPoseFromEnd(wpose, direction,
                                                  distance_in / step * i)
            waypoints.append(copy.deepcopy(pose_in.pose))

        traj_ls = []
        traj_len_ls = []
        fraction_ls = []
        # 根据路径点计算插补路径
        for i in range(max_num):
            (plan, fraction) = move_group.compute_cartesian_path(
                waypoints, 0.01, 0.0)

            # sys.exit()
            if (fraction >= 0.99):
                traj_len = len(plan.joint_trajectory.points)
                # print("[INFO] Traj", i, " len:", traj_len, "fraction:",
                #       fraction)
                is_ok = True
                pre_pos = move_group.get_current_joint_values()
                for joints_value in plan.joint_trajectory.points:
                    pos = joints_value.positions

                    if (np.max(np.abs(np.array(pre_pos) - np.array(pos))) >
                            0.04):
                        if debug:
                            print(
                                "is not ok",
                                np.max(
                                    np.abs(np.array(pre_pos) - np.array(pos))))
                        is_ok = False
                        break
                    else:
                        # print("is ok",
                        #       np.max(np.abs(np.array(pre_pos) -
                        #                     np.array(pos))))
                        pass
                    pre_pos = pos

                if not is_ok:
                    continue

                traj_ls.append(plan)
                traj_len_ls.append(traj_len)
                fraction_ls.append(fraction)
            else:
                traj_len = len(plan.joint_trajectory.points)
                # print("[INFO] invalid Traj", i, " len:", traj_len, "fraction:",
                #       fraction)

        if (len(traj_ls) == 0):
            rospy.logwarn("oilCartesianPath result is none")
            return None, 0

        traj_len_set = set(traj_len_ls)
        traj_len_count = []
        traj_len_count_list = []
        for data in traj_len_set:
            traj_len_count_list.append(data)
            traj_len_count.append(traj_len_ls.count(data))
            if debug:
                print(data, traj_len_ls.count(data))

        set_index = traj_len_count.index(max(traj_len_count))
        better_traj_idx = traj_len_ls.index(traj_len_count_list[set_index])

        better_traj = traj_ls[better_traj_idx]
        better_traj_len = traj_len_count_list[set_index]
        better_fraction = fraction_ls[better_traj_idx]

        return (better_traj, better_fraction)

    def rotationFrameAxis(self, pose, axis, angle):
        transform2 = tf2.TransformStamped()
        transform2.transform.translation.x = pose.pose.position.x
        transform2.transform.translation.y = pose.pose.position.y
        transform2.transform.translation.z = pose.pose.position.z

        transform2.transform.rotation.x = pose.pose.orientation.x
        transform2.transform.rotation.y = pose.pose.orientation.y
        transform2.transform.rotation.z = pose.pose.orientation.z
        transform2.transform.rotation.w = pose.pose.orientation.w

        quaternion = [0] * 4
        if (axis == OilApp.X):
            quaternion = tf_trans.quaternion_from_euler(angle, 0, 0)
        elif (axis == OilApp.Y):
            quaternion = tf_trans.quaternion_from_euler(0, angle, 0)
        elif (axis == OilApp.Z):
            quaternion = tf_trans.quaternion_from_euler(0, 0, angle)

        pose1 = tf2_gm.PoseStamped()
        pose1.pose.orientation.x = quaternion[0]
        pose1.pose.orientation.y = quaternion[1]
        pose1.pose.orientation.z = quaternion[2]
        pose1.pose.orientation.w = quaternion[3]

        pose2 = tf2_gm.do_transform_pose(pose1, transform2)

        pose2.header.frame_id = "base_link"

        return pose2

    def scale_trajectory_speed(self, traj, scale):
        new_traj = RobotTrajectory()
        new_traj.joint_trajectory = traj.joint_trajectory

        n_joints = len(traj.joint_trajectory.joint_names)
        n_points = len(traj.joint_trajectory.points)
        points = list(traj.joint_trajectory.points)

        for i in range(n_points):
            point = JointTrajectoryPoint()
            point.positions = traj.joint_trajectory.points[i].positions

            point.time_from_start = traj.joint_trajectory.points[
                i].time_from_start / scale
            point.velocities = list(traj.joint_trajectory.points[i].velocities)
            point.accelerations = list(
                traj.joint_trajectory.points[i].accelerations)

            for j in range(n_joints):
                point.velocities[j] = point.velocities[j] * scale
                point.accelerations[j] = point.accelerations[j] * scale * scale
            points[i] = point

        new_traj.joint_trajectory.points = points
        return new_traj

    def find_better_traj(self,
                         group,
                         pose_goal,
                         max_num=50,
                         is_turn=False,
                         debug=False):
        # get short traj
        traj_ls = []
        traj_len_ls = []
        pose_goal_ls = []

        angles = [0]
        if is_turn:
            angles = [0, math.pi]

        for angle in angles:
            print("\n\n[INFO] Try angle:", angle)

            # rotate around z
            # pose_goal = pose_rot_around_axis(pose_goal, angle, 2)
            pose_goal = self.rotationFrameAxis(pose_goal, OilApp.Z, angle)

            # print("[INFO] Pose goal:", pose_goal)
            # print("[INFO] Euler:",
            #       euler_from_quaternion([
            #           pose_goal.pose.orientation.x,
            #           pose_goal.pose.orientation.y,
            #           pose_goal.pose.orientation.z,
            #           pose_goal.pose.orientation.z
            #       ]))
            max_failed = 5
            for i in range(max_num):
                if max_failed < 0 or rospy.is_shutdown():
                    if debug:
                        print(
                            "Maximum number of failures reached or rospy.is_shutdown()"
                        )
                    return None, 0, None
                group.set_start_state_to_current_state()
                group.set_pose_target(pose_goal)
                traj = group.plan()

                traj_len = len(traj.joint_trajectory.points)
                if debug:
                    print("[INFO] Traj", i, " len:", traj_len)
                if traj_len == 0:
                    max_failed -= 1
                    continue

                traj_ls.append(traj)
                traj_len_ls.append(traj_len)
                pose_goal_ls.append(pose_goal)

        better_traj_idx = traj_len_ls.index(min(traj_len_ls))
        better_traj = traj_ls[better_traj_idx]
        better_traj_len = traj_len_ls[better_traj_idx]
        better_pose_goal = pose_goal_ls[better_traj_idx]

        print("[INFO] Better traj index:", better_traj_idx)
        print("[INFO] Better traj len:", better_traj_len)

        if better_traj_len == 0:
            print("[WARN] Not find better traj")

        return better_traj, better_traj_len, better_pose_goal

    # def getGraspPose(self, pose):
    #     pose.
    #     euler = tf_trans.euler_from_quaternion([0, 0, 0, 1])
