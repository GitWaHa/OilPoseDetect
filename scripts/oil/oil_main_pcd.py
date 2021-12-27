#!/usr/bin/env python
#coding=utf-8

import os
import sys
import copy
import math
import threading
import moveit_ros_planning_interface

import rospy
import moveit_commander
from moveit_commander.move_group import MoveGroupCommander
import moveit_msgs.msg
import geometry_msgs.msg

import tf
import tf.transformations as tf_trans

import tf2_ros as tf2
import tf2_geometry_msgs as tf2_gm

from oil_pose_server import OilPoseServer
from oil_app import OilApp

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('oil_app', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_max_velocity_scaling_factor(0.4)  # 设置最大关节速度
    # move_group.set_goal_joint_tolerance(0.1)
    move_group.set_goal_tolerance(0.0001)

    oil_server = OilPoseServer("base_link", "oil_filler")
    oil = OilApp(oil_server, "pcd")

    # 回到home
    # move_group.set_named_target("home")
    # if (not move_group.go()):
    #     rospy.logerr("home is go filed")
    #     sys.exit()
    # move_group.clear_pose_targets()
    # rospy.sleep(2)
    # names = ["oil_look_pcd", "oil_look_pcd_left"]
    names = ["look_mid"]
    for name in names:
        # 设置为寻找二维码时的关节
        move_group.set_named_target(name)
        if (not move_group.go()):
            rospy.logerr("oil_look_pcd is go filed")
            sys.exit()
        move_group.clear_pose_targets()
        rospy.sleep(5)

        # 粗定位 point cloud 识别加油口
        is_find, pose_oil = oil.findOilHole()
        if (not is_find):
            print(" \033[1;31m ArTag is not find \033[0m")
            sys.exit()
        print("[findOilHole] pose_oil", pose_oil)

        # 沿着加油口坐标系 后退一定距离作为加油起始点
        pose_look = oil.translationPoseFromEnd(pose_oil, OilApp.Z, -0.40)
        pose_look = oil.translationPoseFromEnd(pose_look, OilApp.Y, 0.05)
        pose_look = oil.rotationFrameAxis(pose_look, OilApp.X,
                                          -10 * math.pi / 180)
        print("[transformPoseFromEnd] pose_look", pose_look)

        # move_group.set_pose_target(pose_out)
        traj, traj_len, better_pose_goal = oil.find_better_traj(
            move_group, pose_look)
        if traj_len != 0:
            move_group.execute(traj, wait=True)
        else:
            print(" \033[1;31m pose_out move_group.go() is failed \033[0m")
            sys.exit()

        rospy.sleep(5)

        # sys.exit()

        # 精定位 point cloud 识别加油口
        is_find, pose_oil = oil.findOilHole()
        if (not is_find):
            print(" \033[1;31m ArTag is not find \033[0m")
            sys.exit()
        print("[findOilHole] pose_oil", pose_oil)

        # 微调
        pose_oil = oil.translationPoseFromEnd(pose_oil, OilApp.X, -0.003)
        pose_oil = oil.translationPoseFromEnd(pose_oil, OilApp.Y, 0.003)

        # 沿着加油口坐标系 后退一定距离作为加油起始点
        pose_out = oil.translationPoseFromEnd(pose_oil, OilApp.Z, -0.30)
        print("[transformPoseFromEnd] pose_out", pose_out)

        # move_group.set_pose_target(pose_out)
        traj, traj_len, better_pose_goal = oil.find_better_traj(move_group,
                                                                pose_out,
                                                                debug=False)
        if traj_len != 0:
            move_group.execute(traj, wait=True)
        else:
            print(" \033[1;31m pose_out move_group.go() is failed \033[0m")
            sys.exit()

        max_plan = 10
        ## 笛卡尔路径规划，加油动作
        for i in range(max_plan):
            (plan, fraction) = oil.oilCartesianPath(move_group,
                                                    direction=OilApp.Z,
                                                    distance_in=0.08)
            print("compute_cartesian_path fraction:", fraction)

            if (fraction >= 0.95):
                traj_len = len(plan.joint_trajectory.points)
                print("traj_len in", traj_len)
                for data in plan.joint_trajectory.points:
                    print(data.positions)
                # print("compute_cartesian_path fraction:", fraction)
                plan = oil.scale_trajectory_speed(plan, 0.3)
                move_group.execute(plan)
                break
            else:
                rospy.logerr("compute_cartesian_path is failed")
                if i == (max_plan - 1):
                    sys, exit()

        rospy.sleep(5)

        ## 笛卡尔路径规划，退出加油动作
        for i in range(max_plan):
            (plan, fraction) = oil.oilCartesianPath(move_group,
                                                    direction=OilApp.Z,
                                                    distance_in=-0.08)
            print("compute_cartesian_path fraction:", fraction)

            if (fraction >= 0.95):
                traj_len = len(plan.joint_trajectory.points)
                print("traj_len out", traj_len)
                for data in plan.joint_trajectory.points:
                    print(data.positions)
                plan = oil.scale_trajectory_speed(plan, 0.3)
                move_group.execute(plan)
                break
            else:
                rospy.logerr("compute_cartesian_path is failed")
                if i == (max_plan - 1):
                    sys, exit()
