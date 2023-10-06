#!/usr/bin/env python3
# -*- coding: utf-8 -*-

##################################################
# File: moveit_attached_object_demo.py
# Implementation of Experiment 4: Object avoidance
# The obstacle is modified as requirment.
##################################################

import rospy, sys
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Quaternion
# from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from math import pi
from tf.transformations import quaternion_from_euler  
# import numpy as np

class MoveAttachedObjectDemo:
    def __init__(self):
        ##########################
        ##### Initialization #####
        ##########################
        
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_attached_object_demo')
        
        # 初始化场景对象
        scene = PlanningSceneInterface()
        rospy.sleep(1)
                                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('interbotix_arm')
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
        
        # 设置机械臂运动的允许误差值
        arm.set_goal_joint_tolerance(0.001)

        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        
        # 移除场景中之前运行残留的物体
        scene.remove_attached_object(end_effector_link, 'tool')
        scene.remove_world_object('table') 
        
        ##############################
        ##### Set table and tool #####
        ##############################
        
        # 设置桌面的高度
        table_ground = 0.45  # 0.45
        
        # 设置table和tool的三维尺寸
        ratio = 1.5**(1/3)
        table_size = [0.1*ratio, 0.3*ratio, 0.02*ratio]
        tool_size = [0.2, 0.02, 0.02]
        
        # 设置tool的位姿
        p = PoseStamped()
        p.header.frame_id = end_effector_link
        
        p.pose.position.x = tool_size[0] - 0.03
        p.pose.position.y = 0
        p.pose.position.z = -0.015
        p.pose.orientation.w = 1
        
        # 将tool附着到机器人的终端
        scene.attach_box(end_effector_link, 'tool', p, tool_size)

        # 将table加入场景当中
        table_pose = PoseStamped()
        table_pose.header.frame_id = 'world'
        table_pose.pose.position.x = 0.4
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        q_angle = quaternion_from_euler(0, pi/2, 0, axes='sxyz')
        q = Quaternion(*q_angle)  
        # print(q)
        table_pose.pose.orientation = q
        # table_pose.pose.orientation.w = 1
        scene.add_box('table', table_pose, table_size)
        
        rospy.sleep(2)  

        # 控制机械臂回到初始化位置
        arm.set_named_target('Home')
        arm.go()
        rospy.sleep(1)

        # 设置每次运动规划的时间限制：10s
        arm.set_planning_time(10)

        # 更新当前的位姿
        arm.set_start_state_to_current_state()

        ################
        ##### Move #####
        ################
        
        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        # joint_positions = [0.00153, -0.37429, 0.38502, 0.012271, -0.86209, -0.00153]
        joint_positions = [0.00153, -0.37429, 0.7, 0.012271, -0.86209, -0.00153]
        arm.set_joint_value_target(joint_positions)
                 
        ##########################
        ##### Reset and Exit #####
        ##########################
        
        # 控制机械臂完成运动
        arm.go()
        rospy.sleep(1)
        
        # 控制机械臂回到初始化位置
        arm.set_named_target('Sleep')
        arm.go()
        rospy.sleep(1)

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveAttachedObjectDemo()

    
