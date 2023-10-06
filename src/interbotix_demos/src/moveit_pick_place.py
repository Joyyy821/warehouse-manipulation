#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#########################################################################
# File: moveit_pick_place.py
# Implementation of Experiment 3 Step 2: pick and place task.
# Note that the cartesian path planning method is implemented in the 
# file moveit_cartesian_demo.py
#########################################################################

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Quaternion
from math import pi
from tf.transformations import quaternion_from_euler 

class PickAndPlace:
    def __init__(self):
        ##########################
        ##### Initialization #####
        ##########################
        
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = moveit_commander.MoveGroupCommander("interbotix_arm")

        # 初始化需要使用move group控制的夹爪的group
        self.gripper = moveit_commander.MoveGroupCommander("interbotix_gripper")

        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()

        # 设置目标位置所使用的参考坐标系
        self.reference_frame = 'world'
        self.arm.set_pose_reference_frame(self.reference_frame)
                
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.001)
       
        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)

        # 控制机械臂先回到初始化位置
        self.arm.set_named_target('Home')
        self.arm.go()
        rospy.sleep(1)

        # 设置夹爪运动的允许误差值
        self.gripper.set_goal_joint_tolerance(0.001)

        # 设置允许的最大速度和加速度
        self.gripper.set_max_acceleration_scaling_factor(0.5)
        self.gripper.set_max_velocity_scaling_factor(0.5)
        
        # 控制夹爪先回到初始化位置
        self.moveGripper("Home")

        # 设置每次运动规划的时间限制：10s
        self.arm.set_planning_time(10)
        
        ##################
        ##### Grasp  #####
        ##################

        # Go to preparation position
        pos = [0.24, 0.06, 0.25]
        orient = [0, pi/2, 0]
        self.moveArm(pos, orient)

        # Open the gripper
        # 控制夹爪打开
        self.moveGripper("Open")

        # Grasp
        pos[2] -= 0.1
        self.moveArm(pos, orient)
        self.moveGripper("Closed")

        # Go back to the preparation position
        pos[2] += 0.1
        self.moveArm(pos, orient)

        ####################
        ##### Release  #####
        ####################

        # Releasing position
        pos = [0.4, 0.15, 0.15]
        self.moveArm(pos, orient)

        # 控制夹爪打开
        self.moveGripper("Open")

        pos[2] += 0.1
        self.moveArm(pos, orient)

        #####################################
        ##### Exit after task finished  #####
        #####################################

        self.moveGripper("Home")

        # 控制机械臂回到初始化位置
        self.arm.set_named_target('Sleep')
        self.arm.go()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def moveArm(self, xyz, rpy):
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = xyz[0]
        target_pose.pose.position.y = xyz[1]
        target_pose.pose.position.z = xyz[2]
        # target_pose.pose.orientation.w = 1.0
        q_angle = quaternion_from_euler(rpy[0], rpy[1], rpy[2], axes='sxyz')
        q = Quaternion(*q_angle)  
        # print(q)
        target_pose.pose.orientation = q
        
        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        
        # 规划运动路径
        plan_success, traj, planning_time, error_code = self.arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        self.arm.execute(traj)
        rospy.sleep(1)

    def moveGripper(self, target_type):
        if target_type != "Home" and target_type != "Closed" and target_type != "Open":
            rospy.logerr("Invalid input of the target type: "+target_type)
        else:
            self.gripper.set_named_target(target_type)
            self.gripper.go()
            rospy.sleep(1)

if __name__ == "__main__":
    PickAndPlace()

    
    
