#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#############################################################
# File: moveit_cartesian_demo.py
# Implementation of Experiment 3 Step 3: pick and place task
# by using cartesian path planning.
# Note that this demo cannot work with Gazebo simulation (
# where Gazebo may report:
#   Solution found but controller failed during execution.)
# One can use the fake controller provided by MoveIt! RViz 
# interface to replace the Gazebo simulation.
#############################################################

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped, Quaternion
from copy import deepcopy
from math import pi
from tf.transformations import quaternion_from_euler 

class MoveItCartesianDemo:
    def __init__(self):
        ##########################
        ##### Initialization #####
        ##########################
        
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_cartesian_demo', anonymous=True)
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = MoveGroupCommander('interbotix_arm')

        # 初始化需要使用move group控制的夹爪的group
        self.gripper = moveit_commander.MoveGroupCommander("interbotix_gripper")

        # 设置目标位置所使用的参考坐标系
        self.reference_frame = 'world'
        self.arm.set_pose_reference_frame(self.reference_frame)
        
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        self.arm.set_pose_reference_frame('world')
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.001)
        
        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)
        
        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()

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
                                               
        ##################
        ##### Grasp  #####
        ##################

        # Preparation pose
        pos = [0.24, 0.06, 0.25]
        orient = [0, pi/2, 0]
        self.moveArm(pos, orient)

        self.moveGripper("Open")

        # 获取当前位姿数据最为机械臂运动的起始位姿
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose

        print( start_pose )

        # 初始化路点列表
        waypoints = []
                
        # 将初始位姿加入路点列表
        waypoints.append(start_pose)
            
        # 设置路点数据，并加入路点列表
        wpose = deepcopy(start_pose)
        wpose.position.z -= 0.05
        waypoints.append(deepcopy(wpose))

        wpose.position.z -= 0.05
        waypoints.append(deepcopy(wpose))

        # Plan the path by cartesian path planning
        self.cartesianPathPlan(waypoints)
        # Close the gripper to grasp object
        self.moveGripper("Closed")

        ####################
        ##### Release  #####
        ####################

        # 获取当前位姿数据最为机械臂运动的起始位姿
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
        # 初始化路点列表
        waypoints = []
                
        # 将初始位姿加入路点列表
        waypoints.append(start_pose)

        # 设置路点数据，并加入路点列表
        wpose = deepcopy(start_pose)
        wpose.position.z += 0.05
        waypoints.append(deepcopy(wpose))

        wpose.position.z += 0.05
        waypoints.append(deepcopy(wpose))

        wpose.position.x = 0.24
        wpose.position.y = 0.06
        waypoints.append(deepcopy(wpose))

        wpose = deepcopy(start_pose)
        wpose.position.z -= 0.05
        waypoints.append(deepcopy(wpose))

        wpose.position.z -= 0.05
        waypoints.append(deepcopy(wpose))
        # Plan the path by cartesian path planning
        self.cartesianPathPlan(waypoints)
        # Open the gripper to release object
        self.moveGripper("Open")

        #####################################
        ##### Exit after task finished  #####
        #####################################

        # 控制机械臂先回到初始化位置
        self.arm.set_named_target('Sleep')
        self.arm.go()
        rospy.sleep(1)

        # Close the gripper to the Home position
        self.moveGripper("Home")
        
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

    def cartesianPathPlan(self, waypoints):
        fraction = 0.0   #路径规划覆盖率
        maxtries = 100   #最大尝试规划次数
        attempts = 0     #已经尝试规划次数
        
        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()
 
        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.01,        # eef_step，终端步进值
                                    0.0,         # jump_threshold，跳跃阈值
                                    True)        # avoid_collisions，避障规划
            
            # 尝试次数累加
            attempts += 1
            
            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                     
        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            self.arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        rospy.sleep(1)

if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass
