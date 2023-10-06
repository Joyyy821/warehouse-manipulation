#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander

class MoveItGripperDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_gripper_demo', anonymous=True)
 
        # 初始化需要使用move group控制的夹爪的group
        gripper = moveit_commander.MoveGroupCommander("interbotix_gripper")
        
        # 设置夹爪运动的允许误差值
        gripper.set_goal_joint_tolerance(0.001)

        # 设置允许的最大速度和加速度
        gripper.set_max_acceleration_scaling_factor(0.5)
        gripper.set_max_velocity_scaling_factor(0.5)
        
        # 控制夹爪先回到初始化位置
        gripper.set_named_target('Home')
        gripper.go()
        rospy.sleep(1)

        # 控制夹爪打开
        gripper.set_named_target('Open')
        gripper.go()
        rospy.sleep(1)

        # 控制夹爪闭合
        gripper.set_named_target('Closed')
        gripper.go()
        rospy.sleep(1)
                         
        # 设置夹爪的目标位置，使用两个关节的位置数据进行描述（单位：弧度）
        joint_positions = [0.017, -0.017]
        gripper.set_joint_value_target(joint_positions)
                 
        # 控制夹爪完成运动
        gripper.go()
        rospy.sleep(1)

        # 控制夹爪先回到初始化位置
        gripper.set_named_target('Home')
        gripper.go()
        rospy.sleep(1)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItGripperDemo()
    except rospy.ROSInterruptException:
        pass
