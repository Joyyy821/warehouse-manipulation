#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#########################################################################
# File: sorting.py
# Implementation of Project 2: Warehouse Robot.
# Sort the cubes to corresponding trash bins by color detection.
#########################################################################

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from std_msgs.msg import Float64
from object_color_detector.srv import *
from math import pi
from copy import deepcopy
from tf.transformations import quaternion_from_euler 
import numpy as np

class Sorting:
    def __init__(self):
        ##########################
        ##### Initialization #####
        ##########################
        
        self.t_sleep = 0.5 # second
        self.N_col = 3   # 3 colors to detect
        self.N_cube = 0  # Detected cubes
        self.max_wait_time = 10 # seconds
        self.is_shutdown = False

        self.reg_x = rospy.get_param("/image/reg_x")
        self.reg_y = rospy.get_param("/image/reg_y")
        # print(self.reg_x[0], self.reg_x[1])
        # print(self.reg_y)

        # Set releasing pose
        self.setReleasingPose()
        # print("Releasing pose: ")
        # print(self.releasing_pose)

        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_sorting_demo')

        # 初始化场景对象
        self.scene = PlanningSceneInterface()
        # print("tring...")
        rospy.sleep(1)

        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = MoveGroupCommander("interbotix_arm")

        # 初始化需要使用move group控制的夹爪的group
        self.gripper = MoveGroupCommander("interbotix_gripper")

        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()

        # 设置目标位置所使用的参考坐标系
        self.reference_frame = 'world'
        self.arm.set_pose_reference_frame(self.reference_frame)
        
        # Set the camera as an attached object         
        self.attachCamera()
                
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.001)
       
        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)

        # # 控制机械臂先回到初始化位置
        # self.arm.set_named_target('Home')
        # self.arm.go()
        # rospy.sleep(self.t_sleep)

        # 设置夹爪运动的允许误差值
        self.gripper.set_goal_joint_tolerance(0.001)

        # 设置允许的最大速度和加速度
        self.gripper.set_max_acceleration_scaling_factor(0.5)
        self.gripper.set_max_velocity_scaling_factor(0.5)

        # set Gripper command publisher
        self.gripper_pub = rospy.Publisher("/wx250s/gripper/command", Float64, queue_size=1)
        
        # # 控制夹爪先回到初始化位置
        self.moveGripper2("Open")
        # self.moveGripper2("Closed")
        # # self.moveGripper2("Home")

        # 设置每次运动规划的时间限制：10s
        self.arm.set_planning_time(10)
        
        # Define the position of the trash bins
        self.setObstacles(option="trashbin")

        # 定义拍照位姿
        self.capture_point = Point(0.15, -0.00, 0.3)
        self.capture_quaternion = Quaternion(0.00, 2**.5/2., 0.00, 2**.5/2.) 

        # self.toRelease(0) # red
        # self.toRelease(1) # green
        # self.toRelease(2) # blue

        rospy.loginfo("Initialization finished.")

    def attachCamera(self):
        self.scene.remove_attached_object(self.end_effector_link, 'camera')

        camera_size = [0.025, 0.035, 0.045]
        
        # 设置tool的位姿
        self.camera = PoseStamped()
        self.camera.header.frame_id = self.end_effector_link
        
        self.camera.pose.position.x = -0.03
        self.camera.pose.position.y = 0
        self.camera.pose.position.z = 0.06
        self.camera.pose.orientation.w = 1
        # q_angle = quaternion_from_euler(0, pi/2, 0, axes='sxyz')
        # q = Quaternion(*q_angle)  
        # # print(q)
        # self.camera.pose.orientation = q
        
        # 将tool附着到机器人的终端
        self.scene.attach_box(self.end_effector_link, 'camera', self.camera, camera_size)
        rospy.sleep(1.5)  
        rospy.loginfo("Camera has been set as an attached object to the end effector.")


    def setObstacles(self, option="cube"):
        # Input: option = "cube" / option = "trashbin"
        # Based on the position of the cubes/trashbins, 
        # set them as environment obstacles.
        if option == "trashbin":
            # Set the trashbins as obstacles
            self.scene.remove_world_object("trashbin")

            bin_size = [0.27, 0.09, 0.12]

            self.trashbin = PoseStamped()
            self.trashbin.header.frame_id = self.reference_frame

            # 设置tool的位姿
            self.trashbin.pose.position.x = 0
            self.trashbin.pose.position.y = 0.15
            self.trashbin.pose.position.z = 0.06
            self.trashbin.pose.orientation.w = 1

            # Set the trashbin as an obstacle in the scene
            self.scene.add_box('trashbin', self.trashbin, bin_size)
            rospy.sleep(1.5)  
            rospy.loginfo("Trashbin has been set as an obstacle in the scene.")
        elif option == "cube":
            # Set the cubes as the obstacles
            cube_size = [0.02, 0.02, 0.02]
            self.clearCubes()
            # self.N_cube = 0
            for i in range(self.N_col):
                c_cubes = self.cubes[i]
                for cube in c_cubes:
                    self.N_cube += 1
                    cube_pose = PoseStamped()
                    cube_pose.header.frame_id = self.reference_frame

                    # Set the pose of cube
                    cube_pose.pose.position = cube
                    cube_pose.pose.orientation.w = 1

                    # Set the trashbin as an obstacle in the scene
                    self.scene.add_box('cube'+str(self.N_cube), cube_pose, cube_size)
            rospy.sleep(1.5)  
            rospy.loginfo("Cube(s) has/have been set as obstacle(s) in the scene.")

    def clearCubes(self):
        # Remove the cubes which have been set as obstacles before
        for i in range(1, self.N_cube+1):
            self.scene.remove_world_object('cube'+str(i))
        self.N_cube = 0

    def setReleasingPose(self):
        # Save a list of fixed releasing poses (in the order red, green, blue)
        self.releasing_pose = [Pose(), Pose(), Pose()]
        bin_x = [0.0, 0.09, -0.09]
        bin_y = 0.15
        bin_z = 0.23
        for i in range(self.N_col):
            # position
            # print("i: ", i)
            # print("bin_x: ", bin_x[i])
            self.releasing_pose[i].position.x = bin_x[i]
            self.releasing_pose[i].position.y = bin_y
            self.releasing_pose[i].position.z = bin_z
            # orientation
            rpy = [0, pi/2, 0]
            q_angle = quaternion_from_euler(rpy[0], rpy[1], rpy[2], axes='sxyz')
            q = Quaternion(*q_angle)  
            self.releasing_pose[i].orientation = q


    def sort(self):
        # Main function for the warehouse robot sorting task.
        while True:
            self.toCapture()
            print("Detected cubes: ", self.cubes)
            # self.shutdown()
            if self.is_shutdown:
                return
            for i in range(self.N_col):
                # Capture and grasp
                N0 = len(self.cubes[i]) 
                if N0 == 0:
                    continue
                N1 = N0
                while N1 > 0:
                    self.toGrasp(i)
                    self.toCapture(to_wait=False)
                    print("Detected cubes: ", self.cubes)
                    # self.shutdown()
                    # return
                    N1 = len(self.cubes[i])
                    if N0 - N1 == 1:
                        rospy.loginfo("Grasping successed.")
                        break
                        # self.shutdown()
                        # return
            
                # Release
                self.toRelease(i)


    def toCapture(self, to_wait=True):
        # Input: to_wait = True (wait until there is/are cube(s), 
        # or the maximum waiting time is arrived -> shutdown).
        # Move the manipulator to the capture pose and detect the cubes
        # Save the color and position in the world frame of the cube(s)
        # as class attributes, and set the cubes as obstacles.
        self.cubes = [[], [], []]
        # Move to the caputure pose.
        self.moveArm2("Capture")

        # 目标识别
        rospy.loginfo("Attempt to recognize objects ...")
        try:
            # 请求服务
            rospy.wait_for_service('/object_detect')

            stamp = rospy.Time.now()
            t_s = stamp.secs
            t_e = t_s
            while True:
                detect_object_service = rospy.ServiceProxy('/object_detect', DetectObjectSrv)
                response = detect_object_service(DetectObjectSrvRequest.ALL_OBJECT) 
                ObjList = []
                ObjList.append(response.redObjList)
                ObjList.append(response.greenObjList)
                ObjList.append(response.blueObjList)
                for i in range(self.N_col):
                    c_obj_lst = ObjList[i]
                    for c_obj in c_obj_lst:
                        xc = c_obj.position.x
                        yc = c_obj.position.y
                        x = self.reg_x[0] * yc + self.reg_x[1]
                        y = self.reg_y[0] * xc + self.reg_y[1]
                        self.cubes[i].append(Point(x, y, 0.01))
                rslt = self.isDetectionEmpty()
                # print(rslt)
                if to_wait and rslt:
                    rospy.sleep(self.t_sleep)
                    stamp = rospy.Time.now()
                    t_e = stamp.secs
                else:
                    break
                if not to_wait:
                    break
                if t_e - t_s > self.max_wait_time:
                    rospy.loginfo("Exceed the maximum waiting time for object detection.")
                    self.shutdown()
                    return
            rospy.loginfo("Detect object over" )
            
            # Set the detected cubes as obstacles
            # self.setObstacles()

        except rospy.ROSException:
            rospy.loginfo("Timeout waiting for image data.")
    

    def isDetectionEmpty(self):
        rslt = True
        # print(self.cubes)
        for i in range(self.N_col):
            # print(self.cubes[i])
            if len(self.cubes[i]) > 0:
                # print("success!")
                rslt = False
                break

        return rslt


    def toGrasp(self, c):
        # Input: color (0 = red, 1 = green, 2 = blue)
        # Grasp the object correspondingly:
        # 1) Move to the preparation position
        # 2) Open the gripper
        # 3) Move downwards to grasp
        # 4) Close the gripper
        # 5) Move back to the preparation position

        ##################
        ##### Grasp  #####
        ##################

        # Go to preparation position
        cube_pos = self.cubes[c]
        if len(cube_pos) == 0:
            print("No avaliable cube with color code: ", c)
        else:
            cube_pos = cube_pos[0]
        pos = [cube_pos.x, cube_pos.y, cube_pos.z+0.1]
        orient = [0, pi/2, 0]

        self.moveArm(pos, orient)

        # Open the gripper
        # 控制夹爪打开
        self.moveGripper2("Open")

        # 获取当前位姿数据最为机械臂运动的起始位姿
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
        # 初始化路点列表
        waypoints = []
                
        # 将初始位姿加入路点列表
        waypoints.append(start_pose)

        # intermediate pose
        # 设置路点数据，并加入路点列表
        wpose = deepcopy(start_pose)
        wpose.position.z -= 0.01
        waypoints.append(deepcopy(wpose))
        wpose = deepcopy(start_pose)
        wpose.position.z -= 0.018
        waypoints.append(deepcopy(wpose))

        # Grasp
        # Plan the path by cartesian path planning
        self.cartesianPathPlan(waypoints)
        # pos[2] -= 0.02
        # self.moveArm(pos, orient)
        self.moveGripper2("Closed")

        # # Go back to the preparation position
        # pos[2] += 0.03
        # self.moveArm(pos, orient)
    

    def toRelease(self, c):
        # Input color of the cube (this method 
        # uses the same code as the method toGrasp)
        # 1) Move to a preset position
        # 2) Generate a cartesian path (straight line) from the preset position 
        #    to the selcted trash bin based on the input color.
        # 3) Move to the trash bin
        # 4) Open the gripper
        # 5) Move back to the preparation position

        ####################
        ##### Release  #####
        ####################

        # self.moveArm2("Capture")

        # 获取当前位姿数据最为机械臂运动的起始位姿
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose

        # Set the end point based on the color
        # print("Color code: ", c)
        end_pose = self.releasing_pose[c]
        print("end_pose: ", end_pose, "\n")

        delta_pos = np.array([end_pose.position.x - start_pose.position.x, 
                            end_pose.position.y - start_pose.position.y,
                            end_pose.position.z - start_pose.position.z])

        N_points = 11
        # 初始化路点列表
        waypoints = []
                
        # 将初始位姿加入路点列表
        waypoints.append(start_pose)
        
        for i in range(1, N_points+1):
            # 设置路点数据，并加入路点列表
            wpose = deepcopy(start_pose)
            wpose.position.x += delta_pos[0]*(i/N_points)
            wpose.position.y += delta_pos[1]*(i/N_points)
            wpose.position.z += delta_pos[2]*(i/N_points)
            waypoints.append(deepcopy(wpose))

        # # Add the ending pose into the waypoints list
        # waypoints.append(end_pose)

        # print(waypoints)

        # Plan the path by cartesian path planning
        self.cartesianPathPlan(waypoints)

        # 控制夹爪打开
        self.moveGripper2("Open")

        # # Move the gripper to the home position
        # self.moveGripper2("Home")

        # Reverse the waypoints to go back
        waypoints.reverse()

        # print(waypoints)

        # Plan the path by cartesian path planning
        self.cartesianPathPlan(waypoints)


    def shutdown(self):
        #####################################
        ##### Exit after task finished  #####
        #####################################
        rospy.loginfo("Shutting down ...")

        self.moveGripper2("Home")

        # 控制机械臂回到初始化位置
        self.arm.set_named_target('Sleep')
        self.arm.go()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

        self.is_shutdown = True


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


    def moveArm(self, xyz, rpy):
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = xyz[0]
        target_pose.pose.position.y = xyz[1]
        target_pose.pose.position.z = xyz[2]
        q_angle = quaternion_from_euler(rpy[0], rpy[1], rpy[2], axes='sxyz')
        q = Quaternion(*q_angle)  
        target_pose.pose.orientation = q
        
        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        
        # 规划运动路径
        plan_success, traj, planning_time, error_code = self.arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        self.arm.execute(traj)
        rospy.sleep(self.t_sleep)


    def moveArm2(self, target_type):
        if target_type != "Home" and \
            target_type != "Sleep" and \
            target_type != "Upright" and \
            target_type != "Capture":
            rospy.logerr("Invalid input of the target type: "+target_type)
        elif target_type == "Capture":
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.reference_frame
            target_pose.header.stamp = rospy.Time.now()  
            target_pose.pose.position = self.capture_point
            target_pose.pose.orientation = self.capture_quaternion

            # 设置机器臂当前的状态作为运动初始状态
            self.arm.set_start_state_to_current_state()
            
            # 设置机械臂终端运动的目标位姿
            self.arm.set_pose_target(target_pose, self.end_effector_link)
            
            # 规划运动路径
            plan_success, traj, planning_time, error_code = self.arm.plan()
            
            # 按照规划的运动路径控制机械臂运动
            self.arm.execute(traj)
            rospy.sleep(self.t_sleep)
        else:
            self.arm.set_named_target(target_type)
            self.arm.go()
            rospy.sleep(self.t_sleep)


    def moveGripper(self, pos, set_pos=True):
        # set_pos = False -> set the motor position
        # set_pos = True -> set the gripper position\
        if not set_pos:
            if pos < -0.02 or pos > -0.01:
                rospy.logerr("Invalid motor command: "+str(pos))
                return
            self.gripper_pub.publish(pos)
            print('Published gripper motor pos: ', pos)
        rospy.sleep(self.t_sleep)


    def moveGripper2(self, target_type):
        if target_type != "Home" and target_type != "Closed" and target_type != "Open":
            rospy.logerr("Invalid input of the target type: "+target_type)
        # elif target_type == "Closed":
        #     self.moveGripper(-0.015, False)
        # elif target_type == "Open":
        #     self.moveGripper(-0.02, False)
        else:
            self.gripper.set_named_target(target_type)
            self.gripper.go()
            rospy.sleep(self.t_sleep)
        rospy.loginfo("Gripper set to: "+target_type)



if __name__ == "__main__":
    # Initialization
    s = Sorting()
    # Execute the sorting program
    s.sort()

