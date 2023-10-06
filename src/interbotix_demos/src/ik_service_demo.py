#!/usr/bin/env python3

import rospy
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetPositionIKRequest
from moveit_msgs.srv import GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
import random
import numpy as np

class GetIK(object):
    def __init__(self, group, ik_timeout=0.005, avoid_collisions=False):
        """
        :param str group: MoveIt! group name
        :param float ik_timeout: default timeout for IK
        :param int ik_attempts: default number of attempts
        :param bool avoid_collisions: if to ask for IKs that take
        into account collisions
        """
        rospy.loginfo("Initalizing GetIK...")
        self.group_name = group
        self.ik_timeout = ik_timeout
        self.avoid_collisions = avoid_collisions
        rospy.loginfo("Computing IKs for group: " + self.group_name)
        rospy.loginfo("With IK timeout: " + str(self.ik_timeout))
        rospy.loginfo("Setting avoid collisions to: " + str(self.avoid_collisions))
        self.ik_srv = rospy.ServiceProxy('/wx250s/compute_ik', GetPositionIK)
        rospy.loginfo("Waiting for compute_ik service...")
        self.ik_srv.wait_for_service()
        rospy.loginfo("Connected!")

    def get_ik(self, pose_stamped):
        """
        Do an IK call to pose_stamped pose.
        :param geometry_msgs/PoseStamped pose_stamped: The 3D pose
            (with header.frame_id)
            to which compute the IK.
        """
        req = GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout = rospy.Duration(self.ik_timeout)
        req.ik_request.avoid_collisions = self.avoid_collisions
        req.ik_request.robot_state.joint_state.name = ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]
        req.ik_request.robot_state.joint_state.position = [0., 0., 0., 0., 0., 0.]

        try:
            resp = self.ik_srv.call(req)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service exception: " + str(e))
            resp = GetPositionIKResponse()
            resp.error_code.val = 99999  # Failure
            return resp

if __name__ == '__main__':
    # Node initialization
    rospy.init_node('ik_service_demo', anonymous=True)

    # Create a solver
    ik_solver = GetIK("interbotix_arm")
    
    # Initialize a stamped pose object to represent the target
    target = PoseStamped()
    target.header.stamp = rospy.Time.now()

    # Range of the goal position
    x_range = [-0.15, 0.15]
    y_range = [-0.15, 0.15]
    z_range = [0, 0.15]
    # Orientation is fixed
    target.pose.orientation.x = 0.
    target.pose.orientation.y = 0.
    target.pose.orientation.z = 0.
    target.pose.orientation.w = 1.
    # Total number of trials
    N = 2000
    # Count for the number of success trials
    cnt_success = 0
    # Store the time cost of each trial
    t_lst = []
    i = 0
    while i < N:
        # Random select a target in the given range.
        target.pose.position.x = random.uniform(x_range[0], x_range[1])
        target.pose.position.y = random.uniform(y_range[0], y_range[1])
        target.pose.position.z = random.uniform(z_range[0], z_range[1])
    # target.pose.position.x = 0.24
    # target.pose.position.y = 0.06
    # target.pose.position.z =  0.15

        # Record the start time
        t_start = rospy.Time.now()
        # Solve IK
        res = ik_solver.get_ik(target)
        # Success
        if res.error_code.val == 1:
            cnt_success += 1
            # print("Success")
        # else:
            # if res.error_code.val == -31:
            #     # Invalid position
            #     continue
            # else:
            # print("Failed at trial No. ", i)
            # print("Error code: ", res.error_code.val)

        # Record the end time
        t_end = rospy.Time.now()
        t_lst.append((t_end-t_start).to_sec())
        # print("Time cost:", (rospy.Time.now()-t_start).to_sec()/1000., "ms")
        i += 1

    t_lst = np.array(t_lst)
    # Calculate the average time
    t_avg = np.mean(t_lst)
    # Display the result
    print("Success trials: ", cnt_success, " out of ", N, " attempts.")
    print("Success rate: ", cnt_success/N)
    print("Average calculation time: ", t_avg)
