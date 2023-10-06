#!/usr/bin/env python3
import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import random

def main():
    rospy.init_node('gazebo_set_cube_pose')

    cube_pos = []
    while len(cube_pos) <= 3:
        new_pos = (random.uniform(0.14, 0.3), random.uniform(-0.14, 0.14))
        ok = True
        for pos in cube_pos:
            if ((pos[0] - new_pos[0])**2 + (pos[1] - new_pos[1])**2)**.5 < 0.06:
                ok = False
                break
        if ok:
            cube_pos.append(new_pos)

    state_msgs = []
    for pos, name in zip(cube_pos, ["red_cube", "green_cube", "blue_cube"]):
        state_msg = ModelState()
        state_msg.model_name = name
        state_msg.pose.position.x = pos[0]
        state_msg.pose.position.y = pos[1]
        state_msg.pose.position.z = 0.01
        state_msgs.append(state_msg)

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        for msg in state_msgs:
            set_state( msg )

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

