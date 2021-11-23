#!/usr/bin/env python
import os
import sys
import rospy

sys.path.append(os.path.join('/home/rllab/catkin_ws/src/'))
#from moveit_tutorials.srv import *

from ur_msgs.srv import JointTrajectory

import moveit_commander
from moveit_msgs.srv import *
from moveit_commander import RobotCommander, MoveGroupCommander, roscpp_initialize

#group_name = "both_arms" #"left_arm", "both_arms"
#group = moveit_commander.MoveGroupCommander(group_name)

def send_joint_trajectory(req):
    if len(req.arm_pose) != 0:
        print(req.arm_pose)
        arm_pose = dict(zip(req.arm_joint_name, req.arm_pose))
        robot_arm.set_joint_value_target(arm_pose)
    elif len(req.arm_pose) == 0:
        print(req.endeffector_pose)
        robot_arm.set_pose_target(req.endeffector_pose)

    plan_results = robot_arm.plan()
    #robot_arm.go(wait=True)
    return plan_results.joint_trajectory

def move_code():
          
    """
    robot_arm.set_named_target("home")  #go to goal state.
    robot_arm.go(wait=True)
    print("====== move plan go to home 1 ======")        
    rospy.sleep(1)  
    """      

    """robot_arm.set_named_target("up")  #go to goal state.
    robot_arm.go(wait=True)
    print("====== move plan go to up ======")        
    rospy.sleep(1)"""

    #robot_state = robot_arm.get_current_pose()
    #robot_angle = robot_arm.get_current_joint_values()

    TARGET_JOINT_QPOS = [-4.52900676522404e-05, -1.5707255042219068, -9.767265589907765e-05, -1.5706097928549163, 4.973337803967298e-05, -7.33426604885608e-05]
    robot_arm.set_joint_value_target(TARGET_JOINT_QPOS)
    robot_arm.go(wait=True)
    rospy.sleep(1)  

    TARGET_JOINT_QPOS = [-4.52900676522404e-05, -1.5707255042219068, -1.57, -1.5706097928549163, 4.973337803967298e-05, -7.33426604885608e-05]
    robot_arm.set_joint_value_target(TARGET_JOINT_QPOS)
    robot_arm.go(wait=True)
    rospy.sleep(1)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 0.0
    pose_goal.orientation.y = 0.7071068
    pose_goal.orientation.z = 0.0
    pose_goal.orientation.w = 0.7071068
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.0
    pose_goal.position.z = 0.4
    robot_arm.set_pose_target(pose_goal)
    robot_arm.go(wait=True)
    print("====== move plan go to random pose ======")        
    rospy.sleep(1)


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5_move', anonymous=True)

    GROUP_NAME_ARM = "ur5_robot"
    robot_cmd = RobotCommander()
    robot_arm = MoveGroupCommander(GROUP_NAME_ARM)

    rospy.Service('send_joint_trajectory', JointTrajectory, send_joint_trajectory)
    #move_code()
    rospy.spin()




    




