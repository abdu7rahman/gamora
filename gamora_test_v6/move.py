#!/usr/bin/env python
# coding: utf-8
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

rospy.init_node('gamora', anonymous=False)

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
arm_group=moveit_commander.MoveGroupCommander("arm")
grasp_pose=Pose()

grasp_pose.position.x = -0.14738496092831854
grasp_pose.position.y = 0.014994630138540962
grasp_pose.position.z = 0.2562754992270229
grasp_pose.orientation.x = -0.08643387766022224
grasp_pose.orientation.y = -0.7033921930997056
grasp_pose.orientation.z = 0.7035986137006519
grasp_pose.orientation.w = 0.052130588693444675
arm_group.set_pose_target(grasp_pose)    
plan1=arm_group.go()

rospy.spin()
