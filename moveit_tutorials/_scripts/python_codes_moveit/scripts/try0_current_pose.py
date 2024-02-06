#!/usr/bin/env python

import rospy
import sys
import moveit_commander

rospy.init_node('get_robot_pose')

# Initialize MoveIt Commander
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("panda_arm")

# Get the current pose of the end-effector (TCP)
current_pose = group.get_current_pose().pose

# Print the current pose
print("Current Pose:")
print("Position (x, y, z):", current_pose.position.x, current_pose.position.y, current_pose.position.z)
print("Orientation (qx, qy, qz, qw):",
      current_pose.orientation.x, current_pose.orientation.y,
      current_pose.orientation.z, current_pose.orientation.w)

# Clean up
moveit_commander.roscpp_shutdown()
