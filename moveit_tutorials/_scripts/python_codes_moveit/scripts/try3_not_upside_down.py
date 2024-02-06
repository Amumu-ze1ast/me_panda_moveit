#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

# Initialize ROS node
rospy.init_node('moveit_quaternion_example', anonymous=True)

# Initialize MoveIt Commander
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
group_name = "panda_arm"  # Replace with your arm's planning group name
group = moveit_commander.MoveGroupCommander(group_name)

# Define the target position (modify as needed)
target_position = [0.2, 0.2, 0.65]  # Replace with your desired position [x, y, z]

# Create a quaternion to represent the orientation (using Euler angles)
# Example: Roll, Pitch, Yaw angles in radians

deg_roll = 180
deg_pitch = 0
deg_yaw = 315

deg2rad = 3.14/180

roll = deg_roll * deg2rad
pitch = deg_pitch * deg2rad
yaw = deg_yaw * deg2rad

quaternion = quaternion_from_euler(roll, pitch, yaw)

# Set the target pose for the end effector
target_pose = geometry_msgs.msg.Pose()
target_pose.position.x = target_position[0]
target_pose.position.y = target_position[1]
target_pose.position.z = target_position[2]
target_pose.orientation.x = quaternion[0]
target_pose.orientation.y = quaternion[1]
target_pose.orientation.z = quaternion[2]
target_pose.orientation.w = quaternion[3]

# Set the target pose for the end effector
group.set_pose_target(target_pose)

# Plan a trajectory to the target pose
plan = group.plan()

# Execute the planned trajectory
group.execute(plan, wait=True)
