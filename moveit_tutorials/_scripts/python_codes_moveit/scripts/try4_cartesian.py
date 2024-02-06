#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

robot = moveit_commander.RobotCommander()

scale=1
value = 0.1

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('cartesian_path', anonymous=True)

group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

## Cartesian Paths
waypoints = []

wpose = move_group.get_current_pose().pose

#wpose.position.x += scale * 0.1  # forward/backwards in (x)
#wpose.position.y += scale * 0.1  # left/right (y)
wpose.position.x += scale * value  # up/down (z)
waypoints.append(copy.deepcopy(wpose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
(plan, fraction) = move_group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold

# Display trajectory
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

## any AttachedCollisionObjects and add our plan to the trajectory.
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)

# Publish the plan
display_trajectory_publisher.publish(display_trajectory);

print "============ Press `Enter` to execute a saved path ..."
raw_input()

## Use execute the plan
move_group.execute(plan, wait=True)

