#!/usr/bin/env python

import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg

import sys

# Initialize ROS node
rospy.init_node('moveit_straight_line_movement', anonymous=True)

# Initialize MoveIt Commander
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
group_name = "panda_arm"  # Replace with your arm's planning group name
group = moveit_commander.MoveGroupCommander(group_name)

wpose = group.get_current_pose().pose

x_old = wpose.position.x
y_old = wpose.position.y
z_old = wpose.position.z

x_new = x_old 
y_new = y_old + 0.2
z_new = z_old 


# Set the start and end positions (modify as needed)
start_position = [x_old, y_old, z_old]  # Replace with your start position [x, y, z]
end_position = [x_new, y_new, z_new]   # Replace with your end position [x, y, z]

# Define the number of intermediate waypoints
num_waypoints = 10

# Compute intermediate waypoints between start and end positions
waypoints = []
for i in range(num_waypoints + 1):
    x = start_position[0] + (end_position[0] - start_position[0]) * (i / float(num_waypoints))
    y = start_position[1] + (end_position[1] - start_position[1]) * (i / float(num_waypoints))
    z = start_position[2] + (end_position[2] - start_position[2]) * (i / float(num_waypoints))

    pose = geometry_msgs.msg.Pose()
    pose.orientation = group.get_current_pose().pose.orientation  # Maintain the same orientation
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    waypoints.append(pose)

# Plan a trajectory through the waypoints
(plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)

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

# Execute the planned trajectory
group.execute(plan, wait=True)
