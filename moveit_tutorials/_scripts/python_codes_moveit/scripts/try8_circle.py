#!/usr/bin/env python


import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import math

def move_arm_in_circle():
    # Initialize ROS node
    rospy.init_node('moveit_arm_circle_demo', anonymous=True)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("panda_arm")  # Replace with your arm's planning group name

    # Define circle parameters
    center_x = 0.5
    center_y = 0.5
    center_z = 0.5
    radius = 0.1  # Radius of the circle
    num_points = 50  # Number of points on the circle

    # Calculate waypoints forming a circle
    waypoints = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        z = center_z
        waypoints.append([x, y, z])

    # Move the arm along the circular trajectory
    for waypoint in waypoints:
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = waypoint[0]
        target_pose.position.y = waypoint[1]
        target_pose.position.z = waypoint[2]
        target_pose.orientation.w = 1.0

        group.set_pose_target(target_pose)
        group.go(wait=True)

if __name__ == '__main__':
    try:
        move_arm_in_circle()
    except rospy.ROSInterruptException:
        pass
