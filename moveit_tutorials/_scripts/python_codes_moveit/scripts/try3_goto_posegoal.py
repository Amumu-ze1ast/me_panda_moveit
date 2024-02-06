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

def go_to_pose_goal():

  ## First initialize `moveit_commander`_ and a `rospy`_ node:
  moveit_commander.roscpp_initialize(sys.argv)

  ## This interface can be used to plan and execute motions:
  group_name = "panda_arm"
  move_group = moveit_commander.MoveGroupCommander(group_name)

  ## We can plan a motion for this group to a desired pose for the end-effector:
  pose_goal = geometry_msgs.msg.Pose()
  pose_goal.orientation.w = 1.0
  pose_goal.position.x = 0.3
  pose_goal.position.y = 0.0
  pose_goal.position.z = 0.75

  move_group.set_pose_target(pose_goal)

  ## Now, we call the planner to compute the plan and execute it.
  move_group.go(wait=True)

  # Calling `stop()` ensures that there is no residual movement
  move_group.stop()

  # It is always good to clear your targets after planning with poses.
  # Note: there is no equivalent function for clear_joint_value_targets()
  move_group.clear_pose_targets()


def main():
  try:

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    go_to_pose_goal()


  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()


