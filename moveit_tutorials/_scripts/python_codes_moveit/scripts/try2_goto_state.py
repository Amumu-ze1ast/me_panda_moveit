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

def go_to_joint_state():

  ## First initialize `moveit_commander`_ and a `rospy`_ node:
  moveit_commander.roscpp_initialize(sys.argv)

  ## This interface can be used to plan and execute motions:
  group_name = "panda_arm"
  move_group = moveit_commander.MoveGroupCommander(group_name)

  ## Planning to a Joint Goal

  joint_goal = move_group.get_current_joint_values()

  joint_goal[7] = 0.035
  # joint_goal[1] = -pi/4
  # joint_goal[2] = 0
  # joint_goal[3] = -pi/2
  # joint_goal[4] = 0
  # joint_goal[5] = pi/3
  # joint_goal[6] = 0



  # The go command can be called with joint values, poses, or without any parameters
  move_group.go(joint_goal, wait=True)

  # Calling ``stop()`` ensures that there is no residual movement
  move_group.stop()


def main():
  try:

    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    raw_input()
    go_to_joint_state()


  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()


