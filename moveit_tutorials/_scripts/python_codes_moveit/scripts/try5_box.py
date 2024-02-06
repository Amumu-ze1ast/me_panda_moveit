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

scene = moveit_commander.PlanningSceneInterface()

box_name = "box"
scene = scene

def wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4):

    box_name = box_name
    scene = scene
    
    start = rospy.get_time()
    seconds = rospy.get_time()

    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in attached objects
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0

        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        is_known = box_name in scene.get_known_object_names()

        # Test if we are in the expected state
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False


def add_box(timeout=4):

    box_name = box_name
    scene = scene

    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "panda_leftfinger"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = 0.07 # slightly above the end effector
    
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    box_name=box_name
    return wait_for_state_update(box_is_known=True, timeout=timeout)

print "============ Press `Enter` to add a box to the planning scene ..."
raw_input()
add_box()