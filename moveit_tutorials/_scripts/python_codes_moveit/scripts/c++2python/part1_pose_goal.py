#!/usr/bin/env python

import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_visual_tools import MoveItVisualTools
from geometry_msgs.msg import Pose

rospy.init_node('move_group_interface_tutorial')
robot = RobotCommander()
scene = PlanningSceneInterface()

PLANNING_GROUP = "panda_arm"
move_group = MoveGroupCommander(PLANNING_GROUP)
joint_model_group = robot.get_joint_model_group(PLANNING_GROUP)

# Visualization setup
visual_tools = MoveItVisualTools("panda_link0")
visual_tools.deleteAllMarkers()
visual_tools.loadRemoteControl()

text_pose = visual_tools.convertPoseStampedToMsg(visual_tools.getPoseStampedMsg("panda_link0", [0, 0, 1.75, 0, 0, 0]))  # Adjust the pose accordingly
text_pose.text = "MoveGroupInterface Demo"
text_pose.color = "white"
text_pose.text_scale = 3
visual_tools.publishText(text_pose)
visual_tools.trigger()

rospy.loginfo("Planning frame: %s", move_group.get_planning_frame())
rospy.loginfo("End effector link: %s", move_group.get_end_effector_link())
rospy.loginfo("Available Planning Groups: %s", ", ".join(move_group.get_joint_model_group_names()))

# Start the demo
print("Please press 'next' in the RVizVisualToolsGui window to start the demo")

# Planning to a Pose goal
target_pose1 = Pose()
target_pose1.orientation.w = 1.0
target_pose1.position.x = 0.28
target_pose1.position.y = -0.2
target_pose1.position.z = 0.5
move_group.set_pose_target(target_pose1)

my_plan = move_group.plan()
success = my_plan[0]

rospy.loginfo("Visualizing plan 1 (pose goal) %s" % ("SUCCESS" if success else "FAILED"))

# Visualizing plans
rospy.loginfo("Visualizing plan 1 as trajectory line")
visual_tools.publishAxisLabeled(target_pose1, "pose1")
text_pose = move_group.get_current_pose().pose
visual_tools.publishText(text_pose, "Pose Goal", "white", "xlarge")
visual_tools.publishTrajectoryLine(my_plan, joint_model_group)
visual_tools.trigger()

print("Please press 'next' in the RVizVisualToolsGui window to continue the demo")

rospy.signal_shutdown("Demo finished")
