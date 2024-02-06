
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)

{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "panda_arm";   // MoveIt operates on sets of joints called "planning groups" and stores them in an object called the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group" are used interchangeably.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);   // The :move_group_interface:`MoveGroupInterface` class can be easily setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;   // We will use the :planning_scene_interface:`PlanningSceneInterface` class to add and remove collision objects in our "virtual world" scene
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);   // Raw pointers are frequently used to refer to the planning group for improved performance.



  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
  // Visualization
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");   // The package MoveItVisualTools provides many capabilities for visualizing objects, robots, and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();   // Remote control is an introspection tool that allows users to step through a high level scriptvia buttons and keyboard shortcuts in RViz

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();   // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  visual_tools.trigger();   // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations


  // Getting Basic Information
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str()); // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");   // We can get a list of all the groups in the robot:
  
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  
  // Start the demo
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");



  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
  // Planning to a Pose goal
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

  geometry_msgs::Pose target_pose1;   // We can plan a motion for this group to a desired pose for the end-effector.
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;
  move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;   // Now, we call the planner to compute the plan and visualize it. Note that we are just planning, not asking move_group to actually move the robot.
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");


  // Visualizing plans
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");   // We can also visualize the plan as a line with markers in RViz.
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


  // Moving to a pose goal   /* Uncomment below line when working with a real robot */   // Moving to a pose goal is similar to the step above except we now use the move() function. Note that the pose goal we had set earlier is still active and so the robot will try to move to that goal. We will not use that function in this tutorial since it is a blocking function and requires a controller to be active and report success on execution of a trajectory.
  /* move_group.move(); */



  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
  // Planning to a joint-space goal. 
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //  
  
  // Let's set a joint space goal and move towards it.  This will replace the pose target we set above. To start, we'll create an pointer that references the current robot's state.
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();   // RobotState is the object that contains all the current position/velocity/acceleration data.

  std::vector<double> joint_group_positions;   // Next get the current set of joint values for the group.
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  joint_group_positions[0] = -1.0;  // radians   // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  move_group.setJointValueTarget(joint_group_positions);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");


  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  
  
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
  // Planning with Path Constraints.  
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
  
  // Path constraints can easily be specified for a link on the robot. Let's specify a path constraint and a pose goal for our group. First define the path constraint.
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "panda_link7";
  ocm.header.frame_id = "panda_link0";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  moveit_msgs::Constraints test_constraints;   // Now, set it as the path constraint for the group.
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

  robot_state::RobotState start_state(*move_group.getCurrentState());   // We will reuse the old goal that we had and plan to it. Note that this will only work if the current state already satisfies the path constraints. So, we need to set the start state to a new pose.
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position .y = -0.05; 
  start_pose2.position.z = 0.8;
  start_state.setFromIK(joint_model_group, start_pose2);
  move_group.setStartState(start_state);

  move_group.setPoseTarget(target_pose1);   // Now we will plan to the earlier pose target from the new start state that we have just created.

  move_group.setPlanningTime(10.0);   // Planning with constraints can be slow because every sample must call an inverse kinematics solver. Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  
  visual_tools.prompt("next step");

  move_group.clearPathConstraints();   // When done with the path constraint be sure to clear it.

 

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
  // Cartesian Paths.   
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
  
  // You can plan a Cartesian path directly by specifying a list of waypoints for the end-effector to go through. Note that we are starting from the new start state above.  The initial pose (start state) does not need to be added to the waypoint list but adding it can help with visualizations
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose2);

  geometry_msgs::Pose target_pose3 = start_pose2;

  target_pose3.position.z -= 0.2;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // right

  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // up and left

  move_group.setMaxVelocityScalingFactor(0.8);   // Cartesian motions are frequently needed to be slower for actions such as approach and retreat grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor of the maximum speed of each joint. Note this is not the speed of the end effector point.

  moveit_msgs::RobotTrajectory trajectory;   // We want the Cartesian path to be interpolated at a resolution of 1 cm  which is why we will specify 0.01 as the max step in Cartesian translation. We will specify the jump threshold as 0.0, effectively disabling it. Warning - disabling the jump threshold while operating real hardware can cause large unpredictable motions of redundant joints and could be a safety issue
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);


  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  


  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
  // Adding/Removing Objects and Attaching/Detaching Objects
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

  moveit_msgs::CollisionObject collision_object;   // Define a collision object ROS message.
  collision_object.header.frame_id = move_group.getPlanningFrame();

  collision_object.id = "box1";   // The id of the object is used to identify it.


  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;


  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = -0.2;
  box_pose.position.z = 1.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  ROS_INFO_NAMED("tutorial", "Add an object into the world");   // Now, let's add the collision object into the world
  planning_scene_interface.addCollisionObjects(collision_objects);

  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);   // Show text in RViz of status

  visual_tools.trigger();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");   // Wait for MoveGroup to receive and process the collision object message


  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
  // Now when we plan a trajectory it will avoid the obstacle
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::Pose another_pose;
  another_pose.orientation.w = 1.0;
  another_pose.position.x = 0.4;
  another_pose.position.y = -0.4;
  another_pose.position.z = 0.9;
  move_group.setPoseTarget(another_pose);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");   // Now, let's attach the collision object to the robot.
  move_group.attachObject(collision_object.id);

  visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);   // Show text in RViz of status
  visual_tools.trigger();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the "   /* Wait for MoveGroup to receive and process the attached collision object message */
                      "robot");



  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
  // Now, let's detach the collision object from the robot.
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  move_group.detachObject(collision_object.id);

  visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);   // Show text in RViz of status
  visual_tools.trigger();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object detaches to the "   /* Wait for MoveGroup to receive and process the attached collision object message */
                      "robot");

  ROS_INFO_NAMED("tutorial", "Remove the object from the world");   // Now, let's remove the collision object from the world.
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);   // Show text in RViz of status
  visual_tools.trigger();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disappears");   /* Wait for MoveGroup to receive and process the attached collision object message */

  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
