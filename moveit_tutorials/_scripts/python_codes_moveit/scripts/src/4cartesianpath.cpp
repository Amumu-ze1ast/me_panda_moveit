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

  static const std::string PLANNING_GROUP = "panda_arm";  
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);   
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;   
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);



  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
  // Visualization
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");  
  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();   

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  visual_tools.trigger();  


  // Getting Basic Information
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str()); 
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());  
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");

  
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
             std::ostream_iterator<std::string>(std::cout, ", "));

  
  // Start the demo
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Get the current pose of the end-effector (TCP)
  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();

  // Print the current pose
  ROS_INFO("Current Pose:");
  ROS_INFO("Position (x, y, z): %f, %f, %f", 
           current_pose.pose.position.x, 
           current_pose.pose.position.y, 
           current_pose.pose.position.z);
  ROS_INFO("Orientation (qx, qy, qz, qw): %f, %f, %f, %f", 
           current_pose.pose.orientation.x, 
           current_pose.pose.orientation.y, 
           current_pose.pose.orientation.z, 
           current_pose.pose.orientation.w);

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
  // Cartesian Paths.   
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
  
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose.pose);

  geometry_msgs::Pose target_pose3 = current_pose.pose;


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
  

 
  
  // Plan and execute the motion
  move_group.execute(trajectory);

  
    ros::shutdown();
  return 0;
}