#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_group_interface_cpp_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator"; // Change this to your planning group name

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;

    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = start_pose.position.x + 0.1; // Move 0.1m in x direction
    target_pose.position.y = start_pose.position.y + 0.1; // Move 0.1m in y direction
    target_pose.position.z = start_pose.position.z + 0.1; // Move 0.1m in z direction
    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        ROS_INFO("Planning successfully");
        visual_tools.deleteAllMarkers();
        visual_tools.publishAxisLabeled(start_pose, "start_pose");
        visual_tools.publishAxisLabeled(target_pose, "target_pose");
        visual_tools.publishPath(my_plan.trajectory_, rviz_visual_tools::GREEN, rviz_visual_tools::SMALL);
        visual_tools.trigger();
        move_group.execute(my_plan);
    } else {
        ROS_INFO("Planning failed");
    }

    ros::shutdown();
    return 0;
}
