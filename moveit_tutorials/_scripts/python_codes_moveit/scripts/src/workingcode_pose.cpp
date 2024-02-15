#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "move_to_position");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Define move group
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");

    // Set a target position
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = 1.0;  // Identity quaternion for orientation
    target_pose.position.x = 0.3;     // X position
    target_pose.position.y = 0.0;     // Y position
    target_pose.position.z = 0.75;    // Z position

    // Set the target pose
    move_group.setPoseTarget(target_pose);

    // Plan and execute the motion
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
        move_group.execute(my_plan);
    } else {
        ROS_ERROR("Failed to plan motion to target position");
    }

    return 0;
}
