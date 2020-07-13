#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <stdlib.h>
#include <cmath>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <blitzcrank/GetTargetAlignment.h>

/**
 * Approach target object, gripper horizontal in front of target
**/

const double APPROACH_DISTANCE = 0.25;

bool gazebo = false;
std::string target_frame;
std::string root_frame;

// rosrun kinova_demo fingers_action_client.py j2s6s300 percent 75 75 75
void move_fingers(int percent_closed) {
  if(!gazebo) {
    std::ostringstream command;
    command << "rosrun kinova_demo fingers_action_client.py j2s6s300 percent " << percent_closed << " " << percent_closed << " "<< percent_closed;
    system(command.str().c_str());
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "horizontal_approach_target");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  if(node_handle.hasParam("gazebo")) {
    node_handle.getParam("gazebo", gazebo);
  }

  if(node_handle.hasParam("root_frame")) {
    node_handle.getParam("root_frame", root_frame);
  } else {
    ROS_ERROR("'root_frame' param not given");
    ros::shutdown();
  }
  ROS_INFO("Root frame: %s", root_frame.c_str());

  if(node_handle.hasParam("target_frame")) {
    node_handle.getParam("target_frame", target_frame);
  } else {
    ROS_ERROR("'target_frame' param not given");
    ros::shutdown();
  }
  ROS_INFO("Target frame: %s", target_frame.c_str());

  static const std::string PLANNING_GROUP = "arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // Get target pose
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  geometry_msgs::TransformStamped targetTransform;
  try{
    targetTransform = tfBuffer.lookupTransform(root_frame, target_frame, ros::Time(0), ros::Duration(2.0));
    ROS_INFO("Target transform: X %f | Y %f | Z %f", targetTransform.transform.translation.x, targetTransform.transform.translation.y, targetTransform.transform.translation.z);
  }
  catch (tf2::TransformException &ex) {
    ROS_ERROR("Error getting (root -> target) transform: %s",ex.what());
    ros::shutdown();
    return 0;
  }

  const double VEL_SCALING = 0.5; // TODO: use a param?
  ROS_INFO("Setting velocity scaling: %f", VEL_SCALING);
  move_group.setMaxVelocityScalingFactor(VEL_SCALING);

  move_group.setStartStateToCurrentState();

  geometry_msgs::Pose target_pose;

  // Target arm rotation will be parallel to xy (ground) plane, and have an angle pointing from robot base XY -> target XY
  // double rot_angle = atan2(targetTransform.transform.translation.y, targetTransform.transform.translation.x) - 3.14; // TODO normalize

  // FOR HORIZONTAL BASE IN MBOT: xz instead of xy
  double rot_angle = atan2(targetTransform.transform.translation.z, targetTransform.transform.translation.x) - 3.14; // TODO normalize

  tf2::Quaternion q_rot;
  double r=0, p=-0, y=rot_angle; 
  q_rot.setRPY(r, p, y);
  q_rot.normalize();

  tf2::convert(q_rot, target_pose.orientation);
 
 // vertical arm:
 // target_pose.position.x = targetTransform.transform.translation.x + APPROACH_DISTANCE*cos(rot_angle);
 // target_pose.position.y = targetTransform.transform.translation.y + APPROACH_DISTANCE*sin(rot_angle);
 // target_pose.position.z = targetTransform.transform.translation.z + 0.05;
  
// horizontal arm (mbot)
 target_pose.position.x = targetTransform.transform.translation.x + APPROACH_DISTANCE*cos(rot_angle);
 target_pose.position.z = targetTransform.transform.translation.z + APPROACH_DISTANCE*sin(rot_angle);
 target_pose.position.y = targetTransform.transform.translation.y - 0.05;

  move_group.setPoseTarget(target_pose);

  ROS_INFO("Making movegroup plan to target pose: X: %f | Y: %f |  Z: %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Pose goal %s", success ? "SUCCESS" : "FAILED");

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  // give time to see the planned motion in rviz
  ros::Duration(5).sleep();
  move_group.move();

  if(gazebo) {
    // Wait 2 seconds for controller to finish (sometimes "controller failed during execution", but the arm still reaches the goal pose)
    ros::Duration(2).sleep();

    // Call service to evaluate target alignment
    ros::ServiceClient getAlignmentClient = node_handle.serviceClient<blitzcrank::GetTargetAlignment>("/gazebo_interface/get_target_alignment");
    blitzcrank::GetTargetAlignment get_target_alignment;
    get_target_alignment.request.parent = "coke_marker::marker_link";
    get_target_alignment.request.child = "j2s6s300::j2s6s300_link_finger_tip_1";
    get_target_alignment.request.desired_transform.translation.x = APPROACH_DISTANCE*cos(rot_angle);
    get_target_alignment.request.desired_transform.translation.y = APPROACH_DISTANCE*sin(rot_angle);
    get_target_alignment.request.desired_transform.translation.z + 0.05;

    if (getAlignmentClient.call(get_target_alignment)) {
      ROS_INFO("Translation error (2D for now): %f", get_target_alignment.response.translation_error);
    } else {
      ROS_ERROR("Failed to call service get_target_alignment (are Gazebo and gazebo_interface running?)");
    }
  }

  // Since we set the start state we have to clear it before planning other paths
  move_group.setStartStateToCurrentState();

  // OPEN FINGERS //
  //move_fingers(0); - TODO do this with PoseVelocityWithFingers

  ros::shutdown();
  return 0;
}
