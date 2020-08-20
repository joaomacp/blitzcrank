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
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/Constraints.h>

#include <blitzcrank/GetTargetAlignment.h>
#include <std_srvs/Trigger.h>

/**
 * Move to a pre-grasp pose - gripper horizontal in front of target
**/

const double APPROACH_DISTANCE = 0.08;

bool gazebo = false;
std::string target_frame;
std::string root_frame;

ros::ServiceClient getAlignmentClient;

// rosrun kinova_demo fingers_action_client.py j2s6s300 percent 75 75 75
void move_fingers(int percent_closed) {
  if(!gazebo) {
    std::ostringstream command;
    command << "rosrun kinova_demo fingers_action_client.py j2s6s300 percent " << percent_closed << " " << percent_closed << " "<< percent_closed;
    system(command.str().c_str());
  }
}

bool add_collision_objects() {
  // Get base_link -> target pose
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped targetTransform;
  try {
    targetTransform = tfBuffer.lookupTransform("base_link", target_frame, ros::Time(0), ros::Duration(5.0));
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("Error getting (base_link -> target) transform: %s",ex.what());
    ros::shutdown();
    return false;
  }

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  //
  // Adding Cylinder to Planning Scene
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = "target_object";

  // Define a cylinder which will be added to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  /* Setting height of cylinder. */ // TODO hardcoded for now, change this
  primitive.dimensions[0] = 0.2;
  /* Setting radius of cylinder. */
  primitive.dimensions[1] = 0.02;

  // Define a pose for the cylinder (specified relative to frame_id).
  geometry_msgs::Pose cylinder_pose;
  /* Computing and setting quaternion from axis angle representation. */
  // TODO: compute angle later, for now hardcoding as vertical
  //Eigen::Vector3d cylinder_z_direction(cylinder_params->direction_vec[0], cylinder_params->direction_vec[1], cylinder_params->direction_vec[2]);
  Eigen::Vector3d origin_z_direction(0., 0., 1.);
  Eigen::Vector3d axis;
  //axis = origin_z_direction.cross(cylinder_z_direction);
  axis = origin_z_direction; // hardcode vertical
  axis.normalize();
  //double angle = acos(cylinder_z_direction.dot(origin_z_direction));
  double angle = acos(0);
  cylinder_pose.orientation.x = axis.x() * sin(angle / 2);
  cylinder_pose.orientation.y = axis.y() * sin(angle / 2);
  cylinder_pose.orientation.z = axis.z() * sin(angle / 2);
  cylinder_pose.orientation.w = cos(angle / 2);

  // Setting the position of cylinder.
  cylinder_pose.position.x = targetTransform.transform.translation.x;
  cylinder_pose.position.y = targetTransform.transform.translation.y;
  cylinder_pose.position.z = targetTransform.transform.translation.z;

  // Ground plane
  moveit_msgs::CollisionObject ground_collision_object;
  ground_collision_object.header.frame_id = "base_link";
  ground_collision_object.id = "ground_plane";

  // Define a box
  shape_msgs::SolidPrimitive box_primitive;
  box_primitive.type = box_primitive.BOX;
  box_primitive.dimensions.resize(3);
  box_primitive.dimensions[0] = 1.7;
  box_primitive.dimensions[1] = 1.7;
  box_primitive.dimensions[2] = 0.1;

  // Define a pose for the box (specified relative to frame_id).
  geometry_msgs::Pose box_pose;
  box_pose.position.x = 0;
  box_pose.position.y = 0;
  box_pose.position.z = -0.05;

  // Side plane (to prevent arm extending too much to the side)
  moveit_msgs::CollisionObject side_collision_object;
  side_collision_object.header.frame_id = "base_link";
  side_collision_object.id = "side_plane";

  // Define a box
  shape_msgs::SolidPrimitive side_primitive;
  side_primitive.type = side_primitive.BOX;
  side_primitive.dimensions.resize(3);
  side_primitive.dimensions[0] = 1.5;
  side_primitive.dimensions[1] = 0.1;
  side_primitive.dimensions[2] = 1.5;

  // Define a pose for the box (specified relative to frame_id).
  geometry_msgs::Pose side_pose;
  side_pose.position.x = 0;
  side_pose.position.y = -0.85;
  side_pose.position.z = 0.5;

  /// -- Planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr psm = planning_scene_monitor::PlanningSceneMonitorPtr(
  new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

  planning_scene_monitor::LockedPlanningSceneRW planning_scene = planning_scene_monitor::LockedPlanningSceneRW(psm);

  // Add cylinder as collision object
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(cylinder_pose);
  collision_object.operation = collision_object.ADD;

  // Add ground plane as collision object
  ground_collision_object.primitives.push_back(box_primitive);
  ground_collision_object.primitive_poses.push_back(box_pose);
  ground_collision_object.operation = ground_collision_object.ADD;

  // Add side plane as collision object
  side_collision_object.primitives.push_back(side_primitive);
  side_collision_object.primitive_poses.push_back(side_pose);
  side_collision_object.operation = side_collision_object.ADD;

  // Apply collision objects
  planning_scene_interface.applyCollisionObject(collision_object);
  planning_scene_interface.applyCollisionObject(ground_collision_object);
  planning_scene_interface.applyCollisionObject(side_collision_object);

  // Allow cylinder to collide with robot
  collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();
  acm.setDefaultEntry("target_object", true);

  return true;
}

bool pregrasp(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  // Add collision objects to moveit scene: floor plane, side plane, target object (to make hole in octomap)
  if (!add_collision_objects()) {
    res.success = false;
    res.message = "Error adding collision objects - probably failed to obtain target transform";
    return true;
  }

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
    res.success = false;
    res.message = "Error obtaining target transform";
    return true;
  }

  const double YAW_ANGLE = 0.7;

  // Below is commented: instead of doing this (which seems to cause the arm to stop too soon), we set the moveit config/joint_limits.yaml to have 70% joint velocity
  //const double VEL_SCALING = 0.7; // TODO: use a param?
  //ROS_INFO("Setting velocity scaling: %f", VEL_SCALING);
  //move_group.setMaxVelocityScalingFactor(VEL_SCALING);
  //move_group.setMaxAccelerationScalingFactor(VEL_SCALING);

  move_group.setStartStateToCurrentState();

  geometry_msgs::Pose target_pose;

  // Target arm rotation will be parallel to xy (ground) plane, and have an angle pointing from robot base XY -> target XY
  // double rot_angle = atan2(targetTransform.transform.translation.y, targetTransform.transform.translation.x) - 3.14; // TODO normalize

  double rot_angle = atan2(targetTransform.transform.translation.y, targetTransform.transform.translation.x) - 3.14; // TODO normalize
  // Add manipulator's yaw angle
  rot_angle += YAW_ANGLE;

  tf2::Quaternion q_rot, q_rot_2, q_rot_3, q_new;

  double r=0, p=1.57, y=0;
  q_rot.setRPY(r, p, y);

  q_rot_2.setRPY(1.57, 0, 0);

  q_rot_3.setRPY(0, 0, YAW_ANGLE);

  q_new = q_rot_3*q_rot_2*q_rot;  

  q_new.normalize();

  tf2::convert(q_new, target_pose.orientation);
  
  target_pose.position.x = targetTransform.transform.translation.x + APPROACH_DISTANCE*cos(rot_angle);
  target_pose.position.y = targetTransform.transform.translation.y + APPROACH_DISTANCE*sin(rot_angle);
  target_pose.position.z = targetTransform.transform.translation.z + 0.06;

  move_group.setPoseTarget(target_pose);

  // Set joint constraint: pi/2 < joint1 angle < 3pi/2 - so that arm doesn't move around, and trajectory is more direct
  moveit_msgs::JointConstraint joint1_constraint;
  joint1_constraint.joint_name = "kinova_joint_1";
  joint1_constraint.position = 3.14;
  joint1_constraint.tolerance_below = 3.14/2 + 0.2;
  joint1_constraint.tolerance_above = 3.14/2 + 0.2;
  joint1_constraint.weight = 1.0;

  moveit_msgs::Constraints path_constraints;
  path_constraints.joint_constraints.push_back(joint1_constraint);
  move_group.setPathConstraints(path_constraints);

  ROS_INFO("Making movegroup plan to target pose: X: %f | Y: %f |  Z: %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success) {
    ROS_INFO("Pose goal SUCCESS");
  }
  else {
    ROS_INFO("Pose goal FAILURE");
    res.success = false;
    res.message = "Failed to plan the pregrasp motion";
    return true;
  }

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  // give time to see the planned motion in rviz
  ros::Duration(5).sleep();
  move_group.move();

  if(gazebo) {
    // Wait 2 seconds for controller to finish (sometimes "controller failed during execution", but the arm still reaches the goal pose)
    ros::Duration(2).sleep();

    // Call service to evaluate target alignment
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

  res.success = true;
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pregrasp_service");
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

  getAlignmentClient = node_handle.serviceClient<blitzcrank::GetTargetAlignment>("/gazebo_interface/get_target_alignment");

  ros::ServiceServer pregrasp_server = node_handle.advertiseService("/kinova_manipulation/pregrasp", pregrasp);

  ros::spin();
  return 0;
}
