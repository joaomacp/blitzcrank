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
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/AllowedCollisionEntry.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/Constraints.h>
#include <darknet_ros_py/RecognizedObject.h>
#include <darknet_ros_py/RecognizedObjectArrayStamped.h>
#include <sensor_msgs/RegionOfInterest.h>

#include <std_srvs/Trigger.h>

const double APPROACH_DISTANCE = 0.08;

std::string target_frame;
std::string root_frame;

tf2_ros::Buffer tfBuffer;
geometry_msgs::TransformStamped setGraspTargetTransform;

ros::Publisher set_grasp_target_pub;

ros::ServiceClient getPlanningSceneClient;

bool add_collision_objects(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  // Get target pose, to place collision cylinder
  geometry_msgs::TransformStamped targetTransform;
  try {
    targetTransform = tfBuffer.lookupTransform(root_frame, target_frame, ros::Time(0), ros::Duration(5.0));
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("Error getting %s->target transform: %s", root_frame.c_str(), ex.what());
    res.success = false;
    res.message = "Error obtaining target transform";
    return true;
  }

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = "target_object";

  // Define a cylinder which will be added to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = 0.2; // Cylinder height
  primitive.dimensions[1] = 0.03; // Cylinder radius

  geometry_msgs::Pose cylinder_pose;
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

  // Allow cylinder to collide with robot: the moveit planning scene API is complex:
  // We request the current planning scene, modify its allowedcollisionmatrix to add an entry for "target_object",
  // add a new row with all 1s (true values), and add a new column (a value to every row) also with 1s.
  // Then we apply the planning scene as a diff.
  moveit_msgs::GetPlanningScene::Request request;
  request.components.components = request.components.SCENE_SETTINGS |
                                  request.components.ROBOT_STATE |
                                  request.components.ROBOT_STATE_ATTACHED_OBJECTS |
                                  request.components.WORLD_OBJECT_NAMES |
                                  request.components.WORLD_OBJECT_GEOMETRY |
                                  request.components.OCTOMAP |
                                  request.components.TRANSFORMS |
                                  request.components.ALLOWED_COLLISION_MATRIX |
                                  request.components.LINK_PADDING_AND_SCALING |
                                  request.components.OBJECT_COLORS;

  moveit_msgs::GetPlanningScene::Response response;

  getPlanningSceneClient.call(request, response);

  moveit_msgs::PlanningScene ps = response.scene;

  // Check if "target_object" is already in the AllowedCollisionMatrix - in that case, nothing more is needed
  for(int i = 0; i < ps.allowed_collision_matrix.entry_names.size(); i++) {
    if(ps.allowed_collision_matrix.entry_names[i] == "target_object") {
      res.success = true;
      return true;
    }
  }

  // Continue: modify the obtained PlanningScene by adding "target_object" to the AllowedCollisionMatrix
  ps.robot_state.is_diff = true;
  ps.is_diff = true;
  ps.allowed_collision_matrix.entry_names.push_back("target_object");

  // Add row for target object entry, collision allowed = true for every other object
  moveit_msgs::AllowedCollisionEntry target_object_entry;
  for(int i = 0; i < response.scene.allowed_collision_matrix.entry_names.size(); i++) {
    target_object_entry.enabled.push_back(true);
  }
  ps.allowed_collision_matrix.entry_values.push_back(target_object_entry);

  // For every row (object), set collision allowed = true when colliding with target object
  for(int i = 0; i < ps.allowed_collision_matrix.entry_values.size(); i++) {
    ps.allowed_collision_matrix.entry_values[i].enabled.push_back(true);
  }

  planning_scene_interface.applyPlanningScene(ps);

  res.success = true;
  return true;
}

// Calculate the pregrasp pose based on the object's bounding box (currently), or object class (todo), or other factors like table plane... (todo)
geometry_msgs::Pose get_pregrasp_pose(geometry_msgs::TransformStamped targetTransform) {
  std::string target_object_class;
  if (!ros::param::get("/target_object_class", target_object_class)) {
    ROS_ERROR("'/target_object_class' param not given");
    ros::shutdown();
  }

  geometry_msgs::Pose pregrasp_pose;

  const double YAW_ANGLE = 0.7;

  double rot_angle = atan2(targetTransform.transform.translation.y, targetTransform.transform.translation.x) - 3.14;
  rot_angle += YAW_ANGLE;

  pregrasp_pose.position.x = targetTransform.transform.translation.x + APPROACH_DISTANCE*cos(rot_angle);
  pregrasp_pose.position.y = targetTransform.transform.translation.y + APPROACH_DISTANCE*sin(rot_angle);
  pregrasp_pose.position.z = targetTransform.transform.translation.z + 0.06;
  
  // Get object's bounding box
  sensor_msgs::RegionOfInterest bounding_box;
  bool found_bounding_box = false;
  bool tries = 4;
  for(int attempts = 0; attempts < 4; attempts++) {
    darknet_ros_py::RecognizedObjectArrayStampedConstPtr object_list = ros::topic::waitForMessage<darknet_ros_py::RecognizedObjectArrayStamped>("/mbot_perception/generic_detector/detections", ros::Duration(5.0));

    for(const auto& obj: object_list->objects.objects) {
      if(obj.class_name == target_object_class) {
        bounding_box = obj.bounding_box;
        found_bounding_box = true;
        break;
      }
    }
  }
  if(!found_bounding_box) {
    ROS_ERROR("Couldn't obtain YOLO bounding box of target object");
    ros::shutdown();
  }

  // Modify pregrasp pose based on bounding box (vertical vs horizontal)
  tf2::Quaternion q_rot, q_rot_2, q_rot_3, q_new;
  if(bounding_box.height > bounding_box.width*1.3) {
    ROS_INFO("Vertical bounding box: pregrasp pose with flat hand");

    // TODO simplify these multiplications
    q_rot.setRPY(0, 1.57, 0);

    q_rot_2.setRPY(1.57, 0, 0);

    q_rot_3.setRPY(0, 0, YAW_ANGLE);

    q_new = q_rot_3*q_rot_2*q_rot;
  }
  else {
    ROS_INFO("Horizontal bounding box: pregrasp pose with angled hand");

    q_rot.setRPY(0, 2.3, 0.7);

    // EEF rotation (finger positioning)
    q_rot_2.setRPY(0, 0, 1.57);

    q_new = q_rot*q_rot_2;
  }

  q_new.normalize();

  tf2::convert(q_new, pregrasp_pose.orientation);

  return pregrasp_pose;
}

bool pregrasp(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  // Get target pose
  geometry_msgs::TransformStamped targetTransform;
  try {
    targetTransform = tfBuffer.lookupTransform(root_frame, target_frame, ros::Time(0), ros::Duration(5.0));
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("Error getting %s->target transform: %s", root_frame.c_str(), ex.what());
    res.success = false;
    res.message = "Error obtaining target transform";
    return true;
  }

  // Publish grasp target once: will be republished by the transform_republish node
  // TODO parameterize: don't do this if we want continuous detection - in that case, visual servo will republish the tf.
  setGraspTargetTransform = targetTransform; // copy
  set_grasp_target_pub.publish(setGraspTargetTransform);

  static const std::string PLANNING_GROUP = "arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  move_group.setStartStateToCurrentState();

  geometry_msgs::Pose pregrasp_pose = get_pregrasp_pose(targetTransform);

  move_group.setPoseTarget(pregrasp_pose);

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

  ROS_INFO("Making movegroup plan to pregrasp pose: X: %f | Y: %f |  Z: %f", pregrasp_pose.position.x, pregrasp_pose.position.y, pregrasp_pose.position.z);

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
  ros::Duration(3).sleep();
  move_group.move();

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

  tf2_ros::TransformListener tfListener(tfBuffer);

  set_grasp_target_pub = node_handle.advertise<geometry_msgs::TransformStamped>("/set_grasp_target", 1);

  getPlanningSceneClient = node_handle.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  ros::ServiceServer pregrasp_server = node_handle.advertiseService("/kinova_manipulation/pregrasp", pregrasp);

  ros::ServiceServer add_collision_objects_server = node_handle.advertiseService("/kinova_manipulation/add_collision_objects", add_collision_objects);

  ros::spin();
  return 0;
}
