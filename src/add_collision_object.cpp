#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

tf2_ros::Buffer tfBuffer;
std::string target_frame;
geometry_msgs::TransformStamped targetTransform;

int main(int argc, char **argv)
{
  ros::init (argc, argv, "add_collision_object");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  if(node_handle.hasParam("/target_frame")) {
    node_handle.getParam("/target_frame", target_frame);
  } else {
    ROS_ERROR("'target_frame' param not given");
    ros::shutdown();
  }
  ROS_INFO("Target frame: %s", target_frame.c_str());

  // Get base_link -> target pose
  tf2_ros::TransformListener tfListener(tfBuffer);
  try {
    targetTransform = tfBuffer.lookupTransform("base_link", target_frame, ros::Time(0), ros::Duration(5.0));
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("Error getting (base_link -> target) transform: %s",ex.what());
    ros::shutdown();
    return -1;
  }

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // BEGIN_SUB_TUTORIAL add_cylinder
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

    // by PlanningSceneInterface
    planning_scene_interface.applyCollisionObject(collision_object);
    planning_scene_interface.applyCollisionObject(ground_collision_object);
    planning_scene_interface.applyCollisionObject(side_collision_object);
    
    // By planning scene monitor
    //moveit_msgs::PlanningScene planning_scene_msg;
    // planning_scene_msg.world.collision_objects.push_back(collision_object);
    // planning_scene_msg.is_diff = true;

    // planning_scene::PlanningScenePtr scene_diff = planning_scene->diff(planning_scene_msg);

    // planning_scene->pushDiffs(scene_diff);

    // Allow cylinder to collide with robot
    collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();
    acm.setDefaultEntry("target_object", true);

    std::vector<std::string> acmEntryNames;
    acm.getAllEntryNames(acmEntryNames);
    for(auto i : acmEntryNames) {
        ROS_INFO("%s",i.c_str());
    } 
}
