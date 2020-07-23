#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

bool acquired_both_transforms = false;

std::string target_frame, end_effector_frame, cam_frame;

tf2_ros::Buffer tfBuffer;
geometry_msgs::TransformStamped targetTransform, endEffectorTransform;

void republish(tf2_ros::TransformBroadcaster broadcaster) {
  try{
    targetTransform = tfBuffer.lookupTransform(cam_frame, target_frame, ros::Time(0));
    endEffectorTransform = tfBuffer.lookupTransform(cam_frame, end_effector_frame, ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    if(!acquired_both_transforms) {
      ros::Duration(1.0).sleep();
      return;
    }
  }

  acquired_both_transforms = true;
  // Republish transforms, based on the last time they were acquired
  broadcaster.sendTransform(targetTransform);
  broadcaster.sendTransform(endEffectorTransform);
  ros::Duration(1.0).sleep();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ar_track_republish");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  if(node_handle.hasParam("cam_frame")) {
    node_handle.getParam("cam_frame", cam_frame);
  } else {
    ROS_ERROR("'cam_frame' param not given");
    ros::shutdown();
  }
  ROS_INFO("Camera frame: %s", cam_frame.c_str());

  if(node_handle.hasParam("target_frame")) {
    node_handle.getParam("target_frame", target_frame);
  } else {
    ROS_ERROR("'target_frame' param not given");
    ros::shutdown();
  }
  ROS_INFO("Target frame: %s", target_frame.c_str());

  if(node_handle.hasParam("end_effector_frame")) {
    node_handle.getParam("end_effector_frame", end_effector_frame);
  } else {
    ROS_ERROR("'end_effector_frame' param not given");
    ros::shutdown();
  }
  ROS_INFO("End-effector frame: %s", end_effector_frame.c_str());

  tf2_ros::TransformListener tfListener(tfBuffer);
  tf2_ros::TransformBroadcaster tfBroadcaster;

  while(node_handle.ok()) {
    republish(tfBroadcaster);
  }

  return 0;
}
