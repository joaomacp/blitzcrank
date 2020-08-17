#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

bool acquired_target = false;
bool acquired_end_effector = false;

std::string input_target_frame, input_end_effector_frame, cam_frame;

tf2_ros::Buffer tfBuffer;
geometry_msgs::TransformStamped inputTargetTransform, outputTargetTransform, inputEndEffectorTransform, outputEndEffectorTransform;

void republish(tf2_ros::TransformBroadcaster broadcaster) {
  try{
    inputTargetTransform = tfBuffer.lookupTransform(cam_frame, input_target_frame, ros::Time(0));
    acquired_target = true;
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    if(!acquired_target) {
      ros::Duration(1.0).sleep();
    }
  }

  try{
    inputEndEffectorTransform = tfBuffer.lookupTransform(cam_frame, input_end_effector_frame, ros::Time(0));
    acquired_end_effector = true;
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    if(!acquired_end_effector) {
      ros::Duration(1.0).sleep();
    }
  }

  // Republish transforms, based on the last time they were acquired
  if(acquired_target) {
    outputTargetTransform.header.stamp = ros::Time::now();
    outputTargetTransform.transform = inputTargetTransform.transform;
    broadcaster.sendTransform(outputTargetTransform);
  }

  if(acquired_end_effector) {
    outputEndEffectorTransform.header.stamp = ros::Time::now();
    outputEndEffectorTransform.transform = inputEndEffectorTransform.transform;
    broadcaster.sendTransform(outputEndEffectorTransform);
  }
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

  if(node_handle.hasParam("target_frame")) {
    node_handle.getParam("target_frame", input_target_frame);
  } else {
    ROS_ERROR("'target_frame' param not given");
    ros::shutdown();
  }

  if(node_handle.hasParam("end_effector_frame")) {
    node_handle.getParam("end_effector_frame", input_end_effector_frame);
  } else {
    ROS_ERROR("'end_effector_frame' param not given");
    ros::shutdown();
  }

  tf2_ros::TransformListener tfListener(tfBuffer);
  tf2_ros::TransformBroadcaster tfBroadcaster;

  outputTargetTransform.header.frame_id = cam_frame;
  outputTargetTransform.child_frame_id = "target_marker";

  outputEndEffectorTransform.header.frame_id = cam_frame;
  outputEndEffectorTransform.child_frame_id = "end_effector_marker";

  while(node_handle.ok()) {
    republish(tfBroadcaster);
  }

  return 0;
}