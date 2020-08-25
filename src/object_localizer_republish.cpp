/*
  This node republishes the last seen transform for the target object given by generic_localizer.
  If the object becomes occluded or stops being recognized, generic_localizer will stop publishing
  the transform, but its last known position will keep being published by this node.
  The target object's name is obtained through ROS param "target_object"
*/

#include <ros/ros.h>
#include <mbot_perception_msgs/RecognizedObject3D.h>
#include <mbot_perception_msgs/RecognizedObject3DList.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>

std::string target_object_class;
geometry_msgs::Pose target_object_pose;
int current_pose_seq = -1;
int latest_published_pose_seq = -1;
bool acquired_object = false;

tf2_ros::Buffer tfBuffer;
geometry_msgs::TransformStamped targetToCamTransform, camToBaseLinkTransform, targetToBaseLinkTransform;

void objectsCallback(mbot_perception_msgs::RecognizedObject3DList msg) {
  for(int i = 0; i < msg.objects.size(); i++) {
    mbot_perception_msgs::RecognizedObject3D object = msg.objects[i];
    if(object.class_name == target_object_class) {
      target_object_pose = object.pose;
      current_pose_seq = msg.header.seq;
      acquired_object = true;
      break;
    }
  }
}

void republish(tf2_ros::TransformBroadcaster broadcaster) {
  if(acquired_object) {
    if(current_pose_seq > latest_published_pose_seq) {
      targetToCamTransform.header.stamp = ros::Time::now();
      targetToCamTransform.transform.translation.x = target_object_pose.position.x;
      targetToCamTransform.transform.translation.y = target_object_pose.position.y;
      targetToCamTransform.transform.translation.z = target_object_pose.position.z;
      targetToCamTransform.transform.rotation = target_object_pose.orientation;

      // The localizer returns the object's pose in camera frame, which we turn into the 
      // above TransformStamped. But if the object stops being recognized, and we republish
      // the frame, it will move with the head. To stop this, we republish in the base_link frame:
      camToBaseLinkTransform = tfBuffer.lookupTransform("base_link", "head_camera_rgb_optical_frame", ros::Time(0));
      tf2::doTransform(targetToCamTransform, targetToBaseLinkTransform, camToBaseLinkTransform);

      latest_published_pose_seq = current_pose_seq;
    }

    broadcaster.sendTransform(targetToBaseLinkTransform);
  }
  else {
    ros::Duration(1.0).sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ar_track_republish");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  if (!node_handle.getParam("/target_object_class", target_object_class)) {
    ROS_ERROR("'target_object_class' param not given");
    ros::shutdown();
  }

  ros::Subscriber objects_sub = node_handle.subscribe("/mbot_perception/generic_localizer/localized_objects", 1, objectsCallback);

  tf2_ros::TransformListener tfListener(tfBuffer);
  tf2_ros::TransformBroadcaster tfBroadcaster;

  targetToCamTransform.header.frame_id = "head_camera_rgb_optical_frame";
  targetToCamTransform.child_frame_id = "localized_object";

  targetToBaseLinkTransform.header.frame_id = "base_link";
  targetToBaseLinkTransform.child_frame_id = "localized_object";
  
  ros::Rate republish_rate(20); // 20Hz
  while(node_handle.ok()) {
    republish(tfBroadcaster);
    republish_rate.sleep();
  }

  return 0;
}
