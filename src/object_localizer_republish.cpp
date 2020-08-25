/*
  This node republishes the last seen transform for the target object given by generic_localizer.
  If the object becomes occluded or stops being recognized, generic_localizer will stop publishing
  the transform, but its last known position will keep being published by this node.
  The target object's name is obtained through ROS param "target_object"
*/

#include <ros/ros.h>
#include <mbot_perception_msgs/RecognizedObject3D.h>
#include <mbot_perception_msgs/RecognizedObject3DList.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>

std::string target_object_class;
geometry_msgs::Pose target_object_pose;
bool acquired_object = false;

geometry_msgs::TransformStamped outputTargetTransform;

void objectsCallback(mbot_perception_msgs::RecognizedObject3DList msg) {
  for(int i = 0; i < msg.objects.size(); i++) {
    mbot_perception_msgs::RecognizedObject3D object = msg.objects[i];
    if(object.class_name == target_object_class) {
      target_object_pose = object.pose;
      acquired_object = true;
    }
  }
}

void republish(tf2_ros::TransformBroadcaster broadcaster) {
  if(acquired_object) {
    outputTargetTransform.header.stamp = ros::Time::now();
    outputTargetTransform.transform.translation.x = target_object_pose.position.x;
    outputTargetTransform.transform.translation.y = target_object_pose.position.y;
    outputTargetTransform.transform.translation.z = target_object_pose.position.z;
    outputTargetTransform.transform.rotation = target_object_pose.orientation;
    broadcaster.sendTransform(outputTargetTransform);
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

  tf2_ros::TransformBroadcaster tfBroadcaster;

  outputTargetTransform.header.frame_id = "head_camera_rgb_optical_frame";
  outputTargetTransform.child_frame_id = "localized_object";
  
  ros::Rate republish_rate(20); // 20Hz
  while(node_handle.ok()) {
    republish(tfBroadcaster);
    republish_rate.sleep();
  }

  return 0;
}
