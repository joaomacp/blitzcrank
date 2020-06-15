// Publish tfs to watch in rviz, for testing purposes

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

geometry_msgs::TransformStamped testTransform;
tf2_ros::Buffer tfBuffer;

void broadcast_test_tf(const ros::TimerEvent& event) {
  static tf2_ros::TransformBroadcaster broadcaster;

  try{
    testTransform = tfBuffer.lookupTransform("root", "marker0_link", ros::Time(0), ros::Duration(5.0));
  }
  catch (tf2::TransformException &ex) {
    ROS_ERROR("Error getting (marker0 -> root) transform: %s",ex.what());
    ros::shutdown();
    return;
  }

  tf2::Quaternion q_orig, q_rot, q_new;
  tf2::convert(testTransform.transform.rotation, q_orig);

  double r=-1.57, p=0, y=1.57;
  q_rot.setRPY(r, p, y);

  q_new = q_orig*q_rot; // On the right: relative to orig angles. On the left: relative to base frame (robot root = world) angles
  q_new.normalize();

  tf2::convert(q_new, testTransform.transform.rotation);

  testTransform.child_frame_id = "marker0_rotated";
  
  broadcaster.sendTransform(testTransform);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "test_tf_broadcaster");

  ros::NodeHandle node;
  
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Timer test_tf_timer = node.createTimer(ros::Duration(0.1), broadcast_test_tf);

  ros::spin();
  return 0;
};