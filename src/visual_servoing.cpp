#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <stdlib.h>
#include <cmath>

bool gazebo = false;
std::string target_frame;

ros::Publisher vel_pub;

tf2_ros::Buffer tfBuffer;
geometry_msgs::TransformStamped targetTransform, rootToGripperTransform;

void visual_servo(const ros::TimerEvent&) {
  try{
    targetTransform = tfBuffer.lookupTransform("ar_marker_0", target_frame, ros::Time(0), ros::Duration(5.0));
    //ROS_INFO("Target transform: X %f | Y %f | Z %f", targetTransform.transform.translation.x, targetTransform.transform.translation.y, targetTransform.transform.translation.z);

    rootToGripperTransform = tfBuffer.lookupTransform("root", "ar_marker_0", ros::Time(0), ros::Duration(5.0));
    rootToGripperTransform.transform.translation = geometry_msgs::Vector3(); // We only want rotation
  }
  catch (tf2::TransformException &ex) {
    ROS_ERROR("Error getting (eef -> target) transform: %s",ex.what());
    ros::shutdown();
    return;
  }

  // Transform the pose to the root frame
  //targetTransform = tfBuffer.transform(targetTransform, "root", ros::Duration(1));
  //tfBuffer.transform(targetTransform, targetTransform, "root", ros::Duration(1));
  tf2::doTransform(targetTransform, targetTransform, rootToGripperTransform);
  //ROS_INFO("Target transform (root frame): X %f | Y %f | Z %f", targetTransform.transform.translation.x, targetTransform.transform.translation.y, targetTransform.transform.translation.z);

  // Get 2D angle and error (xy plane)
  double rot_angle = atan2(targetTransform.transform.translation.y, targetTransform.transform.translation.x);
  double error = sqrt(pow(targetTransform.transform.translation.x, 2.0) + pow(targetTransform.transform.translation.y, 2.0));

  double speed = std::min(error, 0.01); // meters/sec

  // EEF velocity
  double eef_vel_x = speed*cos(rot_angle);
  double eef_vel_y = speed*sin(rot_angle);

  ROS_INFO("Sending x: %f,y: %f", eef_vel_x, eef_vel_y);

  if (!gazebo) {
    // Real robot. TODO: send vels to driver (cartesian velocity control)
  }
  else {
    // Simulated robot. send twist to joint_trajectory_control
    geometry_msgs::Twist twist;
    twist.linear.x = eef_vel_x;
    twist.linear.y = eef_vel_y;

    vel_pub.publish(twist);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "visual_servoing");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  vel_pub = node_handle.advertise<geometry_msgs::Twist>("/blitzcrank/velocity_control", 1000);

  if(node_handle.hasParam("gazebo")) {
    node_handle.getParam("gazebo", gazebo);
  }

  if(node_handle.hasParam("target_frame")) {
    node_handle.getParam("target_frame", target_frame);
  } else {
    ROS_ERROR("'target_frame' param not given");
    ros::shutdown();
  }
  ROS_INFO("Target frame: %s", target_frame.c_str());

  // Get eef<-target pose
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Duration(2).sleep();

  ros::Timer timer = node_handle.createTimer(ros::Duration(0.2), visual_servo); //5Hz

  ros::waitForShutdown();

  return 0;
}