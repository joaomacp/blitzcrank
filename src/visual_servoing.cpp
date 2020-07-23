#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <stdlib.h>
#include <cmath>

std::string target_frame;

ros::Publisher vel_pub;

tf2_ros::Buffer tfBuffer;
geometry_msgs::TransformStamped targetTransform, rootToGripperTransform, eefTransform, toolTransform;
geometry_msgs::Transform errorTransform;
tf::StampedTransform targetTf, eefTf, toolTf;
tf::Transform errorTf;

double visual_servoing_k, visual_servoing_speed_cap;

void visual_servo(const ros::TimerEvent&) {
  ROS_INFO("----------");

  try{
    eefTransform = tfBuffer.lookupTransform("root", "end_effector_marker", ros::Time(0), ros::Duration(5.0));
    //toolTransform = tfBuffer.lookupTransform("marker0_rotated", "virtual_tool", ros::Time(0), ros::Duration(5.0));
    targetTransform = tfBuffer.lookupTransform("root", target_frame, ros::Time(0), ros::Duration(5.0));
  }
  catch (tf2::TransformException &ex) {
    ROS_ERROR("Error getting (eef -> target) transform: %s",ex.what());
    ros::shutdown();
    return;
  }
  ROS_INFO("eefTransform: X %f | Y %f | Z %f", eefTransform.transform.translation.x, eefTransform.transform.translation.y, eefTransform.transform.translation.z);
  ROS_INFO("toolTransform: X %f | Y %f | Z %f", toolTransform.transform.translation.x, toolTransform.transform.translation.y, toolTransform.transform.translation.z);
  ROS_INFO("targetTransform: X %f | Y %f | Z %f", targetTransform.transform.translation.x, targetTransform.transform.translation.y, targetTransform.transform.translation.z);


  tf::transformStampedMsgToTF(eefTransform, eefTf);
  //tf::transformStampedMsgToTF(toolTransform, toolTf);
  tf::transformStampedMsgToTF(targetTransform, targetTf);

  ROS_INFO("eefTf: X %f | Y %f | Z %f", eefTf.getOrigin().getX(), eefTf.getOrigin().getY(), eefTf.getOrigin().getZ());
  ROS_INFO("targetTf: X %f | Y %f | Z %f", targetTf.getOrigin().getX(), targetTf.getOrigin().getY(), targetTf.getOrigin().getZ());

  // Discarding rotation, because we only care about translation
  //eefTf.setRotation(tf::Quaternion(0, 0, 0, 1));
  //toolTf.setRotation(tf::Quaternion(0, 0, 0, 1));
  targetTf.setRotation(tf::Quaternion(0, 0, 0, 1));

  // multiply first, then discard rotation
  //eefTf *= toolTf;
  eefTf.setRotation(tf::Quaternion(0, 0, 0, 1));

  //errorTf = (eefTf*toolTf).inverseTimes(targetTf);
  errorTf = eefTf.inverseTimes(targetTf);

  //ROS_INFO("eefTf*toolTf: X %f | Y %f | Z %f", (eefTf*toolTf).getOrigin().getX(), (eefTf*toolTf).getOrigin().getY(), (eefTf*toolTf).getOrigin().getZ());
  //ROS_INFO("eefTf*toolTf: X %f | Y %f | Z %f", eefTf.getOrigin().getX(), eefTf.getOrigin().getY(), eefTf.getOrigin().getZ());

  ROS_INFO("errorTf: X %f | Y %f | Z %f", errorTf.getOrigin().getX(), errorTf.getOrigin().getY(), errorTf.getOrigin().getZ());

  tf::transformTFToMsg(errorTf, errorTransform);

  ROS_INFO("errorTransform: X %f | Y %f | Z %f", errorTransform.translation.x, errorTransform.translation.y, errorTransform.translation.z);

  ROS_INFO("----------");

  // Scale the transform vector, based on K
  errorTransform.translation.x *= visual_servoing_k;
  errorTransform.translation.y *= visual_servoing_k;
  errorTransform.translation.z *= visual_servoing_k;

  // clamp the vector's magnitude (speed cap)
  double magnitude = sqrt( pow(targetTransform.transform.translation.x, 2.0) + pow(targetTransform.transform.translation.y, 2.0) + pow(targetTransform.transform.translation.z, 2.0) );
  if(magnitude > visual_servoing_speed_cap) { 
    // Divide transform by its magnitude : normalize it to length 1
    errorTransform.translation.x /= magnitude;
    errorTransform.translation.y /= magnitude;
    errorTransform.translation.z /= magnitude;

    // Multiply by speed cap
    errorTransform.translation.x *= visual_servoing_speed_cap;
    errorTransform.translation.y *= visual_servoing_speed_cap;
    errorTransform.translation.z *= visual_servoing_speed_cap;
  }

  ROS_INFO("Sending x: %f,y: %f,z: %f", errorTransform.translation.x, errorTransform.translation.y, errorTransform.translation.z);

  geometry_msgs::Twist twist;
  twist.linear.x = errorTransform.translation.x;
  twist.linear.y = errorTransform.translation.y;
  twist.linear.z = errorTransform.translation.z;

  vel_pub.publish(twist);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "visual_servoing");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  vel_pub = node_handle.advertise<geometry_msgs::Twist>("/blitzcrank/velocity_control", 1000);

  if(node_handle.hasParam("target_frame")) {
    node_handle.getParam("target_frame", target_frame);
  } else {
    ROS_ERROR("'target_frame' param not given");
    ros::shutdown();
  }
  ROS_INFO("Target frame: %s", target_frame.c_str());

  if(node_handle.hasParam("visual_servoing_k")) {
    node_handle.getParam("visual_servoing_k", visual_servoing_k);
  } else {
    ROS_ERROR("'visual_servoing_k' param not given");
    ros::shutdown();
  }
  ROS_INFO("Visual-servoing K: %d", visual_servoing_k);

  if(node_handle.hasParam("visual_servoing_speed_cap")) {
    node_handle.getParam("visual_servoing_speed_cap", visual_servoing_speed_cap);
  } else {
    ROS_ERROR("'visual_servoing_speed_cap' param not given");
    ros::shutdown();
  }
  ROS_INFO("Visual-servoing speed cap: %d", visual_servoing_speed_cap);

  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Duration(2).sleep();

  ros::Timer timer = node_handle.createTimer(ros::Duration(0.2), visual_servo); //5Hz

  ros::waitForShutdown();

  return 0;
}
