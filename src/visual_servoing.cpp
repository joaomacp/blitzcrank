#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <stdlib.h>
#include <cmath>

bool gazebo = false;
std::string target_frame;

ros::Publisher vel_pub;

tf2_ros::Buffer tfBuffer;
geometry_msgs::TransformStamped targetTransform, rootToGripperTransform, eefTransform, toolTransform;
geometry_msgs::Transform errorTransform;
tf::StampedTransform targetTf, eefTf, toolTf;
tf::Transform errorTf;

void visual_servo(const ros::TimerEvent&) {
  /* Old way
  try{
    targetTransform = tfBuffer.lookupTransform("ar_marker_0", target_frame, ros::Time(0), ros::Duration(5.0));
    //ROS_INFO("Target transform: X %f | Y %f | Z %f", targetTransform.transform.translation.x, targetTransform.transform.translation.y, targetTransform.transform.translation.z);

    rootToGripperTransform = tfBuffer.lookupTransform("root", "ar_marker_0", ros::Time(0), ros::Duration(5.0)); // TODO this ar_marker_0 should probably be marker_0_link (think about it - also, this doesn't exist in the real robot, will need to be a static transform)
    rootToGripperTransform.transform.translation = geometry_msgs::Vector3(); // We only want rotation
  }
  catch (tf2::TransformException &ex) {
    ROS_ERROR("Error getting (eef -> target) transform: %s",ex.what());
    ros::shutdown();
    return;
  }

  // Transform the pose to the root frame
  tf2::doTransform(targetTransform, targetTransform, rootToGripperTransform);
  //ROS_INFO("Target transform (root frame): X %f | Y %f | Z %f", targetTransform.transform.translation.x, targetTransform.transform.translation.y, targetTransform.transform.translation.z);
  */

  // New way, with differences
  ROS_INFO("----------");

  // TODO make the marker0_link<-virtual_tool tf be actually marker0_rotated<-virtual_tool, according to rotations on test_tf_broadcaster

  try{
    eefTransform = tfBuffer.lookupTransform("root", "ar_marker_0", ros::Time(0), ros::Duration(5.0));
    toolTransform = tfBuffer.lookupTransform("marker0_link", "virtual_tool", ros::Time(0), ros::Duration(5.0));
    targetTransform = tfBuffer.lookupTransform("root", target_frame, ros::Time(0), ros::Duration(5.0));
  }
  catch (tf2::TransformException &ex) {
    ROS_ERROR("Error getting (eef -> target) transform: %s",ex.what());
    ros::shutdown();
    return;
  }
  ROS_INFO("eefTransform: X %f | Y %f | Z %f", eefTransform.transform.translation.x, eefTransform.transform.translation.y, eefTransform.transform.translation.z);
  ROS_INFO("targetTransform: X %f | Y %f | Z %f", targetTransform.transform.translation.x, targetTransform.transform.translation.y, targetTransform.transform.translation.z);


  tf::transformStampedMsgToTF(eefTransform, eefTf);
  tf::transformStampedMsgToTF(toolTransform, toolTf);
  tf::transformStampedMsgToTF(targetTransform, targetTf);

  ROS_INFO("eefTf: X %f | Y %f | Z %f", eefTf.getOrigin().getX(), eefTf.getOrigin().getY(), eefTf.getOrigin().getZ());
  ROS_INFO("targetTf: X %f | Y %f | Z %f", targetTf.getOrigin().getX(), targetTf.getOrigin().getY(), targetTf.getOrigin().getZ());

  // Discarding rotation, because we only care about translation
  eefTf.setRotation(tf::Quaternion(0, 0, 0, 1));
  toolTf.setRotation(tf::Quaternion(0, 0, 0, 1));
  targetTf.setRotation(tf::Quaternion(0, 0, 0, 1));

  errorTf = (eefTf*toolTf).inverseTimes(targetTf);

  ROS_INFO("errorTf: X %f | Y %f | Z %f", errorTf.getOrigin().getX(), errorTf.getOrigin().getY(), errorTf.getOrigin().getZ());

  tf::transformTFToMsg(errorTf, errorTransform);

  ROS_INFO("errorTransform: X %f | Y %f | Z %f", errorTransform.translation.x, errorTransform.translation.y, errorTransform.translation.z);

  ROS_INFO("----------");

  // 2D
  // Get 2D angle and error (xy plane)
  // double rot_angle = atan2(targetTransform.transform.translation.y, targetTransform.transform.translation.x);
  // double error = sqrt(pow(targetTransform.transform.translation.x, 2.0) + pow(targetTransform.transform.translation.y, 2.0));

  // double speed = std::min(error, 0.01); // meters/sec

  // // EEF velocity
  // double eef_vel_x = speed*cos(rot_angle);
  // double eef_vel_y = speed*sin(rot_angle);

  // ROS_INFO("Sending x: %f,y: %f", eef_vel_x, eef_vel_y);

  // if (!gazebo) {
  //   // Real robot. TODO: send vels to driver (cartesian velocity control)
  // }
  // else {
  //   // Simulated robot. send twist to joint_trajectory_control
  //   geometry_msgs::Twist twist;
  //   twist.linear.x = eef_vel_x;
  //   twist.linear.y = eef_vel_y;

  //   vel_pub.publish(twist);
  // }

  // 3D
  // targetTransform is the gripper_marker->target_marker transform, rotated to base frame
  // TODO should be gripper->target, based on fixed transforms between markers and real objects

  // Scale the transform vector, based on K
  double K = 2; // TODO make this a ROS param
  errorTransform.translation.x *= K;
  errorTransform.translation.y *= K;
  errorTransform.translation.z *= K;

  // clamp the vector's magnitude (speed cap)
  double SPEED_CAP = 0.1; // TODO make this a ROS param - Speed cap is 10cm/sec
  double magnitude = sqrt( pow(targetTransform.transform.translation.x, 2.0) + pow(targetTransform.transform.translation.y, 2.0) + pow(targetTransform.transform.translation.z, 2.0) );
  if(magnitude > SPEED_CAP) { 
    // Divide transform by its magnitude : normalize it to length 1
    errorTransform.translation.x /= magnitude;
    errorTransform.translation.y /= magnitude;
    errorTransform.translation.z /= magnitude;

    // Multiply by speed cap
    errorTransform.translation.x *= SPEED_CAP;
    errorTransform.translation.y *= SPEED_CAP;
    errorTransform.translation.z *= SPEED_CAP;
  }

  ROS_INFO("Sending x: %f,y: %f,z: %f", errorTransform.translation.x, errorTransform.translation.y, errorTransform.translation.z);

  if (!gazebo) {
    // Real robot. TODO: send vels to driver (cartesian velocity control)
  }
  else {
    // Simulated robot. send twist to joint_trajectory_control
    geometry_msgs::Twist twist;
    twist.linear.x = errorTransform.translation.x;
    twist.linear.y = errorTransform.translation.y;
    twist.linear.z = errorTransform.translation.z;

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

  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Duration(2).sleep();

  ros::Timer timer = node_handle.createTimer(ros::Duration(0.2), visual_servo); //5Hz

  ros::waitForShutdown();

  return 0;
}