#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <stdlib.h>
#include <cmath>

std::string target_frame;

ros::Publisher vel_pub;

tf2_ros::Buffer tfBuffer;
geometry_msgs::TransformStamped targetTransform, rootToGripperTransform, eefMarkerVisionTransform, eefMarkerToEefTransform;
geometry_msgs::Transform errorTransform;
tf::StampedTransform targetTf, vTf;
tf::Transform errorTf;

double visual_servoing_k, visual_servoing_speed_cap, visual_servoing_stopping_distance;
bool target_tracking;

bool visual_servo(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  if(!target_tracking) {
    // Obtain the target transform once, at the start (assuming robot remains still while servoing)
    targetTransform = tfBuffer.lookupTransform("root", target_frame, ros::Time(0), ros::Duration(5.0));
  }
  
  ros::Rate vs_rate(20); // 20Hz

  while(true) {
    //ROS_INFO("----------");

    try{
      eefMarkerVisionTransform = tfBuffer.lookupTransform("root", "grasp_tool", ros::Time(0), ros::Duration(5.0));

      if(target_tracking) {
        targetTransform = tfBuffer.lookupTransform("root", target_frame, ros::Time(0), ros::Duration(5.0));
      }
    }
    catch (tf2::TransformException &ex) {
      ROS_ERROR("Error getting eef marker transform: %s",ex.what());
      res.success = false;
      res.message = "Error getting eef marker transform";
      return true;
    }
    //ROS_INFO("eefMarkerVisionTransform: X %f | Y %f | Z %f", eefMarkerVisionTransform.transform.translation.x, eefMarkerVisionTransform.transform.translation.y, eefMarkerVisionTransform.transform.translation.z);
    //ROS_INFO("targetTransform: X %f | Y %f | Z %f", targetTransform.transform.translation.x, targetTransform.transform.translation.y, targetTransform.transform.translation.z);


    tf::transformStampedMsgToTF(eefMarkerVisionTransform, vTf); // vision
    tf::transformStampedMsgToTF(targetTransform, targetTf);

    //ROS_INFO("vTf: X %f | Y %f | Z %f", vTf.getOrigin().getX(), vTf.getOrigin().getY(), vTf.getOrigin().getZ());
    //ROS_INFO("targetTf: X %f | Y %f | Z %f", targetTf.getOrigin().getX(), targetTf.getOrigin().getY(), targetTf.getOrigin().getZ());

    // Discarding rotation, because we only care about translation
    //vTf.setRotation(tf::Quaternion(0, 0, 0, 1));
    //mTf.setRotation(tf::Quaternion(0, 0, 0, 1));
    targetTf.setRotation(tf::Quaternion(0, 0, 0, 1));

    // multiply first, then discard rotation
    //vTf = mTf*vTf;
    vTf.setRotation(tf::Quaternion(0, 0, 0, 1));

    //errorTf = (vTf*mTf).inverseTimes(targetTf);
    errorTf = vTf.inverseTimes(targetTf);

    //ROS_INFO("errorTf: X %f | Y %f | Z %f", errorTf.getOrigin().getX(), errorTf.getOrigin().getY(), errorTf.getOrigin().getZ());

    tf::transformTFToMsg(errorTf, errorTransform);

    //ROS_INFO("errorTransform: X %f | Y %f | Z %f", errorTransform.translation.x, errorTransform.translation.y, errorTransform.translation.z);

    //ROS_INFO("----------");

    double magnitude = sqrt( pow(errorTransform.translation.x, 2.0) + pow(errorTransform.translation.y, 2.0) + pow(errorTransform.translation.z, 2.0) );
    if(magnitude < visual_servoing_stopping_distance) {
      // Converged
      ROS_INFO("Visual servoing finished: reached desired distance to goal.");

      // Send one last message: set velocity to 0
      geometry_msgs::TwistStamped twist;
      twist.header.stamp = ros::Time::now();
      vel_pub.publish(twist);

      res.success = true;
      return true;
    }

    // Scale the transform vector, based on K
    errorTransform.translation.x *= visual_servoing_k;
    errorTransform.translation.y *= visual_servoing_k;
    errorTransform.translation.z *= visual_servoing_k;

    // clamp the vector's magnitude (speed cap)
    magnitude = sqrt( pow(errorTransform.translation.x, 2.0) + pow(errorTransform.translation.y, 2.0) + pow(errorTransform.translation.z, 2.0) );
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

    //ROS_INFO("Sending x: %f,y: %f,z: %f", errorTransform.translation.x, errorTransform.translation.y, errorTransform.translation.z);

    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();
    twist.twist.linear.x = errorTransform.translation.x;
    twist.twist.linear.y = errorTransform.translation.y;
    twist.twist.linear.z = errorTransform.translation.z;

    vel_pub.publish(twist);

    vs_rate.sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "visual_servoing");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  vel_pub = node_handle.advertise<geometry_msgs::TwistStamped>("/kinova_manipulation/apply_eef_velocity", 1000);

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
  ROS_INFO("Visual-servoing K: %f", visual_servoing_k);

  if(node_handle.hasParam("visual_servoing_speed_cap")) {
    node_handle.getParam("visual_servoing_speed_cap", visual_servoing_speed_cap);
  } else {
    ROS_ERROR("'visual_servoing_speed_cap' param not given");
    ros::shutdown();
  }
  ROS_INFO("Visual-servoing speed cap: %f", visual_servoing_speed_cap);

  if(node_handle.hasParam("visual_servoing_stopping_distance")) {
    node_handle.getParam("visual_servoing_stopping_distance", visual_servoing_stopping_distance);
  } else {
    ROS_ERROR("'visual_servoing_stopping_distance' param not given");
    ros::shutdown();
  }
  ROS_INFO("Visual-servoing stopping distance: %f", visual_servoing_stopping_distance);

  if(node_handle.hasParam("target_tracking")) {
    node_handle.getParam("target_tracking", target_tracking);
  } else {
    ROS_ERROR("'target_tracking' param not given");
    ros::shutdown();
  }
  ROS_INFO("Target tracking: %s", target_tracking ? "Enabled" : "Disabled");

  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Duration(2).sleep();

  ros::ServiceServer vs_server = node_handle.advertiseService("/kinova_manipulation/visual_servo", visual_servo);

  ros::spin();
  return 0;
}
