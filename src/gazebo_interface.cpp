#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

tf::Transform cam_tf;

void broadcast_cam_tf(const ros::TimerEvent& event) {
  static tf::TransformBroadcaster broadcaster;
  broadcaster.sendTransform(tf::StampedTransform(cam_tf, ros::Time::now(), "world", "camera_rgb_optical_frame"));

  // TODO Below: delete
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

// TODO Make this function generalizable, if possible: return transform between any two given Gazebo links
tf::Transform get_gazebo_transform() {

}

int main(int argc, char** argv){
  ros::init(argc, argv, "gazebo_interface");

  ros::NodeHandle nh;

  // TODO Get transform world/base <- camera from Gazebo


  ros::Timer publish_pose_timer = nh.createTimer(ros::Duration(0.1), broadcast_cam_tf);
  
  ros::spin();

  return 0;
}