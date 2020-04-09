#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <blitzcrank/SetCameraOffset.h>
#include <blitzcrank/GetTargetAlignment.h>

geometry_msgs::Transform cam_tf;

ros::ServiceClient getModelClient;
ros::ServiceClient getLinkClient;

enum GazeboEntityType {MODEL, LINK};

// type=MODEL: target is a model name (e.g. camera), reference can be model or model::link (e.g. j2s6s300::root)
geometry_msgs::Transform get_gazebo_transform(GazeboEntityType type, std::string target, std::string reference) {
  geometry_msgs::Quaternion rotation;
  geometry_msgs::Point translation;

  if (type == MODEL) {
    gazebo_msgs::GetModelState get_model_state;
    get_model_state.request.model_name = target;
    get_model_state.request.relative_entity_name = reference;

    if (getModelClient.call(get_model_state)) {
      rotation = get_model_state.response.pose.orientation;
      translation = get_model_state.response.pose.position;
    } else {
      ROS_ERROR("Failed to call service get_model_state (is Gazebo running?)");
    }
  } else if (type == LINK) {
    gazebo_msgs::GetLinkState get_link_state;
    get_link_state.request.link_name = target;
    get_link_state.request.reference_frame = reference;

    if (getLinkClient.call(get_link_state)) {
      rotation = get_link_state.response.link_state.pose.orientation;
      translation = get_link_state.response.link_state.pose.position;
    } else {
      ROS_ERROR("Failed to call service get_link_state (is Gazebo running?)");
    }
  }

  geometry_msgs::Transform tf = geometry_msgs::Transform();
  tf.rotation = rotation;
  tf.translation = geometry_msgs::Vector3();
  tf.translation.x = translation.x; tf.translation.y = translation.y; tf.translation.z = translation.z;
  return tf;
}

// Get accurate camera transform from Gazebo
geometry_msgs::Transform get_cam_tf() {
  // Get transform world/base <- camera from Gazebo
  geometry_msgs::Transform tf = get_gazebo_transform(MODEL, "camera", "j2s6s300::root");

  // Modify tf
  tf2::Quaternion q_orig, q_rot, q_new;
  tf2::convert(tf.rotation, q_orig);

  double r=-1.57, p=0, y=-1.57;
  q_rot.setRPY(r, p, y);

  q_new = q_orig*q_rot; // On the right: relative to orig (camera) angles. On the left: relative to base frame (robot root = world) angles
  q_new.normalize();

  tf2::convert(q_new, tf.rotation);

  return tf;
}

bool set_camera_offset(blitzcrank::SetCameraOffset::Request &req,  blitzcrank::SetCameraOffset::Response &res) {
  // Put camera on its accurate position: offset from there
  cam_tf = get_cam_tf();

  int x_sign = std::rand() & 1;
  if(x_sign == 0) x_sign = -1;

  int y_sign = std::rand() & 1;
  if(y_sign == 0) y_sign = -1;

  int z_sign = std::rand() & 1;
  if(z_sign == 0) z_sign = -1;

  double x_transl = req.dist*x_sign;
  double y_transl = req.dist*y_sign;
  double z_transl = req.dist*z_sign;

  // Apply translation
  cam_tf.translation.x += x_transl; cam_tf.translation.y += y_transl; cam_tf.translation.z += z_transl;

  res.success = true;
  return true;
}

bool get_target_alignment(blitzcrank::GetTargetAlignment::Request &req,  blitzcrank::GetTargetAlignment::Response &res) {
  ROS_INFO("desired transform: x=%f y=%f z=%f", req.desired_transform.translation.x, req.desired_transform.translation.y, req.desired_transform.translation.z);

  geometry_msgs::Transform parent_tf = get_gazebo_transform(LINK, req.parent, "world");
  geometry_msgs::Transform child_tf = get_gazebo_transform(LINK, req.child, "world");

  double diff_x = child_tf.translation.x - parent_tf.translation.x;
  double diff_y = child_tf.translation.y - parent_tf.translation.y;

  ROS_INFO("diff: x=%f y=%f", diff_x, diff_y);

  // TODO 2D error now: change to 3D error
  res.translation_error = sqrt(pow(diff_x - req.desired_transform.translation.x, 2) + pow(diff_y - req.desired_transform.translation.y, 2));
  return true;
}

void broadcast_cam_tf(const ros::TimerEvent& event) {
  static tf2_ros::TransformBroadcaster broadcaster;

  geometry_msgs::TransformStamped tf_stamped = geometry_msgs::TransformStamped();
  tf_stamped.transform = cam_tf;
  tf_stamped.header.stamp = ros::Time::now();
  tf_stamped.header.frame_id = "world";
  tf_stamped.child_frame_id = "camera_rgb_optical_frame";
  broadcaster.sendTransform(tf_stamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "gazebo_interface");

  ros::NodeHandle nh;

  ros::ServiceServer cam_offset_server = nh.advertiseService("/gazebo_interface/set_camera_offset", set_camera_offset);

  ros::ServiceServer get_target_alignment_server = nh.advertiseService("/gazebo_interface/get_target_alignment", get_target_alignment);

  getModelClient = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  getLinkClient = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

  cam_tf = get_cam_tf();

  ros::Timer cam_tf_timer = nh.createTimer(ros::Duration(0.1), broadcast_cam_tf);
  
  ros::spin();

  return 0;
}