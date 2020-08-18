#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <kinova_msgs/JointVelocity.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

const double DURATION = 0.5; // Assuming 2Hz Visual-Servoing (only used if gazebo=true, for Joint Trajectory Controller)
robot_state::JointModelGroup* arm_group;
robot_state::RobotStatePtr kinematic_state;

bool gazebo = false;
bool debug = false;

moveit::planning_interface::MoveGroupInterface *move_group;

ros::Publisher traj_pub, joint_vel_pub;

kinova_msgs::JointVelocity joint_vel_msg;

ros::Time latest_timestamp;
bool servoing = false;

Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& u_matrix, const Eigen::MatrixXd& v_matrix, const Eigen::MatrixXd& s_diagonals) {
  return v_matrix * s_diagonals.inverse() * u_matrix.transpose();
}

/*
  Use inverse jacobian to apply joint goal (sent to ros_control) based on the given end-effector velocity
 Inverse jacobian adapted from: https://github.com/UTNuclearRoboticsPublic/jog_arm/blob/kinetic/jog_arm/src/jog_arm/jog_arm_server.cpp
*/
void setTrajectoryFromVelocity(geometry_msgs::TwistStamped msg) {
  latest_timestamp = ros::Time(msg.header.stamp);

  // end-effector velocity vector
  Eigen::VectorXd eef_vel(6);
  eef_vel[0] = msg.twist.linear.x;
  eef_vel[1] = msg.twist.linear.y;
  eef_vel[2] = msg.twist.linear.z;
  eef_vel[3] = msg.twist.angular.x;
  eef_vel[4] = msg.twist.angular.y;
  eef_vel[5] = msg.twist.angular.z;

  kinematic_state = move_group->getCurrentState();

  // Calculate delta_theta
  Eigen::MatrixXd jacobian = kinematic_state->getJacobian(arm_group);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd delta_theta = pseudoInverse(svd.matrixU(), svd.matrixV(), svd.singularValues().asDiagonal()) * eef_vel;

  if(gazebo) {
    // Get current joint thetas
    static const std::string PLANNING_GROUP = "arm";
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(arm_group, joint_values);

    // Add delta_theta to current thetas, send to ros_control
    trajectory_msgs::JointTrajectory traj_msg;

    traj_msg.header.stamp = ros::Time::now();
    traj_msg.joint_names = arm_group->getVariableNames();

    trajectory_msgs::JointTrajectoryPoint point;
    for(int i = 0; i < 6; i++) { // 6 joints
      point.positions.push_back(joint_values[i] + delta_theta[i]);
      point.velocities.push_back(0);
      point.accelerations.push_back(0);
      point.effort.push_back(0);

      point.time_from_start = ros::Duration(DURATION);
    }
    traj_msg.points.push_back(point);

    traj_pub.publish(traj_msg);
  }
  else {
    if(!debug && servoing) {
      // Send delta_thetas directly as velocity
      joint_vel_msg.joint1 = delta_theta[0];
      joint_vel_msg.joint2 = delta_theta[1];
      joint_vel_msg.joint3 = delta_theta[2];
      joint_vel_msg.joint4 = delta_theta[3];
      joint_vel_msg.joint5 = delta_theta[4];
      joint_vel_msg.joint6 = delta_theta[5];
    }
  }
}

void send_kinova_joint_vels(const ros::TimerEvent&) {
  if(!gazebo && !debug && servoing) {
    joint_vel_pub.publish(joint_vel_msg);
  }
}

void check_timestamp(const ros::TimerEvent&) {
  servoing = (ros::Time::now() - latest_timestamp < ros::Duration(1));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "blitzcrank_traj_control");

  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(3);
  spinner.start();

  if(nh.hasParam("gazebo")) {
    nh.getParam("gazebo", gazebo);
  }
  
  if(nh.hasParam("debug")) {
    nh.getParam("debug", debug);
  }

  // Load globals
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  arm_group = kinematic_model->getJointModelGroup("arm");
  move_group = new moveit::planning_interface::MoveGroupInterface("arm");

  move_group->startStateMonitor();

  ros::Subscriber vel_sub = nh.subscribe("/blitzcrank/velocity_control", 1, setTrajectoryFromVelocity);

  traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/j2s6s300/effort_joint_trajectory_controller/command", 1000);

  joint_vel_pub = nh.advertise<kinova_msgs::JointVelocity>("/kinova_driver/in/joint_velocity", 1000);

  ros::Timer vs_timer = nh.createTimer(ros::Duration(0.01), send_kinova_joint_vels); // 100Hz

  ros::Timer check_stamp_timer = nh.createTimer(ros::Duration(1), check_timestamp); // 1Hz

  ros::waitForShutdown();
  return 0;
}
