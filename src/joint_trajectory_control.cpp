#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

const int ATTEMPTS = 10;
const double TIMEOUT = 0.1;
const double DURATION = 0.5; // Assuming 2Hz Visual-Servoing
robot_state::JointModelGroup* arm_group;
robot_state::RobotStatePtr kinematic_state;

moveit::planning_interface::MoveGroupInterface *move_group;

ros::Publisher traj_pub;

Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& u_matrix, const Eigen::MatrixXd& v_matrix, const Eigen::MatrixXd& s_diagonals) {
  return v_matrix * s_diagonals.inverse() * u_matrix.transpose();
}

void setTrajectory(geometry_msgs::Pose msg) {
  bool found_ik = kinematic_state->setFromIK(arm_group, msg, ATTEMPTS, TIMEOUT);

  if (found_ik) {
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(arm_group, joint_values);

    // Send to ros_control
    trajectory_msgs::JointTrajectory traj_msg;

    traj_msg.header.stamp = ros::Time::now();
    traj_msg.joint_names = arm_group->getVariableNames();

    trajectory_msgs::JointTrajectoryPoint point;
    for(int i = 0; i < 6; i++) { // 6 joints
      point.positions.push_back(joint_values[i]);
      point.velocities.push_back(0);
      point.accelerations.push_back(0);
      point.effort.push_back(0);

      point.time_from_start = ros::Duration(DURATION);
    }
    traj_msg.points.push_back(point);

    traj_pub.publish(traj_msg);
  }
  else {
    ROS_INFO("Did not find IK solution");
  }
}

/*
  Use inverse jacobian to apply joint goal (sent to ros_control) based on the given end-effector velocity
  Inverse jacobian adapted from: https://github.com/UTNuclearRoboticsPublic/jog_arm/blob/kinetic/jog_arm/src/jog_arm/jog_arm_server.cpp
*/
void setTrajectoryFromVelocity(geometry_msgs::Twist msg) {
  // end-effector velocity vector
  Eigen::VectorXd eef_vel(6);
  eef_vel[0] = msg.linear.x;
  eef_vel[1] = msg.linear.y;
  eef_vel[2] = msg.linear.z;
  eef_vel[3] = msg.angular.x;
  eef_vel[4] = msg.angular.y;
  eef_vel[5] = msg.angular.z;

  kinematic_state = move_group->getCurrentState();

  // Calculate delta_theta
  Eigen::MatrixXd jacobian = kinematic_state->getJacobian(arm_group);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd delta_theta = pseudoInverse(svd.matrixU(), svd.matrixV(), svd.singularValues().asDiagonal()) * eef_vel;

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

int main(int argc, char** argv){
  ros::init(argc, argv, "blitzcrank_traj_control");

  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Load globals
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  arm_group = kinematic_model->getJointModelGroup("arm");
  kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
  move_group = new moveit::planning_interface::MoveGroupInterface("arm");

  ros::Subscriber traj_sub = nh.subscribe("/blitzcrank/joint_trajectory_control", 1000, setTrajectory);

  ros::Subscriber vel_sub = nh.subscribe("/blitzcrank/velocity_control", 1000, setTrajectoryFromVelocity);

  traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/j2s6s300/effort_joint_trajectory_controller/command", 1000);

  ros::waitForShutdown();

  return 0;
}