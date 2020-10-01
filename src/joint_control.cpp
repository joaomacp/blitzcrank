/*
  This node subscribes to /blitzcrank/apply_eef_velocity, transforms the end-effector velocity into a
  vector of joint velocities (using the Jacobian pseudoinverse and SVD), and applies the velocities to the arm.
  This is necessary because Kinova drivers don't handle cartesian end-effector velocities well.

  If Gazebo is being used, a Joint Trajectory Controller is used (velocity control doesn't exist
  for the simulated arm).
  
  Inverse jacobian adapted from: https://github.com/UTNuclearRoboticsPublic/jog_arm/blob/kinetic/jog_arm/src/jog_arm/jog_arm_server.cpp
*/

#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <kinova_msgs/JointVelocity.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>

const double TRAJECTORY_TIMESTEP = 0.5; //seconds. Used for Joint Trajectory Controller (only when gazebo=true)

robot_state::JointModelGroup* arm_group;
robot_state::RobotStatePtr kinematic_state;

bool gazebo = false;
bool debug = false;

moveit::planning_interface::MoveGroupInterface *move_group;

ros::Publisher traj_pub, joint_vel_pub;

kinova_msgs::JointVelocity joint_vel_msg;

ros::Time latest_timestamp;
bool servoing = false;

// Calculate Moore-Penrose pseudoinverse given U, V and S from singular value decomposition
Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& u_matrix, const Eigen::MatrixXd& v_matrix, const Eigen::MatrixXd& s_diagonals) {
  return v_matrix * s_diagonals.inverse() * u_matrix.transpose();
}

void applyEndEffectorVelocity(geometry_msgs::TwistStamped msg) {
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

  // Get the arm's jacobian
  Eigen::MatrixXd jacobian = kinematic_state->getJacobian(arm_group);

  // Singular value decomposition
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Joint velocities = (pseudoinverse of Jacobian) * (end-effector velocity)
  Eigen::VectorXd joint_vels = pseudoInverse(svd.matrixU(), svd.matrixV(), svd.singularValues().asDiagonal()) * eef_vel;

  if(gazebo) {
    // Get current joint angles
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(arm_group, joint_values);

    // Apply a joint trajectory: joint_deltas = joint_vels*TIMESTEP. TIMESTEP seconds from now, apply the new joint position.
    trajectory_msgs::JointTrajectory traj_msg;
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.joint_names = arm_group->getVariableNames();

    trajectory_msgs::JointTrajectoryPoint point;
    for(int i = 0; i < 6; i++) { // 6 joints
      point.positions.push_back(joint_values[i] + (joint_vels[i]*TRAJECTORY_TIMESTEP));
      point.velocities.push_back(0);
      point.accelerations.push_back(0);
      point.effort.push_back(0);

      point.time_from_start = ros::Duration(TRAJECTORY_TIMESTEP);
    }
    traj_msg.points.push_back(point);

    traj_pub.publish(traj_msg);
  }
  else {
    if(!debug && servoing) {
      // Set joint velocity directly. This is being published at 100Hz by publish_kinova_joint_vels
      joint_vel_msg.joint1 = joint_vels[0];
      joint_vel_msg.joint2 = joint_vels[1];
      joint_vel_msg.joint3 = joint_vels[2];
      joint_vel_msg.joint4 = joint_vels[3];
      joint_vel_msg.joint5 = joint_vels[4];
      joint_vel_msg.joint6 = joint_vels[5];
    }
  }
}

void publish_kinova_joint_vels(const ros::TimerEvent&) {
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

  // Create publishers and subscribers
  ros::Subscriber vel_sub = nh.subscribe("/blitzcrank/apply_eef_velocity", 1, applyEndEffectorVelocity);

  traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/j2s6s300/effort_joint_trajectory_controller/command", 1000);

  joint_vel_pub = nh.advertise<kinova_msgs::JointVelocity>("/kinova_driver/in/joint_velocity", 1000);

  if(!gazebo) {
    // 0.01s period - 100Hz frequency required by Kinova driver - see https://github.com/Kinovarobotics/kinova-ros#velocity-control-for-joint-space-and-cartesian-space
    ros::Timer vs_timer = nh.createTimer(ros::Duration(0.01), publish_kinova_joint_vels);
  }

  // At 1Hz, check if a velocity message was received in the last second. If not, stop servoing
  ros::Timer check_stamp_timer = nh.createTimer(ros::Duration(1), check_timestamp);

  ros::waitForShutdown();
  return 0;
}
