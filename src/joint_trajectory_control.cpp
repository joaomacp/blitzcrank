#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

const int ATTEMPTS = 10;
const double TIMEOUT = 0.1;
const double DURATION = 1; // Assuming 1Hz Visual-Servoing
robot_state::JointModelGroup* arm_group;
robot_state::RobotStatePtr kinematic_state;

ros::Publisher traj_pub;

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

      point.time_from_start = ros::Duration(5);
    }
    traj_msg.points.push_back(point);

    traj_pub.publish(traj_msg);
  }
  else {
    ROS_INFO("Did not find IK solution");
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "blitzcrank_traj_control");

  ros::NodeHandle nh;

  // Load globals
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  arm_group = kinematic_model->getJointModelGroup("arm");
  kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));

  ros::Subscriber sub = nh.subscribe("/blitzcrank/joint_trajectory_control", 1000, setTrajectory);

  traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/j2s6s300/effort_joint_trajectory_controller/command", 1000);
  
  ros::spin();

  return 0;
}