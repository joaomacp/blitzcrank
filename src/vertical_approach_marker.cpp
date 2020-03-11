#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <iostream>
#include <stdlib.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// rosrun kinova_demo fingers_action_client.py j2s6s300 percent 75 75 75
void move_fingers(int percent_closed) {
  std::ostringstream command;
  command << "rosrun kinova_demo fingers_action_client.py j2s6s300 percent " << percent_closed << " " << percent_closed << " "<< percent_closed;
  system(command.str().c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vertical_approach_marker");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // Raw pointers are frequently used to refer to the move group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

  ROS_INFO("ACTIVE JOINT MODEL NAMES:");
  for( auto const& s : joint_names )
        std::cout << s << std::endl;

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^

  // Get desired pose: ar-track marker
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  geometry_msgs::TransformStamped markerTransform;
  try{
    markerTransform = tfBuffer.lookupTransform("ar_marker_3", "world", ros::Time(0), ros::Duration(2.0));
  }
  catch (tf2::TransformException &ex) {
    ROS_ERROR("Error getting (world -> AR marker) transform: %s",ex.what());
    ros::shutdown();
    return 0;
  }

  // // world -> end-effector
  // try{
  //   markerTransform = tfBuffer.lookupTransform("j2s6s300_end_effector", "world", ros::Time(0), ros::Duration(2.0));
  //   ROS_INFO("TRANSL %f %f %f", markerTransform.transform.translation.x, markerTransform.transform.translation.y, markerTransform.transform.translation.z);
  //   ROS_INFO("ROT %f %f %f %f", markerTransform.transform.rotation.x, markerTransform.transform.rotation.y, markerTransform.transform.rotation.z, markerTransform.transform.rotation.w);
  // }
  // catch (tf2::TransformException &ex) {
  //   ROS_ERROR("Error getting (world -> eef) transform: %s",ex.what());
  //   ros::shutdown();
  //   return 0;
  // }

  const double VEL_SCALING = 0.5;
  ROS_INFO("Setting velocity scaling: %f", VEL_SCALING);
  move_group.setMaxVelocityScalingFactor(VEL_SCALING);

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  move_group.setStartStateToCurrentState();

  geometry_msgs::Pose target_pose;
  target_pose.orientation.x = 1.0;
  target_pose.orientation.y = 0.0; 
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0; 
  //0.001745 -0.219008 0.138630
  target_pose.position.x = markerTransform.transform.translation.x;
  target_pose.position.y = markerTransform.transform.translation.y - 0.05;
  target_pose.position.z = 0.25;
  move_group.setPoseTarget(target_pose);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Pose goal %s", success ? "SUCCESS" : "FAILED");

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  // give time to see the planned motion in rviz
  ros::Duration(5).sleep();
  move_group.move();

  // Since we set the start state we have to clear it before planning other paths
  move_group.setStartStateToCurrentState();

  // OPEN FINGERS //
  move_fingers(0);

  // MOVE DOWN //

  target_pose.position.x = markerTransform.transform.translation.x;
  target_pose.position.y = markerTransform.transform.translation.y - 0.05;
  target_pose.position.z = 0.117;
  move_group.setPoseTarget(target_pose);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Pose goal %s", success ? "SUCCESS" : "FAILED");

  // give time to see the planned motion in rviz
  ros::Duration(5).sleep();
  move_group.move();
  ROS_INFO("Finished moving down.");
  move_group.setStartStateToCurrentState();

  // CLOSE FINGERS //
  move_fingers(100);

  // MOVE UP //

  target_pose.position.x = markerTransform.transform.translation.x;
  target_pose.position.y = markerTransform.transform.translation.y - 0.05;
  target_pose.position.z = 0.30;
  move_group.setPoseTarget(target_pose);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Pose goal %s", success ? "SUCCESS" : "FAILED");

  // give time to see the planned motion in rviz
  ros::Duration(5).sleep();
  move_group.move();
  move_group.setStartStateToCurrentState();

  // MOVE DOWN //

  target_pose.position.x = markerTransform.transform.translation.x;
  target_pose.position.y = markerTransform.transform.translation.y - 0.05;
  target_pose.position.z = 0.121;
  move_group.setPoseTarget(target_pose);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Pose goal %s", success ? "SUCCESS" : "FAILED");

  // give time to see the planned motion in rviz
  ros::Duration(5).sleep();
  move_group.move();
  move_group.setStartStateToCurrentState();

  // OPEN FINGERS //
  move_fingers(0);

  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.
//   moveit_msgs::OrientationConstraint ocm;
//   ocm.link_name = "panda_link7";
//   ocm.header.frame_id = "panda_link0";
//   ocm.orientation.w = 1.0;
//   ocm.absolute_x_axis_tolerance = 0.1;
//   ocm.absolute_y_axis_tolerance = 0.1;
//   ocm.absolute_z_axis_tolerance = 0.1;
//   ocm.weight = 1.0;

//   // Now, set it as the path constraint for the group.
//   moveit_msgs::Constraints test_constraints;
//   test_constraints.orientation_constraints.push_back(ocm);
//   move_group.setPathConstraints(test_constraints);

//   // We will reuse the old goal that we had and plan to it.
//   // Note that this will only work if the current state already
//   // satisfies the path constraints. So, we need to set the start
//   // state to a new pose.
//   robot_state::RobotState start_state(*move_group.getCurrentState());
//   geometry_msgs::Pose start_pose2;
//   start_pose2.orientation.w = 1.0;
//   start_pose2.position.x = 0.55;
//   start_pose2.position.y = -0.05;
//   start_pose2.position.z = 0.8;
//   start_state.setFromIK(joint_model_group, start_pose2);
//   move_group.setStartState(start_state);

//   // Now we will plan to the earlier pose target from the new
//   // start state that we have just created.
//   move_group.setPoseTarget(target_pose);

//   // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
//   // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
//   move_group.setPlanningTime(10.0);

//   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

//   // When done with the path constraint be sure to clear it.
//   move_group.clearPathConstraints();

//   // Since we set the start state we have to clear it before planning other paths
//   move_group.setStartStateToCurrentState();

  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a Cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list but adding it can help with visualizations
//   geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;

//   std::vector<geometry_msgs::Pose> waypoints;
//   waypoints.push_back(target_pose3);

//   target_pose3.position.z -= 0.2;
//   waypoints.push_back(target_pose3);  // down

//   target_pose3.position.y -= 0.2;
//   waypoints.push_back(target_pose3);  // right

//   target_pose3.position.z += 0.2;
//   target_pose3.position.y += 0.2;
//   target_pose3.position.x -= 0.2;
//   waypoints.push_back(target_pose3);  // up and left

//   // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
//   // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
//   // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
//   move_group.setMaxVelocityScalingFactor(0.1);

//   // We want the Cartesian path to be interpolated at a resolution of 1 cm
//   // which is why we will specify 0.01 as the max step in Cartesian
//   // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
//   // Warning - disabling the jump threshold while operating real hardware can cause
//   // large unpredictable motions of redundant joints and could be a safety issue
//   moveit_msgs::RobotTrajectory trajectory;
//   const double jump_threshold = 0.0;
//   const double eef_step = 0.01;
//   double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // END_TUTORIAL

  ros::shutdown();
  return 0;
}