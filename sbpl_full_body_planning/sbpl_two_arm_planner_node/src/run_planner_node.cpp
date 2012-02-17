#include <iostream>
#include <ros/ros.h>
#include <sbpl_two_arm_planner_node/sbpl_two_arm_planner_node.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbpl_two_arm_planner");
  ros::AsyncSpinner spinner(1); // Use 1 thread
  spinner.start();
  ros::NodeHandle nh("~");

  sbpl_two_arm_planner::SBPLTwoArmPlannerNode planner;
  if(!planner.init())
  {
    ROS_ERROR("Failed to initialize arm planner node. Exiting.");
    return 0;
  }
  
  geometry_msgs::Pose goal, start;
  double roll, pitch, yaw, start_roll, start_pitch, start_yaw;

  nh.param("goal_x",goal.position.x,0.68);
  nh.param("goal_y",goal.position.y,0.0);
  nh.param("goal_z",goal.position.z,0.7);
  nh.param("goal_r",roll,0.0);
  nh.param("goal_p",pitch,0.0);
  nh.param("goal_y",yaw,0.0);

  nh.param("start_x",start.position.x,0.68);
  nh.param("start_y",start.position.y,0.0);
  nh.param("start_z",start.position.z,0.7);
  nh.param("start_r",start_roll,0.0);
  nh.param("start_p",start_pitch,0.0);
  nh.param("start_y",start_yaw,0.0);

  btQuaternion goal_quat;
  //geometry_msgs::Quaternion gripper_goal_msg;
  goal_quat.setRPY(roll,pitch,yaw);
  tf::quaternionTFToMsg(goal_quat,goal.orientation);

  btQuaternion start_quat;
  //geometry_msgs::Quaternion start_msg;
  start_quat.setRPY(start_roll,start_pitch,start_yaw);
  tf::quaternionTFToMsg(goal_quat,start.orientation);

  ROS_INFO("[Start Object Pose] xyz: %0.3f %0.3f %0.3f  rpy: %0.3f %0.3f %0.3f", goal.position.x, goal.position.y, goal.position.z, start_roll, start_pitch, start_yaw);

  sleep(2);
  //spinner.stop();
  planner.sendArmsToStart(start); 
  //spinner.start();
  sleep(5);

  planner.planToPosition(start, goal);

  //spinner.stop();
  return 0;
}

