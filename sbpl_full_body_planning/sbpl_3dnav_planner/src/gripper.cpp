#include <sbpl_two_arm_planner_node/gripper.h>

Gripper::Gripper(std::string side)
{
  //Initialize the client for the Action interface to the gripper controller
  //and tell the action client that we want to spin a thread by default
  if(side.compare("left") == 0)
    gripper_client_ = new GripperClient("l_gripper_controller/gripper_action", true);
  else
    gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);

  //wait for the gripper action server to come up 
  while(!gripper_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
  }
}

Gripper::~Gripper()
{
  delete gripper_client_;
}

//Open the gripper
void Gripper::open()
{
  pr2_controllers_msgs::Pr2GripperCommandGoal open;
  open.command.position = 0.085;
  open.command.max_effort = -1;  // -1 Do not limit effort (negative)

  ROS_INFO("Sending open goal");
  gripper_client_->sendGoal(open);
  gripper_client_->waitForResult();
  if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The gripper opened!");
  else
    ROS_INFO("The gripper failed to open.");
}

//Close the gripper
void Gripper::close()
{
  close(0.06);
}

void Gripper::close(double position)
{
  pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
  squeeze.command.position = position;
  squeeze.command.max_effort = -1; //max_effort;  // 50.0 Close gently

  ROS_INFO("Sending squeeze goal");
  gripper_client_->sendGoal(squeeze);
  gripper_client_->waitForResult();
  if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The gripper closed!");
  else
    ROS_INFO("The gripper failed to close.");
}

