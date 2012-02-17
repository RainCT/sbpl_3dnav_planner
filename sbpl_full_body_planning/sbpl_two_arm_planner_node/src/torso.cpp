#include <sbpl_two_arm_planner_node/torso.h>

//Action client initialization
Torso::Torso()
{

	torso_client_ = new TorsoClient("torso_controller/position_joint_action", true);

	//wait for the action server to come up
	while(!torso_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the torso action server to come up");
	}
}

Torso::~Torso()
{
	delete torso_client_;
}

//tell the torso to go up
void Torso::up()
{
	pr2_controllers_msgs::SingleJointPositionGoal up;
	up.position = 0.22587620070341488;  //all the way up is 0.2
	up.min_duration = ros::Duration(2.0);
	up.max_velocity = 1.0;

	ROS_DEBUG("Sending up goal to torso");
	torso_client_->sendGoal(up);
	torso_client_->waitForResult();
}

//tell the torso to go down
void Torso::down()
{
	pr2_controllers_msgs::SingleJointPositionGoal down;
	down.position = 0.012;
	down.min_duration = ros::Duration(2.0);
	down.max_velocity = 1.0;

	ROS_DEBUG("Sending down goal to torso");
	torso_client_->sendGoal(down);
	torso_client_->waitForResult();
}    

