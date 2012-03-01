#include <ros/ros.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <actionlib/client/simple_action_client.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;

class Torso
{
private:
  TorsoClient *torso_client_;

public:
  //Action client initialization
  Torso();
  ~Torso();
  void up();  //tell the torso to go down
  void down();
};

