#include <ros/ros.h>
#include <string>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper{
private:
  GripperClient* gripper_client_;  

public:
  //Action client initialization
  Gripper(std::string side);
  ~Gripper();

  //Open the gripper
  void open();
  void close();
  void close(double position);
};


