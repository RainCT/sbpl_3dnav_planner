#include <ros/ros.h>
#include <time.h>
#include <vector>
#include <tf/tf.h>
#include <mapping_msgs/AttachedCollisionObject.h>
#include <sbpl_two_arm_planner_node/GetTwoArmPlan.h>
#include <sbpl_two_arm_planner_node/arm.h>
#include <sbpl_two_arm_planner_node/torso.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <pr2_controllers_msgs/JointTrajectoryGoal.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_mechanism_msgs/ListControllers.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <motion_planning_msgs/FilterJointTrajectoryWithConstraints.h>

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

bool switchArmControllers(int num_arms, int strictness);

static const std::string SWITCH_CONTROLLERS_SERVICE="pr2_controller_manager/switch_controller";
static const std::string LIST_CONTROLLERS_SERVICE="pr2_controller_manager/list_controllers";
static const std::string TWO_ARM_CONTROLLER="both_arms_controller";
static const std::string TWO_ARM_JOINT_TRAJECTORY_ACTION="both_arms_controller/joint_trajectory_action";
static const std::string LEFT_ARM_CONTROLLER="l_arm_controller";
static const std::string RIGHT_ARM_CONTROLLER="r_arm_controller";
static const std::string PLANNING_SERVICE="/sbpl_planning/plan_path";
static const std::string REFERENCE_FRAME="map";
static const std::string COLLISION_OBJECT_REFERENCE_FRAME="map";
static const double SECONDS_PER_WAYPOINT = 0.5;
static const double ALLOWED_SMOOTHING_TIME=3.0;
static const double DISTANCE_FROM_WRIST_TO_GRIPPER=0.14;

bool checkController(std::string controller)
{
  ros::NodeHandle nh;
  pr2_mechanism_msgs::ListControllers srv;
  ros::service::waitForService(LIST_CONTROLLERS_SERVICE);
  ROS_DEBUG("Found the service...");
  ros::ServiceClient client = nh.serviceClient<pr2_mechanism_msgs::ListControllers> (LIST_CONTROLLERS_SERVICE, true);
  ROS_DEBUG("Calling the service...");
  if(client.call(srv.request, srv.response))
    ROS_DEBUG("[move_both_arms] Received a list of controllers.");
  else
  {
    ROS_ERROR("[move_both_arms] Failed to retrieve a list of controllers.");
    return false;
  }
  
  for(size_t i=0; i < srv.response.controllers.size(); i++)
  {
    if(controller.compare(srv.response.controllers[i]) == 0)
    {
      if(srv.response.state[i].compare("running") == 0) 
        return true;
      return false;
    }
  }

  ROS_WARN("[move_both_arms] Controller %s not found when checking status!", controller.c_str());
  return false;
}

void printTrajectory(const trajectory_msgs::JointTrajectory &traj)
{
  ROS_INFO("Frame: %s  Stamp: %f  Seq: %d", traj.header.frame_id.c_str(),traj.header.stamp.toSec(),int(traj.header.seq));
  ROS_INFO("Joints(%d):", int(traj.joint_names.size()));
  for(size_t i = 0; i < traj.joint_names.size(); ++i)
    ROS_INFO("[%d] %s", int(i), traj.joint_names[i].c_str());

  ROS_INFO("Waypoints(%d):",int(traj.points.size()));
  for(size_t i = 0; i < traj.points.size(); ++i)
  {
    if(traj.points[i].positions.size() == 14)
      ROS_INFO(" [%d] (t:%0.3f) {r: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f} {l: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f}",int(i),traj.points[i].time_from_start.toSec(),traj.points[i].positions[0],traj.points[i].positions[1],traj.points[i].positions[2],traj.points[i].positions[3],traj.points[i].positions[4],traj.points[i].positions[5],traj.points[i].positions[6],traj.points[i].positions[7],traj.points[i].positions[8],traj.points[i].positions[9],traj.points[i].positions[10],traj.points[i].positions[11],traj.points[i].positions[12],traj.points[i].positions[13]);
    
    else if(traj.points[i].positions.size() == 7)
      ROS_INFO(" [%d] (t:%0.3f) %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f",int(i),traj.points[i].time_from_start.toSec(),traj.points[i].positions[0],traj.points[i].positions[1],traj.points[i].positions[2],traj.points[i].positions[3],traj.points[i].positions[4],traj.points[i].positions[5],traj.points[i].positions[6]);
    else
    {
      for(size_t j = 0; j < traj.points[i].positions.size(); ++j)
        ROS_INFO("[%d][%d] %0.2f", int(i), int(j),traj.points[i].positions[j]);
    }
  }
}

bool filterTrajectory(const trajectory_msgs::JointTrajectory &trajectory_in, trajectory_msgs::JointTrajectory &trajectory_out)
{
  ros::NodeHandle nh;
  ros::ServiceClient filter_trajectory_client = nh.serviceClient<motion_planning_msgs::FilterJointTrajectoryWithConstraints>("trajectory_filter/filter_trajectory_with_constraints");

  motion_planning_msgs::FilterJointTrajectoryWithConstraints::Request  req;
  motion_planning_msgs::FilterJointTrajectoryWithConstraints::Response res;
  
  req.trajectory = trajectory_in;
  //req.goal_constraints = original_request_.motion_plan_request.goal_constraints;
  req.allowed_time = ros::Duration(ALLOWED_SMOOTHING_TIME);
  ros::Time smoothing_time = ros::Time::now();

  if(filter_trajectory_client.call(req,res))
  {
    ROS_INFO("[move_both_arms] Smoothing time is %0.3f", (ros::Time::now()-smoothing_time).toSec());
    trajectory_out = res.trajectory;
    return true;
  }
  else
  {
    ROS_ERROR("[move_both_arms] Service call to filter trajectory failed.");
    return false;
  }
}

void addCurrentStateToTrajectory(trajectory_msgs::JointTrajectory &trajectory_in, trajectory_msgs::JointTrajectory &trajectory_out)
{
  trajectory_out = trajectory_in;
  trajectory_out.points.resize(trajectory_in.points.size()+1);

  ROS_INFO("[move_both_arms] Get state of both arms.");
  pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr state = ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>("both_arms_controller/state");
  
  trajectory_out.points[0].positions = state->actual.positions;
  trajectory_out.points[0].time_from_start = ros::Duration(0.5);

  printf("CURRENT POSITION OF ARMS:  ");
  for (size_t i = 0; i < state->actual.positions.size(); ++i)
    printf("%0.3f  ", state->actual.positions[i]);
  printf("\n");

  for (size_t i = 0; i < trajectory_in.points.size(); ++i)
  {
    trajectory_out.points[i+1].time_from_start = trajectory_in.points[i].time_from_start+ros::Duration(0.5);
    trajectory_out.points[i+1].positions = trajectory_in.points[i].positions;
  }
  trajectory_out.header.stamp = ros::Time::now();
}

bool addCollisionObjects(std::string filename, std::vector<double> &offset)
{
  int num_obs;
  char sTemp[1024];
  std::vector<std::vector<double> > objects;
  std::vector<std::string> object_ids;
  mapping_msgs::CollisionObject object;
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<mapping_msgs::CollisionObject>("collision_object", 2);

  FILE* fCfg = fopen(filename.c_str(), "r");
  if(fCfg == NULL)
    return false;

  // get number of objects
  if(fscanf(fCfg,"%d",&num_obs) < 1)
    printf("Parsed string has length < 1.(number of obstacles)\n");

  ROS_INFO("[move_both_arms] Parsing collision object file with %i objects.",num_obs);

  //get {x y z dimx dimy dimz} for each object
  objects.resize(num_obs, std::vector<double>(6,0.0));
  object_ids.clear();
  for (int i=0; i < num_obs; ++i)
  {
    if(fscanf(fCfg,"%s",sTemp) < 1)
      printf("Parsed string has length < 1.\n");
    object_ids.push_back(sTemp);

    for(int j=0; j < 6; ++j)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        printf("Parsed string has length < 1. (object parameters for %s)\n", object_ids.back().c_str());
      if(!feof(fCfg) && strlen(sTemp) != 0)
        objects[i][j] = atof(sTemp);
    }
  }

  if(object_ids.size() != objects.size())
  {
    ROS_INFO("object id list is not same length as object list. exiting.");
    return false;
  }

  sleep(2); //hack
  object.shapes.resize(1);
  object.poses.resize(1);
  object.shapes[0].dimensions.resize(3);
  for(size_t i = 0; i < objects.size(); i++)
  {
    object.id = object_ids[i];
    object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
    object.shapes[0].type = geometric_shapes_msgs::Shape::BOX;
    object.header.frame_id = COLLISION_OBJECT_REFERENCE_FRAME;
    object.header.stamp = ros::Time::now();

    object.poses[0].position.x = objects[i][0]+offset[0];
    object.poses[0].position.y = objects[i][1]+offset[1];
    object.poses[0].position.z = objects[i][2]+offset[2];
    object.poses[0].orientation.x = 0;  
    object.poses[0].orientation.y = 0;  
    object.poses[0].orientation.z = 0;  
    object.poses[0].orientation.w = 1;  

    object.shapes[0].dimensions[0] = objects[i][3];
    object.shapes[0].dimensions[1] = objects[i][4];
    object.shapes[0].dimensions[2] = objects[i][5];

    pub.publish(object);
    ROS_INFO("[move_both_arms] [%d] id: %s xyz: %0.3f %0.3f %0.3f dims: %0.3f %0.3f %0.3f",int(i),object_ids[i].c_str(),objects[i][0],objects[i][1],objects[i][2],objects[i][3],objects[i][4],objects[i][5]);
    sleep(1);
    pub.publish(object); //hack
    sleep(1);
  }
  return true;
}

void removeObject(std::string id)
{
  mapping_msgs::CollisionObject object;
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<mapping_msgs::CollisionObject>("collision_object", 2);
  sleep(2); //hack

  object.id = id;
  object.operation.operation = mapping_msgs::CollisionObjectOperation::REMOVE;
  object.header.stamp = ros::Time::now();

  pub.publish(object);
  sleep(1);
}

bool attachObject(std::string object_file, geometry_msgs::Pose rarm_object_pose)
{
  char sTemp[1024];
  double radius=0;
  float temp[4];
  ros::NodeHandle nh;
  geometry_msgs::Point point;
  mapping_msgs::AttachedCollisionObject att_object, sbpl_att_object;
 
  ros::Publisher pub = nh.advertise<mapping_msgs::AttachedCollisionObject>("attached_collision_object", 1);
  ros::Publisher pub_sbpl = nh.advertise<mapping_msgs::AttachedCollisionObject>("sbpl_attached_collision_object", 1);
  
  att_object.link_name = "r_gripper_r_finger_tip_link";
  att_object.touch_links.push_back("r_gripper_palm_link");
  att_object.touch_links.push_back("r_gripper_r_finger_link");
  att_object.touch_links.push_back("r_gripper_l_finger_link");
  att_object.touch_links.push_back("r_gripper_l_finger_tip_link");
  att_object.touch_links.push_back("l_gripper_palm_link");
  att_object.touch_links.push_back("l_gripper_r_finger_link");
  att_object.touch_links.push_back("l_gripper_l_finger_link");
  att_object.touch_links.push_back("l_gripper_l_finger_tip_link");
  att_object.touch_links.push_back("l_gripper_r_finger_tip_link");
  att_object.object.header.frame_id = "r_wrist_roll_link";
  att_object.object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
  att_object.object.header.stamp = ros::Time::now();
  att_object.object.shapes.resize(1);
  att_object.object.shapes[0].type = geometric_shapes_msgs::Shape::BOX;
  att_object.object.poses.resize(1);
  att_object.object.poses[0] = rarm_object_pose;

  FILE* fid = fopen(object_file.c_str(), "r");
  if(fid == NULL)
  {
    ROS_ERROR("[move_both_arms] Failed to open object file. (%s)", object_file.c_str());
    return false;
  }

  // object name
  if(fscanf(fid,"%s",sTemp) < 1)
    ROS_WARN("Parsed string has length < 1. (%s)", sTemp);
  att_object.object.id = sTemp;

  // dims
  if(fscanf(fid,"%s",sTemp) < 1)
    ROS_WARN("Parsed string has length < 1. (%s)", sTemp);
  
  if(strcmp(sTemp, "dims:") == 0)
  {
    if(fscanf(fid,"%f %f %f",&(temp[0]),&(temp[1]),&(temp[2])) < 1)
      ROS_WARN("Failed to parse dims.");
    att_object.object.shapes[0].dimensions.resize(3,0);
    att_object.object.shapes[0].dimensions[0] = temp[0];
    att_object.object.shapes[0].dimensions[1] = temp[1];
    att_object.object.shapes[0].dimensions[2] = temp[2];  
  }

  att_object.object.poses[0].position.x += att_object.object.shapes[0].dimensions[0];
  att_object.object.poses[0].position.y = -rarm_object_pose.position.y;

  ROS_DEBUG("TRANSLATED OF POSE: %f %f %f ", att_object.object.poses[0].position.x, att_object.object.poses[0].position.y, att_object.object.poses[0].position.z);

  sbpl_att_object = att_object;

  sbpl_att_object.object.shapes[0].dimensions.clear();

  //the dimensions of the object are secretly stored in the pose :)
  sbpl_att_object.object.poses[0].position.x = att_object.object.shapes[0].dimensions[0];
  sbpl_att_object.object.poses[0].position.y = att_object.object.shapes[0].dimensions[1];
  sbpl_att_object.object.poses[0].position.z = att_object.object.shapes[0].dimensions[2];

  bool first_point = true;
  while(!feof(fid))
  {
    if(sTemp[0] == '#') //comments
      fgets(sTemp, 1024,fid);
    else
    {
      if(fscanf(fid,"%f %f %f %f", &(temp[0]),&(temp[1]),&(temp[2]),&(temp[3])) < 1)
      {
        ROS_WARN("Read an incorrect line. May not be a problem.");
        continue;
      }
      point.x = temp[0];
      point.y = temp[1];
      point.z = temp[2];
      radius = temp[3];
      sbpl_att_object.object.shapes[0].vertices.push_back(point);
      sbpl_att_object.object.shapes[0].dimensions.push_back(radius);

      //set the Z inflation
      //HACK: This just sets above, assuming some kind of tray for now
      if(first_point)
      {
        sbpl_att_object.object.poses[0].orientation.x = temp[2]+temp[3]/2.0;
        first_point = false;
      }
    }
  }
  ROS_INFO("[move_both_arms] Parsed an object file with object '%s' with %d points", sbpl_att_object.object.id.c_str(),int(sbpl_att_object.object.shapes[0].vertices.size()));

  ROS_DEBUG("SBPL Attached Object:");
  for(size_t i = 0; i < sbpl_att_object.object.shapes[0].vertices.size(); ++i)
    ROS_DEBUG("[%s] [%d] xyz:  %0.3f %0.3f %0.3f radius: %0.3f",sbpl_att_object.object.id.c_str(),int(i),sbpl_att_object.object.shapes[0].vertices[i].x,sbpl_att_object.object.shapes[0].vertices[i].y,sbpl_att_object.object.shapes[0].vertices[i].z,sbpl_att_object.object.shapes[0].dimensions[i]);

  ROS_INFO("[move_both_arms] Publishing the attached collision object");
  pub.publish(att_object);
  ROS_INFO("[move_both_arms] Publishing the sbpl attached collision object");
  pub_sbpl.publish(sbpl_att_object);
  sleep(1);
  ROS_DEBUG("Publishing the attached collision object");
  pub.publish(att_object);
  ROS_DEBUG("Publishing the sbpl attached collision object");
  pub_sbpl.publish(sbpl_att_object);
  sleep(1);

  return true;
}

bool sendBothArmsToConfiguration(const trajectory_msgs::JointTrajectory &traj)
{
  TrajClient* traj_client = new TrajClient(TWO_ARM_JOINT_TRAJECTORY_ACTION, true);
  pr2_controllers_msgs::JointTrajectoryGoal goal;
  std::vector<std::string> joint_names(7);  //NOT NEEDED
  joint_names[0] = "_shoulder_pan_joint";
  joint_names[1] = "_shoulder_lift_joint";
  joint_names[2] = "_upper_arm_roll_joint";
  joint_names[3] = "_elbow_flex_joint";
  joint_names[4] = "_forearm_roll_joint";
  joint_names[5] = "_wrist_flex_joint";
  joint_names[6] = "_wrist_roll_joint";

  /*
  if(!switchArmControllers(2, pr2_mechanism_msgs::SwitchController::Request::STRICT))
    return false;
  */

  if(traj.points.size() == 0)
  {
    ROS_ERROR("Trajectory is empty. Exiting.");
    return false;
  }
  
  goal.trajectory = traj;
  goal.trajectory.header.stamp = ros::Time::now();
  goal.trajectory.header.frame_id = "base_footprint";
  ROS_DEBUG("Waiting for both arms joint trajectory action server");
  while(!traj_client->waitForServer(ros::Duration(5.0)))
    ROS_ERROR("Waiting for the joint_trajectory_action server");

  traj_client->sendGoal(goal);

  ROS_DEBUG("sent trajectory");

  delete traj_client;

  return true;
}

bool sendBothArmsToConfiguration(const std::vector<std::vector<double> >&rangles, const std::vector<std::vector<double> > &langles, double seconds_per_waypoint)
{
  TrajClient* traj_client = new TrajClient(TWO_ARM_JOINT_TRAJECTORY_ACTION, true);
  pr2_controllers_msgs::JointTrajectoryGoal goal;
  std::vector<std::string> joint_names(7);
  joint_names[0] = "_shoulder_pan_joint";
  joint_names[1] = "_shoulder_lift_joint";
  joint_names[2] = "_upper_arm_roll_joint";
  joint_names[3] = "_elbow_flex_joint";
  joint_names[4] = "_forearm_roll_joint";
  joint_names[5] = "_wrist_flex_joint";
  joint_names[6] = "_wrist_roll_joint";

  if(!switchArmControllers(2, pr2_mechanism_msgs::SwitchController::Request::STRICT))
    return false;

  if(rangles.size() != langles.size())
  {
    ROS_ERROR("Right arm trajectory size does not equal left arm trajectory size.");
    return false;
  }

  goal.trajectory.header.stamp = ros::Time::now();
  goal.trajectory.header.frame_id = REFERENCE_FRAME;
  goal.trajectory.joint_names.resize(14);
  for(size_t i = 0; i < joint_names.size(); ++i)
  {
    goal.trajectory.joint_names[i] = "r" + joint_names[i];
    goal.trajectory.joint_names[i+7] = "l" + joint_names[i];
  }

  goal.trajectory.points.resize(rangles.size());
  ROS_INFO("Time stamp on the trajectory: %f", goal.trajectory.header.stamp.toSec()); 
  for(size_t i = 0; i < goal.trajectory.points.size(); ++i)
  {
    goal.trajectory.points[i].positions.resize(14,0);
    for(size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[i].positions[j] = rangles[i][j];
      goal.trajectory.points[i].positions[j+7] = langles[i][j];
    }
    goal.trajectory.points[i].time_from_start = ros::Duration((double(i)+1.0)*seconds_per_waypoint);
    ROS_INFO("[%d] time_from_start: %0.3f", int(i), goal.trajectory.points[i].time_from_start.toSec());
  }

  ROS_DEBUG("Waiting for both arms joint trajectory action server");
  while(!traj_client->waitForServer(ros::Duration(5.0)))
    ROS_ERROR("Waiting for the joint_trajectory_action server");

  traj_client->sendGoal(goal);

  ROS_DEBUG("sent trajectory");

  delete traj_client;

  return true;
}

bool switchArmControllers(int num_arms, int strictness)
{
  ros::NodeHandle nh;
  pr2_mechanism_msgs::SwitchController srv;

  if(num_arms == 2)
  {
    if(checkController(TWO_ARM_CONTROLLER))
    {
      ROS_INFO("[move_both_arms] No need to switch controllers. %s is running.",TWO_ARM_CONTROLLER.c_str()); 
      return true;
    }
    srv.request.start_controllers.push_back(TWO_ARM_CONTROLLER);
    srv.request.stop_controllers.push_back(LEFT_ARM_CONTROLLER);
    srv.request.stop_controllers.push_back(RIGHT_ARM_CONTROLLER);
  }
  else
  {
    srv.request.start_controllers.push_back(LEFT_ARM_CONTROLLER);
    srv.request.start_controllers.push_back(RIGHT_ARM_CONTROLLER);
    srv.request.stop_controllers.push_back(TWO_ARM_CONTROLLER);
  }

  srv.request.strictness = strictness;

  ROS_DEBUG("Waiting for the controller switching service...(%s)", SWITCH_CONTROLLERS_SERVICE.c_str());
  ros::service::waitForService(SWITCH_CONTROLLERS_SERVICE);
  ROS_DEBUG("Found the service...");
  
  ros::ServiceClient client = nh.serviceClient<pr2_mechanism_msgs::SwitchController> (SWITCH_CONTROLLERS_SERVICE, true);

  ROS_DEBUG("Calling the service...");
  if(client.call(srv.request, srv.response))
  {
    ROS_DEBUG("Obtained a response.");
    if(srv.response.ok)
      ROS_INFO("[move_both_arms] Successfully switched controllers.");    
    else
    {
      ROS_ERROR("[move_both_arms] Failed to switch controllers.");
      return false;
    }
  }
  return true;
}

bool sendBothArmsToPose(geometry_msgs::PoseStamped pose, geometry_msgs::Pose rarm_object, geometry_msgs::Pose larm_object, std::vector<double> rangles, std::vector<double> langles)
{
  Arm* rarm = new Arm("right");
  Arm* larm = new Arm("left");
  //rarm->setReferenceFrame("map");
  //larm->setReferenceFrame("map");
  std::vector<double> rik_solution(7,0), lik_solution(7,0);
  tf::Pose tstart, tright, tleft;
  geometry_msgs::Pose mright, mleft;
  geometry_msgs::PoseStamped p;

  tf::poseMsgToTF(pose.pose,tstart);
  tf::poseMsgToTF(rarm_object,tright);
  tf::poseMsgToTF(larm_object,tleft);

  tright = tstart*tright;
  tleft = tstart*tleft;

  tf::poseTFToMsg(tright, mright);
  tf::poseTFToMsg(tleft, mleft);

  if(!switchArmControllers(2, 1))
  {
    ROS_ERROR("Failed to switch controllers");
    return false;
  }

  ROS_INFO("[move_both_arms] Sending the right arm to the start position xyz: %0.3f %0.3f %0.3f  quat: %0.3f %0.3f %0.3f %03f.", mright.position.x, mright.position.y, mright.position.z, mright.orientation.x, mright.orientation.y, mright.orientation.z, mright.orientation.w);
  p.pose = mright;
  p.header.frame_id = pose.header.frame_id;
  p.header.stamp = ros::Time::now();
  if(!rarm->computeIK(p, rangles, rik_solution))
  {
    ROS_ERROR("[move_both_arms] Failed to compute IK for the right arm.");
    return false;
  }

  ROS_INFO("[move_both_arms] Sending the left arm to the start position xyz: %0.3f %0.3f %0.3f  quat: %0.3f %0.3f %0.3f %0.3f.", mleft.position.x, mleft.position.y, mleft.position.z, mleft.orientation.x, mleft.orientation.y, mleft.orientation.z, mleft.orientation.w);
  p.pose = mleft;
  p.header.stamp = ros::Time::now();
  if(!larm->computeIK(p, langles, lik_solution))
  {
    ROS_ERROR("[move_both_arms] Failed to compute IK for the left arm.");
    return false;
  }
  
  std::vector<std::vector<double> > rpath, lpath;
  rpath.push_back(rik_solution);
  lpath.push_back(lik_solution);
  if(!sendBothArmsToConfiguration(rpath,lpath,5.0))
  {
    ROS_ERROR("[move_both_arms] Failed to use the both_arms_controller to move the arms to the desired poses.");
    return false;
  }

  ROS_INFO("[move_both_arms] Arms moved to the desired poses using the both_arms_controller.");
  delete rarm;
  delete larm;

  return true;
}

void adjustTorso(int direction)
{
  Torso torso;
  if(direction == 0)
    torso.down();
  else
    torso.up();
}

void fuckPoopy()
{
  double p[14] = {-3.4135595328166346e-05, 0.018054267217705977, -0.0052482025914821762, -0.43538332064833618, 3.14, -0.70388993850957604, -0.00073822462991829374, 2.9466032750669058e-05, 0.018055107785013291, 0.0049674212797405914, -0.43544760577650798, 3.14, -0.70323083951968535, 0.0};
  std::vector<double> pv(p, p+sizeof(p)/sizeof(double));
  std::vector<std::vector<double> > rangles(1,std::vector<double>(7,0)), langles(1,std::vector<double>(7,0));

  ROS_ERROR("FUCK POOPY");
  ROS_ERROR("FUCK POOPY");
  ROS_ERROR("FUCK POOPY");

  for(size_t i = 0; i < 7; ++i)
  {
    rangles[0][i] = pv[i];
    langles[0][i] = pv[i+7];
  }

  sendBothArmsToConfiguration(rangles, langles, 0.5);
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "request_two_arm_plan");
  ros::NodeHandle nh;
  ros::NodeHandle ph("~");
  bool raise_torso=false, execute_plan, go_to_start, plan_path, add_collision_objects, print_path,filter_traj, setup_experiment;
  double goal_roll=0,goal_pitch=0,goal_yaw=0;
  double start_roll=0,start_pitch=0,start_yaw=0;
  double rarm_object_roll=0,rarm_object_pitch=0,rarm_object_yaw=0;
  double larm_object_roll=0,larm_object_pitch=0,larm_object_yaw=0;
  std::string attached_object, known_objects, remove_object;
  btQuaternion btgoal, btstart, btoffset;
  geometry_msgs::Pose rarm_object, larm_object;
  std::vector<double> langles(7,0), rangles(7,0), object_offset(3,0.0);
  sbpl_two_arm_planner_node::GetTwoArmPlan::Request req;
  sbpl_two_arm_planner_node::GetTwoArmPlan::Response res;

  ph.param("raise_torso", raise_torso, false);
  ph.param<std::string>("attached_object_filename",attached_object, "");
  ph.param<std::string>("known_objects_filename",known_objects, "");
  ph.param<std::string>("remove_current_object",remove_object, "");
 
  ph.param<double>("start_x", req.start.pose.position.x, 0.0);
  ph.param<double>("start_y", req.start.pose.position.y, 0.0);
  ph.param<double>("start_z", req.start.pose.position.z, 0.0);
  ph.param<double>("start_roll",start_roll,0.0);
  ph.param<double>("start_pitch",start_pitch,0.0);
  ph.param<double>("start_yaw",start_yaw,0.0);
  ph.param("go_to_start",go_to_start,true);

  ph.param<double>("goal_x", req.goal.pose.position.x, 0.0);
  ph.param<double>("goal_y", req.goal.pose.position.y, 0.0);
  ph.param<double>("goal_z", req.goal.pose.position.z, 0.0);
  ph.param<double>("goal_roll",goal_roll,0.0);
  ph.param<double>("goal_pitch",goal_pitch,0.0);
  ph.param<double>("goal_yaw",goal_yaw,0.0);
  ph.param("plan_path",plan_path,true);
  ph.param("print_path",print_path,false);
  ph.param("filter_trajectory",filter_traj,true);
  ph.param("execute_plan",execute_plan,true);
  ph.param("setup_experiment",setup_experiment,true);
  ph.param("add_collision_objects",add_collision_objects,true);

  ph.param<double>("object_offset_x", object_offset[0], 0.0);
  ph.param<double>("object_offset_y", object_offset[1], 0.0);
  ph.param<double>("object_offset_z", object_offset[2], 0.0);

  req.absolute_xyzrpy_tolerance.resize(6,0.0);
  ph.param<double>("tolerance_x",req.absolute_xyzrpy_tolerance[0], 0.025);
  ph.param<double>("tolerance_y",req.absolute_xyzrpy_tolerance[1], 0.025);
  ph.param<double>("tolerance_z",req.absolute_xyzrpy_tolerance[2], 0.025);
  ph.param<double>("tolerance_roll",req.absolute_xyzrpy_tolerance[3],0.1);
  ph.param<double>("tolerance_pitch",req.absolute_xyzrpy_tolerance[4],0.1);
  ph.param<double>("tolerance_yaw",req.absolute_xyzrpy_tolerance[5],0.1);

  ph.param<double>("right_arm_pose_on_object_x", rarm_object.position.x, 0.0);
  ph.param<double>("right_arm_pose_on_object_y", rarm_object.position.y, -0.15);
  ph.param<double>("right_arm_pose_on_object_z", rarm_object.position.z, 0.0);
  ph.param<double>("right_arm_pose_on_object_roll", rarm_object_roll, 0.0);
  ph.param<double>("right_arm_pose_on_object_pitch", rarm_object_pitch, 0.0);
  ph.param<double>("right_arm_pose_on_object_yaw", rarm_object_yaw, 0.0);
  ph.param<double>("left_arm_pose_on_object_x", larm_object.position.x, 0.0);
  ph.param<double>("left_arm_pose_on_object_y", larm_object.position.y, 0.15);
  ph.param<double>("left_arm_pose_on_object_z", larm_object.position.z, 0.0); 
  ph.param<double>("left_arm_pose_on_object_roll", larm_object_roll, 0.0);
  ph.param<double>("left_arm_pose_on_object_pitch", larm_object_pitch, 0.0);
  ph.param<double>("left_arm_pose_on_object_yaw", larm_object_yaw, 0.0);
  btoffset.setRPY(rarm_object_roll,rarm_object_pitch,rarm_object_yaw);
  tf::quaternionTFToMsg(btoffset,rarm_object.orientation);
  btoffset.setRPY(larm_object_roll,larm_object_pitch,larm_object_yaw);
  tf::quaternionTFToMsg(btoffset,larm_object.orientation);

  //translate right arm pose on object by wrist-finger distance
  //ROS_WARN("IF IT'S NOT PLANNING IT COULD BE CAUSE YOU DIDN'T TRANSLATE THE WRIST.");
   
  ROS_WARN("[move_both_arms] Translating right grip and left grip in object frame by wrist - gripper distance.");
  rarm_object.position.x -= DISTANCE_FROM_WRIST_TO_GRIPPER; 
  larm_object.position.x -= DISTANCE_FROM_WRIST_TO_GRIPPER;
  ROS_WARN("[move_both_arms] For wine tests you need to remove this translation!!!");
  

  /*
  geometry_msgs::Pose mwrist;
  mwrist.position.x = -DISTANCE_FROM_WRIST_TO_GRIPPER;
  mwrist.position.y = 0.0;
  mwrist.position.z = 0.0;
  mwrist.orientation.x = 0.0;
  mwrist.orientation.y = 0.0;
  mwrist.orientation.z = 0.0;
  mwrist.orientation.w = 1.0;

  tf::Pose tright, tleft,twrist;
  geometry_msgs::Pose mright, mleft;

  tf::poseMsgToTF(mwrist,twrist);
  tf::poseMsgToTF(rarm_object,tright);
  tf::poseMsgToTF(larm_object,tleft);

  tright.setOrigin(tright.getOrigin()-twrist.getOrigin());
  tleft.setOrigin(tleft.getOrigin()-twrist.getOrigin());

  tf::poseTFToMsg(tright, rarm_object);
  tf::poseTFToMsg(tleft, larm_object);
*/


  if(setup_experiment)
  {
    //remove collision object from previous test
    if(!remove_object.empty())
      removeObject(remove_object);

    //collision objects
    if(!known_objects.empty() || !add_collision_objects)
    {
      if(!addCollisionObjects(known_objects, object_offset))
      {
        ROS_ERROR("[move_both_arms] Failed to add collision objects from %s.", known_objects.c_str());
        return 0;
      }
    }

    //attach object
    if(!attached_object.empty())
    {
      if(!attachObject(attached_object, rarm_object))
      {
        ROS_INFO("[move_both_arms] Failed to attach the object from %s.", attached_object.c_str());
        return 0;
      }
    }
  }
  else
    ROS_WARN("[move_both_arms] Not setting up experiment. Assuming it's been done previously.");

    if(!go_to_start)
      return 0;

    //adjust torso
    if(raise_torso)
      adjustTorso(1);
    else  
      adjustTorso(0); 
    ROS_INFO("[move_both_arms] Adjusted the torso.");

    btgoal.setRPY(goal_roll,goal_pitch,goal_yaw);
    tf::quaternionTFToMsg(btgoal,req.goal.pose.orientation);
    btstart.setRPY(start_roll,start_pitch,start_yaw);
    tf::quaternionTFToMsg(btstart,req.start.pose.orientation);

    req.goal.header.frame_id = "base_footprint";
    req.start.header.frame_id = "base_footprint";
    req.goal.header.stamp = ros::Time();
    req.start.header.stamp = ros::Time();
    req.rarm_object.pose = rarm_object;
    req.larm_object.pose = larm_object;
/*
    //switch to single arm controllers
    if(!switchArmControllers(1, pr2_mechanism_msgs::SwitchController::Request::BEST_EFFORT))
      ROS_WARN("Failed to switch controllers. Something will break.");

    //move arms into starting position
    ROS_INFO("[move_both_arms] Get state of right arm and left arm.");
    pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr state_msg = ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>("r_arm_controller/state");
    rangles = state_msg->actual.positions;
    state_msg = ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>("l_arm_controller/state");
    langles = state_msg->actual.positions;
    ROS_DEBUG("[move_both_arms] Got the state of the arms.");
*/
    //get state of the arms
    ROS_INFO("[move_both_arms] Get state of both arms from both_arm_controller.");
    pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr state_msg = ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>("both_arms_controller/state");
    for(size_t i = 0; i < state_msg->actual.positions.size()/2; ++i)
    {
      rangles[i] = state_msg->actual.positions[i];
      langles[i] = state_msg->actual.positions[i+7];
    }
    ROS_DEBUG("[move_both_arms] Got the state of the arms.");
/*
    if(!sendBothArmsToPose(req.start, rarm_object, larm_object, rangles, langles))
    {
      ROS_ERROR("[move_both_arms] Failed to send arms to starting poses.");
      return false;
    }
    ROS_INFO("[move_both_arms] Waiting for the arms to get into position.");
*/
    fuckPoopy();

    if(setup_experiment)
      sleep(10);
    else
      sleep(6);


   //call planner
  if(!plan_path)
    return 0;

  ROS_INFO("[move_both_arms] Goal:");
  ROS_INFO("[move_both_arms] position: %0.3f %0.3f %0.3f", req.goal.pose.position.x, req.goal.pose.position.y, req.goal.pose.position.z);
  ROS_INFO("[move_both_arms] orientation: rpy: %0.3f %0.3f %0.3f quat: %0.3f %0.3f %0.3f %0.3f", goal_roll, goal_pitch, goal_yaw, req.goal.pose.orientation.x, req.goal.pose.orientation.y, req.goal.pose.orientation.z, req.goal.pose.orientation.w);

  ROS_INFO("[move_both_arms] Waiting for the planning service...");
  ros::service::waitForService(PLANNING_SERVICE);
  ROS_DEBUG("Communicating with planning service...");
  ros::ServiceClient client = nh.serviceClient<sbpl_two_arm_planner_node::GetTwoArmPlan>(PLANNING_SERVICE, true);

  if(client.call(req, res))
  {
    ROS_INFO("[move_both_arms] Planner returned.");  

    if(res.stats.size() > 18)
    {
      if(res.stats.size() == res.stats_field_names.size())
      { 
        ROS_INFO("\n%50s","-- Planning Statistics --");
        for(size_t i = 0; i < res.stats.size()-18; ++i)
          ROS_INFO("%44s: %0.2f", res.stats_field_names[i].c_str(),res.stats[i]);
        ROS_INFO("\n");

        printf("\n");
        for(size_t i = 0; i < res.stats.size()-18; ++i)
          printf("%0.2f, ", res.stats[i]);
        printf("\n");

        FILE* file = fopen("/tmp/planner_stats.csv", "a");
        if(file != NULL)
        {
          time_t clock;
          time(&clock);
          fprintf(file,"%s, ",ctime(&clock));
          for(size_t i = 0; i < res.stats.size()-18; ++i)
            fprintf(file, "%0.2f, ", res.stats[i]);
          fprintf(file,"\n");
          fclose(file);
        }
      }
    }

    if(res.trajectory.points.empty())
    {
      ROS_ERROR("[move_both_arms] Planner failed.");
      return 0;
    }
  }
  else
  {
    ROS_ERROR("[move_both_arms] Planning service failed to respond. Exiting.");
    return 0;
  }
  
  if(print_path)
  {
    ROS_INFO("\n--Planned Trajectory--");
    printTrajectory(res.trajectory);
  }

  trajectory_msgs::JointTrajectory traj_with_current;
  addCurrentStateToTrajectory(res.trajectory,traj_with_current);

  if(print_path)
  {
    ROS_INFO("\n--Trajectory with Start--");
    printTrajectory(traj_with_current);
  }

  if(!execute_plan)
    return 0;

  sleep(1);

  //filter trajectory
  trajectory_msgs::JointTrajectory filtered_traj;
  if(filter_traj)
  {
    ROS_INFO("[move_both_arms] Filtering trajectory...");
    if(!filterTrajectory(traj_with_current,filtered_traj))
    {
      ROS_ERROR("[move_both_arms] Failed to filter trajectory. Exiting.");
      return 0;
    }

    if(print_path)
    {
      ROS_INFO("\n--Filtered Trajectory--");
      printTrajectory(filtered_traj);
    }
  }
  else
  {
    ROS_INFO("[move_both_arms] Not filtering trajectory...");
    filtered_traj = traj_with_current;
  }


  //execute plan
  if(!sendBothArmsToConfiguration(filtered_traj))
  {
    ROS_WARN("Failed to send arms to a configuration using both_arm_controller");
    return 0;
  }
  sleep(1);
  return 0;  
}

                                                                         
