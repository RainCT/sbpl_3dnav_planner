/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/** \author Benjamin Cohen */

#include <sbpl_two_arm_planner_node/sbpl_two_arm_planner_node.h>
#include <math.h>

clock_t starttime;

using namespace std;
using namespace sbpl_arm_planner;

namespace sbpl_two_arm_planner {

/** Initializers -------------------------------------------------------------*/
SBPLTwoArmPlannerNode::SBPLTwoArmPlannerNode() : node_handle_("~"),collision_map_subscriber_(root_handle_,"collision_map_occ",1), collision_map_filter_(NULL),grid_(NULL),laviz_(NULL),raviz_(NULL)
{
  planner_initialized_ = false;
  forward_search_ = true;
  planning_joint_ = "r_wrist_roll_link";
  allocated_time_ = 10.0;
  env_resolution_ = 0.02;

  langles_.resize(7,0);
  rangles_.resize(7,0);

  ljoint_names_.resize(7);
  rjoint_names_.resize(7);

  rarm_object_offset_.orientation.x = 0.0;
  rarm_object_offset_.orientation.y = 0.0;
  rarm_object_offset_.orientation.z = 0.0;
  rarm_object_offset_.orientation.w = 1.0;
  larm_object_offset_.orientation.x = 0.0;
  larm_object_offset_.orientation.y = 0.0;
  larm_object_offset_.orientation.z = 0.0;
  larm_object_offset_.orientation.w = 1.0;

  rarm_ = new Arm(std::string("right"));
  larm_ = new Arm(std::string("left"));
}

SBPLTwoArmPlannerNode::~SBPLTwoArmPlannerNode()
{
  if(laviz_ != NULL)
    delete laviz_;
  if(raviz_ != NULL)
    delete raviz_;
  if(collision_map_filter_ != NULL)
    delete collision_map_filter_;
  if(planner_ != NULL)
    delete planner_;
  if(rarm_ != NULL)
    delete rarm_;
  if(larm_ != NULL)
    delete larm_;
}

bool SBPLTwoArmPlannerNode::init()
{
  //planner
  node_handle_.param ("planner/search_mode", search_mode_, true); //true: stop after first solution
  node_handle_.param<std::string>("planner/left_arm_description_file", left_arm_description_filename_, "/home/bcohen/ros/sbpl_cartesian_arm_planning/sbpl_cartesian_arm_planning/sbpl_arm_planner/config/pr2_left_arm.cfg");
  node_handle_.param<std::string>("planner/right_arm_description_file", right_arm_description_filename_, "/home/bcohen/ros/sbpl_cartesian_arm_planning/sbpl_cartesian_arm_planning/sbpl_arm_planner/config/pr2_right_arm.cfg");
  node_handle_.param<std::string>("planner/motion_primitive_file", mprims_filename_, "/home/bcohen/ros/sbpl_two_arm_planning/sbpl_two_arm_planning/sbpl_two_arm_planner/config/pr2-26.mprim");
  node_handle_.param ("debug/print_out_path", print_path_, true);
  node_handle_.param ("robot/waypoint_time", waypoint_time_, 0.2);
  node_handle_.param<std::string>("reference_frame", reference_frame_, std::string("base_link"));
  node_handle_.param<std::string>("left_fk_service_name", left_fk_service_name_, "pr2_left_arm_kinematics/get_fk");
  node_handle_.param<std::string>("left_ik_service_name", left_ik_service_name_, "pr2_left_arm_kinematics/get_ik");
  node_handle_.param<std::string>("right_fk_service_name", right_fk_service_name_, "pr2_right_arm_kinematics/get_fk");
  node_handle_.param<std::string>("right_ik_service_name", right_fk_service_name_, "pr2_right_arm_kinematics/get_ik");

  node_handle_.param ("right_arm_pose_on_object_x", rarm_object_offset_.position.x, 0.0);
  node_handle_.param ("right_arm_pose_on_object_y", rarm_object_offset_.position.y, -0.15);
  node_handle_.param ("right_arm_pose_on_object_z", rarm_object_offset_.position.z, 0.0);
  node_handle_.param ("left_arm_pose_on_object_x", larm_object_offset_.position.x, 0.0);
  node_handle_.param ("left_arm_pose_on_object_y", larm_object_offset_.position.y, 0.15);
  node_handle_.param ("left_arm_pose_on_object_z", larm_object_offset_.position.z, 0.0);

  //robot description
  node_handle_.param<std::string>("robot/arm_name", arm_name_, "right_arm");
  std::string robot_urdf_param;
  if(!node_handle_.searchParam("robot_description",robot_urdf_param))
  {
    ROS_ERROR("Unable to find robot description on param server (/robot_description is not set). Exiting");
    return false;
  }
  node_handle_.param<std::string>(robot_urdf_param, robot_description_, "robot_description");
  node_handle_.param ("robot/num_joints", num_joints_, 7);
  
  //pr2 specific
  ljoint_names_[0] = "l_shoulder_pan_joint";
  ljoint_names_[1] = "l_shoulder_lift_joint";
  ljoint_names_[2] = "l_upper_arm_roll_joint";
  ljoint_names_[3] = "l_elbow_flex_joint";
  ljoint_names_[4] = "l_forearm_roll_joint";
  ljoint_names_[5] = "l_wrist_flex_joint";
  ljoint_names_[6] = "l_wrist_roll_joint";

  rjoint_names_[0] = "r_shoulder_pan_joint";
  rjoint_names_[1] = "r_shoulder_lift_joint";
  rjoint_names_[2] = "r_upper_arm_roll_joint";
  rjoint_names_[3] = "r_elbow_flex_joint";
  rjoint_names_[4] = "r_forearm_roll_joint";
  rjoint_names_[5] = "r_wrist_flex_joint";
  rjoint_names_[6] = "r_wrist_roll_joint";
 
  //collision space
  node_handle_.param<std::string>("collision_space/collision_map_topic", collision_map_topic_, "collision_map_occ");

  //visualizations
  node_handle_.param ("visualizations/goal", visualize_goal_, true);
  node_handle_.param ("visualizations/expanded_states",visualize_expanded_states_,true);
  node_handle_.param ("visualizations/heuristic", visualize_heuristic_, true);
  node_handle_.param ("visualizations/voxel_size", env_resolution_, 0.02);
  node_handle_.param ("visualizations/trajectory", visualize_trajectory_, false);
  node_handle_.param ("visualizations/collision_model_trajectory", visualize_collision_model_trajectory_, false);
  node_handle_.param ("visualizations/trajectory_throttle", throttle_, 4);

  map_frame_ = "base_footprint";

  //initialize planner
  if(!initializePlannerAndEnvironment())
    return false;

  collision_map_filter_ = new tf::MessageFilter<mapping_msgs::CollisionMap>(collision_map_subscriber_,tf_,reference_frame_,1);
  collision_map_filter_->registerCallback(boost::bind(&SBPLTwoArmPlannerNode::collisionMapCallback, this, _1));

  joint_states_subscriber_ = root_handle_.subscribe("joint_states", 1, &SBPLTwoArmPlannerNode::jointStatesCallback,this);

  // main planning service
  //planning_service_ = root_handle_.advertiseService("/sbpl_planning/plan_path", &SBPLTwoArmPlannerNode::planKinematicPath,this);
  
  planner_initialized_ = true;

  ROS_INFO("The SBPL arm planner node initialized succesfully.");
  return true;
}

int SBPLTwoArmPlannerNode::run()
{
  ros::spin();
  return 0;
}

bool SBPLTwoArmPlannerNode::initializePlannerAndEnvironment()
{
  planner_ = new ARAPlanner(&sbpl_arm_env_, forward_search_);

  if(robot_description_.empty())
  {
    ROS_ERROR("Robot description file is empty. Exiting.");
    return false;
  }

  //initialize arm planner environment
  if(!sbpl_arm_env_.initEnvironment(right_arm_description_filename_,left_arm_description_filename_,mprims_filename_))
  {
    ROS_ERROR("[sbpl_node] ERROR: initEnvironment failed");
    return false;
  }

  //initialize MDP 
  if(!sbpl_arm_env_.InitializeMDPCfg(&mdp_cfg_))
  {
    ROS_ERROR("[sbpl_node] ERROR: InitializeMDPCfg failed");
    return false;
  }

  cspace_ = sbpl_arm_env_.getCollisionSpace();

  //sad excuse for self-collision checking
  cspace_->addArmCuboidsToGrid(0);
  cspace_->addArmCuboidsToGrid(1);

  grid_ = sbpl_arm_env_.getOccupancyGrid();

  //set epsilon
  planner_->set_initialsolution_eps(sbpl_arm_env_.getEpsilon());

  //set search mode (true - settle with first solution)
  planner_->set_search_mode(search_mode_);

  laviz_ = new sbpl_two_arm_planner::VisualizeArm(std::string("left_arm"));
  raviz_ = new sbpl_two_arm_planner::VisualizeArm(std::string("right_arm"));
  laviz_->setReferenceFrame(reference_frame_);
  raviz_->setReferenceFrame(reference_frame_);

  ROS_INFO("[sbpl_node] Initialized sbpl planning environment.");
  return true;
}

/** Callbacks ----------------------------------------------------------------*/
void SBPLTwoArmPlannerNode::collisionMapCallback(const mapping_msgs::CollisionMapConstPtr &collision_map)
{
  updateMapFromCollisionMap(collision_map);
}

void SBPLTwoArmPlannerNode::updateMapFromCollisionMap(const mapping_msgs::CollisionMapConstPtr &collision_map)
{
  ROS_INFO("collision map callback");
  if(collision_map_mutex_.try_lock())
  {
    ROS_INFO("collision map callback got mutex");
    if(collision_map->header.frame_id.compare(reference_frame_) != 0)
    {
      ROS_WARN("collision_map_occ is in %s not in %s", collision_map->header.frame_id.c_str(), reference_frame_.c_str());
      ROS_DEBUG("the collision map has %i cubic obstacles", int(collision_map->boxes.size()));
    }

    // add collision map msg
    grid_->updateFromCollisionMap(*collision_map);

    // add self collision blocks
    cspace_->addArmCuboidsToGrid(0);
    cspace_->addArmCuboidsToGrid(1);

    map_frame_ = collision_map->header.frame_id; 
    setArmToMapTransform(map_frame_);

    grid_->visualize();
    collision_map_mutex_.unlock();
  }
  return;
}

void SBPLTwoArmPlannerNode::jointStatesCallback(const sensor_msgs::JointStateConstPtr &state)
{
  ROS_INFO("joint states callback");
  if(joint_states_mutex_.try_lock())
  {
    ROS_INFO("joint states callback got mutex");
    rangles_[0] = state->position[17];
    rangles_[1] = state->position[18];
    rangles_[2] = state->position[16];
    rangles_[3] = state->position[20];
    rangles_[4] = state->position[19];
    rangles_[5] = state->position[21];
    rangles_[6] = state->position[22];

    langles_[0] = state->position[29];
    langles_[1] = state->position[30];
    langles_[2] = state->position[28];
    langles_[3] = state->position[32];
    langles_[4] = state->position[31];
    langles_[5] = state->position[33];
    langles_[6] = state->position[34];
    /*
       printf("\n");
       printf("right arm: ");
       for(size_t i = 0; i < 7; ++i)
       printf("%0.3f ", rangles_[i]);
       printf("\n");

       printf("left arm:  ");
       for(size_t i = 0; i < 7; ++i)
       printf("%0.3f ", langles_[i]);
       printf("\n");
    */                                                                            
    joint_states_mutex_.unlock(); 
  }
}
 
/** Planner Interface  -------------------------------------------------------*/
bool SBPLTwoArmPlannerNode::setStart(geometry_msgs::Pose start)
{
  double roll,pitch,yaw;
  tf::Pose tstart;
  //std::vector<std::vector<double> > xyz;
  std::vector<double> vstart(6,0);;

  ROS_INFO("start0: %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f", rangles_[0],rangles_[1],rangles_[2],rangles_[3],rangles_[4],rangles_[5],rangles_[6]);
  ROS_INFO("start1: %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f", langles_[0],langles_[1],langles_[2],langles_[3],langles_[4],langles_[5],langles_[6]);

  vstart[0] = start.position.x;
  vstart[1] = start.position.y;
  vstart[2] = start.position.z;

  tf::poseMsgToTF(start, tstart);
  tstart.getBasis().getRPY(roll,pitch,yaw);
  vstart[3] = roll;
  vstart[4] = pitch;
  vstart[5] = yaw;

  if(sbpl_arm_env_.setStartConfiguration(vstart, rangles_,langles_) == 0)
  {
    ROS_ERROR("[sbpl_node] Environment failed to set start state. Not Planning.\n");
    return false;
  }

  if(planner_->set_start(mdp_cfg_.startstateid) == 0)
  {
    ROS_ERROR("Failed to set start state. Not Planning.");
    return false;
  }

  return true;
}

bool SBPLTwoArmPlannerNode::setGoalPosition(geometry_msgs::Pose goal)
{
  double roll,pitch,yaw;
  geometry_msgs::Pose pgoal;
  tf::Pose tf_pose;
  std::vector <std::vector <double> > sbpl_goal(1, std::vector<double> (11,0));  //Changed to include Quaternion
  std::vector <std::vector <double> > sbpl_tolerance(1, std::vector<double> (12,0));

  //currently only supports one goal
  sbpl_goal[0][0] = goal.position.x;
  sbpl_goal[0][1] = goal.position.y;
  sbpl_goal[0][2] = goal.position.z;

  //perturb quaternion to prevent gimbal lock when using grasping pipeline
  pgoal = goal;
  pgoal.orientation.w += 0.005;

  tf::poseMsgToTF(pgoal, tf_pose);
  tf_pose.getBasis().getRPY(roll,pitch,yaw);
  sbpl_goal[0][3] = roll;
  sbpl_goal[0][4] = pitch;
  sbpl_goal[0][5] = yaw;

  //6dof goal: true, 3dof: false 
  sbpl_goal[0][6] = true;
 
  //orientation constraint as a quaternion 
  sbpl_goal[0][7] = goal.orientation.x;
  sbpl_goal[0][8] = goal.orientation.y;
  sbpl_goal[0][9] = goal.orientation.z;
  sbpl_goal[0][10] = goal.orientation.w;

  //allowable tolerance from goal
  sbpl_tolerance[0][0] = 0.04;
  sbpl_tolerance[0][1] = 0.04;
  sbpl_tolerance[0][2] = 0.04;
  sbpl_tolerance[0][3] = 0.10;
  sbpl_tolerance[0][4] = 0.10;
  sbpl_tolerance[0][5] = 0.10;

  ROS_INFO("goal xyz(%s): %.3f %.3f %.3f (tol: %.3fm) rpy: %.3f %.3f %.3f (tol: %.3frad)", map_frame_.c_str(),sbpl_goal[0][0],sbpl_goal[0][1],sbpl_goal[0][2],sbpl_tolerance[0][0],sbpl_goal[0][3],sbpl_goal[0][4],sbpl_goal[0][5], sbpl_tolerance[0][1]);

  KDL::Frame rarm_offset, larm_offset;
  rarm_offset.p.x(rarm_object_offset_.position.x);
  rarm_offset.p.y(rarm_object_offset_.position.y);
  rarm_offset.p.z(rarm_object_offset_.position.z);
  larm_offset.p.x(larm_object_offset_.position.x);
  larm_offset.p.y(larm_object_offset_.position.y);
  larm_offset.p.z(larm_object_offset_.position.z);

  rarm_offset.M = KDL::Rotation::Quaternion(rarm_object_offset_.orientation.x, rarm_object_offset_.orientation.y, rarm_object_offset_.orientation.z, rarm_object_offset_.orientation.w);
  larm_offset.M = KDL::Rotation::Quaternion(larm_object_offset_.orientation.x, larm_object_offset_.orientation.y, larm_object_offset_.orientation.z, larm_object_offset_.orientation.w);

  //set sbpl environment goal
  if(!sbpl_arm_env_.setGoalPosition(sbpl_goal, sbpl_tolerance, rarm_offset, larm_offset))
  {
    ROS_ERROR("Failed to set goal state. Perhaps goal position is out of reach. Exiting.");
    return false;
  }

  //set planner goal	
  if(planner_->set_goal(mdp_cfg_.goalstateid) == 0)
  {
    ROS_ERROR("Failed to set goal state. Exiting.");
    return false;
  }
  return true;
}

bool SBPLTwoArmPlannerNode::planToPosition(geometry_msgs::Pose start, geometry_msgs::Pose goal)
{
  unsigned int i;
  int nind = 0;
  std::vector<trajectory_msgs::JointTrajectoryPoint> arm_path;

  starttime = clock();

  ROS_INFO("[planToPosition] About to set start configuration");

  //set start
  if(!setStart(start))
  {
    ROS_ERROR("Failed to set the starting configuration.");
    return false;
  }

  //set goal
  if(!setGoalPosition(goal))
  {
    ROS_ERROR("Failed to set the goal pose.");
    return false;
  }

  if(visualize_goal_)
    visualizeGoal(goal);

  //plan a path
  if(plan(arm_path))
  {
    if(print_path_)
      printPath(arm_path);

    // compute distance to goal
    /*
    if(!isGoalConstraintSatisfied(arm_path[arm_path.size()-1].positions, req.motion_plan_request.goal_constraints))
      ROS_WARN("Uh Oh. Goal constraint isn't satisfied.");
    */

    // visualizations
    if(visualize_expanded_states_)
      displayARAStarStates();

    if(visualize_heuristic_)
      displayShortestPath();
    
    visualizeEndEffectorPath();
  }
  else
  {
    ROS_ERROR("Failed to plan within alotted time frame (%0.2f seconds).", allocated_time_);

    if(visualize_expanded_states_)
      displayARAStarStates();

    if(visualize_heuristic_)
      displayShortestPath();
  }

  return false;
}

bool SBPLTwoArmPlannerNode::plan(std::vector<trajectory_msgs::JointTrajectoryPoint> &arm_path)
{
  bool b_ret(false);
  int solution_cost;
  std::vector<double>angles(num_joints_,0);

  collision_map_mutex_.lock();
  joint_states_mutex_.lock();

  ROS_INFO("\n[plan] Calling planner");
  //reinitialize the search space
  planner_->force_planning_from_scratch();

  //plan
  b_ret = planner_->replan(allocated_time_, &solution_state_ids_, &solution_cost);

  //check if an empty plan was received.
  if(b_ret && solution_state_ids_.size() <= 0)
    b_ret = false;

  // if a path is returned, then pack it into msg form
  if(b_ret && (solution_state_ids_.size() > 0))
  {
    std::vector<std::vector<double> > angles_path;
    sbpl_arm_env_.convertStateIDPathToJointAnglesPath(solution_state_ids_,angles_path);
    ROS_INFO("SHORTENED VERSION OF THE PATH");
    sbpl_arm_env_.convertStateIDPathToShortenedJointAnglesPath(solution_state_ids_short_,angles_path, solution_state_ids_short_);
    
    ROS_DEBUG("[plan] A path was returned with %d waypoints.", int(solution_state_ids_.size()));
    ROS_INFO("Initial Epsilon: %0.3f  Final Epsilon: %0.3f Solution Cost: %d", planner_->get_initial_eps(),planner_->get_final_epsilon(), solution_cost);
    ROS_INFO("Original Path Length: %d   Shortened Path Length: %d", int(solution_state_ids_.size()), int(solution_state_ids_short_.size()));

    arm_path.resize(angles_path.size());
    for(size_t i=0; i < angles_path.size(); i++)
    {       
      arm_path[i].positions.resize(num_joints_);
      
      for (int p = 0; p < num_joints_; ++p)
        arm_path[i].positions[p] = angles_path[i][p];
      
      ROS_INFO("%i: %.4f %.4f %.4f %.4f %.4f %.4f %.4f", int(i), arm_path[i].positions[0],arm_path[i].positions[1],arm_path[i].positions[2],arm_path[i].positions[3],arm_path[i].positions[4],arm_path[i].positions[5],arm_path[i].positions[6]);
    }
  }

  collision_map_mutex_.unlock();
  joint_states_mutex_.unlock();

  return b_ret;
}

bool SBPLTwoArmPlannerNode::isGoalConstraintSatisfied(const std::vector<double> &angles, const motion_planning_msgs::Constraints &goal)
{
  bool satisfied = true;
  geometry_msgs::Pose pose, err;

  if(!computeFK(angles,"right_arm", pose))
  {
    ROS_ERROR("Failed to check if goal constraint is satisfied because the FK service failed.");
    return false;
  }

  if(goal.position_constraints.size() > 0)
  {
    err.position.x = fabs(pose.position.x - goal.position_constraints[0].position.x);
    err.position.y = fabs(pose.position.y - goal.position_constraints[0].position.y);
    err.position.z = fabs(pose.position.z - goal.position_constraints[0].position.z);
  }

  if(goal.orientation_constraints.size() > 0)
  {
    err.orientation.x = fabs(pose.orientation.x - goal.orientation_constraints[0].orientation.x);
    err.orientation.y = fabs(pose.orientation.y - goal.orientation_constraints[0].orientation.y);
    err.orientation.z = fabs(pose.orientation.z - goal.orientation_constraints[0].orientation.z);
    err.orientation.w = fabs(pose.orientation.w - goal.orientation_constraints[0].orientation.w);
  }

  ROS_INFO(" ");
  ROS_INFO("Pose:  xyz: %0.4f %0.4f %0.4f  quat: %0.4f %0.4f %0.4f %0.4f", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  ROS_INFO("Goal:  xyz: %0.4f %0.4f %0.4f  quat: %0.4f %0.4f %0.4f %0.4f", goal.position_constraints[0].position.x, goal.position_constraints[0].position.y, goal.position_constraints[0].position.z, goal.orientation_constraints[0].orientation.x, goal.orientation_constraints[0].orientation.y, goal.orientation_constraints[0].orientation.z, goal.orientation_constraints[0].orientation.w);
  ROS_INFO("Error: xyz: %0.4f %0.4f %0.4f  quat: %0.4f %0.4f %0.4f %0.4f", err.position.x, err.position.y, err.position.z, err.orientation.x, err.orientation.y, err.orientation.z, err.orientation.w);
  ROS_INFO(" ");

  if(goal.position_constraints[0].constraint_region_shape.type == geometric_shapes_msgs::Shape::BOX)
  {
    if(goal.position_constraints[0].constraint_region_shape.dimensions.size() < 3)
    { 
      ROS_WARN("Goal constraint region shape is a BOX but fewer than 3 dimensions are defined.");
      return false;
    }
    if(err.position.x >= goal.position_constraints[0].constraint_region_shape.dimensions[0])
    {
      ROS_WARN("X is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.x, goal.position_constraints[0].constraint_region_shape.dimensions[0]);
      satisfied = false;
    }
    if(err.position.y >= goal.position_constraints[0].constraint_region_shape.dimensions[1])
    {
      ROS_WARN("Y is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.y, goal.position_constraints[0].constraint_region_shape.dimensions[1]); 
      satisfied = false;
    }
    if(err.position.z >= goal.position_constraints[0].constraint_region_shape.dimensions[2])
    {
      ROS_WARN("Z is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.z, goal.position_constraints[0].constraint_region_shape.dimensions[2]);
      satisfied = false;
    }
  }
  else if(goal.position_constraints[0].constraint_region_shape.type == geometric_shapes_msgs::Shape::SPHERE)
  {
    if(goal.position_constraints[0].constraint_region_shape.dimensions.size() < 1)
    { 
      ROS_WARN("Goal constraint region shape is a SPHERE but it has no dimensions...");
      return false;
    }
    if(err.position.x >= goal.position_constraints[0].constraint_region_shape.dimensions[0])
    {
      ROS_WARN("X is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.x, goal.position_constraints[0].constraint_region_shape.dimensions[0]);
      satisfied = false;
    }
    if(err.position.y >= goal.position_constraints[0].constraint_region_shape.dimensions[0])
    {
      ROS_WARN("Y is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.y, goal.position_constraints[0].constraint_region_shape.dimensions[1]);
      satisfied = false;
    }
    if(err.position.z >= goal.position_constraints[0].constraint_region_shape.dimensions[0])
    {
      ROS_WARN("Z is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.z, goal.position_constraints[0].constraint_region_shape.dimensions[2]);
      satisfied = false;
    }
  }
  else
    ROS_WARN("Goal constraint region shape is of type %d.", goal.position_constraints[0].constraint_region_shape.type);

  return satisfied;
}


/* Kinematics ----------------------------------------------------------------*/
bool SBPLTwoArmPlannerNode::computeFK(const std::vector<double> &jnt_pos, std::string arm_name, geometry_msgs::Pose &pose)
{
  kinematics_msgs::GetPositionFK::Request  request;
  kinematics_msgs::GetPositionFK::Response response;

  std::string fk_service;
  request.header.stamp = ros::Time();
  request.header.frame_id = reference_frame_;
  request.fk_link_names.resize(1);

  for(size_t j = 0 ; j < jnt_pos.size(); ++j)
    request.robot_state.joint_state.position.push_back(jnt_pos[j]);

  if(arm_name.compare("right_arm") == 0)
  {
    request.robot_state.joint_state.name = rjoint_names_;
    request.fk_link_names[0] = "r_wrist_roll_link";
    fk_service = right_fk_service_name_;
  }
  else if(arm_name.compare("left_arm") == 0)
  {
    request.robot_state.joint_state.name = ljoint_names_;
    request.fk_link_names[0] = "l_wrist_roll_link";
    fk_service = left_fk_service_name_;
  }
  else
  {
    ROS_ERROR("Invalid arm name. Forward kinematics only supports 'right_arm' and 'left_arm'. (%s)", arm_name.c_str());
    return false;
  }

  ROS_DEBUG("waiting for %s service", fk_service.c_str());
  ros::service::waitForService(fk_service);
  ros::ServiceClient client = root_handle_.serviceClient<kinematics_msgs::GetPositionFK>(fk_service);

  if(client.call(request, response))
  {
    if(response.error_code.val == response.error_code.SUCCESS)
    {
      pose = response.pose_stamped[0].pose;
      return true;
    }
    else
      return false;
  }
  else
  {
    ROS_ERROR("FK service failed");
    return false;
  }
}

void SBPLTwoArmPlannerNode::setArmToMapTransform(std::string &map_frame)
{
  std::string fk_root_frame;

  // frame that the sbpl_arm_model is working in
  sbpl_arm_env_.getArmChainRootLinkName(fk_root_frame);

  // get transform to frame that collision map is in
  try
  {
    tf_.lookupTransform(map_frame, fk_root_frame, ros::Time(0), transform_);

    ROS_DEBUG("Received transform from %s to %s (translation: %f %f %f)",fk_root_frame.c_str(),map_frame.c_str(), transform_.getOrigin().x(),transform_.getOrigin().y(),transform_.getOrigin().z());
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }

  // convert transform to a KDL object
  tf::TransformTFToKDL(transform_,kdl_transform_);
  sbpl_arm_env_.setReferenceFrameTransform(kdl_transform_, map_frame);
}

/* Visualizations ------------------------------------------------------------*/
void SBPLTwoArmPlannerNode::displayARAStarStates()
{
  std::vector<std::vector<double> > expanded_states;
  std::vector<double> expanded_color(4,1);
  expanded_color[0] = 0.5;
  expanded_color[2] = 0;

  sbpl_arm_env_.getExpandedStates(expanded_states);

  if(!expanded_states.empty())
  {
    std::vector<std::vector<double> > detailed_color(2);
    detailed_color[0].resize(4,0);
    detailed_color[0][0] = 1;
    detailed_color[0][1] = 0;
    detailed_color[0][2] = 0;
    detailed_color[0][3] = 1;

    detailed_color[1].resize(4,0);
    detailed_color[1][0] = 0;
    detailed_color[1][1] = 1;
    detailed_color[1][2] = 0;
    detailed_color[1][3] = 1;

    laviz_->visualizeDetailedStates(expanded_states, detailed_color,"expanded",0.01);
  }

  ROS_INFO("[displayARAStarStates] displaying %d expanded states",int(expanded_states.size()));
}

void SBPLTwoArmPlannerNode::visualizeGoalPosition(const motion_planning_msgs::Constraints &goal_pose)
{
  geometry_msgs::Pose pose;
  pose.position = goal_pose.position_constraints[0].position;
  pose.orientation = goal_pose.orientation_constraints[0].orientation;
  laviz_->visualizePose(pose, "goal_pose");
  ROS_DEBUG("[visualizeGoalPosition] publishing goal marker visualizations.");
}

void SBPLTwoArmPlannerNode::displayShortestPath()
{
  dpath_ = sbpl_arm_env_.getShortestPath();

  //check if the list is empty
  if(dpath_.empty())
  {
    ROS_INFO("The heuristic path has a length of 0");
    return;
  }
  else
    ROS_DEBUG("Visualizing heuristic path from start to goal with %d waypoints.",int(dpath_.size()));
 
  laviz_->visualizeSpheres(dpath_, 45, "heuristic_path", 0.04);
}

void SBPLTwoArmPlannerNode::visualizeEndEffectorPath()
{
  std::vector<geometry_msgs::Point> points;
  std::vector<std::vector<double> > path, path0, path1;

  sbpl_arm_env_.convertStateIDPathToPoints(solution_state_ids_, path);

  points.resize(path.size());

  for(size_t i = 0; i < path.size(); ++i)
  {
    if(path[i][3] == 1)
      path1.push_back(path[i]);
    else
      path0.push_back(path[i]);

    points[i].x = path[i][0];
    points[i].y = path[i][1];
    points[i].z = path[i][2];
  }

  ROS_INFO("path0: %d, path1: %d", int(path0.size()), int(path1.size()));
  
  laviz_->visualizeSpheres(path0, 230, "end_effector_path0", 0.01);
  laviz_->visualizeSpheres(path1, 30, "end_effector_path1", 0.01);

  laviz_->visualizeLine(points,"end_effector_path", 0, 120, 0.005);
}

void SBPLTwoArmPlannerNode::visualizeGoal(geometry_msgs::Pose goal)
{
  tf::Pose tgoal, tright, tleft;
  geometry_msgs::Pose mright, mleft;

  tf::poseMsgToTF(goal,tgoal);
  tf::poseMsgToTF(rarm_object_offset_,tright);
  tf::poseMsgToTF(larm_object_offset_,tleft);

  tright = tgoal*tright;
  tleft = tgoal*tleft;

  tf::poseTFToMsg(tright, mright);
  tf::poseTFToMsg(tleft, mleft);

  laviz_->visualizePose(goal, "object_goal");
  laviz_->visualizePose(mright, "right_gripper");
  laviz_->visualizePose(mleft, "left_gripper");
}

void SBPLTwoArmPlannerNode::printPath(const std::vector<trajectory_msgs::JointTrajectoryPoint> &arm_path)
{
  double roll,pitch,yaw;
  tf::Pose tf_pose;
  geometry_msgs::Pose pose;
  std::vector<double> jnt_pos(num_joints_,0);

  ROS_INFO("Path:");
  for(unsigned int i = 0; i < arm_path.size(); i++)
  {
    for(int j = 0; j < num_joints_; ++j)
      jnt_pos[j] = arm_path[i].positions[j];

    ROS_INFO("printPath not done yet");
    computeFK(jnt_pos, "right_arm", pose);
    tf::poseMsgToTF(pose, tf_pose);
    tf_pose.getBasis().getRPY(roll,pitch,yaw);

    ROS_INFO("%3d: %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f\n   xyz: %2.3f %2.3f %2.3f  rpy: %0.3f %0.3f %0.3f  quat: %0.2f %0.2f %0.2f %0.2f", i,arm_path[i].positions[0],arm_path[i].positions[1],arm_path[i].positions[2],arm_path[i].positions[3],arm_path[i].positions[4],arm_path[i].positions[5],arm_path[i].positions[6],pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw, pose.orientation.x,pose.orientation.y, pose.orientation.z, pose.orientation.w);
  }
}

void SBPLTwoArmPlannerNode::printPath(FILE* fOut, const std::vector<std::vector<double> > path)
{
  time_t init_time;
  time(&init_time);
  std::string str_time(asctime (localtime(&init_time)));

  fprintf(fOut, "%s", str_time.c_str());
  for(unsigned int i = 0; i < path.size(); i++)
    fprintf(fOut, "state %3d: %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f",i,path[i][0],path[i][1],path[i][2],path[i][3],path[i][4],path[i][5],path[i][6]);
  fprintf(fOut,"---------------------------------");
}

void SBPLTwoArmPlannerNode::sendArmsToStart(geometry_msgs::Pose &object_pose)
{
   tf::Pose tstart, tright, tleft;
  geometry_msgs::Pose mright, mleft;
  geometry_msgs::PoseStamped p;

  tf::poseMsgToTF(object_pose,tstart);
  tf::poseMsgToTF(rarm_object_offset_,tright);
  tf::poseMsgToTF(larm_object_offset_,tleft);

  tright = tstart*tright;
  tleft = tstart*tleft;

  tf::poseTFToMsg(tright, mright);
  tf::poseTFToMsg(tleft, mleft);

  ROS_INFO("Sending the right arm to the start position {%0.3f %0.3f %0.3f}.", mright.position.x, mright.position.y, mright.position.z);
  p.pose = mright;
  p.header.frame_id = reference_frame_;
  p.header.stamp = ros::Time::now();
  rarm_->sendArmToPose(p, rangles_, 2.0);

  ROS_INFO("Sending the left arm to the start position {%0.3f %0.3f %0.3f}.", mleft.position.x, mleft.position.y, mleft.position.z);
  p.pose = mleft;
  p.header.stamp = ros::Time::now();
  larm_->sendArmToPose(p, langles_, 2.0);

  ROS_INFO("Arms should now be at the start configurations.");
}


}

/* Node
 * ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbpl_two_arm_planner");
  sbpl_two_arm_planner::SBPLTwoArmPlannerNode arm_planner;
  if(!arm_planner.init())
  {
    ROS_ERROR("Failed to initialize arm planner node. Exiting.");
    return 0;
  }

  return arm_planner.run();
}
*/


