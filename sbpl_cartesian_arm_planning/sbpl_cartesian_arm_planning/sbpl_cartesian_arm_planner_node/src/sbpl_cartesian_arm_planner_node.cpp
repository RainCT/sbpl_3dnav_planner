/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#include <sbpl_cartesian_arm_planner_node/sbpl_cartesian_arm_planner_node.h>
#include <algorithm>
#include <math.h>

clock_t starttime;

using namespace std;
using namespace sbpl_arm_planner;

namespace sbpl_cartesian_arm_planner {

/** Initializers -------------------------------------------------------------*/
SBPLCartArmPlannerNode::SBPLCartArmPlannerNode() : node_handle_("~"),collision_map_subscriber_(root_handle_,"collision_map_occ",1), collision_map_filter_(NULL),jnt_to_pose_solver_(NULL),grid_(NULL),aviz_(NULL)
{
  planner_initialized_ = false;
  attached_object_ = false;
  forward_search_ = true;
  planning_joint_ = "r_wrist_roll_link";
  attached_object_frame_ = "r_gripper_r_finger_tip_link";
  allocated_time_ = 10.0;
  env_resolution_ = 0.02;
}

SBPLCartArmPlannerNode::~SBPLCartArmPlannerNode()
{
  if(aviz_ != NULL)
    delete aviz_;
  if(collision_map_filter_ != NULL)
    delete collision_map_filter_;
  if(jnt_to_pose_solver_ != NULL)
    delete jnt_to_pose_solver_;
  if(planner_ != NULL)
    delete planner_;
}

bool SBPLCartArmPlannerNode::init()
{
  //planner
  node_handle_.param ("planner/search_mode", search_mode_, true); //true: stop after first solution
  node_handle_.param<std::string>("planner/arm_description_file", arm_description_filename_, "");
  node_handle_.param<std::string>("planner/motion_primitive_file", mprims_filename_, "");
  node_handle_.param ("debug/print_out_path", print_path_, true);
  node_handle_.param ("robot/waypoint_time", waypoint_time_, 0.2);
  node_handle_.param<std::string>("reference_frame", reference_frame_, std::string("base_link"));
  node_handle_.param<std::string>("fk_service_name", fk_service_name_, "pr2_right_arm_kinematics/get_fk");
  node_handle_.param<std::string>("ik_service_name", ik_service_name_, "pr2_right_arm_kinematics/get_ik");

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
  
  joint_names_.resize(num_joints_);
  if(arm_name_ == "left_arm")
  {
    side_ = "l";
    planning_joint_ = "l_wrist_roll_link";
    attached_object_frame_ = "l_gripper_r_finger_tip_link";
  }
  else
  {
    side_ = "r";
    planning_joint_ = "r_wrist_roll_link";
    attached_object_frame_ = "r_gripper_r_finger_tip_link";
  }

  //pr2 specific
  joint_names_[0] = side_ + "_shoulder_pan_joint";
  joint_names_[1] = side_ + "_shoulder_lift_joint";
  joint_names_[2] = side_ + "_upper_arm_roll_joint";
  joint_names_[3] = side_ + "_elbow_flex_joint";
  joint_names_[4] = side_ + "_forearm_roll_joint";
  joint_names_[5] = side_ + "_wrist_flex_joint";
  joint_names_[6] = side_ + "_wrist_roll_joint";
  
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

  map_frame_ = "base_link";

  //initialize planner
  if(!initializePlannerAndEnvironment())
    return false;

  collision_map_filter_ = new tf::MessageFilter<mapping_msgs::CollisionMap>(collision_map_subscriber_,tf_,reference_frame_,1);
  collision_map_filter_->registerCallback(boost::bind(&SBPLCartArmPlannerNode::collisionMapCallback, this, _1));

  collision_object_subscriber_ = root_handle_.subscribe("collision_object", 3, &SBPLCartArmPlannerNode::collisionObjectCallback, this);
  object_subscriber_ = root_handle_.subscribe("attached_collision_object", 3, &SBPLCartArmPlannerNode::attachedObjectCallback,this);

  // main planning service
  planning_service_ = root_handle_.advertiseService("/sbpl_planning/plan_path", &SBPLCartArmPlannerNode::planKinematicPath,this);
  
  planner_initialized_ = true;

  ROS_INFO("The SBPL arm planner node initialized succesfully.");
  return true;
}

int SBPLCartArmPlannerNode::run()
{
  ros::spin();
  return 0;
}

bool SBPLCartArmPlannerNode::initializePlannerAndEnvironment()
{
  planner_ = new ARAPlanner(&sbpl_arm_env_, forward_search_);

  if(robot_description_.empty())
  {
    ROS_ERROR("Robot description file is empty. Exiting.");
    return false;
  }

  //initialize arm planner environment
  if(!sbpl_arm_env_.initEnvironment(arm_description_filename_,mprims_filename_))
  {
    ROS_ERROR("ERROR: initEnvironment failed");
    return false;
  }

  //initialize MDP 
  if(!sbpl_arm_env_.InitializeMDPCfg(&mdp_cfg_))
  {
    ROS_ERROR("ERROR: InitializeMDPCfg failed");
    return false;
  }

  cspace_ = sbpl_arm_env_.getCollisionSpace();

  //sad excuse for self-collision checking
  cspace_->addArmCuboidsToGrid();

  grid_ = sbpl_arm_env_.getOccupancyGrid();

  //set epsilon
  planner_->set_initialsolution_eps(sbpl_arm_env_.getEpsilon());

  //set search mode (true - settle with first solution)
  planner_->set_search_mode(search_mode_);

  if(!initChain(robot_description_))
  {
    ROS_ERROR("Unable to initialize KDL chain.");
    return false;
  }

  aviz_ = new VisualizeArm(arm_name_);
  aviz_->setReferenceFrame(reference_frame_);

  ROS_INFO("Initialized sbpl planning environment.");
  return true;
}

/** Callbacks ----------------------------------------------------------------*/
void SBPLCartArmPlannerNode::collisionMapCallback(const mapping_msgs::CollisionMapConstPtr &collision_map)
{
  updateMapFromCollisionMap(collision_map);
}

void SBPLCartArmPlannerNode::updateMapFromCollisionMap(const mapping_msgs::CollisionMapConstPtr &collision_map)
{
  ROS_DEBUG("[updateMapFromCollisionMap] trying to get colmap_mutex_");
  if(colmap_mutex_.try_lock())
  {
    ROS_DEBUG("[updateMapFromCollisionMap] locked colmap_mutex_");

    if(collision_map->header.frame_id.compare(reference_frame_) != 0)
    {
      ROS_WARN("collision_map_occ is in %s not in %s", collision_map->header.frame_id.c_str(), reference_frame_.c_str());
      ROS_DEBUG("the collision map has %i cubic obstacles", int(collision_map->boxes.size()));
    }

    // add collision map msg
    grid_->updateFromCollisionMap(*collision_map);

    // add self collision blocks
    cspace_->addArmCuboidsToGrid();
  
    cspace_->putCollisionObjectsInGrid();

    map_frame_ = collision_map->header.frame_id; 
    setArmToMapTransform(map_frame_);

    colmap_mutex_.unlock();
    ROS_DEBUG("[updateMapFromCollisionMap] released colmap_mutex_ mutex.");

    visualizeCollisionObjects();

    grid_->visualize();
    return;
  }
  else
  {
    ROS_DEBUG("[updateMapFromCollisionMap] failed trying to get colmap_mutex_ mutex");
    return;
  }
}

void SBPLCartArmPlannerNode::attachedObjectCallback(const mapping_msgs::AttachedCollisionObjectConstPtr &attached_object)
{
  if(object_mutex_.try_lock())
  {
    // remove all objects
    if(attached_object->link_name.compare(mapping_msgs::AttachedCollisionObject::REMOVE_ALL_ATTACHED_OBJECTS) == 0 &&
        attached_object->object.operation.operation == mapping_msgs::CollisionObjectOperation::REMOVE)
    {
      ROS_DEBUG("[attachedObjectCallback] Removing all attached objects.");
      attached_object_ = false;
      cspace_->removeAttachedObject();
    }
    // add object
    else if(attached_object->object.operation.operation == mapping_msgs::CollisionObjectOperation::ADD)
    {
      ROS_DEBUG("[attachedObjectCallback] Received a message to ADD an object (%s) with %d shapes.", attached_object->object.id.c_str(), int(attached_object->object.shapes.size()));
      attachObject(attached_object->object);
    }
    // attach object and remove it from collision space
    else if( attached_object->object.operation.operation == mapping_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT)
    {
      ROS_DEBUG("[attachedObjectCallback] Received a message to ATTACH_AND_REMOVE_AS_OBJECT of object: %s", attached_object->object.id.c_str());
      
      // have we seen this collision object before?
      if(object_map_.find(attached_object->object.id) != object_map_.end())
      {
        ROS_DEBUG("[attachedObjectCallback] We have seen this object (%s) before.", attached_object->object.id.c_str());
        attachObject(object_map_.find(attached_object->object.id)->second);
      }
      else
      {
        ROS_DEBUG("[attachedObjectCallback] We have NOT seen this object (%s) before.", attached_object->object.id.c_str());
        object_map_[attached_object->object.id] = attached_object->object;
        attachObject(attached_object->object);
      }
      cspace_->removeCollisionObject(attached_object->object);
    }
    // remove object
    else if(attached_object->object.operation.operation == mapping_msgs::CollisionObjectOperation::REMOVE)
    {
      attached_object_ = false;
      ROS_DEBUG("[attachedObjectCallback] Removing object (%s) from gripper.", attached_object->object.id.c_str());
      cspace_->removeAttachedObject();
    }
    else if(attached_object->object.operation.operation == mapping_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT)
    {
      attached_object_ = false;
      ROS_DEBUG("[attachedObjectCallback] Removing object (%s) from gripper and adding to collision map.", attached_object->object.id.c_str());
      cspace_->removeAttachedObject();
      cspace_->addCollisionObject(attached_object->object);
    }
    else
      ROS_WARN("Received a collision object with an unknown operation");

    object_mutex_.unlock();
  }
  visualizeCollisionObjects();
}

void SBPLCartArmPlannerNode::collisionObjectCallback(const mapping_msgs::CollisionObjectConstPtr &collision_object)
{
  // for some reason, it wasn't getting all of the 'all' messages...
  if(collision_object->id.compare("all") == 0)
    cspace_->removeAllCollisionObjects();

  if(object_mutex_.try_lock())
  {
    // debug: have we seen this collision object before?
    if(object_map_.find(collision_object->id) != object_map_.end())
      ROS_DEBUG("[collisionObjectCallback] We have seen this object ('%s')  before.", collision_object->id.c_str());
    else
      ROS_DEBUG("[collisionObjectCallback] We have NOT seen this object ('%s') before.", collision_object->id.c_str());
    object_map_[collision_object->id] = (*collision_object);
    object_mutex_.unlock();
  }

  ROS_DEBUG("[collisionObjectCallback] %s", collision_object->id.c_str());
  cspace_->processCollisionObjectMsg((*collision_object));

  visualizeCollisionObjects();
}

void SBPLCartArmPlannerNode::attachObject(const mapping_msgs::CollisionObject &obj)
{
  geometry_msgs::PoseStamped pose_in, pose_out;
  mapping_msgs::CollisionObject object(obj);

  attached_object_ = true;

  ROS_INFO("Received a collision object message with %d shapes.", int(object.shapes.size()));

  for(size_t i = 0; i < object.shapes.size(); i++)
  {
    pose_in.header = object.header;
    pose_in.header.stamp = ros::Time();
    pose_in.pose = object.poses[i];
    tf_.transformPose(attached_object_frame_, pose_in, pose_out);
    object.poses[i] = pose_out.pose;
    ROS_DEBUG("[attachObject] Converted shape from %s (%0.2f %0.2f %0.2f) to %s (%0.3f %0.3f %0.3f)", pose_in.header.frame_id.c_str(), pose_in.pose.position.x, pose_in.pose.position.y, pose_in.pose.position.z, attached_object_frame_.c_str(), pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z);

    if(object.shapes[i].type == geometric_shapes_msgs::Shape::SPHERE)
    {
      ROS_INFO("Attaching a sphere with radius: %0.3fm", object.shapes[i].dimensions[0]);
      cspace_->attachSphereToGripper(object.header.frame_id, object.poses[i], object.shapes[i].dimensions[0]);
    }
    else if(object.shapes[i].type == geometric_shapes_msgs::Shape::CYLINDER)
    {
      ROS_INFO("Attaching a cylinder with radius: %0.3fm & length %0.3fm", object.shapes[i].dimensions[0], object.shapes[i].dimensions[1]);

      cspace_->attachCylinderToGripper(object.header.frame_id, object.poses[i], object.shapes[i].dimensions[0], object.shapes[i].dimensions[1]);
    }
    else if(object.shapes[i].type == geometric_shapes_msgs::Shape::MESH)
    {
      ROS_INFO("Attaching a mesh with %d triangles  & %d vertices.", int(object.shapes[i].triangles.size()/3), int(object.shapes[i].vertices.size()));

      cspace_->attachMeshToGripper(object.header.frame_id, object.poses[i], object.shapes[i].triangles, object.shapes[i].vertices);
    }
    else if(object.shapes[i].type == geometric_shapes_msgs::Shape::BOX)
    {
      std::vector<double> dims(object.shapes[i].dimensions);
      sort(dims.begin(),dims.end());
      ROS_INFO("Attaching a box as a cylinder with length: %0.3fm   radius: %0.3fm", dims[2], dims[1]);
      cspace_->attachCylinderToGripper(object.header.frame_id, object.poses[i], dims[1], dims[2]);
    }
    else
      ROS_WARN("Currently attaching objects of type '%d' aren't supported.", object.shapes[i].type);
  }
}

/** Planner Interface  -------------------------------------------------------*/
bool SBPLCartArmPlannerNode::setStart(const sensor_msgs::JointState &start_state)
{
  std::vector<double> sbpl_start(start_state.position.size(),0);

  for(unsigned int i=0; i< start_state.position.size(); i++)
    sbpl_start[i] = (double)(start_state.position[i]);

  std::vector<std::vector<double> > xyz;

  ROS_INFO("start: %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f", sbpl_start[0],sbpl_start[1],sbpl_start[2],sbpl_start[3],sbpl_start[4],sbpl_start[5],sbpl_start[6]);

  if(sbpl_arm_env_.setStartConfiguration(sbpl_start) == 0)
  {
    ROS_ERROR("Environment failed to set start state. Not Planning.\n");
    return false;
  }

  if(planner_->set_start(mdp_cfg_.startstateid) == 0)
  {
    ROS_ERROR("Failed to set start state. Not Planning.");
    return false;
  }

  if(attached_object_)
    visualizeAttachedObject(sbpl_start);

  return true;
}

bool SBPLCartArmPlannerNode::setGoalPosition(const motion_planning_msgs::Constraints &goals)
{
  double roll,pitch,yaw;
  geometry_msgs::Pose pose_msg;
  tf::Pose tf_pose;
  std::vector <std::vector <double> > sbpl_goal(1, std::vector<double> (11,0));  //Changed to include Quaternion
  std::vector <std::vector <double> > sbpl_tolerance(1, std::vector<double> (12,0));

  if(goals.position_constraints.size() != goals.orientation_constraints.size())
    ROS_WARN("There are %d position contraints and %d orientation constraints.", int(goals.position_constraints.size()),int(goals.orientation_constraints.size()));

  //currently only supports one goal
  sbpl_goal[0][0] = goals.position_constraints[0].position.x;
  sbpl_goal[0][1] = goals.position_constraints[0].position.y;
  sbpl_goal[0][2] = goals.position_constraints[0].position.z;

  //convert quaternion into roll,pitch,yaw
  pose_msg.position = goals.position_constraints[0].position;
  pose_msg.orientation = goals.orientation_constraints[0].orientation;

  //perturb quaternion if rpy will suffer from gimbal lock
  //if(pose_msg.orientation.x == 0 && pose_msg.orientation.z == 0)
  pose_msg.orientation.w += 0.005;

  tf::poseMsgToTF(pose_msg, tf_pose);
  tf_pose.getBasis().getRPY(roll,pitch,yaw);
  sbpl_goal[0][3] = roll;
  sbpl_goal[0][4] = pitch;
  sbpl_goal[0][5] = yaw;

  //6dof goal: true, 3dof: false 
  sbpl_goal[0][6] = true;
 
  //orientation constraint as a quaternion 
  sbpl_goal[0][7] = goals.orientation_constraints[0].orientation.x;
  sbpl_goal[0][8] = goals.orientation_constraints[0].orientation.y;
  sbpl_goal[0][9] = goals.orientation_constraints[0].orientation.z;
  sbpl_goal[0][10] = goals.orientation_constraints[0].orientation.w;

  //allowable tolerance from goal
  sbpl_tolerance[0][0] = goals.position_constraints[0].constraint_region_shape.dimensions[0] / 2.0;
  sbpl_tolerance[0][1] = goals.position_constraints[0].constraint_region_shape.dimensions[0] / 2.0;
  sbpl_tolerance[0][2] = goals.position_constraints[0].constraint_region_shape.dimensions[0] / 2.0;
  sbpl_tolerance[0][3] = goals.orientation_constraints[0].absolute_roll_tolerance;
  sbpl_tolerance[0][4] = goals.orientation_constraints[0].absolute_pitch_tolerance;
  sbpl_tolerance[0][5] = goals.orientation_constraints[0].absolute_yaw_tolerance;

  ROS_INFO("goal quat from move_arm: %0.3f %0.3f %0.3f %0.3f", goals.orientation_constraints[0].orientation.x, goals.orientation_constraints[0].orientation.y, goals.orientation_constraints[0].orientation.z, goals.orientation_constraints[0].orientation.w);

  ROS_INFO("goal xyz(%s): %.3f %.3f %.3f (tol: %.3fm) rpy: %.3f %.3f %.3f (tol: %.3frad)", map_frame_.c_str(),sbpl_goal[0][0],sbpl_goal[0][1],sbpl_goal[0][2],sbpl_tolerance[0][0],sbpl_goal[0][3],sbpl_goal[0][4],sbpl_goal[0][5], sbpl_tolerance[0][1]);

  //set sbpl environment goal
  if(!sbpl_arm_env_.setGoalPosition(sbpl_goal, sbpl_tolerance))
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

  tf::Quaternion q;
  q.setRPY(roll,pitch,yaw);

  ROS_DEBUG("Quat from MoveArm: %0.3f %0.3f %0.3f %0.3f", goals.orientation_constraints[0].orientation.x, goals.orientation_constraints[0].orientation.y, goals.orientation_constraints[0].orientation.z, goals.orientation_constraints[0].orientation.w);
  ROS_DEBUG("      RPY with TF: %0.3f %0.3f %0.3f", roll,pitch,yaw);
  ROS_DEBUG("     Quat with TF: %0.3f %0.3f %0.3f %0.3f", q.x(), q.y(), q.z(), q.w());

  return true;
}

bool SBPLCartArmPlannerNode::planToPosition(motion_planning_msgs::GetMotionPlan::Request &req, motion_planning_msgs::GetMotionPlan::Response &res)
{
  unsigned int i;
  int nind = 0;
  std::vector<trajectory_msgs::JointTrajectoryPoint> arm_path;
  sensor_msgs::JointState start;

  starttime = clock();

  //check for an empty start state
  if(req.motion_plan_request.start_state.joint_state.position.size() <= 0)
  {
    ROS_ERROR("No start state given. Unable to plan.");
    return false;
  }

  //check if goal constraint is empty
  if(req.motion_plan_request.goal_constraints.position_constraints.size() <= 0 || 
      req.motion_plan_request.goal_constraints.orientation_constraints.size() <= 0)
  {
    ROS_ERROR("Position constraint or orientation constraint is empty. Unable to plan.");
    return false;
  }

  //check if there is more than one goal constraint
  if(req.motion_plan_request.goal_constraints.position_constraints.size() > 1 || 
      req.motion_plan_request.goal_constraints.orientation_constraints.size() > 1)
    ROS_WARN("The planning request message contains %d position and %d orientation constraints. Currently the planner only supports one position & orientation constraint pair at a time. Planning to the first goal may not satisfy move_arm.", int(req.motion_plan_request.goal_constraints.position_constraints.size()), int(req.motion_plan_request.goal_constraints.orientation_constraints.size()));

  // add collision objects to occupancy grid
  cspace_->putCollisionObjectsInGrid();

  //check if planning for wrist (only link supported for now)
  planning_joint_ = req.motion_plan_request.goal_constraints.position_constraints[0].link_name;

  if(planning_joint_ != "r_wrist_roll_link" && arm_name_ == "right_arm")
  {
    ROS_ERROR("Planner is configured to plan for the right arm. It has only been tested with pose constraints for r_wrist_roll_link. Other links may be supported in the future.");
    return false;
  }
  else if(planning_joint_ != "l_wrist_roll_link" && arm_name_ == "left_arm")
  {
    ROS_ERROR("Planner is configured to plan for the left arm. It has only been tested with pose constraints for l_wrist_roll_link. Other links may be supported in the future.");
    return false;
  }

  //transform goal pose into reference_frame_
  geometry_msgs::PoseStamped pose_in, pose_out;
  pose_in.header = req.motion_plan_request.goal_constraints.position_constraints[0].header;
  pose_in.header.stamp = ros::Time();
  pose_in.pose.position = req.motion_plan_request.goal_constraints.position_constraints[0].position;
  pose_in.pose.orientation = req.motion_plan_request.goal_constraints.orientation_constraints[0].orientation;

  // TODO: catch the exception
  tf_.transformPose(map_frame_,pose_in,pose_out);

  req.motion_plan_request.goal_constraints.position_constraints[0].position = pose_out.pose.position;
  req.motion_plan_request.goal_constraints.orientation_constraints[0].orientation = pose_out.pose.orientation;

  ROS_DEBUG("[planToPosition] Transformed goal from (%s): %0.3f %0.3f %0.3f to (%s): %0.3f %0.3f %0.3f", req.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id.c_str(),pose_in.pose.position.x, pose_in.pose.position.y, pose_in.pose.position.z, map_frame_.c_str(), pose_out.pose.position.x,pose_out.pose.position.y,pose_out.pose.position.z);

  //get the initial state of the planning joints
  start.position.resize(num_joints_);
  for(i = 0; i < req.motion_plan_request.start_state.joint_state.position.size(); i++)
  {
    if(joint_names_[nind].compare(req.motion_plan_request.start_state.joint_state.name[i]) == 0)
    {
      start.position[nind] = req.motion_plan_request.start_state.joint_state.position[i];
      nind++;
    }
    if(nind == num_joints_)
      break;
  }
  if(nind != num_joints_)
    ROS_WARN("Not all of the expected joints in the arm were assigned a starting position.");

  allocated_time_ = req.motion_plan_request.allowed_planning_time.toSec();

  colmap_mutex_.lock();
  object_mutex_.lock();

  ROS_DEBUG("[planToPosition] About to set start configuration");
  if(setStart(start))
  {
    ROS_DEBUG("[planToPosition] Successfully set starting configuration");

    if(visualize_goal_)
      visualizeGoalPosition(req.motion_plan_request.goal_constraints);

    if(setGoalPosition(req.motion_plan_request.goal_constraints))
    {
      if(plan(arm_path))
      {
        colmap_mutex_.unlock();
        object_mutex_.unlock();

        res.trajectory.joint_trajectory.points.resize(arm_path.size());
        res.trajectory.joint_trajectory.points = arm_path;

        // fill in the waypoint times (not scaled as of now)
        res.trajectory.joint_trajectory.points[0].time_from_start.fromSec(waypoint_time_);
        for(i = 1; i < res.trajectory.joint_trajectory.points.size(); i++)
          res.trajectory.joint_trajectory.points[i].time_from_start.fromSec(res.trajectory.joint_trajectory.points[i-1].time_from_start.toSec() + waypoint_time_);

        res.trajectory.joint_trajectory.header.seq = req.motion_plan_request.goal_constraints.position_constraints[0].header.seq; 
        res.trajectory.joint_trajectory.header.stamp = ros::Time::now();

        if(!req.motion_plan_request.start_state.joint_state.header.frame_id.empty())
          res.trajectory.joint_trajectory.header.frame_id = req.motion_plan_request.start_state.joint_state.header.frame_id;
        else
          res.trajectory.joint_trajectory.header.frame_id = reference_frame_;

        // fill in the joint names 
        res.trajectory.joint_trajectory.joint_names.resize(num_joints_);
        for(i = 0; i < (unsigned int)num_joints_; i++)
          res.trajectory.joint_trajectory.joint_names[i] = joint_names_[i];
  
        ROS_INFO("Planner completed in %lf seconds. Planned trajectory has %d waypoints.",(clock() - starttime) / (double)CLOCKS_PER_SEC, int(res.trajectory.joint_trajectory.points.size()));

        if(print_path_)
          printPath(res.trajectory.joint_trajectory.points);
        
        // compute distance to goal
        if(!isGoalConstraintSatisfied(res.trajectory.joint_trajectory.points[res.trajectory.joint_trajectory.points.size()-1].positions, req.motion_plan_request.goal_constraints))
          ROS_WARN("Uh Oh. Goal constraint isn't satisfied.");
        
        // visualizations
        if(visualize_expanded_states_)
          displayARAStarStates();

        if(visualize_heuristic_)
          displayShortestPath();

        if(visualize_trajectory_)
          aviz_->visualizeJointTrajectoryMsg(res.trajectory.joint_trajectory, throttle_);

        if(visualize_collision_model_trajectory_)
          aviz_->visualizeCollisionModelFromJointTrajectoryMsg(res.trajectory.joint_trajectory, *cspace_, throttle_);

        visualizeEndEffectorPath();

        return true;
      }
      else
      {
        ROS_ERROR("Failed to plan within alotted time frame (%0.2f seconds).", allocated_time_);
        
        if(visualize_expanded_states_)
          displayARAStarStates();

        if(visualize_heuristic_)
          displayShortestPath();
      }
    }
    else
    {
      ROS_ERROR("Failed to set goal pose.");
    }
  }
  else
  {
    ROS_ERROR("Failed to set start configuration.");
  }

  colmap_mutex_.unlock();
  object_mutex_.unlock();

  return false;
}

bool SBPLCartArmPlannerNode::planKinematicPath(motion_planning_msgs::GetMotionPlan::Request &req, motion_planning_msgs::GetMotionPlan::Response &res)
{
  if(!planner_initialized_)
  {
    ROS_ERROR("Hold up a second...the planner isn't initialized yet. Try again in a second or two.");
    return false;
  }

  if(req.motion_plan_request.goal_constraints.position_constraints.empty())
  {
    ROS_ERROR("There are no goal pose constraints in the request message. We need those to plan :).");
    return false;
  }

  if(!planToPosition(req, res))
    return false;

  return true;
}

bool SBPLCartArmPlannerNode::plan(std::vector<trajectory_msgs::JointTrajectoryPoint> &arm_path)
{
  bool b_ret(false);
  int solution_cost;
  std::vector<double>angles(num_joints_,0);

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
    sbpl_arm_env_.convertStateIDPathToShortenedJointAnglesPath(solution_state_ids_,angles_path, solution_state_ids_short_);
    
    ROS_DEBUG("[plan] A path was returned with %d waypoints.", int(solution_state_ids_.size()));
    ROS_INFO("Initial Epsilon: %0.3f  Final Epsilon: %0.3f Solution Cost: %d", planner_->get_initial_eps(),planner_->get_final_epsilon(), solution_cost);
    ROS_INFO("Original Path Length: %d   Shortened Path Length: %d", int(solution_state_ids_.size()), int(solution_state_ids_short_.size()));

    arm_path.resize(angles_path.size());
    for(size_t i=0; i < angles_path.size(); i++)
    {       
      arm_path[i].positions.resize(num_joints_);
      
      for (int p = 0; p < num_joints_; ++p)
        arm_path[i].positions[p] = angles_path[i][p];
      
      ROS_DEBUG("%i: %.4f %.4f %.4f %.4f %.4f %.4f %.4f", int(i), arm_path[i].positions[0],arm_path[i].positions[1],arm_path[i].positions[2],arm_path[i].positions[3],arm_path[i].positions[4],arm_path[i].positions[5],arm_path[i].positions[6]);
    }
  }

  return b_ret;
}

bool SBPLCartArmPlannerNode::isGoalConstraintSatisfied(const std::vector<double> &angles, const motion_planning_msgs::Constraints &goal)
{
  bool satisfied = true;
  geometry_msgs::Pose pose, err;

  if(!computeFK(angles,pose))
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
bool SBPLCartArmPlannerNode::computeIK(const geometry_msgs::Pose &pose, std::vector<double> jnt_pos, std::vector<double> &solution)
{
  kinematics_msgs::GetPositionIK::Request request;
  kinematics_msgs::GetPositionIK::Response response;

  request.ik_request.ik_link_name = planning_joint_;

  request.ik_request.pose_stamped.pose = pose;
  request.ik_request.pose_stamped.header.stamp = ros::Time();
  request.ik_request.pose_stamped.header.frame_id = reference_frame_;

  request.ik_request.ik_seed_state.joint_state.header.stamp = ros::Time();
  request.ik_request.ik_seed_state.joint_state.header.frame_id = reference_frame_;
  request.ik_request.ik_seed_state.joint_state.name = joint_names_;
  request.ik_request.ik_seed_state.joint_state.position.clear();

  for(int j = 0 ; j < num_joints_; ++j)
    request.ik_request.ik_seed_state.joint_state.position.push_back(jnt_pos[j]);

  ros::service::waitForService(ik_service_name_);
  ros::ServiceClient client = root_handle_.serviceClient<kinematics_msgs::GetPositionIK>(ik_service_name_, true);

  if(client.call(request, response))
  {
    ROS_DEBUG("Obtained IK solution");
    if(response.error_code.val == response.error_code.SUCCESS)
      for(unsigned int i=0; i < response.solution.joint_state.name.size(); i ++)
      {
        solution[i] = response.solution.joint_state.position[i];
        ROS_INFO("Joint: %s %f",response.solution.joint_state.name[i].c_str(),response.solution.joint_state.position[i]);
      }
    else
    {
      ROS_ERROR("Inverse kinematics failed");
      return false;
    }

    ROS_DEBUG("IK Solution");
    for(unsigned int i = 0; i < solution.size() ; ++i)
      ROS_DEBUG("%i: %f", i, solution[i]);
  }
  else
  {
    ROS_ERROR("IK service failed");
    return false;
  }
  return true;
}

bool SBPLCartArmPlannerNode::computeFK(const std::vector<double> &jnt_pos, geometry_msgs::Pose &pose)
{
  kinematics_msgs::GetPositionFK::Request  request;
  kinematics_msgs::GetPositionFK::Response response;

  request.header.stamp = ros::Time();
  request.header.frame_id = reference_frame_;

  request.robot_state.joint_state.name = joint_names_;

  for(unsigned int j = 0 ; j < jnt_pos.size(); ++j)
    request.robot_state.joint_state.position.push_back(jnt_pos[j]);

  request.fk_link_names.resize(1);
  request.fk_link_names[0] = planning_joint_;

  ROS_DEBUG("waiting for %s service", fk_service_name_.c_str());
  ros::service::waitForService(fk_service_name_);
  ros::ServiceClient client = root_handle_.serviceClient<kinematics_msgs::GetPositionFK>(fk_service_name_);

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

void SBPLCartArmPlannerNode::computeFKwithKDL(const std::vector<double> &jnt_pos, geometry_msgs::Pose &pose, int joint_num)
{
  KDL::JntArray jnt_array;
  KDL::Frame frame_out;

  jnt_array.resize(arm_chain_.getNrOfJoints());

  for(int i = 0; i < num_joints_; ++i)
    jnt_array(i+1)=jnt_pos[i];

  if(jnt_to_pose_solver_->JntToCart(jnt_array, frame_out, joint_num) < 0)
  {
    ROS_ERROR("JntToCart returned < 0. Exiting.\n");
    return;
  }

  pose.position.x = frame_out.p[0];
  pose.position.y = frame_out.p[1];
  pose.position.z = frame_out.p[2];
}

bool SBPLCartArmPlannerNode::initChain(std::string robot_description)
{
  KDL::Tree my_tree;

  if (!kdl_parser::treeFromString(robot_description, my_tree))
  {
    ROS_ERROR("Failed to parse tree from manipulator description file.\n");
    return false;;
  }

  if (!my_tree.getChain(reference_frame_, planning_joint_, arm_chain_))
  {
    ROS_ERROR("Could not fetch the KDL chain for the desired manipulator. Exiting.\n"); 
    return false;
  }

  jnt_to_pose_solver_ = new KDL::ChainFkSolverPos_recursive(arm_chain_);

  ROS_DEBUG("[initChain] arm_chain has %d segments and %d joints", arm_chain_.getNrOfSegments(), arm_chain_.getNrOfJoints());

  return true;
}

void SBPLCartArmPlannerNode::setArmToMapTransform(std::string &map_frame)
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
void SBPLCartArmPlannerNode::displayARAStarStates()
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

    aviz_->visualizeDetailedStates(expanded_states, detailed_color,"expanded",0.01);
  }

  ROS_INFO("[displayARAStarStates] displaying %d expanded states",int(expanded_states.size()));
}

void SBPLCartArmPlannerNode::visualizeGoalPosition(const motion_planning_msgs::Constraints &goal_pose)
{
  geometry_msgs::Pose pose;
  pose.position = goal_pose.position_constraints[0].position;
  pose.orientation = goal_pose.orientation_constraints[0].orientation;
  aviz_->visualizePose(pose, "goal_pose");
  ROS_DEBUG("[visualizeGoalPosition] publishing goal marker visualizations.");
}

void SBPLCartArmPlannerNode::visualizeCollisionObjects()
{
  std::vector<geometry_msgs::Pose> poses;
  std::vector<std::vector<double> > points(1,std::vector<double>(3,0));
  std::vector<double> color(4,1);
  color[2] = 0;

  cspace_->getCollisionObjectVoxelPoses(poses);

  points.resize(poses.size());
  for(size_t i = 0; i < poses.size(); ++i)
  {
    points[i].resize(3);
    points[i][0] = poses[i].position.x;
    points[i][1] = poses[i].position.y;
    points[i][2] = poses[i].position.z;
  }

  ROS_DEBUG("[visualizeCollisionObjects] Displaying %d known collision object voxels.", int(points.size()));
  aviz_->visualizeBasicStates(points, color, "known_objects", 0.01);
}

void SBPLCartArmPlannerNode::visualizeAttachedObject(trajectory_msgs::JointTrajectory &traj_msg, int throttle)
{
  std::vector<double> angles(7,0);
  std::vector<std::vector<double> > xyz;
  
  if(traj_msg.points.empty())
  {
    ROS_WARN("Trajectory message is empty. Not visualizing anything.");
    return;
  }

  for(size_t i = 0; i < traj_msg.points.size(); i++)
  {
    angles = traj_msg.points[i].positions;

    if(!cspace_->getAttachedObject(angles, xyz))
      continue;
   
    aviz_->visualizeSpheres(xyz, 10*(i+1), "sbpl_attached_object_" + boost::lexical_cast<std::string>(i), cspace_->getAttachedObjectRadius());
  }
}

void SBPLCartArmPlannerNode::visualizeAttachedObject(const std::vector<double> &angles)
{
  std::vector<std::vector<double> > xyz;

  if(angles.size() < 7)
  {
    ROS_WARN("[visualizeAttachedObject] Joint configuration is not of the right length.");
    return;
  }

  if(!cspace_->getAttachedObject(angles, xyz))
    return;

  aviz_->visualizeSpheres(xyz, 50, "sbpl_attached_object", cspace_->getAttachedObjectRadius());
}

void SBPLCartArmPlannerNode::displayShortestPath()
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
 
  aviz_->visualizeSpheres(dpath_, 45, "heuristic_path", 0.04);
}

void SBPLCartArmPlannerNode::visualizeEndEffectorPath()
{
  std::vector<geometry_msgs::Point> points;
  std::vector<std::vector<double> > path, path0, path1;

  sbpl_arm_env_.convertShortStateIDPathToPoints(solution_state_ids_short_, path);

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
  
  aviz_->visualizeSpheres(path0, 230, "end_effector_path0", 0.01);
  aviz_->visualizeSpheres(path1, 30, "end_effector_path1", 0.01);

  aviz_->visualizeLine(points,"end_effector_path", 0, 120, 0.005);
}

void SBPLCartArmPlannerNode::visualizeMotionPrimitives()
{
  char type = sbpl_arm_planner::LONG_DISTANCE;
  std::vector<std::vector<btVector3> > mprims;
  geometry_msgs::Point origin;
  std::vector<geometry_msgs::Point> motion;
  sbpl_arm_env_.getContMotionPrims(type, mprims);
 
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 1.75;

  for(size_t i = 0; i < mprims.size(); ++i)
  {
    motion.resize(mprims[i].size());
    
    for(size_t j = 0; j < mprims[i].size(); ++j)
    {
      motion[j].x = mprims[i][j].getX() + origin.x;
      motion[j].y = mprims[i][j].getY() + origin.y;
      motion[j].z = mprims[i][j].getZ() + origin.z;
    
      ROS_DEBUG("[%d-%d] %0.3f %0.3f %0.3f", int(i), int(j), motion[j].x, motion[j].y, motion[j].z);
    }
    
    aviz_->visualizeLine(motion, MOTION_PRIMS_NS, i, 180, 0.02);
  }
}

void SBPLCartArmPlannerNode::printPath(const std::vector<trajectory_msgs::JointTrajectoryPoint> &arm_path)
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

    computeFK(jnt_pos, pose);
    tf::poseMsgToTF(pose, tf_pose);
    tf_pose.getBasis().getRPY(roll,pitch,yaw);

    ROS_INFO("%3d: %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f\n   xyz: %2.3f %2.3f %2.3f  rpy: %0.3f %0.3f %0.3f  quat: %0.2f %0.2f %0.2f %0.2f", i,arm_path[i].positions[0],arm_path[i].positions[1],arm_path[i].positions[2],arm_path[i].positions[3],arm_path[i].positions[4],arm_path[i].positions[5],arm_path[i].positions[6],pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw, pose.orientation.x,pose.orientation.y, pose.orientation.z, pose.orientation.w);
  }
}

void SBPLCartArmPlannerNode::printPath(FILE* fOut, const std::vector<std::vector<double> > path)
{
  time_t init_time;
  time(&init_time);
  std::string str_time(asctime (localtime(&init_time)));

  fprintf(fOut, "%s", str_time.c_str());
  for(unsigned int i = 0; i < path.size(); i++)
    fprintf(fOut, "state %3d: %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f",i,path[i][0],path[i][1],path[i][2],path[i][3],path[i][4],path[i][5],path[i][6]);
  fprintf(fOut,"---------------------------------");
}

}

/* Node
 * ---------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbpl_arm_planner");
  sbpl_cartesian_arm_planner::SBPLCartArmPlannerNode arm_planner;
  if(!arm_planner.init())
  {
    ROS_ERROR("Failed to initialize arm planner node. Exiting.");
    return 0;
  }

  return arm_planner.run();
}



