/*
 * Copyright (c) 2011, Maxim Likhachev
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

/** \author Benjamin Cohen  */

#ifndef __SBPL_TWO_ARM_PLANNER_NODE_H_
#define __SBPL_TWO_ARM_PLANNER_NODE_H_

#define USE_LEARNING 0

#include <iostream>
#include <map>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <angles/angles.h>
#include <sbpl_two_arm_planner_node/visualize_arm.h>
#include <sbpl_arm_planner/body_pose.h>
//#include <sbpl_two_arm_planner_node/arm.h>
#include <sbpl_two_arm_planner_node/GetTwoArmPlan.h>
#include <sbpl_two_arm_planner/environment_dualrobarm3d.h>
#include <pviz/pviz.h>

/** Messages **/
#include <geometry_msgs/Pose.h>
#include <mapping_msgs/CollisionMap.h>
#include <mapping_msgs/CollisionObject.h>
#include <mapping_msgs/AttachedCollisionObject.h>
#include <motion_planning_msgs/GetMotionPlan.h>
#include <trajectory_msgs/JointTrajectoryPoint.h> 
#include <kinematics_msgs/GetPositionFK.h>

#include <ros/console.h>
#include <log4cxx/logger.h>

namespace sbpl_two_arm_planner {

class SBPLTwoArmPlannerNode
{
  public:
    SBPLTwoArmPlannerNode();
    ~SBPLTwoArmPlannerNode();
    
    bool init();
    
    int run();

    void changeLoggerLevel(std::string name, std::string level);

    bool setStart(geometry_msgs::Pose start, geometry_msgs::Pose rarm_object, geometry_msgs::Pose larm_object);

    bool setGoalPosition(geometry_msgs::Pose goal, geometry_msgs::Pose rarm_object, geometry_msgs::Pose larm_object, std::vector<double> &goal_tolerance);

    bool planToPosition(sbpl_two_arm_planner_node::GetTwoArmPlan::Request &req, sbpl_two_arm_planner_node::GetTwoArmPlan::Response &res);

    //void sendArmsToStart(geometry_msgs::Pose &object_pose);

  private:

    ros::NodeHandle node_handle_, root_handle_;
    ros::ServiceServer planning_service_;
    ros::Subscriber joint_states_subscriber_;
    ros::Subscriber collision_object_subscriber_;
    ros::Subscriber object_subscriber_;
    ros::Subscriber path_subscriber_;
    message_filters::Subscriber<mapping_msgs::CollisionMap> collision_map_subscriber_;
    tf::MessageFilter<mapping_msgs::CollisionMap> *collision_map_filter_;

    boost::mutex joint_states_mutex_;
    boost::mutex collision_map_mutex_;

    std::string left_fk_service_name_;
    std::string right_fk_service_name_;
    std::string left_ik_service_name_;
    std::string right_ik_service_name_;

    double allocated_time_;
    double waypoint_time_;
    double env_resolution_;

    clock_t totalPlanTime;

    /* params */
    bool forward_search_;
    bool search_mode_;
    bool visualize_expanded_states_;
    bool visualize_heuristic_;
    bool visualize_goal_;
    bool visualize_end_effector_path_;
    bool planner_initialized_;
    bool print_path_;
    bool visualize_trajectory_;
    bool visualize_collision_model_trajectory_;
    bool use_first_solution_;
    bool use_collision_map_from_sensors_;
    bool use_shortened_path_;
    bool use_inner_circle_;
    bool visualize_heuristic_grid_;
    int throttle_;
    int num_joints_;
    double object_radius_;

    std::string collision_map_topic_;
    std::string robot_description_;
    std::string reference_frame_;
    std::string map_frame_;
    std::string planning_joint_;
    std::string arm_name_;
    std::string left_arm_description_filename_;
    std::string right_arm_description_filename_;
    std::string mprims_filename_;
    std::string base_mprims_filename_;
    std::vector<std::string> ljoint_names_;
    std::vector<std::string> rjoint_names_;
    std::vector<std::vector<double> > dpath_;
    std::vector<std::vector<double> > dpath0_;
    std::vector<std::vector<double> > dpath1_;
    std::vector<int> solution_state_ids_;
    std::vector<int> solution_state_ids_short_;
    std::vector<double> rangles_;
    std::vector<double> langles_;
    double torso_lift_;
    double head_pan_;
    double head_tilt_;
    BodyPose body_pos_;

    geometry_msgs::Pose object_start_;
    geometry_msgs::Pose rarm_object_offset_;
    geometry_msgs::Pose larm_object_offset_;

    /* planner & environment */
    MDPConfig mdp_cfg_;
#if USE_LEARNING
    OLCG_Body sbpl_arm_env_;
#else
    sbpl_two_arm_planner::EnvironmentDUALROBARM3D sbpl_arm_env_;
#endif
    SBPLPlanner *planner_;
    sbpl_two_arm_planner::SBPLDualCollisionSpace* cspace_;
    sbpl_arm_planner::OccupancyGrid* grid_;
    sbpl_two_arm_planner::VisualizeArm* laviz_;
    sbpl_two_arm_planner::VisualizeArm* raviz_;

    /* transforms and kinematics */
    tf::TransformListener tf_;
    tf::StampedTransform transform_;
    tf::StampedTransform base_map_transform_;
    KDL::Frame kdl_transform_;

    //Arm* rarm_;
    //Arm* larm_;

    /* debug */
    double x_min_, x_max_, x_inc_;
    double y_min_, y_max_, y_inc_;
    double z_min_, z_max_, z_inc_;
    std::string succ_log_name_;
    std::string succ_log_level_;
    std::vector<std::string> stats_field_names_;
    std::vector<double> stats_;
    std::vector<std::string> debug_code_names_;
    PViz pviz_;

    /* attached objects */
    bool attached_object_;

    /* collision objects */
    std::map<std::string, mapping_msgs::CollisionObject> object_map_;

    /** callbacks **/
    void updateMapFromCollisionMap(const mapping_msgs::CollisionMapConstPtr &collision_map);

    void collisionMapCallback(const mapping_msgs::CollisionMapConstPtr &collision_map);

    void jointStatesCallback(const sensor_msgs::JointStateConstPtr &state);

    void collisionObjectCallback(const mapping_msgs::CollisionObjectConstPtr &collision_object);

    void attachedObjectCallback(const mapping_msgs::AttachedCollisionObjectConstPtr &attached_object);

    /** planning **/
    bool initializePlannerAndEnvironment();

    bool plan(std::vector<trajectory_msgs::JointTrajectoryPoint> &rpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &lpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &bpath);

    bool isGoalConstraintSatisfied(const std::vector<double> &rangles, const std::vector<double> &langles, const geometry_msgs::Pose &goal);

    void setArmToMapTransform(std::string &map_frame);
    
    /** kinematics **/
    bool computeFK(const std::vector<double> &jnt_pos, std::string arm_name, geometry_msgs::Pose &pose);

    void getRobotState(BodyPose &body_pos, std::vector<double> &rangles, std::vector<double> &langles);

    /** visualizations **/
    void visualizeExpansions();

    void visualizeUniqueExpansions();

    void displayShortestPath();

    void printPath(FILE* fOut, const std::vector<std::vector<double> > path);

    void printPath(const std::vector<trajectory_msgs::JointTrajectoryPoint> &rpath, const std::vector<trajectory_msgs::JointTrajectoryPoint> &lpath, const std::vector<trajectory_msgs::JointTrajectoryPoint> &bpath);

    void visualizeGoal(geometry_msgs::Pose goal);

    void visualizeCollisionObjects();

    void visualizeGoalPosition(const motion_planning_msgs::Constraints &goal);
    
    void visualizeEndEffectorPath();
    
    void visualizeHeuristicInfo();

    void visualizeExpansionsPerHValue();

    void visualizeHeuristicGrid();

    void visualizeAttachedObject();

    void visualizeCollisionObject(const mapping_msgs::CollisionObject &object);

    void visualizeAttachedObjectPath();

    void visualizeTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> &rpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &lpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &bpath);

    void printRobotState(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &body_pos, std::string text);

    void visualizeObjectPath();
};

}

#endif
