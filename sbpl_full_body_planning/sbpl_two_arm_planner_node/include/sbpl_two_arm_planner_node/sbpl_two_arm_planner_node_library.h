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
#include <sbpl_two_arm_planner_node/arm.h>
#include <sbpl_two_arm_planner/environment_dualrobarm3d.h>

/** Messages **/
#include <geometry_msgs/Pose.h>
#include <mapping_msgs/CollisionMap.h>
#include <motion_planning_msgs/GetMotionPlan.h>
#include <trajectory_msgs/JointTrajectoryPoint.h> 
#include <kinematics_msgs/GetPositionFK.h>

namespace sbpl_two_arm_planner {

class SBPLTwoArmPlannerNode
{
  public:
    SBPLTwoArmPlannerNode();
    ~SBPLTwoArmPlannerNode();
    
    bool init();
    
    int run();

    bool setStart(geometry_msgs::Pose start);

    bool setGoalPosition(geometry_msgs::Pose goal);

    bool planToPosition(geometry_msgs::Pose start, geometry_msgs::Pose goal);

    void sendArmsToStart(geometry_msgs::Pose &object_pose);

  private:

    ros::NodeHandle node_handle_, root_handle_;
    ros::ServiceServer planning_service_;
    ros::Subscriber joint_states_subscriber_;
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

    /* params */
    bool forward_search_;
    bool search_mode_;
    bool visualize_expanded_states_;
    bool visualize_heuristic_;
    bool visualize_goal_;
    bool planner_initialized_;
    bool print_path_;
    bool visualize_trajectory_;
    bool visualize_collision_model_trajectory_;
    bool use_first_solution_;
    int throttle_;
    int num_joints_;

    std::string collision_map_topic_;
    std::string robot_description_;
    std::string reference_frame_;
    std::string map_frame_;
    std::string planning_joint_;
    std::string arm_name_;
    std::string left_arm_description_filename_;
    std::string right_arm_description_filename_;
    std::string mprims_filename_;
    std::vector<std::string> ljoint_names_;
    std::vector<std::string> rjoint_names_;
    std::vector<std::vector<double> > dpath_;
    std::vector<int> solution_state_ids_;
    std::vector<int> solution_state_ids_short_;
    std::vector<double> rangles_;
    std::vector<double> langles_;

    geometry_msgs::Pose object_start_;
    geometry_msgs::Pose rarm_object_offset_;
    geometry_msgs::Pose larm_object_offset_;

    /* planner & environment */
    MDPConfig mdp_cfg_;
    sbpl_two_arm_planner::EnvironmentDUALROBARM3D sbpl_arm_env_;
    SBPLPlanner *planner_;
    sbpl_two_arm_planner::SBPLDualCollisionSpace* cspace_;
    sbpl_arm_planner::OccupancyGrid* grid_;
    sbpl_two_arm_planner::VisualizeArm* laviz_;
    sbpl_two_arm_planner::VisualizeArm* raviz_;

    /* transforms and kinematics */
    tf::TransformListener tf_;
    tf::StampedTransform transform_;
    KDL::Frame kdl_transform_;

    Arm* rarm_;
    Arm* larm_;

    /** callbacks **/
    void updateMapFromCollisionMap(const mapping_msgs::CollisionMapConstPtr &collision_map);

    void collisionMapCallback(const mapping_msgs::CollisionMapConstPtr &collision_map);

    void jointStatesCallback(const sensor_msgs::JointStateConstPtr &state);

    /** planning **/
    bool initializePlannerAndEnvironment();

    bool plan(std::vector<trajectory_msgs::JointTrajectoryPoint> &arm_path);

    bool isGoalConstraintSatisfied(const std::vector<double> &angles, const motion_planning_msgs::Constraints &goal);

    void setArmToMapTransform(std::string &map_frame);
    
    /** kinematics **/
    bool computeFK(const std::vector<double> &jnt_pos, std::string arm_name, geometry_msgs::Pose &pose);

    /** visualizations **/
    void displayARAStarStates();
    
    void displayShortestPath();

    void printPath(FILE* fOut, const std::vector<std::vector<double> > path);

    void printPath(const std::vector<trajectory_msgs::JointTrajectoryPoint> &arm_path);

    void visualizeGoal(geometry_msgs::Pose goal);

    void visualizeGoalPosition(const motion_planning_msgs::Constraints &goal);
    
    void visualizeEndEffectorPath();
};

}

#endif
