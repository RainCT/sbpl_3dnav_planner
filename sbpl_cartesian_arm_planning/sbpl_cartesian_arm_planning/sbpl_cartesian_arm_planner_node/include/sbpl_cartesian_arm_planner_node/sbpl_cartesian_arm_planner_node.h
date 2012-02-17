/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Benjamin Cohen, Sachin Chitta  */

#ifndef __SBPL_CART_ARM_PLANNER_NODE_H_
#define __SBPL_CART_ARM_PLANNER_NODE_H_

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
#include <kdl/chainjnttojacsolver.hpp>
#include <angles/angles.h>
#include <sbpl_cartesian_arm_planner_node/visualize_arm.h>
#include <sbpl_cartesian_arm_planner/environment_cartrobarm3d.h>

/** Messages **/
#include <geometry_msgs/Pose.h>
#include <mapping_msgs/CollisionMap.h>
#include <mapping_msgs/AttachedCollisionObject.h>
#include <motion_planning_msgs/GetMotionPlan.h>
#include <trajectory_msgs/JointTrajectoryPoint.h> 
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>

namespace sbpl_cartesian_arm_planner {

static const std::string MOTION_PRIMS_NS = "motion_prims";

class SBPLCartArmPlannerNode
{
  public:
    /** \brief Constructor-doesn't initialize class, you need to run init() */
    SBPLCartArmPlannerNode();

    ~SBPLCartArmPlannerNode();

    /** \brief Initialize the planner node. */
    bool init();

    /** @brief Run the node! */
    int run();

    /** \brief Planning service call back function */
    bool planKinematicPath(motion_planning_msgs::GetMotionPlan::Request &req, motion_planning_msgs::GetMotionPlan::Response &res);

  private:

    ros::NodeHandle node_handle_, root_handle_;
    ros::ServiceServer planning_service_;
    ros::Subscriber object_subscriber_;
    message_filters::Subscriber<mapping_msgs::CollisionMap> collision_map_subscriber_;
    tf::MessageFilter<mapping_msgs::CollisionMap> *collision_map_filter_;

    //remove
    ros::Subscriber collision_object_subscriber_;
    void collisionObjectCallback(const mapping_msgs::CollisionObjectConstPtr &collision_object);
    std::vector<std::vector<double> > col_objects_;

    /** @brief A FK solver. (The FK service is currently being used) */
    KDL::ChainFkSolverPos_recursive *jnt_to_pose_solver_;

    /** @brief The name of the forward kinematics service. */
    std::string fk_service_name_;

    /** @brief The name of the inverse kinematics service. */
    std::string ik_service_name_;

    /** @brief "r" - right arm, "l" - left arm */
    std::string side_;

    /** @brief Maximum time for planner to return a solution */
    double allocated_time_;

    /** @brief Time interval sent with each waypoint in trajectory to
     * movearm */
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
    bool attached_object_;
    int throttle_;
    int num_joints_;

    std::string collision_map_topic_;
    std::string robot_description_;
    std::string node_name_;
    std::string reference_frame_;
    std::string map_frame_;
    std::string planning_joint_;
    std::string arm_name_;
    std::string trajectory_type_;
    std::string arm_description_filename_;
    std::string mprims_filename_;
    std::string attached_object_frame_;
    std::vector<std::string> joint_names_;
    std::vector<std::vector<double> > dpath_;
    std::map<std::string, mapping_msgs::CollisionObject> object_map_;
    std::vector<int> solution_state_ids_;
    std::vector<int> solution_state_ids_short_;

    /* planner & environment */
    MDPConfig mdp_cfg_;
    sbpl_cartesian_arm_planner::EnvironmentCARTROBARM3D sbpl_arm_env_;
    SBPLPlanner *planner_;
    sbpl_arm_planner::SBPLCollisionSpace* cspace_;
    sbpl_arm_planner::OccupancyGrid* grid_;
    sbpl_cartesian_arm_planner::VisualizeArm* aviz_;

    boost::mutex colmap_mutex_;
    boost::mutex object_mutex_;

    /* transforms and kinematics */
    tf::TransformListener tf_;
    tf::StampedTransform transform_;
    KDL::Chain arm_chain_;
    KDL::JntArray jnt_pos_;
    KDL::Frame kdl_transform_;

    /** \brief Initialize the SBPL planner and the sbpl_arm_planner environment */
    bool initializePlannerAndEnvironment();

    /** \brief Set start configuration */
    bool setStart(const sensor_msgs::JointState &start_state);

    /** \brief Set cartesian goal(s) */
    bool setGoalPosition(const motion_planning_msgs::Constraints &goals);

    /** \brief Callback function that's called by the collision map topic. It reformats the collision map for the sbpl grid */
    void updateMapFromCollisionMap(const mapping_msgs::CollisionMapConstPtr &collision_map);

    /** \brief Callback function that gets called when a new collision map
     * is available. It updates the internal distance field with the map
     * and visualizes it (if desired).*/
    void collisionMapCallback(const mapping_msgs::CollisionMapConstPtr &collision_map);

    void attachedObjectCallback(const mapping_msgs::AttachedCollisionObjectConstPtr &attached_object);

    void attachObject(const mapping_msgs::CollisionObject &obj);

    /** \brief Plan a path to a cartesian goal(s) */
    bool planToPosition(motion_planning_msgs::GetMotionPlan::Request &req, motion_planning_msgs::GetMotionPlan::Response &res);

    /** \brief Retrieve plan from sbpl */
    bool plan(std::vector<trajectory_msgs::JointTrajectoryPoint> &arm_path);

    /** \brief Check if path satisfies goal constraints */
    bool isGoalConstraintSatisfied(const std::vector<double> &angles, const motion_planning_msgs::Constraints &goal);

    /** \brief Compute IK using pr2_ik service */
    bool computeIK(const geometry_msgs::Pose &pose_stamped_msg, std::vector<double> jnt_pos, std::vector<double> &solution);

    /** \brief Compute FK using pr2_fk service */
    bool computeFK(const std::vector<double> &jnt_pos, geometry_msgs::Pose &pose);

    /** \brief Compute FK using the KDL library */
    void computeFKwithKDL(const std::vector<double> &jnt_pos, geometry_msgs::Pose &pose, int joint_num);

    /** \brief Initialize KDL Chain for using FK */
    bool initChain(std::string robot_description);

    /** \brief Publish a visualization marker to display the goal end effector position in rviz */
    void visualizeGoalPosition(const motion_planning_msgs::Constraints &goal);

    /** \brief Computes and visualizes the 3D path to goal returned by heuristic */
    void displayShortestPath();

    /** \brief Print the trajectory to a file */
    void printPath(FILE* fOut, const std::vector<std::vector<double> > path);

    /** 
     * @brief Print out the trajectory along with the end effector coords
     * of each waypoint. 
     * @param a vector of points
     */
    void printPath(const std::vector<trajectory_msgs::JointTrajectoryPoint> &arm_path);

    /** \brief Set the transform in the environment class (remove this)*/
    void setArmToMapTransform(std::string &map_frame);

    /** \brief Display states expanded by ARA* search */
    void displayARAStarStates();

    /* \brief Visualize the collision model of the attached object */
    void visualizeAttachedObject(trajectory_msgs::JointTrajectory &traj_msg, int throttle);

    /* \brief Visualize the collision model of the attached object */
    void visualizeAttachedObject(const std::vector<double> &angles);

    /** \brief Visualize voxels occupying known collision objects */
    void visualizeCollisionObjects();

    void visualizeEndEffectorPath();

    void visualizeMotionPrimitives();
};
}

#endif
