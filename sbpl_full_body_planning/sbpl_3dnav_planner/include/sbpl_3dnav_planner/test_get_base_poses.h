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

/** \author Sachin Chitta */

#include <iostream>
#include <map>

#include <boost/thread/mutex.hpp>
#include <log4cxx/logger.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>

/** Messages **/
#include <sensor_msgs/JointState.h>
#include <arm_navigation_msgs/RobotState.h>
#include <sbpl_3dnav_planner/GetBasePoses.h>

namespace sbpl_3dnav_planner
{
class TestSBPLBasePoses 
{
public:
  TestSBPLBasePoses();
  ~TestSBPLBasePoses();    
  void jointStatesCallback(const sensor_msgs::JointStateConstPtr &state);
  void callService(const std::string &group_name);
    
private:
  ros::NodeHandle root_handle_;
  ros::NodeHandle node_handle_;
  ros::Subscriber joint_states_subscriber_;
  boost::mutex joint_states_lock_;  
  ros::ServiceClient test_client_;
  arm_navigation_msgs::RobotState robot_state_;
  tf::TransformListener tf_;
  tf::StampedTransform transform_;
  tf::StampedTransform base_map_transform_;

};
}

