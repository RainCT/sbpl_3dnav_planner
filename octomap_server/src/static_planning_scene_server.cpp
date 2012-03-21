/*
 * Copyright (c) 2010-2011, Willow Garage
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
 *     * Neither the name of Willow Garage Inc. nor the names of its
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

#include <octomap_server/static_planning_scene_server.h>

namespace octomap
{

StaticPlanningSceneServer::StaticPlanningSceneServer() : node_handle_("~")
{
  collision_map_subscriber_ = root_handle_.subscribe("collision_map_out", 1, &StaticPlanningSceneServer::collisionMapCallback,this);
	static_planning_scene_service_ = node_handle_.advertiseService("get_static_planning_scene", &StaticPlanningSceneServer::staticSceneCallback, this);
	revert_planning_scene_service_ = node_handle_.advertiseService("revert_planning_scene", &StaticPlanningSceneServer::revertSceneCallback, this);
}

StaticPlanningSceneServer::~StaticPlanningSceneServer()
{
}

void StaticPlanningSceneServer::collisionMapCallback(const arm_navigation_msgs::CollisionMapConstPtr &collision_map)
{
  static_collision_map_ = *collision_map;
}

bool StaticPlanningSceneServer::staticSceneCallback(arm_navigation_msgs::GetPlanningScene::Request &req, 
                                                    arm_navigation_msgs::GetPlanningScene::Response &res)
{
  current_collision_map_ = req.planning_scene_diff.collision_map;
  res.planning_scene.collision_map = static_collision_map_;  
	return true;
}

bool StaticPlanningSceneServer::revertSceneCallback(arm_navigation_msgs::GetPlanningScene::Request &req, 
                                                    arm_navigation_msgs::GetPlanningScene::Response &res)
{
  res.planning_scene.collision_map = current_collision_map_;  
	return true;
}


}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_planning_scene_server");
  octomap::StaticPlanningSceneServer s_server;
  ros::spin();

  return 0;
}

