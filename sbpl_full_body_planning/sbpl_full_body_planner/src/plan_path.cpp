/*
 * Copyright (c) 2010, Maxim Likhachev
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
 *     * Neither the name of the University of Pennsylvania nor the names of its
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

#include <iostream>
#include <sbpl_full_body_planner/environment_dualrobarm3d.h>
#include <ros/ros.h>

#define MAX_RUNTIME 5.0
#define NUM_JOINTS 7

int planRobarm(int argc, char *argv[])
{
  int bRet = 0;
  int sol_cost;
  double allocated_time_secs = MAX_RUNTIME; //in seconds
  MDPConfig MDPCfg;
  std::string arm0_filename("/home/bcohen/ros/sbpl_cartesian_arm_planning/sbpl_cartesian_arm_planning/sbpl_arm_planner/config/pr2_right_arm.cfg");
  std::string arm1_filename("/home/bcohen/ros/sbpl_cartesian_arm_planning/sbpl_cartesian_arm_planning/sbpl_arm_planner/config/pr2_left_arm.cfg");
  std::string params_filename("/home/bcohen/ros/sbpl_two_arm_planning/sbpl_two_arm_planning/sbpl_two_arm_planner/config/params.cfg");
  std::string mprims_filename("/home/bcohen/ros/sbpl_two_arm_planning/sbpl_two_arm_planning/sbpl_two_arm_planner/config/pr2-26.mprim");

  //Initialize Environment (should be called before initializing anything else)
  sbpl_full_body_planner::EnvironmentDUALROBARM3D environment_robarm;

  /*
  if(!environment_robarm.InitializeEnv(argv[1], params_filename, arm0_filename, arm1_filename, mprims_filename))
  {
    printf("ERROR: InitializeEnv failed\n");
    return 0;
  }
  */

  //Initialize MDP Info
  if(!environment_robarm.InitializeMDPCfg(&MDPCfg))
  {
    printf("ERROR: InitializeMDPCfg failed\n");
    return 0;
  }

  //plan a path
  clock_t starttime = clock();

  vector<int> solution_stateIDs_V;
  bool bforwardsearch = true;
    
  ARAPlanner planner(&environment_robarm, bforwardsearch);

  if(planner.set_start(MDPCfg.startstateid) == 0)
  {
    printf("ERROR: failed to set start state\n");
    return 0;
  }

  if(planner.set_goal(MDPCfg.goalstateid) == 0)
  {
    printf("ERROR: failed to set goal state\n");
    return 0;
  }

  //set epsilon
  planner.set_initialsolution_eps(environment_robarm.getEpsilon());

  //set search mode (true - settle with first solution)
  planner.set_search_mode(false);

  //plan 
  printf("start planning...\n");
  bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V,&sol_cost);
  printf("completed in %.4f seconds.\n", double(clock()-starttime) / CLOCKS_PER_SEC);
  /*
  for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++)
    environment_robarm.PrintState(solution_stateIDs_V[i], true, stdout);
  */

  fflush(stdout);
  return bRet;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "sbpl_arm_planner");
  
  if(argc < 2)
  {
    printf("Missing second argument.\n");
    exit(1);
  }
  
  ros::NodeHandle nh;

  sleep(1);
  planRobarm(argc, argv);
  return 0;
}

