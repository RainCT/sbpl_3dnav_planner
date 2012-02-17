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
/** \author Benjamin Cohen, Maxim Likhachev */

#include <sbpl_cartesian_arm_planner/environment_cartrobarm3d.h>

#define DEG2RAD(d) ((d)*(M_PI/180.0))
#define RAD2DEG(r) ((r)*(180.0/M_PI))

#define DEBUG_SEARCH 0
#define DEBUG_HEURISTIC 0

#if DEBUG_SEARCH
  FILE* fSucc = SBPL_FOPEN("/tmp/debug_succs.txt", "w");
#endif

FILE* fMP = fopen("/home/bcohen/Desktop/debug_succs_detailed.csv", "w");
FILE* fMP2 = fopen("/home/bcohen/Desktop/debug_succs_simple.csv", "w");

//Statistics
bool near_goal = false;
clock_t starttime;
double time_to_goal_region;

using namespace std;

namespace sbpl_cartesian_arm_planner{

EnvironmentCARTROBARM3D::EnvironmentCARTROBARM3D()
{
  EnvROBARMCfg.bInitialized = false;
  save_expanded_states = true;

  params_filename_ = "./config/params.cfg";
  arm_desc_filename_ = "./config/pr2_right_arm.cfg"; 
  free_angle_index_ = 2;
  using_short_mprims_ = false;
  ndof_ = 7;
}

EnvironmentCARTROBARM3D::~EnvironmentCARTROBARM3D()
{
  if(cspace_ != NULL)
    delete cspace_;
  if(arm_ != NULL)
    delete arm_;
  if(dijkstra_ != NULL)
    delete dijkstra_;
  if(grid_ != NULL)
    delete grid_;

  for(size_t i = 0; i < EnvROBARM.StateID2CoordTable.size(); i++)
  {
    delete EnvROBARM.StateID2CoordTable.at(i);
    EnvROBARM.StateID2CoordTable.at(i) = NULL;
  }
  EnvROBARM.StateID2CoordTable.clear();

  if(EnvROBARM.Coord2StateIDHashTable != NULL)
  {
    delete [] EnvROBARM.Coord2StateIDHashTable;
    EnvROBARM.Coord2StateIDHashTable = NULL;
  }

#if DEBUG_SEARCH
    SBPL_FCLOSE(fSucc);
#endif

#if DEBUG_HEURISTIC
    SBPL_FCLOSE(fHeur);
#endif
}

/////////////////////////////////////////////////////////////////////////////
//                      SBPL Planner Interface
/////////////////////////////////////////////////////////////////////////////

bool EnvironmentCARTROBARM3D::InitializeMDPCfg(MDPConfig *MDPCfg)
{
  //initialize MDPCfg with the start and goal ids	
  MDPCfg->goalstateid = EnvROBARM.goalHashEntry->stateID;
  MDPCfg->startstateid = EnvROBARM.startHashEntry->stateID;
  return true;
}

bool EnvironmentCARTROBARM3D::InitializeEnv(const char* sEnvFile, std::string params_file, std::string arm_file)
{
  params_filename_ = params_file;
  arm_desc_filename_ = arm_file;

  return InitializeEnv(sEnvFile);
}

bool EnvironmentCARTROBARM3D::InitializeEnv(const char* sEnvFile)
{
  //parse the parameter file
  FILE* fCfg = fopen(params_filename_.c_str(), "r");
  if(fCfg == NULL)
  {
    SBPL_ERROR("Unable to open params file: %s. Exiting.", params_filename_.c_str());
    return false;
  }

  SBPL_DEBUG("[InitializeEnv] Retrieving parameters from file: %s", params_filename_.c_str());
  if(!prms_.initFromParamFile(fCfg))
   {
    SBPL_ERROR("[InitializeEnv] Failed to retrieve parameters from %s. Exiting.", params_filename_.c_str());
    SBPL_FCLOSE(fCfg);
    return false;
  }
  SBPL_FCLOSE(fCfg);

  //parse the arm description file
  SBPL_DEBUG("[InitializeEnv] Retrieving arm description from file: %s", arm_desc_filename_.c_str());
  FILE* aCfg = fopen(arm_desc_filename_.c_str(), "r");
  if(aCfg == NULL)
  {
    SBPL_ERROR("[InitializeEnv] Unable to open arm description file: %s. Exiting.", arm_desc_filename_.c_str());
    return false;
  }

  std::string robot_description("ROS_PARAM");
  SBPL_DEBUG("[InitializeEnv] Getting URDF from ROS Param server.");
  if(!initArmModel(aCfg, robot_description))
  {
    SBPL_ERROR("[InitializeEnv] Failed to initialize Arm Model from %s. Exiting.", arm_desc_filename_.c_str());
    return false;
  }
  SBPL_FCLOSE(aCfg);

  //parse the environment file
  fCfg = fopen(sEnvFile, "r");
  if(fCfg == NULL)
  {
    SBPL_ERROR("[InitializeEnv] Unable to open environment file: %s. Exiting.", sEnvFile);
    return false;
  }

  readConfiguration(fCfg);
  SBPL_FCLOSE(fCfg);

  if(!initGeneral())
  {
    SBPL_ERROR("InitGeneral() failed.");
    return false;
  }

  SBPL_DEBUG("Ready to start planning. Setting start configuration.");

  //add self collision cuboids to grid
  cspace_->addArmCuboidsToGrid();

  //add obstacle cuboids in environment to grid
  for(int i = 0; i < (int)EnvROBARMCfg.obstacles.size(); i++)
  {
    if(EnvROBARMCfg.obstacles[i].size() == 6)
      grid_->addCollisionCuboid(EnvROBARMCfg.obstacles[i][0],EnvROBARMCfg.obstacles[i][1],EnvROBARMCfg.obstacles[i][2],EnvROBARMCfg.obstacles[i][3],EnvROBARMCfg.obstacles[i][4],EnvROBARMCfg.obstacles[i][5]);
    else
      SBPL_DEBUG("[InitializeEnv] Obstacle #%d has an incomplete obstacle description. Not adding obstacle it to grid.", i);
  }

  //initialize the angles of the start states
  if(!setStartConfiguration(EnvROBARMCfg.start_configuration))
  {
    SBPL_ERROR("[InitializeEnv] Failed to set initial state of robot. Exiting.");
    return false;
  }

  //set 'Environment is Initialized' flag (so fk can be used)
  EnvROBARMCfg.bInitialized = true;

  //set goals
  if(!setGoalPosition(EnvROBARMCfg.ParsedGoals,EnvROBARMCfg.ParsedGoalTolerance))
  {
    SBPL_ERROR("[InitializeEnv] Failed to set goal pose. Exiting.");
    return false;
  }

  grid_->visualize();

  //for statistics purposes
  starttime = clock();		

  SBPL_INFO("Successfully initialized planning environment.");
  return true;
}

int EnvironmentCARTROBARM3D::GetFromToHeuristic(int FromStateID, int ToStateID)
{
#if USE_HEUR==0
  return 0;
#endif

  return getEndEffectorHeuristic(FromStateID,ToStateID);
}

int EnvironmentCARTROBARM3D::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
  return 0;
#endif

#if DEBUG_HEUR
  if(stateID >= (int)EnvROBARM.StateID2CoordTable.size())
  {
    SBPL_ERROR("ERROR in EnvROBARM... function: stateID illegal");
    return -1;
  }
#endif

  return GetFromToHeuristic(stateID, EnvROBARM.goalHashEntry->stateID);
}

int EnvironmentCARTROBARM3D::GetStartHeuristic(int stateID)
{
#if USE_HEUR==0
  return 0;
#endif

#if DEBUG_HEUR
  if(stateID >= (int)EnvROBARM.StateID2CoordTable.size())
  {
    SBPL_ERROR("ERROR in EnvROBARM... function: stateID illegal\n");
    return -1;
  }
#endif

  return GetFromToHeuristic(stateID, EnvROBARM.startHashEntry->stateID);
}

int EnvironmentCARTROBARM3D::SizeofCreatedEnv()
{
  return EnvROBARM.StateID2CoordTable.size();
}

void EnvironmentCARTROBARM3D::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG_HEUR
  if(stateID >= (int)EnvROBARM.StateID2CoordTable.size())
  {
    SBPL_ERROR("ERROR in EnvROBARM... function: stateID illegal (2)\n");
    throw new SBPL_Exception();
  }
#endif

  if(fOut == NULL)
    fOut = stdout;

  EnvCARTROBARM3DHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[stateID];

  bool bGoal = false;
  if(stateID == EnvROBARM.goalHashEntry->stateID)
    bGoal = true;

  printJointArray(fOut, HashEntry, bGoal, bVerbose);
}

void EnvironmentCARTROBARM3D::PrintEnv_Config(FILE* fOut)
{
  //implement this function if the planner needs to print out the EnvROBARM. configuration
  SBPL_ERROR("ERROR in EnvROBARM... function: PrintEnv_Config is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentCARTROBARM3D::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
  GetSuccs(SourceStateID,SuccIDV,CostV,NULL);
}

void EnvironmentCARTROBARM3D::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<int>* ActionV)
{
  bool invalid_prim = false;
  char use_prim_type=sbpl_arm_planner::LONG_DISTANCE;
  unsigned char dist=0, dist_temp=0;
  size_t i=0, a=0, j=0, q=0;
  int motion_cost=0;
  std::vector<short unsigned int> succcoord(ndof_,0);
  std::vector<int> mp_coord(ndof_,0);
  std::vector<double> pose(6,0), angles(ndof_,0), source_angles(ndof_,0), parent_angles(ndof_,0), sangles(ndof_,0), pangles(ndof_,0), iangles(ndof_,0), succ_wcoord(ndof_,0), parent_wcoord(ndof_,0);
  short unsigned int xyz[3]={0},rpy[3]={0},fa=0;
  double wxyz[3]={0},wrpy[3]={0},wfa=0, xyz_source[3]={0},rpy_source[3]={0},fa_source=0;

  //clear the successor arrays
  SuccIDV->clear();
  CostV->clear();
  SuccIDV->reserve(prms_.mp_.size());
  CostV->reserve(prms_.mp_.size());
  if(ActionV != NULL)
  {
    ActionV->clear();
    ActionV->reserve(prms_.mp_.size());
  }

  //goal state should be absorbing
  if(SourceStateID == EnvROBARM.goalHashEntry->stateID)
    return;

  //get X, Y, Z for the state
  EnvCARTROBARM3DHashEntry_t* parent = EnvROBARM.StateID2CoordTable[SourceStateID];

  //default coords of successor
  for(i = 0; i < parent->coord.size(); i++)
    succcoord[i] = parent->coord[i];

  stateIDToWorldPose(SourceStateID, xyz_source, rpy_source, &fa_source);

  if(!convertCoordToAngles(&succcoord, &parent_angles))
  {
    ROS_WARN("[search] Failed to convert the parent coords into angles. IK must have failed. (stateID: %d)", SourceStateID);
    for(size_t p = 0; p < parent->angles.size(); ++p)
      parent_angles[p] = parent->angles[p];
  }

//#if DEBUG_SEARCH
  ROS_DEBUG_NAMED("search", "\nstate %d: xyz: %.3f %.3f %.3f  rpy: %.3f %.3f %.3f fa: %.3f",SourceStateID,xyz_source[0],xyz_source[1],xyz_source[2],rpy_source[0],rpy_source[1],rpy_source[2],fa_source);
//#endif

  int dist_to_goal = getDijkstraDistToGoal(parent->coord[0],parent->coord[1],parent->coord[2]);
  
  if(prms_.use_multires_mprims_)
  {
    if(dist_to_goal <= prms_.short_dist_mprims_thresh_c_)
    {
      if(!using_short_mprims_)
        SBPL_INFO("[search] Switching to short distance mprims after %d expansions. (SourceState ID, #%d, is %0.3fm from goal)", int(expanded_states.size()), SourceStateID, double(dist_to_goal)/double(prms_.cost_per_cell_)*prms_.resolution_); 
      use_prim_type = sbpl_arm_planner::SHORT_DISTANCE;
      using_short_mprims_ = true;
    }
  }

  //precalculate the difference to the goal orientation to create an adaptive mprim
  if(dist_to_goal <= 50)
  {
    prms_.mp_[prms_.mp_.size()-1].m[0][3] = angles::normalize_angle(double(EnvROBARM.goalHashEntry->coord[3] - parent->coord[3])*EnvROBARMCfg.coord_delta[3]);
    prms_.mp_[prms_.mp_.size()-1].m[0][4] = angles::normalize_angle(double(EnvROBARM.goalHashEntry->coord[4] - parent->coord[4])*EnvROBARMCfg.coord_delta[4]);
    prms_.mp_[prms_.mp_.size()-1].m[0][5] = angles::normalize_angle(double(EnvROBARM.goalHashEntry->coord[5] - parent->coord[5])*EnvROBARMCfg.coord_delta[5]);

    prms_.mp_[prms_.mp_.size()-1].coord[3] = EnvROBARM.goalHashEntry->coord[3] - parent->coord[3];
    prms_.mp_[prms_.mp_.size()-1].coord[4] = EnvROBARM.goalHashEntry->coord[4] - parent->coord[4];
    prms_.mp_[prms_.mp_.size()-1].coord[5] = EnvROBARM.goalHashEntry->coord[5] - parent->coord[5];

    ROS_DEBUG("Adaptive MP: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f  Coord: %d %d %d (dist: %d)",prms_.mp_[prms_.mp_.size()-1].m[0][0],prms_.mp_[prms_.mp_.size()-1].m[0][1],prms_.mp_[prms_.mp_.size()-1].m[0][2],prms_.mp_[prms_.mp_.size()-1].m[0][3],prms_.mp_[prms_.mp_.size()-1].m[0][4],prms_.mp_[prms_.mp_.size()-1].m[0][5],prms_.mp_[prms_.mp_.size()-1].m[0][6],  prms_.mp_[prms_.mp_.size()-1].coord[3], prms_.mp_[prms_.mp_.size()-1].coord[4], prms_.mp_[prms_.mp_.size()-1].coord[5], dist_to_goal);
  }

  ROS_DEBUG("[Source: %d] xyz: %.3f %.3f %.3f  rpy: %.3f %.3f %.3f fa: %.3f",SourceStateID,xyz_source[0],xyz_source[1],xyz_source[2],rpy_source[0],rpy_source[1],rpy_source[2],fa_source);

  //iterate through successors of source state
  for (i = 0; i < prms_.mp_.size(); i++)
  {
    //if adaptive mprim, are we at goal yet?
    if(prms_.mp_[i].type == sbpl_arm_planner::ADAPTIVE)
    {
      if(dist_to_goal > 0)
        continue;
    }
    //is primitive of right type? (short or long distance) 
    else if(prms_.mp_[i].type != use_prim_type)
      continue;

    dist = 100;
    invalid_prim = false;
    motion_cost = 0;

    for(q = 0; q < parent->coord.size(); ++q)
    {
      succcoord[q] = parent->coord[q];
      source_angles[q] = parent_angles[q];
    }

    iangles = pangles = source_angles;
    
    coordToWorldPose(parent->coord, parent_wcoord);

    mp_coord = prms_.mp_[i].coord;

    //get the successor
    EnvCARTROBARM3DHashEntry_t* OutHashEntry;
    bool bSuccisGoal = false;

    //loop through the waypoints in the motion primitive
    for(j = 0; j < prms_.mp_[i].m.size(); ++j)
    {
      //add the motion primitive to the current joint configuration
      for(a = 0; a < prms_.mp_[i].m[j].size(); ++a)
        succ_wcoord[a] = parent_wcoord[a] + prms_.mp_[i].m[j][a];

      ROS_DEBUG("[%d] -> [%d-%d] xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f fa: %0.3f ", SourceStateID, int(i), int(j), succ_wcoord[0],succ_wcoord[1],succ_wcoord[2],succ_wcoord[3],succ_wcoord[4],succ_wcoord[5],succ_wcoord[6]);

      //run IK - does solution exist?
      if(!convertWorldPoseToAngles(succ_wcoord, sangles))
      {
        ROS_DEBUG("[%d] -> [%d-%d] Failed to convert coords into angles. IK failed.", SourceStateID, int(i), int(j));
        
        std::vector<double> dpose(6,0), dseed(7,0);
        dseed = parent_angles;
        dseed[2] = succ_wcoord[6];
        for(size_t u = 0; u < 6; ++u)
          dpose[u] = succ_wcoord[u];

        //first try letting IK change the free angle
        //clock_t dclock = clock();
        if(!arm_->computeIK(dpose, dseed, sangles))
        {
          //fprintf(fMP, "%d %d %d %d %d %d %f %f %f %f\n", SourceStateID, int(i), int(j), 0, 0, 0, succ_wcoord[6], 0.0, 0.0, 1000.0*(clock() - dclock) / (double)CLOCKS_PER_SEC); 
          //ROS_INFO("[%d-%d] Failed IK search.    FA: %0.3f  Time: %0.4fms", int(i), int(j), succ_wcoord[6], 1000.0*(clock() - dclock) / (double)CLOCKS_PER_SEC);
 
          invalid_prim = true;
          break;
         
          //now try letting IK ignore the RPY
          /*
          if(!arm_->computeTranslationalIK(dpose, succ_wcoord[6], sangles))
            ROS_INFO("  [%d-%d] Failed translational IK.", int(i), int(j));
          else
            ROS_INFO("  [%d-%d] Succeeded translational IK.", int(i), int(j));
          */
        }
        else
        {
          //fprintf(fMP, "%d %d %d %d %d %d %f %f %f %f\n", SourceStateID, int(i), int(j), 0, 0, 1, succ_wcoord[6], sangles[2], fabs(succ_wcoord[6]-sangles[2]), 1000.0*(clock() - dclock) / (double)CLOCKS_PER_SEC); 
          //ROS_DEBUG("[%d-%d] Succeeded IK search. FA_Desired: %0.3f  FA_Solved: %0.3f (diff: %0.3f)  Time: %0.4fms", int(i), int(j), succ_wcoord[6], sangles[2], succ_wcoord[6] - sangles[2], 1000.0*(clock() - dclock) / (double)CLOCKS_PER_SEC);
        
          //store the modified end pose of the mprim
          if(j == prms_.mp_[i].m.size()-1)
          {
            short unsigned int fa;
            worldToDiscFAngle(sangles[2], &fa);
            mp_coord[6] = int(parent->coord[6]) - int(fa);
            //ROS_DEBUG("Adapted Mprim: %d %d %d %d %d %d %d -> %d %d %d %d %d %d %d", prms_.mp_[i].coord[0],prms_.mp_[i].coord[1],prms_.mp_[i].coord[2],prms_.mp_[i].coord[3],prms_.mp_[i].coord[4],prms_.mp_[i].coord[5],prms_.mp_[i].coord[6],mp_coord[0],mp_coord[1],mp_coord[2],mp_coord[3],mp_coord[4],mp_coord[5],mp_coord[6]);
          }
        }
      }
      //else
      //  fprintf(fMP, "%d %d %d %d %d %d %f %f %f %f\n", SourceStateID, int(i), int(j), 1, 0, 0, succ_wcoord[6], 0.0, 0.0, 0.0); 

      
      //check joint limits
      if(!arm_->checkJointLimits(sangles, prms_.verbose_))
      {
        //fprintf(fMP, "%d %d %d %d %d %d %f %f %f %f\n", SourceStateID, int(i), int(j), 0, 1, 0, 0.0, 0.0, 0.0, 0.0); 
        invalid_prim = true;
        break;
      }

      //check for collisions
      if(!cspace_->checkCollision(sangles, prms_.verbose_, false, dist_temp))
      {
#if DEBUG_SEARCH
        SBPL_DEBUG_NAMED("search", " succ: %2d  dist: %2d is in collision.", i, int(dist_temp));
#endif
        //fprintf(fMP, "%d %d %d %d %d %d %f %f %f %f\n", SourceStateID, int(i), int(j), 0, 2, 0, 0.0, 0.0, 0.0, 0.0); 
        invalid_prim = true;
        break;
      }
    
      // check for collision along interpolated path between sourcestate and succ
      if(!cspace_->checkPathForCollision(iangles, sangles, prms_.verbose_, dist_temp))
      {
#if DEBUG_SEARCH
        SBPL_DEBUG_NAMED("search", " succ: %2d  dist: %2d is in collision along interpolated path.", i, int(dist_temp));
#endif
        //fprintf(fMP, "%d %d %d %d %d %d %f %f %f %f\n", SourceStateID, int(i), int(j), 0, 3, 0, 0.0, 0.0, 0.0, 0.0); 
        invalid_prim = true;
        break;
      }

      if(dist_temp < dist)
        dist = dist_temp;

      // add up motion cost for all waypoints in mprim
      motion_cost += computeMotionCost(iangles, sangles);

      //save joint angles of current waypoint for next waypoint in mprim
      iangles = sangles;

      ROS_DEBUG_NAMED("search", "     [%d-%d] xyz: %0d %d %d rpy: %d %d %d fa: %d {%0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f}", int(i), int(j), succcoord[0],succcoord[1],succcoord[2],succcoord[3],succcoord[4],succcoord[5],succcoord[6],angles[0],angles[1],angles[2],angles[3],angles[4],angles[5],angles[6]);
    }
    
    if(invalid_prim)
    {
      //fprintf(fMP2, "%d %d %d %d\n", SourceStateID, int(i), int(j), 0); 
      continue;
    }
    /*
    else
    {
      fprintf(fMP2, "%d %d %d %d\n", SourceStateID, int(i), int(j), 1); 
    }
    */

    //compute the successor coords
    for(a = 0; a < mp_coord.size(); ++a)
    { 
      if((succcoord[a] + mp_coord[a]) < 0)
        succcoord[a] = ((EnvROBARMCfg.coord_vals[a] + succcoord[a] + int(mp_coord[a])) % EnvROBARMCfg.coord_vals[a]);
      else
        succcoord[a] = ((int)(succcoord[a] + int(mp_coord[a])) % EnvROBARMCfg.coord_vals[a]);
    }

    ROS_DEBUG("mprim: %d parent: %d %d %d  succ: %d %d %d  mprim: %d %d %d", int(i), parent->coord[0], parent->coord[1],parent->coord[2],succcoord[0],succcoord[1],succcoord[2], int(mp_coord[0]), int(mp_coord[1]), int(mp_coord[2]));

    /* write a function that makes this conversion smarter */
    coordToPose(succcoord, xyz, rpy, &fa);
    discToWorldXYZ(xyz,wxyz);
    discToWorldRPY(rpy,wrpy);
    discToWorldFAngle(fa,&wfa);

    //check if this state meets the goal criteria
    if(isGoalPosition(wxyz, wrpy, wfa))
    {
      bSuccisGoal = true;

      for (int j = 0; j < ndof_; j++)
        EnvROBARM.goalHashEntry->coord[j] = succcoord[j];
      
      EnvROBARM.goalHashEntry->dist = dist;
      EnvROBARM.goalHashEntry->angles = sangles;

      SBPL_INFO("[search] Goal state has been found. Parent StateID: %d (obstacle distance: %d)",SourceStateID, int(dist));
      SBPL_INFO("[search]   coord: %d %d %d %d %d %d %d", succcoord[0], succcoord[1],succcoord[2],succcoord[3],succcoord[4],succcoord[5],succcoord[6]);
    }

    //check if hash entry already exists, if not then create one
    if((OutHashEntry = getHashEntry(succcoord, bSuccisGoal)) == NULL)
    {
      OutHashEntry = createHashEntry(succcoord);
      OutHashEntry->dist = dist;
      OutHashEntry->angles = sangles;

//#if DEBUG_SEARCH
      if(prms_.mp_[i].type == sbpl_arm_planner::ADAPTIVE)
        motion_cost = 1;

      ROS_INFO_NAMED("search", " parentid: %d  stateid: %d  mprim: %d  cost: %5d heur: %2d  xyz: %3d %3d %3d  rpy: %3d %3d %3d  fa: %3d", SourceStateID, OutHashEntry->stateID, int(i),  motion_cost, GetFromToHeuristic(OutHashEntry->stateID, EnvROBARM.goalHashEntry->stateID), OutHashEntry->coord[0],OutHashEntry->coord[1],OutHashEntry->coord[2], OutHashEntry->coord[3],OutHashEntry->coord[4],OutHashEntry->coord[5],OutHashEntry->coord[6]);
//#endif
    }

    if(prms_.mp_[i].type == sbpl_arm_planner::ADAPTIVE)
        motion_cost = 1;

    //put successor on successor list with the proper cost
    SuccIDV->push_back(OutHashEntry->stateID);
    CostV->push_back(motion_cost); //(cost(parent,OutHashEntry,bSuccisGoal));
    if(ActionV != NULL)
      ActionV->push_back(i);
  }

  if(save_expanded_states)
    expanded_states.push_back(SourceStateID);
}

void EnvironmentCARTROBARM3D::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{
  SBPL_ERROR("ERROR in EnvROBARM... function: GetPreds is undefined\n");
  throw new SBPL_Exception();
}

bool EnvironmentCARTROBARM3D::AreEquivalent(int StateID1, int StateID2)
{
  SBPL_ERROR("Error: AreEquivalent() is undefined.");
  return false;
}

void EnvironmentCARTROBARM3D::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
  SBPL_ERROR("Error: SetAllActionsandOutcomes is undefined.");
  throw new SBPL_Exception();
}

void EnvironmentCARTROBARM3D::SetAllPreds(CMDPSTATE* state)
{
  //implement this if the planner needs access to predecessors
  SBPL_ERROR("Error: SetAllPreds is undefined.");
  throw new SBPL_Exception();
}

/////////////////////////////////////////////////////////////////////////////
//                      End of SBPL Planner Interface
/////////////////////////////////////////////////////////////////////////////

void EnvironmentCARTROBARM3D::printHashTableHist()
{
  int s0=0, s1=0, s50=0, s100=0, s200=0, s300=0, slarge=0;

  for(int  j = 0; j < EnvROBARM.HashTableSize; j++)
  {
    if((int)EnvROBARM.Coord2StateIDHashTable[j].size() == 0)
      s0++;
    else if((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 50)
      s1++;
    else if((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 100)
      s50++;
    else if((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 200)
      s100++;
    else if((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 300)
      s200++;
    else if((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 400)
      s300++;
    else
      slarge++;
  }
  SBPL_DEBUG("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d",
      s0,s1, s50, s100, s200,s300,slarge);
}

EnvCARTROBARM3DHashEntry_t* EnvironmentCARTROBARM3D::getHashEntry(const std::vector<short unsigned int> &coord, bool bIsGoal)
{
  //if it is goal
  if(bIsGoal)
    return EnvROBARM.goalHashEntry;

  int binid = getHashBin(coord);

  //iterate over the states in the bin and select the perfect match
  for(size_t ind = 0; ind < EnvROBARM.Coord2StateIDHashTable[binid].size(); ind++)
  {
    size_t j = 0;
    
    for(j=0; j<coord.size(); j++)
    {
      if(EnvROBARM.Coord2StateIDHashTable[binid][ind]->coord[j] != coord[j])
        break;
    }

    if (j == 7)
      return EnvROBARM.Coord2StateIDHashTable[binid][ind];
  }

  return NULL;
}

EnvCARTROBARM3DHashEntry_t* EnvironmentCARTROBARM3D::getHashEntry(short unsigned int *xyz, short unsigned int *rpy, short unsigned int fangle, bool bIsGoal)
{
  std::vector<short unsigned int> coord(ndof_,0);
  coord[0] = xyz[0];
  coord[1] = xyz[1];
  coord[2] = xyz[2];
  coord[3] = rpy[0];
  coord[4] = rpy[1];
  coord[5] = rpy[2];
  coord[6] = fangle;

  return getHashEntry(coord,bIsGoal);
}

EnvCARTROBARM3DHashEntry_t* EnvironmentCARTROBARM3D::createHashEntry(const std::vector<short unsigned int> &coord)
{
  int i=0;

  EnvCARTROBARM3DHashEntry_t* HashEntry = new EnvCARTROBARM3DHashEntry_t;

  HashEntry->coord = coord;
 
  HashEntry->dist = 200.0;

  HashEntry->angles.resize(ndof_,-1);

  //assign a stateID to HashEntry to be used 
  HashEntry->stateID = EnvROBARM.StateID2CoordTable.size();

  //insert into the tables
  EnvROBARM.StateID2CoordTable.push_back(HashEntry);

  //get the hash table bin
  i = getHashBin(HashEntry->coord);

  //insert the entry into the bin
  EnvROBARM.Coord2StateIDHashTable[i].push_back(HashEntry);

  //insert into and initialize the mappings
  int* entry = new int [NUMOFINDICES_STATEID2IND];
  StateID2IndexMapping.push_back(entry);
  for(int i = 0; i < NUMOFINDICES_STATEID2IND; i++)
    StateID2IndexMapping[HashEntry->stateID][i] = -1;

  if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1)
  {
    SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID");
    throw new SBPL_Exception();
  }

  return HashEntry;
}

void EnvironmentCARTROBARM3D::initDijkstra()
{
  int dimX,dimY,dimZ;
  grid_->getGridSize(dimX, dimY, dimZ);

  dijkstra_ = new sbpl_arm_planner::BFS3D(dimX, dimY, dimZ, int(arm_->getLinkRadiusCells(2)), prms_.cost_per_cell_);

  dijkstra_->configDistanceField(true, grid_->getDistanceFieldPtr());

#if DEBUG_SEARCH
  if(prms_.verbose_)
    dijkstra_->printConfig(fSucc);
#endif 
  SBPL_DEBUG("[initDijkstra] BFS is initialized.");
}

int EnvironmentCARTROBARM3D::cost(EnvCARTROBARM3DHashEntry_t* HashEntry1, EnvCARTROBARM3DHashEntry_t* HashEntry2, bool bState2IsGoal)
{
 // if(bState2IsGoal)
 // 	return 0;

  if(prms_.use_uniform_cost_)
    return prms_.cost_multiplier_;
  else
  {
    // Max's suggestion is to just put a high cost on being close to
    // obstacles but don't provide some sort of gradient 
    if(int(HashEntry2->dist) < 6) //in cells of resolution_cc_
      return prms_.cost_multiplier_ * 5;
    else
      return prms_.cost_multiplier_;
  }
}

bool EnvironmentCARTROBARM3D::initEnvConfig()
{
  std::vector<short unsigned int> coord(ndof_,0);

  //these probably don't have to be done
  EnvROBARMCfg.start_configuration.resize(ndof_);

  //CARTTODO
  EnvROBARMCfg.coord_delta.resize(ndof_,1);
  EnvROBARMCfg.coord_vals.resize(ndof_,360);

  EnvROBARMCfg.coord_delta[0] = prms_.xyz_resolution_;
  EnvROBARMCfg.coord_delta[1] = prms_.xyz_resolution_;
  EnvROBARMCfg.coord_delta[2] = prms_.xyz_resolution_;
  EnvROBARMCfg.coord_delta[3] = prms_.rpy_resolution_;
  EnvROBARMCfg.coord_delta[4] = prms_.rpy_resolution_;
  EnvROBARMCfg.coord_delta[5] = prms_.rpy_resolution_;
  EnvROBARMCfg.coord_delta[6] = prms_.fa_resolution_;

  EnvROBARMCfg.coord_vals[0] = prms_.sizeX_ / prms_.xyz_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[1] = prms_.sizeY_ / prms_.xyz_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[2] = prms_.sizeZ_ / prms_.xyz_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[3] = (2.0*M_PI) / prms_.rpy_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[4] = (2.0*M_PI) / prms_.rpy_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[5] = (2.0*M_PI) / prms_.rpy_resolution_ + 0.5;
  EnvROBARMCfg.coord_vals[6] = (2.0*M_PI) / prms_.fa_resolution_ + 0.5;

  SBPL_INFO("Discretization of Statespace:");
  for(int i = 0; i < ndof_; ++i)
    SBPL_INFO("[%d] delta: %0.3f  vals: %d",i, EnvROBARMCfg.coord_delta[i],EnvROBARMCfg.coord_vals[i]);

  //initialize the map from Coord to StateID
  EnvROBARM.HashTableSize = 32*1024; //should be power of two
  EnvROBARM.Coord2StateIDHashTable = new std::vector<EnvCARTROBARM3DHashEntry_t*>[EnvROBARM.HashTableSize];

  //initialize the map from StateID to Coord
  EnvROBARM.StateID2CoordTable.clear();

  //create empty start & goal states
  EnvROBARM.startHashEntry = createHashEntry(coord);
  EnvROBARM.goalHashEntry = createHashEntry(coord);

  return true;
}

bool EnvironmentCARTROBARM3D::initArmModel(FILE* aCfg, const std::string robot_description)
{
  arm_ = new sbpl_arm_planner::SBPLArmModel(aCfg);

  arm_->setResolution(prms_.resolution_);

  if(robot_description.compare("ROS_PARAM") == 0)
  {
    if(arm_->initKDLChainFromParamServer())
      return true;
  }
  else
  {
    if(arm_->initKDLChain(robot_description))
      return true;
  }

  return false; 
}

/*
bool EnvironmentCARTROBARM3D::initEnvironment(FILE* pCfg, FILE* aCfg)
{
  //initialize the arm planner parameters
  prms_.initFromParamServer();

  //parse motion primitives file
  if(!prms_.initMotionPrimsFromFile(pCfg))
    return false;

  //initialize the arm model
  std::string test("ROS_PARAM");
  if(!initArmModel(aCfg,test))
    return false;

  //initialize the environment & planning variables  
  if(!initGeneral())
    return false;

  //set 'Environment is Initialized' flag
  EnvROBARMCfg.bInitialized = true;

  //for statistics purposes
  starttime = clock();

  prms_.printMotionPrims(stdout);

  SBPL_INFO("[initEnvironment] Environment has been initialized.");
  return true;
}
*/

bool EnvironmentCARTROBARM3D::initEnvironment(std::string arm_description_filename, std::string mprims_filename)
{
  FILE* mprims_fp=NULL;
  FILE* arm_fp=NULL;

  //initialize the arm planner parameters
  prms_.initFromParamServer();

  //parse motion primitives file
  if((mprims_fp=fopen(mprims_filename.c_str(),"r")) == NULL)
  {
    SBPL_ERROR("Failed to open motion primitive file.");
    return false;
  }
  if(!prms_.initLongMotionPrimsFromFile(mprims_fp))
  {
    SBPL_ERROR("Failed to parse motion primitive file.");
    fclose(mprims_fp);
    return false;
  }
  fclose(mprims_fp);

  //initialize the arm model
  if((arm_fp=fopen(arm_description_filename.c_str(),"r")) == NULL)
  {
    SBPL_ERROR("Failed to open arm description file.");
    return false;
  }
  std::string ros_param("ROS_PARAM");
  if(!initArmModel(arm_fp,ros_param))
  {
    SBPL_ERROR("Failed to initialize arm model.");
    fclose(arm_fp);
    return false;
  }
  fclose(arm_fp);

  //initialize the environment & planning variables  
  if(!initGeneral())
  {
    SBPL_ERROR("Failed to initialize environment.");
    return false;
  }

  //set 'Environment is Initialized' flag
  EnvROBARMCfg.bInitialized = true;

  SBPL_INFO("[initEnvironment] Environment has been initialized.");

  //for statistics purposes
  starttime = clock();

  return true;
}

bool EnvironmentCARTROBARM3D::initGeneral()
{
  //create the occupancy grid
  grid_ = new sbpl_arm_planner::OccupancyGrid(prms_.sizeX_,prms_.sizeY_,prms_.sizeZ_, prms_.resolution_,prms_.originX_,prms_.originY_,prms_.originZ_);

  //create the collision space
  cspace_ = new sbpl_arm_planner::SBPLCollisionSpace(arm_, grid_);

#if DEBUG_SEARCH
  cspace_->setDebugFile(fSucc);
  arm_->setDebugFile(std::string("sbpl"));
#endif

  //initialize Environment
  if(initEnvConfig() == false)
    return false;

  //compute the cost per cell to be used by heuristic
  computeCostPerCell();

  //initialize dijkstra 
  initDijkstra();

  if(prms_.verbose_)
  {
    arm_->printArmDescription(std::string("sbpl_cartesian_arm"));
    prms_.printParams(std::string("sbpl_cartesian_arm"));
    prms_.printLongMotionPrims(std::string("sbpl_cartesian_arm"));
  }

  return true;
}

double EnvironmentCARTROBARM3D::getEpsilon()
{
  return prms_.epsilon_;
}

void EnvironmentCARTROBARM3D::readConfiguration(FILE* fCfg)
{
  char sTemp[1024];
  int i;
  std::vector<double> cube(6,0);

  if(fscanf(fCfg,"%s",sTemp) < 1) 
    SBPL_DEBUG("Parsed string has length < 1.\n");
  while(!feof(fCfg) && strlen(sTemp) != 0)
  {
    if(strcmp(sTemp, "linkstartangles(radians):") == 0)
    {
      EnvROBARMCfg.start_configuration.resize(arm_->num_joints_,0);
      for(i = 0; i < arm_->num_joints_; i++)
      {
        if(fscanf(fCfg,"%s",sTemp) < 1) 
          SBPL_DEBUG("Parsed string has length < 1.\n");
        EnvROBARMCfg.start_configuration[i] = atof(sTemp);
      }
    }
    else if(strcmp(sTemp, "endeffectorgoal(meters):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_DEBUG("Parsed string has length < 1.\n");
      EnvROBARMCfg.ParsedGoals.resize(atoi(sTemp));
      EnvROBARMCfg.ParsedGoalTolerance.resize(1);
      EnvROBARMCfg.ParsedGoalTolerance[0].resize(2);
      EnvROBARMCfg.ParsedGoalTolerance[0][0] = 0.02;
      EnvROBARMCfg.ParsedGoalTolerance[0][1] = 0.10;

      for(unsigned int i = 0; i < EnvROBARMCfg.ParsedGoals.size(); i++)
      {
        EnvROBARMCfg.ParsedGoals[i].resize(7);

        for(unsigned int k = 0; k < 7; k++)
        {
          if(fscanf(fCfg,"%s",sTemp) < 1) 
            SBPL_DEBUG("Parsed string has length < 1.\n");
          EnvROBARMCfg.ParsedGoals[i][k] = atof(sTemp);
        }
      }
    }
    else if(strcmp(sTemp, "goal_tolerance(meters,radians):") == 0)
    {
      EnvROBARMCfg.ParsedGoalTolerance.resize(1);
      EnvROBARMCfg.ParsedGoalTolerance[0].resize(2);

      //distance tolerance (m)
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_DEBUG("Parsed string has length < 1.\n");
      EnvROBARMCfg.ParsedGoalTolerance[0][0] = atof(sTemp);
      //orientation tolerance (rad)
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_DEBUG("Parsed string has length < 1.\n");
      EnvROBARMCfg.ParsedGoalTolerance[0][1] = atof(sTemp);
    }
    else if(strcmp(sTemp, "cube:") == 0)
    {
      for(int j = 0; j < 6; j++)
      {
        if(fscanf(fCfg,"%s",sTemp) < 1)
          SBPL_DEBUG("Parsed string has length < 1.\n");
        cube[j] = atof(sTemp);
      }
      EnvROBARMCfg.obstacles.push_back(cube);
    }
    else
      SBPL_WARN("ERROR: Unknown parameter name in environment config file: %s.\n", sTemp);

    if(fscanf(fCfg,"%s",sTemp) < 1) 
      SBPL_DEBUG("Parsed string has length < 1.\n");
  }
}

bool EnvironmentCARTROBARM3D::isGoalPosition(double *xyz, double *rpy, double fangle)
{
  //check position
  if(fabs(xyz[0]-EnvROBARMCfg.goal.xyz[0]) <= EnvROBARMCfg.goal.xyz_tol-0.005 &&
      fabs(xyz[1]-EnvROBARMCfg.goal.xyz[1]) <= EnvROBARMCfg.goal.xyz_tol-0.005 &&
      fabs(xyz[2]-EnvROBARMCfg.goal.xyz[2]) <= EnvROBARMCfg.goal.xyz_tol-0.005)
  {
    //log the amount of time required for the search to get close to the goal
    if(!near_goal && save_expanded_states)
    {
      time_to_goal_region = (clock() - starttime) / (double)CLOCKS_PER_SEC;
      near_goal = true;
      SBPL_INFO("Search is within %0.3f meters of the goal (%0.3f %0.3f %0.3f) after %.4f sec. (after %d expansions)", EnvROBARMCfg.goal.xyz_tol, EnvROBARMCfg.goal.xyz[0], EnvROBARMCfg.goal.xyz[1], EnvROBARMCfg.goal.xyz[2], time_to_goal_region, (int)expanded_states.size());
    }
    //check orientation
    if (fabs(rpy[0]-EnvROBARMCfg.goal.rpy[0]) <= EnvROBARMCfg.goal.roll_tol &&
        fabs(rpy[1]-EnvROBARMCfg.goal.rpy[1]) <= EnvROBARMCfg.goal.pitch_tol &&
        fabs(rpy[2]-EnvROBARMCfg.goal.rpy[2]) <= EnvROBARMCfg.goal.yaw_tol)
      return true;

    //ROS_WARN("xyz goal is met. rpy isn't. rpy: %0.3f %0.3f %0.3f  tolerance: %0.3f", rpy[0],rpy[1],rpy[2],EnvROBARMCfg.goal.roll_tol);
  }
  return false;
}

int EnvironmentCARTROBARM3D::getActionCost(const std::vector<double> &from_config, const std::vector<double> &to_config, int dist)
{
  SBPL_ERROR("getActionCost() is not yet implemented.");
  return 1;
}

int EnvironmentCARTROBARM3D::getEdgeCost(int FromStateID, int ToStateID)
{
#if DEBUG
  if(FromStateID >= (int)EnvROBARM.StateID2CoordTable.size() 
      || ToStateID >= (int)EnvROBARM.StateID2CoordTable.size())
  {
    SBPL_ERROR("ERROR in EnvROBARM... function: stateID illegal\n");
    throw new SBPL_Exception();
  }
#endif

  //get X, Y for the state
  EnvCARTROBARM3DHashEntry_t* FromHashEntry = EnvROBARM.StateID2CoordTable[FromStateID];
  EnvCARTROBARM3DHashEntry_t* ToHashEntry = EnvROBARM.StateID2CoordTable[ToStateID];

  return cost(FromHashEntry, ToHashEntry, false);
}

bool EnvironmentCARTROBARM3D::setStartConfiguration(const std::vector<double> &angles)
{
  if(int(angles.size()) < ndof_)
    return false;

  //check joint limits of starting configuration but plan anyway
  if(!arm_->checkJointLimits(angles, true))
    SBPL_WARN("Starting configuration violates the joint limits. Attempting to plan anyway.");

  //set start position
  anglesToCoord(angles, EnvROBARM.startHashEntry->coord);

  EnvROBARM.startHashEntry->angles = angles;

  ROS_INFO("[start state] xyz: %d %d %d rpy: %d %d %d angle: %d",EnvROBARM.startHashEntry->coord[0],EnvROBARM.startHashEntry->coord[1],EnvROBARM.startHashEntry->coord[2],EnvROBARM.startHashEntry->coord[3],EnvROBARM.startHashEntry->coord[4],EnvROBARM.startHashEntry->coord[5],EnvROBARM.startHashEntry->coord[6]);

  return true;
}

bool EnvironmentCARTROBARM3D::setGoalPosition(const std::vector<std::vector<double> > &goals, const std::vector<std::vector<double> > &tolerances)
{
  //goals: {{x1,y1,z1,r1,p1,y1,fangle,is_6dof},...}

  if(!EnvROBARMCfg.bInitialized)
  {
    SBPL_ERROR("Cannot set goal position because environment is not initialized.");
    return false;
  }

  if(goals.empty())
  {
    SBPL_ERROR("[setGoalPosition] No goal constraint set.");
    return false;
  }

  // debugging - check if an IK solution exists for the goal pose before we do the search
  // we plan even if there is no solution
  std::vector<double> pose(7,0), jnt_angles(7,0), ik_solution(7,0);
  pose = goals[0];
  if(!arm_->computeIK(pose, jnt_angles, ik_solution))
    SBPL_WARN("[setGoalPosition] No valid IK solution for the goal pose.");

  // only supports a single goal
  EnvROBARMCfg.goal.xyz[0] = goals[0][0];
  EnvROBARMCfg.goal.xyz[1] = goals[0][1];
  EnvROBARMCfg.goal.xyz[2] = goals[0][2];
  EnvROBARMCfg.goal.rpy[0] = goals[0][3];
  EnvROBARMCfg.goal.rpy[1] = goals[0][4];
  EnvROBARMCfg.goal.rpy[2] = goals[0][5];
  EnvROBARMCfg.goal.fangle = goals[0][6];

  EnvROBARMCfg.goal.xyz_tol = tolerances[0][0];
  EnvROBARMCfg.goal.roll_tol = tolerances[0][3];
  EnvROBARMCfg.goal.pitch_tol = tolerances[0][4];
  EnvROBARMCfg.goal.yaw_tol = tolerances[0][5];

  EnvROBARMCfg.goal.type = goals[0][7];
  prms_.use_6d_pose_goal_ = goals[0][7];

  worldPoseToState(EnvROBARMCfg.goal.xyz, EnvROBARMCfg.goal.rpy, EnvROBARMCfg.goal.fangle, true, EnvROBARM.goalHashEntry);

  if(!prms_.use_6d_pose_goal_)
    SBPL_DEBUG("[setGoalPosition] Goal position constraint set. No goal orientation constraint requested.\n");

  SBPL_INFO("[goal]");
  SBPL_INFO(" xyz: %.2f %.2f %.2f (meters) (tol: %.3fm)", EnvROBARMCfg.goal.xyz[0],EnvROBARMCfg.goal.xyz[1],EnvROBARMCfg.goal.xyz[2],EnvROBARMCfg.goal.xyz_tol);
  SBPL_INFO(" rpy: %1.2f %1.2f %1.2f (radians) (tol: %.3frad)", EnvROBARMCfg.goal.rpy[0],EnvROBARMCfg.goal.rpy[1],EnvROBARMCfg.goal.rpy[2],EnvROBARMCfg.goal.roll_tol);
  SBPL_INFO("grid: %u %u %u (cells)", EnvROBARM.goalHashEntry->coord[0], EnvROBARM.goalHashEntry->coord[1], EnvROBARM.goalHashEntry->coord[2]);

  // precompute heuristics
  if(!precomputeHeuristics())
  {
    SBPL_ERROR("Precomputing heuristics failed. Exiting.");
    return false;
  }

  return true;
}

bool EnvironmentCARTROBARM3D::precomputeHeuristics()
{
  std::vector<short unsigned int> dij_goal(3,0);
  dij_goal[0] = EnvROBARM.goalHashEntry->coord[0];
  dij_goal[1] = EnvROBARM.goalHashEntry->coord[1];
  dij_goal[2] = EnvROBARM.goalHashEntry->coord[2];

  //set the goal for h_endeff
  if(!dijkstra_->setGoal(dij_goal))
  {
    SBPL_ERROR("[precomputeHeuristics] Failed to set goal for Dijkstra search.");
    return false;
  }

  //precompute h_endeff
  if(!dijkstra_->runBFS())
  {
    SBPL_ERROR("[precomputeHeuristics] Precomputing the BFS for the end-effector heuristic failed. Exiting.");
    return false;
  }

  return true;
}

void EnvironmentCARTROBARM3D::clearStats()
{
  //a flag used for debugging only
  near_goal = false;

  //clear lists of stateIDs (for debugging only)
  expanded_states.clear();

  //start the 'planning time' clock
  starttime = clock();
}

void EnvironmentCARTROBARM3D::stateIDToPose(int stateID, short unsigned int *xyz, short unsigned int *rpy, short unsigned int *fangle)
{
  EnvCARTROBARM3DHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[stateID];

  if(stateID == EnvROBARM.goalHashEntry->stateID)
  {
    xyz[0] = EnvROBARM.goalHashEntry->coord[0];
    xyz[1] = EnvROBARM.goalHashEntry->coord[1];
    xyz[2] = EnvROBARM.goalHashEntry->coord[2];
    rpy[0] = EnvROBARM.goalHashEntry->coord[3];
    rpy[1] = EnvROBARM.goalHashEntry->coord[4];
    rpy[2] = EnvROBARM.goalHashEntry->coord[5];
    *fangle = EnvROBARM.goalHashEntry->coord[6];
  }
  else
  {
    xyz[0] = HashEntry->coord[0];
    xyz[1] = HashEntry->coord[1];
    xyz[2] = HashEntry->coord[2];
    rpy[0] = HashEntry->coord[3];
    rpy[1] = HashEntry->coord[4];
    rpy[2] = HashEntry->coord[5];
    *fangle = HashEntry->coord[6];
  }

  //SBPL_INFO("xyz: %u %u %u  rpy: %u %u %u  fa: %u",xyz[0],xyz[1],xyz[2],rpy[0],rpy[1],rpy[2],*fangle);
}

void EnvironmentCARTROBARM3D::stateIDToWorldPose(int stateID, double *xyz, double *rpy, double *fangle)
{
  short unsigned int dxyz[3]={0}, drpy[3]={0}, dfangle=0;

  stateIDToPose(stateID, dxyz, drpy, &dfangle);

  discToWorldXYZ(dxyz, xyz);
  discToWorldRPY(drpy, rpy);
  discToWorldFAngle(dfangle, fangle);
}

void EnvironmentCARTROBARM3D::worldPoseToCoord(double *wxyz, double *wrpy, double wfangle, std::vector<short unsigned int> &coord)
{
  short unsigned int xyz[3]={0}, rpy[3]={0}, fangle=0;

  worldToDiscXYZ(wxyz, xyz);
  worldToDiscRPY(wrpy, rpy);
  worldToDiscFAngle(wfangle, &fangle);

  coord[0] = xyz[0];
  coord[1] = xyz[1];
  coord[2] = xyz[2];
  coord[3] = rpy[0];
  coord[4] = rpy[1];
  coord[5] = rpy[2];
  coord[6] = fangle;
}

void EnvironmentCARTROBARM3D::worldPoseToState(double *wxyz, double *wrpy, double wfangle, bool is_goal, EnvCARTROBARM3DHashEntry_t *state)
{
  std::vector<short unsigned int> coord(7,0);

  worldPoseToCoord(wxyz,wrpy,wfangle,coord);

  if((state = getHashEntry(coord,is_goal)) == NULL)
    state = createHashEntry(coord);
  else
    state->coord = coord;
}

void EnvironmentCARTROBARM3D::anglesToCoord(const std::vector<double> &angles, std::vector<short unsigned int> &coord)
{
  std::vector<double> pose(6,0);
  
  arm_->getPlanningJointPose(angles, pose);
  
  double fangle=angles[free_angle_index_];
  double wxyz[3] = {pose[0], pose[1], pose[2]};
  double wrpy[3] = {pose[3], pose[4], pose[5]};

  worldPoseToCoord(wxyz,wrpy,fangle,coord);
}

void EnvironmentCARTROBARM3D::printJointArray(FILE* fOut, EnvCARTROBARM3DHashEntry_t* HashEntry, bool bGoal, bool bVerbose)
{
/* CARTTODO
  if(bGoal)
    coordToAngles(EnvROBARM.goalHashEntry->coord, angles);
  else
    coordToAngles(HashEntry->coord, angles);
*/

  SBPL_DEBUG_NAMED(fOut, "xyz: %-2d %-2d %-2d  rpy: %-2d %-2d %-2d  fangle: %-2d", HashEntry->coord[0], HashEntry->coord[1], HashEntry->coord[2], HashEntry->coord[3], HashEntry->coord[4], HashEntry->coord[5], HashEntry->coord[6]);
}

void EnvironmentCARTROBARM3D::computeCostPerCell()
{
  //prms_.setCellCost(prms_.cost_multiplier_);
  int cost_per_cell = prms_.cost_per_second_ * prms_.time_per_cell_;
  prms_.setCellCost(cost_per_cell);
  prms_.cost_per_meter_ = int(prms_.cost_per_cell_ / EnvROBARMCfg.xyz_resolution);
  ROS_INFO("Cost per cell: %d, Time per cell: %0.3fsec", cost_per_cell, prms_.time_per_cell_);
}

int EnvironmentCARTROBARM3D::computeMotionCost(const std::vector<double> &a, const std::vector<double> &b)
{
  double time = 0, time_max = 0;

  for(size_t i = 0; i < a.size(); ++i)
  {
    time = fabs(angles::normalize_angle(a[i]-b[i])) / prms_.joint_vel_[i];

   ROS_DEBUG("%d: a: %0.4f b: %0.4f dist: %0.4f vel:%0.4f time: %0.4f", int(i), a[i], b[i], fabs(angles::normalize_angle(a[i]-b[i])), prms_.joint_vel_[i], time);
 
   if(time > time_max)
     time_max = time;
  }

  ROS_DEBUG("motion cost: %d  max_time:%0.4f",  int(prms_.cost_per_second_ * time_max), time_max);

  return prms_.cost_per_second_ * time_max;
}

int EnvironmentCARTROBARM3D::getDijkstraDistToGoal(short unsigned int x, short unsigned int y, short unsigned int z) const
{
  return int(dijkstra_->getDist(x,y,z));
}

void EnvironmentCARTROBARM3D::updateOccupancyGridFromCollisionMap(const mapping_msgs::CollisionMap &collision_map)
{
  if(collision_map.boxes.empty())
  {
    SBPL_ERROR("[updateOccupancyGridFromCollisionMap] collision map received is empty.");
    return;
  }
  else
    SBPL_DEBUG("[updateOccupancyGridFromCollisionMap] updating distance field with collision map with %d boxes.", int(collision_map.boxes.size()));

  grid_->updateFromCollisionMap(collision_map);
}

std::vector<std::vector<double> > EnvironmentCARTROBARM3D::getShortestPath()
{
  std::vector<short unsigned int> start(3,0);
  std::vector<double> waypoint(3,0);
  std::vector<std::vector<int> > path;
  std::vector<std::vector<double> > dpath; 

  //compute a Dijkstra path to goal 
  if(prms_.use_dijkstra_heuristic_)
  {
    start[0] = EnvROBARM.startHashEntry->coord[0];
    start[1] = EnvROBARM.startHashEntry->coord[1];
    start[2] = EnvROBARM.startHashEntry->coord[2];

    if(!dijkstra_->getShortestPath(start, path))
    {
      ROS_WARN("Unable to retrieve shortest path.");
      return dpath;
    }

    for(int i=0; i < (int)path.size(); ++i)
    {
      grid_->gridToWorld(path[i][0], path[i][1], path[i][2], waypoint[0], waypoint[1], waypoint[2]);
      dpath.push_back(waypoint);
    }
  }
  //compute a straight line path to goal
  else
  {
    getBresenhamPath(EnvROBARM.startHashEntry->xyz,EnvROBARM.goalHashEntry->xyz,&path);
  }

  for(int i=0; i < int(path.size()); ++i)
  {
    grid_->gridToWorld(path[i][0], path[i][1], path[i][2], waypoint[0], waypoint[1], waypoint[2]);
    dpath.push_back(waypoint);
  }

  return dpath;
}

void EnvironmentCARTROBARM3D::getBresenhamPath(const short unsigned int a[],const short unsigned int b[], std::vector<std::vector<int> > *path)
{
  bresenham3d_param_t params;
  std::vector<int> nXYZ(3,0);

  //iterate through the points on the segment
  get_bresenham3d_parameters(a[0], a[1], a[2], b[0], b[1], b[2], &params);
  do {
    get_current_point3d(&params, &(nXYZ[0]), &(nXYZ[1]), &(nXYZ[2]));

    path->push_back(nXYZ);
  } while (get_next_point3d(&params));

  SBPL_DEBUG("[getBresenhamPath] Path has %d waypoints.",int(path->size()));
}

void EnvironmentCARTROBARM3D::visualizeOccupancyGrid()
{
  grid_->visualize();
}

void EnvironmentCARTROBARM3D::setReferenceFrameTransform(KDL::Frame f, std::string &name)
{
  arm_->setRefFrameTransform(f, name);
}

std::vector<double> EnvironmentCARTROBARM3D::getPlanningStats()
{
  /* {dist_from_goal_to_nearest_obs(m),x_error,y_error,z_error,roll_error,pitch_error,yaw_error} */

  //CARTTODO
  std::vector<double> stats;
  return stats;
}

int EnvironmentCARTROBARM3D::getEndEffectorHeuristic(int FromStateID, int ToStateID)
{
  int heur = 0;

  //get X, Y, Z for the state
  EnvCARTROBARM3DHashEntry_t* FromHashEntry = EnvROBARM.StateID2CoordTable[FromStateID];

  //get distance heuristic
  if(prms_.use_dijkstra_heuristic_)
    heur = dijkstra_->getDist(int(FromHashEntry->coord[0]),int(FromHashEntry->coord[1]),int(FromHashEntry->coord[2]));
  else
  {
    double xyz[3]={0}, rpy[3]={0}, fangle=0;
    stateIDToWorldPose(FromStateID, xyz, rpy, &fangle);
    heur =  getEuclideanDistance(xyz[0],xyz[1],xyz[2],EnvROBARMCfg.goal.xyz[0],EnvROBARMCfg.goal.xyz[1], EnvROBARMCfg.goal.xyz[2]) * prms_.cost_per_meter_;
  }

  return heur;
}

std::vector<int> EnvironmentCARTROBARM3D::debugExpandedStates()
{
  return expanded_states;
}

void EnvironmentCARTROBARM3D::getExpandedStates(std::vector<std::vector<double> > &ara_states)
{
/* CARTTODO
  std::vector<double> angles(arm_->num_joints_,0);
  std::vector<double>state(7,0);

  for(unsigned int i = 0; i < expanded_states.size(); ++i)
  {
    StateID2Angles(expanded_states[i],angles);
    arm_->getPlanningJointPose(angles,state);
    state[6] = EnvROBARM.StateID2CoordTable[expanded_states[i]]->heur;
    ara_states->push_back(state);
  }
*/
}

bool EnvironmentCARTROBARM3D::convertCoordToAngles(const std::vector<short unsigned int> *coord, std::vector<double> *angles)
{
  //ROS_INFO("[convertCoordToAngles] Converting:  xyz: %u %u %u  rpy: %u %u %u  a: %u",coord->at(0),coord->at(1),coord->at(2),coord->at(3),coord->at(4),coord->at(5),coord->at(6));

  double wxyz[3]={0}, wrpy[3]={0}, wfangle=0;
  std::vector<double> pose(6,0), seed(7,0);

  coordToWorldPose(*coord,wxyz,wrpy,&wfangle);
  pose[0] = wxyz[0];
  pose[1] = wxyz[1];
  pose[2] = wxyz[2];
  pose[3] = wrpy[0];
  pose[4] = wrpy[1];
  pose[5] = wrpy[2];
  seed[free_angle_index_] = wfangle;

  if(!arm_->computeFastIK(pose,seed,*angles))
  {
    ROS_DEBUG("computeFastIK failed to return a solution");

    /*
    if(!arm_->computeIK(pose, seed, angles))
    {
      ROS_DEBUG("IK Search found solution");
      return false;
    }
    */
  }

  return true;
}

bool EnvironmentCARTROBARM3D::convertWorldPoseToAngles(const std::vector<double> &wpose, std::vector<double> &angles)
{
  std::vector<double> pose(6,0), seed(7,0);

  for(size_t i = 0; i < 6; ++i)
    pose[i] = wpose[i];

  seed[free_angle_index_] = wpose[6];

  if(!arm_->computeFastIK(pose,seed,angles))
  {
    ROS_DEBUG("computeFastIK failed to return a solution");

    if(!arm_->computeIK(pose,seed,angles))
    {
      ROS_DEBUG("IK Search found solution");
      return false;
    }
  }

  return true;
}

void EnvironmentCARTROBARM3D::getArmChainRootLinkName(std::string &name)
{
  arm_->getArmChainRootLinkName(name);
}

void EnvironmentCARTROBARM3D::StateID2Angles(int stateID, std::vector<double> &angles)
{
  EnvCARTROBARM3DHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[stateID];

  if(stateID == EnvROBARM.goalHashEntry->stateID)
    angles = EnvROBARM.goalHashEntry->angles;
  else
    angles = HashEntry->angles;

  for (size_t i = 0; i < angles.size(); i++)
  {
    if(angles[i] >= M_PI)
      angles[i] = -2.0*M_PI + angles[i];
  }
}

void EnvironmentCARTROBARM3D::convertStateIDPathToJointAnglesPath(const std::vector<int> &idpath, std::vector<std::vector<double> > &path)
{
  int sourceid, targetid, bestcost, bestsucc;
  std::vector<int> cost, succid, action;
  std::vector<double> sangles(ndof_,0), interm_angles(ndof_,0), interm_wcoord(ndof_,0), source_wcoord(ndof_,0);
  std::vector<short unsigned int> interm_coord(ndof_,0);
  EnvCARTROBARM3DHashEntry_t* source_entry;
 
  path.clear();

  ROS_DEBUG("idpath has length %d", int(idpath.size()));
  for(size_t p = 0; p < idpath.size()-1; ++p)
  {
    sourceid = idpath[p];
    targetid = idpath[p+1];

    GetSuccs(sourceid, &succid, &cost, &action);

    bestcost = INFINITECOST;
    bestsucc = -1;

    for(size_t s = 0; s < succid.size(); ++s)
    {
      if(succid[s] == targetid && cost[s] <= bestcost)
      {
        bestcost = cost[s];
        bestsucc = action[s];
      }
    }

    if(bestsucc == -1)
      SBPL_ERROR("[%i] Successor not found for transition.", int(p));

    source_entry = EnvROBARM.StateID2CoordTable[sourceid];
    sangles = source_entry->angles;
   
    coordToWorldPose(source_entry->coord, source_wcoord);

    for(size_t i = 0; i < prms_.mp_[bestsucc].m.size(); ++i)
    {
      for(size_t a = 0; a < interm_angles.size(); ++a)
        interm_wcoord[a] = source_wcoord[a] + prms_.mp_[bestsucc].m[i][a];
       
      ROS_INFO("[%i-%i] sourceid: %d targetid: %d mprim: %d xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f fa: %0.3f",int(p),int(i), sourceid, targetid, bestsucc, interm_wcoord[0],interm_wcoord[1],interm_wcoord[2],interm_wcoord[3],interm_wcoord[4],interm_wcoord[5],interm_wcoord[6]);

      if(convertWorldPoseToAngles(interm_wcoord, interm_angles))
      {
        for(size_t q = 0; q < interm_angles.size(); ++q)
          interm_angles[q] = angles::normalize_angle(interm_angles[q]);

        path.push_back(interm_angles); 
      }
      else
      {
        SBPL_WARN("[%i-%i] Can't convert world coords to angles.",int(p),int(i));
        SBPL_WARN("This MP has %d steps", int(prms_.mp_[bestsucc].m.size()));
        SBPL_WARN("    MP: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", prms_.mp_[bestsucc].m[i][0], prms_.mp_[bestsucc].m[i][1], prms_.mp_[bestsucc].m[i][2], prms_.mp_[bestsucc].m[i][3], prms_.mp_[bestsucc].m[i][4], prms_.mp_[bestsucc].m[i][5], prms_.mp_[bestsucc].m[i][6]);
        SBPL_WARN("hash_entry->angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", sangles[0],sangles[1],sangles[2],sangles[3],sangles[4],sangles[5],sangles[6]);
        path.push_back(sangles);
      }
    }
  }
}

void EnvironmentCARTROBARM3D::convertStateIDPathToShortenedJointAnglesPath(const std::vector<int> &idpath, std::vector<std::vector<double> > &path, std::vector<int> &idpath_short)
{
  int sourceid, targetid, bestcost, bestsucc;
  std::vector<int> cost, succid, action, mp_path, idpath2;
  std::vector<double> sangles(ndof_,0), interm_angles(ndof_,0), interm_wcoord(ndof_,0), source_wcoord(ndof_,0);
  std::vector<short unsigned int> interm_coord(ndof_,0);
  EnvCARTROBARM3DHashEntry_t* source_entry;
  path.clear();
  idpath_short.clear();

  //retrieve list of motion primitives used
  for(size_t p = 0; p < idpath.size()-1; ++p)
  {
    sourceid = idpath[p];
    targetid = idpath[p+1];

    GetSuccs(sourceid, &succid, &cost, &action);

    bestcost = INFINITECOST;
    bestsucc = -1;

    for(size_t s = 0; s < succid.size(); ++s)
    {
      if(succid[s] == targetid && cost[s] <= bestcost)
      {
        bestcost = cost[s];
        bestsucc = action[s];
      }
    }

    if(bestsucc == -1)
    {
      SBPL_ERROR("[%i] When retrieving the path, successor not found for transition.", int(p));
      return;
    }
    mp_path.push_back(bestsucc);
  }
  //remove stateids that use the same mprim as previous stateid
  idpath2 = idpath;
  for(int p = int(mp_path.size()-1); p > 0; p--)
  {
    if(mp_path[p] == mp_path[p-1])
      idpath2[p-1] = -1;
  }

  //debugging
  for(size_t p = 0; p < idpath.size()-1; ++p)
    ROS_INFO("[%d] original path: %d  mprim: %d  adjusted path %d", int(p), idpath[p], mp_path[p], idpath2[p]);

  //get joint angles for shortened paths
  for(size_t p = 0; p < idpath2.size()-1; ++p)
  {
    if(idpath2[p] == -1)
      continue;

    idpath_short.push_back(idpath[p]);

    sourceid = idpath2[p];
    targetid = idpath2[p+1];

    source_entry = EnvROBARM.StateID2CoordTable[sourceid];
    sangles = source_entry->angles;
    coordToWorldPose(source_entry->coord, source_wcoord);

    bestsucc = mp_path[p];
    for(size_t i = 0; i < prms_.mp_[bestsucc].m.size(); ++i)
    {
      for(size_t a = 0; a < interm_angles.size(); ++a)
        interm_wcoord[a] = source_wcoord[a] + prms_.mp_[bestsucc].m[i][a];

      ROS_INFO("[%i-%i] sourceid: %d targetid: %d mprim: %d xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f fa: %0.3f",int(p),int(i), sourceid, targetid, bestsucc, interm_wcoord[0],interm_wcoord[1],interm_wcoord[2],interm_wcoord[3],interm_wcoord[4],interm_wcoord[5],interm_wcoord[6]);

      if(convertWorldPoseToAngles(interm_wcoord, interm_angles))
      {
        for(size_t q = 0; q < interm_angles.size(); ++q)
          interm_angles[q] = angles::normalize_angle(interm_angles[q]);

        path.push_back(interm_angles); 
      }
      else
      {
        SBPL_WARN("[%i-%i] When retrieving the path, can't convert world coords to angles.",int(p),int(i));
        path.push_back(sangles);
      }
    }
  }
}

void EnvironmentCARTROBARM3D::convertStateIDPathToPoints(const std::vector<int> &idpath, std::vector<std::vector<double> > &path)
{
  int sourceid, targetid, bestcost, bestsucc;
  std::vector<int> cost, succid, action;
  std::vector<double> interm_point(4,0), source_wcoord(ndof_,0), wcoord(ndof_,0);
 
  path.clear();

  for(size_t p = 0; p < idpath.size()-1; ++p)
  {
    sourceid = idpath[p];
    targetid = idpath[p+1];

    GetSuccs(sourceid, &succid, &cost, &action);

    bestcost = INFINITECOST;
    bestsucc = -1;

    for(size_t s = 0; s < succid.size(); ++s)
    {
      if(succid[s] == targetid && cost[s] <= bestcost)
      {
        bestcost = cost[s];
        bestsucc = action[s];
      }
    }

    if(bestsucc == -1)
      SBPL_ERROR("[%i] Successor not found for transition.", int(p));

    coordToWorldPose(EnvROBARM.StateID2CoordTable[sourceid]->coord, source_wcoord);
    
    for(size_t i = 0; i < prms_.mp_[bestsucc].m.size(); ++i)
    {
      for(size_t a = 0; a < wcoord.size(); ++a)
        wcoord[a] = source_wcoord[a] + prms_.mp_[bestsucc].m[i][a];
      
      ROS_DEBUG("[%i-%i] sourceid: %d targetid: %d mprim: %d xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f fa: %0.3f",int(p),int(i), sourceid, targetid, bestsucc,wcoord[0],wcoord[1],wcoord[2],wcoord[3],wcoord[4],wcoord[5],wcoord[6]);
      
      interm_point[0] = wcoord[0];
      interm_point[1] = wcoord[1];
      interm_point[2] = wcoord[2];
      if(p % 2 == 0)
        interm_point[3] = 0;
      else
        interm_point[3] = 1;

      path.push_back(interm_point);
    }

    ROS_INFO("[%i] sourceid: %d targetid: %d mprim: %d xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f fa: %0.3f  (cell: %d %d %d)",int(p), sourceid, targetid, bestsucc,wcoord[0],wcoord[1],wcoord[2],wcoord[3],wcoord[4],wcoord[5],wcoord[6], int(EnvROBARM.StateID2CoordTable[targetid]->coord[0]),int(EnvROBARM.StateID2CoordTable[targetid]->coord[1]),int(EnvROBARM.StateID2CoordTable[targetid]->coord[2]));
  }
}

void EnvironmentCARTROBARM3D::convertShortStateIDPathToPoints(const std::vector<int> &idpath, std::vector<std::vector<double> > &path)
{
  std::vector<double> interm_point(4,0), wcoord(ndof_,0);
 
  path.clear();

  for(size_t p = 0; p < idpath.size()-1; ++p)
  {
    coordToWorldPose(EnvROBARM.StateID2CoordTable[idpath[p]]->coord, wcoord);

    interm_point[0] = wcoord[0];
    interm_point[1] = wcoord[1];
    interm_point[2] = wcoord[2];
    if(p % 2 == 0)
      interm_point[3] = 0;
    else
      interm_point[3] = 1;

    path.push_back(interm_point);
  }
  ROS_INFO("NOTE: convertStateIDPathToPoints will only work with one waypoint motion primitives. It's temporary.");
}

void EnvironmentCARTROBARM3D::getContMotionPrims(char type, std::vector<std::vector<btVector3> > &mprims)
{
  std::vector<btVector3> m;
  btVector3 origin(0.0, 0.0, 0.0);
  mprims.clear();

  for(size_t i = 0; i < prms_.mp_.size(); ++i)
  {
    if(prms_.mp_[i].type == type)
    {
      m.resize(prms_.mp_[i].nsteps+1);
      m[0] = origin;

      for(int j = 0; j < prms_.mp_[i].nsteps; ++j)
      {
        m[j+1].setX(prms_.xyz_resolution_*prms_.mp_[i].m[j][0]);
        m[j+1].setY(prms_.xyz_resolution_*prms_.mp_[i].m[j][1]);
        m[j+1].setZ(prms_.xyz_resolution_*prms_.mp_[i].m[j][2]);

        m[j+1] += m[j];
      }
      mprims.push_back(m);
    }
  }

  ROS_DEBUG("[getContMotionPrimitives] Returning %d motions of type %d.", int(mprims.size()),int(type));
}

}
