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

/** \Author: Benjamin Cohen /bcohen@seas.upenn.edu **/

#include <time.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>
#include <vector>
#include <string>
#include <list>
#include <algorithm>

#include <sbpl/headers.h>
#include <angles/angles.h>
#include <LinearMath/btVector3.h>
#include <sbpl_arm_planner/bfs_3d.h>
#include <sbpl_arm_planner/occupancy_grid.h>
#include <sbpl_arm_planner/sbpl_collision_space.h>
#include <sbpl_arm_planner/sbpl_arm_planner_params.h>

#ifndef __ENVIRONMENT_CARTROBARM3D_H_
#define __ENVIRONMENT_CARTROBARM3D_H_

namespace sbpl_cartesian_arm_planner {

/** @brief struct that describes a basic pose constraint */
typedef struct
{
  char type;
  double xyz[3];
  double rpy[3];
  double fangle;
  double xyz_tol;
  double roll_tol;
  double pitch_tol;
  double yaw_tol;
} GoalPos;

/** @brief A hash entry that contains all state information */
typedef struct
{
  int stateID;
  double dist;
  short unsigned int xyz[3];
  std::vector<short unsigned int> coord;
  std::vector<double> angles;
} EnvCARTROBARM3DHashEntry_t;

/* @brief an outdated struct that is gradually being torn apart */
typedef struct
{
  bool bInitialized;

  double xyz_resolution;
  double rpy_resolution;
  double fangle_resolution;

  std::vector<double> start_configuration;
  std::vector<double> coord_delta;
  std::vector<int> coord_vals;
  std::vector<std::vector<double> > obstacles;

  GoalPos goal;
  std::vector <std::vector <double> > ParsedGoals;
  std::vector <std::vector <double> > ParsedGoalTolerance;
} EnvCARTROBARM3DConfig_t;

/** main structure that stores environment data used in planning */
typedef struct
{
  EnvCARTROBARM3DHashEntry_t* goalHashEntry;
  EnvCARTROBARM3DHashEntry_t* startHashEntry;

  //Maps from coords to stateID
  int HashTableSize;
  std::vector<EnvCARTROBARM3DHashEntry_t*>* Coord2StateIDHashTable;

  //vector that maps from stateID to coords	
  std::vector<EnvCARTROBARM3DHashEntry_t*> StateID2CoordTable;

} EnvironmentCARTROBARM3D_t;


/** Environment to be used when planning for a Robotic Arm using the SBPL. */
class EnvironmentCARTROBARM3D: public DiscreteSpaceInformation 
{
  public:

    std::vector<int> expanded_states;
    bool save_expanded_states;

    EnvironmentCARTROBARM3D();

    ~EnvironmentCARTROBARM3D();

    bool InitializeEnv(const char* sEnvFile);
    
    bool InitializeEnv(const char* sEnvFile, std::string params_file, std::string arm_file);

    bool AreEquivalent(int StateID1, int StateID2);

    bool initEnvironment(std::string arm_description_filename, std::string mprims_filename);

    bool InitializeMDPCfg(MDPConfig *MDPCfg);
    
    bool setStartConfiguration(const std::vector<double> &angles);

    int GetFromToHeuristic(int FromStateID, int ToStateID);
    
    int GetGoalHeuristic(int stateID);

    int GetStartHeuristic(int stateID);

    void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);

    void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);
    
    void StateID2Angles(int stateID, std::vector<double> &angles);

    int	SizeofCreatedEnv();

    void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);

    void SetAllActionsandAllOutcomes(CMDPSTATE* state);

    void SetAllPreds(CMDPSTATE* state);

    void PrintEnv_Config(FILE* fOut);

    bool setGoalPosition(const std::vector <std::vector<double> > &goals, const std::vector<std::vector<double> > &tolerances);

    double getEpsilon();

    void updateOccupancyGridFromCollisionMap(const mapping_msgs::CollisionMap &collision_map);
    
    std::vector<int> debugExpandedStates();
    
    std::vector<std::vector<double> > getShortestPath();
    
    void getExpandedStates(std::vector<std::vector<double> > &ara_states);

    void visualizeOccupancyGrid();

    void setReferenceFrameTransform(KDL::Frame f, std::string &name);

    sbpl_arm_planner::SBPLCollisionSpace* getCollisionSpace() const;
 
    sbpl_arm_planner::OccupancyGrid* getOccupancyGrid() const;
    
    std::vector<double> getPlanningStats();

    void getArmChainRootLinkName(std::string &name);

    void convertStateIDPathToJointAnglesPath(const std::vector<int> &idpath, std::vector<std::vector<double> > &path);
    
    void convertStateIDPathToShortenedJointAnglesPath(const std::vector<int> &idpath, std::vector<std::vector<double> > &path, std::vector<int> &idpath_short);

    void convertStateIDPathToPoints(const std::vector<int> &idpath, std::vector<std::vector<double> > &path);

    //TODO: Will not work with multiple waypoint motion prims...It's just for fast debugging.
    void convertShortStateIDPathToPoints(const std::vector<int> &idpath, std::vector<std::vector<double> > &path);

    void getContMotionPrims(char type, std::vector<std::vector<btVector3> > &mprims);

  protected:
    virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<int>* ActionV=NULL );

  private:

    EnvCARTROBARM3DConfig_t EnvROBARMCfg;
    EnvironmentCARTROBARM3D_t EnvROBARM;
    bool using_short_mprims_;

    sbpl_arm_planner::OccupancyGrid *grid_;
    sbpl_arm_planner::BFS3D *dijkstra_;
    sbpl_arm_planner::SBPLArmModel *arm_;
    sbpl_arm_planner::SBPLCollisionSpace *cspace_;
    sbpl_arm_planner::SBPLArmPlannerParams prms_;

    std::string params_filename_;
    std::string arm_desc_filename_;
    int free_angle_index_;
    int ndof_;

    /** hash table */
    unsigned int intHash(unsigned int key);
    unsigned int getHashBin(const std::vector<short unsigned int> &coord);

    EnvCARTROBARM3DHashEntry_t* getHashEntry(const std::vector<short unsigned int> &coord, bool bIsGoal);
    EnvCARTROBARM3DHashEntry_t* getHashEntry(short unsigned int *xyz, short unsigned int *rpy, short unsigned int fangle, bool bIsGoal);
    EnvCARTROBARM3DHashEntry_t* createHashEntry(const std::vector<short unsigned int> &coord);

    /** initialization */
    bool initEnvConfig();
    bool initGeneral();
    void initDijkstra();
    bool initArmModel(FILE* aCfg, const std::string robot_description);
    void readConfiguration(FILE* fCfg);

    /* discretization */
    void coordToPose(const std::vector<short unsigned int> &coord, short unsigned int *xyz, short unsigned int *rpy, short unsigned int *fangle);
    void coordToWorldPose(const std::vector<short unsigned int> &coord, double *xyz, double *rpy, double *fangle);
    void coordToWorldPose(const std::vector<short unsigned int> &coord, std::vector<double> &wcoord);
    void anglesToCoord(const std::vector<double> &angles, std::vector<short unsigned int> &coord);
    void stateIDToPose(int stateID, short unsigned int *xyz, short unsigned int *rpy, short unsigned int *fangle);
    void stateIDToWorldPose(int stateID, double *wxyz, double *wrpy, double *wfangle);
    void worldPoseToCoord(double *wxyz, double *wrpy, double wfangle, std::vector<short unsigned int> &coord);
    void worldPoseToState(double *wxyz, double *wrpy, double wfangle, bool is_goal, EnvCARTROBARM3DHashEntry_t *state);
    void coordToAngles(const std::vector<short unsigned int> &coord, std::vector<double> &angles);

    void discToWorldXYZ(short unsigned int *xyz, double *wxyz);
    void discToWorldRPY(short unsigned int *rpy, double *wrpy);
    void discToWorldFAngle(short unsigned int fangle, double *wfangle);
    void worldToDiscXYZ(double *wxyz, short unsigned int *xyz);
    void worldToDiscRPY(double *wrpy, short unsigned int *rpy);
    void worldToDiscFAngle(double wfangle, short unsigned int *fangle);
    bool convertCoordToAngles(const std::vector<short unsigned int> *coord, std::vector<double> *angles);
    bool convertWorldPoseToAngles(const std::vector<double> &wpose, std::vector<double> &angles);
    
    /** planning */
    bool isGoalPosition(double *xyz, double *rpy, double fangle);
    bool precomputeHeuristics();

    /** costs */
    int cost(EnvCARTROBARM3DHashEntry_t* HashEntry1, EnvCARTROBARM3DHashEntry_t* HashEntry2, bool bState2IsGoal);
    int getEdgeCost(int FromStateID, int ToStateID);
    void computeCostPerCell();
    int getActionCost(const std::vector<double> &from_config, const std::vector<double> &to_config, int dist);
    int computeMotionCost(const std::vector<double> &a, const std::vector<double> &b);

    /** output */
    void printHashTableHist();
    void printJointArray(FILE* fOut, EnvCARTROBARM3DHashEntry_t* HashEntry, bool bGoal, bool bVerbose);

    /** distance */
    int getDijkstraDistToGoal(short unsigned int x, short unsigned int y, short unsigned int z) const;
    int getEndEffectorHeuristic(int FromStateID, int ToStateID);
    double getEuclideanDistance(double x1, double y1, double z1, double x2, double y2, double z2);
    void getBresenhamPath(const short unsigned int a[],const short unsigned int b[], std::vector<std::vector<int> > *path);

    /* debugging */
    void clearStats();
};

inline unsigned int EnvironmentCARTROBARM3D::intHash(unsigned int key)
{
  key += (key << 12); 
  key ^= (key >> 22);
  key += (key << 4);
  key ^= (key >> 9);
  key += (key << 10);
  key ^= (key >> 2);
  key += (key << 7);
  key ^= (key >> 12);
  return key;
}

inline unsigned int EnvironmentCARTROBARM3D::getHashBin(const std::vector<short unsigned int> &coord)
{
  int val = 0;

  for(size_t i = 0; i<coord.size(); ++i)
    val += intHash(coord[i]) << i;

  return intHash(val) & (EnvROBARM.HashTableSize-1);
}

inline double EnvironmentCARTROBARM3D::getEuclideanDistance(double x1, double y1, double z1, double x2, double y2, double z2)
{
  return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));
}

inline sbpl_arm_planner::SBPLCollisionSpace* EnvironmentCARTROBARM3D::getCollisionSpace() const
{
  return cspace_;
}

inline sbpl_arm_planner::OccupancyGrid* EnvironmentCARTROBARM3D::getOccupancyGrid() const
{
  return grid_;
}

inline void EnvironmentCARTROBARM3D::discToWorldXYZ(short unsigned int *xyz, double *wxyz)
{
  grid_->gridToWorld(int(xyz[0]),int(xyz[1]),int(xyz[2]),wxyz[0],wxyz[1],wxyz[2]);
  SBPL_DEBUG("[discToWorldXYZ] xyz: %d %d %d --> %2.3f %2.3f %2.3f",xyz[0],xyz[1],xyz[2],wxyz[0],wxyz[1],wxyz[2]);
}

inline void EnvironmentCARTROBARM3D::worldToDiscXYZ(double *wxyz, short unsigned int *xyz)
{
  int x,y,z;
  grid_->worldToGrid(wxyz[0],wxyz[1],wxyz[2],x,y,z);
  xyz[0] = x;
  xyz[1] = y;
  xyz[2] = z;
  SBPL_DEBUG("[worldToDiscXYZ] xyz: %2.3f %2.3f %2.3f --> %d %d %d",wxyz[0],wxyz[1],wxyz[2],xyz[0],xyz[1],xyz[2]);
}

inline void EnvironmentCARTROBARM3D::discToWorldRPY(short unsigned int *rpy, double *wrpy)
{
  wrpy[0] = angles::normalize_angle(double(rpy[0])*EnvROBARMCfg.coord_delta[3]);
  wrpy[1] = angles::normalize_angle(double(rpy[1])*EnvROBARMCfg.coord_delta[4]);
  wrpy[2] = angles::normalize_angle(double(rpy[2])*EnvROBARMCfg.coord_delta[5]);

  SBPL_DEBUG("[discToWorldRPY] rpy: %d %d %d --> %2.3f %2.3f %2.3f",rpy[0],rpy[1],rpy[2],wrpy[0],wrpy[1],wrpy[2]);
}

inline void EnvironmentCARTROBARM3D::worldToDiscRPY(double *wrpy, short unsigned int *rpy)
{
  rpy[0] = (int)((angles::normalize_angle_positive(wrpy[0]) + EnvROBARMCfg.coord_delta[3]*0.5)/EnvROBARMCfg.coord_delta[3]);
  rpy[1] = (int)((angles::normalize_angle_positive(wrpy[1]) + EnvROBARMCfg.coord_delta[4]*0.5)/EnvROBARMCfg.coord_delta[4]);
  rpy[2] = (int)((angles::normalize_angle_positive(wrpy[2]) + EnvROBARMCfg.coord_delta[5]*0.5)/EnvROBARMCfg.coord_delta[5]);

  SBPL_DEBUG("[worldToDiscRPY] rpy: %2.3f %2.3f %2.3f --> %d %d %d",wrpy[0],wrpy[1],wrpy[2],rpy[0],rpy[1],rpy[2]);
}

inline void EnvironmentCARTROBARM3D::discToWorldFAngle(short unsigned int fangle, double *wfangle)
{
  *wfangle = angles::normalize_angle(double(fangle)*EnvROBARMCfg.coord_delta[6]);

  SBPL_DEBUG("[discToWorldFAngle] fangle: %d --> %2.3f",fangle,*wfangle);
}

inline void EnvironmentCARTROBARM3D::worldToDiscFAngle(double wfangle, short unsigned int *fangle)
{
  *fangle = (int)((angles::normalize_angle_positive(wfangle) + EnvROBARMCfg.coord_delta[6]*0.5)/EnvROBARMCfg.coord_delta[6]);

  SBPL_DEBUG("[worldToDiscFAngle] fangle: %2.3f --> %d",wfangle,*fangle);
}

inline void EnvironmentCARTROBARM3D::coordToPose(const std::vector<short unsigned int> &coord, short unsigned int *xyz, short unsigned int *rpy, short unsigned int *fangle)
{
  xyz[0] = coord[0];
  xyz[1] = coord[1];
  xyz[2] = coord[2];
  rpy[0] = coord[3];
  rpy[1] = coord[4];
  rpy[2] = coord[5];
  *fangle = coord[6];

  SBPL_DEBUG("[coordToPose] xyz: %u %u %u  rpy: %d %d %d fangle: %u", xyz[0],xyz[1],xyz[2],rpy[0],rpy[1],rpy[2],*fangle);
}

inline void EnvironmentCARTROBARM3D::coordToWorldPose(const std::vector<short unsigned int> &coord, double *wxyz, double *wrpy, double *wfangle)
{
  short unsigned int xyz[3]={0}, rpy[3]={0}, fangle=0;
  
  coordToPose(coord,xyz,rpy,&fangle);

  discToWorldXYZ(xyz,wxyz);
  discToWorldRPY(rpy,wrpy);
  discToWorldFAngle(fangle,wfangle);
}

inline void EnvironmentCARTROBARM3D::coordToWorldPose(const std::vector<short unsigned int> &coord, std::vector<double> &wcoord)
{
  double xyz[3]={0}, rpy[3]={0}, fangle=0;
  
  coordToWorldPose(coord, xyz, rpy, &fangle);

  wcoord[0] = xyz[0];
  wcoord[1] = xyz[1];
  wcoord[2] = xyz[2];
  wcoord[3] = rpy[0];
  wcoord[4] = rpy[1];
  wcoord[5] = rpy[2]; 
  wcoord[6] = fangle;
}

//angles are counterclockwise from 0 to 360 in radians, 0 is the center of bin 0, ...
inline void EnvironmentCARTROBARM3D::coordToAngles(const std::vector<short unsigned int> &coord, std::vector<double> &angles)
{
  EnvCARTROBARM3DHashEntry_t* h;
  if((h = getHashEntry(coord,false)) == NULL)
  {
    ROS_WARN("[coordToAngles] Failed to fetch hash entry");
    return;
  }

  angles = h->angles;
}

} //namespace

#endif

