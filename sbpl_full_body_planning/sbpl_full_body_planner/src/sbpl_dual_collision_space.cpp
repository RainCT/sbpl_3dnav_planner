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
 /** \author Benjamin Cohen */

#include <sbpl_full_body_planner/sbpl_dual_collision_space.h>

#define SMALL_NUM  0.00000001     // to avoid division overflow

namespace sbpl_full_body_planner {

SBPLDualCollisionSpace::SBPLDualCollisionSpace(sbpl_arm_planner::SBPLArmModel* right_arm, sbpl_arm_planner::SBPLArmModel* left_arm, sbpl_arm_planner::OccupancyGrid* grid) : grid_(grid)
{
  arm_.resize(2);
  arm_[0] = right_arm;
  arm_[1] = left_arm;
  fOut_ = stdout;

  cspace_log_ = "cspace";

  //changed inc_ to a vector 8/28/2010
  inc_.resize(arm_[0]->num_joints_,0.0348);
  inc_[5] = 0.1392; // 8 degrees
  inc_[6] = M_PI; //rolling the wrist doesn't change the arm's shape
}

bool SBPLDualCollisionSpace::checkCollisionArms(const std::vector<double> &langles, const std::vector<double> &rangles, BodyPose &pose, bool verbose, unsigned char &dist)
{
  int code;
  return checkCollisionArms(langles,rangles,pose,verbose,dist,code);
}

bool SBPLDualCollisionSpace::checkCollisionArms(const std::vector<double> &langles, const std::vector<double> &rangles, BodyPose &pose, bool verbose, unsigned char &dist, int &debug_code)
{
  unsigned char dist_temp = 100;
  //dist = 100; NOTE: We are assuming dist comes in with some value that matters
  
  //check the right arm against the environment
  if(!checkCollision(rangles, pose, 0, verbose, dist_temp))
  {
    if(verbose)
      ROS_DEBUG_NAMED(cspace_log_, "  Right arm is in collision with the environment. (dist: %d)", int(dist_temp));
    dist = dist_temp;
    //code_ = "Right arm + Environment";
    debug_code = sbpl_arm_planner::RIGHT_ARM_IN_COLLISION;
    return false;
  }
  dist = min(dist_temp,dist);
  ROS_DEBUG("[Right] dist: %d dist_temp: %d\n",int(dist), int(dist_temp));

  //check the left arm against the environment
  if(!checkCollision(langles, pose, 1, verbose, dist_temp))
  {
    if(verbose)
      ROS_DEBUG_NAMED(cspace_log_,"  Left arm is in collision with the environment. (dist: %d)", int(dist_temp));
    dist = dist_temp;
    //code_ = "Left arm + Environment";
    debug_code = sbpl_arm_planner::LEFT_ARM_IN_COLLISION;
    return false;
  }
  dist = min(dist_temp,dist);
  ROS_DEBUG("[Left] dist: %d dist_temp: %d\n",int(dist), int(dist_temp));
 
  //check the arms against each other
  if(!checkCollisionBetweenArms(langles,rangles, pose, verbose, dist_temp))
  {
    if(verbose)
      ROS_DEBUG_NAMED(cspace_log_,"  Arms are in collision with each other. (dist %d)", int(dist_temp));
    dist = dist_temp;
    //code_ = "Left arm + Right arm";
    debug_code = sbpl_arm_planner::COLLISION_BETWEEN_ARMS;
    return false;
  }
  dist = min(dist_temp,dist);
  ROS_DEBUG("[Arms] dist: %d dist_temp: %d\n",int(dist), int(dist_temp));

  return true;
}

bool SBPLDualCollisionSpace::checkCollision(const std::vector<double> &angles, BodyPose &pose, char i_arm, bool verbose, unsigned char &dist)
{
  unsigned char dist_temp=100;
  std::vector<std::vector<int> > jnts;
  //dist = 100; NOTE: We are assuming dist comes in with some value that matters

  //get position of joints in the occupancy grid
  if(!getJointPosesInGrid(angles, pose, i_arm, jnts))
  {
    if(verbose)
      ROS_DEBUG_NAMED(cspace_log_,"  Unable to get poses of the joints to check for collision. [arm: %d]",int(i_arm));
    return false;
  }

  //check bounds
  for(size_t i = 0; i < jnts.size(); ++i)
  {
    if(!grid_->isInBounds(jnts[i][0],jnts[i][1],jnts[i][2]))
    {
      if(verbose)
        ROS_DEBUG_NAMED(cspace_log_,"  End of link %d is out of bounds (%d %d %d). [arm: %d]", int(i), jnts[i][0],jnts[i][1],jnts[i][2],int(i_arm));
      return false;
    }
  }

  //test each line segment for collision
  for(int i = 0; i < int(jnts.size()-1); i++)
  {
    dist_temp = isValidLineSegment(jnts[i], jnts[i+1], arm_[i_arm]->getLinkRadiusCells(i));
    
    //if the line's distance to the nearest obstacle is less than the radius
    if(dist_temp <= arm_[i_arm]->getLinkRadiusCells(i))
    { 
      if(verbose)
        ROS_DEBUG_NAMED(cspace_log_,"  Link %d: {%d %d %d} -> {%d %d %d} with radius %0.2f is in collision. [arm: %d]",i,jnts[i][0],jnts[i][1],jnts[i][2],jnts[i+1][0],jnts[i+1][1],jnts[i+1][2], arm_[i_arm]->getLinkRadius(i), int(i_arm));
      dist = dist_temp;
      return false;
    }

    if(dist_temp < dist)
      dist = dist_temp;
  }

  if(dist_temp < dist)
      dist = dist_temp;

  return true;
}

bool SBPLDualCollisionSpace::checkCollision(std::vector<double> &langles, std::vector<double> &rangles, BodyPose &pose, bool verbose, unsigned char &dist, int &debug_code)
{
  if(!checkCollisionArms(langles, rangles, pose, verbose, dist, debug_code))
  {
    ROS_DEBUG("[cspace] Arms are in collision.");
    return false;
  }
  if(!isBodyValid(pose.x, pose.y, pose.theta, pose.z, dist))
  {
    ROS_DEBUG("[cspace] Body is in collision.");
    return false;
  }
  if(!checkCollisionArmsToBody(langles, rangles, pose, dist))
  {
    ROS_DEBUG("[cspace] Arms are in collision with body.");
    return false;
  }
  return true;
}

bool SBPLDualCollisionSpace::checkLinkForCollision(const std::vector<double> &angles, BodyPose &pose, char i_arm, int link_num, bool verbose, unsigned char &dist)
{
  std::vector<std::vector<int> > jnts;

  if(link_num >= arm_[i_arm]->num_links_)
  {
    SBPL_WARN("[checkLinkInCollision] %d is not a valid link index. There are %d links.", link_num, arm_[i_arm]->num_links_);
    return false;
  }
  
  //get position of joints in the occupancy grid
  if(!getJointPosesInGrid(angles, pose, i_arm, jnts))
    return false;

  //check bounds
  if(!grid_->isInBounds(jnts[link_num][0],jnts[link_num][1],jnts[link_num][2]))
  {
    if(verbose)
      ROS_DEBUG_NAMED(cspace_log_,"End of link %d is out of bounds. (%d %d %d)", link_num, jnts[link_num][0],jnts[link_num][1],jnts[link_num][2]);
    return false;
  }

  //is link in collision?
  dist = isValidLineSegment(jnts[link_num], jnts[link_num+1], arm_[i_arm]->getLinkRadiusCells(link_num));

  //if the line's distance to the nearest obstacle is less than the radius
  if(dist <= arm_[i_arm]->getLinkRadiusCells(link_num))
  {
    if(verbose)
      ROS_DEBUG_NAMED(cspace_log_,"Link %d: {%d %d %d} -> {%d %d %d} with radius %0.2f is in collision.",link_num,jnts[link_num][0],jnts[link_num][1],jnts[link_num][2],jnts[link_num+1][0],jnts[link_num+1][1],jnts[link_num+1][2],arm_[i_arm]->getLinkRadius(link_num));
    return false;
  }

  return true;
}

bool SBPLDualCollisionSpace::checkPathForCollision(const std::vector<double> &start, const std::vector<double> &end, BodyPose &pose, char i_arm, bool verbose, unsigned char &dist)
{
  int inc_cc = 10;
  unsigned char dist_temp = 0;
  std::vector<double> start_norm(start);
  std::vector<double> end_norm(end);
  std::vector<std::vector<double> > path;
  //dist = 100; NOTE: We are assuming dist comes in with some value that matters
  
  for(size_t i=0; i < start.size(); i++)
  {
    start_norm[i] = angles::normalize_angle(start[i]);
    end_norm[i] = angles::normalize_angle(end[i]);
  }
 
  //TODO: should read from text file
  if(i_arm == 0)
  {
    if(start[2] > arm_[i_arm]->getMaxJointLimit(2))
      start_norm[2] = start[2] + (2*-M_PI);

    if(end[2] > arm_[i_arm]->getMaxJointLimit(2))
      end_norm[2] = end[2] + (2*-M_PI);
  }
  else
  {
    if(start[2] < arm_[i_arm]->getMinJointLimit(2))
      start_norm[2] = start[2] + (2*M_PI);

    if(end[2] < arm_[i_arm]->getMinJointLimit(2))
      end_norm[2] = end[2] + (2*M_PI);
  }

  getInterpolatedPath(start_norm, end_norm, inc_, path);

  // optimization: try to find collisions that might come later in the path earlier
  if(int(path.size()) > inc_cc)
  {
    for(int i = 0; i < inc_cc; i++)
    {
      for(size_t j = i; j < path.size(); j=j+inc_cc)
      {
        if(!checkCollision(path[j], pose, i_arm, verbose, dist_temp))
        {
          dist = dist_temp;
          return false; 
        }

        if(dist_temp < dist)
          dist = dist_temp;
      }
    }
  }
  else
  {
    for(size_t i = 0; i < path.size(); i++)
    {
      if(!checkCollision(path[i], pose, i_arm, verbose, dist_temp))
      {
        dist = dist_temp;
        return false;
      }

      if(dist_temp < dist)
        dist = dist_temp;
    }
  }

  return true;
}

bool SBPLDualCollisionSpace::checkPathForCollision(const std::vector<double> &start0, const std::vector<double> &end0, const std::vector<double> &start1, const std::vector<double> &end1, BodyPose &pose, bool verbose, unsigned char &dist)
{
  int debug_code;
  return checkPathForCollision(start0,end0,start1,end1,pose,verbose,dist,debug_code);
}

bool SBPLDualCollisionSpace::checkPathForCollision(const std::vector<double> &start0, const std::vector<double> &end0, const std::vector<double> &start1, const std::vector<double> &end1, BodyPose &pose, bool verbose, unsigned char &dist, int &debug_code)
{
  int inc_cc = 10;
  unsigned char dist_temp = 0;
  std::vector<double> start0_norm(start0), start1_norm(start1), end0_norm(end0), end1_norm(end1);
  std::vector<std::vector<double> > path0, path1;
  //dist = 100;  NOTE: We are assuming dist comes in with some value that matters

  for(size_t i=0; i < start0.size(); i++)
  {
    start0_norm[i] = angles::normalize_angle(start0[i]);
    end0_norm[i] = angles::normalize_angle(end0[i]);
    start1_norm[i] = angles::normalize_angle(start1[i]);
    end1_norm[i] = angles::normalize_angle(end1[i]);
  }

  //right arm - upper_arm roll
  if(start0[2] > arm_[0]->getMaxJointLimit(2))
    start0_norm[2] = start0[2] + (2*-M_PI);
  if(end0[2] > arm_[0]->getMaxJointLimit(2))
    end0_norm[2] = end0[2] + (2*-M_PI);

  //left arm - upper_arm roll
  if(start1[2] < arm_[1]->getMinJointLimit(2))
    start1_norm[2] = start1[2] + (2*M_PI);
  if(end1[2] < arm_[1]->getMinJointLimit(2))
    end1_norm[2] = end1[2] + (2*M_PI);

  getInterpolatedPath(start0_norm, end0_norm, inc_, path0);
  getFixedLengthInterpolatedPath(start1_norm, end1_norm, path0.size(), path1);

  // optimization: try to find collisions that might come later in the path earlier
  if(int(path0.size()) > inc_cc)
  {
    for(int i = 0; i < inc_cc; i++)
    {
      for(size_t j = i; j < path0.size(); j=j+inc_cc)
      {
        if(!checkCollision(path0[j], path1[j], pose, verbose, dist_temp, debug_code))
        {
          dist = dist_temp;
          return false; 
        }

        if(dist_temp < dist)
          dist = dist_temp;
      }
    }
  }
  else
  {
    for(size_t i = 0; i < path0.size(); i++)
    {
      if(!checkCollision(path0[i], path1[i], pose, verbose, dist_temp, debug_code))
      {
        dist = dist_temp;
        return false;
      }

      if(dist_temp < dist)
        dist = dist_temp;
    }
  }
  return true;
}

bool SBPLDualCollisionSpace::checkLinkPathForCollision(const std::vector<double> &start, const std::vector<double> &end, BodyPose &pose, char i_arm, int link_num, bool verbose, unsigned char &dist)
{
  int inc_cc = 10;
  unsigned char dist_temp = 0;
  std::vector<double> start_norm(start);
  std::vector<double> end_norm(end);
  std::vector<std::vector<double> > path;
  //dist = 100;  NOTE: We are assuming dist comes in with some value that matters

  for(size_t i=0; i < start.size(); i++)
  {
    start_norm[i] = angles::normalize_angle(start[i]);
    end_norm[i] = angles::normalize_angle(end[i]);
  }
 
  //problem with upper_arm_roll
  //TODO: Do this more gracefully?
  if(i_arm == 0)
  {
    if(start[2] > arm_[i_arm]->getMaxJointLimit(2))
      start_norm[2] = start[2] + (2*-M_PI);

    if(end[2] > arm_[i_arm]->getMaxJointLimit(2))
      end_norm[2] = end[2] + (2*-M_PI);
  }
  else
  {
    if(start[2] < arm_[i_arm]->getMinJointLimit(2))
      start_norm[2] = start[2] + (2*M_PI);

    if(end[2] < arm_[i_arm]->getMinJointLimit(2))
      end_norm[2] = end[2] + (2*M_PI);
  }

  getInterpolatedPath(start_norm, end_norm, inc_, path);

  //try to find collisions that might come later in the path earlier
  if(int(path.size()) > inc_cc)
  {
    for(int i = 0; i < inc_cc; i++)
    {
      for(size_t j = i; j < path.size(); j=j+inc_cc)
      {
        if(!checkLinkForCollision(path[j], pose, i_arm, link_num, verbose, dist_temp))
        {
          dist = dist_temp;
          return false; 
        }

        if(dist_temp < dist)
          dist = dist_temp;
      }
    }
  }
  else
  {
    for(size_t i = 0; i < path.size(); i++)
    {
      if(!checkLinkForCollision(path[i], pose, i_arm, link_num, verbose, dist_temp))
      {
        dist = dist_temp;
        return false;
      }

      if(dist_temp < dist)
        dist = dist_temp;
    }
  }

  return true;
}

bool SBPLDualCollisionSpace::getJointPosesInGrid(const std::vector<double> angles, BodyPose &pose, char i_arm, std::vector<std::vector<int> > &jnts)
{
  std::vector<std::vector<double> > jnts_m;

  if(!arm_[i_arm]->getJointPositions(angles, pose, jnts_m, frame_))
    return false;

  // replace r_finger_tip_link 6/15/11
  KDL::Vector tip_f_ref, tip_f_wrist(0.16,0,0);
  tip_f_ref = frame_.M * tip_f_wrist + frame_.p;
  //jnts_m[3][0] = tip_f_ref.x();
  //jnts_m[3][1] = tip_f_ref.y();
  //jnts_m[3][2] = tip_f_ref.z();

  jnts_m.back()[0] = tip_f_ref.x();
  jnts_m.back()[1] = tip_f_ref.y();
  jnts_m.back()[2] = tip_f_ref.z();

  jnts.resize(jnts_m.size());
  for(size_t i = 0; i < jnts.size(); ++i)
  {
    jnts[i].resize(3);
    grid_->worldToGrid(jnts_m[i][0],jnts_m[i][1],jnts_m[i][2],jnts[i][0],jnts[i][1],jnts[i][2]);
  }

  return true;
}

unsigned char SBPLDualCollisionSpace::isValidLineSegment(const std::vector<int> a, const std::vector<int> b, const short unsigned int radius)
{
  bresenham3d_param_t params;
  int nXYZ[3], retvalue = 1;
  unsigned char cell_val, min_dist = 255;
  CELL3V tempcell;
  vector<CELL3V>* pTestedCells=NULL;

  //iterate through the points on the segment
  get_bresenham3d_parameters(a[0], a[1], a[2], b[0], b[1], b[2], &params);
  do {
    get_current_point3d(&params, &(nXYZ[0]), &(nXYZ[1]), &(nXYZ[2]));

    if(!grid_->isInBounds(nXYZ[0],nXYZ[1],nXYZ[2]))
      return 0;

    cell_val = grid_->getCell(nXYZ[0],nXYZ[1],nXYZ[2]);
    if(cell_val <= radius)
    {
      if(pTestedCells == NULL)
        return cell_val;   //return 0
      else
        retvalue = 0;
    }

    if(cell_val < min_dist)
      min_dist = cell_val;

    //insert the tested point
    if(pTestedCells)
    {
      if(cell_val <= radius)
        tempcell.bIsObstacle = true;
      else
        tempcell.bIsObstacle = false;
      tempcell.x = nXYZ[0];
      tempcell.y = nXYZ[1];
      tempcell.z = nXYZ[2];
      pTestedCells->push_back(tempcell);
    }
  } while (get_next_point3d(&params));

  if(retvalue)
    return min_dist;
  else
    return 0;
}

double SBPLDualCollisionSpace::distanceBetween3DLineSegments(std::vector<int> l1a, std::vector<int> l1b,std::vector<int> l2a, std::vector<int> l2b)
{
  // Copyright 2001, softSurfer (www.softsurfer.com)
  // This code may be freely used and modified for any purpose
  // providing that this copyright notice is included with it.
  // SoftSurfer makes no warranty for this code, and cannot be held
  // liable for any real or imagined damage resulting from its use.
  // Users of this code must verify correctness for their application.

  double u[3];
  double v[3];
  double w[3];
  double dP[3];

  u[0] = l1b[0] - l1a[0];
  u[1] = l1b[1] - l1a[1];
  u[2] = l1b[2] - l1a[2];

  v[0] = l2b[0] - l2a[0];
  v[1] = l2b[1] - l2a[1];
  v[2] = l2b[2] - l2a[2];

  w[0] = l1a[0] - l2a[0];
  w[1] = l1a[1] - l2a[1];
  w[2] = l1a[2] - l2a[2];

  double a = u[0] * u[0] + u[1] * u[1] + u[2] * u[2]; // always >= 0
  double b = u[0] * v[0] + u[1] * v[1] + u[2] * v[2]; // dot(u,v);
  double c = v[0] * v[0] + v[1] * v[1] + v[2] * v[2]; // dot(v,v);        // always >= 0
  double d = u[0] * w[0] + u[1] * w[1] + u[2] * w[2]; // dot(u,w);
  double e = v[0] * w[0] + v[1] * w[1] + v[2] * w[2]; // dot(v,w);
  double D = a*c - b*b;       // always >= 0
  double sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
  double tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0

  // compute the line parameters of the two closest points
  if (D < SMALL_NUM) { // the lines are almost parallel
    sN = 0.0;        // force using point P0 on segment S1
    sD = 1.0;        // to prevent possible division by 0.0 later
    tN = e;
    tD = c;
  }
  else {                // get the closest points on the infinite lines
    sN = (b*e - c*d);
    tN = (a*e - b*d);
    if (sN < 0.0) {       // sc < 0 => the s=0 edge is visible
      sN = 0.0;
      tN = e;
      tD = c;
    }
    else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
      sN = sD;
      tN = e + b;
      tD = c;
    }
  }

  if (tN < 0.0) {           // tc < 0 => the t=0 edge is visible
    tN = 0.0;
    // recompute sc for this edge
    if (-d < 0.0)
      sN = 0.0;
    else if (-d > a)
      sN = sD;
    else {
      sN = -d;
      sD = a;
    }
  }
  else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
    tN = tD;
    // recompute sc for this edge
    if ((-d + b) < 0.0)
      sN = 0;
    else if ((-d + b) > a)
      sN = sD;
    else {
      sN = (-d + b);
      sD = a;
    }
  }

  // finally do the division to get sc and tc
  sc = (fabs(sN) < SMALL_NUM ? 0.0 : sN / sD);
  tc = (fabs(tN) < SMALL_NUM ? 0.0 : tN / tD);

  // get the difference of the two closest points
  // dP = w + (sc * u) - (tc * v);  // = S1(sc) - S2(tc)

  dP[0] = w[0] + (sc * u[0]) - (tc * v[0]);
  dP[1] = w[1] + (sc * u[1]) - (tc * v[1]);
  dP[2] = w[2] + (sc * u[2]) - (tc * v[2]);

  return  sqrt(dP[0]*dP[0] + dP[1]*dP[1] + dP[2]*dP[2]);   // return the closest distance
}

void SBPLDualCollisionSpace::addArmCuboidsToGrid(char i_arm)
{
  std::vector<std::vector<double> > cuboids = arm_[i_arm]->getCollisionCuboids();

  ROS_DEBUG("[SBPLDualCollisionSpace] received %d cuboids\n",int(cuboids.size()));

  for(unsigned int i = 0; i < cuboids.size(); i++)
  {
    if(cuboids[i].size() == 6)
      grid_->addCollisionCuboid(cuboids[i][0],cuboids[i][1],cuboids[i][2],cuboids[i][3],cuboids[i][4],cuboids[i][5]);
    else
      ROS_DEBUG("[addArmCuboidsToGrid] Self-collision cuboid #%d has an incomplete description.\n", i);
  }
}

bool SBPLDualCollisionSpace::getCollisionCylinders(const std::vector<double> &angles, BodyPose &pose, char i_arm, std::vector<std::vector<double> > &cylinders)
{
  int num_arm_spheres = 0;
  std::vector<double> xyzr(4,0);
  std::vector<std::vector<int> > points, jnts;

  //get position of joints in the occupancy grid
  if(!getJointPosesInGrid(angles, pose, i_arm, jnts))
    return false;

  for(int i = 0; i < int(jnts.size()-1); ++i)
  {
    points.clear();
    getLineSegment(jnts[i], jnts[i+1], points);

    //write points to cylinder {x,y,z,radius} all in centimeters
    for(int j = 0; j < int(points.size()); ++j)
    {
      grid_->gridToWorld(points[j][0],points[j][1],points[j][2],xyzr[0],xyzr[1],xyzr[2]); 
      xyzr[3]=double(arm_[i_arm]->getLinkRadiusCells(i))*arm_[i_arm]->resolution_;
      cylinders.push_back(xyzr);
    }
  }

  num_arm_spheres = cylinders.size();
  return true;
}

void SBPLDualCollisionSpace::getLineSegment(const std::vector<int> a,const std::vector<int> b,std::vector<std::vector<int> > &points){
  bresenham3d_param_t params;
  std::vector<int> nXYZ(3,0);

  //iterate through the points on the segment
  get_bresenham3d_parameters(a[0], a[1], a[2], b[0], b[1], b[2], &params);
  do {
    get_current_point3d(&params, &(nXYZ[0]), &(nXYZ[1]), &(nXYZ[2]));

    points.push_back(nXYZ);

  } while (get_next_point3d(&params));
}

bool SBPLDualCollisionSpace::checkCollisionBetweenArms(const std::vector<double> &langles, const std::vector<double> &rangles, BodyPose &pose, bool verbose, unsigned char &dist)
{
  double d = 100, d_min = 100;
  std::vector<std::vector<int> > ljnts, rjnts;

  //get position of joints in the occupancy grid
  if(!getJointPosesInGrid(rangles, pose, 0, rjnts))
    return false;
  if(!getJointPosesInGrid(langles, pose, 1, ljnts))
    return false;

  //measure distance between links of each arms
  ROS_DEBUG_NAMED(cspace_log_,"right arm joints: %d  left arm joints: %d",int(rjnts.size()), int(ljnts.size()));

  for(size_t i = 0; i < rjnts.size()-1; ++i)
  {
    for(size_t j = 0; j < ljnts.size()-1; ++j)
    {
      d = distanceBetween3DLineSegments(rjnts[i], rjnts[i+1], ljnts[j], ljnts[j+1]);
      if(verbose)
      {
        ROS_DEBUG_NAMED(cspace_log_,"Right %d: %d %d %d -> %d %d %d  dist: %0.3f", int(i), rjnts[i][0], rjnts[i][1], rjnts[i][2], rjnts[i+1][0],rjnts[i+1][1],rjnts[i+1][2],d);
        ROS_DEBUG_NAMED(cspace_log_,"Left  %d: %d %d %d -> %d %d %d  dist: %0.3f", int(j), ljnts[j][0], ljnts[j][1], ljnts[j][2], ljnts[j+1][0],ljnts[j+1][1],ljnts[j+1][2],d);
      }
      if(d <= max(arm_[0]->getLinkRadiusCells(i), arm_[1]->getLinkRadiusCells(j)))
      {
        if(verbose)
          ROS_DEBUG_NAMED(cspace_log_,"  Right arm link %d is in collision with left arm link %d. (dist: %0.3fm)", int(i), int(j), d*arm_[0]->resolution_);
        dist = d;
        return false;
      }
      /*
      else
        if(verbose)
          ROS_INFO("Distance between right link %d and left link %d is %0.3fm", int(i), int(j), d*arm_[0]->resolution_);
      */

      if(d < d_min)
        d_min = d; //min(d_min, d);
    }
  }
  dist = d_min;
  return true;
}

void SBPLDualCollisionSpace::getInterpolatedPath(const std::vector<double> &start, const std::vector<double> &end, double inc, std::vector<std::vector<double> > &path)
{
  bool changed = true; 
  std::vector<double> next(start);
  
  //check if both input configurations have same size
  if(start.size() != end.size())
  {
    SBPL_WARN("[getInterpolatedPath] The start and end configurations have different sizes.\n");
    return;
  }

  while(changed)
  {
    changed = false;

    for (int i = 0; i < int(start.size()); i++) 
    {
      if (fabs(next[i] - end[i]) > inc) 
      {
        changed = true;
        
        if(end[i] > next[i]) 
          next[i] += inc;
        else
          next[i] += -inc;
      }
    }

    if (changed)
      path.push_back(next);
  }
}

void SBPLDualCollisionSpace::getInterpolatedPath(const std::vector<double> &start, const std::vector<double> &end, std::vector<double> &inc, std::vector<std::vector<double> > &path)
{
  bool changed = true; 
  std::vector<double> next(start);
  
  //check if both input configurations have same size
  if(start.size() != end.size())
  {
    SBPL_WARN("[getInterpolatedPath] The start and end configurations have different sizes.\n");
    return;
  }

  while(changed)
  {
    changed = false;

    for (int i = 0; i < int(start.size()); i++) 
    {
      if (fabs(next[i] - end[i]) > inc[i]) 
      {
        changed = true;
        
        if(end[i] > next[i]) 
          next[i] += inc[i];
        else
          next[i] += -inc[i];
      }
    }

    if (changed)
      path.push_back(next);
  }
}

void SBPLDualCollisionSpace::getFixedLengthInterpolatedPath(const std::vector<double> &start, const std::vector<double> &end, int path_length, std::vector<std::vector<double> > &path)
{
  std::vector<double> next(start), inc(start.size(),0);
  path.clear();

  //check if both input configurations have same size
  if(start.size() != end.size())
  {
    SBPL_WARN("[getInterpolatedPath] The start and end configurations have different sizes.\n");
    return;
  }

  //compute the increments from start to end configurations
  for(size_t i = 0; i < start.size(); ++i)
  {
    if((start[i]-end[i]) == 0.0)
      inc[i] = 0.0;
    else
      inc[i] = (start[i]-end[i])/path_length;
  }

  for(int i = 0; i < path_length-1; ++i)
  {
    for(int j = 0; j < int(start.size()); ++j)
      next[j] += inc[j];

    path.push_back(next);
  }

  path.push_back(end);
}

void SBPLDualCollisionSpace::processCollisionObjectMsg(const arm_navigation_msgs::CollisionObject &object)
{
  if(object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ADD)
  {
    object_map_[object.id] = object;
    addCollisionObject(object);
  }
  else if(object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE)
  {
    if(object.id.compare("all") == 0)
      removeAllCollisionObjects();
    else
      removeCollisionObject(object);
  }
  else if(object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT)
  {
    object_map_[object.id] = object;
    addCollisionObject(object);
  }
  else if(object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT)
  {
    //TODO: Attach to gripper
    removeCollisionObject(object);
  }
  else
    ROS_WARN("*** Operation isn't supported. ***\n\n");
}

void SBPLDualCollisionSpace::addCollisionObject(const arm_navigation_msgs::CollisionObject &object)
{
  geometry_msgs::Pose pose;

  for(size_t i = 0; i < object.shapes.size(); ++i)
  {
    if(object.shapes[i].type == arm_navigation_msgs::Shape::BOX)
    {
      transformPose(object.header.frame_id, grid_->getReferenceFrame(), object.poses[i], pose);
      object_voxel_map_[object.id].clear();
      grid_->getVoxelsInBox(pose, object.shapes[i].dimensions, object_voxel_map_[object.id]);

      ROS_DEBUG_NAMED(cspace_log_,"[%s] TransformPose from %s to %s.", object.id.c_str(), object.header.frame_id.c_str(), grid_->getReferenceFrame().c_str());
      ROS_DEBUG_NAMED(cspace_log_,"[%s] %s: xyz: %0.3f %0.3f %0.3f   quat: %0.3f %0.3f %0.3f %0.3f", object.id.c_str(), object.header.frame_id.c_str(), object.poses[i].position.x,  object.poses[i].position.y, object.poses[i].position.z, object.poses[i].orientation.x, object.poses[i].orientation.y, object.poses[i].orientation.z,object.poses[i].orientation.w);
      ROS_DEBUG_NAMED(cspace_log_,"[%s] %s: xyz: %0.3f %0.3f %0.3f   quat: %0.3f %0.3f %0.3f %0.3f", object.id.c_str(), grid_->getReferenceFrame().c_str(), pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
      ROS_DEBUG_NAMED(cspace_log_,"[%s] occupies %d voxels.",object.id.c_str(), int(object_voxel_map_[object.id].size()));
    }
    else
      ROS_WARN("Collision objects of type %d are not yet supported.", object.shapes[i].type);
  }

  // add this object to list of objects that get added to grid
  bool new_object = true;
  for(size_t i = 0; i < known_objects_.size(); ++i)
  {
    if(known_objects_[i].compare(object.id) == 0)
      new_object = false;
  }
  if(new_object)
    known_objects_.push_back(object.id);

  grid_->addPointsToField(object_voxel_map_[object.id]);
  ROS_INFO("[cspace] Just added %s to the distance field.", object.id.c_str());
}

void SBPLDualCollisionSpace::getCollisionObjectVoxelPoses(std::vector<geometry_msgs::Pose> &points)
{
  geometry_msgs::Pose pose;

  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  for(size_t i = 0; i < known_objects_.size(); ++i)
  {
    for(size_t j = 0; j < object_voxel_map_[known_objects_[i]].size(); ++j)
    {
      pose.position.x = object_voxel_map_[known_objects_[i]][j].x();
      pose.position.y = object_voxel_map_[known_objects_[i]][j].y();
      pose.position.z = object_voxel_map_[known_objects_[i]][j].z();
      points.push_back(pose);
    }
  }
}

void SBPLDualCollisionSpace::removeCollisionObject(const arm_navigation_msgs::CollisionObject &object)
{
  for(size_t i = 0; i < known_objects_.size(); ++i)
  {
    if(known_objects_[i].compare(object.id) == 0)
    {
      known_objects_.erase(known_objects_.begin() + i);
      object_voxel_map_[object.id].clear();
      ROS_DEBUG("[removeCollisionObject] Removing %s from list of known objects.", object.id.c_str());
    }
  }
}

void SBPLDualCollisionSpace::removeAllCollisionObjects()
{
  known_objects_.clear();
}

void SBPLDualCollisionSpace::putCollisionObjectsInGrid()
{
  ROS_DEBUG("[putCollisionObjectsInGrid] Should we reset first?");

  for(size_t i = 0; i < known_objects_.size(); ++i)
  {
    grid_->addPointsToField(object_voxel_map_[known_objects_[i]]);
    ROS_DEBUG("[putCollisionObjectsInGrid] Added %s to grid with %d voxels.",known_objects_[i].c_str(), int(object_voxel_map_[known_objects_[i]].size()));
  }
}

void SBPLDualCollisionSpace::transformPose(const std::string &current_frame, const std::string &desired_frame, const geometry_msgs::Pose &pose_in, geometry_msgs::Pose &pose_out)
{
  geometry_msgs::PoseStamped stpose_in, stpose_out;
  stpose_in.header.frame_id = current_frame;
  stpose_in.header.stamp = ros::Time();
  stpose_in.pose = pose_in;
  tf_.transformPose(desired_frame, stpose_in, stpose_out);
  pose_out = stpose_out.pose;
}

void SBPLDualCollisionSpace::removeAllAttachedObjects()
{
  attached_object_.clear();
  object_radius_.clear();
  object_radius_w_.clear();
  is_object_attached_ = false;
}

void SBPLDualCollisionSpace::addAttachedObject(const arm_navigation_msgs::CollisionObject &object)
{
  KDL::Vector point;

  //TODO: This is a hack. Maybe we should allow multiple attached objects?
  removeAllAttachedObjects();

  for(size_t i = 0; i < object.shapes.size(); ++i)
  {
    if(object.shapes[i].vertices.size() != object.shapes[i].dimensions.size())
      ROS_WARN("[cspace] There are %d points to %s but only %d dimensions.",int(object.shapes[i].vertices.size()),object.id.c_str(), int(object.shapes[i].dimensions.size()));

    for(size_t j = 0; j < object.shapes[i].vertices.size(); ++j)
    {
      point.x(object.shapes[i].vertices[j].x);
      point.y(object.shapes[i].vertices[j].y);
      point.z(object.shapes[i].vertices[j].z);
      attached_object_.push_back(point);

      object_radius_w_.push_back(object.shapes[i].dimensions[j]);
      object_radius_.push_back(int(object_radius_w_.back()/grid_->getResolution()+0.5));
    }
  }

  is_object_attached_ = true;
 
  /* 
  for(size_t i = 0; i < attached_object_.size(); ++i)
    ROS_INFO("[cspace] [%d] xyz: %0.3f %0.3f %0.3f radius: %0.3f (%d)",int(i),attached_object_[i].x(),attached_object_[i].y(),attached_object_[i].z(),object_radius_w_[i],object_radius_[i]);
  */
}

void SBPLDualCollisionSpace::getAttachedObjectVoxels(const std::vector<double> &pose, std::vector<std::vector<int> > &objectv)
{
  KDL::Frame f;
  KDL::Vector v;

  f.p.x(pose[0]);
  f.p.y(pose[1]);
  f.p.z(pose[2]);
  f.M.RPY(pose[3],pose[4],pose[5]);

  objectv.resize(attached_object_.size(),std::vector<int>(3,0));
  for(size_t i = 0; i < attached_object_.size(); ++i)
  {
    v = f*attached_object_[i];
    grid_->worldToGrid(v.data[0],v.data[1],v.data[2],objectv[i][0],objectv[i][1],objectv[i][2]);
  }
}

void SBPLDualCollisionSpace::getAttachedObjectInWorldFrame(const std::vector<double> &pose, std::vector<std::vector<double> > &objectv)
{
  KDL::Frame f;
  KDL::Vector v;

  f.p.x(pose[0]);
  f.p.y(pose[1]);
  f.p.z(pose[2]);
  f.M.RPY(pose[3],pose[4],pose[5]);

  objectv.resize(attached_object_.size(),std::vector<double>(4,0));
  for(size_t i = 0; i < attached_object_.size(); ++i)
  {
    v = f*attached_object_[i];
    objectv[i][0] = v.x();
    objectv[i][1] = v.y();
    objectv[i][2] = v.z();
    objectv[i][3] = object_radius_w_[i];
  }
}

bool SBPLDualCollisionSpace::isValidAttachedObject(const std::vector<double> &pose, unsigned char &dist, int &debug_code)
{
  if(!is_object_attached_)
    return true;

  unsigned char dist_temp = 100;

  getAttachedObjectVoxels(pose, attached_voxels_);

  for(size_t i = 0; i < attached_voxels_.size(); ++i)
  {
    if((dist_temp = grid_->getCell(attached_voxels_[i][0],attached_voxels_[i][1],attached_voxels_[i][2])) <= object_radius_[i])
    {
      dist = dist_temp;
      debug_code = sbpl_arm_planner::ATTACHED_OBJECT_IN_COLLISION;
      return false;
    }
    
    if(dist > dist_temp)
      dist = dist_temp;
  }
  return true;
}


bool SBPLDualCollisionSpace::getCollisionLinks()
{
  XmlRpc::XmlRpcValue xml_links;
  CollisionLink cl;
  cl_.clear();
  std::string links_name = "collision_links";

  ros::NodeHandle ph("~");
  if(!ph.hasParam(links_name)) 
  {
    ROS_WARN_STREAM("No groups for planning specified in " << links_name);
    return false;
  }
  ph.getParam(links_name, xml_links);

  if(xml_links.getType() != XmlRpc::XmlRpcValue::TypeArray) 
  {
    ROS_WARN("'Collision Links' is not an array.");
    return false;
  }

  if(xml_links.size() == 0)
  {
    ROS_WARN("'Collision Links' is empty.");
    return false;
  }

  for(int i = 0; i < xml_links.size(); i++) 
  {
    if(xml_links[i].hasMember("name"))
      cl.name = std::string(xml_links[i]["name"]);
    else
      cl.name = "link_" + boost::lexical_cast<std::string>(i);

    if(xml_links[i].hasMember("x1") && xml_links[i].hasMember("y1") && xml_links[i].hasMember("z1") &&
       xml_links[i].hasMember("x2") && xml_links[i].hasMember("y2") && xml_links[i].hasMember("z2"))
    {
      cl.x1 = xml_links[i]["x1"];
      cl.y1 = xml_links[i]["y1"];
      cl.z1 = xml_links[i]["z1"];
      cl.x2 = xml_links[i]["x2"];
      cl.y2 = xml_links[i]["y2"];
      cl.z2 = xml_links[i]["z2"]; 
      cl.p1 = KDL::Vector(double(xml_links[i]["x1"]),double(xml_links[i]["y1"]),double(xml_links[i]["z1"]));
      cl.p2 = KDL::Vector(double(xml_links[i]["x2"]),double(xml_links[i]["y2"]),double(xml_links[i]["z2"]));
    }
    else
    {
      ROS_ERROR("Collision link '%s' is missing one of the coordinates.", cl.name.c_str());
      return false;
    }

    if(xml_links[i].hasMember("frame"))
      cl.frame = std::string(xml_links[i]["frame"]);
    else
    {
      ROS_WARN("Collision link '%s' is missing the reference frame.", cl.name.c_str());
      return false;
    }

    if(xml_links[i].hasMember("radius"))
      cl.radius = xml_links[i]["radius"];
    else
    {
      ROS_WARN("Description of collision link '%s' is missing the radius.", cl.name.c_str());
      return false;
    }
 
    if(xml_links[i].hasMember("priority"))
      cl.priority = xml_links[i]["priority"];
    else
      cl.priority = 1;
 
    if(xml_links[i].hasMember("group"))
      cl.group = std::string(xml_links[i]["group"]);
    else
    {
      ROS_WARN("Description of collision link '%s' is missing the group.", cl.name.c_str());
      return false;
    }
  
    cl_.push_back(cl);
    if(cl.group.compare("base") == 0)
      basel_.push_back(cl);
    else if(cl.group.compare("torso") == 0)
      torsol_.push_back(cl);
    else if(cl.group.compare("head") == 0)
      headl_.push_back(cl);
    else
    {
      ROS_WARN("%s link is in the %s group and it isn't supported. Right now, only base, torso, head groups are supported.", cl.name.c_str(),cl.group.c_str());
    }
  }
  ROS_INFO("Successfully parsed list of %d collision links.", int(cl_.size()));
  return true;
}

void SBPLDualCollisionSpace::printCollisionLinks()
{
  for(size_t i = 0; i < cl_.size(); ++i)
  {
    ROS_INFO("-------------------------------------");
    ROS_INFO("%d: %s", int(i), cl_[i].name.c_str());
    ROS_INFO("x1: %0.3f  y1: %0.3f  z1: %0.3f", cl_[i].x1, cl_[i].y1, cl_[i].z1);
    ROS_INFO("x2: %0.3f  y2: %0.3f  z2: %0.3f", cl_[i].x2, cl_[i].y2, cl_[i].z2);
    ROS_INFO("radius: %0.3fm", cl_[i].radius);
    ROS_INFO("priority: %d", cl_[i].priority);
    ROS_INFO("frame: %s", cl_[i].frame.c_str());
    if(i == cl_.size()-1)
      ROS_INFO("-------------------------------------");
  }
}

bool SBPLDualCollisionSpace::getSphereGroups()
{
  XmlRpc::XmlRpcValue all_groups;
  Sphere s;
  Group g;
  std::string groups_name = "groups";

  ros::NodeHandle ph("~");
  ph.param<std::string>("full_body_kinematics_chain/root_frame", full_body_chain_root_name_, "base_footprint");
  ph.param<std::string>("full_body_kinematics_chain/tip_frame", full_body_chain_tip_name_, "head_tilt_link");

  if(!initFullBodyKinematics())
  {
    ROS_ERROR("[cspace] Failed to initialize the full body kinematics chain.");
    return false;
  }

  if(!ph.hasParam(groups_name))
  {
    ROS_WARN_STREAM("No groups for planning specified in " << groups_name);
    return false;
  }
  ph.getParam(groups_name, all_groups);

  if(all_groups.getType() != XmlRpc::XmlRpcValue::TypeArray) 
  {
    ROS_WARN("Groups is not an array.");
    return false;
  }

  if(all_groups.size() == 0) 
  {
    ROS_WARN("No groups in groups");
    return false;
  }

  for(int i = 0; i < all_groups.size(); i++) 
  {
    if(!all_groups[i].hasMember("name"))
    {
      ROS_WARN("All groups must have a name.");
      return false;
    } 
    g.name = std::string(all_groups[i]["name"]);

    if(!all_groups[i].hasMember("frame"))
    {
      ROS_WARN("All groups must have a frame.");
      return false;
    }
    g.root_frame = std::string(all_groups[i]["frame"]);

    for(size_t k = 0; k < full_body_chain_.getNrOfSegments(); ++k)
    {
      if(full_body_chain_.getSegment(k).getName().compare(g.root_frame) == 0)
      {
        g.kdl_segment = k + 1;
        ROS_INFO("[cspace] %s group is rooted at %s with kdl segment #%d", g.name.c_str(), g.root_frame.c_str(), int(k));
        break;
      }
    }

    if((g.name.compare("right_gripper") == 0) || (g.name.compare("left_gripper") == 0))
      g.kdl_segment = 10;
    else if((g.name.compare("right_forearm") == 0) || (g.name.compare("left_forearm") == 0))
      g.kdl_segment = 7;

    std::stringstream ss(all_groups[i]["spheres"]);
    double x,y,z;
    g.spheres.clear();
    std::string sphere_name;
    while(ss >> sphere_name)
    {
      ros::NodeHandle s_nh(ph, sphere_name);
      s.name = sphere_name;
      s_nh.param("x", x, 0.0);
      s_nh.param("y", y, 0.0);
      s_nh.param("z", z, 0.0);
      s_nh.param("radius", s.radius, 0.0);
      s_nh.param("priority", s.priority, 1);
      s.v.x(x);
      s.v.y(y);
      s.v.z(z);
      s.radius_c = s.radius / arm_[0]->resolution_ + 0.5;
      g.spheres.push_back(s);
    }
    
    if(g.name.compare("base") == 0)
      base_g_ = g;
    else if(g.name.compare("torso_upper") == 0)
      torso_upper_g_ = g;
    else if(g.name.compare("torso_lower") == 0)
      torso_lower_g_ = g;
    else if(g.name.compare("turrets") == 0)
      turrets_g_ = g;
    else if(g.name.compare("tilt_laser") == 0)
      tilt_laser_g_ = g;
    else if(g.name.compare("head") == 0)
      head_g_ = g;
    else if(g.name.compare("left_gripper") == 0)
      lgripper_g_ = g;
    else if(g.name.compare("right_gripper") == 0)
      rgripper_g_ = g;
    else if(g.name.compare("left_forearm") == 0)
      lforearm_g_ = g;
    else if(g.name.compare("right_forearm") == 0)
      rforearm_g_ = g;
    else
    {
      ROS_ERROR("The list of spheres contains a group with an unrecognized name, '%s'. Temporarily, only specific names are supported.Exiting.", g.name.c_str());
      return false;
    }
  }

  // make a master list of sphere groups
  all_g_.push_back(base_g_);
  all_g_.push_back(torso_upper_g_);
  all_g_.push_back(torso_lower_g_);
  all_g_.push_back(turrets_g_);
  all_g_.push_back(tilt_laser_g_);
  all_g_.push_back(rgripper_g_);
  all_g_.push_back(lgripper_g_);
  all_g_.push_back(rforearm_g_);
  all_g_.push_back(lforearm_g_);
  all_g_.push_back(head_g_);
  
  ROS_INFO("Successfully parsed collision groups.");
  return true;
}

void SBPLDualCollisionSpace::printSphereGroups()
{
  for(size_t i = 0; i < all_g_.size(); ++i)
  {
    ROS_INFO("----------------[%d]-------------------",int(i));
    ROS_INFO("group: %s", all_g_[i].name.c_str());
    ROS_INFO("frame: %s", all_g_[i].root_frame.c_str());

    if(all_g_[i].spheres.size() == 0)
      ROS_WARN("(no spheres)"); 
    for(size_t j = 0; j < all_g_[i].spheres.size(); ++j)
      ROS_INFO("[%s] x: %0.3f  y: %0.3f  z: %0.3f  radius: %0.3fm  priority: %d", all_g_[i].spheres[j].name.c_str(), all_g_[i].spheres[j].v.x(), all_g_[i].spheres[j].v.y(), all_g_[i].spheres[j].v.z(), all_g_[i].spheres[j].radius, all_g_[i].spheres[j].priority);
    
    if(i == all_g_.size()-1)
      ROS_INFO("-------------------------------------");   
  }
}

bool SBPLDualCollisionSpace::initFullBodyKinematics()
{
  ros::NodeHandle nh("~");
  std::string robot_description;
  std::string robot_param;

  nh.searchParam("robot_description",robot_param);
  nh.param<std::string>(robot_param,robot_description,"");

  if(robot_description.empty())
  {
    SBPL_ERROR("[cspace] Unable to get robot_description from param server.");
    return false;
  }

  if(!kdl_parser::treeFromString(robot_description, full_body_tree_))
  {
    SBPL_ERROR("[cspace] Failed to parse tree from robot_description on param server.");
    return false;;
  }

  if(!full_body_tree_.getChain(full_body_chain_root_name_, full_body_chain_tip_name_, full_body_chain_))
  {
    SBPL_ERROR("[cspace] Failed to fetch the KDL chain for the desired robot frames. Exiting.");
    return false;
  }

  fk_solver_ = new KDL::ChainFkSolverPos_recursive(full_body_chain_);
  jnt_array_.resize(full_body_chain_.getNrOfJoints());

  ROS_INFO("[cspace] The full body kinematics chain has %d segments & %d joints.", full_body_chain_.getNrOfSegments(), full_body_chain_.getNrOfJoints());
  ROS_INFO("[cspace] root: %s tip: %s.", full_body_chain_root_name_.c_str(), full_body_chain_tip_name_.c_str());

  printKDLChain("full body chain", full_body_chain_);
  return true;
}

void SBPLDualCollisionSpace::printKDLChain(std::string name, KDL::Chain &chain)
{
  ROS_INFO("chain: %s", name.c_str());
  for(unsigned int j = 0; j < chain.getNrOfSegments(); ++j)
  {
    ROS_INFO("[%2d] segment: % 0.3f % 0.3f %0.3f joint: % 0.3f % 0.3f % 0.3f type: %9s name: %21s",j,
        chain.getSegment(j).pose(0).p.x(),
        chain.getSegment(j).pose(0).p.y(),
        chain.getSegment(j).pose(0).p.z(),
        chain.getSegment(j).getJoint().pose(0).p.x(),
        chain.getSegment(j).getJoint().pose(0).p.y(),
        chain.getSegment(j).getJoint().pose(0).p.z(),
        chain.getSegment(j).getJoint().getTypeName().c_str(),
        chain.getSegment(j).getJoint().getName().c_str());
  }
  ROS_INFO(" ");
}

void SBPLDualCollisionSpace::setNonPlanningJointPosition(std::string name, double value)
{
  if(name.compare("head_tilt_joint") == 0)
    head_tilt_angle_ = value;
  else if(name.compare("head_pan_joint") == 0)
    head_pan_angle_ = value;
  else
    ROS_ERROR("[cspace] The %s is not a known non-planning joint.", name.c_str());
}

bool SBPLDualCollisionSpace::computeFullBodyKinematics(double x, double y, double theta, double torso, int frame_num, KDL::Frame &fk_out)
{
  jnt_array_(0) = torso; // fill in torso height
  //TODO: Fill in head pan and tilt angles to be extra fancy

  if(fk_solver_->JntToCart(jnt_array_, fk_out, frame_num) < 0)
  {
    ROS_ERROR("JntToCart returned < 0. Exiting.");
    return false;
  }

  getMaptoRobotTransform(x,y,theta,map_to_robot_);
  fk_out = map_to_robot_*fk_out;

  return true;
}

void SBPLDualCollisionSpace::getMaptoRobotTransform(double x, double y, double theta, KDL::Frame &frame)
{
  KDL::Rotation r1;
  r1.DoRotZ(theta);
  KDL::Vector t1(x,y,0.0);
  KDL::Frame base_footprint_in_map(r1,t1);
  frame = base_footprint_in_map;
}

void SBPLDualCollisionSpace::getVoxelsInGroup(KDL::Frame &frame, Group &group)
{
  KDL::Vector v;
  for(size_t i = 0; i < group.spheres.size(); ++i)
  {
    v = frame*group.spheres[i].v;
    grid_->worldToGrid(v.data[0],v.data[1],v.data[2],group.spheres[i].voxel[0], group.spheres[i].voxel[1], group.spheres[i].voxel[2]);
  }
}

void SBPLDualCollisionSpace::getPointsInGroup(KDL::Frame &frame, Group &group, std::vector<std::vector<double> > &points)
{
  KDL::Vector v;
  points.resize(group.spheres.size(), std::vector<double>(4,0));
  for(size_t i = 0; i < group.spheres.size(); ++i)
  {
    v = frame*group.spheres[i].v;
    points[i][0] = v.data[0];
    points[i][1] = v.data[1];
    points[i][2] = v.data[2];
    points[i][3] = group.spheres[i].radius;
  }
}

bool SBPLDualCollisionSpace::isBaseValid(double x, double y, double theta, unsigned char &dist)
{
  unsigned char dist_temp;

  getMaptoRobotTransform(x,y,theta,base_g_.f);

  // base
  getVoxelsInGroup(base_g_.f, base_g_);
  for(size_t i = 0; i < base_g_.spheres.size(); ++i)
  {
    if((dist_temp = grid_->getCell(base_g_.spheres[i].voxel[0], base_g_.spheres[i].voxel[1], base_g_.spheres[i].voxel[2])) <= base_g_.spheres[i].radius_c)
    {
      dist = dist_temp;
      return false;
    }
    
    if(dist > dist_temp)
      dist = dist_temp;
  }

  // lower torso
  torso_lower_g_.f = base_g_.f;
  getVoxelsInGroup(base_g_.f, torso_lower_g_);
  for(size_t i = 0; i < torso_lower_g_.spheres.size(); ++i)
  {
    if((dist_temp = grid_->getCell(torso_lower_g_.spheres[i].voxel[0], torso_lower_g_.spheres[i].voxel[1], torso_lower_g_.spheres[i].voxel[2])) <= torso_lower_g_.spheres[i].radius_c)
    {
      dist = dist_temp;
      return false;
    }
    
    if(dist > dist_temp)
      dist = dist_temp;
  }
  return true;
}

bool SBPLDualCollisionSpace::isTorsoValid(double x, double y, double theta, double torso, unsigned char &dist)
{
  unsigned char dist_temp;

  ROS_DEBUG("[cspace] Checking Torso. x: %0.3f y: %0.3f theta: %0.3f torso: %0.3f torso_kdl_num: %d",x,y,theta,torso,torso_upper_g_.kdl_segment);
  if(!computeFullBodyKinematics(x,y,theta,torso, torso_upper_g_.kdl_segment, torso_upper_g_.f))
  {
    ROS_ERROR("[cspace] Failed to compute FK for torso.");
    return false;
  }

  // upper torso
  getVoxelsInGroup(torso_upper_g_.f, torso_upper_g_);
  for(size_t i = 0; i < torso_upper_g_.spheres.size(); ++i)
  {
    if((dist_temp = grid_->getCell(torso_upper_g_.spheres[i].voxel[0], torso_upper_g_.spheres[i].voxel[1], torso_upper_g_.spheres[i].voxel[2])) <= torso_upper_g_.spheres[i].radius_c)
    {
      dist = dist_temp;
      return false;
    }
    
    if(dist > dist_temp)
      dist = dist_temp;
  }
  // tilt laser
  tilt_laser_g_.f = torso_upper_g_.f;
  getVoxelsInGroup(tilt_laser_g_.f, tilt_laser_g_);
  for(size_t i = 0; i < tilt_laser_g_.spheres.size(); ++i)
  {
    if((dist_temp = grid_->getCell(tilt_laser_g_.spheres[i].voxel[0], tilt_laser_g_.spheres[i].voxel[1], tilt_laser_g_.spheres[i].voxel[2])) <= tilt_laser_g_.spheres[i].radius_c)
    {
      dist = dist_temp;
      return false;
    }
    
    if(dist > dist_temp)
      dist = dist_temp;
  }
  // turrets
  turrets_g_.f = torso_upper_g_.f;
  getVoxelsInGroup(turrets_g_.f, turrets_g_);
  for(size_t i = 0; i < turrets_g_.spheres.size(); ++i)
  {
    if((dist_temp = grid_->getCell(turrets_g_.spheres[i].voxel[0], turrets_g_.spheres[i].voxel[1], turrets_g_.spheres[i].voxel[2])) <= turrets_g_.spheres[i].radius_c)
    {
      dist = dist_temp;
      return false;
    }
    
    if(dist > dist_temp)
      dist = dist_temp;
  }

  return true;
}

bool SBPLDualCollisionSpace::isHeadValid(double x, double y, double theta, double torso, unsigned char &dist)
{
  unsigned char dist_temp;

  ROS_DEBUG("[cspace] Checking Head. x: %0.3f y: %0.3f theta: %0.3f torso: %0.3f head_kdl_num: %d",x,y,theta,torso,head_g_.kdl_segment);
  if(!computeFullBodyKinematics(x,y,theta,torso, head_g_.kdl_segment, head_g_.f))
  {
    ROS_ERROR("[cspace] Failed to compute FK for torso.");
    return false;
  }

  // head
  getVoxelsInGroup(head_g_.f, head_g_);
  for(size_t i = 0; i < head_g_.spheres.size(); ++i)
  {
    if((dist_temp = grid_->getCell(head_g_.spheres[i].voxel[0], head_g_.spheres[i].voxel[1], head_g_.spheres[i].voxel[2])) <= head_g_.spheres[i].radius_c)
    {
      dist = dist_temp;
      return false;
    }
    
    if(dist > dist_temp)
      dist = dist_temp;
  }
  return true;
}

bool SBPLDualCollisionSpace::isBodyValid(double x, double y, double theta, double torso, unsigned char &dist)
{
  return isBaseValid(x,y,theta,dist) &
         isTorsoValid(x,y,theta,torso,dist) &
         isHeadValid(x,y,theta,torso,dist);
}

bool SBPLDualCollisionSpace::checkCollisionArmsToGroup(Group &group, unsigned char &dist)
{
  //NOTE: Assumes you computed the kinematics for all the joints already.
  int d;
  /*
  printGroupVoxels(group, group.name);
  printGroupVoxels(rgripper_g_, "right gripper");
  printGroupVoxels(lgripper_g_, "left gripper");
  printGroupVoxels(rforearm_g_, "right forearm");
  printGroupVoxels(lforearm_g_, "left forearm");
  */

  for(size_t i = 0; i < group.spheres.size(); ++i)
  {
    // vs right gripper
    for(size_t j = 0; j < rgripper_g_.spheres.size(); ++j)
    {
      if((d = getDistanceBetweenPoints(rgripper_g_.spheres[j].voxel[0], rgripper_g_.spheres[j].voxel[1], rgripper_g_.spheres[j].voxel[2],group.spheres[i].voxel[0], group.spheres[i].voxel[1], group.spheres[i].voxel[2])) <= max(rgripper_g_.spheres[j].radius_c, group.spheres[i].radius_c))
      {
        if(d < dist)
          dist = d;
        ROS_INFO("COLLISION");
        return false;
      }
      dist = min(d, int(dist));
      //ROS_INFO("[cspace] [%s-right_gripper] sphere: %d arm_sphere: %d dist: %d   arm_radius: %d  other_radius: %d", group.name.c_str(), int(i), int(j), d, rgripper_g_.spheres[j].radius_c, group.spheres[i].radius_c);
    }
    // vs left gripper
    for(size_t j = 0; j < lgripper_g_.spheres.size(); ++j)
    {
      if((d = getDistanceBetweenPoints(lgripper_g_.spheres[j].voxel[0], lgripper_g_.spheres[j].voxel[1], lgripper_g_.spheres[j].voxel[2],group.spheres[i].voxel[0], group.spheres[i].voxel[1], group.spheres[i].voxel[2])) <= max(lgripper_g_.spheres[j].radius_c, group.spheres[i].radius_c))
      {
        if(d < dist)
          dist = d;
        ROS_INFO("COLLISION");
        return false;
      }
      dist = min(d, int(dist));
      //ROS_INFO("[cspace] [%s-left_gripper] sphere: %d arm_sphere: %d dist: %d   arm_radius: %d  other_radius: %d", group.name.c_str(), int(i), int(j), d, lgripper_g_.spheres[j].radius_c, group.spheres[i].radius_c);
    }
    // vs right forearm
    for(size_t j = 0; j < rforearm_g_.spheres.size(); ++j)
    {
      if((d = getDistanceBetweenPoints(rforearm_g_.spheres[j].voxel[0], rforearm_g_.spheres[j].voxel[1], rforearm_g_.spheres[j].voxel[2],group.spheres[i].voxel[0], group.spheres[i].voxel[1], group.spheres[i].voxel[2])) <= max(rforearm_g_.spheres[j].radius_c, group.spheres[i].radius_c))
      {
        if(d < dist)
          dist = d;
        ROS_INFO("COLLISION");
        return false;
      }
      dist = min(d, int(dist));
      //ROS_INFO("[cspace] [%s-right_forearm] sphere: %d arm_sphere: %d dist: %d  arm_radius: %d  other_radius: %d", group.name.c_str(), int(i), int(j), d, rforearm_g_.spheres[j].radius_c, group.spheres[i].radius_c);
    }
    // vs left forearm
    for(size_t j = 0; j < lforearm_g_.spheres.size(); ++j)
    {
      if((d = getDistanceBetweenPoints(lforearm_g_.spheres[j].voxel[0], lforearm_g_.spheres[j].voxel[1], lforearm_g_.spheres[j].voxel[2],group.spheres[i].voxel[0], group.spheres[i].voxel[1], group.spheres[i].voxel[2])) <= max(lforearm_g_.spheres[j].radius_c, group.spheres[i].radius_c))
      {
        if(d < int(dist))
          dist = d;
        ROS_INFO("COLLISION");
        return false;
      }
      dist = min(d, int(dist));
      //ROS_INFO("[cspace] [%s-left_forearm] sphere: %d arm_sphere: %d dist: %d  arm_radius: %d  other_radius: %d", group.name.c_str(), int(i), int(j), d, lforearm_g_.spheres[j].radius_c, group.spheres[i].radius_c);
    }
  }
  return true;
}

bool SBPLDualCollisionSpace::checkCollisionArmsToBody(std::vector<double> &langles, std::vector<double> &rangles, BodyPose &pose, unsigned char &dist)
{
  // compute arm kinematics
  arm_[0]->computeFK(rangles, pose, 10, &(rgripper_g_.f)); // right gripper
  getVoxelsInGroup(rgripper_g_.f, rgripper_g_); 
  arm_[0]->computeFK(rangles, pose, 7, &(rforearm_g_.f)); // right forearm
  getVoxelsInGroup(rforearm_g_.f, rforearm_g_); 
  arm_[1]->computeFK(langles, pose, 10, &(lgripper_g_.f)); // left gripper
  getVoxelsInGroup(lgripper_g_.f, lgripper_g_); 
  arm_[1]->computeFK(langles, pose, 7, &(lforearm_g_.f)); // left forearm
  getVoxelsInGroup(lforearm_g_.f, lforearm_g_); 
 
  if(!checkCollisionArmsToGroup(base_g_, dist))
  {
    ROS_INFO("[cspace] base - arms collision. (dist: %d)",dist);
    return false;
  }
  if(!checkCollisionArmsToGroup(turrets_g_, dist))
  {
    ROS_INFO("[cspace] turrets - arms collision. (dist: %d)",dist);
    return false;
  }
  if(!checkCollisionArmsToGroup(tilt_laser_g_, dist))
  {
    ROS_INFO("[cspace] tilt_laser - arms collision. (dist: %d)",dist);
    return false;
  }
  if(!checkCollisionArmsToGroup(torso_upper_g_, dist))
  {
    ROS_INFO("[cspace] torso_upper - arms collision. (dist: %d)",dist);
    return false;
  }
  if(!checkCollisionArmsToGroup(torso_lower_g_, dist))
  {
    ROS_INFO("[cspace] torso_lower - arms collision. (dist: %d)",dist);
    return false;
  }
  if(!checkCollisionArmsToGroup(head_g_, dist))
  {
  ROS_INFO("[cspace] head - arms collision. (dist: %d)",dist);
  return false;
  }

  return true;
}

void SBPLDualCollisionSpace::getCollisionSpheres(std::vector<double> &langles, std::vector<double> &rangles, BodyPose &pose, std::string group_name, std::vector<std::vector<double> > &spheres)
{
  Group g;
  if(group_name.compare("base") == 0)
  {
    g = base_g_;
    if(!computeFullBodyKinematics(pose.x,pose.y,pose.theta,pose.z, g.kdl_segment, g.f))
    {
      ROS_ERROR("[cspace] Failed to compute FK.");
      return;
    }
  }
  else if(group_name.compare("torso_upper") == 0)
  {
    g = torso_upper_g_;
    if(!computeFullBodyKinematics(pose.x,pose.y,pose.theta,pose.z, g.kdl_segment, g.f))
    {
      ROS_ERROR("[cspace] Failed to compute FK.");
      return;
    }
  }
  else if(group_name.compare("torso_lower") == 0)
  {
    g = torso_lower_g_;
    if(!computeFullBodyKinematics(pose.x,pose.y,pose.theta,pose.z, g.kdl_segment, g.f))
    {
      ROS_ERROR("[cspace] Failed to compute FK.");
      return;
    }
  }
  else if(group_name.compare("turrets") == 0)
  {
    g = turrets_g_;
    if(!computeFullBodyKinematics(pose.x,pose.y,pose.theta,pose.z, g.kdl_segment, g.f))
    {
      ROS_ERROR("[cspace] Failed to compute FK.");
      return;
    }
  }
  else if(group_name.compare("tilt_laser") == 0)
  {
    g = tilt_laser_g_;
    if(!computeFullBodyKinematics(pose.x,pose.y,pose.theta,pose.z, g.kdl_segment, g.f))
    {
      ROS_ERROR("[cspace] Failed to compute FK.");
      return;
    }
  }
  else if(group_name.compare("head") == 0)
  {
    g = head_g_;
    if(!computeFullBodyKinematics(pose.x,pose.y,pose.theta,pose.z, g.kdl_segment, g.f))
    {
      ROS_ERROR("[cspace] Failed to compute FK.");
      return;
    }
  }
  else if(group_name.compare("left_gripper") == 0)
  {
    g = lgripper_g_;
    arm_[1]->computeFK(langles, pose, g.kdl_segment, &(g.f));
  }
  else if(group_name.compare("right_gripper") == 0)
  {
    g = rgripper_g_;
    arm_[0]->computeFK(rangles, pose, g.kdl_segment, &(g.f));
  }
  else if(group_name.compare("left_forearm") == 0)
  {
    g = lforearm_g_;
    arm_[1]->computeFK(langles, pose, g.kdl_segment, &(g.f));
  }
  else if(group_name.compare("right_forearm") == 0)
  {
    g = rforearm_g_;
    arm_[0]->computeFK(rangles, pose, g.kdl_segment, &(g.f));
  }
  
  getPointsInGroup(g.f, g, spheres);
  ROS_DEBUG("[cspace] Returning %d spheres for %s group", int(spheres.size()), group_name.c_str());
}

void SBPLDualCollisionSpace::printGroupVoxels(Group &g, std::string text)
{
  ROS_INFO("[cspace] voxels: %s", text.c_str());
  for(size_t i = 0; i < g.spheres.size(); ++i)
  {
    ROS_INFO("[cspace] voxel %d: %d %d %d", int(i), g.spheres[i].voxel[0], g.spheres[i].voxel[1], g.spheres[i].voxel[2]);
  }
}

bool SBPLDualCollisionSpace::checkSpineMotion(std::vector<double> &langles, std::vector<double> &rangles, BodyPose &pose, bool verbose, unsigned char &dist, int &debug_code)
{
  // arm-base
  getMaptoRobotTransform(pose.x,pose.y,pose.theta,base_g_.f);
  getVoxelsInGroup(base_g_.f, base_g_);
  if(!checkCollisionArmsToGroup(base_g_, dist))
    return false;

  // torso-world
  if(!isTorsoValid(pose.x, pose.y, pose.theta, pose.z, dist))
    return false;

  // head-world
  if(!isHeadValid(pose.x, pose.y, pose.theta, pose.z, dist))
    return false;

  // arms-world
  if(!checkCollision(rangles, pose, 0, verbose, dist))
  {
    debug_code = sbpl_arm_planner::RIGHT_ARM_IN_COLLISION;
    return false;
  }
  if(!checkCollision(langles, pose, 1, verbose, dist))
  {
    debug_code = sbpl_arm_planner::LEFT_ARM_IN_COLLISION;
    return false;
  }
 
  return true;
}

bool SBPLDualCollisionSpace::checkBaseMotion(std::vector<double> &langles, std::vector<double> &rangles, BodyPose &pose, bool verbose, unsigned char &dist, int &debug_code)
{
  // base-world
  if(!isBaseValid(pose.x, pose.y, pose.theta, dist))
  {
    //ROS_INFO("[cspace] base-world collision.");
    return false;
  }

  // arms-world
  if(!checkCollision(rangles, pose, 0, verbose, dist))
  {
    debug_code = sbpl_arm_planner::RIGHT_ARM_IN_COLLISION;
    return false;
  }
  if(!checkCollision(langles, pose, 1, verbose, dist))
  {
    debug_code = sbpl_arm_planner::LEFT_ARM_IN_COLLISION;
    return false;
  }

  // torso-world
  if(!isTorsoValid(pose.x, pose.y, pose.theta, pose.z, dist))
    return false;

  // head-world
  if(!isHeadValid(pose.x, pose.y, pose.theta, pose.z, dist))
    return false;
 
  return true;
}

bool SBPLDualCollisionSpace::checkArmsMotion(std::vector<double> &langles, std::vector<double> &rangles, BodyPose &pose, bool verbose, unsigned char &dist, int &debug_code)
{
  // arms-world, arms-arms
  if(!checkCollisionArms(langles, rangles, pose, verbose, dist, debug_code))
    return false;

  // arms-body
  if(!checkCollisionArmsToBody(langles, rangles, pose, dist))
    return false;

  return true;
}

bool SBPLDualCollisionSpace::checkAllMotion(std::vector<double> &langles, std::vector<double> &rangles, BodyPose &pose, bool verbose, unsigned char &dist, int &debug_code)
{
  // arms-world, arms-arms
  if(!checkCollisionArms(langles, rangles, pose, verbose, dist, debug_code))
    return false;

  // body-world
  if(!isBodyValid(pose.x, pose.y, pose.theta, pose.z, dist))
    return false;

  // arms-body
  if(!checkCollisionArmsToBody(langles, rangles, pose, dist))
    return false;

  return true;
}

}
