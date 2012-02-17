#include <sbpl_full_body_tests/sbpl_full_body_experiment.h>
#include <sys/stat.h>
#include <dirent.h>

SBPLFullBodyExperiment::SBPLFullBodyExperiment() : ph_("~")
{
  action_list_.push_back("nothing");
  action_list_.push_back("attach");
  action_list_.push_back("detach");
  action_map_["nothing"] = 0;
  action_map_["attach"] = 1;
  action_map_["detach"] = 2;

  trajectory_folder_path_ = "/tmp";

  use_current_state_as_start_ = false;

  collision_object_pub_ = nh_.advertise<mapping_msgs::CollisionObject>("collision_object", 5);
  attached_object_pub_ = nh_.advertise<mapping_msgs::AttachedCollisionObject>("attached_collision_object", 1);
  sbpl_attached_object_pub_ = nh_.advertise<mapping_msgs::AttachedCollisionObject>("sbpl_attached_collision_object", 1);

  ROS_INFO("[exp] Waiting for the planning service..Did you start it up first?.");
  ros::service::waitForService(PLANNING_SERVICE);
  planner_client_ = nh_.serviceClient<sbpl_two_arm_planner_node::GetTwoArmPlan>(PLANNING_SERVICE, true);
  ROS_INFO("[exp] Connected to the planning service.");

  goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("goal", 1, &SBPLFullBodyExperiment::sendGoal, this);
}

bool SBPLFullBodyExperiment::getParams()
{
  XmlRpc::XmlRpcValue plist;
  double t;
  std::string p;

  ph_.param<std::string>("known_objects_filename",known_objects_filename_, "");
  ph_.param<std::string>("trajectory_folder_path",trajectory_folder_path_, "/tmp");
  time_t clock;
  time(&clock);
  std::string time(ctime(&clock));;
  time.erase(time.size()-1, 1);
  trajectory_files_path_ = trajectory_folder_path_ + "/" + time;
  if(mkdir(trajectory_files_path_.c_str(),  S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0)
    ROS_INFO("Successfully created the trajectory folder: %s", trajectory_files_path_.c_str());
  else
    ROS_WARN("Failed to create the trajectory folder: %s", trajectory_files_path_.c_str());
  
  if(ph_.hasParam("goal_tolerance/xyz"))
  {
    ph_.getParam("goal_tolerance/xyz", plist);
    std::stringstream ss(plist);
    while(ss >> p)
      goal_tolerance_.push_back(atof(p.c_str()));

    ph_.getParam("goal_tolerance/yaw",t);
    goal_tolerance_.push_back(t);
  }
  else
  {
    goal_tolerance_.resize(4,0.02);
    goal_tolerance_[3] = 0.12;
  }
 
  if(ph_.hasParam("object_pose_in_gripper"))
  {
    rarm_object_offset_.clear();
    larm_object_offset_.clear();
    ph_.getParam("object_pose_in_gripper/right/xyz", plist);
    std::stringstream ss(plist);
    while(ss >> p)
      rarm_object_offset_.push_back(atof(p.c_str()));

    ph_.getParam("object_pose_in_gripper/right/rpy", plist); 
    std::stringstream ss1(plist);
    while(ss1 >> p)
      rarm_object_offset_.push_back(atof(p.c_str()));
    
    ph_.getParam("object_pose_in_gripper/left/xyz", plist);
    std::stringstream ss2(plist);
    while(ss2 >> p)
      larm_object_offset_.push_back(atof(p.c_str()));

    ph_.getParam("object_pose_in_gripper/left/rpy", plist); 
    std::stringstream ss3(plist);
    while(ss3 >> p)
      larm_object_offset_.push_back(atof(p.c_str()));
  }
  
  btQuaternion btoffset;
  rarm_object_pose_.position.x = rarm_object_offset_[0];
  rarm_object_pose_.position.y = rarm_object_offset_[1];
  rarm_object_pose_.position.z = rarm_object_offset_[2];
  larm_object_pose_.position.x = larm_object_offset_[0];
  larm_object_pose_.position.y = larm_object_offset_[1];
  larm_object_pose_.position.z = larm_object_offset_[2];
  btoffset.setRPY(rarm_object_offset_[3],rarm_object_offset_[4],rarm_object_offset_[5]);
  tf::quaternionTFToMsg(btoffset,rarm_object_pose_.orientation);
  btoffset.setRPY(larm_object_offset_[3],larm_object_offset_[4],larm_object_offset_[5]);
  tf::quaternionTFToMsg(btoffset,larm_object_pose_.orientation);

  if(ph_.hasParam("use_current_pose_as_start_state"))
    ph_.getParam("use_current_pose_as_start_state", use_current_state_as_start_);
  
  if(ph_.hasParam("start_state"))
  {
    start_pose_.rangles.clear();
    start_pose_.langles.clear();
    std::vector<double> bpose;
    ph_.getParam("start_state/right", plist);
    std::stringstream ss(plist);
    while(ss >> p)
      start_pose_.rangles.push_back(atof(p.c_str()));
    ph_.getParam("start_state/left", plist);
    std::stringstream ss1(plist);
    ss.str(plist);
    while(ss1 >> p)
      start_pose_.langles.push_back(atof(p.c_str()));
    ph_.getParam("start_state/base", plist);
    std::stringstream ss2(plist);
    while(ss2 >> p)
      bpose.push_back(atof(p.c_str()));
    if(bpose.size() == 3)
    {
      start_pose_.body.x = bpose[0];
      start_pose_.body.y = bpose[1];
      start_pose_.body.theta = bpose[2];
    }
    ph_.getParam("start_state/spine", start_pose_.body.z);  
  }

  if(goal_tolerance_.size() < 4 || 
      start_pose_.rangles.size() < 7 ||
      start_pose_.langles.size() < 7 ||
      rarm_object_offset_.size() < 6 ||
      larm_object_offset_.size() < 6)
    return false;

  if(!getLocations() || !getExperiments())
    return false;

  return true;
}

bool SBPLFullBodyExperiment::getLocations()
{
  XmlRpc::XmlRpcValue loc_list;
  geometry_msgs::Pose p;
  std::string name;
  std::string loc_name = "locations";

  if(!ph_.hasParam(loc_name))
  {
    ROS_WARN("[exp] No list of locations found on the param server.");
    return false;
  }
  ph_.getParam(loc_name, loc_list);

  if(loc_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_WARN("[exp] Location list is not an array. Something is wrong...exiting.");
    return false;
  }

  if(loc_list.size() == 0)
  {
    ROS_ERROR("[exp] List of locations is empty.");
    return false;
  }

  for(int i = 0; i < loc_list.size(); ++i)
  {
    if(!loc_list[i].hasMember("name"))
    {
      ROS_ERROR("Each location must have a name.");
      return false;
    }
    name = std::string(loc_list[i]["name"]);
    std::stringstream ss(loc_list[i]["pose"]);
    std::string p;
    while(ss >> p)
      loc_map_[name].push_back(atof(p.c_str()));
  }

  ROS_INFO("[exp] Successfully fetched %d locations from param server.", int(loc_list.size()));
  return true;
}

bool SBPLFullBodyExperiment::getExperiments()
{
  XmlRpc::XmlRpcValue exp_list;
  Experiment e;
  std::string exp_name = "experiments";
  XmlRpc::XmlRpcValue plist;
  std::string p;

  if(!ph_.hasParam(exp_name))
  {
    ROS_WARN("No list of experiments found on the param server.");
    return false;
  }
  ph_.getParam(exp_name, exp_list);

  if(exp_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_WARN("Experiment list is not an array. Something is wrong...exiting.");
    return false;
  }

  if(exp_list.size() == 0)
  {
    ROS_ERROR("List of experiments is empty.");
    return false;
  }

  for(int i = 0; i < exp_list.size(); ++i)
  {
    if(!exp_list[i].hasMember("name"))
    {
      ROS_ERROR("Each experiment must have a name.");
      return false;
    }
    e.name = std::string(exp_list[i]["name"]);

    if(!exp_list[i].hasMember("goal"))
    {
      ROS_ERROR("Each experiment must have a goal....duh.");
      return false;
    }
    e.goal = std::string(exp_list[i]["goal"]);

    if(!exp_list[i].hasMember("pre_action"))
      e.pre_action = 0;
    else
      e.pre_action = action_map_[exp_list[i]["pre_action"]];

    if(!exp_list[i].hasMember("post_action"))
      e.post_action = 0;
    else
      e.post_action = action_map_[exp_list[i]["post_action"]];

    if(!exp_list[i].hasMember("sound_bite"))
      e.sound_bite = "";
    else
      e.sound_bite = std::string(exp_list[i]["sound_bite"]);

    if(exp_list[i].hasMember("start"))
    {
      e.start.rangles.clear();
      e.start.langles.clear();
      std::vector<double> bpose;
      plist = exp_list[i]["start"]["right"];
      std::stringstream ss(plist);
      while(ss >> p)
        e.start.rangles.push_back(atof(p.c_str()));
      
      plist = exp_list[i]["start"]["left"];
      std::stringstream ss1(plist);
      while(ss1 >> p)
        e.start.langles.push_back(atof(p.c_str()));
      
      plist = exp_list[i]["start"]["base"];
      std::stringstream ss2(plist);
      while(ss2 >> p)
        bpose.push_back(atof(p.c_str()));
      if(bpose.size() == 3)
      {
        e.start.body.x = bpose[0];
        e.start.body.y = bpose[1];
        e.start.body.theta = bpose[2];
      }
      e.start.body.z = double(exp_list[i]["start"]["torso"]);
    }
    else
    {
      if(!use_current_state_as_start_)
      {
        ROS_ERROR("[exp] No start state defined for %s and it isn't configured to use the current state as the start state.",e.name.c_str());
        return false; 
      }
      else  
        ROS_DEBUG("No start state defined for %s but it's OK because it's configured to use the current state as the start.",e.name.c_str());
    }

    ROS_INFO("Adding experiment: %s", e.name.c_str());
    exp_map_[e.name] = e;
  }

  return true;  
}

bool SBPLFullBodyExperiment::addCollisionObjects(std::string filename, std::vector<double> &offset)
{
  int num_obs;
  char sTemp[1024];
  std::vector<std::vector<double> > objects;
  std::vector<std::string> object_ids;
  mapping_msgs::CollisionObject object;

  FILE* fCfg = fopen(filename.c_str(), "r");
  if(fCfg == NULL)
    return false;

  // get number of objects
  if(fscanf(fCfg,"%d",&num_obs) < 1)
    ROS_INFO("[exp] Parsed string has length < 1.(number of obstacles)\n");

  ROS_INFO("[exp] Parsing collision object file with %i objects.",num_obs);

  //get {x y z dimx dimy dimz} for each object
  objects.resize(num_obs, std::vector<double>(10,0.0));
  object_ids.clear();
  for (int i=0; i < num_obs; ++i)
  {
    if(fscanf(fCfg,"%s",sTemp) < 1)
      ROS_INFO("[exp] Parsed string has length < 1.\n");
    object_ids.push_back(sTemp);

    for(int j=0; j < 10; ++j)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_INFO("[exp] Parsed string has length < 1. (object parameters for %s)\n", object_ids.back().c_str());
      if(!feof(fCfg) && strlen(sTemp) != 0)
        objects[i][j] = atof(sTemp);
    }
  }

  if(object_ids.size() != objects.size())
  {
    ROS_INFO("object id list is not same length as object list. exiting.");
    return false;
  }

  object.shapes.resize(1);
  object.poses.resize(1);
  object.shapes[0].dimensions.resize(3);
  object.shapes[0].triangles.resize(4);
  for(size_t i = 0; i < objects.size(); i++)
  {
    object.id = object_ids[i];
    object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
    object.shapes[0].type = geometric_shapes_msgs::Shape::BOX;
    object.header.frame_id = "map";
    object.header.stamp = ros::Time::now();

    object.poses[0].position.x = objects[i][0]+offset[0];
    object.poses[0].position.y = objects[i][1]+offset[1];
    object.poses[0].position.z = objects[i][2]+offset[2];
    object.poses[0].orientation.x = 0; 
    object.poses[0].orientation.y = 0; 
    object.poses[0].orientation.z = 0; 
    object.poses[0].orientation.w = 1;  

    object.shapes[0].dimensions[0] = objects[i][3];
    object.shapes[0].dimensions[1] = objects[i][4];
    object.shapes[0].dimensions[2] = objects[i][5];

    object.shapes[0].triangles[0] = objects[i][6];
    object.shapes[0].triangles[1] = objects[i][7];
    object.shapes[0].triangles[2] = objects[i][8];
    object.shapes[0].triangles[3] = objects[i][9];

    collision_object_pub_.publish(object);
    ROS_INFO("[exp] [%d] id: %s xyz: %0.3f %0.3f %0.3f dims: %0.3f %0.3f %0.3f colors: %0.3f %0.3f %0.3f %0.3f",int(i),object_ids[i].c_str(),objects[i][0],objects[i][1],objects[i][2],objects[i][3],objects[i][4],objects[i][5],objects[i][6],objects[i][7],objects[i][8],objects[i][9]);
    usleep(100000);
  }
  return true;
}

void SBPLFullBodyExperiment::removeCollisionObject(std::string id)
{
  mapping_msgs::CollisionObject object;
  object.id = id;
  object.operation.operation = mapping_msgs::CollisionObjectOperation::REMOVE;
  object.header.stamp = ros::Time::now();

  collision_object_pub_.publish(object);
  ROS_INFO("[exp] Removed %s object.", id.c_str());
}

bool SBPLFullBodyExperiment::attachObject(std::string object_file, geometry_msgs::Pose rarm_object_pose)
{
  char sTemp[1024];
  double radius=0;
  float temp[4];
  geometry_msgs::Point point;
  mapping_msgs::AttachedCollisionObject att_object, sbpl_att_object;
 
  att_object.link_name = "r_gripper_r_finger_tip_link";
  att_object.touch_links.push_back("r_gripper_palm_link");
  att_object.touch_links.push_back("r_gripper_r_finger_link");
  att_object.touch_links.push_back("r_gripper_l_finger_link");
  att_object.touch_links.push_back("r_gripper_l_finger_tip_link");
  att_object.touch_links.push_back("l_gripper_palm_link");
  att_object.touch_links.push_back("l_gripper_r_finger_link");
  att_object.touch_links.push_back("l_gripper_l_finger_link");
  att_object.touch_links.push_back("l_gripper_l_finger_tip_link");
  att_object.touch_links.push_back("l_gripper_r_finger_tip_link");
  att_object.object.header.frame_id = "r_wrist_roll_link";
  att_object.object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
  att_object.object.header.stamp = ros::Time::now();
  att_object.object.shapes.resize(1);
  att_object.object.shapes[0].type = geometric_shapes_msgs::Shape::BOX;
  att_object.object.poses.resize(1);
  att_object.object.poses[0] = rarm_object_pose;

  FILE* fid = fopen(object_file.c_str(), "r");
  if(fid == NULL)
  {
    ROS_ERROR("[exp] Failed to open object file. (%s)", object_file.c_str());
    return false;
  }

  // object name
  if(fscanf(fid,"%s",sTemp) < 1)
    ROS_WARN("Parsed string has length < 1. (%s)", sTemp);
  att_object.object.id = sTemp;

  // dims
  if(fscanf(fid,"%s",sTemp) < 1)
    ROS_WARN("Parsed string has length < 1. (%s)", sTemp); 
  if(strcmp(sTemp, "dims:") == 0)
  {
    if(fscanf(fid,"%f %f %f",&(temp[0]),&(temp[1]),&(temp[2])) < 1)
      ROS_WARN("Failed to parse dims.");
    att_object.object.shapes[0].dimensions.resize(3,0);
    att_object.object.shapes[0].dimensions[0] = temp[0];
    att_object.object.shapes[0].dimensions[1] = temp[1];
    att_object.object.shapes[0].dimensions[2] = temp[2];  
  }

  att_object.object.poses[0].position.x += att_object.object.shapes[0].dimensions[0];
  att_object.object.poses[0].position.y = -rarm_object_pose.position.y;

  ROS_DEBUG("TRANSLATED OF POSE: %f %f %f ", att_object.object.poses[0].position.x, att_object.object.poses[0].position.y, att_object.object.poses[0].position.z);

  sbpl_att_object = att_object;
  sbpl_att_object.object.shapes[0].dimensions.clear();

  //the dimensions of the object are secretly stored in the pose :)
  sbpl_att_object.object.poses[0].position.x = att_object.object.shapes[0].dimensions[0];
  sbpl_att_object.object.poses[0].position.y = att_object.object.shapes[0].dimensions[1];
  sbpl_att_object.object.poses[0].position.z = att_object.object.shapes[0].dimensions[2];

  bool first_point = true;
  while(!feof(fid))
  {
    if(sTemp[0] == '#') //comments
      fgets(sTemp, 1024,fid);
    else
    {
      if(fscanf(fid,"%f %f %f %f", &(temp[0]),&(temp[1]),&(temp[2]),&(temp[3])) < 1)
      {
        ROS_WARN("[exp] Read an incorrect line. May not be a problem.");
        continue;
      }
      point.x = temp[0];
      point.y = temp[1];
      point.z = temp[2];
      radius = temp[3];
      sbpl_att_object.object.shapes[0].vertices.push_back(point);
      sbpl_att_object.object.shapes[0].dimensions.push_back(radius);

      //set the Z inflation
      //HACK: This just sets above, assuming some kind of tray for now
      if(first_point)
      {
        sbpl_att_object.object.poses[0].orientation.x = temp[2]+temp[3]/2.0;
        first_point = false;
      }
    }
  }
  ROS_INFO("[exp] Parsed an object file with object '%s' with %d points", sbpl_att_object.object.id.c_str(),int(sbpl_att_object.object.shapes[0].vertices.size()));

  ROS_INFO("[exp] SBPL Attached Object:");
  for(size_t i = 0; i < sbpl_att_object.object.shapes[0].vertices.size(); ++i)
    ROS_INFO("[%s] [%d] xyz:  %0.3f %0.3f %0.3f radius: %0.3f",sbpl_att_object.object.id.c_str(),int(i),sbpl_att_object.object.shapes[0].vertices[i].x,sbpl_att_object.object.shapes[0].vertices[i].y,sbpl_att_object.object.shapes[0].vertices[i].z,sbpl_att_object.object.shapes[0].dimensions[i]);

  ROS_INFO("[exp] Publishing the attached collision object.");
  attached_object_pub_.publish(att_object);
  ROS_INFO("[exp] Publishing the sbpl attached collision object.");
  sbpl_attached_object_pub_.publish(sbpl_att_object);

  return true;
}

bool SBPLFullBodyExperiment::requestPlan(RobotPose &start_state, RobotPose goal_state, std::vector<std::vector<double> > &traj)
{
  sbpl_two_arm_planner_node::GetTwoArmPlan::Request req;
  sbpl_two_arm_planner_node::GetTwoArmPlan::Response res;
    
  req.goal.header.frame_id = "map";
  req.start.header.frame_id = "map";
  req.goal.header.stamp = ros::Time();
  req.start.header.stamp = ros::Time();
  req.rarm_start = start_state.rangles;
  req.larm_start = start_state.langles;
  req.body_start.push_back(start_state.body.x);
  req.body_start.push_back(start_state.body.y);
  req.body_start.push_back(start_state.body.z);
  req.body_start.push_back(start_state.body.theta);

  req.rarm_object.pose = rarm_object_pose_;
  req.larm_object.pose = larm_object_pose_;
  req.absolute_xyzrpy_tolerance = goal_tolerance_;
  req.goal.pose.position.x = goal_state.body.x;
  req.goal.pose.position.y = goal_state.body.y;
  req.goal.pose.position.z = 0.0;

  btQuaternion btgoal;
  btgoal.setRPY(0.0,0.0,goal_state.body.theta);
  tf::quaternionTFToMsg(btgoal,req.goal.pose.orientation);
  std::string t;

  if(planner_client_.call(req, res))
  {
    ROS_INFO("[exp] Planner returned.");

    if(res.stats.size() > 18)
    {
      if(res.stats.size() == res.stats_field_names.size())
      {
        ROS_INFO("\n%50s","-- Planning Statistics --");
        for(size_t i = 0; i < res.stats.size()-18; ++i)
          ROS_INFO("%44s: %0.2f", res.stats_field_names[i].c_str(),res.stats[i]);
        ROS_INFO("\n");

        printf("\n");
        for(size_t i = 0; i < res.stats.size()-18; ++i)
          printf("%0.2f, ", res.stats[i]);
        printf("\n");

        FILE* file = fopen("/tmp/planner_stats.csv", "a");
        if(file != NULL)
        {
          time_t clock;
          time(&clock);
          t = ctime(&clock);
          fprintf(file,"%s, ", t.c_str());
          for(size_t i = 0; i < res.stats.size()-18; ++i)
            fprintf(file, "%0.2f, ", res.stats[i]);
          fprintf(file,"\n");
          fclose(file);
        }
      }
    }
    
    /*
    std::string filename = trajectory_files_path_ + "/" + name + ".csv";
    ROS_INFO("[exp] Saving the trajectory to %s.", filename.c_str());
    if(!printTrajectoryToFile(filename,res.trajectory, res.body_trajectory))
    {
      ROS_ERROR("[exp] Planner succeeded but failed to write the trajectory to file.");
      return false;
    }
    */

    if(res.trajectory.points.empty())
    {
      ROS_ERROR("[exp] Planner failed.");
      return false;
    }
  }
  else
  {
    ROS_ERROR("[exp] Planning service failed to respond. Exiting.");
    return false;
  }

  current_pose_.rangles.resize(7);
  current_pose_.langles.resize(7);
  for(size_t i = 0; i < 7; ++i)
  {
    current_pose_.rangles[i] = res.trajectory.points.back().positions[i];
    current_pose_.langles[i] = res.trajectory.points.back().positions[i+7];
  }
  current_pose_.body.x = res.body_trajectory.points.back().positions[0];
  current_pose_.body.y = res.body_trajectory.points.back().positions[1];
  current_pose_.body.z = res.body_trajectory.points.back().positions[2];
  current_pose_.body.theta = res.body_trajectory.points.back().positions[3];
  
  return true;
}

bool SBPLFullBodyExperiment::prepareEnvironment()
{
  if(known_objects_filename_.compare("") != 0)
  {
    std::vector<double> offset(3,0.0);
    if(!addCollisionObjects(known_objects_filename_, offset))
    {
      ROS_ERROR("[exp] Failed to add collision objects to environment.");
      return false;
    }
  }
  return true;
}

void SBPLFullBodyExperiment::sendGoal(const geometry_msgs::PoseStamped::ConstPtr& goal){
  RobotPose goal_state;
  goal_state.body.x = goal->pose.position.x;
  goal_state.body.y = goal->pose.position.y;
  goal_state.body.z = start_pose_.body.z;
  goal_state.body.theta = 2*atan2(goal->pose.orientation.z, goal->pose.orientation.w);
  std::vector<std::vector<double> > traj;
  if(!requestPlan(start_pose_, goal_state, traj))
    ROS_ERROR("[exp] failed to plan.");
  else
    ROS_INFO("[exp] It's a planning miracle!");
  start_pose_.body = goal_state.body;
}

void SBPLFullBodyExperiment::startCompleteExperimentFile(){
  FILE* fout = fopen("completeExperiment.yaml","w");
  fprintf(fout,"goal_tolerance:\n  xyz: 0.02 0.02 0.02\n  yaw: 0.1\n\n");
  fprintf(fout,"object_pose_in_gripper:\n  right:\n    xyz: -0.20 -0.1 0.0\n    rpy: 0.0 0.0 0.0\n  left:\n    xyz: -0.20 0.1 0.0\n    rpy: 0.0 0.0 0.0\n\n");
  fprintf(fout,"use_current_pose_as_start_state: false\n");
  fprintf(fout,"start_state:\n  right: 0.0 0.018 0.00 -0.43 0.00 -0.00 0.0\n  left: 0.0 0.018 0.00 -0.43 0.00 -0.00 0.0\n  base: 1.9 0.6 1.57\n  spine: 0.0\n\n");
  fprintf(fout,"experiments:\n\n");
  fclose(fout);
}

void SBPLFullBodyExperiment::writeCompleteExperimentFile(Experiment e, RobotPose start){
  FILE* fout = fopen("completeExperiment.yaml","a");
  fprintf(fout,"  - name: %s\n",e.name.c_str());
  fprintf(fout,"    goal: %s\n",e.goal.c_str());
  if(e.pre_action==0)
    fprintf(fout,"    pre_action: nothing\n");
  else if(e.pre_action==1)
    fprintf(fout,"    pre_action: attach\n");
  else if(e.pre_action==2)
    fprintf(fout,"    pre_action: detach\n");
  if(e.post_action==0)
    fprintf(fout,"    post_action: nothing\n");
  else if(e.post_action==1)
    fprintf(fout,"    post_action: attach\n");
  else if(e.post_action==2)
    fprintf(fout,"    post_action: detach\n");
  fprintf(fout,"    sound_bite: \"%s\"\n",e.sound_bite.c_str());
  fprintf(fout,"    start:\n");
  fprintf(fout,"      right: %f %f %f %f %f %f %f\n",start.rangles[0],start.rangles[1],start.rangles[2],start.rangles[3],start.rangles[4],start.rangles[5],start.rangles[6]);
  fprintf(fout,"      left: %f %f %f %f %f %f %f\n",start.langles[0],start.langles[1],start.langles[2],start.langles[3],start.langles[4],start.langles[5],start.langles[6]);
  fprintf(fout,"      base: %f %f %f\n",start.body.x,start.body.y,start.body.theta);
  fprintf(fout,"      torso: %f\n",start.body.z);
  fclose(fout);
}

void SBPLFullBodyExperiment::visualizeStartPose()
{
  visualizeRobotPose(start_pose_, "start", 0);
}

void SBPLFullBodyExperiment::visualizeRobotPose(RobotPose &pose, std::string name, int id)
{
  pviz_.visualizeRobot(pose.rangles, pose.langles, pose.body, 120, name, id);
}

void SBPLFullBodyExperiment::visualizeLocations()
{
  std::vector<std::vector<double> > poses;
  poses.resize(std::distance(loc_map_.begin(),loc_map_.end()));
  for(std::map<std::string,std::vector<double> >::iterator iter = loc_map_.begin(); iter != loc_map_.end(); ++iter)
  {
    int i = std::distance(loc_map_.begin(), iter);
    poses[i].resize(6,0.0);
    poses[i][0] = iter->second.at(0);
    poses[i][1] = iter->second.at(1);
    poses[i][2] = iter->second.at(2);
    poses[i][5] = iter->second.at(3);
  }

  ROS_INFO("[exp] Visualizing %d locations.", int(poses.size()));
  pviz_.visualizePoses(poses);
}

void SBPLFullBodyExperiment::printLocations()
{
  if(loc_map_.begin() == loc_map_.end())
  {
    ROS_ERROR("[exp] No locations found.");
    return;
  }
  for(std::map<std::string,std::vector<double> >::const_iterator iter = loc_map_.begin(); iter != loc_map_.end(); ++iter)
  {
    ROS_INFO("name: %s", iter->first.c_str());
    ROS_INFO("  x: % 0.3f  y: % 0.3f  z: % 0.3f theta: % 0.3f", iter->second.at(0), iter->second.at(1), iter->second.at(2), iter->second.at(3));
  }
}

void SBPLFullBodyExperiment::printExperiments()
{
  if(exp_map_.begin() == exp_map_.end())
  {
    ROS_ERROR("[exp] No experiments found.");
    return;
  }
  for(std::map<std::string,Experiment>::iterator iter = exp_map_.begin(); iter != exp_map_.end(); ++iter)
  {
    int p = std::distance(exp_map_.begin(), iter);
    ROS_INFO("------------------------------");
    ROS_INFO("[%d] name: %s", p, iter->second.name.c_str());
    ROS_INFO("[%d] goal: %s", p, iter->second.goal.c_str());
    ROS_INFO("[%d] pre_action: %s", p, action_list_[iter->second.pre_action].c_str());
    ROS_INFO("[%d] post_action: %s", p, action_list_[iter->second.post_action].c_str());
    ROS_INFO("[%d] sound_bite: %s", p, iter->second.sound_bite.c_str());
    if(!use_current_state_as_start_)
    {
      ROS_INFO("[%d] start:", p);
      ROS_INFO("[%d]   right: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", p, iter->second.start.rangles[0], iter->second.start.rangles[1], iter->second.start.rangles[2], iter->second.start.rangles[3], iter->second.start.rangles[4], iter->second.start.rangles[5], iter->second.start.rangles[6]);
      ROS_INFO("[%d]    left: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", p, iter->second.start.langles[0], iter->second.start.langles[1], iter->second.start.langles[2], iter->second.start.langles[3], iter->second.start.langles[4], iter->second.start.langles[5], iter->second.start.langles[6]);
      ROS_INFO("[%d]    base: % 0.3f % 0.3f % 0.3f", p, iter->second.start.body.x, iter->second.start.body.y, iter->second.start.body.theta);
      ROS_INFO("[%d]   torso: % 0.3f", p, iter->second.start.body.z);
    }
  }
}

void SBPLFullBodyExperiment::printRobotPose(RobotPose &pose, std::string name)
{
  ROS_INFO("[%s] right: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", name.c_str(), pose.rangles[0], pose.rangles[1], pose.rangles[2], pose.rangles[3], pose.rangles[4], pose.rangles[5], pose.rangles[6]);
  ROS_INFO("[%s]  left: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", name.c_str(), pose.langles[0], pose.langles[1], pose.langles[2], pose.langles[3], pose.langles[4], pose.langles[5], pose.langles[6]);
  ROS_INFO("[%s]  base: % 0.3f % 0.3f % 0.3f", name.c_str(), pose.body.x, pose.body.y, pose.body.theta);
  ROS_INFO("[%s] torso: % 0.3f", name.c_str(), pose.body.z);
}

void SBPLFullBodyExperiment::printParams()
{
  ROS_INFO("   [goal tolerance] x: % 0.3f  y: % 0.3f  z: % 0.3f  theta: % 0.3f", goal_tolerance_[0], goal_tolerance_[1], goal_tolerance_[2], goal_tolerance_[3]);
  ROS_INFO("[right object pose] x: % 0.3f  y: % 0.3f  z: % 0.3f  r: % 0.3f  p: % 0.3f  y: % 0.3f", rarm_object_offset_[0], rarm_object_offset_[1], rarm_object_offset_[2], rarm_object_offset_[3], rarm_object_offset_[4], rarm_object_offset_[5]);
  ROS_INFO(" [left object pose] x: % 0.3f  y: % 0.3f  z: % 0.3f  r: % 0.3f  p: % 0.3f  y: % 0.3f", larm_object_offset_[0], larm_object_offset_[1], larm_object_offset_[2], larm_object_offset_[3], larm_object_offset_[4], larm_object_offset_[5]);
  ROS_INFO("      [start state] right: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", start_pose_.rangles[0], start_pose_.rangles[1], start_pose_.rangles[2], start_pose_.rangles[3], start_pose_.rangles[4], start_pose_.rangles[5], start_pose_.rangles[6]);
  ROS_INFO("      [start state]  left: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", start_pose_.langles[0], start_pose_.langles[1], start_pose_.langles[2], start_pose_.langles[3], start_pose_.langles[4], start_pose_.langles[5], start_pose_.langles[6]);
  ROS_INFO("      [start state]  base: % 0.3f % 0.3f % 0.3f", start_pose_.body.x, start_pose_.body.y, start_pose_.body.theta);
  ROS_INFO("      [start state] torso: % 0.3f", start_pose_.body.z);

  printLocations();
  printExperiments();
}


bool SBPLFullBodyExperiment::printTrajectoryToFile(std::string filename, trajectory_msgs::JointTrajectory &traj, trajectory_msgs::JointTrajectory &btraj)
{
  if(traj.points.size() == 0 || btraj.points.size() == 0)
  {
    ROS_WARN("[exp] Arm or body trajectory is empty. Can't write to file.");
    return false;
  }
  if(traj.points.size() != btraj.points.size())
  {
    ROS_WARN("[exp] Arm trajectory and body trajectory don't have same length. Can't write to file.");
    return false;
  }

  FILE* file = fopen(filename.c_str(), "w");
  if(file == NULL)
  {
    ROS_ERROR("[exp] Failed to open the file for writing: %s", filename.c_str());
    return false;
  }

  for(size_t i = 0; i < btraj.points.size(); ++i)
  {
    //for(size_t j = 0; j < btraj.points[i].positions.size(); ++j)
    //  fprintf(file, "%0.4f, ", btraj.points[i].positions[j]);
    
    fprintf(file, "%0.4f, ", btraj.points[i].positions[0]);
    fprintf(file, "%0.4f, ", btraj.points[i].positions[1]);
    fprintf(file, "%0.4f, ", btraj.points[i].positions[3]);
    fprintf(file, "%0.4f, ", btraj.points[i].positions[2]);

    for(size_t j = 0; j < traj.points[i].positions.size(); ++j)
      fprintf(file, "%0.4f, ", traj.points[i].positions[j]);
    fprintf(file,"\n");
  }

  fclose(file);

  ROS_INFO("[exp] Wrote the trajectory to file: %s.",filename.c_str());
  return true;
}

bool SBPLFullBodyExperiment::parseTrajectoryFile(std::string filename, trajectory_msgs::JointTrajectory &traj, trajectory_msgs::JointTrajectory &btraj)
{
  std::ifstream input_file(filename.c_str());
  if(!input_file.good())
  {
    printf("[exp] Unable to open '%s' for reading.\n",filename.c_str());
    return false;
  }

  int row(0), col(0), num_cols(18);
  char line[256];
  input_file.seekg(0);

  std::vector<std::vector<double> > raw_data;

  row = -1;
  raw_data.clear();

  while (input_file.getline(line, 256, ','))
  {
    raw_data.resize(raw_data.size()+1);
    raw_data[++row].resize(num_cols);
    raw_data[row][0] = atof(line);

    for(col = 1; col < num_cols; col++)
    {
      input_file.getline(line, 256, ',');
      raw_data[row][col] = atof(line);
    }
  }

  if(raw_data.size() == 0)
  {
    ROS_WARN("[exp] %s is empty. No trajectory found.",filename.c_str());
    return true;
  }

  std::vector<double> zero_line(num_cols,0);
  btraj.points.resize(raw_data.size()-1);
  traj.points.resize(raw_data.size()-1);
  for(int i = 0; i < int(traj.points.size()); i++)
  {
    if(raw_data[i] != zero_line)
    {
      btraj.points[i].positions.resize(4,0);
      traj.points[i].positions.resize(14,0);
      for(size_t j = 0; j < 4; ++j)
        btraj.points[i].positions[j] = raw_data[i][j];
      for(size_t j = 0; j < 14; ++j)
        traj.points[i].positions[j] = raw_data[i][j+4];
    }
  }
  std::string traj_name;
  if(filename.find_last_of("/") != std::string::npos)
    traj_name = filename.substr(filename.find_last_of("/")+1);

  printTrajectory(traj_name, traj, btraj);
  return true;
}

void SBPLFullBodyExperiment::printTrajectory(std::string name, trajectory_msgs::JointTrajectory &traj, trajectory_msgs::JointTrajectory &btraj)
{
  if(traj.points.size() == 0 || btraj.points.size() == 0)
  {
    ROS_WARN("[exp] Arm or body trajectory is empty. Not displaying to screen.");
    return;
  }
  if(traj.points.size() != btraj.points.size())
  {
    ROS_WARN("[exp] Arm trajectory and body trajectory don't have same length. Not displaying to screen.");
    return;
  }
  for(size_t i = 0; i < btraj.points.size(); ++i)
  {
    if(traj.points[i].positions.size() == 14)
    {
      ROS_INFO("[%s][%d] right: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", name.c_str(), int(i), traj.points[i].positions[0], traj.points[i].positions[1], traj.points[i].positions[2], traj.points[i].positions[3], traj.points[i].positions[4], traj.points[i].positions[5], traj.points[i].positions[6]);
      ROS_INFO("[%s][%d]  left: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", name.c_str(), int(i), traj.points[i].positions[7], traj.points[i].positions[8], traj.points[i].positions[9], traj.points[i].positions[10], traj.points[i].positions[11], traj.points[i].positions[12], traj.points[i].positions[13]);
    }
    else
      ROS_ERROR("[exp] WARNING! arms waypoint %d only has %d values.", int(i), traj.points[i].positions.size());

    if(btraj.points[i].positions.size() == 4)
    {
      ROS_INFO("[%s][%d]  base: % 0.3f % 0.3f % 0.3f", name.c_str(), int(i), btraj.points[i].positions[0], btraj.points[i].positions[1], btraj.points[i].positions[2]);
      ROS_INFO("[%s][%d] torso: % 0.3f", name.c_str(), int(i), btraj.points[i].positions[3]);
    }
    else
      ROS_ERROR("[exp] WARNING! base waypoint %d only has %d values.", int(i), traj.points[i].positions.size());
  }
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "sbpl_full_body_tests"); 
  SBPLFullBodyExperiment exp;

  if(!exp.getParams())
    return false;
  exp.printParams();

  sleep(3);
  ROS_INFO("------------ READY TO ROCK ------------");

  //exp.visualizeLocations();
  //exp.visualizeStartPose();

  if(!exp.prepareEnvironment())
    ROS_ERROR("Failed to prepare environment for testing.");

  sleep(2);
  //if(!exp.performAllExperiments())
    //ROS_ERROR("Experiments failed.");
    


  ros::spin();
  return 0;
}

