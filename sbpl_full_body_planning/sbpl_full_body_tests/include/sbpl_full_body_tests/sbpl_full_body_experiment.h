#include <iostream>
#include <ros/ros.h>
#include <fstream>
#include <vector>
#include <iterator>
#include <list>
#include <map>
#include <tf/tf.h>
#include <arm_navigation_msgs/AttachedCollisionObject.h>
#include <sbpl_3dnav_planner/GetTwoArmPlan.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sbpl_arm_planner/body_pose.h>
#include <pviz/pviz.h>

static const std::string PLANNING_SERVICE="/sbpl_planning/plan_path";
static const double DISTANCE_FROM_WRIST_TO_GRIPPER=0.14;


typedef struct
{
  std::vector<double> langles;
  std::vector<double> rangles;
  BodyPose body;
} RobotPose;

typedef struct
{
  int pre_action;  // 0:nothing, 1:attach, 2:detach
  int post_action; // 0:nothing, 1:attach, 2:detach

  std::string name;
  std::string goal;
  std::string sound_bite;
  RobotPose start;
} Experiment;

class SBPLFullBodyExperiment
{
  public:
    
    SBPLFullBodyExperiment();
    ~SBPLFullBodyExperiment(){};

    bool getParams();
    bool getLocations();
    bool getExperiments();
    bool getDemonstratedPaths();
    void printLocations();
    void printExperiments();
    void printParams();
    void printRobotPose(RobotPose &pose, std::string name);
    void startCompleteExperimentFile();
    void writeCompleteExperimentFile(Experiment e, RobotPose start);

    bool addCollisionObjects(std::string filename, std::vector<double> &offset);
    void removeCollisionObject(std::string id);
    bool attachObject(std::string object_file, geometry_msgs::Pose rarm_object_pose);

    bool requestPlan(RobotPose &start_state, RobotPose goal_state, std::vector<std::vector<double> > &traj);
    bool performAllExperiments();
    bool prepareEnvironment();

    void visualizeLocations();
    void visualizeRobotPose(RobotPose &pose, std::string name, int id);
    void visualizeStartPose();

    bool printTrajectoryToFile(std::string filename, trajectory_msgs::JointTrajectory &traj, trajectory_msgs::JointTrajectory &btraj);
    bool parseTrajectoryFile(std::string filename, trajectory_msgs::JointTrajectory &traj, trajectory_msgs::JointTrajectory &btraj);
    void printTrajectory(std::string name, trajectory_msgs::JointTrajectory &traj, trajectory_msgs::JointTrajectory &btraj);

    void sendGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
    ros::Publisher attached_object_pub_;
    ros::Publisher sbpl_attached_object_pub_;
    ros::Publisher collision_object_pub_;
    ros::Publisher demonstrated_path_pub_;
    ros::ServiceClient planner_client_;
    ros::Subscriber goal_sub_;

    PViz pviz_;

    std::map<std::string, Experiment> exp_map_;
    std::map<std::string, int> action_map_;
    std::map<std::string, std::vector<double> > loc_map_;
    std::vector<std::string> action_list_;
    RobotPose start_pose_; 
    RobotPose current_pose_;  // current pose of the robot

    std::vector<double> goal_tolerance_;
    std::vector<double> rarm_object_offset_;
    std::vector<double> larm_object_offset_;
    geometry_msgs::Pose rarm_object_pose_;
    geometry_msgs::Pose larm_object_pose_;

    std::string known_objects_filename_;

    bool use_current_state_as_start_;

    std::string trajectory_folder_name_;
    std::string trajectory_folder_path_;
    std::string trajectory_files_path_;
    std::string demonstrated_paths_folder_;
};

