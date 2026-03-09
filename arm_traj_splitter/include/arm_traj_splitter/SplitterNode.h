#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace arm_traj_splitter
{

class SplitterNode
{
public:
  SplitterNode(ros::NodeHandle &nh, ros::NodeHandle &pnh);

private:
  using FjtAction = control_msgs::FollowJointTrajectoryAction;
  using FjtGoal = control_msgs::FollowJointTrajectoryGoal;
  using FjtGoalConstPtr = control_msgs::FollowJointTrajectoryGoalConstPtr;
  using FjtClient = actionlib::SimpleActionClient<FjtAction>;
  using FjtServer = actionlib::SimpleActionServer<FjtAction>;

  struct Backend
  {
    std::string name;
    std::string action_ns;
    std::vector<std::string> joints;
    std::unique_ptr<FjtClient> client;
  };

  void executeGoal(const FjtGoalConstPtr &goal);
  bool splitGoal(const FjtGoal &full_goal, std::vector<FjtGoal> &part_goals, std::string &error) const;
  bool splitTrajectory(const trajectory_msgs::JointTrajectory &full,
                       std::vector<trajectory_msgs::JointTrajectory> &parts,
                       std::string &error) const;
  void splitTolerances(const FjtGoal &full_goal,
                       const std::vector<std::vector<std::size_t>> &index_maps,
                       std::vector<FjtGoal> &part_goals) const;

  bool loadBackends();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::unique_ptr<FjtServer> server_;
  std::vector<Backend> backends_;
  std::unordered_map<std::string, std::size_t> joint_to_backend_;

  double wait_server_timeout_sec_{5.0};
  double poll_rate_hz_{50.0};
};

}  // namespace arm_traj_splitter
