#include "arm_traj_splitter/SplitterNode.h"

#include <set>
#include <sstream>
#include <stdexcept>

#include <actionlib/client/simple_client_goal_state.h>
#include <control_msgs/FollowJointTrajectoryResult.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace arm_traj_splitter
{

namespace
{
std::vector<control_msgs::JointTolerance> selectTolerances(
    const std::vector<control_msgs::JointTolerance> &full,
    const std::set<std::string> &selected_joint_names)
{
  std::vector<control_msgs::JointTolerance> out;
  out.reserve(full.size());
  for (const auto &tol : full) {
    if (selected_joint_names.count(tol.name) > 0) {
      out.push_back(tol);
    }
  }
  return out;
}

bool validatePointFieldSize(const trajectory_msgs::JointTrajectoryPoint &pt,
                            const std::size_t joint_count,
                            std::string &error)
{
  auto check_size = [&](const std::vector<double> &field, const char *field_name) -> bool {
    if (!field.empty() && field.size() != joint_count) {
      std::ostringstream oss;
      oss << "Field '" << field_name << "' size mismatch: expected " << joint_count
          << ", got " << field.size();
      error = oss.str();
      return false;
    }
    return true;
  };

  return check_size(pt.positions, "positions") && check_size(pt.velocities, "velocities") &&
         check_size(pt.accelerations, "accelerations") && check_size(pt.effort, "effort");
}

void extractFieldByIndex(const std::vector<double> &src, const std::vector<std::size_t> &idxs,
                         std::vector<double> &dst)
{
  dst.clear();
  if (src.empty()) {
    return;
  }
  dst.reserve(idxs.size());
  for (const auto idx : idxs) {
    dst.push_back(src[idx]);
  }
}

}  // namespace

SplitterNode::SplitterNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  : nh_(nh), pnh_(pnh)
{
  pnh_.param("wait_server_timeout_sec", wait_server_timeout_sec_, wait_server_timeout_sec_);
  pnh_.param("poll_rate_hz", poll_rate_hz_, poll_rate_hz_);

  if (!loadBackends()) {
    ROS_FATAL("[arm_traj_splitter] Failed to load backends from parameters.");
    throw std::runtime_error("Failed to load backends.");
  }

  for (auto &backend : backends_) {
    backend.client.reset(new FjtClient(backend.action_ns, true));
    const bool ready = backend.client->waitForServer(ros::Duration(wait_server_timeout_sec_));
    if (!ready) {
      ROS_WARN("[arm_traj_splitter] Timeout waiting action server: %s", backend.action_ns.c_str());
    } else {
      ROS_INFO("[arm_traj_splitter] Connected action server: %s", backend.action_ns.c_str());
    }
  }

  std::string controller_name = "arm_controller";
  pnh_.param<std::string>("arm_controller_name", controller_name, controller_name);
  const std::string action_name = "/" + controller_name + "/follow_joint_trajectory";

  server_.reset(new FjtServer(nh_, action_name, boost::bind(&SplitterNode::executeGoal, this, _1), false));
  server_->start();
  ROS_INFO("[arm_traj_splitter] Action server started: %s", action_name.c_str());
}

bool SplitterNode::loadBackends()
{
  XmlRpc::XmlRpcValue backend_list;
  if (!pnh_.getParam("backend_controllers", backend_list)) {
    ROS_ERROR("[arm_traj_splitter] Missing required param: backend_controllers");
    return false;
  }
  if (backend_list.getType() != XmlRpc::XmlRpcValue::TypeArray || backend_list.size() == 0) {
    ROS_ERROR("[arm_traj_splitter] backend_controllers must be a non-empty list");
    return false;
  }

  backends_.clear();
  joint_to_backend_.clear();

  for (int i = 0; i < backend_list.size(); ++i) {
    if (backend_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_ERROR("[arm_traj_splitter] backend_controllers[%d] must be a struct", i);
      return false;
    }

    Backend backend;
    auto &item = backend_list[i];

    if (!item.hasMember("name") || !item.hasMember("action_ns") || !item.hasMember("joints")) {
      ROS_ERROR("[arm_traj_splitter] backend_controllers[%d] missing required fields", i);
      return false;
    }

    backend.name = static_cast<std::string>(item["name"]);
    backend.action_ns = static_cast<std::string>(item["action_ns"]);

    auto &joints = item["joints"];
    if (joints.getType() != XmlRpc::XmlRpcValue::TypeArray || joints.size() == 0) {
      ROS_ERROR("[arm_traj_splitter] backend '%s' joints must be non-empty list", backend.name.c_str());
      return false;
    }

    std::set<std::string> unique_joints;
    for (int j = 0; j < joints.size(); ++j) {
      const std::string joint = static_cast<std::string>(joints[j]);
      if (!unique_joints.insert(joint).second) {
        ROS_ERROR("[arm_traj_splitter] Duplicate joint '%s' in backend '%s'", joint.c_str(),
                  backend.name.c_str());
        return false;
      }
      if (joint_to_backend_.count(joint) > 0) {
        ROS_ERROR("[arm_traj_splitter] Joint '%s' assigned to multiple backends", joint.c_str());
        return false;
      }
      joint_to_backend_[joint] = static_cast<std::size_t>(backends_.size());
      backend.joints.push_back(joint);
    }

    backends_.push_back(std::move(backend));
  }

  return true;
}

void SplitterNode::executeGoal(const FjtGoalConstPtr &goal)
{
  std::vector<FjtGoal> part_goals;
  std::string error;
  if (!splitGoal(*goal, part_goals, error)) {
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    result.error_string = error;
    server_->setAborted(result, error);
    return;
  }

  for (std::size_t i = 0; i < backends_.size(); ++i) {
    backends_[i].client->sendGoal(part_goals[i]);
  }

  ros::Rate poll(std::max(1.0, poll_rate_hz_));
  while (ros::ok()) {
    if (server_->isPreemptRequested()) {
      for (auto &backend : backends_) {
        backend.client->cancelGoal();
      }
      server_->setPreempted();
      return;
    }

    bool all_done = true;
    bool any_failed = false;
    std::string failed_backend;
    actionlib::SimpleClientGoalState failed_state(actionlib::SimpleClientGoalState::PENDING);

    for (auto &backend : backends_) {
      const auto state = backend.client->getState();
      if (!state.isDone()) {
        all_done = false;
        continue;
      }
      if (state != actionlib::SimpleClientGoalState::SUCCEEDED) {
        any_failed = true;
        failed_backend = backend.name;
        failed_state = state;
      }
    }

    if (any_failed) {
      for (auto &backend : backends_) {
        if (!backend.client->getState().isDone()) {
          backend.client->cancelGoal();
        }
      }
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      result.error_string = "Backend failed: " + failed_backend + " state=" + failed_state.toString();
      server_->setAborted(result, result.error_string);
      return;
    }

    if (all_done) {
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
      server_->setSucceeded(result);
      return;
    }

    poll.sleep();
  }

  control_msgs::FollowJointTrajectoryResult result;
  result.error_code = control_msgs::FollowJointTrajectoryResult::OLD_HEADER_TIMESTAMP;
  result.error_string = "ROS shutdown while executing goal.";
  server_->setAborted(result, result.error_string);
}

bool SplitterNode::splitGoal(const FjtGoal &full_goal, std::vector<FjtGoal> &part_goals, std::string &error) const
{
  std::vector<trajectory_msgs::JointTrajectory> part_trajectories;
  if (!splitTrajectory(full_goal.trajectory, part_trajectories, error)) {
    return false;
  }

  part_goals.assign(backends_.size(), FjtGoal());

  std::unordered_map<std::string, std::size_t> full_joint_index;
  full_joint_index.reserve(full_goal.trajectory.joint_names.size());
  for (std::size_t full_idx = 0; full_idx < full_goal.trajectory.joint_names.size(); ++full_idx) {
    full_joint_index[full_goal.trajectory.joint_names[full_idx]] = full_idx;
  }

  std::vector<std::vector<std::size_t>> index_maps(backends_.size());
  for (std::size_t b = 0; b < backends_.size(); ++b) {
    for (const auto &joint : part_trajectories[b].joint_names) {
      const auto it = full_joint_index.find(joint);
      if (it == full_joint_index.end()) {
        error = "Internal error: joint missing in source trajectory: " + joint;
        return false;
      }
      index_maps[b].push_back(it->second);
    }
  }

  for (std::size_t b = 0; b < backends_.size(); ++b) {
    part_goals[b].trajectory = part_trajectories[b];
    part_goals[b].goal_time_tolerance = full_goal.goal_time_tolerance;
  }

  splitTolerances(full_goal, index_maps, part_goals);
  return true;
}

bool SplitterNode::splitTrajectory(const trajectory_msgs::JointTrajectory &full,
                                   std::vector<trajectory_msgs::JointTrajectory> &parts,
                                   std::string &error) const
{
  if (full.joint_names.empty()) {
    error = "Trajectory has empty joint_names.";
    return false;
  }

  parts.assign(backends_.size(), trajectory_msgs::JointTrajectory());
  std::vector<std::vector<std::size_t>> index_maps(backends_.size());

  std::set<std::string> seen_joint;
  for (std::size_t j = 0; j < full.joint_names.size(); ++j) {
    const auto &joint = full.joint_names[j];
    if (!seen_joint.insert(joint).second) {
      error = "Duplicate joint in trajectory: " + joint;
      return false;
    }

    auto it = joint_to_backend_.find(joint);
    if (it == joint_to_backend_.end()) {
      error = "Joint not assigned in splitter config: " + joint;
      return false;
    }

    const std::size_t backend_idx = it->second;
    parts[backend_idx].joint_names.push_back(joint);
    parts[backend_idx].header = full.header;
    index_maps[backend_idx].push_back(j);
  }

  for (std::size_t b = 0; b < backends_.size(); ++b) {
    if (parts[b].joint_names.empty()) {
      error = "Trajectory missing joints for backend: " + backends_[b].name;
      return false;
    }
  }

  for (const auto &pt : full.points) {
    if (!validatePointFieldSize(pt, full.joint_names.size(), error)) {
      return false;
    }

    for (std::size_t b = 0; b < backends_.size(); ++b) {
      trajectory_msgs::JointTrajectoryPoint out;
      out.time_from_start = pt.time_from_start;

      extractFieldByIndex(pt.positions, index_maps[b], out.positions);
      extractFieldByIndex(pt.velocities, index_maps[b], out.velocities);
      extractFieldByIndex(pt.accelerations, index_maps[b], out.accelerations);
      extractFieldByIndex(pt.effort, index_maps[b], out.effort);

      parts[b].points.push_back(std::move(out));
    }
  }

  return true;
}

void SplitterNode::splitTolerances(const FjtGoal &full_goal,
                                   const std::vector<std::vector<std::size_t>> &index_maps,
                                   std::vector<FjtGoal> &part_goals) const
{
  for (std::size_t b = 0; b < backends_.size(); ++b) {
    const std::set<std::string> joints(backends_[b].joints.begin(), backends_[b].joints.end());
    auto split_one = [&](const std::vector<control_msgs::JointTolerance> &full_tolerances,
                         const char *tol_type) -> std::vector<control_msgs::JointTolerance> {
      std::vector<control_msgs::JointTolerance> out = selectTolerances(full_tolerances, joints);
      bool has_unnamed = false;
      for (const auto &tol : full_tolerances) {
        if (tol.name.empty()) {
          has_unnamed = true;
          break;
        }
      }

      if (!has_unnamed) {
        return out;
      }

      if (full_tolerances.size() == full_goal.trajectory.joint_names.size()) {
        ROS_WARN("[arm_traj_splitter] Backend '%s' uses index fallback for unnamed %s entries.",
                 backends_[b].name.c_str(), tol_type);
        for (const auto full_idx : index_maps[b]) {
          auto tol = full_tolerances[full_idx];
          if (!tol.name.empty()) {
            continue;
          }
          tol.name = full_goal.trajectory.joint_names[full_idx];
          out.push_back(std::move(tol));
        }
      } else {
        ROS_WARN(
            "[arm_traj_splitter] Backend '%s' drops unnamed %s entries: count (%zu) does not "
            "match trajectory joints (%zu).",
            backends_[b].name.c_str(), tol_type, full_tolerances.size(),
            full_goal.trajectory.joint_names.size());
      }
      return out;
    };

    part_goals[b].path_tolerance = split_one(full_goal.path_tolerance, "path_tolerance");
    part_goals[b].goal_tolerance = split_one(full_goal.goal_tolerance, "goal_tolerance");
  }
}

}  // namespace arm_traj_splitter
