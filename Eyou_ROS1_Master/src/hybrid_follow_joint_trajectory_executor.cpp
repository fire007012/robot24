#include "Eyou_ROS1_Master/hybrid_follow_joint_trajectory_executor.hpp"

#include <algorithm>
#include <chrono>

namespace eyou_ros1_master {

namespace {

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

bool BuildGoalToConfigIndices(const std::vector<std::string>& goal_joint_names,
                              const std::vector<std::string>& config_joint_names,
                              std::vector<std::size_t>* goal_to_config_indices,
                              std::string* error) {
    if (goal_to_config_indices == nullptr) {
        SetError(error, "goal_to_config_indices is null");
        return false;
    }

    goal_to_config_indices->assign(goal_joint_names.size(), 0);
    for (std::size_t goal_index = 0; goal_index < goal_joint_names.size();
         ++goal_index) {
        const auto it =
            std::find(config_joint_names.begin(), config_joint_names.end(),
                      goal_joint_names[goal_index]);
        if (it == config_joint_names.end()) {
            SetError(error, "goal contains unknown joint: " + goal_joint_names[goal_index]);
            return false;
        }
        (*goal_to_config_indices)[goal_index] =
            static_cast<std::size_t>(std::distance(config_joint_names.begin(), it));
    }
    return true;
}

double StateValueOrZero(const std::vector<double>& values, std::size_t index) {
    return index < values.size() ? values[index] : 0.0;
}

}  // namespace

HybridFollowJointTrajectoryExecutor::HybridFollowJointTrajectoryExecutor(
    ros::NodeHandle* pnh,
    hardware_interface::RobotHW* hw,
    std::mutex* loop_mtx,
    HybridJointTargetExecutor* target_executor,
    Config config)
    : hw_raw_(hw),
      loop_mtx_(loop_mtx),
      target_executor_(target_executor),
      config_(std::move(config)) {
    if (target_executor_ == nullptr || !target_executor_->valid()) {
        config_valid_ = false;
        config_error_ = "target executor is null or invalid";
        return;
    }

    if (hw_raw_ == nullptr) {
        config_valid_ = false;
        config_error_ = "RobotHW pointer is null";
        return;
    }

    auto* state_iface = hw_raw_->get<hardware_interface::JointStateInterface>();
    if (state_iface == nullptr) {
        config_valid_ = false;
        config_error_ = "RobotHW missing JointStateInterface";
        return;
    }

    state_handles_.reserve(config_.joint_names.size());
    for (const auto& name : config_.joint_names) {
        try {
            state_handles_.push_back(state_iface->getHandle(name));
        } catch (const hardware_interface::HardwareInterfaceException& e) {
            config_valid_ = false;
            config_error_ = "failed to get state handle for joint '" + name +
                            "': " + e.what();
            state_handles_.clear();
            return;
        }
    }

    config_valid_ = true;
    if (pnh == nullptr) {
        return;
    }

    server_ = std::make_unique<Server>(
        *pnh, config_.action_ns,
        [this](const GoalConstPtr& goal) { ExecuteGoal(goal); }, false);
    server_->start();
}

bool HybridFollowJointTrajectoryExecutor::ValidateGoal(
    const control_msgs::FollowJointTrajectoryGoal& goal,
    const std::vector<std::string>& joint_names, std::string* error) {
    return canopen_hw::IpFollowJointTrajectoryExecutor::ValidateGoal(goal,
                                                                     joint_names,
                                                                     error);
}

HybridFollowJointTrajectoryExecutor::State
HybridFollowJointTrajectoryExecutor::ReadActualState() const {
    State actual;
    actual.positions.resize(config_.joint_names.size(), 0.0);
    actual.velocities.resize(config_.joint_names.size(), 0.0);
    actual.accelerations.resize(config_.joint_names.size(), 0.0);
    for (std::size_t i = 0; i < config_.joint_names.size(); ++i) {
        actual.positions[i] = state_handles_[i].getPosition();
        actual.velocities[i] = state_handles_[i].getVelocity();
    }
    return actual;
}

bool HybridFollowJointTrajectoryExecutor::buildTargetForWaypoint(
    const control_msgs::FollowJointTrajectoryGoal& goal,
    std::size_t waypoint_index,
    HybridJointTargetExecutor::Target* target,
    std::string* error) const {
    if (target == nullptr) {
        SetError(error, "target output is null");
        return false;
    }
    if (waypoint_index >= goal.trajectory.points.size()) {
        SetError(error, "waypoint index out of range");
        return false;
    }

    std::vector<std::size_t> goal_to_config_indices;
    if (!BuildGoalToConfigIndices(goal.trajectory.joint_names, config_.joint_names,
                                  &goal_to_config_indices, error)) {
        return false;
    }

    const auto& point = goal.trajectory.points[waypoint_index];
    HybridJointTargetExecutor::State state;
    state.positions.resize(config_.joint_names.size(), 0.0);
    state.velocities.resize(config_.joint_names.size(), 0.0);
    state.accelerations.resize(config_.joint_names.size(), 0.0);

    for (std::size_t goal_index = 0; goal_index < goal_to_config_indices.size();
         ++goal_index) {
        const std::size_t axis_index = goal_to_config_indices[goal_index];
        state.positions[axis_index] = point.positions[goal_index];
        state.velocities[axis_index] = StateValueOrZero(point.velocities, goal_index);
        state.accelerations[axis_index] =
            StateValueOrZero(point.accelerations, goal_index);
    }

    target->state = std::move(state);
    const double previous_time =
        (waypoint_index == 0)
            ? 0.0
            : goal.trajectory.points[waypoint_index - 1].time_from_start.toSec();
    const double current_time = point.time_from_start.toSec();
    const double segment_duration = current_time - previous_time;
    if (segment_duration > 0.0) {
        target->minimum_duration_sec = segment_duration;
    } else {
        target->minimum_duration_sec.reset();
    }
    return true;
}

bool HybridFollowJointTrajectoryExecutor::waypointReached(
    const State& actual,
    const control_msgs::FollowJointTrajectoryGoal& goal,
    std::size_t waypoint_index) const {
    if (waypoint_index >= goal.trajectory.points.size()) {
        return false;
    }

    std::vector<std::size_t> goal_to_config_indices;
    std::string error;
    if (!BuildGoalToConfigIndices(goal.trajectory.joint_names, config_.joint_names,
                                  &goal_to_config_indices, &error)) {
        return false;
    }

    const auto& point = goal.trajectory.points[waypoint_index];
    for (std::size_t goal_index = 0; goal_index < goal_to_config_indices.size();
         ++goal_index) {
        const std::size_t axis_index = goal_to_config_indices[goal_index];
        if (std::abs(actual.positions[axis_index] - point.positions[goal_index]) >
            config_.goal_tolerances[axis_index]) {
            return false;
        }
    }
    return true;
}

bool HybridFollowJointTrajectoryExecutor::startGoal(
    const control_msgs::FollowJointTrajectoryGoal& goal,
    std::string* error) {
    if (!config_valid_) {
        SetError(error, config_error_);
        return false;
    }
    if (!ValidateGoal(goal, config_.joint_names, error)) {
        return false;
    }

    std::lock_guard<std::mutex> lock(exec_mtx_);
    active_goal_ = goal;
    waypoint_index_ = 0;
    segment_target_active_ = false;
    last_terminal_status_.reset();
    last_terminal_error_.clear();
    return true;
}

void HybridFollowJointTrajectoryExecutor::cancelGoal() {
    {
        std::lock_guard<std::mutex> lock(exec_mtx_);
        active_goal_.reset();
        waypoint_index_ = 0;
        segment_target_active_ = false;
        last_terminal_status_ = StepStatus::kIdle;
        last_terminal_error_.clear();
    }
    if (target_executor_ != nullptr) {
        target_executor_->clearTarget();
    }
    exec_cv_.notify_all();
}

bool HybridFollowJointTrajectoryExecutor::hasActiveGoal() const {
    std::lock_guard<std::mutex> lock(exec_mtx_);
    return active_goal_.has_value();
}

void HybridFollowJointTrajectoryExecutor::publishFeedback(const State& actual) const {
    if (!server_ || !server_->isActive()) {
        return;
    }

    std::lock_guard<std::mutex> lock(exec_mtx_);
    if (!active_goal_) {
        return;
    }
    if (waypoint_index_ >= active_goal_->trajectory.points.size()) {
        return;
    }

    const auto& point = active_goal_->trajectory.points[waypoint_index_];
    control_msgs::FollowJointTrajectoryFeedback feedback;
    feedback.header.stamp = ros::Time::now();
    feedback.joint_names = config_.joint_names;

    feedback.actual.positions = actual.positions;
    feedback.actual.velocities = actual.velocities;
    feedback.actual.accelerations = actual.accelerations;

    feedback.desired.positions.resize(config_.joint_names.size(), 0.0);
    feedback.desired.velocities.resize(config_.joint_names.size(), 0.0);
    feedback.desired.accelerations.resize(config_.joint_names.size(), 0.0);
    feedback.error.positions.resize(config_.joint_names.size(), 0.0);
    feedback.error.velocities.resize(config_.joint_names.size(), 0.0);
    feedback.error.accelerations.resize(config_.joint_names.size(), 0.0);

    std::vector<std::size_t> goal_to_config_indices;
    std::string error;
    if (!BuildGoalToConfigIndices(active_goal_->trajectory.joint_names, config_.joint_names,
                                  &goal_to_config_indices, &error)) {
        return;
    }

    for (std::size_t goal_index = 0; goal_index < goal_to_config_indices.size();
         ++goal_index) {
        const std::size_t axis_index = goal_to_config_indices[goal_index];
        feedback.desired.positions[axis_index] = point.positions[goal_index];
        feedback.desired.velocities[axis_index] =
            StateValueOrZero(point.velocities, goal_index);
        feedback.desired.accelerations[axis_index] =
            StateValueOrZero(point.accelerations, goal_index);
        feedback.error.positions[axis_index] =
            feedback.desired.positions[axis_index] - actual.positions[axis_index];
        feedback.error.velocities[axis_index] =
            feedback.desired.velocities[axis_index] - actual.velocities[axis_index];
        feedback.error.accelerations[axis_index] =
            feedback.desired.accelerations[axis_index] - actual.accelerations[axis_index];
    }
    feedback.desired.time_from_start = point.time_from_start;
    feedback.error.time_from_start = ros::Duration(0.0);

    server_->publishFeedback(feedback);
}

void HybridFollowJointTrajectoryExecutor::update(const ros::Time& now,
                                                 const ros::Duration& /*period*/) {
    if (!config_valid_) {
        return;
    }

    std::optional<control_msgs::FollowJointTrajectoryGoal> goal;
    std::size_t waypoint_index = 0;
    bool segment_target_active = false;
    {
        std::lock_guard<std::mutex> lock(exec_mtx_);
        goal = active_goal_;
        waypoint_index = waypoint_index_;
        segment_target_active = segment_target_active_;
    }
    if (!goal.has_value()) {
        return;
    }

    const State actual = ReadActualState();
    if (!segment_target_active) {
        HybridJointTargetExecutor::Target target;
        std::string target_error;
        if (!buildTargetForWaypoint(*goal, waypoint_index, &target, &target_error) ||
            !target_executor_->setTarget(target, &target_error)) {
            {
                std::lock_guard<std::mutex> lock(exec_mtx_);
                active_goal_.reset();
                waypoint_index_ = 0;
                segment_target_active_ = false;
                last_terminal_status_ = StepStatus::kError;
                last_terminal_error_ = target_error;
            }
            exec_cv_.notify_all();
            return;
        }
        {
            std::lock_guard<std::mutex> lock(exec_mtx_);
            segment_target_active_ = true;
        }
    }

    if (now.isValid() && (last_feedback_pub_time_.isZero() ||
                          (now - last_feedback_pub_time_) >= ros::Duration(0.05))) {
        publishFeedback(actual);
        last_feedback_pub_time_ = now;
    }

    if (!waypointReached(actual, *goal, waypoint_index)) {
        return;
    }

    const bool is_last_waypoint = (waypoint_index + 1 >= goal->trajectory.points.size());
    if (is_last_waypoint) {
        {
            std::lock_guard<std::mutex> lock(exec_mtx_);
            active_goal_.reset();
            waypoint_index_ = 0;
            segment_target_active_ = false;
            last_terminal_status_ = StepStatus::kFinished;
            last_terminal_error_.clear();
        }
        exec_cv_.notify_all();
        return;
    }

    {
        std::lock_guard<std::mutex> lock(exec_mtx_);
        ++waypoint_index_;
        segment_target_active_ = false;
    }
}

void HybridFollowJointTrajectoryExecutor::ExecuteGoal(const GoalConstPtr& goal_ptr) {
    if (goal_ptr == nullptr || server_ == nullptr) {
        return;
    }
    if (!config_valid_) {
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
        result.error_string = config_error_;
        server_->setAborted(result, result.error_string);
        return;
    }
    if (target_executor_ == nullptr || !target_executor_->valid()) {
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
        result.error_string = "target executor is unavailable";
        server_->setAborted(result, result.error_string);
        return;
    }

    std::string error;
    if (!startGoal(*goal_ptr, &error)) {
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
        result.error_string = error.empty() ? "failed to start goal" : error;
        server_->setAborted(result, result.error_string);
        return;
    }

    std::unique_lock<std::mutex> lock(exec_mtx_);
    while (!last_terminal_status_.has_value()) {
        if (server_->isPreemptRequested()) {
            lock.unlock();
            cancelGoal();

            control_msgs::FollowJointTrajectoryResult result;
            result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
            result.error_string = "goal preempted";
            server_->setPreempted(result, result.error_string);
            return;
        }
        exec_cv_.wait_for(lock, std::chrono::milliseconds(5));
    }

    control_msgs::FollowJointTrajectoryResult result;
    if (*last_terminal_status_ == StepStatus::kFinished) {
        result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
        result.error_string = "goal completed";
        server_->setSucceeded(result, result.error_string);
        return;
    }

    target_executor_->clearTarget();
    result.error_code =
        control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
    result.error_string =
        last_terminal_error_.empty() ? "goal execution failed"
                                     : last_terminal_error_;
    server_->setAborted(result, result.error_string);
}

}  // namespace eyou_ros1_master
