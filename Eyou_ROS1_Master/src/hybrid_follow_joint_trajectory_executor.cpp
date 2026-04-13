#include "Eyou_ROS1_Master/hybrid_follow_joint_trajectory_executor.hpp"

#include <chrono>
#include <cmath>

namespace eyou_ros1_master {

namespace {

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

double executorCycleSec(double command_rate_hz) {
    return 1.0 / std::max(1.0, command_rate_hz);
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

bool HybridFollowJointTrajectoryExecutor::sampleActiveGoal(
    double time_from_start_sec,
    HybridTrajectorySample* sample,
    std::string* error) const {
    std::lock_guard<std::mutex> lock(exec_mtx_);
    if (!active_goal_) {
        SetError(error, "no active goal");
        return false;
    }
    return SampleTrajectoryStateAtTime(*active_goal_,
                                       active_goal_to_config_indices_,
                                       active_goal_start_state_,
                                       time_from_start_sec,
                                       sample,
                                       error);
}

bool HybridFollowJointTrajectoryExecutor::activeGoalReached(const State& actual) const {
    HybridTrajectorySample final_sample;
    std::string error;
    if (!sampleActiveGoal(active_goal_duration_sec_, &final_sample, &error)) {
        return false;
    }
    for (std::size_t axis_index = 0; axis_index < config_.goal_tolerances.size();
         ++axis_index) {
        if (std::abs(actual.positions[axis_index] -
                     final_sample.state.positions[axis_index]) >
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

    std::vector<std::size_t> goal_to_config_indices;
    if (!BuildGoalToConfigIndices(goal.trajectory.joint_names,
                                  config_.joint_names,
                                  &goal_to_config_indices,
                                  error)) {
        return false;
    }
    if (goal.trajectory.points.empty()) {
        SetError(error, "trajectory contains no points");
        return false;
    }

    const State start_state = ReadActualState();
    const double goal_duration_sec =
        goal.trajectory.points.back().time_from_start.toSec();
    if (!std::isfinite(goal_duration_sec) || goal_duration_sec < 0.0) {
        SetError(error, "trajectory final time_from_start must be finite and >= 0");
        return false;
    }

    std::lock_guard<std::mutex> lock(exec_mtx_);
    active_goal_ = goal;
    active_goal_to_config_indices_ = std::move(goal_to_config_indices);
    active_goal_start_state_ = start_state;
    active_goal_duration_sec_ = goal_duration_sec;
    active_goal_elapsed_sec_ = 0.0;
    last_terminal_status_.reset();
    last_terminal_error_.clear();
    last_feedback_pub_time_ = ros::Time(0);
    return true;
}

void HybridFollowJointTrajectoryExecutor::cancelGoal() {
    {
        std::lock_guard<std::mutex> lock(exec_mtx_);
        active_goal_.reset();
        active_goal_to_config_indices_.clear();
        active_goal_start_state_ = State{};
        active_goal_duration_sec_ = 0.0;
        active_goal_elapsed_sec_ = 0.0;
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

void HybridFollowJointTrajectoryExecutor::publishFeedback(
    const State& actual,
    const HybridTrajectorySample& desired) const {
    if (!server_ || !server_->isActive()) {
        return;
    }

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

    feedback.desired.positions = desired.state.positions;
    feedback.desired.velocities = desired.state.velocities;
    feedback.desired.accelerations = desired.state.accelerations;
    for (std::size_t axis_index = 0; axis_index < config_.joint_names.size();
         ++axis_index) {
        feedback.error.positions[axis_index] =
            feedback.desired.positions[axis_index] - actual.positions[axis_index];
        feedback.error.velocities[axis_index] =
            feedback.desired.velocities[axis_index] - actual.velocities[axis_index];
        feedback.error.accelerations[axis_index] =
            feedback.desired.accelerations[axis_index] - actual.accelerations[axis_index];
    }
    feedback.desired.time_from_start =
        ros::Duration(desired.sample_time_from_start_sec);
    feedback.error.time_from_start = ros::Duration(0.0);

    server_->publishFeedback(feedback);
}

void HybridFollowJointTrajectoryExecutor::update(const ros::Time& now,
                                                 const ros::Duration& period) {
    if (!config_valid_) {
        return;
    }

    if (!hasActiveGoal()) {
        return;
    }

    const State actual = ReadActualState();
    const double cycle_sec = executorCycleSec(config_.command_rate_hz);
    const double period_sec =
        (period.toSec() > 0.0 && std::isfinite(period.toSec())) ? period.toSec() : cycle_sec;
    double elapsed_sec = 0.0;
    double goal_duration_sec = 0.0;
    {
        std::lock_guard<std::mutex> lock(exec_mtx_);
        goal_duration_sec = active_goal_duration_sec_;
        elapsed_sec = active_goal_elapsed_sec_;
        active_goal_elapsed_sec_ += period_sec;
    }
    const double preview_sec = std::max(period_sec, cycle_sec);

    HybridTrajectorySample desired_now;
    HybridTrajectorySample desired_next;
    std::string sample_error;
    if (!sampleActiveGoal(elapsed_sec, &desired_now, &sample_error) ||
        !sampleActiveGoal(std::min(goal_duration_sec, elapsed_sec + preview_sec),
                          &desired_next,
                          &sample_error)) {
        {
            std::lock_guard<std::mutex> lock(exec_mtx_);
            active_goal_.reset();
            active_goal_to_config_indices_.clear();
            active_goal_start_state_ = State{};
            active_goal_duration_sec_ = 0.0;
            active_goal_elapsed_sec_ = 0.0;
            last_terminal_status_ = StepStatus::kError;
            last_terminal_error_ = sample_error;
        }
        exec_cv_.notify_all();
        return;
    }

    HybridJointTargetExecutor::Target target;
    target.state = desired_next.state;
    target.minimum_duration_sec.reset();
    target.continuous_reference = true;
    std::string target_error;
    if (!target_executor_->setTarget(target, &target_error)) {
        {
            std::lock_guard<std::mutex> lock(exec_mtx_);
            active_goal_.reset();
            active_goal_to_config_indices_.clear();
            active_goal_start_state_ = State{};
            active_goal_duration_sec_ = 0.0;
            active_goal_elapsed_sec_ = 0.0;
            last_terminal_status_ = StepStatus::kError;
            last_terminal_error_ = target_error;
        }
        exec_cv_.notify_all();
        return;
    }

    if (now.isValid() && (last_feedback_pub_time_.isZero() ||
                          (now - last_feedback_pub_time_) >= ros::Duration(0.05))) {
        publishFeedback(actual, desired_now);
        last_feedback_pub_time_ = now;
    }

    if (elapsed_sec < goal_duration_sec || !activeGoalReached(actual)) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(exec_mtx_);
        active_goal_.reset();
        active_goal_to_config_indices_.clear();
        active_goal_start_state_ = State{};
        active_goal_duration_sec_ = 0.0;
        active_goal_elapsed_sec_ = 0.0;
        last_terminal_status_ = StepStatus::kFinished;
        last_terminal_error_.clear();
    }
    exec_cv_.notify_all();
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
