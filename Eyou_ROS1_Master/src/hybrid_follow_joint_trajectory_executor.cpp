#include "Eyou_ROS1_Master/hybrid_follow_joint_trajectory_executor.hpp"

#include <chrono>
#include <cmath>

#include <ros/ros.h>

#include "canopen_hw/controllers/ip_follow_joint_trajectory_executor.hpp"

namespace eyou_ros1_master {

namespace {

struct ToleranceViolation {
    std::size_t axis_index{0};
    const char* component{nullptr};
    double error{0.0};
    double tolerance{0.0};
};

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

double executorCycleSec(double command_rate_hz) {
    return 1.0 / std::max(1.0, command_rate_hz);
}

double ValueOrZero(const std::vector<double>& values, std::size_t index) {
    return index < values.size() ? values[index] : 0.0;
}

bool ShouldForwardActionVelocityHint(ActionVelocityHintMode mode,
                                     std::size_t waypoint_count) {
    switch (mode) {
        case ActionVelocityHintMode::kDisabled:
            return false;
        case ActionVelocityHintMode::kSinglePoint:
            return waypoint_count == 1u;
        case ActionVelocityHintMode::kAll:
            return true;
    }
    return false;
}

bool IsStateWithinTolerance(
    const HybridFollowJointTrajectoryExecutor::State& actual,
    const HybridFollowJointTrajectoryExecutor::State& desired,
    const std::vector<JointStateTolerance>& tolerances) {
    for (std::size_t axis_index = 0; axis_index < tolerances.size(); ++axis_index) {
        const auto& tolerance = tolerances[axis_index];
        if (tolerance.position > 0.0 &&
            std::abs(ValueOrZero(actual.positions, axis_index) -
                     ValueOrZero(desired.positions, axis_index)) >
                tolerance.position) {
            return false;
        }
        if (tolerance.velocity > 0.0 &&
            std::abs(ValueOrZero(actual.velocities, axis_index) -
                     ValueOrZero(desired.velocities, axis_index)) >
                tolerance.velocity) {
            return false;
        }
        if (tolerance.acceleration > 0.0 &&
            std::abs(ValueOrZero(actual.accelerations, axis_index) -
                     ValueOrZero(desired.accelerations, axis_index)) >
                tolerance.acceleration) {
            return false;
        }
    }
    return true;
}

std::optional<ToleranceViolation> FindToleranceViolation(
    const HybridFollowJointTrajectoryExecutor::State& actual,
    const HybridFollowJointTrajectoryExecutor::State& desired,
    const std::vector<JointStateTolerance>& tolerances) {
    for (std::size_t axis_index = 0; axis_index < tolerances.size(); ++axis_index) {
        const auto& tolerance = tolerances[axis_index];

        const double position_error =
            ValueOrZero(actual.positions, axis_index) -
            ValueOrZero(desired.positions, axis_index);
        if (tolerance.position > 0.0 &&
            std::abs(position_error) > tolerance.position) {
            return ToleranceViolation{axis_index, "position", position_error,
                                      tolerance.position};
        }

        const double velocity_error =
            ValueOrZero(actual.velocities, axis_index) -
            ValueOrZero(desired.velocities, axis_index);
        if (tolerance.velocity > 0.0 &&
            std::abs(velocity_error) > tolerance.velocity) {
            return ToleranceViolation{axis_index, "velocity", velocity_error,
                                      tolerance.velocity};
        }

        const double acceleration_error =
            ValueOrZero(actual.accelerations, axis_index) -
            ValueOrZero(desired.accelerations, axis_index);
        if (tolerance.acceleration > 0.0 &&
            std::abs(acceleration_error) > tolerance.acceleration) {
            return ToleranceViolation{axis_index, "acceleration",
                                      acceleration_error,
                                      tolerance.acceleration};
        }
    }
    return std::nullopt;
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
    const std::size_t dofs = config_.joint_names.size();
    if (config_.default_path_tolerances.empty()) {
        config_.default_path_tolerances.assign(dofs, 0.0);
    }
    if (config_.default_goal_tolerances.empty()) {
        config_.default_goal_tolerances.assign(dofs, 0.0);
    }
    if (config_.default_path_tolerances.size() != dofs ||
        config_.default_goal_tolerances.size() != dofs) {
        config_valid_ = false;
        config_error_ =
            "default path/goal tolerance vectors must match joint_names size";
        return;
    }
    if (!std::isfinite(config_.default_stopped_velocity_tolerance) ||
        config_.default_stopped_velocity_tolerance < 0.0) {
        config_valid_ = false;
        config_error_ = "default_stopped_velocity_tolerance must be >= 0";
        return;
    }
    if (!std::isfinite(config_.default_goal_time_tolerance) ||
        config_.default_goal_time_tolerance < 0.0) {
        config_valid_ = false;
        config_error_ = "default_goal_time_tolerance must be >= 0";
        return;
    }
    if (!std::isfinite(config_.planning_deviation_warn_threshold) ||
        config_.planning_deviation_warn_threshold < 0.0) {
        config_valid_ = false;
        config_error_ = "planning_deviation_warn_threshold must be >= 0";
        return;
    }

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

control_msgs::FollowJointTrajectoryFeedback
HybridFollowJointTrajectoryExecutor::BuildFeedbackMessage(
    const std::vector<std::string>& joint_names,
    const State& actual,
    const HybridTrajectorySample& desired) {
    control_msgs::FollowJointTrajectoryFeedback feedback;
    feedback.header.stamp = ros::Time::now();
    feedback.joint_names = joint_names;

    feedback.actual.positions = actual.positions;
    feedback.actual.velocities = actual.velocities;
    feedback.actual.accelerations = actual.accelerations;
    feedback.actual.time_from_start =
        ros::Duration(desired.sample_time_from_start_sec);

    feedback.desired.positions = desired.state.positions;
    feedback.desired.velocities = desired.state.velocities;
    feedback.desired.accelerations = desired.state.accelerations;
    feedback.error.positions.resize(joint_names.size(), 0.0);
    feedback.error.velocities.resize(joint_names.size(), 0.0);
    feedback.error.accelerations.resize(joint_names.size(), 0.0);

    for (std::size_t axis_index = 0; axis_index < joint_names.size(); ++axis_index) {
        // Keep ROS FollowJointTrajectory feedback aligned with JTC semantics:
        // error = desired - actual, where desired is the nominal sampled reference.
        feedback.error.positions[axis_index] =
            feedback.desired.positions[axis_index] - actual.positions[axis_index];
        feedback.error.velocities[axis_index] =
            feedback.desired.velocities[axis_index] - actual.velocities[axis_index];
        feedback.error.accelerations[axis_index] =
            feedback.desired.accelerations[axis_index] -
            actual.accelerations[axis_index];
    }
    feedback.desired.time_from_start =
        ros::Duration(desired.sample_time_from_start_sec);
    feedback.error.time_from_start = ros::Duration(0.0);
    return feedback;
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
    double goal_duration_sec = 0.0;
    std::vector<JointStateTolerance> goal_tolerances;
    {
        std::lock_guard<std::mutex> lock(exec_mtx_);
        if (!active_goal_) {
            return false;
        }
        goal_duration_sec = active_goal_duration_sec_;
        goal_tolerances = active_goal_tolerances_.goal_state_tolerance;
    }

    std::string error;
    if (!sampleActiveGoal(goal_duration_sec, &final_sample, &error)) {
        return false;
    }
    return IsStateWithinTolerance(actual, final_sample.state, goal_tolerances);
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

    FollowJointTrajectoryResolvedTolerances resolved_tolerances;
    if (!ResolveFollowJointTrajectoryTolerances(goal,
                                                config_.joint_names,
                                                config_.default_path_tolerances,
                                                config_.default_goal_tolerances,
                                                config_.default_stopped_velocity_tolerance,
                                                config_.default_goal_time_tolerance,
                                                &resolved_tolerances,
                                                error)) {
        return false;
    }

    std::lock_guard<std::mutex> lock(exec_mtx_);
    active_goal_ = goal;
    active_goal_tolerances_ = std::move(resolved_tolerances);
    active_goal_to_config_indices_ = std::move(goal_to_config_indices);
    active_goal_start_state_ = start_state;
    active_goal_duration_sec_ = goal_duration_sec;
    active_goal_elapsed_sec_ = 0.0;
    last_terminal_status_.reset();
    last_terminal_result_code_.reset();
    last_terminal_error_.clear();
    last_feedback_pub_time_ = ros::Time(0);
    return true;
}

void HybridFollowJointTrajectoryExecutor::cancelGoal() {
    {
        std::lock_guard<std::mutex> lock(exec_mtx_);
        resetActiveGoalLocked();
        last_terminal_status_ = StepStatus::kIdle;
        last_terminal_result_code_ =
            control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
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

std::optional<int> HybridFollowJointTrajectoryExecutor::getLastTerminalResultCode()
    const {
    std::lock_guard<std::mutex> lock(exec_mtx_);
    return last_terminal_result_code_;
}

std::string HybridFollowJointTrajectoryExecutor::getLastTerminalError() const {
    std::lock_guard<std::mutex> lock(exec_mtx_);
    return last_terminal_error_;
}

HybridFollowJointTrajectoryExecutor::DiagnosticState
HybridFollowJointTrajectoryExecutor::getDiagnosticState() const {
    std::lock_guard<std::mutex> lock(exec_mtx_);
    return diagnostic_state_;
}

void HybridFollowJointTrajectoryExecutor::resetActiveGoalLocked() {
    active_goal_.reset();
    active_goal_tolerances_ = FollowJointTrajectoryResolvedTolerances{};
    active_goal_to_config_indices_.clear();
    active_goal_start_state_ = State{};
    active_goal_duration_sec_ = 0.0;
    active_goal_elapsed_sec_ = 0.0;
    diagnostic_state_ = DiagnosticState{};
}

void HybridFollowJointTrajectoryExecutor::setTerminalStateLocked(
    StepStatus status,
    int result_code,
    const std::string& error) {
    resetActiveGoalLocked();
    last_terminal_status_ = status;
    last_terminal_result_code_ = result_code;
    last_terminal_error_ = error;
}

void HybridFollowJointTrajectoryExecutor::publishFeedback(
    const State& actual,
    const HybridTrajectorySample& desired) const {
    if (!server_ || !server_->isActive()) {
        return;
    }

    control_msgs::FollowJointTrajectoryFeedback feedback =
        BuildFeedbackMessage(config_.joint_names, actual, desired);
    server_->publishFeedback(feedback);
}

void HybridFollowJointTrajectoryExecutor::finalizeGoal(StepStatus status,
                                                       int result_code,
                                                       const std::string& error) {
    {
        std::lock_guard<std::mutex> lock(exec_mtx_);
        setTerminalStateLocked(status, result_code, error);
    }
    if (target_executor_ != nullptr) {
        target_executor_->clearTarget();
    }
    exec_cv_.notify_all();
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
    const double period_sec =
        (period.toSec() > 0.0 && std::isfinite(period.toSec()))
            ? period.toSec()
            : executorCycleSec(config_.command_rate_hz);
    double elapsed_sec = 0.0;
    double goal_duration_sec = 0.0;
    double goal_time_tolerance_sec = 0.0;
    std::size_t trajectory_point_count = 0u;
    std::vector<JointStateTolerance> path_tolerances;
    {
        std::lock_guard<std::mutex> lock(exec_mtx_);
        goal_duration_sec = active_goal_duration_sec_;
        goal_time_tolerance_sec = active_goal_tolerances_.goal_time_tolerance;
        path_tolerances = active_goal_tolerances_.path_state_tolerance;
        elapsed_sec = active_goal_elapsed_sec_;
        active_goal_elapsed_sec_ += period_sec;
        trajectory_point_count = active_goal_.has_value()
                                     ? active_goal_->trajectory.points.size()
                                     : 0u;
    }

    HybridTrajectorySample desired_now;
    std::string sample_error;
    if (!sampleActiveGoal(std::min(elapsed_sec, goal_duration_sec), &desired_now,
                          &sample_error)) {
        finalizeGoal(
            StepStatus::kError,
            control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED,
            sample_error.empty() ? "failed to sample active goal" : sample_error);
        return;
    }
    {
        std::lock_guard<std::mutex> lock(exec_mtx_);
        diagnostic_state_.has_nominal_reference = true;
        diagnostic_state_.nominal_reference = desired_now;
        diagnostic_state_.trajectory_duration_sec = goal_duration_sec;
    }

    HybridJointTargetExecutor::Target target;
    target.state.positions = desired_now.state.positions;
    if (ShouldForwardActionVelocityHint(config_.action_velocity_hint_mode,
                                        trajectory_point_count)) {
        target.state.velocities = desired_now.state.velocities;
    } else {
        target.state.velocities.clear();
    }
    target.state.accelerations.clear();
    target.continuous_reference = true;
    target.minimum_duration_sec.reset();

    std::string target_error;
    if (!target_executor_->setTarget(target, &target_error)) {
        finalizeGoal(
            StepStatus::kError,
            control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED,
            target_error.empty() ? "failed to update action target"
                                 : target_error);
        return;
    }

    if (target_executor_->getExecutionStatus() ==
        HybridJointTargetExecutor::ExecutionStatus::kTrackingFault) {
        std::string fault_error = "executor tracking fault";
        if (const auto fault = target_executor_->getTrackingFault(); fault.has_value()) {
            fault_error = "tracking fault on joint '" + fault->joint_name +
                          "' with position error " +
                          std::to_string(fault->position_error);
        }
        finalizeGoal(
            StepStatus::kError,
            control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED,
            fault_error);
        return;
    }

    if (const auto violation =
            FindToleranceViolation(actual, desired_now.state, path_tolerances);
        violation.has_value()) {
        const std::string error = "path tolerance violated for joint '" +
                                  config_.joint_names[violation->axis_index] + "' " +
                                  violation->component + " error " +
                                  std::to_string(violation->error) +
                                  " exceeds " +
                                  std::to_string(violation->tolerance);
        finalizeGoal(
            StepStatus::kError,
            control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED,
            error);
        return;
    }

    const State command = target_executor_->getCurrentCommand();
    for (std::size_t axis_index = 0; axis_index < config_.joint_names.size();
         ++axis_index) {
        const double deviation =
            ValueOrZero(command.positions, axis_index) -
            ValueOrZero(desired_now.state.positions, axis_index);
        const double warn_threshold = config_.planning_deviation_warn_threshold;
        if (warn_threshold > 0.0 && std::abs(deviation) > warn_threshold) {
            ROS_WARN_THROTTLE(
                2.0,
                "Action joint '%s' planning deviation %.6f exceeds warn threshold %.6f",
                config_.joint_names[axis_index].c_str(), deviation, warn_threshold);
        }
    }

    if (now.isValid() && (last_feedback_pub_time_.isZero() ||
                          (now - last_feedback_pub_time_) >= ros::Duration(0.05))) {
        publishFeedback(actual, desired_now);
        last_feedback_pub_time_ = now;
    }

    if (elapsed_sec < goal_duration_sec) {
        return;
    }

    if (activeGoalReached(actual)) {
        finalizeGoal(StepStatus::kFinished,
                     control_msgs::FollowJointTrajectoryResult::SUCCESSFUL,
                     "goal completed");
        return;
    }

    if (elapsed_sec <= goal_duration_sec + goal_time_tolerance_sec) {
        return;
    }

    finalizeGoal(
        StepStatus::kError,
        control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED,
        "goal tolerance violated after goal_time_tolerance expired");
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
        result.error_code = last_terminal_result_code_.value_or(
            control_msgs::FollowJointTrajectoryResult::SUCCESSFUL);
        result.error_string =
            last_terminal_error_.empty() ? "goal completed" : last_terminal_error_;
        server_->setSucceeded(result, result.error_string);
        return;
    }

    target_executor_->clearTarget();
    result.error_code = last_terminal_result_code_.value_or(
        control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED);
    result.error_string =
        last_terminal_error_.empty() ? "goal execution failed"
                                     : last_terminal_error_;
    server_->setAborted(result, result.error_string);
}

}  // namespace eyou_ros1_master
