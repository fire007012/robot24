#include "Eyou_ROS1_Master/hybrid_joint_target_executor.hpp"

#include <algorithm>
#include <cmath>

namespace eyou_ros1_master {

namespace {

bool HasDuplicateNames(const std::vector<std::string>& names,
                       std::string* duplicate_name) {
    for (std::size_t i = 0; i < names.size(); ++i) {
        for (std::size_t j = i + 1; j < names.size(); ++j) {
            if (names[i] == names[j]) {
                if (duplicate_name != nullptr) {
                    *duplicate_name = names[i];
                }
                return true;
            }
        }
    }
    return false;
}

bool HasDuplicateIndices(const std::vector<std::size_t>& indices,
                         std::size_t* duplicate_index) {
    for (std::size_t i = 0; i < indices.size(); ++i) {
        for (std::size_t j = i + 1; j < indices.size(); ++j) {
            if (indices[i] == indices[j]) {
                if (duplicate_index != nullptr) {
                    *duplicate_index = indices[i];
                }
                return true;
            }
        }
    }
    return false;
}

bool ValidateVectorSize(const std::vector<double>& values,
                        std::size_t expected_size,
                        const std::string& field_name,
                        std::string* error) {
    if (values.size() != expected_size) {
        if (error != nullptr) {
            *error = field_name + " size mismatch: expected " +
                     std::to_string(expected_size) + ", got " +
                     std::to_string(values.size());
        }
        return false;
    }
    return true;
}

bool ValidatePositiveVector(const std::vector<double>& values,
                            const std::string& field_name,
                            bool allow_zero,
                            std::string* error) {
    for (std::size_t i = 0; i < values.size(); ++i) {
        const bool ok = allow_zero ? (values[i] >= 0.0) : (values[i] > 0.0);
        if (!ok || !std::isfinite(values[i])) {
            if (error != nullptr) {
                *error = field_name + "[" + std::to_string(i) + "] must be " +
                         (allow_zero ? ">= 0" : "> 0");
            }
            return false;
        }
    }
    return true;
}

bool IsTerminalRuckigError(ruckig::Result result) {
    return result == ruckig::Result::ErrorInvalidInput ||
           result == ruckig::Result::Error ||
           result == ruckig::Result::ErrorTrajectoryDuration ||
           result == ruckig::Result::ErrorExecutionTimeCalculation ||
           result == ruckig::Result::ErrorSynchronizationCalculation;
}

}  // namespace

HybridJointTargetExecutor::HybridJointTargetExecutor(
    hardware_interface::RobotHW* hw,
    std::mutex* loop_mtx,
    Config config)
    : hw_raw_(hw),
      loop_mtx_(loop_mtx),
      config_(std::move(config)),
      otg_(config_.joint_names.size(),
           1.0 / std::max(1.0, config_.command_rate_hz)),
      input_(config_.joint_names.size()),
      output_(config_.joint_names.size()) {
    config_valid_ = ValidateConfig(config_, &config_error_);
    if (!config_valid_) {
        return;
    }

    if (hw_raw_ == nullptr) {
        config_valid_ = false;
        config_error_ = "RobotHW pointer is null";
        return;
    }

    auto* state_iface = hw_raw_->get<hardware_interface::JointStateInterface>();
    auto* pos_iface = hw_raw_->get<hardware_interface::PositionJointInterface>();
    if (state_iface == nullptr || pos_iface == nullptr) {
        config_valid_ = false;
        config_error_ = "RobotHW missing JointStateInterface or PositionJointInterface";
        return;
    }

    state_handles_.reserve(config_.joint_names.size());
    pos_cmd_handles_.reserve(config_.joint_names.size());
    for (const auto& name : config_.joint_names) {
        try {
            state_handles_.push_back(state_iface->getHandle(name));
            pos_cmd_handles_.push_back(pos_iface->getHandle(name));
        } catch (const hardware_interface::HardwareInterfaceException& e) {
            config_valid_ = false;
            config_error_ =
                "failed to get handle for joint '" + name + "': " + e.what();
            state_handles_.clear();
            pos_cmd_handles_.clear();
            return;
        }
    }

    input_.synchronization = ruckig::Synchronization::Time;
    input_.duration_discretization = ruckig::DurationDiscretization::Discrete;
}

bool HybridJointTargetExecutor::ValidateConfig(const Config& config,
                                               std::string* error) {
    if (config.command_rate_hz <= 0.0 || !std::isfinite(config.command_rate_hz)) {
        SetError(error, "command_rate_hz must be > 0");
        return false;
    }

    const std::size_t dofs = config.joint_names.size();
    if (dofs == 0) {
        SetError(error, "joint_names must not be empty");
        return false;
    }

    if (config.joint_indices.size() != dofs) {
        SetError(error, "joint_indices size mismatch: expected " +
                            std::to_string(dofs) + ", got " +
                            std::to_string(config.joint_indices.size()));
        return false;
    }
    if (!ValidateVectorSize(config.max_velocities, dofs, "max_velocities", error) ||
        !ValidateVectorSize(config.max_accelerations, dofs,
                            "max_accelerations", error) ||
        !ValidateVectorSize(config.max_jerks, dofs, "max_jerks", error) ||
        !ValidateVectorSize(config.goal_tolerances, dofs,
                            "goal_tolerances", error)) {
        return false;
    }

    std::string duplicate_name;
    if (HasDuplicateNames(config.joint_names, &duplicate_name)) {
        SetError(error, "joint_names contains duplicate joint: " + duplicate_name);
        return false;
    }

    std::size_t duplicate_index = 0;
    if (HasDuplicateIndices(config.joint_indices, &duplicate_index)) {
        SetError(error, "joint_indices contains duplicate axis index: " +
                            std::to_string(duplicate_index));
        return false;
    }

    if (!ValidatePositiveVector(config.max_velocities, "max_velocities", false, error) ||
        !ValidatePositiveVector(config.max_accelerations, "max_accelerations", false, error) ||
        !ValidatePositiveVector(config.max_jerks, "max_jerks", false, error) ||
        !ValidatePositiveVector(config.goal_tolerances, "goal_tolerances", true, error)) {
        return false;
    }

    return true;
}

bool HybridJointTargetExecutor::ValidateState(const State& state,
                                              std::size_t dofs,
                                              std::string* error) {
    if (state.positions.size() != dofs) {
        SetError(error, "state.positions size mismatch: expected " +
                            std::to_string(dofs) + ", got " +
                            std::to_string(state.positions.size()));
        return false;
    }
    if (!state.velocities.empty() && state.velocities.size() != dofs) {
        SetError(error, "state.velocities size mismatch: expected 0 or " +
                            std::to_string(dofs) + ", got " +
                            std::to_string(state.velocities.size()));
        return false;
    }
    if (!state.accelerations.empty() && state.accelerations.size() != dofs) {
        SetError(error, "state.accelerations size mismatch: expected 0 or " +
                            std::to_string(dofs) + ", got " +
                            std::to_string(state.accelerations.size()));
        return false;
    }
    return true;
}

void HybridJointTargetExecutor::EnsureStateArrays(State* state, std::size_t dofs) {
    if (state == nullptr) {
        return;
    }
    state->positions.resize(dofs, 0.0);
    state->velocities.resize(dofs, 0.0);
    state->accelerations.resize(dofs, 0.0);
}

double HybridJointTargetExecutor::StateValueOrZero(const std::vector<double>& values,
                                                   std::size_t index) {
    return index < values.size() ? values[index] : 0.0;
}

void HybridJointTargetExecutor::SetError(std::string* error,
                                         const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

bool HybridJointTargetExecutor::setTarget(const State& target, std::string* error) {
    if (!config_valid_) {
        SetError(error, config_error_);
        return false;
    }
    if (!ValidateState(target, dofs(), error)) {
        return false;
    }

    std::lock_guard<std::mutex> lock(target_mtx_);
    latest_target_ = target;
    ++target_generation_;
    return true;
}

void HybridJointTargetExecutor::clearTarget() {
    std::lock_guard<std::mutex> lock(target_mtx_);
    latest_target_.reset();
    ++target_generation_;
}

bool HybridJointTargetExecutor::hasTarget() const {
    std::lock_guard<std::mutex> lock(target_mtx_);
    return latest_target_.has_value();
}

HybridJointTargetExecutor::State HybridJointTargetExecutor::ReadActualState() const {
    State actual;
    EnsureStateArrays(&actual, dofs());
    for (std::size_t i = 0; i < dofs(); ++i) {
        actual.positions[i] = state_handles_[i].getPosition();
        actual.velocities[i] = state_handles_[i].getVelocity();
    }
    return actual;
}

void HybridJointTargetExecutor::WriteHoldPosition(const State& actual) {
    for (std::size_t i = 0; i < dofs(); ++i) {
        pos_cmd_handles_[i].setCommand(actual.positions[i]);
    }
}

void HybridJointTargetExecutor::WriteCommandPosition(
    const std::vector<double>& positions) {
    for (std::size_t i = 0; i < dofs(); ++i) {
        pos_cmd_handles_[i].setCommand(positions[i]);
    }
}

bool HybridJointTargetExecutor::InitializeTrajectory(const State& actual,
                                                     const State& target,
                                                     std::string* error) {
    for (std::size_t axis_index = 0; axis_index < dofs(); ++axis_index) {
        input_.current_position[axis_index] = actual.positions[axis_index];
        input_.current_velocity[axis_index] =
            StateValueOrZero(actual.velocities, axis_index);
        input_.current_acceleration[axis_index] =
            StateValueOrZero(actual.accelerations, axis_index);
        input_.target_position[axis_index] = target.positions[axis_index];
        input_.target_velocity[axis_index] =
            StateValueOrZero(target.velocities, axis_index);
        input_.target_acceleration[axis_index] =
            StateValueOrZero(target.accelerations, axis_index);
        input_.max_velocity[axis_index] = config_.max_velocities[axis_index];
        input_.max_acceleration[axis_index] =
            config_.max_accelerations[axis_index];
        input_.max_jerk[axis_index] = config_.max_jerks[axis_index];
    }
    input_.minimum_duration.reset();
    output_.time = 0.0;
    otg_.reset();

    if (!otg_.validate_input(input_)) {
        SetError(error, "ruckig rejected target input");
        return false;
    }
    return true;
}

void HybridJointTargetExecutor::update(const ros::Duration& period) {
    if (!config_valid_ || state_handles_.empty()) {
        return;
    }

    cycle_remainder_sec_ += period.toSec();
    const double cycle = 1.0 / std::max(1.0, config_.command_rate_hz);
    if (cycle_remainder_sec_ + 1e-9 < cycle) {
        return;
    }
    cycle_remainder_sec_ = std::fmod(cycle_remainder_sec_, cycle);

    const State actual = ReadActualState();

    std::optional<State> latest_target;
    std::uint64_t target_generation = 0;
    {
        std::lock_guard<std::mutex> lock(target_mtx_);
        latest_target = latest_target_;
        target_generation = target_generation_;
    }

    if (!latest_target.has_value()) {
        trajectory_initialized_ = false;
        trajectory_finished_ = false;
        WriteHoldPosition(actual);
        return;
    }

    if (!trajectory_initialized_ || target_generation != active_target_generation_) {
        std::string error;
        if (!InitializeTrajectory(actual, *latest_target, &error)) {
            trajectory_initialized_ = false;
            trajectory_finished_ = false;
            WriteHoldPosition(actual);
            return;
        }
        active_target_generation_ = target_generation;
        trajectory_initialized_ = true;
        trajectory_finished_ = false;
    }

    if (trajectory_finished_) {
        WriteCommandPosition(latest_target->positions);
        return;
    }

    const ruckig::Result result = otg_.update(input_, output_);
    if (IsTerminalRuckigError(result)) {
        trajectory_initialized_ = false;
        trajectory_finished_ = false;
        WriteHoldPosition(actual);
        return;
    }

    WriteCommandPosition(output_.new_position);
    output_.pass_to_input(input_);

    if (result == ruckig::Result::Finished) {
        trajectory_finished_ = true;
    }
}

}  // namespace eyou_ros1_master
