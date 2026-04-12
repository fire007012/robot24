#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/duration.h>
#include <ruckig/input_parameter.hpp>
#include <ruckig/output_parameter.hpp>
#include <ruckig/ruckig.hpp>

namespace eyou_ros1_master {

class HybridJointTargetExecutor {
public:
    struct State {
        std::vector<double> positions;
        std::vector<double> velocities;
        std::vector<double> accelerations;
    };

    struct Config {
        std::vector<std::string> joint_names;
        std::vector<std::size_t> joint_indices;
        double command_rate_hz{100.0};
        std::vector<double> max_velocities;
        std::vector<double> max_accelerations;
        std::vector<double> max_jerks;
        std::vector<double> goal_tolerances;
    };

    HybridJointTargetExecutor(hardware_interface::RobotHW* hw,
                              std::mutex* loop_mtx,
                              Config config);

    bool valid() const { return config_valid_; }
    const std::string& config_error() const { return config_error_; }
    std::size_t dofs() const { return config_.joint_names.size(); }

    bool setTarget(const State& target, std::string* error = nullptr);
    void clearTarget();
    bool hasTarget() const;

    void update(const ros::Duration& period);

private:
    static bool ValidateConfig(const Config& config, std::string* error);
    static bool ValidateState(const State& state, std::size_t dofs, std::string* error);
    static void EnsureStateArrays(State* state, std::size_t dofs);
    static double StateValueOrZero(const std::vector<double>& values, std::size_t index);
    static void SetError(std::string* error, const std::string& message);

    State ReadActualState() const;
    void WriteHoldPosition(const State& actual);
    void WriteCommandPosition(const std::vector<double>& positions);
    bool InitializeTrajectory(const State& actual,
                              const State& target,
                              std::string* error);

    hardware_interface::RobotHW* hw_raw_{nullptr};
    std::mutex* loop_mtx_{nullptr};
    Config config_;
    std::vector<hardware_interface::JointStateHandle> state_handles_;
    std::vector<hardware_interface::JointHandle> pos_cmd_handles_;

    mutable std::mutex target_mtx_;
    std::optional<State> latest_target_;
    std::uint64_t target_generation_{0};
    std::uint64_t active_target_generation_{0};

    bool config_valid_{false};
    std::string config_error_;
    double cycle_remainder_sec_{0.0};
    bool trajectory_initialized_{false};
    bool trajectory_finished_{false};

    ruckig::Ruckig<0> otg_;
    ruckig::InputParameter<0> input_;
    ruckig::OutputParameter<0> output_;
};

}  // namespace eyou_ros1_master
