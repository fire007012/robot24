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
#include <ros/time.h>
#include <ruckig/input_parameter.hpp>
#include <ruckig/output_parameter.hpp>
#include <ruckig/ruckig.hpp>

namespace eyou_ros1_master {

class HybridJointTargetExecutor {
public:
    enum class Source : std::uint8_t {
        kAction,
        kServo,
    };

    enum class ExecutionStatus : std::uint8_t {
        kHold,
        kTracking,
        kFinished,
        kTrackingFault,
        kResyncing,
        kError,
    };

    enum class ContinuousModeState : std::uint8_t {
        kInactive,
        kInitFromHardware,
        kFollowInternalState,
        kResyncFromHardware,
    };

    struct State {
        std::vector<double> positions;
        std::vector<double> velocities;
        std::vector<double> accelerations;
    };

    struct TrackingFault {
        std::size_t joint_index{0};
        std::string joint_name;
        double position_error{0.0};
        ros::Time stamp;
    };

    struct Config {
        std::vector<std::string> joint_names;
        std::vector<std::size_t> joint_indices;
        double command_rate_hz{100.0};
        std::vector<double> max_velocities;
        std::vector<double> max_accelerations;
        std::vector<double> max_jerks;
        std::vector<double> goal_tolerances;
        double continuous_resync_threshold{0.04};
        double continuous_resync_recovery_threshold{0.02};
        std::size_t continuous_resync_enter_cycles{2};
        std::size_t continuous_resync_recovery_cycles{2};
        double tracking_fault_threshold{0.08};
    };

    struct Target {
        State state;
        std::optional<double> minimum_duration_sec;
        bool continuous_reference{false};
    };

    HybridJointTargetExecutor(hardware_interface::RobotHW* hw,
                              std::mutex* loop_mtx,
                              Config config);

    bool valid() const { return config_valid_; }
    const std::string& config_error() const { return config_error_; }
    std::size_t dofs() const { return config_.joint_names.size(); }
    const std::vector<std::string>& getJointNames() const { return config_.joint_names; }

    bool setTarget(const Target& target, std::string* error = nullptr);
    bool setTarget(const State& target, std::string* error = nullptr);
    bool setTargetFrom(Source source, const Target& target, std::string* error = nullptr);
    bool setTargetFrom(Source source, const State& target, std::string* error = nullptr);
    void clearTarget();
    void clearTargetFrom(Source source);
    bool hasTarget() const;
    std::optional<Source> active_source() const;
    std::optional<Target> getActiveTarget() const;
    ExecutionStatus getExecutionStatus() const;
    ContinuousModeState getContinuousModeState() const;
    State getMeasuredState() const;
    State getCurrentCommand() const;
    std::optional<TrackingFault> getTrackingFault() const;

    void update(const ros::Duration& period);

private:
    static bool ValidateConfig(const Config& config, std::string* error);
    static bool ValidateState(const State& state, std::size_t dofs, std::string* error);
    static bool ValidateTarget(const Target& target, std::size_t dofs, std::string* error);
    static void EnsureStateArrays(State* state, std::size_t dofs);
    static double StateValueOrZero(const std::vector<double>& values, std::size_t index);
    static void SetError(std::string* error, const std::string& message);

    State ReadActualState() const;
    void WriteHoldPosition(const State& actual);
    void WriteCommandPosition(const std::vector<double>& positions);
    void UpdateObservedState(const State& measured,
                             const State& command,
                             ExecutionStatus status,
                             ContinuousModeState continuous_mode_state,
                             const std::optional<TrackingFault>& tracking_fault);
    bool InitializeTrajectory(const State& actual,
                              const Target& target,
                              std::string* error);
    bool UpdateTrajectoryTarget(const Target& target,
                                std::string* error);
    void ResetContinuousMode();
    bool IsPositionErrorAboveThreshold(const State& measured,
                                       const State& command,
                                       double threshold) const;
    bool ArePositionErrorsWithinThreshold(const State& measured,
                                          const State& command,
                                          double threshold) const;
    std::optional<TrackingFault> DetectTrackingFault(const State& measured,
                                                     const State& command) const;
    void ClearTrackingFault();

    hardware_interface::RobotHW* hw_raw_{nullptr};
    std::mutex* loop_mtx_{nullptr};
    Config config_;
    std::vector<hardware_interface::JointStateHandle> state_handles_;
    std::vector<hardware_interface::JointHandle> pos_cmd_handles_;

    mutable std::mutex target_mtx_;
    std::optional<Source> active_source_;
    std::optional<Target> latest_target_;
    std::uint64_t target_generation_{0};
    std::uint64_t active_target_generation_{0};

    bool config_valid_{false};
    std::string config_error_;
    double cycle_remainder_sec_{0.0};
    bool trajectory_initialized_{false};
    bool trajectory_finished_{false};
    bool active_target_is_continuous_{false};
    ContinuousModeState continuous_mode_state_{ContinuousModeState::kInactive};
    std::size_t continuous_resync_enter_counter_{0};
    std::size_t continuous_resync_recovery_counter_{0};
    bool tracking_fault_active_{false};
    std::optional<TrackingFault> tracking_fault_;
    mutable std::mutex state_mtx_;
    State last_measured_state_;
    State last_command_state_;
    ExecutionStatus execution_status_{ExecutionStatus::kHold};
    ContinuousModeState observed_continuous_mode_state_{
        ContinuousModeState::kInactive};
    std::optional<TrackingFault> observed_tracking_fault_;

    ruckig::Ruckig<0> otg_;
    ruckig::InputParameter<0> input_;
    ruckig::OutputParameter<0> output_;
};

}  // namespace eyou_ros1_master
