#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include "Eyou_ROS1_Master/hybrid_joint_target_executor.hpp"
#include "Eyou_ROS1_Master/hybrid_tolerance_resolver.hpp"
#include "Eyou_ROS1_Master/hybrid_trajectory_time_sampler.hpp"
#include "canopen_hw/controllers/ip_follow_joint_trajectory_executor.hpp"

namespace eyou_ros1_master {

class HybridFollowJointTrajectoryExecutor {
public:
    using State = HybridJointTargetExecutor::State;
    using Config = canopen_hw::IpFollowJointTrajectoryExecutor::Config;

    enum class StepStatus {
        kIdle,
        kWorking,
        kFinished,
        kError,
    };

    HybridFollowJointTrajectoryExecutor(ros::NodeHandle* pnh,
                                        hardware_interface::RobotHW* hw,
                                        std::mutex* loop_mtx,
                                        HybridJointTargetExecutor* target_executor,
                                        Config config);

    void update(const ros::Time& now, const ros::Duration& period);
    bool enabled() const { return config_valid_ && server_ != nullptr; }
    bool config_valid() const { return config_valid_; }
    const std::string& config_error() const { return config_error_; }

    static bool ValidateGoal(
        const control_msgs::FollowJointTrajectoryGoal& goal,
        const std::vector<std::string>& joint_names, std::string* error);

    bool startGoal(const control_msgs::FollowJointTrajectoryGoal& goal,
                   std::string* error);
    void cancelGoal();
    bool hasActiveGoal() const;

private:
    using Action = control_msgs::FollowJointTrajectoryAction;
    using GoalConstPtr = control_msgs::FollowJointTrajectoryGoalConstPtr;
    using Server = actionlib::SimpleActionServer<Action>;

    void ExecuteGoal(const GoalConstPtr& goal);
    void publishFeedback(const State& actual,
                         const HybridTrajectorySample& desired) const;
    State ReadActualState() const;
    bool sampleActiveGoal(double time_from_start_sec,
                          HybridTrajectorySample* sample,
                          std::string* error) const;
    bool activeGoalReached(const State& actual) const;

    hardware_interface::RobotHW* hw_raw_{nullptr};
    std::mutex* loop_mtx_{nullptr};
    HybridJointTargetExecutor* target_executor_{nullptr};
    Config config_;
    std::unique_ptr<Server> server_;
    std::vector<hardware_interface::JointStateHandle> state_handles_;

    mutable std::mutex exec_mtx_;
    std::condition_variable exec_cv_;
    std::optional<control_msgs::FollowJointTrajectoryGoal> active_goal_;
    FollowJointTrajectoryResolvedTolerances active_goal_tolerances_;
    std::vector<std::size_t> active_goal_to_config_indices_;
    State active_goal_start_state_;
    double active_goal_duration_sec_{0.0};
    double active_goal_elapsed_sec_{0.0};
    std::optional<StepStatus> last_terminal_status_;
    std::string last_terminal_error_;
    std::string config_error_;
    bool config_valid_{false};
    ros::Time last_feedback_pub_time_;
};

}  // namespace eyou_ros1_master
