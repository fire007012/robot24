#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "Eyou_ROS1_Master/hybrid_joint_target_executor.hpp"

namespace eyou_ros1_master {

class HybridServoBridge {
public:
    struct Config {
        std::vector<std::string> joint_names;
        std::string input_topic{"servo_joint_targets"};
        double timeout_sec{0.1};
    };

    HybridServoBridge(ros::NodeHandle* pnh,
                      HybridJointTargetExecutor* target_executor,
                      Config config);

    bool valid() const { return config_valid_; }
    const std::string& config_error() const { return config_error_; }
    void update(const ros::Time& now);

    bool acceptTrajectory(const trajectory_msgs::JointTrajectory& msg,
                          const ros::Time& stamp,
                          std::string* error = nullptr);

private:
    void OnTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
    static void SetError(std::string* error, const std::string& message);

    HybridJointTargetExecutor* target_executor_{nullptr};
    Config config_;
    ros::Subscriber sub_;

    mutable std::mutex mtx_;
    std::optional<HybridJointTargetExecutor::Target> latest_target_;
    ros::Time last_message_time_;
    bool config_valid_{false};
    std::string config_error_;
};

}  // namespace eyou_ros1_master
