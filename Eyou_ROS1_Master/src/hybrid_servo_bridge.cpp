#include "Eyou_ROS1_Master/hybrid_servo_bridge.hpp"

#include <algorithm>
#include <cmath>

#include <ros/ros.h>

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

}  // namespace

HybridServoBridge::HybridServoBridge(ros::NodeHandle* pnh,
                                     HybridJointTargetExecutor* target_executor,
                                     Config config)
    : target_executor_(target_executor), config_(std::move(config)) {
    if (target_executor_ == nullptr || !target_executor_->valid()) {
        config_valid_ = false;
        config_error_ = "target executor is null or invalid";
        return;
    }
    if (config_.joint_names.empty()) {
        config_valid_ = false;
        config_error_ = "joint_names must not be empty";
        return;
    }
    if (config_.input_topic.empty()) {
        config_valid_ = false;
        config_error_ = "input_topic must not be empty";
        return;
    }
    if (!std::isfinite(config_.timeout_sec) || config_.timeout_sec <= 0.0) {
        config_valid_ = false;
        config_error_ = "timeout_sec must be > 0";
        return;
    }
    std::string duplicate_name;
    if (HasDuplicateNames(config_.joint_names, &duplicate_name)) {
        config_valid_ = false;
        config_error_ = "joint_names contains duplicate joint: " + duplicate_name;
        return;
    }

    config_valid_ = true;
    if (pnh != nullptr) {
        sub_ = pnh->subscribe<trajectory_msgs::JointTrajectory>(
            config_.input_topic, 1, &HybridServoBridge::OnTrajectory, this);
    }
}

void HybridServoBridge::SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

bool HybridServoBridge::acceptTrajectory(const trajectory_msgs::JointTrajectory& msg,
                                         const ros::Time& stamp,
                                         std::string* error) {
    if (!config_valid_) {
        SetError(error, config_error_);
        return false;
    }
    if (msg.joint_names.empty()) {
        SetError(error, "trajectory contains no joint names");
        return false;
    }
    std::string duplicate_name;
    if (HasDuplicateNames(msg.joint_names, &duplicate_name)) {
        SetError(error, "trajectory contains duplicate joint: " + duplicate_name);
        return false;
    }
    if (msg.joint_names.size() != config_.joint_names.size()) {
        SetError(error, "trajectory joint count mismatch: expected " +
                            std::to_string(config_.joint_names.size()) + ", got " +
                            std::to_string(msg.joint_names.size()));
        return false;
    }
    if (msg.points.empty()) {
        SetError(error, "trajectory contains no points");
        return false;
    }

    std::vector<std::size_t> msg_to_config_indices(msg.joint_names.size(), 0);
    for (std::size_t msg_index = 0; msg_index < msg.joint_names.size(); ++msg_index) {
        const auto it = std::find(config_.joint_names.begin(), config_.joint_names.end(),
                                  msg.joint_names[msg_index]);
        if (it == config_.joint_names.end()) {
            SetError(error, "trajectory contains unknown joint: " +
                                msg.joint_names[msg_index]);
            return false;
        }
        msg_to_config_indices[msg_index] =
            static_cast<std::size_t>(std::distance(config_.joint_names.begin(), it));
    }

    const auto& point = msg.points.back();
    if (point.positions.size() != msg.joint_names.size()) {
        SetError(error, "trajectory point positions size mismatch");
        return false;
    }
    if (!point.velocities.empty() && point.velocities.size() != msg.joint_names.size()) {
        SetError(error, "trajectory point velocities size mismatch");
        return false;
    }
    if (!point.accelerations.empty() &&
        point.accelerations.size() != msg.joint_names.size()) {
        SetError(error, "trajectory point accelerations size mismatch");
        return false;
    }

    HybridJointTargetExecutor::Target target;
    target.state.positions.resize(config_.joint_names.size(), 0.0);
    target.continuous_reference = true;
    if (!point.velocities.empty()) {
        target.state.velocities.resize(config_.joint_names.size(), 0.0);
    }

    for (std::size_t msg_index = 0; msg_index < msg.joint_names.size(); ++msg_index) {
        const std::size_t config_index = msg_to_config_indices[msg_index];
        target.state.positions[config_index] = point.positions[msg_index];
        if (!point.velocities.empty()) {
            target.state.velocities[config_index] = point.velocities[msg_index];
        }
    }

    {
        std::lock_guard<std::mutex> lock(mtx_);
        latest_target_ = std::move(target);
        last_message_time_ = stamp;
    }
    return true;
}

void HybridServoBridge::OnTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
    if (msg == nullptr) {
        return;
    }
    std::string error;
    if (!acceptTrajectory(*msg, ros::Time::now(), &error)) {
        ROS_WARN_THROTTLE(1.0, "Servo trajectory rejected: %s", error.c_str());
    }
}

void HybridServoBridge::update(const ros::Time& now) {
    if (!config_valid_) {
        return;
    }

    const auto active_source = target_executor_->active_source();
    if (target_executor_->getExecutionStatus() ==
            HybridJointTargetExecutor::ExecutionStatus::kTrackingFault &&
        active_source.has_value() &&
        *active_source == HybridJointTargetExecutor::Source::kServo) {
        std::string detail = "executor reported servo tracking fault";
        if (const auto fault = target_executor_->getTrackingFault(); fault.has_value()) {
            detail = "executor tracking fault on joint '" + fault->joint_name +
                     "' with position error " +
                     std::to_string(fault->position_error);
        }
        target_executor_->clearTargetFrom(HybridJointTargetExecutor::Source::kServo);
        {
            std::lock_guard<std::mutex> lock(mtx_);
            latest_target_.reset();
            last_message_time_ = ros::Time(0);
        }
        ROS_WARN_THROTTLE(1.0, "Servo target cleared after fault: %s",
                          detail.c_str());
        return;
    }

    std::optional<HybridJointTargetExecutor::Target> latest_target;
    ros::Time last_message_time;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        latest_target = latest_target_;
        last_message_time = last_message_time_;
    }

    if (!latest_target.has_value()) {
        return;
    }

    if (!last_message_time.isZero() &&
        (now - last_message_time).toSec() > config_.timeout_sec) {
        HybridJointTargetExecutor::Target stop_target;
        stop_target.state = target_executor_->getMeasuredState();
        stop_target.state.velocities.assign(config_.joint_names.size(), 0.0);
        stop_target.state.accelerations.assign(config_.joint_names.size(), 0.0);
        stop_target.continuous_reference = false;
        std::string error;
        target_executor_->setTargetFrom(HybridJointTargetExecutor::Source::kServo,
                                        stop_target, &error);
        std::lock_guard<std::mutex> lock(mtx_);
        latest_target_.reset();
        last_message_time_ = ros::Time(0);
        return;
    }

    std::string error;
    target_executor_->setTargetFrom(HybridJointTargetExecutor::Source::kServo,
                                    *latest_target, &error);
}

}  // namespace eyou_ros1_master
