#include <gtest/gtest.h>

#include <algorithm>
#include <string>

#include <controller_manager_msgs/ListControllers.h>
#include <ros/master.h>
#include <ros/ros.h>

namespace {

class HybridLaunchIpExecutorTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "test_hybrid_launch_ip_executor",
                      ros::init_options::AnonymousName |
                          ros::init_options::NoSigintHandler);
        }
        ros::Time::init();
        ASSERT_TRUE(ros::master::check())
            << "test_hybrid_launch_ip_executor requires a running ROS master; run it via rostest.";
    }

    static bool HasPublishedTopic(const std::string& topic_name,
                                  const std::string& topic_type) {
        ros::master::V_TopicInfo topics;
        if (!ros::master::getTopics(topics)) {
            return false;
        }

        return std::any_of(topics.begin(), topics.end(),
                           [&](const ros::master::TopicInfo& info) {
                               return info.name == topic_name &&
                                      info.datatype == topic_type;
                           });
    }
};

TEST_F(HybridLaunchIpExecutorTest,
       LaunchWithIpExecutorAdvertisesActionServerAndKeepsJointStateControllerRunning) {
    ros::NodeHandle nh;

    const ros::Duration service_timeout(15.0);
    ASSERT_TRUE(ros::service::waitForService("/hybrid_motor_hw_node/init",
                                             service_timeout));
    ASSERT_TRUE(ros::service::waitForService("/controller_manager/list_controllers",
                                             service_timeout));

    ros::ServiceClient list_client =
        nh.serviceClient<controller_manager_msgs::ListControllers>(
            "/controller_manager/list_controllers", true);
    ASSERT_TRUE(list_client.waitForExistence(service_timeout));

    const ros::WallTime controller_deadline =
        ros::WallTime::now() + ros::WallDuration(10.0);
    bool found_running_controller = false;
    while (ros::WallTime::now() < controller_deadline) {
        controller_manager_msgs::ListControllers list_srv;
        ASSERT_TRUE(list_client.call(list_srv));

        const auto controller_it = std::find_if(
            list_srv.response.controller.begin(), list_srv.response.controller.end(),
            [](const controller_manager_msgs::ControllerState& state) {
                return state.name == "joint_state_controller" &&
                       state.state == "running";
            });
        if (controller_it != list_srv.response.controller.end()) {
            found_running_controller = true;
            break;
        }
        ros::WallDuration(0.1).sleep();
    }
    EXPECT_TRUE(found_running_controller)
        << "joint_state_controller did not appear in running state within timeout";

    const ros::WallTime action_deadline =
        ros::WallTime::now() + ros::WallDuration(10.0);
    bool found_status_topic = false;
    bool found_feedback_topic = false;
    bool found_diagnostics_topic = false;
    while (ros::WallTime::now() < action_deadline) {
        found_status_topic = HasPublishedTopic(
            "/arm_position_controller/follow_joint_trajectory/status",
            "actionlib_msgs/GoalStatusArray");
        found_feedback_topic = HasPublishedTopic(
            "/arm_position_controller/follow_joint_trajectory/feedback",
            "control_msgs/FollowJointTrajectoryActionFeedback");
        found_diagnostics_topic = HasPublishedTopic(
            "/hybrid_motor_hw_node/trajectory_execution_state",
            "Eyou_ROS1_Master/TrajectoryExecutionState");
        if (found_status_topic && found_feedback_topic &&
            found_diagnostics_topic) {
            break;
        }
        ros::WallDuration(0.1).sleep();
    }

    EXPECT_TRUE(found_status_topic)
        << "IP executor action status topic was not advertised at the expected global namespace";
    EXPECT_TRUE(found_feedback_topic)
        << "IP executor action feedback topic was not advertised at the expected global namespace";
    EXPECT_TRUE(found_diagnostics_topic)
        << "trajectory diagnostics topic was not advertised from hybrid_motor_hw_node";
}

}  // namespace
