#include <gtest/gtest.h>

#include <algorithm>
#include <string>

#include <controller_manager_msgs/ListControllers.h>
#include <ros/master.h>
#include <ros/ros.h>

namespace {

class HybridLaunchSmokeTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "test_hybrid_launch_smoke",
                      ros::init_options::AnonymousName |
                          ros::init_options::NoSigintHandler);
        }
        ros::Time::init();
        ASSERT_TRUE(ros::master::check())
            << "test_hybrid_launch_smoke requires a running ROS master; run it via rostest.";
    }
};

TEST_F(HybridLaunchSmokeTest, DefaultLaunchAdvertisesServicesAndStartsJointStateController) {
    ros::NodeHandle nh;

    const ros::Duration service_timeout(15.0);
    for (const std::string& service_name : {
             "/hybrid_motor_hw_node/init",
             "/hybrid_motor_hw_node/enable",
             "/hybrid_motor_hw_node/disable",
             "/hybrid_motor_hw_node/halt",
             "/hybrid_motor_hw_node/resume",
             "/hybrid_motor_hw_node/recover",
             "/hybrid_motor_hw_node/shutdown",
             "/hybrid_motor_hw_node/set_joint_mode",
             "/hybrid_motor_hw_node/set_joint_zero",
             "/hybrid_motor_hw_node/apply_joint_limits",
             "/controller_manager/list_controllers"}) {
        EXPECT_TRUE(ros::service::waitForService(service_name, service_timeout))
            << "service did not appear: " << service_name;
    }

    ros::ServiceClient list_client =
        nh.serviceClient<controller_manager_msgs::ListControllers>(
            "/controller_manager/list_controllers", true);
    ASSERT_TRUE(list_client.waitForExistence(service_timeout));

    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(10.0);
    bool found_running_controller = false;
    while (ros::WallTime::now() < deadline) {
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
}

}  // namespace
