#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Joy.h>
#include "vision_pkg/panorama.h"

class PanoramaNode
{
public:
    PanoramaNode() : nh_("~"), it_(nh_)
    {
        pub_ = it_.advertise("panorama_image", 1);

        std::string cam1_topic, cam2_topic;
        nh_.param<std::string>("cam1_topic", cam1_topic, "/camera/forward/image_raw");
        nh_.param<std::string>("cam2_topic", cam2_topic, "/camera/back/image_raw");

        sub_cam1_ = it_.subscribe(cam1_topic, 1, &PanoramaNode::cam1Callback, this);
        sub_cam2_ = it_.subscribe(cam2_topic, 1, &PanoramaNode::cam2Callback, this);
        sub_joy_ = nh_.subscribe("/joy", 1, &PanoramaNode::joyCallback, this);

        panorama_.setPanoramaReadyCallback([this](const cv::Mat& img) {
            auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
            msg->header.stamp = ros::Time::now();
            pub_.publish(msg);
        });
    }

private:
    void cam1Callback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        panorama_.receiveCam1(frame);
    }

    void cam2Callback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        panorama_.receiveCam2(frame);
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        // axes[3] = right stick X, axes[4] = right stick Y (standard joy mapping)
        // buttons[10] = right thumb press
        float rx = 0.0f, ry = 0.0f;
        bool rightThumb = false;
        if (msg->axes.size() > 4) {
            rx = msg->axes[3] * 32767.0f;
            ry = msg->axes[4] * 32767.0f;
        }
        if (msg->buttons.size() > 10) {
            rightThumb = msg->buttons[10];
        }
        panorama_.handleControllerState(rx, ry, rightThumb);
    }

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
    image_transport::Subscriber sub_cam1_;
    image_transport::Subscriber sub_cam2_;
    ros::Subscriber sub_joy_;
    Panorama panorama_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "panorama_node");
    PanoramaNode node;
    ros::spin();
    return 0;
}
