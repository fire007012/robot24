#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <thread>
#include "vision_pkg/realsense.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsense_node");
    ros::NodeHandle nh("~");

    std::string serial;
    std::string topic_name;
    int device_index;
    nh.param<std::string>("serial", serial, "");
    nh.param<std::string>("topic_name", topic_name, "image_raw");
    nh.param<int>("device_index", device_index, -1);

    if (serial.empty() && device_index < 0) {
        ROS_WARN("No 'serial' specified, using first available device");
        device_index = 0;
    }

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise(topic_name, 1);

    Realsense rs(serial, device_index);
    rs.setFrameCallback([&pub](const cv::Mat& frame) {
        auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = ros::Time::now();
        pub.publish(msg);
    });

    if (!rs.initAndOpenCamera()) {
        ROS_ERROR("Failed to open RealSense: %s", serial.c_str());
        return 1;
    }

    ROS_INFO("realsense_node started: serial=%s -> %s", serial.c_str(), topic_name.c_str());

    std::thread capture_thread([&rs]() { rs.captureLoop(); });

    ros::spin();
    rs.closeCamera();
    capture_thread.join();
    return 0;
}
