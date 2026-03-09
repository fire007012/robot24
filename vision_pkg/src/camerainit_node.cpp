#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <thread>
#include "vision_pkg/camerainit.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camerainit_node");
    ros::NodeHandle nh("~");

    std::string device_path;
    std::string topic_name;
    nh.param<std::string>("device_path", device_path, "/dev/video0");
    nh.param<std::string>("topic_name", topic_name, "image_raw");

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise(topic_name, 1);

    CameraInit cam(device_path);
    cam.setFrameCallback([&pub](const cv::Mat& frame) {
        auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = ros::Time::now();
        pub.publish(msg);
    });

    if (!cam.initAndOpenCamera()) {
        ROS_ERROR("Failed to open camera: %s", device_path.c_str());
        return 1;
    }

    ROS_INFO("camerainit_node started: %s -> %s", device_path.c_str(), topic_name.c_str());

    std::thread capture_thread([&cam]() { cam.captureLoop(); });

    ros::spin();
    cam.closeCamera();
    capture_thread.join();
    return 0;
}
