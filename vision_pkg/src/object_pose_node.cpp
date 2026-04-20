#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <vector>
#include "vision_pkg/Detection.h"

class ObjectPoseNode
{
public:
    ObjectPoseNode() : nh_("~"), has_camera_info_(false), has_depth_(false), has_pose_(false), first_pose_(true)
    {
        std::string detection_topic, depth_topic, camera_info_topic;
        nh_.param<std::string>("detection_topic", detection_topic, "/yolov8/detections");
        nh_.param<std::string>("depth_topic", depth_topic, "/paw_camera/depth/image_rect_raw");
        nh_.param<std::string>("camera_info_topic", camera_info_topic, "/paw_camera/color/camera_info");
        nh_.param<int>("publish_rate", publish_rate_, 20);
        nh_.param<double>("smooth_alpha", smooth_alpha_, 0.3);

        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/detected_object_pose", 10);

        det_sub_ = nh_.subscribe(detection_topic, 10, &ObjectPoseNode::detectionCallback, this);
        depth_sub_ = nh_.subscribe(depth_topic, 1, &ObjectPoseNode::depthCallback, this);
        info_sub_ = nh_.subscribe(camera_info_topic, 1, &ObjectPoseNode::cameraInfoCallback, this);

        ROS_INFO("object_pose_node started");
        ROS_INFO("  detection_topic: %s", detection_topic.c_str());
        ROS_INFO("  depth_topic:     %s", depth_topic.c_str());
        ROS_INFO("  camera_info:     %s", camera_info_topic.c_str());
    }

    void run()
    {
        ros::Rate rate(publish_rate_);
        while (ros::ok()) {
            ros::spinOnce();
            if (has_pose_) {
                cached_pose_.header.stamp = ros::Time::now();
                pose_pub_.publish(cached_pose_);
            }
            rate.sleep();
        }
    }

private:
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
    {
        if (has_camera_info_) return;

        // K matrix: [fx 0 cx; 0 fy cy; 0 0 1]
        fx_ = msg->K[0];
        fy_ = msg->K[4];
        cx_ = msg->K[2];
        cy_ = msg->K[5];
        has_camera_info_ = true;

        info_sub_.shutdown();
        ROS_INFO("Camera intrinsics received: fx=%.1f fy=%.1f cx=%.1f cy=%.1f", fx_, fy_, cx_, cy_);
    }

    void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        try {
            depth_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
            has_depth_ = true;
        } catch (cv_bridge::Exception& e) {
            ROS_WARN_THROTTLE(5.0, "depth cv_bridge error: %s", e.what());
        }
    }

    void detectionCallback(const vision_pkg::Detection::ConstPtr& msg)
    {
        if (!has_camera_info_ || !has_depth_) return;

        // 只处理置信度更高的检测（每帧可能收到多个 Detection）
        if (msg->confidence <= best_confidence_) return;
        best_confidence_ = msg->confidence;

        // 延迟重置：用 ros timer 在下一个周期清零
        if (!reset_timer_.isValid()) {
            reset_timer_ = nh_.createTimer(ros::Duration(0.05), [this](const ros::TimerEvent&) {
                best_confidence_ = 0.0f;
            }, true);
        } else {
            reset_timer_.setPeriod(ros::Duration(0.05));
            reset_timer_.start();
        }

        int u = msg->x + msg->width / 2;
        int v = msg->y + msg->height / 2;

        if (u < 0 || u >= depth_image_.cols || v < 0 || v >= depth_image_.rows) return;

        // --- 5x5 中值滤波 ---
        std::vector<uint16_t> valid_depths;
        valid_depths.reserve(25);
        for (int dy = -2; dy <= 2; ++dy) {
            for (int dx = -2; dx <= 2; ++dx) {
                int su = u + dx;
                int sv = v + dy;
                if (su < 0 || su >= depth_image_.cols || sv < 0 || sv >= depth_image_.rows) continue;
                uint16_t d = depth_image_.at<uint16_t>(sv, su);
                if (d > 0 && d <= 10000) valid_depths.push_back(d);
            }
        }
        if (valid_depths.empty()) return;
        std::nth_element(valid_depths.begin(), valid_depths.begin() + valid_depths.size() / 2, valid_depths.end());
        uint16_t depth_mm = valid_depths[valid_depths.size() / 2];

        double z = depth_mm * 0.001;  // mm -> m
        double x = (u - cx_) * z / fx_;
        double y = (v - cy_) * z / fy_;

        // --- EMA 平滑 ---
        if (first_pose_) {
            smooth_x_ = x;
            smooth_y_ = y;
            smooth_z_ = z;
            first_pose_ = false;
        } else {
            smooth_x_ = smooth_alpha_ * x + (1.0 - smooth_alpha_) * smooth_x_;
            smooth_y_ = smooth_alpha_ * y + (1.0 - smooth_alpha_) * smooth_y_;
            smooth_z_ = smooth_alpha_ * z + (1.0 - smooth_alpha_) * smooth_z_;
        }

        cached_pose_.header.frame_id = "paw_camera_color_optical_frame";
        cached_pose_.pose.position.x = smooth_x_;
        cached_pose_.pose.position.y = smooth_y_;
        cached_pose_.pose.position.z = smooth_z_;
        cached_pose_.pose.orientation.x = 0.0;
        cached_pose_.pose.orientation.y = 0.0;
        cached_pose_.pose.orientation.z = 0.0;
        cached_pose_.pose.orientation.w = 1.0;
        has_pose_ = true;
    }

    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    ros::Subscriber det_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber info_sub_;
    ros::Timer reset_timer_;

    int publish_rate_;
    bool has_camera_info_;
    bool has_depth_;
    bool has_pose_;
    bool first_pose_;

    double fx_, fy_, cx_, cy_;
    double smooth_alpha_;
    double smooth_x_, smooth_y_, smooth_z_;
    float best_confidence_ = 0.0f;
    cv::Mat depth_image_;
    geometry_msgs::PoseStamped cached_pose_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_pose_node");
    ObjectPoseNode node;
    node.run();
    return 0;
}
