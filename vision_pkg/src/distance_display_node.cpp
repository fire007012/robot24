#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc.hpp>

class DistanceDisplayNode
{
public:
    DistanceDisplayNode() : nh_("~"), it_(nh_), has_depth_(false)
    {
        std::string color_topic, depth_topic;
        nh_.param<std::string>("color_topic", color_topic, "/paw_camera/color/image_raw");
        nh_.param<std::string>("depth_topic", depth_topic, "/paw_camera/depth/image_rect_raw");
        nh_.param<double>("distance_threshold", distance_threshold_, 0.1);
        nh_.param<int>("roi_size", roi_size_, 30);

        image_pub_ = it_.advertise("distance_image", 1);

        color_sub_ = nh_.subscribe(color_topic, 1, &DistanceDisplayNode::colorCallback, this);
        depth_sub_ = nh_.subscribe(depth_topic, 1, &DistanceDisplayNode::depthCallback, this);

        ROS_INFO("distance_display_node started");
        ROS_INFO("  color_topic: %s", color_topic.c_str());
        ROS_INFO("  depth_topic: %s", depth_topic.c_str());
        ROS_INFO("  distance_threshold: %.2f m", distance_threshold_);
        ROS_INFO("  roi_size: %d", roi_size_);
    }

private:
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        try {
            depth_ptr_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            has_depth_ = true;
        } catch (cv_bridge::Exception& e) {
            ROS_WARN_THROTTLE(5.0, "depth cv_bridge error: %s", e.what());
        }
    }

    void colorCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        if (!has_depth_) return;

        cv_bridge::CvImagePtr color_ptr;
        try {
            color_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_WARN_THROTTLE(5.0, "color cv_bridge error: %s", e.what());
            return;
        }

        cv::Mat& color = color_ptr->image;
        const cv::Mat& depth = depth_ptr_->image;

        int cx = depth.cols / 2;
        int cy = depth.rows / 2;
        int half = roi_size_ / 2;

        // 计算中心区域有效深度均值
        int count = 0;
        double sum = 0.0;
        for (int y = cy - half; y <= cy + half; ++y) {
            for (int x = cx - half; x <= cx + half; ++x) {
                if (x < 0 || x >= depth.cols || y < 0 || y >= depth.rows) continue;
                uint16_t d = depth.at<uint16_t>(y, x);
                if (d > 0 && d <= 10000) {
                    sum += d;
                    ++count;
                }
            }
        }

        // 画细白线十字准星
        int ccx = color.cols / 2;
        int ccy = color.rows / 2;
        int cross_len = 20;
        int gap = 5;
        cv::Scalar cross_color(255, 255, 255);
        int line_thickness = 1;

        // 上下左右四条线，中间留空隙
        cv::line(color, cv::Point(ccx - cross_len, ccy), cv::Point(ccx - gap, ccy), cross_color, line_thickness);
        cv::line(color, cv::Point(ccx + gap, ccy), cv::Point(ccx + cross_len, ccy), cross_color, line_thickness);
        cv::line(color, cv::Point(ccx, ccy - cross_len), cv::Point(ccx, ccy - gap), cross_color, line_thickness);
        cv::line(color, cv::Point(ccx, ccy + gap), cv::Point(ccx, ccy + cross_len), cross_color, line_thickness);

        // 距离超过阈值时显示
        if (count > 0) {
            double distance_m = (sum / count) * 0.001;
            if (distance_m > distance_threshold_) {
                char text[64];
                snprintf(text, sizeof(text), "%.2f m", distance_m);

                int font = cv::FONT_HERSHEY_SIMPLEX;
                double font_scale = 0.7;
                int thickness = 1;
                int baseline = 0;
                cv::Size text_size = cv::getTextSize(text, font, font_scale, thickness, &baseline);

                cv::Point text_org(ccx - text_size.width / 2, ccy - 30);

                // 半透明黑色背景
                cv::Mat roi = color(cv::Rect(text_org.x - 5, text_org.y - text_size.height - 3,
                                              text_size.width + 10, text_size.height + baseline + 6));
                cv::Mat overlay = roi.clone();
                cv::rectangle(overlay, cv::Point(0, 0), cv::Point(overlay.cols, overlay.rows),
                              cv::Scalar(0, 0, 0), cv::FILLED);
                cv::addWeighted(roi, 0.5, overlay, 0.5, 0, roi);

                cv::putText(color, text, text_org, font, font_scale, cv::Scalar(255, 255, 255), thickness);
            }
        }

        image_pub_.publish(color_ptr->toImageMsg());
    }

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    ros::Subscriber color_sub_;
    ros::Subscriber depth_sub_;

    cv_bridge::CvImageConstPtr depth_ptr_;
    bool has_depth_;

    double distance_threshold_;
    int roi_size_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "distance_display_node");
    DistanceDisplayNode node;
    ros::spin();
    return 0;
}
