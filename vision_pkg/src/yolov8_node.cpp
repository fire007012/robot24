#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "vision_pkg/Detection.h"
#include "vision_pkg/yolov8.h"

class Yolov8Node
{
public:
    Yolov8Node() : nh_("~"), it_(nh_)
    {
        std::string model_path, image_topic;
        bool use_cuda;
        nh_.param<std::string>("model_path", model_path, "model.onnx");
        nh_.param<std::string>("image_topic", image_topic, "/camera/forward/image_raw");
        nh_.param<bool>("use_cuda", use_cuda, false);

        if (!yolo_.ReadModel(net_, model_path, use_cuda)) {
            ROS_ERROR("Failed to load YOLO model: %s", model_path.c_str());
            return;
        }

        det_pub_ = nh_.advertise<vision_pkg::Detection>("detections", 10);
        img_pub_ = it_.advertise("detection_image", 1);
        sub_ = it_.subscribe(image_topic, 1, &Yolov8Node::imageCallback, this);

        // Generate random colors for each class
        for (size_t i = 0; i < yolo_.className.size(); i++) {
            colors_.push_back(cv::Scalar(rand() % 256, rand() % 256, rand() % 256));
        }

        ROS_INFO("yolov8_node started, subscribing to %s", image_topic.c_str());
    }

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        std::vector<OutputParams> results;

        if (yolo_.detect(frame, net_, results)) {
            for (const auto& r : results) {
                vision_pkg::Detection det;
                det.id = r.id;
                det.confidence = r.confidence;
                det.x = r.box.x;
                det.y = r.box.y;
                det.width = r.box.width;
                det.height = r.box.height;
                if (r.id >= 0 && r.id < (int)yolo_.className.size())
                    det.class_name = yolo_.className[r.id];
                det_pub_.publish(det);
            }

            cv::Mat drawn = yolo_.drawPred(frame, results, yolo_.className, colors_);
            auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", drawn).toImageMsg();
            img_pub_.publish(out_msg);
        }
    }

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher img_pub_;
    image_transport::Subscriber sub_;
    ros::Publisher det_pub_;
    Yolov8 yolo_;
    cv::dnn::Net net_;
    std::vector<cv::Scalar> colors_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "yolov8_node");
    Yolov8Node node;
    ros::spin();
    return 0;
}
