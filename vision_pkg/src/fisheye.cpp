#include "vision_pkg/fisheye.h"
#include <ros/ros.h>

Fisheye::Fisheye()
{
    m_K = (cv::Mat_<double>(3, 3) << 454.89445666, 0, 551.30048588,
           0, 455.67670656, 485.74532474,
           0, 0, 1);
    m_D = (cv::Mat_<double>(1, 5) << -0.285545645, 0.0627139372, 0.00739321249, 0.000150928223, -0.00542736549);
}

void Fisheye::processFrames(const cv::Mat &frame1, const cv::Mat &frame2)
{
    cv::Mat panorama1 = core(frame1, m_K, m_D, "frame1");
    cv::Mat panorama2 = core(frame2, m_K, m_D, "frame2");

    if (!panorama1.empty() && !panorama2.empty()) {
        if (m_callback) m_callback(panorama1, panorama2);
    } else {
        ROS_ERROR("Fisheye: One or both panoramas are empty");
    }
}

void Fisheye::compute_mapping(const cv::Mat &fisheye_image, const cv::Mat &panorama_image,
                               cv::Mat &map1, cv::Mat &map2,
                               int start_row, int end_row, int radius, cv::Point2f center)
{
    const double PI = acos(-1);
    const double PI2 = 2 * PI;

    for (int y = start_row; y < end_row; ++y) {
        if (y >= panorama_image.rows) continue;
        double theta = static_cast<double>(y) / panorama_image.rows * PI;
        double sin_theta = sin(theta);
        double cos_theta = cos(theta);

        for (int x = 0; x < panorama_image.cols; ++x) {
            double phi = static_cast<double>(x) / panorama_image.cols * PI2;
            double sin_phi = sin(phi);
            double cos_phi = cos(phi);

            double r = radius * acos(cos_theta) / (PI / 2);
            r = std::min(r, static_cast<double>(radius));

            double u = center.x + r * -cos_phi;
            double v = center.y + r * sin_phi;

            map1.at<float>(y, x) = static_cast<float>(u);
            map2.at<float>(y, x) = static_cast<float>(v);
        }
    }
}

cv::Mat Fisheye::crop_and_replace_images(const cv::Mat &img, int circle_radius)
{
    int img_height = img.rows;
    int img_width = img.cols;
    int center_x = img_width / 2;
    int center_y = img_height / 2;

    int x1 = std::max(center_x - circle_radius, 0);
    int y1 = std::max(center_y - circle_radius, 0);
    int x2 = std::min(center_x + circle_radius, img_width);
    int y2 = std::min(center_y + circle_radius, img_height);

    int width = x2 - x1;
    int height = y2 - y1;

    if (width != height) {
        int new_size = std::min(width, height);
        int delta_width = (width - new_size) / 2;
        int delta_height = (height - new_size) / 2;
        x1 += delta_width;
        y1 += delta_height;
        width = height = new_size;
    }

    return img(cv::Rect(x1, y1, width, height));
}

cv::Mat Fisheye::fisheye_to_panorama(const cv::Mat &fisheye_image, const cv::Mat &K, const cv::Mat &D)
{
    int h = fisheye_image.rows;
    int w = fisheye_image.cols;
    int radius = std::min(h, w) / 2;

    cv::Mat panorama_image = cv::Mat::zeros(2 * radius, 4 * radius, CV_8UC3);
    cv::Mat map1 = cv::Mat::zeros(2 * radius, 4 * radius, CV_32FC1);
    cv::Mat map2 = cv::Mat::zeros(2 * radius, 4 * radius, CV_32FC1);

    compute_mapping(fisheye_image, panorama_image, map1, map2, 0, 2 * radius, radius,
                    cv::Point2f(w / 2.0f, h / 2.0f));

    cv::remap(fisheye_image, panorama_image, map1, map2, cv::INTER_NEAREST, cv::BORDER_CONSTANT);

    cv::Mat lower_half = panorama_image(cv::Rect(0, radius, 4 * radius, radius));
    cv::Mat black_image = cv::Mat::zeros(lower_half.size(), panorama_image.type());
    black_image.copyTo(lower_half);

    return panorama_image;
}

cv::Mat Fisheye::core(const cv::Mat &frame, const cv::Mat &K, const cv::Mat &D, const std::string &frame_name)
{
    if (frame.empty()) {
        ROS_ERROR("Fisheye::core: Frame is empty!");
        return cv::Mat();
    }
    if (K.empty() || D.empty()) {
        ROS_ERROR("Fisheye::core: Camera parameters are missing!");
        return cv::Mat();
    }

    cv::Mat cropped_frame = crop_and_replace_images(frame, 540);
    cv::Mat panorama_image = fisheye_to_panorama(cropped_frame, K, D);
    return panorama_image;
}
