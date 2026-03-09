#ifndef VISION_PKG_FISHEYE_H
#define VISION_PKG_FISHEYE_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <functional>

class Fisheye
{
public:
    using FramesReadyCallback = std::function<void(const cv::Mat&, const cv::Mat&)>;

    Fisheye();

    void setFramesReadyCallback(FramesReadyCallback cb) { m_callback = std::move(cb); }

    void processFrames(const cv::Mat &frame1, const cv::Mat &frame2);

private:
    void compute_mapping(const cv::Mat &fisheye_image, const cv::Mat &panorama_image,
                         cv::Mat &map1, cv::Mat &map2,
                         int start_row, int end_row, int radius, cv::Point2f center);
    cv::Mat crop_and_replace_images(const cv::Mat &img, int circle_radius);
    cv::Mat fisheye_to_panorama(const cv::Mat &fisheye_image, const cv::Mat &K, const cv::Mat &D);
    cv::Mat core(const cv::Mat &frame, const cv::Mat &K, const cv::Mat &D, const std::string &frame_name);

    cv::Mat m_K, m_D;
    FramesReadyCallback m_callback;
};

#endif // VISION_PKG_FISHEYE_H
