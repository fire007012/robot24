#ifndef VISION_PKG_YOLOV8_UTILS_H
#define VISION_PKG_YOLOV8_UTILS_H

#include <opencv2/opencv.hpp>

struct OutputParams {
    int id;
    float confidence;
    cv::Rect box;
    cv::Mat boxMask;
    cv::RotatedRect rotatedBox;
};

inline void DrawRotatedBox(cv::Mat& img, const cv::RotatedRect& box, const cv::Scalar& color, int thickness) {
    cv::Point2f vertices[4];
    box.points(vertices);
    for (int i = 0; i < 4; i++) {
        cv::line(img, vertices[i], vertices[(i + 1) % 4], color, thickness);
    }
}

#endif // VISION_PKG_YOLOV8_UTILS_H
