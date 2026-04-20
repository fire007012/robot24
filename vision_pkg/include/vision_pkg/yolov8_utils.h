#ifndef VISION_PKG_YOLOV8_UTILS_H
#define VISION_PKG_YOLOV8_UTILS_H

#include <opencv2/opencv.hpp>

/**
 * @brief YOLOv8 单个检测结果的输出参数
 */
struct OutputParams {
    int id;                      // 类别ID，对应 className 数组的索引
    float confidence;            // 检测置信度 [0, 1]
    cv::Rect box;                // 水平检测框 (x, y, width, height)
    cv::Mat boxMask;             // 实例分割掩码（仅分割模型使用）
    cv::RotatedRect rotatedBox;  // 旋转检测框（仅OBB模型使用）
};

/**
 * @brief 在图像上绘制旋转矩形框
 * @param img       目标图像
 * @param box       旋转矩形
 * @param color     绘制颜色
 * @param thickness 线条粗细
 */
inline void DrawRotatedBox(cv::Mat& img, const cv::RotatedRect& box, const cv::Scalar& color, int thickness) {
    cv::Point2f vertices[4];
    box.points(vertices);  // 获取旋转矩形的四个顶点
    for (int i = 0; i < 4; i++) {
        cv::line(img, vertices[i], vertices[(i + 1) % 4], color, thickness);
    }
}

#endif // VISION_PKG_YOLOV8_UTILS_H
