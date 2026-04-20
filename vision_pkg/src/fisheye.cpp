#include "vision_pkg/fisheye.h"
#include <ros/ros.h>

/**
 * @brief 构造函数：初始化鱼眼相机内参和畸变系数
 *
 * K - 3x3 相机内参矩阵 [fx, 0, cx; 0, fy, cy; 0, 0, 1]
 * D - 1x5 畸变系数 [k1, k2, p1, p2, k3]
 * 这些参数是针对特定鱼眼相机标定得到的固定值。
 */
Fisheye::Fisheye()
{
    m_K = (cv::Mat_<double>(3, 3) << 454.89445666, 0, 551.30048588,
           0, 455.67670656, 485.74532474,
           0, 0, 1);
    m_D = (cv::Mat_<double>(1, 5) << -0.285545645, 0.0627139372, 0.00739321249, 0.000150928223, -0.00542736549);
}

/**
 * @brief 处理前后两路鱼眼帧
 *
 * 分别对两帧执行鱼眼校正和全景转换，
 * 两路全景图都就绪后通过回调输出。
 */
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

/**
 * @brief 计算鱼眼图到等距柱状投影全景图的像素映射表
 *
 * 对全景图中每个像素 (x, y)，计算其对应的球面坐标 (theta, phi)，
 * 然后映射回鱼眼图中的源像素坐标 (u, v)。
 *
 * 映射原理：
 *   theta = y / rows * PI        （天顶角，0=北极，PI=南极）
 *   phi   = x / cols * 2*PI      （方位角，0~2PI 一圈）
 *   r     = radius * theta / (PI/2)  （鱼眼图中的径向距离）
 *   u     = center.x + r * (-cos(phi))
 *   v     = center.y + r * sin(phi)
 */
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

            // 天顶角映射到鱼眼径向距离
            double r = radius * acos(cos_theta) / (PI / 2);
            r = std::min(r, static_cast<double>(radius));

            // 极坐标转笛卡尔坐标，得到鱼眼图中的源像素位置
            double u = center.x + r * -cos_phi;
            double v = center.y + r * sin_phi;

            map1.at<float>(y, x) = static_cast<float>(u);
            map2.at<float>(y, x) = static_cast<float>(v);
        }
    }
}

/**
 * @brief 裁剪鱼眼图像的有效圆形区域为正方形
 *
 * 以图像中心为圆心，按指定半径裁剪出正方形区域。
 * 如果裁剪区域不是正方形，会居中调整为正方形。
 *
 * @param img           输入鱼眼图像
 * @param circle_radius 裁剪半径（像素），默认使用 540
 * @return 裁剪后的正方形图像（ROI引用，非深拷贝）
 */
cv::Mat Fisheye::crop_and_replace_images(const cv::Mat &img, int circle_radius)
{
    int img_height = img.rows;
    int img_width = img.cols;
    int center_x = img_width / 2;
    int center_y = img_height / 2;

    // 计算裁剪边界
    int x1 = std::max(center_x - circle_radius, 0);
    int y1 = std::max(center_y - circle_radius, 0);
    int x2 = std::min(center_x + circle_radius, img_width);
    int y2 = std::min(center_y + circle_radius, img_height);

    int width = x2 - x1;
    int height = y2 - y1;

    // 确保裁剪结果为正方形
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

/**
 * @brief 将裁剪后的鱼眼图转换为等距柱状投影全景图
 *
 * 输出全景图尺寸为 2R x 4R（R = 鱼眼半径）。
 * 下半部分（南半球）置为黑色，因为单个鱼眼只覆盖上半球。
 *
 * @param fisheye_image 裁剪后的正方形鱼眼图
 * @param K             相机内参（预留参数，当前映射使用几何方法）
 * @param D             畸变系数（预留参数）
 * @return 等距柱状投影全景图
 */
cv::Mat Fisheye::fisheye_to_panorama(const cv::Mat &fisheye_image, const cv::Mat &K, const cv::Mat &D)
{
    int h = fisheye_image.rows;
    int w = fisheye_image.cols;
    int radius = std::min(h, w) / 2;

    // 创建全景图和映射表，尺寸 2R x 4R
    cv::Mat panorama_image = cv::Mat::zeros(2 * radius, 4 * radius, CV_8UC3);
    cv::Mat map1 = cv::Mat::zeros(2 * radius, 4 * radius, CV_32FC1);
    cv::Mat map2 = cv::Mat::zeros(2 * radius, 4 * radius, CV_32FC1);

    // 计算完整映射表
    compute_mapping(fisheye_image, panorama_image, map1, map2, 0, 2 * radius, radius,
                    cv::Point2f(w / 2.0f, h / 2.0f));

    // 使用最近邻插值执行重映射
    cv::remap(fisheye_image, panorama_image, map1, map2, cv::INTER_NEAREST, cv::BORDER_CONSTANT);

    // 将下半部分置黑（单个鱼眼只覆盖上半球）
    cv::Mat lower_half = panorama_image(cv::Rect(0, radius, 4 * radius, radius));
    cv::Mat black_image = cv::Mat::zeros(lower_half.size(), panorama_image.type());
    black_image.copyTo(lower_half);

    return panorama_image;
}

/**
 * @brief 单帧处理核心流程
 *
 * 1. 以半径 540px 裁剪鱼眼有效区域
 * 2. 将裁剪后的鱼眼图转换为等距柱状投影全景图
 */
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
