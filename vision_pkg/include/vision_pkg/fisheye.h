#ifndef VISION_PKG_FISHEYE_H
#define VISION_PKG_FISHEYE_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <functional>

/**
 * @brief 鱼眼相机畸变校正与全景图转换类
 *
 * 将鱼眼相机拍摄的圆形图像转换为等距柱状投影（equirectangular）全景图。
 * 内置了特定鱼眼相机的内参矩阵 K 和畸变系数 D。
 */
class Fisheye
{
public:
    // 双帧全景就绪回调：两路鱼眼分别转换后的全景图
    using FramesReadyCallback = std::function<void(const cv::Mat&, const cv::Mat&)>;

    Fisheye();

    void setFramesReadyCallback(FramesReadyCallback cb) { m_callback = std::move(cb); }

    /**
     * @brief 处理前后两路鱼眼帧，分别转换为全景图后通过回调输出
     * @param frame1 前方鱼眼相机帧
     * @param frame2 后方鱼眼相机帧
     */
    void processFrames(const cv::Mat &frame1, const cv::Mat &frame2);

private:
    /**
     * @brief 计算鱼眼图到全景图的像素映射表
     *
     * 基于球面坐标系，将全景图每个像素 (x,y) 映射回鱼眼图中的源坐标 (u,v)。
     *
     * @param fisheye_image  鱼眼原图（用于获取尺寸）
     * @param panorama_image 全景目标图（用于获取尺寸）
     * @param map1           X 方向映射表（输出）
     * @param map2           Y 方向映射表（输出）
     * @param start_row      处理起始行
     * @param end_row        处理结束行
     * @param radius         鱼眼有效区域半径
     * @param center         鱼眼圆心坐标
     */
    void compute_mapping(const cv::Mat &fisheye_image, const cv::Mat &panorama_image,
                         cv::Mat &map1, cv::Mat &map2,
                         int start_row, int end_row, int radius, cv::Point2f center);

    /**
     * @brief 裁剪鱼眼图像的有效圆形区域
     * @param img           输入图像
     * @param circle_radius 裁剪半径（像素）
     * @return 裁剪后的正方形图像
     */
    cv::Mat crop_and_replace_images(const cv::Mat &img, int circle_radius);

    /**
     * @brief 将裁剪后的鱼眼图转换为等距柱状投影全景图
     * @param fisheye_image 裁剪后的鱼眼图
     * @param K             相机内参矩阵（当前未直接使用，预留）
     * @param D             畸变系数（当前未直接使用，预留）
     * @return 全景图，尺寸为 2R x 4R（R=鱼眼半径），下半部分置黑
     */
    cv::Mat fisheye_to_panorama(const cv::Mat &fisheye_image, const cv::Mat &K, const cv::Mat &D);

    /**
     * @brief 单帧处理核心流程：裁剪 -> 鱼眼转全景
     * @param frame      原始鱼眼帧
     * @param K          相机内参
     * @param D          畸变系数
     * @param frame_name 帧名称（用于调试日志）
     * @return 转换后的全景图
     */
    cv::Mat core(const cv::Mat &frame, const cv::Mat &K, const cv::Mat &D, const std::string &frame_name);

    cv::Mat m_K, m_D;              // 相机内参矩阵和畸变系数
    FramesReadyCallback m_callback;
};

#endif // VISION_PKG_FISHEYE_H
