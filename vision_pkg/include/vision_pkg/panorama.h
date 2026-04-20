#ifndef VISION_PKG_PANORAMA_H
#define VISION_PKG_PANORAMA_H

#include <opencv2/opencv.hpp>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include "vision_pkg/panoramasimulator.h"
#include "vision_pkg/fisheye.h"

/**
 * @brief 全景处理管线：鱼眼校正 -> 球面渲染 -> 视角控制
 *
 * 接收前后两路鱼眼相机帧，经 Fisheye 校正转为全景图后，
 * 送入 PanoramaSimulator 进行球面贴图渲染。
 * 支持手柄右摇杆交互控制观察视角。
 */
class Panorama
{
public:
    using PanoramaReadyCallback = std::function<void(const cv::Mat&)>;

    Panorama();
    ~Panorama();

    /** @brief 接收前方相机帧，与后方帧配对后触发全景处理 */
    void receiveCam1(const cv::Mat &frame);
    /** @brief 接收后方相机帧（会旋转180°），与前方帧配对后触发全景处理 */
    void receiveCam2(const cv::Mat &frame);

    /**
     * @brief 处理手柄控制器输入，调整全景视角
     * @param thumbRX    右摇杆 X 轴原始值 [-32767, 32767]
     * @param thumbRY    右摇杆 Y 轴原始值 [-32767, 32767]
     * @param rightThumb 右摇杆按下状态（true 时重置视角）
     */
    void handleControllerState(float thumbRX, float thumbRY, bool rightThumb);

    void setPanoramaReadyCallback(PanoramaReadyCallback cb) { m_callback = std::move(cb); }

private:
    /** @brief Fisheye 校正完成后的回调，将全景图送入球面渲染器 */
    void onFisheyeReady(const cv::Mat &panorama1, const cv::Mat &panorama2);

    PanoramaSimulator simulator;  // 球面渲染器
    Fisheye fisheye;              // 鱼眼校正器

    cv::Mat frame1, frame2;       // 缓存的前后相机帧
    std::mutex frameMutex;        // 帧缓冲互斥锁

    float x = 0.5f;              // 归一化摇杆 X 值 [0,1]
    float y = 0.5f;              // 归一化摇杆 Y 值 [0,1]
    bool resetPerspective = false; // 是否重置视角
    const float deadZone = 0.1f;  // 摇杆死区阈值

    PanoramaReadyCallback m_callback;
};

#endif // VISION_PKG_PANORAMA_H
