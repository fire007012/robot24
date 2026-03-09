#ifndef VISION_PKG_CAMERAINIT_H
#define VISION_PKG_CAMERAINIT_H

#include <string>
#include <atomic>
#include <functional>
#include <opencv2/opencv.hpp>

class CameraInit
{
public:
    using FrameCallback = std::function<void(const cv::Mat&)>;

    explicit CameraInit(const std::string& devicePath);
    ~CameraInit();

    bool initAndOpenCamera();
    void closeCamera();
    bool isCameraOpen();

    // Blocking capture loop — call from a dedicated thread
    void captureLoop();

    void setFrameCallback(FrameCallback cb) { m_callback = std::move(cb); }

private:
    std::string m_devicePath;
    cv::VideoCapture m_cap;
    std::atomic<bool> m_isCapturing{false};
    FrameCallback m_callback;
};

#endif // VISION_PKG_CAMERAINIT_H
