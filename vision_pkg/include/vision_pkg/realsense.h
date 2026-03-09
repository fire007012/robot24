#ifndef VISION_PKG_REALSENSE_H
#define VISION_PKG_REALSENSE_H

#include <string>
#include <atomic>
#include <functional>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

class Realsense
{
public:
    using FrameCallback = std::function<void(const cv::Mat&)>;

    Realsense(const std::string &deviceSerial, int deviceIndex = -1);
    ~Realsense();

    bool initAndOpenCamera();
    void closeCamera();

    // Blocking capture loop — call from a dedicated thread
    void captureLoop();

    void setFrameCallback(FrameCallback cb) { m_callback = std::move(cb); }

private:
    std::string m_deviceSerial;
    int m_deviceIndex;
    std::atomic<bool> m_isCapturing{false};

    rs2::context *m_context = nullptr;
    rs2::pipeline *m_pipe = nullptr;
    rs2::device *m_device = nullptr;
    rs2::pipeline_profile m_profile;

    FrameCallback m_callback;
};

#endif // VISION_PKG_REALSENSE_H
