#include "vision_pkg/panorama.h"
#include <ros/ros.h>

Panorama::Panorama()
{
    fisheye.setFramesReadyCallback(
        [this](const cv::Mat &p1, const cv::Mat &p2) { onFisheyeReady(p1, p2); });

    simulator.setPanoramaReadyCallback(
        [this](const cv::Mat &img) { if (m_callback) m_callback(img); });

    simulator.initialize();
}

Panorama::~Panorama()
{
}

void Panorama::receiveCam1(const cv::Mat &frame)
{
    if (frame.empty()) return;

    cv::Mat bgr;
    if (frame.channels() == 3) {
        bgr = frame.clone();
    } else {
        cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
    }

    std::lock_guard<std::mutex> lock(frameMutex);
    frame1 = bgr;
    if (!frame2.empty()) {
        fisheye.processFrames(frame1, frame2);
    }
}

void Panorama::receiveCam2(const cv::Mat &frame)
{
    if (frame.empty()) return;

    cv::Mat bgr;
    if (frame.channels() == 3) {
        bgr = frame.clone();
    } else {
        cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
    }
    cv::rotate(bgr, bgr, cv::ROTATE_180);

    std::lock_guard<std::mutex> lock(frameMutex);
    frame2 = bgr;
    if (!frame1.empty()) {
        fisheye.processFrames(frame1, frame2);
    }
}

void Panorama::handleControllerState(float thumbRX, float thumbRY, bool rightThumb)
{
    float rawX = (thumbRX + 32767.0f) / 65534.0f;
    float rawY = (thumbRY + 32767.0f) / 65534.0f;
    resetPerspective = rightThumb;

    float dx = (rawX - 0.5f) * 2.0f;
    float dy = (rawY - 0.5f) * 2.0f;

    if (std::abs(dx) < deadZone && std::abs(dy) < deadZone) {
        x = 0.5f;
        y = 0.5f;
    } else {
        x = rawX;
        y = rawY;
    }

    if (resetPerspective) {
        simulator.setPerspective();
    } else {
        simulator.setPerspective(x, y);
    }
}

void Panorama::onFisheyeReady(const cv::Mat &panorama1, const cv::Mat &panorama2)
{
    if (!panorama1.empty() && !panorama2.empty()) {
        simulator.processFrames(panorama1, panorama2);
    } else {
        ROS_ERROR("Panorama: Received empty fisheye output");
    }
}
