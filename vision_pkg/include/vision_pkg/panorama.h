#ifndef VISION_PKG_PANORAMA_H
#define VISION_PKG_PANORAMA_H

#include <opencv2/opencv.hpp>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include "vision_pkg/panoramasimulator.h"
#include "vision_pkg/fisheye.h"

class Panorama
{
public:
    using PanoramaReadyCallback = std::function<void(const cv::Mat&)>;

    Panorama();
    ~Panorama();

    void receiveCam1(const cv::Mat &frame);
    void receiveCam2(const cv::Mat &frame);
    void handleControllerState(float thumbRX, float thumbRY, bool rightThumb);
    void setPanoramaReadyCallback(PanoramaReadyCallback cb) { m_callback = std::move(cb); }

private:
    void onFisheyeReady(const cv::Mat &panorama1, const cv::Mat &panorama2);

    PanoramaSimulator simulator;
    Fisheye fisheye;

    cv::Mat frame1, frame2;
    std::mutex frameMutex;

    float x = 0.5f;
    float y = 0.5f;
    bool resetPerspective = false;
    const float deadZone = 0.1f;

    PanoramaReadyCallback m_callback;
};

#endif // VISION_PKG_PANORAMA_H
