#include "vision_pkg/camerainit.h"
#include <ros/ros.h>

CameraInit::CameraInit(const std::string& devicePath)
    : m_devicePath(devicePath), m_isCapturing(false)
{
}

CameraInit::~CameraInit()
{
    closeCamera();
}

void CameraInit::closeCamera()
{
    m_isCapturing = false;
    if (m_cap.isOpened()) {
        m_cap.release();
    }
}

bool CameraInit::isCameraOpen()
{
    return m_cap.isOpened();
}

void CameraInit::captureLoop()
{
    ros::Rate rate(30);
    while (m_isCapturing && ros::ok()) {
        cv::Mat frame;
        m_cap >> frame;
        if (!frame.empty()) {
            if (m_callback) m_callback(frame);
        }
        rate.sleep();
    }
}

bool CameraInit::initAndOpenCamera()
{
    m_cap.open(m_devicePath, cv::CAP_V4L2);
    if (m_cap.isOpened()) {
        m_isCapturing = true;
        return true;
    }
    return false;
}
