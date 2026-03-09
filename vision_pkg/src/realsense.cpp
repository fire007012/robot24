#include "vision_pkg/realsense.h"
#include <ros/ros.h>

Realsense::Realsense(const std::string &deviceSerial, int deviceIndex)
    : m_deviceSerial(deviceSerial), m_deviceIndex(deviceIndex)
{
}

Realsense::~Realsense()
{
    closeCamera();
    delete m_pipe;
    delete m_context;
    delete m_device;
}

bool Realsense::initAndOpenCamera()
{
    if (m_isCapturing) {
        ROS_WARN("Camera with serial %s is already capturing", m_deviceSerial.c_str());
        return false;
    }

    m_context = new rs2::context();
    rs2::device_list devices = m_context->query_devices();

    if (devices.size() == 0) {
        ROS_ERROR("No RealSense devices found");
        delete m_context; m_context = nullptr;
        return false;
    }

    bool deviceFound = false;
    if (!m_deviceSerial.empty()) {
        for (auto&& dev : devices) {
            if (dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == m_deviceSerial) {
                m_device = new rs2::device(dev);
                deviceFound = true;
                break;
            }
        }
    } else if (m_deviceIndex >= 0 && m_deviceIndex < (int)devices.size()) {
        m_device = new rs2::device(devices[m_deviceIndex]);
        m_deviceSerial = m_device->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        deviceFound = true;
        ROS_INFO("Auto-selected RealSense device[%d]: %s", m_deviceIndex, m_deviceSerial.c_str());
    }

    if (!deviceFound) {
        ROS_ERROR("RealSense device with serial %s not found", m_deviceSerial.c_str());
        delete m_context; m_context = nullptr;
        return false;
    }

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 1280, 720, RS2_FORMAT_Y8, 30);
    cfg.enable_device(m_deviceSerial);
    m_pipe = new rs2::pipeline(*m_context);

    try {
        m_profile = m_pipe->start(cfg);
    } catch (const rs2::error& e) {
        ROS_ERROR("Failed to start pipeline for serial %s: %s", m_deviceSerial.c_str(), e.what());
        delete m_pipe; m_pipe = nullptr;
        delete m_device; m_device = nullptr;
        delete m_context; m_context = nullptr;
        return false;
    }

    m_isCapturing = true;
    ROS_INFO("Camera with serial %s started successfully", m_deviceSerial.c_str());
    return true;
}

void Realsense::closeCamera()
{
    if (!m_isCapturing) return;
    m_isCapturing = false;
    if (m_pipe) {
        try {
            m_pipe->stop();
        } catch (const rs2::error& e) {
            ROS_ERROR("Error stopping pipeline for serial %s: %s", m_deviceSerial.c_str(), e.what());
        }
    }
}

void Realsense::captureLoop()
{
    ros::Rate rate(30);
    while (m_isCapturing && ros::ok()) {
        rs2::frameset frames = m_pipe->wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();
        cv::Mat cv_color_image(
            cv::Size(color_frame.as<rs2::video_frame>().get_width(),
                     color_frame.as<rs2::video_frame>().get_height()),
            CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        if (m_callback) m_callback(cv_color_image.clone());
        rate.sleep();
    }
}
