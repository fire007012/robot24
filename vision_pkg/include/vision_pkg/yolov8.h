#ifndef VISION_PKG_YOLOV8_H
#define VISION_PKG_YOLOV8_H

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <string>
#include <vector>
#include "yolov8_utils.h"

class Yolov8
{
public:
    Yolov8();
    ~Yolov8();

    bool ReadModel(cv::dnn::Net& net, const std::string& netPath, bool isCuda = false);
    bool detect(cv::Mat& srcImg, cv::dnn::Net& net, std::vector<OutputParams>& output);
    cv::Mat drawPred(cv::Mat& img, const std::vector<OutputParams>& result,
                     const std::vector<std::string>& classNames,
                     const std::vector<cv::Scalar>& color);

    std::vector<std::string> className = {
        "FLAMMABLEGAS", "FUELOIL", "ORGANICPEROXIDE", "OXIDIZER",
        "DANGEROUS", "FLAMMABLESOLID", "EXPLOSIVES", "OXYGEN",
        "POISON", "NON-FLAMMABLE GAS", "COMBUSTIBLE", "INHALATION HAZARD",
        "RADIOACTIVE", "BLASTING AGENTS", "FLAMMABLE SOLID",
        "FLAMMABLE GAS", "CORROSIVE"
    };

private:
    void letterBox(const cv::Mat& image, cv::Mat& outImage, cv::Vec4d& params,
                   const cv::Size& newShape = cv::Size(640, 640),
                   bool autoShape = false, bool scaleFill = false,
                   bool scaleUp = true, int stride = 32,
                   const cv::Scalar& color = cv::Scalar(114, 114, 114));

    int _netWidth = 640;
    int _netHeight = 640;
    float _classThreshold = 0.25;
    float _nmsThreshold = 0.45;
};

#endif // VISION_PKG_YOLOV8_H
