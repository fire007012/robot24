#include "vision_pkg/yolov8.h"
#include <ros/ros.h>

Yolov8::Yolov8() {}
Yolov8::~Yolov8() = default;

bool Yolov8::ReadModel(cv::dnn::Net& net, const std::string& netPath, bool isCuda) {
    try {
        net = cv::dnn::readNetFromONNX(netPath);
#if (CV_VERSION_MAJOR*100 + CV_VERSION_MINOR) >= 406
        net.enableWinograd(false);
#endif
    } catch (const std::exception& e) {
        ROS_ERROR("Error reading model: %s", e.what());
        return false;
    }

    if (isCuda) {
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    } else {
        ROS_INFO("Inference device: CPU");
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    return true;
}

bool Yolov8::detect(cv::Mat& srcImg, cv::dnn::Net& net, std::vector<OutputParams>& output)
{
    cv::Mat blob;
    output.clear();
    cv::Mat netInputImg;
    cv::Vec4d params;
    letterBox(srcImg, netInputImg, params, cv::Size(_netWidth, _netHeight));
    cv::dnn::blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(_netWidth, _netHeight),
                           cv::Scalar(0, 0, 0), true, false);

    net.setInput(blob);
#if (CV_VERSION_MAJOR*100 + CV_VERSION_MINOR) >= 406
    net.enableWinograd(false);
#endif
    std::vector<cv::Mat> net_output_img;
    net.forward(net_output_img, net.getUnconnectedOutLayersNames());

    cv::Mat output0 = cv::Mat(cv::Size(net_output_img[0].size[2], net_output_img[0].size[1]),
                              CV_32F, (float*)net_output_img[0].data).t();
    int net_width = output0.cols;
    int rows = output0.rows;
    int score_array_length = net_width - 4;
    float* pdata = (float*)output0.data;

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (int r = 0; r < rows; ++r) {
        cv::Mat scores(1, score_array_length, CV_32FC1, pdata + 4);
        cv::Point classIdPoint;
        double max_class_score;
        cv::minMaxLoc(scores, 0, &max_class_score, 0, &classIdPoint);
        max_class_score = (float)max_class_score;
        if (max_class_score >= _classThreshold) {
            float x = (pdata[0] - params[2]) / params[0];
            float y = (pdata[1] - params[3]) / params[1];
            float w = pdata[2] / params[0];
            float h = pdata[3] / params[1];
            int left = std::max(int(x - 0.5 * w + 0.5), 0);
            int top = std::max(int(y - 0.5 * h + 0.5), 0);
            class_ids.push_back(classIdPoint.x);
            confidences.push_back(max_class_score);
            boxes.push_back(cv::Rect(left, top, int(w + 0.5), int(h + 0.5)));
        }
        pdata += net_width;
    }

    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, _classThreshold, _nmsThreshold, nms_result);
    cv::Rect holeImgRect(0, 0, srcImg.cols, srcImg.rows);
    for (size_t i = 0; i < nms_result.size(); ++i) {
        int idx = nms_result[i];
        OutputParams result;
        result.id = class_ids[idx];
        result.confidence = confidences[idx];
        result.box = boxes[idx] & holeImgRect;
        output.push_back(result);
    }
    return !output.empty();
}

cv::Mat Yolov8::drawPred(cv::Mat& img, const std::vector<OutputParams>& result,
                         const std::vector<std::string>& classNames,
                         const std::vector<cv::Scalar>& color)
{
    cv::Mat outputImg = img.clone();
    cv::Mat mask = img.clone();
    for (size_t i = 0; i < result.size(); i++) {
        int left = 0, top = 0;
        if (result[i].box.area() > 0) {
            cv::rectangle(outputImg, result[i].box, color[result[i].id], 2, 8);
            left = result[i].box.x;
            top = result[i].box.y;
        }
        if (result[i].rotatedBox.size.width * result[i].rotatedBox.size.height > 0) {
            DrawRotatedBox(outputImg, result[i].rotatedBox, color[result[i].id], 2);
            left = result[i].rotatedBox.center.x;
            top = result[i].rotatedBox.center.y;
        }
        if (result[i].boxMask.rows && result[i].boxMask.cols > 0) {
            mask(result[i].box).setTo(color[result[i].id], result[i].boxMask);
        }
        std::string label = classNames[result[i].id] + ":" + std::to_string(result[i].confidence);
        int baseLine;
        cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        top = std::max(top, labelSize.height);
        cv::putText(outputImg, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 1,
                    color[result[i].id], 2);
    }
    cv::addWeighted(outputImg, 0.5, mask, 0.5, 0, outputImg);
    return outputImg;
}

void Yolov8::letterBox(const cv::Mat& image, cv::Mat& outImage, cv::Vec4d& params,
                       const cv::Size& newShape, bool autoShape, bool scaleFill,
                       bool scaleUp, int stride, const cv::Scalar& color)
{
    cv::Size shape = image.size();
    float r = std::min((float)newShape.height / (float)shape.height,
                       (float)newShape.width / (float)shape.width);
    if (!scaleUp) r = std::min(r, 1.0f);

    float ratio[2]{ r, r };
    int new_un_pad[2] = { (int)std::round((float)shape.width * r),
                          (int)std::round((float)shape.height * r) };

    auto dw = (float)(newShape.width - new_un_pad[0]);
    auto dh = (float)(newShape.height - new_un_pad[1]);

    if (autoShape) {
        dw = (float)((int)dw % stride);
        dh = (float)((int)dh % stride);
    } else if (scaleFill) {
        dw = 0.0f; dh = 0.0f;
        new_un_pad[0] = newShape.width;
        new_un_pad[1] = newShape.height;
        ratio[0] = (float)newShape.width / (float)shape.width;
        ratio[1] = (float)newShape.height / (float)shape.height;
    }

    dw /= 2.0f; dh /= 2.0f;

    if (shape.width != new_un_pad[0] || shape.height != new_un_pad[1]) {
        cv::resize(image, outImage, cv::Size(new_un_pad[0], new_un_pad[1]));
    } else {
        outImage = image.clone();
    }

    int top = int(std::round(dh - 0.1f));
    int bottom = int(std::round(dh + 0.1f));
    int left = int(std::round(dw - 0.1f));
    int right = int(std::round(dw + 0.1f));
    params[0] = ratio[0]; params[1] = ratio[1];
    params[2] = left; params[3] = top;
    cv::copyMakeBorder(outImage, outImage, top, bottom, left, right, cv::BORDER_CONSTANT, color);
}
