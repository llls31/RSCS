#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

class ObjectDetector {
public:
    ObjectDetector(const std::string& model_weights, const std::string& model_cfg, const std::string& framework, const std::vector<std::string>& class_names);

    void detectObjects(const cv::Mat& frame, std::vector<int>& classIds, std::vector<float>& confidences, std::vector<cv::Rect>& boxes, double threshold = 0.5, double nmsThreshold = 0.4);

private:
    cv::dnn::Net net;
    std::vector<std::string> classNames;

    void preprocess(const cv::Mat& frame, cv::Mat& inputBlob);
    void postprocess(const cv::Mat& frame, const std::vector<cv::Mat>& outputBlobs, std::vector<int>& classIds, std::vector<float>& confidences, std::vector<cv::Rect>& boxes, double threshold);
};

#endif // OBJECT_DETECTOR_H
