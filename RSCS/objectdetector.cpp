#include "objectdetector.h"


ObjectDetector::ObjectDetector(const std::string& model_weights, const std::string& model_cfg, const std::string& framework, const std::vector<std::string>& class_names)
    : classNames(class_names) {
    net = cv::dnn::readNet(model_weights, model_cfg, framework);
}

void ObjectDetector::detectObjects(const cv::Mat& frame,
                                   std::vector<int>& classIds,
                                   std::vector<float>& confidences,
                                   std::vector<cv::Rect>& boxes,
                                   double threshold,
                                   double nmsThreshold) {
    cv::Mat inputBlob;
    preprocess(frame, inputBlob);
    net.setInput(inputBlob);

    std::vector<cv::Mat> outputBlobs;
    net.forward(outputBlobs, net.getUnconnectedOutLayersNames());

    postprocess(frame, outputBlobs, classIds, confidences, boxes, threshold);

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, threshold, nmsThreshold, indices);

    // Оставляем только пройденные NMS
    std::vector<int> nmsClassIds;
    std::vector<float> nmsConfidences;
    std::vector<cv::Rect> nmsBoxes;

    for (int idx : indices) {
        nmsClassIds.push_back(classIds[idx]);
        nmsConfidences.push_back(confidences[idx]);
        nmsBoxes.push_back(boxes[idx]);
    }

    classIds.swap(nmsClassIds);
    confidences.swap(nmsConfidences);
    boxes.swap(nmsBoxes);
}

void ObjectDetector::preprocess(const cv::Mat& frame, cv::Mat& inputBlob) {
    inputBlob = cv::dnn::blobFromImage(frame, 1 / 255.0, cv::Size(416, 416), cv::Scalar(0, 0, 0), true, false);
}

void ObjectDetector::postprocess(const cv::Mat& frame, const std::vector<cv::Mat>& outputBlobs, std::vector<int>& classIds, std::vector<float>& confidences, std::vector<cv::Rect>& boxes, double threshold) {
    for (size_t i = 0; i < outputBlobs.size(); ++i) {
        float* data = (float*)outputBlobs[i].data;
        for (int j = 0; j < outputBlobs[i].rows; ++j, data += outputBlobs[i].cols) {
            cv::Mat scores = outputBlobs[i].row(j).colRange(5, outputBlobs[i].cols);
            cv::Point classIdPoint;
            double confidence;
            cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > threshold) {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
    }
}
