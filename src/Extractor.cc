#include "Extractor.h"

Extractor::Extractor(cv::Mat &frame, std::vector<cv::Point2f> &pixel_reference) : pixel_reference_(pixel_reference) {
    cv::cvtColor(frame, pre_frame_gray_, cv::COLOR_BGR2GRAY);
}

void Extractor::extract(cv::Mat &frame, std::vector<cv::Point2f> &pixel_correspondence) {
        cv::Mat cur_frame_gray;
        cv::cvtColor(frame, cur_frame_gray, cv::COLOR_BGR2GRAY);
        // pixel_correspondence.clear();
        cv::calcOpticalFlowPyrLK(pre_frame_gray_, cur_frame_gray, pixel_reference_, pixel_correspondence, status, err, cv::Size(360/4,288/4),10, criteria); // also 21,21 window would be good
        // cv::calcOpticalFlowPyrLK(pre_frame_gray_, cur_frame_gray, pixel_reference_, pixel_correspondence, status, err, cv::Size(20,20),10, criteria); // also 21,21 window would be good
}