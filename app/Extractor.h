#include <opencv2/opencv.hpp>
#include <vector>

class Extractor {
    public:
        Extractor(cv::Mat &frame, std::vector<cv::Point2f> &pixel_reference);

        void extract(cv::Mat &frame, std::vector<cv::Point2f> &pixel_correspondence);
        std::vector<uchar> status;
    private:
        cv::Mat pre_frame_gray_;
        std::vector<cv::Point2f> pixel_reference_;
        std::vector<float> err;
        cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
        
    
};
