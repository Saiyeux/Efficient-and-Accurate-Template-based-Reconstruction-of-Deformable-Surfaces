#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <vector>

class MeshMap;
class Extractor;

class Tracking {
    public:
        Tracking(cv::Mat &frame, Eigen::Matrix3d K, std::vector<Eigen::Vector3d> &vertices, std::vector<Eigen::Vector3i> &triangles, int thresholdValue);
        void track(cv::Mat &frame, std::vector<cv::Point2f> &pixel);
        std::vector<double> getObservation();
        void setunordered_mapping(MeshMap *unordered_map);
        void getObs(std::vector<double> &obs);

        double alp[4] = {0.3333,1,0,0};
        double bet[4] = {0.3333,0,1,0};
        double gam[4] = {0.3334,0,0,1};

    private:
        void updateObservation();
        void createMask(int thresholdValue);
        void findUsableVerticies();
        void findUsableTriangles();
        void createInitialObseration();
        void draw_correspondence(cv::Mat &frame);

        MeshMap *unordered_map_ = nullptr;
        Extractor *extraction = nullptr;
        cv::Mat pre_frame_, cur_frame_, mask_;
        Eigen::Matrix3d K_;
        std::vector<Eigen::Vector3d> vertices_;
        std::vector<Eigen::Vector3i> triangles_;
        double fx_ = 0;
        double fy_ = 0;
        double cx_ = 0;
        double cy_ = 0;
        int number_vertices_= 0;
        int number_triangles_ = 0;
        std::vector<bool> usable_vertices_;
        std::vector<bool> usable_triangles_;
        std::vector<double> obs; //  I dont think it is useful here! -> but for now let us assume that all pixel are very good! 
                                // put obs and also the creation of it in unordered_mapping! 

        std::vector<cv::Point2f> pixel_reference_;
        std::vector<cv::Point2f> pixel_correspondence_;

        
};