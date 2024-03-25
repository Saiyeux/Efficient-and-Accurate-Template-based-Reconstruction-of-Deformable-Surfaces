#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <vector>
#include <yaml-cpp/yaml.h>

class System {
public:
    System(std::vector<Eigen::Vector3i> ref_triangles, std::vector<Eigen::Vector3d> ref_vertices, cv::Mat ref_img, const YAML::Node &config);

    void monocular_feed(cv::Mat &img);

private:
    std::vector<Eigen::Vector3i> ref_triangles_;
    std::vector<Eigen::Vector3d> ref_vertices_;
    cv::Mat ref_img_;
    const YAML::Node config_;


};

