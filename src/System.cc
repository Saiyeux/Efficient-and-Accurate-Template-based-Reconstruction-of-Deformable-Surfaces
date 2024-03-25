#include "System.h"

System::System(std::vector<Eigen::Vector3i> ref_triangles, std::vector<Eigen::Vector3d> ref_vertices, cv::Mat ref_img, const YAML::Node &config)
 : ref_triangles_(ref_triangles), ref_vertices_(ref_vertices), ref_img_(ref_img), config_(config) {}

void System::monocular_feed(cv::Mat &img) {
    
}