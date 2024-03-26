#include <opencv2/opencv.hpp>
#include <open3d/Open3D.h>
#include <Eigen/Core>
#include <vector>
#include <yaml-cpp/yaml.h>

class Tracking;
class MeshMap;
class Mesh_Visualizer;

class System {
public:
    System(std::vector<Eigen::Vector3i> ref_triangles, std::vector<Eigen::Vector3d> ref_vertices, cv::Mat ref_img, const YAML::Node &config, std::shared_ptr<open3d::geometry::TriangleMesh> mesh);

    void monocular_feed(cv::Mat &img);

private:
    std::vector<Eigen::Vector3i> ref_triangles_;
    std::vector<Eigen::Vector3d> ref_vertices_;
    cv::Mat ref_img_;
    const YAML::Node config_;
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh_ = nullptr;

    Tracking *tracking_ = nullptr;
    MeshMap *map_ = nullptr;
    Mesh_Visualizer *viewer_ = nullptr;

};

