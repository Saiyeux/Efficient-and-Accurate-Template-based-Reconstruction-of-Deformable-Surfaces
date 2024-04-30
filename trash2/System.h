#ifndef SYSTEM_H
#define SYSTEM_H

// #include <opencv2/opencv.hpp>
// #include <open3d/Open3D.h>
// #include <Eigen/Core>
// #include <vector>
// #include <yaml-cpp/yaml.h>
// #include <thread>
// #include "viewer/Viewer.h"

// namespace Viewer {
//     class MeshViewer;
// }

class Tracking;
class MeshMap;
class GroundTruth_compare;
class database;

class System {
public:
    System();
    // System(std::vector<Eigen::Vector3i> ref_triangles, std::vector<Eigen::Vector3d> ref_vertices, cv::Mat ref_img, const YAML::Node &config, std::shared_ptr<open3d::geometry::TriangleMesh> mesh, GroundTruth_compare *gt);

    // void monocular_feed(cv::Mat &img); 

    // Viewer::MeshViewer *viewer_ = nullptr;


private:
    // std::vector<Eigen::Vector3i> ref_triangles_;
    // std::vector<Eigen::Vector3d> ref_vertices_;
    // cv::Mat ref_img_;
    // const YAML::Node config_;
    // std::shared_ptr<open3d::geometry::TriangleMesh> mesh_ = nullptr;

    // Tracking *tracking_ = nullptr;
    // MeshMap *map_ = nullptr;
    // GroundTruth_compare *gt_ = nullptr;
    // database *db_ = nullptr;

    // std::unique_ptr<std::thread> viewing_thread_ = nullptr;

};

#endif