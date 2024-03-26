#include "System.h"
#include "Tracking.h"
#include "MeshMap.h"

#include "viewer/Mesh_Visualizer.h"


System::System(std::vector<Eigen::Vector3i> ref_triangles, std::vector<Eigen::Vector3d> ref_vertices, cv::Mat ref_img, const YAML::Node &config, std::shared_ptr<open3d::geometry::TriangleMesh> mesh)
 : ref_triangles_(ref_triangles), ref_vertices_(ref_vertices), ref_img_(ref_img), config_(config), mesh_(mesh) {
    Eigen::Matrix3d K;
    K <<    config["Image"]["fx"].as<double>(), 0.000000, config["Image"]["cx"].as<double>(),
            0.000000, config["Image"]["fy"].as<double>(), config["Image"]["cy"].as<double>(),
            0.000000, 0.000000, 1.000000;
    
    tracking_ = new Tracking(ref_img_, ref_vertices_, ref_triangles_, config_);
    map_ = new MeshMap(ref_vertices_, ref_triangles_, config_);

    tracking_->set_MeshMap(map_);
    map_->setTracking(tracking_);
    viewer_ = new Mesh_Visualizer(config["Visualization"]["width"].as<int>(), config["Visualization"]["height"].as<int>(), ref_vertices_, ref_triangles_, K, mesh_);
    viewer_->initImageParams(ref_img);
    
}


void System::monocular_feed(cv::Mat &img) {
    tracking_->track(img);
    map_->unordered_map();    
    viewer_->UpdateMesh(img, map_->getVertices(), map_->getTriangles());
}