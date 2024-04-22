// #include "MeshMap.h"
// #include "Tracking.h"
// #include "viewer/Mesh_Visualizer.h"
#include "System.h"
#include "GT_compare/HamlynGT.h"

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <open3d/Open3D.h>
#include <open3d/t/geometry/RaycastingScene.h>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <cmath>


#include <Eigen/Core>
#include <Eigen/Geometry>

Eigen::Vector3d GetJetColor(double value) {
    const double jet_colors[8][3] = {
        {0, 0, 0.5625}, {0, 0, 1}, {0, 0.5, 1}, {0, 1, 1},
        {0.5, 1, 0.5}, {1, 1, 0}, {1, 0.5, 0}, {1, 0, 0}
    };

    value = std::min(std::max(value, 0.0), 1.0) * 7.0;

    int idx = static_cast<int>(std::floor(value));
    double fraction = value - idx;

    Eigen::Vector3d color;
    for (int i = 0; i < 3; ++i) {
        color[i] = (1 - fraction) * jet_colors[idx][i] + fraction * jet_colors[idx + 1][i];
    }

    return color;
}

std::shared_ptr<open3d::geometry::TriangleMesh> createRefMesh(const YAML::Node &config, uint gt_id) {
    // create reference mesh
    std::string file = config["System"]["reference_file_path"].as<std::string>() + std::to_string(gt_id) + ".txt";
    Eigen::Matrix3d K;
    K <<    config["Image"]["fx"].as<double>(), 0.000000, config["Image"]["cx"].as<double>(),
            0.000000, config["Image"]["fy"].as<double>(), config["Image"]["cy"].as<double>(),
            0.000000, 0.000000, 1.000000;

    auto pc = open3d::io::CreatePointCloudFromFile(file, "xyz");

    auto& points = pc->points_;
    std::vector<Eigen::Vector3d> valid_points;
    for (const auto& point : points) {
        if (point.x() != 0 || point.y() != 0 || point.z() != 0) {
            valid_points.push_back(point);
        }
    }
    pc->points_ = valid_points;
    for (size_t i = 0; i < pc->points_.size(); ++i) {
        auto vertex = pc->points_[i];
        double u = K(0, 0) * vertex(0) / vertex(2) + K(0, 2);
        double v = K(1, 1) * vertex(1) / vertex(2) + K(1, 2);
        
        if (u < 0 || u > 360 || v < 0 || v > 288) {

        }
    }

    pc->EstimateNormals();
    pc->OrientNormalsConsistentTangentPlane(config["GT_Mesh"]["OrientNormalsConsistentTangentPlane"].as<int>());
    double depth = config["GT_Mesh"]["Poisson_depth"].as<double>();
    double width = config["GT_Mesh"]["Poisson_width"].as<double>();
    std::tuple<std::shared_ptr<open3d::geometry::TriangleMesh>, std::vector<double>> meshDensity = open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(*pc, depth, width);

    std::shared_ptr< open3d::geometry::TriangleMesh > mesh = std::get<0>(meshDensity);
    mesh->ComputeTriangleNormals();

    std::vector<size_t> bad_vertices;
    for (size_t i = 0; i < mesh->vertices_.size(); ++i) {
        auto vertex = mesh->vertices_[i];
        double u = K(0, 0) * vertex(0) / vertex(2) + K(0, 2);
        double v = K(1, 1) * vertex(1) / vertex(2) + K(1, 2);
        
        if (u < 0 || u > config["Image"]["width"].as<int>() || v < 0 || v > config["Image"]["height"].as<int>()) {
            bad_vertices.push_back(i);
        }
        // else {
        //     good_vertices.push_back(1);
        // }
    }

    mesh->RemoveVerticesByIndex(bad_vertices);

    mesh = mesh->SimplifyQuadricDecimation(config["GT_Mesh"]["simplification"].as<int>(), std::numeric_limits<double>::infinity(), 1.0);
    return mesh;
}

int main() {
    // frame_id
    int FrameNo = 0;

    // Config
    const YAML::Node config = YAML::LoadFile("../app/Phantom7/config.yaml");
    uint gt_id = int(round((double(FrameNo) / config["Hamlyn"]["FPS"].as<double>() + config["Hamlyn"]["addition"].as<double>()) * config["Hamlyn"]["multiplier"].as<double>())) % config["Hamlyn"]["modulo"].as<int>();
    HamlynGT* gt = new HamlynGT(config);
    // Creation of a mesh
    std::string video_file = config["System"]["video_file_path"].as<std::string>();
    // std::string obj_file_path = config["System"]["reference_file_path"].as<std::string>();


    cv::Mat frame;
    cv::VideoCapture cap(video_file);
    cap >> frame;
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh;
    mesh = createRefMesh(config, gt_id);

    std::vector<Eigen::Vector3d> vertices = mesh->vertices_;
    std::vector<Eigen::Vector3i> triangles = mesh->triangles_;

    
    System *sys = new System(triangles, vertices, frame, config, mesh, gt); 
    
    
    
    while(1) {
        if(frame.empty())
            break;
        sys->monocular_feed(frame);

        int key = cv::waitKey(1);
        if (key == 'q')
        {
            std::cout << "q key is pressed by the user. Stopping the video" << std::endl;
            break;
        }
        cap >> frame;

    
    }        

    double sum = 0;
    for (double element : gt->all_mean_) {
        sum += element;
    }
    double average = static_cast<double>(sum) / gt->all_mean_.size();
    std::cout << "RMS: " << average << std::endl;

    cap.release();
    return 0;
}

