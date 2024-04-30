#ifndef UTILS_H
#define UTILS_H

#include <open3d/Open3D.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace utils {
    void getMesh(std::string path, std::vector<Eigen::Vector3d> &vertices, std::vector<Eigen::Vector3i> &triangles) {
        std::shared_ptr<open3d::geometry::TriangleMesh> mesh;
        // mesh = createRefMesh(config, gt_id);
        mesh = open3d::io::CreateMeshFromFile(path);

        for(int i=0; i<mesh->vertices_.size();i++)
            vertices.push_back(mesh->vertices_[i]);

        for(int i=0; i<mesh->triangles_.size();i++)
            triangles.push_back(mesh->triangles_[i]);
            
    
    }


    void createRefMesh(const YAML::Node &config, uint gt_id, std::vector<Eigen::Vector3d> &vertices, std::vector<Eigen::Vector3i> &triangles) {
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
    
    for(int i=0; i<mesh->vertices_.size();i++)
            vertices.push_back(mesh->vertices_[i]);

        for(int i=0; i<mesh->triangles_.size();i++)
            triangles.push_back(mesh->triangles_[i]);
    }
}

#endif 