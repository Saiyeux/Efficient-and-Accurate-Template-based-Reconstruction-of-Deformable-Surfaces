#include "MeshMap.h"
#include "Tracking.h"
#include "optimize/Optimizer.h"
#include "optimize/OptimizerWithoutMiddlePoint.h"
#include "optimize/OptimizerDistanceOnly.h"




MeshMap::MeshMap(std::vector<Eigen::Vector3d> &vertices, std::vector<Eigen::Vector3i> &triangles, Eigen::Matrix3d K, int max_iteration, int optimization_algorithm, bool verbose): K_(K), fx_(K(0,0)), fy_(K(1,1)), cx_(K(0,2)), cy_(K(1,2)),
        vertices_(vertices), triangles_(triangles), number_triangles_(triangles.size()), number_vertices_(vertices.size()), optimization_algorithm_(optimization_algorithm) {
            
            if (optimization_algorithm_ == 0)
                optimizer_ = new Optimizer(max_iteration, vertices_, triangles_, verbose);
            else if (optimization_algorithm_ == 1)
                optimizerDistance_ = new OptimizerDistanceOnly(max_iteration, vertices, triangles_, K, verbose);
            else if (optimization_algorithm_ == 2)
                optimizeWithout_ = new OptimizerWithoutMiddlePoint(max_iteration, vertices_, triangles_, verbose);
                
        }

MeshMap::MeshMap(std::vector<Eigen::Vector3d> vertices, std::vector<Eigen::Vector3i>triangles, const YAML::Node &config) 
: vertices_(vertices), triangles_(triangles), config_(config) {
    K_ <<    config["Image"]["fx"].as<double>(), 0.000000, config["Image"]["cx"].as<double>(),
            0.000000, config["Image"]["fy"].as<double>(), config["Image"]["cy"].as<double>(),
            0.000000, 0.000000, 1.000000;
    fx_= K_(0,0);
    fy_= K_(1,1);
    cx_= K_(0,2);
    cy_= K_(1,2);

    int max_iteration = config["Optimizer"]["max_iteration"].as<int>();
    int optimization_algorithm = config["System"]["optimization_algorithm"].as<int>();
    bool verbose = config["System"]["verbose"].as<bool>();


    if (optimization_algorithm_ == 0)
        optimizer_ = new Optimizer(max_iteration, vertices_, triangles_, verbose);
    else if (optimization_algorithm_ == 1)
        optimizerDistance_ = new OptimizerDistanceOnly(max_iteration, vertices, triangles_, K_, verbose);
    else if (optimization_algorithm_ == 2)
        optimizeWithout_ = new OptimizerWithoutMiddlePoint(max_iteration, vertices_, triangles_, verbose);
}

MeshMap::~MeshMap() {
    // delete everything
}

void MeshMap::set_Observation(std::vector<double> &obs) {
    obs_ = obs;
}


void MeshMap::setTracking(Tracking *tracking) {
    tracking_ = tracking;
}

std::vector<Eigen::Vector3d>& MeshMap::getVertices() {
    return vertices_;
}

std::vector<Eigen::Vector3i>& MeshMap::getTriangles() {
    return triangles_;
}

void MeshMap::unordered_map() {
    tracking_->getObs(obs_);
    int pre_id =-1;
    for(int i=0; i<obs_.size()/6; i++) {
        int id = obs_[i*6];
        
        if(id == pre_id)
            continue;
        pre_id = id;
        
        triangle_unordered_mapping_[id] = 0;
        Eigen::Vector3i triangle = triangles_[id];
        vertices_unordered_mapping_[triangle.x()] = 0;
        vertices_unordered_mapping_[triangle.y()] = 0;
        vertices_unordered_mapping_[triangle.z()] = 0;
        // int f1 = triangle.x();
        // int f2 = triangle.y();
        // int f3 = triangle.z();
        // auto it = vertices_unordered_mapping_.find(f1);
        // if (it == vertices_unordered_mapping_.end())
        //     vertices_unordered_mapping_.insert(std::make_pair(f1, 0));
        // it = vertices_unordered_mapping_.find(f2);
        // if (it == vertices_unordered_mapping_.end())
        //     vertices_unordered_mapping_.insert(std::make_pair(f2, 0));
        // it = vertices_unordered_mapping_.find(f3);
        // if (it == vertices_unordered_mapping_.end())
        //     vertices_unordered_mapping_.insert(std::make_pair(f2, 0));

        // std::cout << triangle.x()<<" " << triangle.y() << " "<<
        //  triangle.z() << std::endl;
    
        // std::cout << vertices_[triangle.x()].x() <<" " <<vertices_[triangle.x()].y() <<" " <<vertices_[triangle.x()].z() <<" " << 
        // vertices_[triangle.y()].x() << " "<<vertices_[triangle.y()].y() << " "<<vertices_[triangle.y()].z() << " "<<
        // vertices_[triangle.z()].x() << " "<< vertices_[triangle.z()].y() <<" "<< vertices_[triangle.z()].z() << std::endl;
    }
    int num_vertices = vertices_unordered_mapping_.size();
    int num_triangles = triangle_unordered_mapping_.size();

    // double optimize_vertices[num_vertices*3];
    // double optimize_triangles[num_triangles*3];

    int counter = 0;
    for(auto& unordered_map: triangle_unordered_mapping_) {
        // Eigen::Vector3i triangle = triangles_[unordered_map.first];
        // optimize_triangles[counter*3] = triangle.x();
        // optimize_triangles[counter*3 + 1] = triangle.y();
        // optimize_triangles[counter*3 + 2] = triangle.z();
        unordered_map.second = counter; 
        counter++;
    }
    counter = 0;
    for(auto& unordered_map: vertices_unordered_mapping_) {
        // Eigen::Vector3d vertex = vertices_[unordered_map.first];
        // optimize_vertices[counter*3] = vertex.x();
        // optimize_vertices[counter*3 + 1] = vertex.y();
        // optimize_vertices[counter*3 + 2] = vertex.z();
        unordered_map.second = counter;
        counter++;
    }

    double vertices[num_vertices*3];
     for(auto& unordered_map: vertices_unordered_mapping_) {
        Eigen::Vector3d vertex = vertices_[unordered_map.first];
        // std::cout << num_vertices << std::endl;
        int id = unordered_map.second;
        vertices[id * 3] = vertex.x();
        vertices[id * 3 + 1] = vertex.y();
        vertices[id * 3 + 2] = vertex.z();
    }
    
    std::vector<int> triangles;
    for(auto& unordered_map: triangle_unordered_mapping_) {
        Eigen::Vector3i triangle = triangles_[unordered_map.first];
        triangles.push_back(vertices_unordered_mapping_[triangle.x()]);
        triangles.push_back(vertices_unordered_mapping_[triangle.y()]);
        triangles.push_back(vertices_unordered_mapping_[triangle.z()]);
    }

    
// int pre = -1;
//     for(int obs_id=0; obs_id < obs_.size()/6; obs_id++) {
//         int face_id = triangle_unordered_mapping_[obs_[obs_id * 6]];
//         int f1 = triangles[face_id*3];
//         int f2 = triangles[face_id*3+1];
//         int f3 = triangles[face_id*3+2];
//         if (pre == face_id)
//             continue;
//             pre = face_id;
//         double *v1= &vertices[f1*3];
//         // double v2 = {xyz[f2*3],xyz[f2*3+1],xyz[f2*3+2]};
//         double *v2 = &vertices[f2*3];
//         // double v3 = {xyz[f3*3],xyz[f3*3+1],xyz[f3*3+2]};
//         double *v3 = &vertices[f3*3];
//         std::cout <<v1[0]<<" "<<v1[1]<<" "<<v1[2]<<" "
//                 <<v2[0]<<" "<<v2[1]<<" "<<v2[2]<<" "
//                 <<v3[0]<<" "<<v3[1]<<" "<<v3[2]<<" "<<std::endl; 
//     }
    // exit(1);

    if(optimization_algorithm_ == 0) {
        optimizer_->setParamater(&obs_[0], vertices_unordered_mapping_, triangle_unordered_mapping_, vertices_unordered_mapping_.size(), triangle_unordered_mapping_.size(), obs_.size() / 6);
        optimizer_->initialize();
        optimizer_->run();
        optimizer_->getVertices(vertices_);
    } else if (optimization_algorithm_ == 1) {
        optimizerDistance_->setParamater(&obs_[0], vertices_unordered_mapping_, triangle_unordered_mapping_, vertices_unordered_mapping_.size(), triangle_unordered_mapping_.size(), obs_.size() / 6);
        optimizerDistance_->initialize();
        optimizerDistance_->run();
        optimizerDistance_->getVertices(vertices_);
    } else if (optimization_algorithm_ == 2) {
        optimizeWithout_->setParamater(&obs_[0], vertices_unordered_mapping_, triangle_unordered_mapping_, vertices_unordered_mapping_.size(), triangle_unordered_mapping_.size(), obs_.size() / 6);
        optimizeWithout_->initialize();
        optimizeWithout_->run();
        optimizeWithout_->getVertices(vertices_);
    }
}