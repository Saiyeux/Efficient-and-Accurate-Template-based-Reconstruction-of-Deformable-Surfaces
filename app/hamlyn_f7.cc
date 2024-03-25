#include "MeshMap.h"
#include "Tracking.h"
#include "viewer/Mesh_Visualizer.h"
#include "System.h"

#include <opencv2/opencv.hpp>
#include <open3d/Open3D.h>
#include <open3d/t/geometry/RaycastingScene.h>
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <memory>
#include <yaml-cpp/yaml.h>

// tmp
#include<fstream>

void compareWithGroundTruth(open3d::geometry::TriangleMesh mesh, cv::Mat &output, int FrameNo) {
    // static int FrameNo = 0;
    double a = FrameNo; 
    a /= 25.0;
    a += 0.093333;
    a *= 30;
    // a -= 1;
    double no = std::round(int(a) % 20);
    std::cout << FrameNo << " " << no << " " << std::fmod(a, 20) <<std::endl;
    std::string no_str = std::to_string(int(no));
    auto pc = open3d::io::CreatePointCloudFromFile("../data/Hamlyn/f7/heartDepthMap_" + no_str + ".txt", "xyz");

    static open3d::t::geometry::TriangleMesh t_mesh = open3d::t::geometry::TriangleMesh::FromLegacy(mesh);
    open3d::t::geometry::PointCloud t_pc = open3d::t::geometry::PointCloud::FromLegacy(*pc);
    // t_mesh.Clear();

    auto scene = open3d::t::geometry::RaycastingScene() ;
    scene.AddTriangles(t_mesh);
    open3d::core::Tensor distances = scene.ComputeDistance(t_pc.GetPointPositions(),0);
    std::vector<float> values = distances.ToFlatVector<float>();
    
    cv::Mat mat(288, 360, CV_32F, values.data());
    cv::Mat scaled_mat;
    cv::normalize(mat, scaled_mat, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::Mat heatmap;
    cv::applyColorMap(scaled_mat, heatmap, cv::COLORMAP_JET);

    // cv::Mat in[] = {cv::Mat::zeros(scaled_mat.rows, scaled_mat.cols, CV_8UC1), cv::Mat::zeros(scaled_mat.rows, scaled_mat.cols, CV_8UC1), scaled_mat};
    cv::Mat in[] = {scaled_mat, scaled_mat, scaled_mat};
    int from_to[] = {0,0, 1,1, 2,2};
    cv::mixChannels(in, 3, &heatmap, 1, from_to, 3);
    
    double mean=0;
    std::vector<Eigen::Vector3d> pce = pc->points_;
    int counter = 0;
    for(int i=0;i<values.size();i++) {
        float val = values[i];
        if((pce[i].x() == 0) && (pce[i].y() == 0) && (pce[i].z() == 0))
            continue;
        counter++;
        mean += val;
        // std::cout << val << std::endl;
    }
    std::cout << mean/counter << " " << counter << std::endl;
    cv::imshow("Heatmap", heatmap);
    output = heatmap;
    
    // Zusammenführen der Kanäle zu einem 3-Kanal-Bild
    // cv::merge(in, 3, output);
}


int main() {
    // Config
    // int thresholdValue=40;
    // yaml_optional_ref(const YAML::Node& ref_node, const std::string& key)
    const YAML::Node config = YAML::LoadFile("../app/config.yaml");
    int max_iteration = config["Optimizer"]["max_iteration"].as<int>();
    int thresholdValue = config["Preprocessing"]["brightness_threshold"].as<int>();
    int optimization_algorithm = config["System"]["optimization_algorithm"].as<int>();
    bool verbose = config["System"]["verbose"].as<bool>();

    // open3d::visualization::ViewControl &view_control = visualizer.GetViewControl();
    // view_control.SetLookat({10.0, 0.0, 120.0}); // Setze den Startpunkt der Kamera auf (0, 0, 0)
    // view_control.SetFront({0.1, 0.0, -1.0});
    
    // Creation of a mesh
    std::string video_file = config["System"]["video_file_path"].as<std::string>();
   std::string obj_file_path = config["System"]["reference_file_path"].as<std::string>();

    Mesh_Visualizer *visualize;
    Eigen::Matrix3d K;
    cv::Mat frame;
    cv::VideoCapture cap(video_file);
    cap >> frame;
    K <<    config["Image"]["fx"].as<double>(), 0.000000, config["Image"]["cx"].as<double>(),
            0.000000, config["Image"]["fy"].as<double>(), config["Image"]["cy"].as<double>(),
            0.000000, 0.000000, 1.000000;

    std::shared_ptr<open3d::geometry::TriangleMesh> mesh = open3d::io::CreateMeshFromFile(obj_file_path);
    std::vector<Eigen::Vector3d> vertices = mesh->vertices_;
    std::vector<Eigen::Vector3i> triangles = mesh->triangles_;
    
    visualize = new Mesh_Visualizer(config["Visualization"]["width"].as<int>(), config["Visualization"]["height"].as<int>(), vertices, triangles, K, mesh);
    visualize->initImageParams(frame);

  
    std::vector<double> inital_obs;
    Tracking *tracking = new Tracking(frame, K, vertices, triangles, thresholdValue);
    MeshMap *map = new MeshMap(vertices, triangles, K, max_iteration, optimization_algorithm, verbose);
    
    tracking->setunordered_mapping(map); // obs_set in here!
    map->setTracking(tracking);
    
    std::vector<cv::Point2f> pixel; // mittlerweile unneccesary?
    
    System *sys = new System(triangles, vertices, frame, config);
    

    // auto scene = open3d::t::geometry::RaycastingScene() ;
   
    
    // exit(1);
    int FrameNo = 0;
    
    while(1) {
        if(frame.empty())
            break;
        
        tracking->track(frame, pixel);
        map->unordered_map();

        mesh->vertices_ = map->getVertices();
        mesh->triangles_ = map->getTriangles();
        
        // cv::Mat error_map;
        // compareWithGroundTruth(*mesh, error_map, FrameNo);
        // FrameNo++;
        // cv::imshow("sad", frame);
        // cv::waitKey(0);
    
        // visualize->UpdateMesh(error_map, mesh);
        frame.convertTo(frame, -1, 1, 50);
        visualize->UpdateMesh(frame, mesh);

        int key = cv::waitKey(1);
        if (key == 'q')
        {
            std::cout << "q key is pressed by the user. Stopping the video" << std::endl;
            break;
        }
        cap >> frame;

        
        // For later in a function!
        // std::ofstream file("Normal/High/distanceOnly_" + std::to_string(FrameNo) + ".txt");
        // FrameNo++;
        // for(int i=0; i < mesh->vertices_.size(); i++) {
        //     file << mesh->vertices_[i].x() << " " << mesh->vertices_[i].y() << " " <<
        //     mesh->vertices_[i].z() << std::endl;
        // }
        // file.close();
        // // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // if(FrameNo == 50)
        //     break;
        // cv::waitKey(0);
    }    
   
    

    cap.release();
    return 0;
}

