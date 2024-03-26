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
    const YAML::Node config = YAML::LoadFile("../app/config.yaml");
    
    // Creation of a mesh
    std::string video_file = config["System"]["video_file_path"].as<std::string>();
   std::string obj_file_path = config["System"]["reference_file_path"].as<std::string>();



    cv::Mat frame;
    cv::VideoCapture cap(video_file);
    cap >> frame;
  

    std::shared_ptr<open3d::geometry::TriangleMesh> mesh = open3d::io::CreateMeshFromFile(obj_file_path);
    std::vector<Eigen::Vector3d> vertices = mesh->vertices_;
    std::vector<Eigen::Vector3i> triangles = mesh->triangles_;

    
    System *sys = new System(triangles, vertices, frame, config, mesh); 
    
    int FrameNo = 0;
    
    while(1) {
        if(frame.empty())
            break;
        sys->monocular_feed(frame);
        // mesh->vertices_ = vertices;
        // mesh->triangles_ = triangles;
        // tracking->track(frame, pixel);
        // map->unordered_map();

        // mesh->vertices_ = map->getVertices();
        // mesh->triangles_ = map->getTriangles();
        
        // cv::Mat error_map;
        // compareWithGroundTruth(*mesh, error_map, FrameNo);
        // FrameNo++;
        // cv::imshow("sad", frame);
        // cv::waitKey(0);
    
        // visualize->UpdateMesh(error_map, mesh);
        // frame.convertTo(frame, -1, 1, 50);
        // visualize->UpdateMesh(frame, mesh);

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

