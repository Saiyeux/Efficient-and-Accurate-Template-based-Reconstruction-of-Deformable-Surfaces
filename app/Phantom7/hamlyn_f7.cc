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



int main() {
    // Config
    const YAML::Node config = YAML::LoadFile("../app/Phantom7/config.yaml");
    
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

        int key = cv::waitKey(1);
        if (key == 'q')
        {
            std::cout << "q key is pressed by the user. Stopping the video" << std::endl;
            break;
        }
        cap >> frame;

    
    }        

    cap.release();
    return 0;
}

