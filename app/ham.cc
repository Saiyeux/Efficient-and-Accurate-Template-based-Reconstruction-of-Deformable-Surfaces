#include "MeshMap.h"
#include "Tracking.h"

#include <opencv2/opencv.hpp>
#include <open3d/Open3D.h>
#include <Eigen/Core>
#include <iostream>
#include <string>
#include <vector>
#include <thread>

void visualizer_mesh(std::shared_ptr<open3d::geometry::TriangleMesh> &mesh) {
    open3d::visualization::Visualizer visualizer;
    
    auto lines = std::make_shared<open3d::geometry::LineSet>();
    for (const auto& triangle : mesh->triangles_) {
        lines->lines_.push_back({triangle(0), triangle(1)});
        lines->lines_.push_back({triangle(1), triangle(2)});
        lines->lines_.push_back({triangle(2), triangle(0)});
    }
    lines->points_ = mesh->vertices_;
    visualizer.CreateVisualizerWindow("Mesh Visualisierung", 1600, 900);  
    visualizer.AddGeometry(mesh);
    visualizer.AddGeometry(lines);

    open3d::visualization::ViewControl &view_control = visualizer.GetViewControl();
    view_control.SetLookat({10.0, 0.0, 120.0}); // Setze den Startpunkt der Kamera auf (0, 0, 0)
    view_control.SetFront({0.1, 0.0, -1.0});

    

    visualizer.Run();

}


int main() {
    // Config
    // int thresholdValue=40;
    int thresholdValue=30;
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Mesh Visualisierung", 1600, 900);
    open3d::visualization::ViewControl &view_control = visualizer.GetViewControl();
    view_control.SetLookat({10.0, 0.0, 120.0}); // Setze den Startpunkt der Kamera auf (0, 0, 0)
    view_control.SetFront({0.1, 0.0, -1.0});
    
    // Creation of a mesh
    std::string obj_file_path = "../data/Hamlyn/ReferenceMesh2.obj";
    // std::string obj_file_path = "../data/Hamlyn/dense reconstruction5.obj";
    std::string video_file = "../data/Hamlyn/output.mp4";

    std::shared_ptr<open3d::geometry::TriangleMesh> mesh = open3d::io::CreateMeshFromFile(obj_file_path);
    std::vector<Eigen::Vector3d> vertices = mesh->vertices_;
    std::vector<Eigen::Vector3i> triangles = mesh->triangles_;
    Eigen::Matrix3d K;
    K <<    391.656525, 0.000000, 165.964371,
            0.000000, 426.835144, 154.498138,
            0.000000, 0.000000, 1.000000;

    cv::Mat frame;
    cv::VideoCapture cap(video_file);
    cap >> frame;

    std::vector<double> inital_obs;

    Tracking *tracking = new Tracking(frame, K, vertices, triangles, thresholdValue);
    MeshMap *map = new MeshMap(vertices, triangles, K);
    
    tracking->setunordered_mapping(map); // obs_set in here!
    map->setTracking(tracking);
    
    
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
                    t << 0,0,100;
                    R << 1,0,0,
                         0,1,0,
                         0,-1,1;

    std::vector<cv::Point2f> pixel;
    Eigen::Vector3d rotation_axis(1.0, 0.0, 0.0); // y-Achse
    double rotation_angle_rad = M_PI;

    // Erstelle eine Rotationsmatrix um die gegebene Achse und Winkel
    Eigen::AngleAxisd rotation(rotation_angle_rad, rotation_axis);
    Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();
    while(1) {
        if(frame.empty())
            break;
        tracking->track(frame, pixel);
        map->unordered_map();
        int key = cv::waitKey(1);
        if (key == 'q')
        {
            std::cout << "q key is pressed by the user. Stopping the video" << std::endl;
            break;
        }
        cap >> frame;

        mesh->vertices_ = map->getVertices();
        mesh->triangles_ = map->getTriangles();

        visualizer_mesh(mesh);
        
        // visualizer.ClearGeometries();
        // visualizer.AddGeometry(mesh);
        // mesh->Rotate(rotation_matrix,t);
        
        // auto lines = std::make_shared<open3d::geometry::LineSet>();
        // for (const auto& triangle : mesh->triangles_) {
        //     lines->lines_.push_back({triangle(0), triangle(1)});
        //     lines->lines_.push_back({triangle(1), triangle(2)});
        //     lines->lines_.push_back({triangle(2), triangle(0)});
        // }
        // // lines->lines_ = mesh->triangles_;
        // lines->points_ = mesh->vertices_;
        // // lines->Rotate(rotation_matrix,t);
        
        // visualizer.AddGeometry(lines);


        // visualizer.UpdateGeometry();
        // visualizer.PollEvents();
        // visualizer.UpdateRender();
        // visualizer.Run();
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }    
   
    
     visualizer.DestroyVisualizerWindow();

    // mesh->vertices_ = map->getVertices();
    // mesh->triangles_ = map->getTriangles();
    
    // visualizer(mesh);
    cap.release();
    return 0;
}