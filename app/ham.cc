#include "MeshMap.h"
#include "Tracking.h"
#include "Mesh_Visualizer.h"

#include <opencv2/opencv.hpp>
#include <open3d/Open3D.h>
#include <Eigen/Core>
#include <iostream>
#include <string>
#include <vector>
#include <thread>

void visualizer_mesh(std::shared_ptr<open3d::geometry::TriangleMesh> &mesh) {
    open3d::visualization::Visualizer visualizer;
    Eigen::Vector3d color(0.15, 0.15, 0.15); 
    auto lines = std::make_shared<open3d::geometry::LineSet>();
    for (const auto& triangle : mesh->triangles_) {
        lines->lines_.push_back({triangle(0), triangle(1)});
        lines->lines_.push_back({triangle(1), triangle(2)});
        lines->lines_.push_back({triangle(2), triangle(0)});

        lines->colors_.push_back(color); lines->colors_.push_back(color); lines->colors_.push_back(color); 
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
    int thresholdValue=0;
    Mesh_Visualizer *visualize;
    // open3d::visualization::ViewControl &view_control = visualizer.GetViewControl();
    // view_control.SetLookat({10.0, 0.0, 120.0}); // Setze den Startpunkt der Kamera auf (0, 0, 0)
    // view_control.SetFront({0.1, 0.0, -1.0});
    
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
    visualize = new Mesh_Visualizer(1600, 900, vertices, triangles, K, mesh);
    visualize->initImageParams(frame);
    visualize->UpdateMesh(frame, mesh);

    // cv::imwrite("ref_img.png", frame);
    
    // for(int i=0; i<mesh->triangles_.size();i++) {
    //     std::cout << mesh->triangles_[i].x() << " " << mesh->triangles_[i].y() << " " << mesh->triangles_[i].z() << std::endl;
    // }
    
    // exit(1);
    std::vector<double> inital_obs;

    Tracking *tracking = new Tracking(frame, K, vertices, triangles, thresholdValue);
    MeshMap *map = new MeshMap(vertices, triangles, K);
    
    tracking->setunordered_mapping(map); // obs_set in here!
    map->setTracking(tracking);
    
    std::vector<cv::Point2f> pixel; // mittlerweile unneccesary?
    
    // auto pc = open3d::io::CreatePointCloudFromFile("../data/Hamlyn/f7/heartDepthMap_0.txt", "xyz");

    //  // Index der Punkte finden, die nicht NaN sind
    // std::vector<size_t> valid_indices;
    // for (int i = 0; i < pc->points_.size(); ++i) {
    //     if (!std::isnan(pc->points_[i].x()) && !::isnan(pc->points_[i].y()) && !::isnan(pc->points_[i].z())) {
    //         valid_indices.push_back(i);
    //     }
    // }

    // // Punktwolke filtern
    // auto filtered_pc = pc->SelectByIndex(valid_indices);

    // // Visualisierung
    // open3d::visualization::DrawGeometries({pc});

    // // KD-Baum für das Mesh erstellen
    // open3d::geometry::KDTreeFlann kdtree;
    // kdtree.SetGeometry(*mesh);
    // open3d::geometry::
    // // Für jeden Punkt in der Punktwolke den nächsten Nachbarn im Mesh finden
    // std::vector<int> nearest_triangle_indices(pc->points_.size());
    // std::vector<double> nearest_triangle_distances(pc->points_.size());
    // for (int i = 0; i < pc->points_.size(); ++i) {
    //     Eigen::Vector3d query_point(pc->points_[i].x(), pc->points_[i].y(), pc->points_[i].z());
    //     kdtree.SearchKNN(query_point, 1, nearest_triangle_indices[i], nearest_triangle_distances[i]);
    // }

    
    // exit(1);
    while(1) {
        if(frame.empty())
            break;
        
        tracking->track(frame, pixel);
        map->unordered_map();

        mesh->vertices_ = map->getVertices();
        mesh->triangles_ = map->getTriangles();
        visualize->UpdateMesh(frame, mesh);

        
        int key = cv::waitKey(1);
        if (key == 'q')
        {
            std::cout << "q key is pressed by the user. Stopping the video" << std::endl;
            break;
        }
        cap >> frame;



        // For later in a function!
        
        
       

        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }    
   
    

    cap.release();
    return 0;
}