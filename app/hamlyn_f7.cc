#include "MeshMap.h"
#include "Tracking.h"
#include "Mesh_Visualizer.h"

#include <opencv2/opencv.hpp>
#include <open3d/Open3D.h>
#include <open3d/t/geometry/RaycastingScene.h>
#include <Eigen/Core>
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <memory>

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
    int thresholdValue=50;
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
    // visualize->UpdateMesh(frame, mesh);

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
    

    

    auto scene = open3d::t::geometry::RaycastingScene() ;
    // auto nmesh = open3d::t::geometry::TriangleMesh::FromLegacy(mesh);
    // open3d::t::geometry::TriangleMesh nmesh;
    // open3d::geometry::TriangleMesh tmp, tmp2; 
    // tmp = *mesh;
    // tmp2 = tmp;
    // nmesh = open3d::t::geometry::TriangleMesh::FromLegacy(tmp2);
    // scene.AddTriangles(nmesh);

    // std::shared_ptr<open3d::geometry::TriangleMesh> mesh_legacy = std::make_shared<open3d::geometry::TriangleMesh>();

    // Erstellen einer Tensor-kompatiblen 'open3d::t::geometry::TriangleMesh' aus dem Legacy-Mesh
    
    // std::cout << mean_tensor << std::endl;
    // cv::waitKey(10000);
    // std::cout << "asdasd"<< std::endl;
    // exit(1);
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
    int FrameNo = 0;
    while(1) {
        if(frame.empty())
            break;
        
        tracking->track(frame, pixel);
        map->unordered_map();

        mesh->vertices_ = map->getVertices();
        mesh->triangles_ = map->getTriangles();
        
    //     cv::Mat error_map;
    //     compareWithGroundTruth(*mesh, error_map, FrameNo);
    //     FrameNo++;
    //     cv::imshow("sad", frame);
    // cv::waitKey(0);
    
        // visualize->UpdateMesh(error_map, mesh);
        frame.convertTo(frame, -1, 1, 50);
        visualize->UpdateMesh(frame, mesh);

        int key = cv::waitKey(0);
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

