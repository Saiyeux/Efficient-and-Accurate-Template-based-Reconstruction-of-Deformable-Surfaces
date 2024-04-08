#include "Mesh_Visualizer.h"


Mesh_Visualizer::Mesh_Visualizer(int width, int height, std::vector<Eigen::Vector3d> &vertices, std::vector<Eigen::Vector3i> &triangles, Eigen::Matrix3d K, std::shared_ptr<open3d::geometry::TriangleMesh> mesh, bool show_only_optimised_part, const YAML::Node &config) :
vertices_(vertices), triangles_(triangles), show_only_optimised_part_(show_only_optimised_part), config_(config) {
    visualizer.CreateVisualizerWindow("Mesh Visualisierung", width, height);
    // mesh_ = std::make_shared<open3d::geometry::TriangleMesh>(vertices, trian);
    mesh_ = std::make_shared<open3d::geometry::TriangleMesh>(open3d::geometry::TriangleMesh());
    std::vector<int> material_ids(triangles.size(), 0); // Setze alle Material-IDs auf 0
    mesh_->triangle_material_ids_ = material_ids;
    // view_control = visualizer.GetViewControl();
    
    K_ = K;
    int w_uv = config_["Visualization"]["w_uv"].as<int>();
    int h_uv = config_["Visualization"]["h_uv"].as<int>();
    ShouldRotate_ = config_["Visualization"]["ShouldRotate"].as<bool>();
    for (int i=0;i< triangles.size(); i++) {
        int f1 = triangles[i].x();
        int f2 = triangles[i].y();
        int f3 = triangles[i].z();

        Eigen::Vector3d v1 = vertices[f1];
        Eigen::Vector3d v2 = vertices[f2];
        Eigen::Vector3d v3 = vertices[f3];

        v1 = K*v1;
        v2 = K*v2;
        v3 = K*v3;

        v1 /= v1.z();
        v2 /= v2.z();
        v3 /= v3.z();
    
        Eigen::Vector2d a,b,c;

        a << v1.x()/w_uv, v1.y()/h_uv;
        b << v2.x()/w_uv, v2.y()/h_uv;
        c << v3.x()/w_uv, v3.y()/h_uv;

       
        
        uv_coordinates_.push_back(a);
        uv_coordinates_.push_back(b);
        uv_coordinates_.push_back(c);
    }
    mesh_->triangle_uvs_ = uv_coordinates_;
}

Mesh_Visualizer::~Mesh_Visualizer() {
    visualizer.DestroyVisualizerWindow();
}

void Mesh_Visualizer::initImageParams(cv::Mat &frame, std::vector<bool> &usable_triangle, std::vector<bool> &usable_vertices) {
    // Setzen Sie die Größe und Kanalinformationen des Image-Objekts
    width_ = frame.cols;
    height_ = frame.rows;
    num_channels_ = frame.channels();
    texture_image.Prepare(width_, height_, num_channels_, bytes_per_channel_);
    texture_error_map.Prepare(width_,height_,1,bytes_per_channel_);
    usable_vertices_ = usable_vertices;
    for(int i=0;i<usable_vertices.size(); i++) {
        usable_vertices_[i] = !usable_vertices[i];
    }

    // usable_triangle_ = usable_triangle;

    // std::cout << frame.size() <<std::endl;
    // std::cout << width_ <<std::endl;
    // std::cout << height_ <<std::endl;
    // std::cout << num_channels_<<std::endl;
    // std::cout << bytes_per_channel_<<std::endl;
    // Zugriff auf die Rohdaten des Image-Objekts
}

void Mesh_Visualizer::updateFrame(cv::Mat &frame) {
    // Kopieren Sie die Pixelwerte aus der cv::Mat in das Image-Objekt
    // std::cout << frame.size() << std::endl;
    // exit(1);
    // std::cout << frame.size() << std::endl;

    uint8_t* data_ptr_ = texture_image.data_.data();
    cv::Mat rgb;
    cv::cvtColor(frame,rgb, cv::COLOR_BGR2RGB);
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            for (int c = 0; c < num_channels_; ++c) {
                *data_ptr_++ = rgb.at<cv::Vec3b>(y, x)[c];
            }
        }
    }
}

void Mesh_Visualizer::updateErrorMap(cv::Mat &frame) {
    // Kopieren Sie die Pixelwerte aus der cv::Mat in das Image-Objekt
    uint8_t* data_ptr_ = texture_error_map.data_.data();
    // cv::Mat rgb;
    // cv::cvtColor(frame,rgb, cv::COLOR_BGR2RGB);
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            *data_ptr_++ = frame.at<uchar>(y, x);
        }
    }
}

void Mesh_Visualizer::UpdateMesh(cv::Mat &frame, std::vector<Eigen::Vector3d> &vertices, std::vector<Eigen::Vector3i> &triangles) {
    
    
    
    frame.convertTo(frame, -1, 1, 50);
    updateFrame(frame);
    // updateErrorMap(frame);
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
                    t << 0,0,100;

    Eigen::Vector3d rotation_axis(1.0, 0.0, 0.0); // y-Achse
    double rotation_angle_rad = M_PI;
    Eigen::AngleAxisd rotation(rotation_angle_rad, rotation_axis);
    Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();

    uv_coordinates_.clear();
    for (int i=0;i< triangles.size(); i++) {
        int f1 = triangles[i].x();
        int f2 = triangles[i].y();
        int f3 = triangles[i].z();

        Eigen::Vector3d v1 = vertices[f1];
        Eigen::Vector3d v2 = vertices[f2];
        Eigen::Vector3d v3 = vertices[f3];

        v1 = K_*v1;
        v2 = K_*v2;
        v3 = K_*v3;

        v1 /= v1.z();
        v2 /= v2.z();
        v3 /= v3.z();
    
        Eigen::Vector2d a,b,c;

        a << v1.x()/frame.cols, v1.y()/frame.rows;
        b << v2.x()/frame.cols, v2.y()/frame.rows;
        c << v3.x()/frame.cols, v3.y()/frame.rows;
        
        uv_coordinates_.push_back(a);
        uv_coordinates_.push_back(b);
        uv_coordinates_.push_back(c);
    }

    mesh_->triangle_uvs_ = uv_coordinates_;
    mesh_->vertices_ = vertices;
    mesh_->triangles_ = triangles;
    mesh_->textures_.clear();
    mesh_->textures_.push_back(texture_image);

    if(show_only_optimised_part_)
        mesh_->RemoveVerticesByMask(usable_vertices_);
    
    
    if (ShouldRotate_)
        mesh_->Rotate(rotation_matrix,t);

    auto lines = std::make_shared<open3d::geometry::LineSet>();
    for (const auto& triangle : mesh_->triangles_) {
        lines->lines_.push_back({triangle(0), triangle(1)});
        lines->lines_.push_back({triangle(1), triangle(2)});
        lines->lines_.push_back({triangle(2), triangle(0)});
    }
    lines->points_ = mesh_->vertices_;
    
    
    visualizer.ClearGeometries();
    visualizer.AddGeometry(mesh_);
    visualizer.AddGeometry(lines);
    // view_control.SetLookat({10.0, 0.0, 120.0}); // Setze den Startpunkt der Kamera auf (0, 0, 0)
    // view_control.SetFront({0.1, 0.0, -1.0});

    visualizer.UpdateGeometry();
    // visualizer.Run();
    visualizer.PollEvents();
    visualizer.UpdateRender();
}