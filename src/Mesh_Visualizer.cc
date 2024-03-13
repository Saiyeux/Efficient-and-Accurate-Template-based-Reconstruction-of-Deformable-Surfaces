#include "Mesh_Visualizer.h"


Mesh_Visualizer::Mesh_Visualizer(int width, int height, std::vector<Eigen::Vector3d> &vertices, std::vector<Eigen::Vector3i> &triangles, Eigen::Matrix3d K, std::shared_ptr<open3d::geometry::TriangleMesh> mesh) :
vertices_(vertices), triangles_(triangles) {
    visualizer.CreateVisualizerWindow("Mesh Visualisierung", width, height);
    // mesh_ = std::make_shared<open3d::geometry::TriangleMesh>(vertices, trian);
    mesh_ = mesh;
    // view_control = visualizer.GetViewControl();
    
    K_ = K;
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

        a << v1.x()/360, v1.y()/288;
        b << v2.x()/360, v2.y()/288;
        c << v3.x()/360, v3.y()/288;
        
        uv_coordinates_.push_back(a);
        uv_coordinates_.push_back(b);
        uv_coordinates_.push_back(c);
    }
    mesh_->triangle_uvs_ = uv_coordinates_;
}

Mesh_Visualizer::~Mesh_Visualizer() {
    visualizer.DestroyVisualizerWindow();
}

void Mesh_Visualizer::initImageParams(cv::Mat &frame) {
    // Setzen Sie die Größe und Kanalinformationen des Image-Objekts
    width_ = frame.cols;
    height_ = frame.rows;
    num_channels_ = frame.channels();
    texture_image.Prepare(width_, height_, num_channels_, bytes_per_channel_);
    texture_error_map.Prepare(width_,height_,1,bytes_per_channel_);
    // Zugriff auf die Rohdaten des Image-Objekts
}

void Mesh_Visualizer::updateFrame(cv::Mat &frame) {
    // Kopieren Sie die Pixelwerte aus der cv::Mat in das Image-Objekt
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

void Mesh_Visualizer::UpdateMesh(cv::Mat &frame, std::shared_ptr<open3d::geometry::TriangleMesh> &mesh) {
    updateFrame(frame);
    // updateErrorMap(frame);
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
                    t << 0,0,100;

    std::vector<Eigen::Vector3d> vertices = mesh->vertices_;
    std::vector<Eigen::Vector3i> triangles = mesh->triangles_;

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

        a << v1.x()/360, v1.y()/288;
        b << v2.x()/360, v2.y()/288;
        c << v3.x()/360, v3.y()/288;
        
        uv_coordinates_.push_back(a);
        uv_coordinates_.push_back(b);
        uv_coordinates_.push_back(c);
    }
    mesh_->triangle_uvs_ = uv_coordinates_;



    Eigen::Vector3d rotation_axis(1.0, 0.0, 0.0); // y-Achse
    double rotation_angle_rad = M_PI;
    Eigen::AngleAxisd rotation(rotation_angle_rad, rotation_axis);
    Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();
    mesh->textures_.clear();
    mesh->textures_.push_back(texture_image);
    // mesh_->vertices_ = vertices;
    // mesh_->triangles_ = triangles;

    mesh->Rotate(rotation_matrix,t);

    auto lines = std::make_shared<open3d::geometry::LineSet>();
    for (const auto& triangle : mesh->triangles_) {
        lines->lines_.push_back({triangle(0), triangle(1)});
        lines->lines_.push_back({triangle(1), triangle(2)});
        lines->lines_.push_back({triangle(2), triangle(0)});
    }
    lines->points_ = mesh->vertices_;
    visualizer.ClearGeometries();
    visualizer.AddGeometry(mesh);
    visualizer.AddGeometry(lines);
    // view_control.SetLookat({10.0, 0.0, 120.0}); // Setze den Startpunkt der Kamera auf (0, 0, 0)
    // view_control.SetFront({0.1, 0.0, -1.0});

    visualizer.UpdateGeometry();
    // visualizer.Run();
    visualizer.PollEvents();
    visualizer.UpdateRender();
    

}