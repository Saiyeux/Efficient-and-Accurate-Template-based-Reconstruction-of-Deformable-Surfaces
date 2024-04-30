#include "MeshViewer.h"
// #include "MeshDrawer.h"
// #include "database.h"

// #include <chrono>
// #include <iostream>
// #include <GL/glew.h>
// #include <GLFW/glfw3.h>
#include <pangolin/pangolin.h>

// #include <pangolin/display/display.h>
// #include <pangolin/gl/gldraw.h>
// #include <pangolin/gl/glvbo.h>
// #include <pangolin/gl/glsl.h>
// #include <pangolin/gl/gl.h>

namespace Viewer {

MeshViewer::MeshViewer() {}

// MeshViewer::MeshViewer(database *db, cv::Mat img, std::vector<Eigen::Vector3d> vertices, std::vector<Eigen::Vector3i> triangles, const YAML::Node &config) {
//     img_ = img;
//     vertices_ = vertices;
//     db_ = db;
//     mDrawer = new MeshDrawer(img, vertices, triangles, config);
// }

void MeshViewer::create_menu_panel() {
    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(150));
    menu_show_mesh_ = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.Show Mesh", true, true));
    menu_show_GT_ = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.Show GT", false, true));
    menu_pause_ = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.Pause", false, true));
    menu_terminate_ = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.Terminate", false, false));
}   

void MeshViewer::request_terminate() {
    isFinished_ = true;
}

void MeshViewer::draw_current_cam_pose() {
    // frustum size of the frame
    // const float w = *menu_frm_size_;

    glLineWidth(2);
    GLfloat color[] = {0.75f, 0.75f, 1.0f};
    glColor3fv(color);
    draw_camera();
}

void MeshViewer::draw_camera() const {
    glPushMatrix();
    draw_frustum();
    glPopMatrix();
}

void MeshViewer::draw_frustum() const {
    constexpr int image_width = 1080;
    constexpr int image_height = 720;
    constexpr float horizontal_fov = 2 * M_PI / 3; // 120 deg
    const float focal_length_pix = 0.5 * image_width / std::tan(0.5 * horizontal_fov);
    Eigen::Matrix3d Kinv = Eigen::Matrix3d::Identity();
    Kinv(0, 0) = 1.0 / focal_length_pix;
    Kinv(1, 1) = 1.0 / focal_length_pix;
    Kinv(0, 2) = -0.5 * image_width / focal_length_pix;
    Kinv(1, 2) = -0.5 * image_height / focal_length_pix;
    const float z = 0.1 / image_width * focal_length_pix;
    pangolin::glDrawFrustum(Kinv, image_width, image_height, z);
}

// void MeshViewer::draw_current_mesh() {
    
//     // db_->getTexture(img_);
//     // db_->getVertices(vertices_);
//     // Get Image and get Vertices
//     mDrawer->addVertices(vertices_);
//     mDrawer->addTextureImage(img_);
//     mDrawer->drawMesh(0.99);
// }

void MeshViewer::run() {
    // glewExperimental = GL_TRUE;
    std::cout << "hey kacke!\n";
    pangolin::CreateWindowAndBind(viewer_name_, width_, height_);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);
    std::cout << "hey kacke!\n";

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    s_cam_ = std::unique_ptr<pangolin::OpenGlRenderState>(new pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(1080,720,420,420,320,240,0.2,100),
        pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
    ));

    // Create Interactive View in window
    // pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1080.0f/720.0f)
            .SetHandler(new pangolin::Handler3D(*s_cam_));


    create_menu_panel();

    std::cout << !isFinished_ << std::endl;
    while(!isFinished_) {
        // std::chrono::microseconds(1000);
        std::cout << !isFinished_ << std::endl;
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(*s_cam_);
        glClearColor(0.95f, 0.95f, 0.95f, 1.0f);

        // // Draw the camera pose -> it is fixed 
        draw_current_cam_pose();

        // // draw Mesh!
        // if(*menu_show_mesh_) {
        //     // draw_current_mesh();
        // }

        // // Setzen Sie die Größe des Punktes
        glPointSize(10.0f);

        // // // Zeichnen Sie den Punkt vor der Kamera
        glBegin(GL_POINTS);
        glVertex3f(0.0f, 0.0f, 1.0f); // Der Punkt wird vor der Kamera positioniert (z.B. 1 Einheit vor der Kamera)
        glEnd();
        // // pangolin::glDrawColouredCube();
        pangolin::FinishFrame();

        if (*menu_terminate_ || pangolin::ShouldQuit()) {
            request_terminate();
        }
    }
}

};// Namespace