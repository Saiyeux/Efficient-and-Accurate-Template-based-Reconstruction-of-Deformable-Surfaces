#include <pangolin/display/display.h>
#include <pangolin/gl/glvbo.h>
#include <pangolin/display/view.h>
#include <pangolin/handler/handler.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

void sample()
{
    pangolin::CreateWindowAndBind("Pango GL Triangle with VBO", 500, 500);

    // Create an OpenGL Buffer containing the vertices of a triangle
    pangolin::GlBuffer vbo(pangolin::GlArrayBuffer,
        std::vector<Eigen::Vector3f>{
           {-0.5f, -0.5f, 0.0f},
           { 0.5f, -0.5f, 0.0f },
           { 0.0f,  0.5f, 0.0f }
        }
    );

    glClearColor(0.64f, 0.5f, 0.81f, 0.0f);
    glColor3f(0.29f, 0.71f, 1.0f);
     pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
        pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
    );

    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);

    while( !pangolin::ShouldQuit() )
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        // Connect the first 3 vertices in the GL Buffer to form a triangle!
        pangolin::RenderVbo(vbo, GL_TRIANGLES);
        pangolin::FinishFrame();
    }
}

int main( int /*argc*/, char** /*argv*/ )
{
    sample();
    return 0;
}