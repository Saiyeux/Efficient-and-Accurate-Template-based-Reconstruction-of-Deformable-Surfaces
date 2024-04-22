#include "MeshViewer.h"
#include <pangolin/pangolin.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace MeshViewer {

MeshViewer::MeshViewer() {

}



void MeshViewer::run() {
//     pangolin::CreateWindowAndBind("Map Viewer", 640, 480);

//      // 3D Mouse handler requires depth testing to be enabled
//     glEnable(GL_DEPTH_TEST);

//     // Issue specific OpenGl we might need
//     glEnable(GL_BLEND);
//     glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

//     pangolin::OpenGlMatrix proj = pangolin::ProjectionMatrix(
//         1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000);
//     //  pangolin::OpenGlMatrix proj2 =
//     //  pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000);

//     pangolin::OpenGlRenderState s_cam(
//         proj, pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ,
//                                         (mViewpointX + (3)), mViewpointY + 0.1,
//                                         mViewpointZ + 2, 0.0, -1, 0.0));

//     // Add named OpenGL viewport to window and provide 3D Handler
//     // Output 3D
//     pangolin::View &d_cam1 =
//         pangolin::CreateDisplay()
//             .SetAspect(640.0f / 480.0f)
//             .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1, -1024.0f / 768.0f)
//             .SetHandler(new pangolin::Handler3D(s_cam));

//     // Create an OpenGL Buffer containing the vertices of a triangle
//     pangolin::GlBuffer vbo(pangolin::GlArrayBuffer,
//         std::vector<Eigen::Vector3f>{
//            {-0.5f, -0.5f, 0.0f},
//            { 0.5f, -0.5f, 0.0f },
//            { 0.0f,  0.5f, 0.0f }
//         }
//     );

//     while(true) {
//         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//         d_cam.Activate(s_cam);
//         // Connect the first 3 vertices in the GL Buffer to form a triangle!
//         pangolin::RenderVbo(vbo, GL_TRIANGLES);
//         pangolin::FinishFrame();
//     }
}

};// Namespace