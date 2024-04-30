#ifndef MESHDRAWER_H
#define MESHDRAWER_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Viewer {

class MeshDrawer {
    public:
        MeshDrawer();
        void addTextureImage(cv::Mat &im);
        void addVertices(std::vector<Eigen::Vector3d> &vertices);
        void addTriangles(std::vector<Eigen::Vector3i> &triangles);

        void drawMesh(double alpha=1, bool useEdges=true);

    private:
        std::vector<Eigen::Vector3i> triangles_;
        std::vector<Eigen::Vector3d> vertices_;

        cv::Mat texture_;
};
} // Namespace

#endif // MESHDRAWER_H