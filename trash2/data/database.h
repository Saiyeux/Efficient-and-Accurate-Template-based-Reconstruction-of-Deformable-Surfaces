#ifndef DATABASE_H
#define DATABASE_H

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>

namespace Viewer {
    class MeshViewer;
}
    class System;

class database {
    public:
        database();

        void setVertices(std::vector<Eigen::Vector3d> vertices);
        void setTriangles(std::vector<Eigen::Vector3i> triangles);
        void setTexture(cv::Mat texture);

        void getVertices(std::vector<Eigen::Vector3d> &vertices);
        void getTriangles(std::vector<Eigen::Vector3i> &triangles);
        void getTexture(cv::Mat &texture);

    private:
        std::vector<Eigen::Vector3d> vertices_;
        std::vector<Eigen::Vector3i> triangles_;
        cv::Mat texture_;

        mutable std::mutex mtx_vertex_;
        mutable std::mutex mtx_texture_;
};

#endif