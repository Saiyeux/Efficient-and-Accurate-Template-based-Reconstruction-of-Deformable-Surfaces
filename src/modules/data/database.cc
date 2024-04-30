#include "database.h"
namespace stbr {
database::database() {}

void database::setVertices(std::vector<Eigen::Vector3d> vertices) {
    std::lock_guard<std::mutex> lock(mtx_vertex_);
    vertices_ = vertices;
}

void database::setTriangles(std::vector<Eigen::Vector3i> triangles) {
    triangles_ = triangles;
}

void database::setTexture(cv::Mat texture) {
    std::lock_guard<std::mutex> lock(mtx_texture_);
    texture_ = texture;
}

void database::getVertices(std::vector<Eigen::Vector3d> &vertices) {
    std::lock_guard<std::mutex> lock(mtx_vertex_);
    vertices = vertices_;
}

void database::getTriangles(std::vector<Eigen::Vector3i> &triangles) {
    triangles = triangles_;
}

void database::getTexture(cv::Mat &texture) {
    std::lock_guard<std::mutex> lock(mtx_texture_);
    texture = texture_;
}
} // namespace