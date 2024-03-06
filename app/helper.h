#include <opencv2/opencv.hpp>
#include <Eigen/Core>

void verifyPoints(std::vector<double> &xyz, bool *usable_point, int &number_vertices, Eigen::Matrix3d K, cv::Mat mask);
void remapping(std::vector<double> &xyz, std::vector<int> &faces, double *vertices, bool *usabale_faces, bool *usable_point, int &number_faces, int &num_points);

