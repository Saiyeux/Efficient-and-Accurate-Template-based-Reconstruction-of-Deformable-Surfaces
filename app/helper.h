#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <vector>

int verifyPoints(std::vector<double> &xyz, bool *usable_point, int &number_vertices, Eigen::Matrix3d K, cv::Mat mask);
int remapping(std::vector<double> &xyz, std::vector<int> &faces, std::vector<int> &new_faces, double *fverticess, bool *usabale_faces, bool *usable_point, int &number_faces, int &num_points);
