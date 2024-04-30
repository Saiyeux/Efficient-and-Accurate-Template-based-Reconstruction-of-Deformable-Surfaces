#ifndef GROUNDTRUTH_H
#define GROUNDTRUTH_H

#include <Eigen/Core>
#include <vector>
#include <iostream>


class GroundTruth_compare {
public:
    GroundTruth_compare();
    virtual void compareWithGroundTruth(std::vector<Eigen::Vector3d> vertices, std::vector<Eigen::Vector3i> triangles) {};

    std::vector<double> all_mean_;
private:

};

#endif // GROUNDTRUTH_H