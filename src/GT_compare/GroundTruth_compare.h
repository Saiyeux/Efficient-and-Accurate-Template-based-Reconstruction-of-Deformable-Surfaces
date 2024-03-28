#ifndef GROUNDTRUTH_H
#define GROUNDTRUTH_H

#include <iostream>
#include <open3d/Open3D.h>

class GroundTruth_compare {
public:
    GroundTruth_compare();
    virtual void compareWithGroundTruth(std::shared_ptr<open3d::geometry::TriangleMesh> mesh) {};
private:

};

#endif // GROUNDTRUTH_H