#ifndef HAMLYNGT_H
#define HAMLYNGT_H


#include "GT_compare/GroundTruth_compare.h"
// #include <iostream>
// #include <open3d/Open3D.h>

class GroundTruth;

class HamlynGT : public GroundTruth_compare {
public:
    HamlynGT();
    void compareWithGroundTruth(std::shared_ptr<open3d::geometry::TriangleMesh> mesh) override;
private:

};

#endif // HAMLYNGT_H