#ifndef HAMLYNGT_H
#define HAMLYNGT_H


#include "GT_compare/GroundTruth_compare.h"
#include <yaml-cpp/yaml.h>
// #include <iostream>
// #include <open3d/Open3D.h>

class GroundTruth;

class HamlynGT : public GroundTruth_compare {
public:
    HamlynGT(const YAML::Node &config);

    void compareWithGroundTruth(std::shared_ptr<open3d::geometry::TriangleMesh> mesh) override;
private:
    const YAML::Node config_;
    int FrameNo_;
    int FPS_;
    double mutliplier_;
    int modulo_;
    double addition_; 
};

#endif // HAMLYNGT_H