#include "GT_compare/HamlynGT.h"



HamlynGT::HamlynGT(const YAML::Node &config) : config_(config) {
    uint FrameNo_;
    FPS_ = config["Hamlyn"]["FPS"].as<int>();
    mutliplier_ = config["Hamlyn"]["multiplier"].as<double>();
    modulo_  = config["Hamlyn"]["modulo"].as<int>();
    addition_ = config["Hamlyn"]["addition"].as<double>(); 
}

void HamlynGT::compareWithGroundTruth(std::shared_ptr<open3d::geometry::TriangleMesh> mesh) {
    std::cout << "test\n\n\n\n" << std::endl;
}