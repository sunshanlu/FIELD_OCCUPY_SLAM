#include <yaml-cpp/yaml.h>

#include "Common.h"

namespace fos {

Options::Options(const std::string &path) {
    YAML::Node Config = YAML::LoadFile(path);
    if (!Config)
        throw std::runtime_error("Failed to load config file");
    
    YAML::Node submap = Config["submap"];
    YAML::Node tracking = Config["tracking"];
    if (!submap || !tracking)
        throw std::runtime_error("Invalid config file");
    
    width_ = submap["width"].as<int>();
    height_ = submap["height"].as<int>();
    resolution_ = submap["resolution"].as<int>();
    robot_width_ = submap["robot_width"].as<float>();
    robot_height_ = submap["robot_height"].as<float>();
    method_ = submap["method"].as<int>();
    occupy_range_ = submap["occupy_range"].as<int>();
    field_range_ = submap["field_range"].as<int>();

    keyframe_pos_th_ = tracking["keyframe_pos_th"].as<float>();
    keyframe_ang_th_ = tracking["keyframe_ang_th"].as<int>() * M_PI / 180.0;
    keyframe_num_th_ = tracking["keyframe_num_th"].as<int>();
}

} // namespace fos