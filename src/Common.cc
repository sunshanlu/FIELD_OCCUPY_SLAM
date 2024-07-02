#include <yaml-cpp/yaml.h>

#include "Common.h"

namespace fos {

Options::Options(const std::string &path) {
    YAML::Node Config = YAML::LoadFile(path);
    if (!Config)
        throw std::runtime_error("Failed to load config file");

    YAML::Node submap = Config["submap"];
    YAML::Node tracking = Config["tracking"];
    YAML::Node viewer = Config["viewer"];
    YAML::Node loopcloser = Config["loopcloser"];
    YAML::Node laser = Config["laser"];
    YAML::Node map = Config["map"];

    if (!submap || !tracking || !viewer || !loopcloser || !laser || !map)
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
    motion_guss_ = tracking["motion_guss"].as<bool>();
    reloc_th_ = tracking["reloc_th"].as<float>();
    only_track_ = tracking["only_track"].as<bool>();

    use_viewer_ = viewer["use_viewer"].as<bool>();
    gp_max_size_ = viewer["gp_max_size"].as<int>();

    use_loopcloser_ = loopcloser["use_loopcloser"].as<bool>();
    layer_num_ = loopcloser["layer_num"].as<int>();
    layer_ratio_ = loopcloser["layer_ratio"].as<float>();
    inlier_ratio_ = loopcloser["inlier_ratio"].as<float>();
    YAML::Node errors = loopcloser["error_th"];
    for (auto it = errors.begin(); it != errors.end(); it++)
        error_th_.push_back(it->as<float>());
    distance_th_ = loopcloser["distance_th"].as<float>();
    submap_gap_ = loopcloser["submap_gap"].as<int>();
    loop_rk_delta_ = loopcloser["loop_rk_delta"].as<float>();

    max_range_ = laser["max_range"].as<float>();
    delta_angle_ = laser["delta_angle"].as<float>();

    save_map_ = map["save_map"].as<bool>();
    load_map_ = map["load_map"].as<bool>();
    map_fp_ = map["map_fp"].as<std::string>();
}

} // namespace fos