#pragma once
#include <Eigen/Core>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sophus/se2.hpp>

namespace fos {

class Frame;
using FramePtr = std::shared_ptr<Frame>;

class OccupyMap;
using OccupyMapPtr = std::shared_ptr<OccupyMap>;

using LaserScan = sensor_msgs::msg::LaserScan;
using SE2 = Sophus::SE2f;
using Vec2 = Eigen::Vector2f;
using Vec3 = Eigen::Vector3f;

struct Options {
    using Ptr = std::shared_ptr<Options>;

    Options(const std::string &path);

    int width_;
    int height_;
    int resolution_;
    float robot_width_;
    float robot_height_;
    int method_;
    int occupy_range_;
    int field_range_;
    float keyframe_pos_th_;
    float keyframe_ang_th_;
    int keyframe_num_th_;
};

} // namespace fos
