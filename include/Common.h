#pragma once
#include <memory>
#include <thread>

#include <Eigen/Core>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sophus/se2.hpp>

namespace fos {

class Frame;
using FramePtr = std::shared_ptr<Frame>;

class OccupyMap;
using OccupyMapPtr = std::shared_ptr<OccupyMap>;

class SubMap;
using SubMapPtr = std::shared_ptr<SubMap>;

using LaserScan = sensor_msgs::msg::LaserScan;
using SE2 = Sophus::SE2f;
using Vec2 = Eigen::Vector2f;
using Vec3 = Eigen::Vector3f;

using ThreadPtr = std::shared_ptr<std::thread>;

using BlockSolverX = g2o::BlockSolverX;
using LinearSolverX = g2o::LinearSolverEigen<BlockSolverX::PoseMatrixType>;
using Optimizer = g2o::SparseOptimizer;

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
    bool use_viewer_;
    int gp_max_size_;
    bool motion_guss_;
    bool use_loopcloser_;
    int layer_num_;
    float layer_ratio_;
    float inlier_ratio_;
    std::vector<float> error_th_;
    float distance_th_;
    int submap_gap_;
    float loop_rk_delta_;
    float max_range_;
    float delta_angle_;
    float reloc_th_;
    bool save_map_;
    bool load_map_;
    std::string map_fp_;
    bool only_track_;
};

} // namespace fos
