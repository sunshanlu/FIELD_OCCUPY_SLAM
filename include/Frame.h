#pragma once

#include <vector>

#include "Common.h"
namespace fos {

class Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<Frame>;
    using OutSubMapPts = std::vector<std::vector<std::pair<cv::Point2i, bool>>>;

    static Ptr Create(LaserScan::SharedPtr scan, SE2 pose = SE2(), SE2 pose_sub = SE2()) {
        Ptr ptr(new Frame(scan, pose, pose_sub));
        return ptr;
    }

    /// 这里拷贝主要用于子地图Expand
    explicit Frame(const Frame::Ptr &other)
        : scan_(other->scan_)
        , pose_(other->pose_)
        , pose_sub_(other->pose_sub_)
        , id_(other->id_) {}

    ///< 获取Tsb
    const SE2 &GetPoseSub() const { return pose_sub_; }

    /// 获取Twb
    const SE2 &GetPose() const { return pose_; }

    /// 设置Tsb
    void SetPoseSub(const SE2 &pose_sub) { pose_sub_ = pose_sub; }

    /// 设置Twb
    void SetPose(const SE2 &pose) { pose_ = pose; };

    Frame &operator=(const Frame &) = delete;
    Frame(const Frame &other) = delete;

    /// 查看Range是否合法，模版法调用函数
    bool RangeVaild(const float &range) {
        if (range < scan_->range_max && range > scan_->range_min)
            return true;
        return false;
    }

    /// 将(r, theta)极坐标转换为(x, y)笛卡尔坐标
    void ComputePb() {
        if (!points_base_.empty())
            return;
        for (int i = 0, rnum = scan_->ranges.size(); i < rnum; ++i) {
            if (RangeVaild(scan_->ranges[i])) {
                float theta = scan_->angle_min + i * scan_->angle_increment;
                float x = scan_->ranges[i] * cos(theta);
                float y = scan_->ranges[i] * sin(theta);
                points_base_.push_back(Vec2(x, y));
            }
        }
    }

private:
    Frame(LaserScan::SharedPtr scan, SE2 pose, SE2 pose_sub)
        : scan_(std::move(scan))
        , id_(next_id_++)
        , pose_(pose)
        , pose_sub_(pose_sub) {
        ComputePb();
    }

public:
    LaserScan::SharedPtr scan_;     ///< 雷达扫描数据
    std::vector<Vec2> points_base_; ///< 雷达扫描数据的笛卡尔形式

private:
    SE2 pose_;           ///< Twc世界坐标系下的位姿
    SE2 pose_sub_;       ///< 在子地图下的位姿
    static int next_id_; ///< 下一id
    int id_;             ///< 帧id
};

} // namespace fos