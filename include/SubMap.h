#pragma once

#include <mutex>

#include "Common.h"
#include "Frame.h"
#include "LikehoodField.h"
#include "OccupyMap.h"

namespace fos {

class LoopClosing;
class Tracking;
struct Map;

class SubMap {
    friend class LoopClosing;
    friend class Tracking;
    friend class Map;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Keyframes = std::vector<Frame::Ptr>;
    using Ptr = std::shared_ptr<SubMap>;

    /// 构造函数
    SubMap(SE2 pose, int width = 1000, int height = 1000, int resolution = 20, float half_rwid = 0.3,
           float half_rhei = 0.3, OccupyMap::Method method = OccupyMap::Method::BRESENHAM, int range_map = 200,
           int range_field = 20);

    SubMap(SE2 pose, const Options::Ptr &option);

    void SetPose(const SE2 &pose) { pose_ = pose; }

    /// 更新子地图
    bool Update(const Frame::Ptr &frame);

    /// 更新关键帧位姿
    void UpdateFramePose() {
        std::lock_guard<std::mutex> lock(mutex_frame_);
        for (auto &frame : keyframes_)
            frame->SetPose(pose_ * frame->GetPoseSub());
    }

    /// scan2map似然场配准
    float Scan2Map(const Frame::Ptr &frame);

    /// 从其他子地图中拓展目前子地图
    void ExpandFromOther(const SubMap::Ptr &other);

    /// 向子地图中添加关键帧
    void AddKeyFrame(const Frame::Ptr &frame) {
        {
            std::lock_guard<std::mutex> lock(mutex_frame_);
            keyframes_.push_back(frame);
        }
        frame->submap_ = this;
    }

    /// 获取地图中关键帧数量
    int GetMapSize() const { return keyframes_.size(); }

    const SE2 &GetPose() const { return pose_; }

    /// 将世界坐标系下的点转换到子地图坐标系下
    cv::Point2i World2Sub(const Vec2 &Pw);

    /// 地图物理系转换为地图图像系（可设置x偏移量）
    cv::Point2i C2Sub(const Vec2 &Ps, int deltay = 0) {
        Vec2 origin(origin_[0], origin_[1] + deltay);
        Eigen::Vector2i pf = (Ps * resolution_ + origin).cast<int>();
        return cv::Point2i(pf[0], pf[1]);
    }

    /// 将子地图坐标系下的点转换到世界坐标系下
    Vec2 Sub2World(const cv::Point2i &Ps);

    std::vector<Frame::Ptr> GetKeyframes() {
        std::lock_guard<std::mutex> lock(mutex_frame_);
        return keyframes_;
    }

    /// 获取地图图像（左边为似然场，右边为栅格图）
    std::pair<cv::Mat, cv::Mat> GetMapImg();

    /// 获取子地图边界，用于可视化
    void GetBound(float &minx, float &maxx, float &miny, float &maxy);

    /// 判断Ps点是否在子地图范围内
    bool IsValid(const cv::Point2i &pt) {
        if (pt.x < width_ && pt.x >= 0 && pt.y < height_ && pt.y >= 0)
            return true;
        return false;
    }

    /// 获取地图值
    const uchar &GetMapVal(const cv::Point2i &pt) {
        std::unique_lock<std::mutex> lock(mutex_);
        return map_->map_.at<uchar>(pt.y, pt.x);
    }

    int GetID() const { return id_; }

    float GetMaxFieldRange() const {
        static float range = std::sqrt(2.f * range_field_ * range_field_);
        return range;
    }

    /// 判断是否超过submap的地图范围
    bool IsOutRange(const Frame::Ptr &frame);

private:
    SE2 pose_;                 ///< Tws
    Keyframes keyframes_;      ///< 子地图中的关键帧
    LikehoodField::Ptr field_; ///< 似然场
    OccupyMap::Ptr map_;       ///< 栅格地图
    int range_;                ///< 似然场模版尺寸
    int id_;                   ///< 子地图id
    static int next_id_;       ///< 下一个子地图id
    int resolution_;           ///< 子地图分辨率(px/m)
    Vec2 origin_;              ///< 中心点
    std::mutex mutex_;         ///< subMap的互斥锁
    std::mutex mutex_frame_;   ///< 关键帧的互斥锁
    int range_field_;          ///< 似然场点模版尺寸
    int width_, height_;       ///< 宽度和高度
    Options::Ptr options_;     ///< 配置指针
};

/// 地图类，维护所有子地图
struct Map {
    using Ptr = std::shared_ptr<Map>;

    Map(Options::Ptr options)
        : options_(std::move(options)) {}

    void SaveMap(const std::string &fp);

    void LoadMap(const std::string &fp);

    std::vector<SubMap::Ptr> all_maps_; ///< 所有子地图
    Options::Ptr options_;              ///< 配置选项
};

} // namespace fos
