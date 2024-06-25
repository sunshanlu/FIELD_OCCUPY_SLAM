#pragma once

#include <mutex>

#include "Common.h"
#include "Frame.h"
#include "LikehoodField.h"
#include "OccupyMap.h"

namespace fos {

class SubMap {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Keyframes = std::vector<Frame::Ptr>;
    using Ptr = std::shared_ptr<SubMap>;

    /// 构造函数
    SubMap(SE2 pose, int width = 1000, int height = 1000, int resolution = 20, float half_rwid = 0.3,
           float half_rhei = 0.3, OccupyMap::Method method = OccupyMap::Method::BRESENHAM, int range_map = 200,
           int range_field = 20);

    SubMap(SE2 pose, const Options::Ptr &option);

    /// 更新子地图
    bool Update(const Frame::Ptr &frame);

    /// scan2map似然场配准
    void Scan2Map(const Frame::Ptr &frame);

    /// 从其他子地图中拓展目前子地图
    void ExpandFromOther(const SubMap::Ptr &other);

    /// 向子地图中添加关键帧
    void AddKeyFrame(const Frame::Ptr &frame) { keyframes_.push_back(frame); }

    /// 获取地图中关键帧数量
    int GetMapSize() const { return keyframes_.size(); }

    /// 将世界坐标系下的点转换到子地图坐标系下
    cv::Point2i World2Sub(const Vec2 &Pw);

    /// 获取地图图像（左边为似然场，右边为栅格图）
    std::pair<cv::Mat, cv::Mat> GetMapImg();

    int GetID() const { return id_; }

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
};

} // namespace fos
