#include "SubMap.h"

namespace fos {

SubMap::SubMap(SE2 pose, int width, int height, int resolution, float half_rwid, float half_rhei,
               OccupyMap::Method method, int range_map, int range_field)
    : pose_(std::move(pose))
    , range_(range_field) {
    field_ = std::make_shared<LikehoodField>();
    map_ = std::make_shared<OccupyMap>(width, height, resolution, half_rwid, half_rhei, method, range_map);
    id_ = next_id_++;
}

/// 使用委托构造函数
SubMap::SubMap(SE2 pose, const Options::Ptr &option)
    : SubMap(pose, option->width_, option->height_, option->resolution_, option->robot_width_ / 2.f,
             option->robot_height_ / 2.f,
             option->method_ == 0 ? OccupyMap::Method::BRESENHAM : OccupyMap::Method::TEMPLATE, option->occupy_range_,
             option->field_range_) {}

/**
 * @brief 基于优化成功的帧进行栅格地图更新
 *
 * @param frame     输入的优化成功的关键帧
 * @return true     在更新地图的过程中出现栅格范围以外的点
 * @return false    在更新地图的过程中没有出现栅格范围以外的点
 */
bool SubMap::Update(const Frame::Ptr &frame) {
    bool bOutRange = map_->AddFrame(frame);
    if (!bOutRange)
        field_->ResetField(map_, range_);
    return bOutRange;
}

/**
 * @brief 使用似然场法进行子地图的扫描配准
 *
 * @param frame 输入的雷达帧
 */
void SubMap::Scan2Map(const Frame::Ptr &frame) { field_->AddFrame(frame, pose_); }

/**
 * @brief 为了使得子地图不会那么空，使用上一个子地图的最新10个关键帧
 *
 * @param other 与当前子地图最近的一个子地图
 */
void SubMap::ExpandFromOther(const SubMap::Ptr &other) {
    for (int i = other->keyframes_.size() - 10; i < other->keyframes_.size(); i++) {
        if (i > 0)
            continue;
        Frame::Ptr frame = std::make_shared<Frame>(other->keyframes_[i]);
        frame->SetPoseSub(pose_.inverse() * frame->GetPose());
        map_->AddFrame(frame);
        field_->ResetField(map_, range_);
    }
}

} // namespace fos
