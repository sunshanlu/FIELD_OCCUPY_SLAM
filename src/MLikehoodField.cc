#include <rclcpp/rclcpp.hpp>

#include "MLikehoodField.h"

namespace fos {

/**
 * @brief 根据栅格地图和缩放比例，重新设置多层似然场
 *
 * @param map   输入的栅格地图
 * @param range 输入的似然场模版大小
 */
void MLikehoodField::ResetField(const OccupyMap::Ptr &map, int range) {
    fields_.clear();
    for (const auto &ratio : layer_ratios_) {
        LikehoodField::Ptr layer_field = std::make_shared<LikehoodField>();
        layer_field->ResetField(map, range, ratio);
        fields_.emplace_back(std::move(layer_field));
    }
}

/**
 * @brief 添加关键帧，用于关键帧的初始值不稳定配准方式
 *
 * @param Tsb       输入的关键帧初始位姿
 * @param frame     输入的关键帧
 * @return true     配准成功
 * @return false    配准失败
 */
bool MLikehoodField::AddKeyframe(SE2 &Tsb, const Frame::Ptr &frame) {
    for (int idx = layer_num_ - 1; idx >= 0; --idx) {
        float ratio = fields_[idx]->AlignG2O(frame->points_base_, Tsb, options_->error_th_[idx]);
        if (ratio < options_->inlier_ratio_)
            return false;
    }
    return true;
}

} // namespace fos
