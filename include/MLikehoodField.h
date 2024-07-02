#pragma once

#include "LikehoodField.h"

namespace fos {

class MLikehoodField {
public:
    using Ptr = std::shared_ptr<MLikehoodField>;
    using LikehoodFields = std::vector<LikehoodField::Ptr>;

    MLikehoodField(Options::Ptr options)
        : layer_num_(options->layer_num_)
        , layer_ratio_(options->layer_ratio_)
        , options_(std::move(options)) {
        float ratio = 1;
        for (int i = 0; i < layer_num_; i++) {
            if (i == 0) {
                layer_ratios_.push_back(ratio);
                continue;
            }
            ratio *= layer_ratio_;
            layer_ratios_.push_back(ratio);
        }
    }

    /// 根据栅格地图进行多层似然场重置
    void ResetField(const OccupyMap::Ptr &map, int range = 20);

    /// 添加关键帧，用于初值不稳定的配准方式
    bool AddKeyframe(SE2 &Tsb, const Frame::Ptr &frame);

private:
    LikehoodFields fields_;           ///< 似然场
    int layer_num_;                   ///< 金字塔层级数
    float layer_ratio_;               ///< 金字塔层级比例
    std::vector<float> layer_ratios_; ///< 金字塔层级比例
    Options::Ptr options_;            ///< 参数指针
};

} // namespace fos
