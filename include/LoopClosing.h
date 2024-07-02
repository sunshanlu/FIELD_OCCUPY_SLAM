#pragma once

#include <atomic>
#include <mutex>
#include <queue>

#include "Common.h"
#include "Frame.h"
#include "MLikehoodField.h"
#include "SubMap.h"
#include "Viewer.h"

namespace fos {

class LoopClosing {
public:
    using SubMaps = std::vector<SubMap::Ptr>;
    using MLikehoodFields = std::map<int, MLikehoodField::Ptr>;
    using Ptr = std::shared_ptr<LoopClosing>;

    /// <当前子地图, 回环闭合子地图, Tssc>
    using LoopEdges = std::multimap<int, std::pair<int, SE2>>;

    LoopClosing(Options::Ptr options)
        : options_(std::move(options))
        , resq_stop_(false)
        , distance_th_(options_->distance_th_)
        , submap_gap_(options_->submap_gap_)
        , loop_rk_delta_(options_->loop_rk_delta_) {}

    /// 向回环闭合线程中添加关键帧信息
    void AddKeyframe(const Frame::Ptr &frame) {
        std::lock_guard<std::mutex> lock(mutex_frame_);
        keyframes_.push(frame);
    }

    /// 添加子地图信息（当tracking新生成子地图时，调用）
    void AddSubMap(const SubMap::Ptr &submap) {
        std::lock_guard<std::mutex> lock(mutex_submap_);
        if (all_maps_.empty()) {
            all_maps_.insert({submap->GetID(), submap});
            return;
        }
        SubMap::Ptr last_sub = all_maps_.rbegin()->second;
        all_maps_.insert({submap->GetID(), submap});
        auto field = std::make_shared<MLikehoodField>(options_);
        field->ResetField(last_sub->map_, options_->field_range_);
        fields_.insert({last_sub->GetID(), field});
    }

    /// 设置可视化线程
    void SetViewer(Viewer::Ptr viewer) { viewer_ = std::move(viewer); }

    /// 回环闭合线程运行主逻辑
    void Run();

    void RequestStop() { resq_stop_.store(true); }

    /// 运行一次，用于调试
    void RunOnce();

private:
    /// 从关键帧队列中获取关键帧信息
    bool GetKeyframe() {
        std::lock_guard<std::mutex> lock(mutex_frame_);
        if (keyframes_.empty())
            return false;
        curr_keyframe_ = keyframes_.front();
        keyframes_.pop();
        return true;
    }

    /// 检测回环闭合候选子地图
    bool DetectCandidates(std::map<int, SubMap::Ptr> &all_maps);

    /// 在候选关键帧中计算SE2变换，用于位姿图优化
    bool ComputeSE2(std::map<int, SubMap::Ptr> &all_maps, MLikehoodFields &fields);

    /// 当明确检查到回环闭合时，进行位姿图优化
    void Optimize(std::map<int, SubMap::Ptr> &all_maps);

    Frame::Ptr curr_keyframe_;            ///< 当前关键帧
    std::vector<int> candidates_;         ///< 候选子地图
    int candidate_;                       ///< 最新回环闭合子地图
    Options::Ptr options_;                ///< 参数指针
    mutable std::mutex mutex_frame_;      ///< 维护关键帧队列的互斥锁
    mutable std::mutex mutex_submap_;     ///< 维护子地图的互斥锁
    std::queue<Frame::Ptr> keyframes_;    ///< 关键帧队列
    std::atomic<bool> resq_stop_;         ///< 是否请求停止
    std::map<int, SubMap::Ptr> all_maps_; ///< 所有的子地图
    float distance_th_;                   ///< 距离阈值
    int submap_gap_;                      ///< 子地图距离间隔
    LoopEdges loop_submaps_;              ///< 已经产生回环闭合的子地图
    MLikehoodFields fields_;              ///< 多层似然场
    float loop_rk_delta_;                 ///< 位姿图边核函数delta值
    Viewer::Ptr viewer_;                  ///< 可视化线程
};

} // namespace fos