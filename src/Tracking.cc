#include <rclcpp/rclcpp.hpp>

#include "MLikehoodField.h"
#include "Tracking.h"

namespace fos {

Tracking::Tracking(const Options::Ptr &options, Map::Ptr map)
    : options_(options)
    , state_(TrackState::NOT_FRAME_YET)
    , all_maps_(std::move(map))
    , start_reloc_(false) {
    keyframe_ang_th_ = options_->keyframe_ang_th_;
    keyframe_pos_th_ = options_->keyframe_pos_th_;
    keyframe_num_th_ = options_->keyframe_num_th_;
    if (options->only_track_)
        start_reloc_ = true;

    if (!options_->only_track_) {
        curr_map_ = std::make_shared<SubMap>(SE2(), options_);
        all_maps_->all_maps_.push_back(curr_map_);
    }
}

SE2 Tracking::GrabFrame(const Frame::Ptr &frame, bool &track_good) {
    curr_frame_ = frame;

    if (state_ == TrackState::NOT_FRAME_YET) {
        state_ = TrackState::NOT_INIT;
        if (options_->only_track_)
            state_ = TrackState::LOST;
    }

    switch (state_) {
    case TrackState::NOT_INIT:
        Init();
        break;
    case TrackState::TRACKED:
        Track();
        break;
    case TrackState::LOST:
        TrackReloc();
        break;
    default:
        throw std::runtime_error("跟踪状态有误!");
        break;
    }

    switch (state_) {
    case TrackState::TRACKED:
        track_good = true;
        break;

    default:
        track_good = false;
        break;
    }

    last_frame_ = curr_frame_;
    return curr_frame_->GetPose();
}

/**
 * @brief 跟踪线程初始化
 * @details
 *      1. 子地图的栅格地图和似然场初始化
 *      2. 将当前帧作为关键帧添加都子地图中
 */
void Tracking::Init() {
    curr_map_->Update(curr_frame_);
    if (IsKeyFrame()) {
        last_keyframe_ = curr_frame_;
        curr_map_->AddKeyFrame(curr_frame_);
    }
    if (viewer_) {
        viewer_->SetSubMap(curr_map_, all_maps_->all_maps_);
        viewer_->SetFrame(curr_frame_);
    }
    if (loop_closer_) {
        loop_closer_->AddSubMap(curr_map_);
        loop_closer_->AddKeyframe(last_keyframe_);
    }
    state_ = TrackState::TRACKED;
}

/**
 * @brief 当跟踪线程初始化后，调用该函数
 * @details
 *      1. 当前帧配准
 *      2. 判断当前帧是否为关键帧
 *          (1) 如果为关键帧，则添加到子地图中
 *          (2) 如果为关键帧，使用该帧更新当前子地图内容
 *      3. 判断子地图是否发生更新
 *          (1) 如果子地图发生更新，判断是否超出地图边界
 *          (2) 如果超过地图边界，则对地图进行扩展
 */
void Tracking::Track() {
    curr_frame_->SetPoseSub(last_frame_->GetPoseSub() * velocity_);
    curr_frame_->submap_ = curr_map_.get();
    float inlier_ratio = curr_map_->Scan2Map(curr_frame_);
    if (inlier_ratio < options_->reloc_th_) {
        RCLCPP_ERROR(rclcpp::get_logger("fos"), "内点比例为：%.2f", inlier_ratio);
        RCLCPP_ERROR(rclcpp::get_logger("fos"), "跟踪丢失！");
        curr_frame_->SetPose(last_frame_->GetPose());
        state_ = TrackState::LOST;
        return;
    }
    if (viewer_)
        viewer_->SetFrame(curr_frame_);

    if (!options_->only_track_ && IsKeyFrame()) {
        last_keyframe_ = curr_frame_;
        bool bOutRange = curr_map_->Update(curr_frame_);
        if (bOutRange || curr_map_->GetMapSize() > keyframe_num_th_) {
            last_map_ = curr_map_;
            curr_map_ = std::make_shared<SubMap>(curr_frame_->GetPose(), options_);
            all_maps_->all_maps_.push_back(curr_map_);
            curr_map_->AddKeyFrame(curr_frame_);
            curr_frame_->SetPoseSub(SE2());
            curr_map_->Update(curr_frame_);
            curr_map_->ExpandFromOther(last_map_);
            if (viewer_)
                viewer_->SetSubMap(curr_map_, all_maps_->all_maps_);

            if (loop_closer_) {
                loop_closer_->AddSubMap(curr_map_);
                loop_closer_->AddKeyframe(last_keyframe_);
            }

        } else {
            curr_map_->AddKeyFrame(curr_frame_);
            if (loop_closer_)
                loop_closer_->AddKeyframe(last_keyframe_);
        }
    }
    /// 更新速度信息
    if (options_->motion_guss_)
        velocity_ = last_frame_->GetPose().inverse() * curr_frame_->GetPose();
    if (options_->only_track_) {
        if (curr_map_->IsOutRange(curr_frame_))
            SetNearstSubMap();
    }
    state_ = TrackState::TRACKED;
}

/**
 * @brief 判断是否是关键帧函数
 * @details
 *      1. 如果当前帧到上一关键帧的位置距离大于阈值
 *      2. 或者当前帧到上一关键帧的角度距离大于阈值
 *      3. 都返回true
 * @return true     需要插入关键帧
 * @return false    不需要插入关键帧
 */
bool Tracking::IsKeyFrame() {
    if (!last_frame_)
        return true;
    SE2 Tlc = last_keyframe_->GetPose().inverse() * curr_frame_->GetPose();
    if (Tlc.translation().norm() > keyframe_pos_th_ || std::fabs(Tlc.so2().log()) > keyframe_ang_th_)
        return true;
    return false;
}

/**
 * @brief 重定位跟踪，遍历所有子地图，进行金字塔法配准
 * @details
 *      1. 如果配准成功，state_会变成 TRACKED
 *      2. 否则，state_还是为 LOST
 * @note
 *      1. 系统使用两种重定位方式，一种是启动重定位，如果以跟踪模式启动的话，启动重定位与回环闭合等价，同等严格
 *      2. 跟踪重定位，这种重定位方式，是由于不良的子地图切换策略导致的跟踪失败的保障
 */
void Tracking::TrackReloc() {
    auto field = std::make_shared<MLikehoodField>(options_);
    bool reloc_success = false;
    for (const auto &map : all_maps_->all_maps_) {
        SE2 reloc_pose;
        field->ResetField(map->map_, options_->field_range_);
        reloc_success = field->AddKeyframe(reloc_pose, curr_frame_);
        if (reloc_success) {
            curr_map_ = map;
            curr_frame_->submap_ = map.get();
            curr_frame_->SetPoseSub(reloc_pose);
            curr_frame_->SetPose(map->GetPose() * reloc_pose);
            break;
        }
    }
    if (reloc_success) {
        state_ = TrackState::TRACKED;
        if (options_->use_viewer_ && start_reloc_) {
            viewer_->SetFrame(curr_frame_);
            viewer_->SetSubMaps(curr_map_, all_maps_->all_maps_);
            start_reloc_ = false;
        }
    }
}

/**
 * @brief 将与当前关键帧最近的子地图作为当前关键帧的子地图
 *
 */
void Tracking::SetNearstSubMap() {
    std::vector<SubMap::Ptr> candidates;
    int id = curr_map_->GetID();
    int id_sub = id - 1, id_add = id + 1;
    if (id_sub >= 0)
        candidates.push_back(all_maps_->all_maps_[id_sub]);
    if (id_add < all_maps_->all_maps_.size())
        candidates.push_back(all_maps_->all_maps_[id_add]);
    candidates.push_back(curr_map_);
    float min_distace = std::numeric_limits<float>::max();
    for (auto &candidate : candidates) {
        const SE2 &Tws = candidate->GetPose();
        const SE2 &Twb = curr_frame_->GetPose();
        float distance = (Tws.translation() - Twb.translation()).lpNorm<2>();
        if (distance < min_distace) {
            min_distace = distance;
            curr_map_ = candidate;
        }
    }
    curr_frame_->SetPoseSub(curr_map_->GetPose().inverse() * curr_frame_->GetPose());
    curr_frame_->submap_ = curr_map_.get();
    viewer_->SetSubMap(curr_map_, all_maps_->all_maps_);
}

} // namespace fos
