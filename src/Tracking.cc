#include "Tracking.h"

namespace fos {

Tracking::Tracking(const Options::Ptr &options)
    : options_(options)
    , state_(TrackState::NOT_FRAME_YET) {
    keyframe_ang_th_ = options_->keyframe_ang_th_;
    keyframe_pos_th_ = options_->keyframe_pos_th_;
    keyframe_num_th_ = options_->keyframe_num_th_;
    curr_map_ = std::make_shared<SubMap>(SE2(), options_);
}

SE2 Tracking::GrabFrame(const Frame::Ptr &frame) {
    curr_frame_ = frame;
    if (viewer_)
        viewer_->SetFrame(curr_frame_);
    state_ = TrackState::NOT_INIT;
    switch (state_) {
    case TrackState::NOT_INIT:
        Init();
        break;
    case TrackState::TRACKED:
        Track();
        break;

    default:
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
    curr_map_->Scan2Map(curr_frame_);
    bool bkf = IsKeyFrame();
    if (bkf) {
        last_keyframe_ = curr_frame_;
        bool bOutRange = curr_map_->Update(curr_frame_);
        if (bOutRange || curr_map_->GetMapSize() > keyframe_num_th_) {
            last_map_ = curr_map_;
            curr_map_ = std::make_shared<SubMap>(curr_frame_->GetPose(), options_);
            if (viewer_)
                viewer_->SetSubMap(curr_map_);
            all_maps_.push_back(curr_map_);
            curr_map_->AddKeyFrame(curr_frame_);
            curr_frame_->SetPoseSub(SE2());
            curr_map_->Update(curr_frame_);
            curr_map_->ExpandFromOther(last_map_);
        } else
            curr_map_->AddKeyFrame(curr_frame_);
    }
    /// 更新速度信息
    velocity_ = last_frame_->GetPose().inverse() * curr_frame_->GetPose();
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

} // namespace fos
