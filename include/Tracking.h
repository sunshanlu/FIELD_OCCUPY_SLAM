#pragma once

#include "Common.h"
#include "Frame.h"
#include "SubMap.h"

namespace fos {

class Tracking {
public:
    using Ptr = std::shared_ptr<Tracking>;
    using SubMaps = std::vector<SubMap::Ptr>;

    enum class TrackState {
        NOT_FRAME_YET, ///< 还没接收到数据
        NOT_INIT,      ///< 还没有完成初始化
        TRACKED,       ///< 当前帧跟踪成功
        LOST           ///< 跟踪丢失
    };

    Tracking(const Options::Ptr &options);

    /// 处理雷达帧的主逻辑函数
    SE2 GrabFrame(const Frame::Ptr &frame);

private:
    /// 初始化后，调用该函数进行位姿估计
    void Track();

    /// 初始化函数
    void Init();

    /// 判断是否为关键帧
    bool IsKeyFrame();

    Frame::Ptr curr_frame_;    ///< 当前帧
    Frame::Ptr last_frame_;    ///< 上一帧
    Frame::Ptr last_keyframe_; ///< 上一个关键帧
    SubMap::Ptr curr_map_;     ///< 当前子地图
    SubMap::Ptr last_map_;     ///< 当前子地图
    SubMaps all_maps_;         ///< 子地图集合
    TrackState state_;         ///< 跟踪线程状态
    SE2 velocity_;             ///< 速度信息Tlc
    float keyframe_pos_th_;    ///< 关键帧位置阈值
    float keyframe_ang_th_;    ///< 关键帧角度阈值
    int keyframe_num_th_;      ///< 关键帧数量阈值
    Options::Ptr options_;     ///< 参数指针
};

} // namespace fos