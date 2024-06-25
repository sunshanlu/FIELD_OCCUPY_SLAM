#pragma once

#include <atomic>
#include <mutex>

#include "Common.h"
#include "Frame.h"
#include "SubMap.h"

namespace fos {

class Viewer {
public:
    using Ptr = std::shared_ptr<Viewer>;

    Viewer()
        : resq_stop_(false) {
        /// 允许保持窗口比例的缩放
        cv::namedWindow("SubMap", cv::WINDOW_KEEPRATIO);
        cv::namedWindow("Map", cv::WINDOW_KEEPRATIO);
    }

    ~Viewer() { cv::destroyAllWindows(); }

    /// 设置子地图
    void SetSubMap(SubMap::Ptr submap) {
        std::lock_guard<std::mutex> lock(sub_mutex_);
        submap_ = std::move(submap);
    }

    /// 设置当前雷达帧
    void SetFrame(Frame::Ptr frame) {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        frame_ = std::move(frame);
    }

    /// 请求可视化线程停止
    void RequestStop() { resq_stop_.store(true); }

    void Run();

private:
    /// 绘制子地图
    cv::Mat DrawSubMap(cv::Mat field, cv::Mat occupy);

    /// 将雷达帧绘制到子地图上
    void DrawFrame(cv::Mat &SubMapImg, const int &fcol);

    SubMap::Ptr submap_;          ///< 维护的子地图
    Frame::Ptr frame_;            ///< 维护的当前帧
    std::mutex sub_mutex_;        ///< 子地图互斥锁
    std::mutex frame_mutex_;      ///< 当前帧互斥锁
    std::atomic<bool> resq_stop_; ///< 是否有外部命令停止
};

} // namespace fos