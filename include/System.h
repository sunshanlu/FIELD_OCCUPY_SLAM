#pragma once

#include <rclcpp/rclcpp.hpp>

#include "Common.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "Viewer.h"

namespace fos {

class System : public rclcpp::Node {
public:
    System(std::string path);

    /// 处理雷达扫描数据
    SE2 GrabLaserScan(LaserScan::SharedPtr scan, bool &track_good);

    /// 运行一次回环闭合检测，用于调试或者串联运行
    void RunLoopClose() {
        if (options_->use_loopcloser_)
            loop_closer_->RunOnce();
    }

    ~System() {
        if (options_->use_viewer_) {
            viewer_->RequestStop();
            if (viewer_thread_->joinable())
                viewer_thread_->join();
        }
        if (options_->use_loopcloser_) {
            loop_closer_->RequestStop();
            if (loop_closer_thread_->joinable())
                loop_closer_thread_->join();
        }
        if (options_->save_map_)
            map_->SaveMap(options_->map_fp_);
    }

private:
    Options::Ptr options_;         ///< 配置器
    Tracking::Ptr tracker_;        ///< 跟踪器
    Viewer::Ptr viewer_;           ///< 可视化器
    LoopClosing::Ptr loop_closer_; ///< 回环闭合检测器
    ThreadPtr viewer_thread_;      ///< 可视化线程
    ThreadPtr loop_closer_thread_; ///< 回环闭合线程
    Map::Ptr map_;                 ///< 全局地图
};

} // namespace fos