#pragma once

#include <rclcpp/rclcpp.hpp>

#include "Common.h"
#include "Tracking.h"
#include "Viewer.h"

namespace fos {

class System : public rclcpp::Node {
public:
    System(std::string path);

    /// 处理雷达扫描数据
    SE2 GrabLaserScan(LaserScan::SharedPtr scan);

private:
    Options::Ptr options_;  ///< 配置器
    Tracking::Ptr tracker_; ///< 跟踪器
    Viewer::Ptr viewer_;    ///< 可视化器
};

} // namespace fos