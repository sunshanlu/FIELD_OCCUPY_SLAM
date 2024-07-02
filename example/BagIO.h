#pragma once

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "Common.h"

namespace fos {

class BagIO {
public:
    using LaserCallback = std::function<void(const LaserScan::SharedPtr &scan)>;
    using Reader = rosbag2_cpp::readers::SequentialReader;

    explicit BagIO(const std::string &BagPath);

    void Go();

    LaserScan::SharedPtr GetLaserScan();

    BagIO &SetLaserCallback(LaserCallback callback) {
        laser_callback_ = std::move(callback);
        return *this;
    }

private:
    std::unique_ptr<Reader> reader_; ///< rosbag2读取器
    LaserCallback laser_callback_;   ///< 雷达数据处理
};

} // namespace fos
