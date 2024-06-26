#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

#include "BagIO.h"
#include "System.h"

namespace fos {

/**
 * @brief BagIO的构造，需要指定bag文件路径
 * @details
 *      1. StorageOptions结构体，{文件路径，数据格式}
 *      2. ConverterOptions结构体，{输入序列化格式，输出序列化格式}
 * @param BagPath bag文件路径
 */
BagIO::BagIO(const std::string &BagPath) {
    reader_ = std::make_unique<Reader>();
    reader_->open({BagPath, "sqlite3"}, {"cdr", "cdr"});
}

/**
 * @brief BagIO运行主逻辑
 *
 */
void BagIO::Go() {
    while (reader_->has_next()) {
        auto meta_data = reader_->read_next();
        if (!meta_data || meta_data->topic_name != "/pavo_scan_bottom")
            continue;

        auto scan = std::make_shared<LaserScan>();
        rclcpp::Serialization<LaserScan> serializer;
        rclcpp::SerializedMessage sm(*meta_data->serialized_data);
        serializer.deserialize_message(&sm, scan.get());
        laser_callback_(scan);
    }
}

} // namespace fos

using namespace fos;
using namespace std::chrono_literals;

int main(int argc, char **argv) {
    if (argc != 3) {
        RCLCPP_ERROR(rclcpp::get_logger("fos"), "usage: ./run_mapping_2d config_path bag_path");
        return -1;
    }
    rclcpp::init(argc, argv);
    auto node = std::make_shared<System>(argv[1]);

    BagIO io(argv[2]);
    io.SetLaserCallback([&](const LaserScan::SharedPtr &scan) {
          SE2 pose = node->GrabLaserScan(scan);
          RCLCPP_INFO(node->get_logger(), "位姿为：%.2f %.2f %.2f", pose.translation().x(), pose.translation().y(),
                      pose.so2().log());
          std::this_thread::sleep_for(30ms);
      }).Go();
    rclcpp::shutdown();

    return 0;
}