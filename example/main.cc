#include <rclcpp/rclcpp.hpp>

#include "BagIO.h"
#include "System.h"

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
          bool success = true;
          SE2 pose = node->GrabLaserScan(scan, success);
          //   RCLCPP_INFO(node->get_logger(), "位姿为：%.2f %.2f %.2f", pose.translation().x(), pose.translation().y(),
          //               pose.so2().log());
          std::this_thread::sleep_for(30ms);
      }).Go();
    rclcpp::shutdown();

    return 0;
}