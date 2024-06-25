#include "System.h"
#include "Frame.h"

namespace fos {

System::System(std::string path)
    : Node("fos") {
    options_ = std::make_shared<Options>(path);
    tracker_ = std::make_shared<Tracking>(options_);
    if (options_->use_viewer_) {
        viewer_ = std::make_shared<Viewer>();
        tracker_->SetViewer(viewer_);
    }
}

SE2 System::GrabLaserScan(LaserScan::SharedPtr scan) {
    Frame::Ptr frame = Frame::Create(scan);
    SE2 pose = tracker_->GrabFrame(frame);
    RCLCPP_INFO(get_logger(), "位姿为：%.2f %.2f %.2f", pose.translation().x(), pose.translation().y(),
                pose.so2().log());
    return pose;
}

} // namespace fos
