#include "System.h"
#include "Frame.h"

namespace fos {

System::System(std::string path)
    : Node("fos") {
    options_ = std::make_shared<Options>(path);
    tracker_ = std::make_shared<Tracking>(options_);
    if (options_->use_viewer_) {
        viewer_ = std::make_shared<Viewer>(options_);
        tracker_->SetViewer(viewer_);
        viewer_thread_ = std::make_shared<std::thread>(&Viewer::Run, viewer_);
    }
}

SE2 System::GrabLaserScan(LaserScan::SharedPtr scan) {
    Frame::Ptr frame = Frame::Create(scan);
    SE2 pose = tracker_->GrabFrame(frame);
    return pose;
}

} // namespace fos
