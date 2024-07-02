#include "System.h"
#include "Frame.h"

namespace fos {

System::System(std::string path)
    : Node("fos") {
    options_ = std::make_shared<Options>(path);
    map_ = std::make_shared<Map>(options_);
    if (options_->load_map_)
        map_->LoadMap(options_->map_fp_);
    tracker_ = std::make_shared<Tracking>(options_, map_);
    if (options_->use_viewer_) {
        viewer_ = std::make_shared<Viewer>(options_);
        tracker_->SetViewer(viewer_);
        viewer_thread_ = std::make_shared<std::thread>(&Viewer::Run, viewer_);
    }
    if (options_->use_loopcloser_) {
        loop_closer_ = std::make_shared<LoopClosing>(options_);
        tracker_->SetLoopCloser(loop_closer_);
        if (options_->use_viewer_)
            loop_closer_->SetViewer(viewer_);
        loop_closer_thread_ = std::make_shared<std::thread>(&LoopClosing::Run, loop_closer_);
    }
}

SE2 System::GrabLaserScan(LaserScan::SharedPtr scan, bool &track_good) {
    Frame::Ptr frame = Frame::Create(scan, options_);
    SE2 pose = tracker_->GrabFrame(frame, track_good);
    return pose;
}

} // namespace fos
