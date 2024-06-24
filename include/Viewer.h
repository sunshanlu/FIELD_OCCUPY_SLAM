#pragma once

#include "Common.h"
#include "Frame.h"

namespace fos {

class Viewer {
public:
private:
    cv::Mat field_;
    cv::Mat occu_;
    Frame::Ptr frame_;
};

} // namespace fos