#include "BagIO.h"
#include "Common.h"
#include "Frame.h"
#include "MLikehoodField.h"
#include "OccupyMap.h"

using namespace fos;

int main() {
    auto options = std::make_shared<Options>("/home/rookie-lu/Project/Field_Occupy_SLAM/config.yaml");
    auto field = std::make_shared<MLikehoodField>(options);
    auto map =
        std::make_shared<OccupyMap>(options->width_, options->height_, options->resolution_, options->robot_width_ / 2,
                                    options->robot_height_ / 2, OccupyMap::Method::BRESENHAM);

    BagIO bag_io("/media/rookie-lu/DATA1/Dataset/2dmapping/floor1");
    auto scan = bag_io.GetLaserScan();

    // cv::Mat scan_img(1000, 1000, CV_8UC3, cv::Scalar(127, 127, 127));
    // for (int i = 0; i < scan->ranges.size(); i++){
    //     float angle = scan->angle_min + i * scan->angle_increment;
    //     float x = scan->ranges[i] * cos(angle);
    //     float y = scan->ranges[i] * sin(angle);
    //     cv::Point2i point(x * 20 + 500, y * 20 + 500);
    //     scan_img.at<cv::Vec3b>(point) = cv::Vec3b(0, 0, 0);
    // }
    // cv::imshow("scan", scan_img);
    // cv::waitKey(0);
    // cv::destroyAllWindows();

    auto frame = Frame::Create(scan);
    map->AddFrame(frame);

    field->ResetField(map);

    frame = Frame::Create(bag_io.GetLaserScan());
    SE2 Tsb(0, Vec2(3, 0));
    bool ret = field->AddKeyframe(Tsb, frame);

    return 0;
}