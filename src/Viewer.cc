#include "Viewer.h"

namespace fos {

/// @brief 可视化线程入口
void Viewer::Run() {
    while (!resq_stop_.load()) {
        std::unique_lock<std::mutex> lock1(sub_mutex_, std::defer_lock);
        std::unique_lock<std::mutex> lock2(frame_mutex_, std::defer_lock);
        std::lock(lock1, lock2);
        auto ret = submap_->GetMapImg();
        cv::Mat SubMapImg = DrawSubMap(ret.second, ret.first);
        cv::imshow("SubMap", SubMapImg);
        cv::waitKey(10);
    }
}

/**
 * @brief 绘制子地图
 *
 * @param FieldMap  输入的似然场图（CV_32FC1）
 * @param OccupyMap 输入的栅格地图（CV_8UC1）
 * @return cv::Mat  输出的绘制结果（CV_8UC3）
 */
cv::Mat Viewer::DrawSubMap(cv::Mat FieldMap, cv::Mat OccupyMap) {
    cv::Mat OccupyBW(OccupyMap.rows, OccupyMap.cols, CV_8UC3, cv::Scalar(127, 127, 127));
    for (int row = 0; row < OccupyMap.rows; ++row) {
        for (int col = 0; col < OccupyMap.cols; ++col) {
            if (OccupyMap.at<uchar>(row, col) < 127)
                OccupyBW.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);
            else if (OccupyMap.at<uchar>(row, col) > 127)
                OccupyBW.at<cv::Vec3b>(row, col) = cv::Vec3b(255, 255, 255);
        }
    }

    cv::Mat FieldImg, SubMapImg;
    FieldMap.convertTo(FieldImg, CV_8UC3);
    cv::hconcat(FieldMap, OccupyBW, SubMapImg);
    cv::putText(SubMapImg, "SubMap " + std::to_string(submap_->GetID()), cv::Point2f(20, 20), cv::FONT_HERSHEY_COMPLEX,
                0.5, cv::Scalar(0, 255, 0));
    cv::putText(SubMapImg, "Keyframes " + std::to_string(submap_->GetMapSize()), cv::Point2f(20, 50),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0));
    DrawFrame(SubMapImg, FieldImg.cols);
    return SubMapImg;
}

/**
 * @brief 绘制雷达帧到子地图中去
 *
 * @param SubMapImg
 */
void Viewer::DrawFrame(cv::Mat &SubMapImg, const int &fcol) {
    for (int i = 0, rnum = frame_->points_base_.size(); i < rnum; i++) {
        Vec2 Pw = frame_->GetPose() * frame_->points_base_[i];
        cv::Point2i PsField = submap_->World2Sub(Pw);
        cv::Point2i PsOccupy(PsField.x + fcol, PsField.y);
        if (PsOccupy.x > 0 && PsOccupy.x < SubMapImg.cols && PsOccupy.y > 0 && PsOccupy.y < SubMapImg.rows)
            SubMapImg.at<cv::Vec3b>(PsOccupy) = cv::Vec3b(0, 0, 255);

        if (PsField.x > 0 && PsField.x < SubMapImg.cols && PsField.y > 0 && PsField.y < SubMapImg.rows)
            SubMapImg.at<cv::Vec3b>(PsField) = cv::Vec3b(0, 0, 255);
    }
}

} // namespace fos