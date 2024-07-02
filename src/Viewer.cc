#include <execution>

#include "Viewer.h"

using namespace std::chrono_literals;

namespace fos {

/// @brief 可视化线程入口
void Viewer::Run() {
    while (!resq_stop_.load()) {
        if (!submap_ || !frame_) {
            std::this_thread::sleep_for(10ms);
            continue;
        }

        cv::Mat SubMapImg;
        int fcols;
        SubMap::Ptr submap;
        SubMaps submaps;
        Frame::Ptr frame;
        std::multimap<int, int> loop_edges;
        bool update;
        {
            std::lock_guard<std::mutex> lock1(sub_mutex_);
            submap = submap_;
            submaps = all_submaps_;
            update = submap_update_;
            if (submap_update_)
                submap_update_ = false;
        }
        {
            std::lock_guard<std::mutex> lock2(frame_mutex_);
            frame = frame_;
        }
        {
            std::lock_guard<std::mutex> lock3(loop_edge_mutex_);
            loop_edges = loop_edges_;
        }
        auto ret = submap->GetMapImg();
        SubMapImg = DrawSubMap(ret.second, ret.first, submap);
        DrawFrame(SubMapImg, submap, frame);
        DrawRobotSub(SubMapImg, submap, frame);
        cv::Mat global_map = DrawGlobalMap(options_->gp_max_size_, submaps, frame, update, loop_edges);

        cv::imshow("SubMap", SubMapImg);
        cv::imshow("Map", global_map);

        cv::waitKey(30);
    }
}

/**
 * @brief 绘制子地图
 *
 * @param FieldMap  输入的似然场图（CV_32FC1）
 * @param OccupyMap 输入的栅格地图（CV_8UC1）
 * @param submap    输入的待绘制的子地图
 * @return cv::Mat  输出的绘制结果（CV_8UC3）
 */
cv::Mat Viewer::DrawSubMap(cv::Mat FieldMap, cv::Mat OccupyMap, const SubMap::Ptr &submap) {
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
    cv::cvtColor(FieldMap / submap->GetMaxFieldRange(), FieldImg, cv::COLOR_GRAY2BGR);
    FieldImg.convertTo(FieldImg, CV_8UC3, 255.0);
    cv::hconcat(FieldImg, OccupyBW, SubMapImg);
    cv::putText(SubMapImg, "SubMap " + std::to_string(submap->GetID()), cv::Point2f(20, 20), cv::FONT_HERSHEY_COMPLEX,
                0.5, cv::Scalar(68, 24, 12));
    cv::putText(SubMapImg, "Keyframes " + std::to_string(submap->GetMapSize()), cv::Point2f(20, 50),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(68, 24, 12));
    return SubMapImg;
}

/**
 * @brief 绘制雷达帧到子地图中去
 *
 * @param SubMapImg
 */
void Viewer::DrawFrame(cv::Mat &SubMapImg, const SubMap::Ptr &submap, const Frame::Ptr &frame) {
    for (int i = 0, rnum = frame->points_base_.size(); i < rnum; i++) {
        Vec2 Pw = frame->GetPose() * frame->points_base_[i];
        cv::Point2i PsField = submap->World2Sub(Pw);
        cv::Point2i PsOccupy(PsField.x + options_->width_, PsField.y);
        if (PsOccupy.x > 0 && PsOccupy.x < SubMapImg.cols && PsOccupy.y > 0 && PsOccupy.y < SubMapImg.rows)
            SubMapImg.at<cv::Vec3b>(PsOccupy) = cv::Vec3b(0, 0, 255);

        if (PsField.x > 0 && PsField.x < SubMapImg.cols && PsField.y > 0 && PsField.y < SubMapImg.rows)
            SubMapImg.at<cv::Vec3b>(PsField) = cv::Vec3b(0, 0, 255);
    }
}

/**
 * @brief 绘制机器人信息，需要options配置
 *
 * @param SubMapImg 子地图图像
 */
void Viewer::DrawRobotSub(cv::Mat &SubMapImg, const SubMap::Ptr &submap, const Frame::Ptr &frame) {
    const auto &Twb = frame->GetPose();
    DrawRobot(SubMapImg, submap->GetPose().inverse() * Twb,
              [&](const Vec2 &Pc) -> cv::Point2i { return submap->C2Sub(Pc); });

    DrawRobot(SubMapImg, submap->GetPose().inverse() * Twb,
              [&](const Vec2 &Pc) -> cv::Point2i { return submap->C2Sub(Pc, options_->width_); });
}

/**
 * @brief 绘制机器人的一般方法
 * @details
 *      1. 获取机器人在Map物理系下的位姿rpose
 *      2. 将机器人的四个边转换到Map物理系下
 *      3. 然后使用c2img函数，将物理系转换为图像系
 * @param Map       输入的地图
 * @param rpose     输入的机器人在Map物理系下的位姿
 * @param c2img     输入的转换函数
 */
void Viewer::DrawRobot(cv::Mat &Map, const SE2 &rpose, const C2ImgFunc &c2img) {
    const float &w = options_->robot_height_;
    const float &h = options_->robot_width_;

    /// 将机器人的Pb转换到Map物理系下
    Vec2 lu = rpose * Vec2(-w / 2, h / 2);
    Vec2 ld = rpose * Vec2(-w / 2, -h / 2);
    Vec2 ru = rpose * Vec2(w / 2, h / 2);
    Vec2 rd = rpose * Vec2(w / 2, -h / 2);

    cv::Point2i luf = c2img(lu);
    cv::Point2i ldf = c2img(ld);
    cv::Point2i ruf = c2img(ru);
    cv::Point2i rdf = c2img(rd);

    std::vector<cv::Point2i> ptsf = {luf, ldf, rdf, ruf};
    cv::fillConvexPoly(Map, ptsf, cv::Scalar(0, 0, 99));
}

/**
 * @brief 绘制全局地图
 * @details
 *      1. 获取全局地图的物理边界
 *      2. 根据物理边界设置动态分辨率（取x或y方向上较大的值进行设置）
 *      3. 获取物理边界的中心坐标，计算Cx和Cy保证物理中心对应图像中心
 *      4. 遍历全局地图图像
 *          (1) 计算对应的世界坐标系坐标
 *          (2) 将Pw投影到子地图中，判断是否在子地图内，只有大于127或者小于127时，设置值
 * @param max_size  地图的最大尺寸
 * @param submaps   所有子地图
 * @param frame     帧信息
 * @return cv::Mat  输出的全局地图
 */
cv::Mat Viewer::DrawGlobalMap(int max_size, const SubMaps &submaps, const Frame::Ptr &frame, const bool &update,
                              const std::multimap<int, int> &loop_edges) {
    cv::Mat golbal_map(max_size, max_size, CV_8UC3, cv::Scalar(127, 127, 127));

    /// step1: 获取全局地图物理边界
    int kfnum = 0;
    std::vector<SE2> poses;
    for (auto &submap : submaps) {
        kfnum += submap->GetMapSize();
        auto kfs = submap->GetKeyframes();
        for (auto &kf : kfs)
            poses.push_back(kf->GetPose());
    }

    /// step2: 设置动态分辨率
    if (update)
        UpdateGMapSize(max_size, submaps[submaps.size() - 1]);

    /// step4: 遍历全局地图，从submap中获取对应像素值
    std::vector<Eigen::Vector2i> reder_pts;
    reder_pts.reserve(max_size * max_size);
    for (int row = 0; row < max_size; ++row) {
        for (int col = 0; col < max_size; ++col)
            reder_pts.emplace_back(col, row);
    }

    Vec2 c(cx_, cy_);

    std::for_each(std::execution::par_unseq, reder_pts.begin(), reder_pts.end(), [&](const Eigen::Vector2i &pt) {
        Vec2 Pw = (pt.cast<float>() - c) / resolution_;
        for (auto &submap : submaps) {
            cv::Point2i Ps = submap->World2Sub(Pw);
            if (!submap->IsValid(Ps))
                continue;
            const uchar &v = submap->GetMapVal(Ps);
            if (v < 127) {
                golbal_map.at<cv::Vec3b>(pt[1], pt[0]) = cv::Vec3b(0, 0, 0);
                break;
            } else if (v > 127) {
                golbal_map.at<cv::Vec3b>(pt[1], pt[0]) = cv::Vec3b(255, 255, 255);
                break;
            }
        }
    });

    cv::putText(golbal_map, "SubMap num: " + std::to_string(submaps.size()), cv::Point2f(20, 20),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(68, 24, 12));

    cv::putText(golbal_map, "Keyframe num: " + std::to_string(kfnum), cv::Point2f(20, 50), cv::FONT_HERSHEY_COMPLEX,
                0.5, cv::Scalar(68, 24, 12));

    auto c2img = [&](const Vec2 &pt) -> cv::Point2i {
        Eigen::Vector2i pf = (resolution_ * pt + c).cast<int>();
        return cv::Point2i(pf[0], pf[1]);
    };
    /// 绘制机器人
    DrawRobot(golbal_map, frame->GetPose(), c2img);
    DrawTraject(golbal_map, poses, c2img);
    DrawSubAxis(golbal_map, submaps, c2img);
    DrawLoopEdges(golbal_map, loop_edges, submaps, c2img);

    return golbal_map;
}

/**
 * @brief 绘制机器人的移动轨迹，只绘制关键帧的
 *
 * @param Map       输入的待绘制轨迹的地图
 * @param poses     输入的待绘制的位姿
 * @param c2img     输入的物理系转换为图像系函数
 */
void Viewer::DrawTraject(cv::Mat &Map, const std::vector<SE2> &poses, const C2ImgFunc &c2img) {
    if (poses.empty())
        return;
    for (std::size_t i = 0; i < poses.size() - 1; ++i) {
        Vec2 Pw0 = poses[i].translation();
        Vec2 Pw1 = poses[i + 1].translation();
        cv::Point2i Pf0 = c2img(Pw0);
        cv::Point2i Pf1 = c2img(Pw1);
        cv::line(Map, Pf0, Pf1, cv::Scalar(0, 0, 255), 2);
    }
}

/**
 * @brief 绘制子地图坐标系
 *
 * @param map       输入的待绘制的地图
 * @param submaps   输入的子地图列表
 * @param c2img     输入的物理系转为图像系的函数
 */
void Viewer::DrawSubAxis(cv::Mat &map, const SubMaps &submaps, const C2ImgFunc &c2img) {
    for (auto &submap : submaps) {
        const auto &Tws = submap->GetPose();
        Vec2 axis_x(1, 0), axis_y(0, 1);
        axis_x = Tws * axis_x;
        axis_y = Tws * axis_y;
        cv::Point2i ax = c2img(axis_x);
        cv::Point2i ay = c2img(axis_y);
        cv::Point2i origin = c2img(Tws.translation());
        cv::line(map, origin, ax, cv::Scalar(0, 0, 255), 1);
        cv::line(map, origin, ay, cv::Scalar(255, 0, 0), 1);
        cv::putText(map, std::to_string(submap->GetID()), origin, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0));
    }
}

/**
 * @brief 绘制回环边
 *
 * @param map   输入的待绘制的地图
 * @param c2img 输入的物理系转为图像系函数
 */
void Viewer::DrawLoopEdges(cv::Mat &map, const std::multimap<int, int> &loop_edges, const SubMaps &all_submaps,
                           const C2ImgFunc &c2img) {
    int max_id = all_submaps[all_submaps.size() - 1]->GetID();
    for (auto &edge : loop_edges) {
        if (edge.first > max_id || edge.second > max_id)
            continue;
        const SE2 &Tws1 = all_submaps[edge.first]->GetPose();
        const SE2 &Tws2 = all_submaps[edge.second]->GetPose();
        cv::Point2i p1 = c2img(Tws1.translation());
        cv::Point2i p2 = c2img(Tws2.translation());
        cv::line(map, p1, p2, cv::Scalar(255, 0, 0), 2);
    }
}

/**
 * @brief 输入一个submap，以更新全局地图的尺寸信息
 * @details
 *      1. 更新全局地图的长宽信息
 *      2. 更新全局地图的分辨率
 *      3. 更新全局地图的偏移向量，保证图片中心对应物理中心
 * @param max_size
 * @param submap
 */
void Viewer::UpdateGMapSize(const int &max_size, const SubMap::Ptr &submap) {
    float minx, miny, maxx, maxy;
    submap->GetBound(minx, maxx, miny, maxy);
    minx < gminx_ ? gminx_ = minx : 0;
    miny < gminy_ ? gminy_ = miny : 0;
    maxx > gmaxx_ ? gmaxx_ = maxx : 0;
    maxy > gmaxy_ ? gmaxy_ = maxy : 0;
    resolution_ = max_size / std::max({gmaxx_ - gminx_, gmaxy_ - gminy_});

    /// step3: 获取物理边界的中心坐标，计算Cx和Cy保证物理中心对应图像中心
    float cwx = (gmaxx_ + gminx_) / 2;
    float cwy = (gmaxy_ + gminy_) / 2;
    cx_ = max_size / 2.f - resolution_ * cwx;
    cy_ = max_size / 2.f - resolution_ * cwy;
}

} // namespace fos