#include <execution>

#include "OccupyMap.h"

namespace fos {

/**
 * @brief 将激光数据帧添加到栅格地图中去
 *
 * @param frame     输入的激光数据帧
 * @param method    栅格地图的添加方法
 * @return true     存在在栅格地图外的点
 * @return false    不存在在栅格地图外的点
 */
bool OccupyMap::AddFrame(const Frame::Ptr &frame) {
    std::vector<cv::Point2i> EndPts;
    auto &ranges = frame->scan_->ranges;
    for (int i = 0, rnum = frame->points_base_.size(); i < rnum; ++i)
        EndPts.push_back(SubMap2Occupy(frame->GetPoseSub() * frame->points_base_[i]));
    
    switch (method_) {
    case (Method::BRESENHAM):
        Bresenham(frame, EndPts);
        break;
    case (Method::TEMPLATE):
        Template(frame, Area(EndPts.begin(), EndPts.end()));
        break;
    }

    bool bOut = false;
    std::vector<int> indices(EndPts.size());
    std::for_each(indices.begin(), indices.end(), [id = 0](int &i) mutable { i = id++; });
    std::for_each(std::execution::par_unseq, indices.begin(), indices.end(), [&](const int &idx) {
        const cv::Point2i &pt = EndPts[idx];
        bool bOccupy = true;
        if (IsNotOccupy(frame->points_base_[idx]))
            bOccupy = false;

        if (!SetPoint(pt.x, pt.y, bOccupy))
            bOut = true;
    });
    return bOut;
}

/**
 * @brief 对整个激光帧进行并发的栅格化处理
 *
 * @param frame     输入的激光帧
 * @param EndPts    输入的激光束的终点坐标（栅格地图坐标系下）
 */
void OccupyMap::Bresenham(const Frame::Ptr &frame, const std::vector<cv::Point2i> &EndPts) {
    cv::Point2i Start = SubMap2Occupy(frame->GetPoseSub().translation());
    std::vector<int> indices(EndPts.size());
    std::for_each(indices.begin(), indices.end(), [id = 0](int &i) mutable { i = id++; });
    std::for_each(std::execution::par_unseq, indices.begin(), indices.end(),
                  [&](const int &idx) { BresenhamOnce(frame, Start, EndPts[idx], idx); });
}

/**
 * @brief BRESENHAM栅格化算法，针对起始点和终止点
 * @note 在使用此算法之前，要求frame保存越界激光束部分的索引是有效的（resize）
 * @param frame     输入的激光雷达帧
 * @param Start     输入的起始点（栅格地图坐标系）
 * @param End       输入的终止点（栅格地图坐标系）
 * @param idx       输入的激光束索引，以便无锁并发
 */
void OccupyMap::BresenhamOnce(const Frame::Ptr &frame, const cv::Point2i &Start, const cv::Point2i &End,
                              const int &idx) {
    bool bOutRange = false;
    int dx = End.x - Start.x;
    int dy = End.y - Start.y;
    int ux = dx > 0 ? 1 : -1;
    int uy = dy > 0 ? 1 : -1;
    dx = abs(dx), dy = abs(dy);
    if (dx >= dy) {
        int e = -dx;
        for (int x = Start.x, y = Start.y; x != End.x; x += ux) {
            e += 2 * dy;
            if (e > 0) {
                y += uy;
                e -= 2 * dx;
            }
            SetPoint(x, y, false);
        }
    } else {
        int e = -dy;
        for (int y = Start.y, x = Start.x; y != End.y; y += uy) {
            e += 2 * dx;
            if (e > 0) {
                x += ux;
                e -= 2 * dy;
            }
            SetPoint(x, y, false);
        }
    }
}

/**
 * @brief 模版法实现雷达帧的栅格化添加
 *
 * @param frame     输入的雷达帧
 * @param EndPts    输入的雷达帧端点
 */
void OccupyMap::Template(const Frame::Ptr &frame, const Area &EndPts) {
    cv::Point2i Start = SubMap2Occupy(frame->GetPoseSub().translation());
    std::vector<int> indices(template_pts_.size());
    std::for_each(indices.begin(), indices.end(), [idx = 0](int &i) mutable { i = idx++; });
    std::for_each(std::execution::par_unseq, indices.begin(), indices.end(), [&](const int &idx) {
        auto &tp = template_pts_[idx];
        int range_id = std::min({(int)((tp.angle_ - frame->scan_->angle_min) / frame->scan_->angle_increment),
                                 (int)frame->scan_->ranges.size() - 1});
        int range_id_p = std::min(range_id + 1, (int)frame->scan_->ranges.size() - 1);
        float real_range = 0.f;
        const float &range = frame->scan_->ranges[range_id];
        const float &range_p = frame->scan_->ranges[range_id_p];
        bool rv = frame->RangeVaild(range), rpv = frame->RangeVaild(range_p);
        if (rv && rpv) {
            if (std::fabs(range - range_p) < 0.2) {
                float idf = (tp.angle_ - frame->scan_->angle_min) / frame->scan_->angle_increment;
                int idi = (tp.angle_ - frame->scan_->angle_min) / frame->scan_->angle_increment;
                float s = idf - idi;
                real_range = (1 - s) * range + s * range_p;
            } else
                real_range = range;
        } else if (rv && !rpv) {
            real_range = range;
        } else if (!rv && rpv) {
            real_range = range_p;
        } else
            return;
        cv::Point2i pt(Start.x + tp.dx_, Start.y + tp.dy_);
        if (tp.range_ < real_range && EndPts.find(pt) == EndPts.end())
            SetPoint(pt.x, pt.y, false);
    });
}

/// 构建模版
void OccupyMap::BuildTemplate(int Range) {
    for (int dx = -Range; dx <= Range; ++dx) {
        for (int dy = -Range; dy <= Range; ++dy) {
            float range = std::sqrt(dx * dx + dy * dy);
            float angle = std::atan2(dy, dx);
            template_pts_.push_back({dx, dy, range, angle});
        }
    }
}

/**
 * @brief 设置栅格地图点
 *
 * @param x         输入的栅格x坐标
 * @param y         输入的栅格y坐标
 * @param Occupied  输入的栅格是否被占据flag
 * @return true     在栅格范围内
 * @return false    不在栅格方位内
 */
bool OccupyMap::SetPoint(const int &x, const int &y, bool Occupied) {
    if (x < 0 || x >= map_.cols || y < 0 || y >= map_.rows)
        return false;
    uchar &v = map_.at<uchar>(y, x);

    if (Occupied && v > 117)
        --v;
    else if (!Occupied && v < 137)
        ++v;

    return true;
}

} // namespace fos