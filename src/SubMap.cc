#include "SubMap.h"

namespace fos {

SubMap::SubMap(SE2 pose, int width, int height, int resolution, float half_rwid, float half_rhei,
               OccupyMap::Method method, int range_map, int range_field)
    : pose_(std::move(pose))
    , range_(range_field)
    , resolution_(resolution)
    , origin_(width / 2, height / 2)
    , range_field_(range_field)
    , width_(width)
    , height_(height) {
    field_ = std::make_shared<LikehoodField>();
    map_ = std::make_shared<OccupyMap>(width, height, resolution, half_rwid, half_rhei, method, range_map);
    id_ = next_id_++;
}

/// 使用委托构造函数
SubMap::SubMap(SE2 pose, const Options::Ptr &option)
    : SubMap(pose, option->width_, option->height_, option->resolution_, option->robot_width_ / 2.f,
             option->robot_height_ / 2.f,
             option->method_ == 0 ? OccupyMap::Method::BRESENHAM : OccupyMap::Method::TEMPLATE, option->occupy_range_,
             option->field_range_) {}

/**
 * @brief 基于优化成功的帧进行栅格地图更新
 *
 * @param frame     输入的优化成功的关键帧
 * @return true     在更新地图的过程中出现栅格范围以外的点
 * @return false    在更新地图的过程中没有出现栅格范围以外的点
 */
bool SubMap::Update(const Frame::Ptr &frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    bool bOutRange = map_->AddFrame(frame);
    if (!bOutRange)
        field_->ResetField(map_, range_);
    return bOutRange;
}

/**
 * @brief 使用似然场法进行子地图的扫描配准
 *
 * @param frame 输入的雷达帧
 */
void SubMap::Scan2Map(const Frame::Ptr &frame) { field_->AddFrame(frame, pose_); }

/**
 * @brief 为了使得子地图不会那么空，使用上一个子地图的最新10个关键帧
 *
 * @param other 与当前子地图最近的一个子地图
 */
void SubMap::ExpandFromOther(const SubMap::Ptr &other) {
    for (int i = other->keyframes_.size() - 10; i < other->keyframes_.size(); i++) {
        if (i > 0)
            continue;
        Frame::Ptr frame = std::make_shared<Frame>(other->keyframes_[i]);
        frame->SetPoseSub(pose_.inverse() * frame->GetPose());
        map_->AddFrame(frame);
        field_->ResetField(map_, range_);
    }
}

/**
 * @brief 获取SubMap的地图图像
 * @details
 *      1. 似然场在左，栅格图在右
 * @return cv::Mat 输出的SubMap地图图像
 */
std::pair<cv::Mat, cv::Mat> SubMap::GetMapImg() {
    std::lock_guard<std::mutex> lock(mutex_);
    const cv::Mat &OccupyMap = map_->GetOccuImg();
    const cv::Mat &LikelihoodMap = field_->GetFieldImg();
    return std::make_pair(OccupyMap, LikelihoodMap);
}

/**
 * @brief 将世界坐标系下的点转换到子地图坐标系下
 *
 * @param Pw 世界坐标系下的点
 * @return cv::Point2i 输出的子地图坐标系下的点
 */
cv::Point2i SubMap::World2Sub(const Vec2 &Pw) {
    Eigen::Vector2i Ps = (resolution_ * (pose_.inverse() * Pw) + origin_).cast<int>();
    return cv::Point2i(Ps[0], Ps[1]);
}

/**
 * @brief 将子地图坐标系下的点转换到世界坐标系下
 *
 * @param Ps    输入的子地图坐标系下的点
 * @return Vec2 输出的世界坐标系下的点
 */
Vec2 SubMap::Sub2World(const cv::Point2i &Ps) { 
    return pose_ * ((Vec2(Ps.x, Ps.y) - origin_) / resolution_); 
}

/**
 * @brief 获取子地图的边界，用于全局地图可视化
 *
 * @param minx 最小x
 * @param maxx 最大x
 * @param miny 最小y
 * @param maxy 最大y
 */
void SubMap::GetBound(float &minx, float &maxx, float &miny, float &maxy) {
    Vec2 Pw1 = Sub2World(cv::Point2i(0, 0));
    Vec2 Pw2 = Sub2World(cv::Point2i(0, height_ - 1));
    Vec2 Pw3 = Sub2World(cv::Point2i(width_ - 1, 0));
    Vec2 Pw4 = Sub2World(cv::Point2i(width_ - 1, height_ - 1));
    minx = std::min({Pw1[0], Pw2[0], Pw3[0], Pw4[0]});
    maxx = std::max({Pw1[0], Pw2[0], Pw3[0], Pw4[0]});
    miny = std::min({Pw1[1], Pw2[1], Pw3[1], Pw4[1]});
    maxy = std::max({Pw1[1], Pw2[1], Pw3[1], Pw4[1]});
}

/// SubMap的静态变量
int SubMap::next_id_ = 0;

} // namespace fos
