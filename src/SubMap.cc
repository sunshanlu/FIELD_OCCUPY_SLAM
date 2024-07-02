#include <execution>

#include <rclcpp/rclcpp.hpp>

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
             option->field_range_) {
    options_ = option;
}

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
float SubMap::Scan2Map(const Frame::Ptr &frame) { return field_->AddFrame(frame, pose_, options_->error_th_[0]); }

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
Vec2 SubMap::Sub2World(const cv::Point2i &Ps) { return pose_ * ((Vec2(Ps.x, Ps.y) - origin_) / resolution_); }

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

/// 将地图信息保存到指定文件中
void Map::SaveMap(const std::string &fp) {
    std::ofstream ofs(fp + "/map_info.txt");
    std::vector<int> compression_param;
    compression_param.push_back(cv::IMWRITE_PNG_COMPRESSION);
    compression_param.push_back(0);
    int ret = system(("mkdir -p " + fp + "/submaps").c_str());
    if (ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("fos"), "保存地图时出现问题！");
        return;
    }
    for (const auto &map : all_maps_) {
        auto imgs = map->GetMapImg();
        int map_id = map->GetID();
        const SE2 &Tws = map->GetPose();
        cv::imwrite(fp + "/submaps/" + std::to_string(map_id) + ".png", imgs.first, compression_param);
        ofs << map_id << " " << Tws.translation().x() << " " << Tws.translation().y() << " " << Tws.so2().log()
            << std::endl;
    }
}

/// 将地图信息加载到内存中
void Map::LoadMap(const std::string &fp) {
    std::ifstream ifs(fp + "/map_info.txt");
    std::string line_str;
    std::vector<int> indices;
    while (std::getline(ifs, line_str)) {
        int id;
        float x, y, theta;
        std::stringstream ss(line_str);
        ss >> id >> x >> y >> theta;
        cv::Mat occupy_img = cv::imread(fp + "/submaps/" + std::to_string(id) + ".png", cv::IMREAD_UNCHANGED);
        SE2 Tws(theta, Vec2(x, y));
        auto submap = std::make_shared<SubMap>(Tws, options_);
        submap->map_->SetOccupyImg(occupy_img);
        all_maps_.push_back(submap);
        indices.push_back(id);
    }
    std::for_each(std::execution::par_unseq, indices.begin(), indices.end(), [&](const int &id) {
        all_maps_[id]->field_->ResetField(all_maps_[id]->map_, options_->field_range_);
    });
}

/// 是否超出地图范围
bool SubMap::IsOutRange(const Frame::Ptr &frame) {
    const SE2 &Tsb = frame->GetPoseSub();
    for (const auto &pb : frame->points_base_) {
        Eigen::Vector2i pp = ((Tsb * pb) * resolution_ + origin_).cast<int>();
        if (pp[0] < 0 || pp[0] > width_ || pp[1] < 0 || pp[1] > height_)
            return true;
    }
    return false;
}

/// SubMap的静态变量
int SubMap::next_id_ = 0;

} // namespace fos
