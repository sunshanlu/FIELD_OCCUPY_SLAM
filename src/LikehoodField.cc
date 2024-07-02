#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <rclcpp/rclcpp.hpp>

#include "LikehoodField.h"

namespace fos {

/**
 * @brief 创建似然场模版点函数，以距离方程作为场函数（最小化即可）
 *
 * @param range 模版点范围
 * @return const cv::Mat 输出的模版点
 */
cv::Mat LikehoodField::ModelPoint::CreateModelPt(int range) {
    auto iter = built_models_.find(range);
    if (iter != built_models_.end())
        return iter->second;
    cv::Mat model(2 * range + 1, 2 * range + 1, CV_32F);
    for (int i = -range; i <= range; ++i) {
        for (int j = -range; j <= range; ++j)
            model.at<float>(i + range, j + range) = std::sqrt(i * i + j * j);
    }
    built_models_.insert({range, model});
    return model;
}

/**
 * @brief 使用g2o的方式进行scan配准
 *
 * @param scan  输入的扫描点云数据Pb
 * @param pose  输入的初始化位姿Tsb
 * @return float    输出的内点比例
 */
float LikehoodField::AlignG2O(const std::vector<Vec2> &scan, SE2 &pose, float rk_delta) {
    g2o::SparseOptimizer optimizer;
    auto lm =
        new g2o::OptimizationAlgorithmLevenberg(std::make_unique<BlockSolverX>(std::make_unique<LinearSolverX>()));
    optimizer.setAlgorithm(lm);

    auto v = new SE2Vertex;
    v->setId(0);
    v->setEstimate(pose);
    optimizer.addVertex(v);

    int EdgeID = 0;

    std::vector<FieldEdge *> edges(scan.size(), nullptr);

    for (int i = 0, rnum = scan.size(); i < rnum; i++) {
        auto e = new FieldEdge(this);
        e->setId(EdgeID++);
        e->setVertex(0, v);
        e->setMeasurement(scan[i]);
        e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        auto rk = new g2o::RobustKernelHuber;
        rk->setDelta(rk_delta);
        e->setRobustKernel(rk);
        optimizer.addEdge(e);
        edges[i] = e;
    }

    optimizer.initializeOptimization(0);
    optimizer.setVerbose(false);
    optimizer.optimize(10);

    int nliner = 0;
    int n = 0;
    for (auto &e : edges) {
        if (e->level() == 1)
            continue;
        if (e->chi2() < rk_delta)
            ++nliner;
        ++n;
    }

    std::vector<float> errors;
    float chi2 = 0.f;
    if (nliner / (float)(n + 1e-3) < 0.1) {
        for (const auto &e : edges)
            errors.push_back(e->chi2());
        std::sort(errors.begin(), errors.end());
        chi2 = std::accumulate(errors.begin(), errors.end(), 0);
        RCLCPP_INFO(rclcpp::get_logger("fos"), "总误差为: %.2f", chi2);
    }

    pose = v->estimate();
    return nliner / (float)(n + 1e-3);
}

/**
 * @brief 似然场配准优化函数
 *
 * @param frame 输入的雷达帧
 * @param Tws   输入的子地图位姿
 * @return float 优化内点的比例
 */
float LikehoodField::AddFrame(const Frame::Ptr &frame, const SE2 &Tws, const float &rk_delta) {

    SE2 Tsb = frame->GetPoseSub();
    float ratio = AlignG2O(frame->points_base_, Tsb, rk_delta);

    frame->SetPoseSub(Tsb);
    frame->SetPose(Tws * Tsb);
    return ratio;
}

/// SE2的更新函数
void SE2Vertex::oplusImpl(const double *update) {
    if (std::isnan(update[2]) || std::isnan(update[1]) || std::isnan(update[0])) {
        setFixed(true);
        return;
    }
    _estimate.translation().x() += update[0];
    _estimate.translation().y() += update[1];
    _estimate.so2() *= Sophus::SO2f::exp(update[2]);
}

/// 计算残差，即似然场值，_measurement是Pb
void FieldEdge::computeError() {
    SE2Vertex *v = dynamic_cast<SE2Vertex *>(_vertices[0]);
    pf_ = field_->SubMap2Field(v->estimate() * _measurement);
    pf_add_x_ = cv::Point2i(pf_.x + 1, pf_.y);
    pf_add_y_ = cv::Point2i(pf_.x, pf_.y + 1);
    pf_sub_x_ = cv::Point2i(pf_.x - 1, pf_.y);
    pf_sub_y_ = cv::Point2i(pf_.x, pf_.y - 1);
    if (IsValid(pf_) && IsValid(pf_add_x_) && IsValid(pf_add_y_) && IsValid(pf_sub_x_) && IsValid(pf_sub_y_))
        _error[0] = field_->field_.at<float>(pf_.y, pf_.x);
    else {
        _error[0] = 0;
        setLevel(1);
    }
}

/// 计算雅可比
void FieldEdge::linearizeOplus() {
    SE2Vertex *v = dynamic_cast<SE2Vertex *>(_vertices[0]);
    const auto &Tsb = v->estimate();
    float theta = Tsb.so2().log();
    float cost = cos(theta);
    float sint = sin(theta);
    const float &xb = _measurement.x();
    const float &yb = _measurement.y();
    float pi_x = (field_->field_.at<float>(pf_add_x_) - field_->field_.at<float>(pf_sub_x_)) / 2;
    float pi_y = (field_->field_.at<float>(pf_add_y_) - field_->field_.at<float>(pf_sub_y_)) / 2;
    _jacobianOplusXi[0] = field_->resolution_ * pi_x;
    _jacobianOplusXi[1] = field_->resolution_ * pi_y;
    _jacobianOplusXi[2] = field_->resolution_ * (pi_x * (-yb * cost - xb * sint) + pi_y * (xb * cost - yb * sint));
}

/// 以frame重置似然场，其中似然场的位姿与frame相同
void LikehoodField::ResetField(const Frame::Ptr &frame, int range, float resolution, int width, int height) {
    range_ = range;
    resolution_ = resolution;
    origin_[0] = width / 2;
    origin_[1] = height / 2;
    field_ = cv::Mat(height, width, CV_32F, cv::Scalar(sqrt(2.f * range_ * range_)));
    auto &ranges = frame->scan_->ranges;
    cv::Mat TemplatePt = ModelPoint::CreateModelPt(range_);
    for (int i = 0, rnum = ranges.size(); i < rnum; ++i) {
        if (ranges[i] < frame->scan_->range_min || ranges[i] > frame->scan_->range_max)
            continue;
        float angle = frame->scan_->angle_min + i * frame->scan_->angle_increment;
        float x = ranges[i] * cos(angle);
        float y = ranges[i] * sin(angle);
        cv::Point2i p = SubMap2Field(Vec2(x, y));
        SetField(TemplatePt, p.y, p.x);
    }
}

/**
 * @brief 根据给定的不同缩放比例设置似然场
 *
 * @param map   输入的栅格地图
 * @param ratio 输入的缩小比例
 * @param range 输入的似然场模版范围
 */
void LikehoodField::ResetField(const OccupyMap::Ptr &map, int range, float ratio) {
    range_ = range;
    resolution_ = map->resolution_ / ratio;
    int width = std::round(map->map_.cols / ratio);
    int height = std::round(map->map_.rows / ratio);
    origin_[0] = width / 2.f;
    origin_[1] = height / 2.f;
    cv::Mat TemplatePt = ModelPoint::CreateModelPt(range_);
    field_ = cv::Mat(height, width, CV_32F, cv::Scalar(sqrt(2.f) * range_));
    for (int row = 0; row < map->map_.rows; ++row) {
        for (int col = 0; col < map->map_.cols; ++col) {
            cv::Point2i pf(col / ratio, row / ratio);
            if (pf.x < 0 || pf.x >= width || pf.y < 0 || pf.y >= height)
                continue;
            if (map->map_.at<uchar>(row, col) < 127)
                SetField(TemplatePt, pf.y, pf.x);
        }
    }
}

/// 设置场函数，只有小的部分才被设置
void LikehoodField::SetField(cv::Mat &template_pt, const int &row, const int &col) {
    for (int dy = -range_; dy <= range_; ++dy) {
        for (int dx = -range_; dx <= range_; ++dx) {
            const float &tval = template_pt.at<float>(dy + range_, dx + range_);
            int real_x = col + dx, real_y = row + dy;
            if (real_x < 0 || real_x >= field_.cols || real_y < 0 || real_y >= field_.rows)
                continue;
            float &fval = field_.at<float>(real_y, real_x);
            if (tval < fval)
                fval = tval;
        }
    }
}

/// LikehoodField::ModelPoint的静态成员
LikehoodField::ModelPoint::BuiltModelPts LikehoodField::ModelPoint::built_models_; ///< 构建过的似然场模版

} // namespace fos