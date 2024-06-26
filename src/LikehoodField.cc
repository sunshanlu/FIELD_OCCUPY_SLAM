#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>

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
 * @brief 似然场配准优化函数
 *
 * @param frame 输入的雷达帧
 * @param Tws   输入的子地图位姿
 */
void LikehoodField::AddFrame(const Frame::Ptr &frame, const SE2 &Tws) {
    g2o::SparseOptimizer optimizer;
    auto lm =
        new g2o::OptimizationAlgorithmLevenberg(std::make_unique<BlockSolverX>(std::make_unique<LinearSolverX>()));
    optimizer.setAlgorithm(lm);

    auto v = new SE2Vertex;
    v->setId(0);
    v->setEstimate(frame->GetPoseSub());
    optimizer.addVertex(v);

    int EdgeID = 0;
    const double rk_delta = 0.8;

    for (int i = 0, rnum = frame->points_base_.size(); i < rnum; i++) {
        auto e = new FieldEdge(this);
        e->setId(EdgeID++);
        e->setVertex(0, v);
        e->setMeasurement(frame->points_base_[i]);
        e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        auto rk = new g2o::RobustKernelHuber;
        rk->setDelta(rk_delta);
        e->setRobustKernel(rk);
        optimizer.addEdge(e);
    }

    optimizer.initializeOptimization(0);
    optimizer.setVerbose(false);
    optimizer.optimize(10);

    frame->SetPoseSub(v->estimate());
    frame->SetPose(Tws * v->estimate());
}

/// SE2的更新函数
void SE2Vertex::oplusImpl(const double *update) {
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
        _error[0] = -1;
        setLevel(1);
    }
}

/// 计算雅可比
void FieldEdge::linearizeOplus() {
    if (_error[0] == -1)
        return;
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
void LikehoodField::ResetField(const Frame::Ptr &frame, int range, int resolution, int width, int height) {
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
        cv::Rect rect(p.x - 20, p.y - 20, 2 * range_ + 1, 2 * range_ + 1);
        bool c1 = rect.contains(cv::Point2i(0, 0));
        bool c2 = rect.contains(cv::Point2i(width, 0));
        bool c3 = rect.contains(cv::Point2i(0, height));
        bool c4 = rect.contains(cv::Point2i(width, height));
        if (c1 || c2 || c3 || c4)
            continue;
        cv::Mat roi = field_(rect);
        SetField(TemplatePt, roi);
    }
}

/// 以栅格地图重置似然场，其中似然场的位姿与栅格地图相同
void LikehoodField::ResetField(const OccupyMap::Ptr &map, int range) {
    range_ = range;
    resolution_ = map->resolution_;
    origin_ = map->origin_;
    int width = map->map_.cols;
    int height = map->map_.rows;
    cv::Mat TemplatePt = ModelPoint::CreateModelPt(range_);
    field_ = cv::Mat(map->map_.rows, map->map_.cols, CV_32F, cv::Scalar(sqrt(2.f * range_ * range_)));
    for (int row = 0; row < map->map_.rows; ++row) {
        for (int col = 0; col < map->map_.cols; ++col) {
            if (map->map_.at<uchar>(row, col) < 127) {
                cv::Point2i p(col, row);
                cv::Point2i lu(p.x - range_, p.y - range_);
                cv::Point2i ld(p.x - range_, p.y + range_);
                cv::Point2i ru(p.x + range_, p.y - range_);
                cv::Point2i rd(p.x + range_, p.y + range_);
                bool c1 = lu.x < 0 || lu.x >= field_.cols || lu.y < 0 || lu.y >= field_.rows;
                bool c2 = ld.x < 0 || ld.x >= field_.cols || ld.y < 0 || ld.y >= field_.rows;
                bool c3 = ru.x < 0 || ru.x >= field_.cols || ru.y < 0 || ru.y >= field_.rows;
                bool c4 = rd.x < 0 || rd.x >= field_.cols || rd.y < 0 || rd.y >= field_.rows;
                if (c1 || c2 || c3 || c4)
                    continue;
                cv::Rect rect(lu, rd);
                cv::Mat roi = field_(rect);
                SetField(TemplatePt, roi);
            }
        }
    }
}

/// 设置场函数，只有小的部分才被设置
void LikehoodField::SetField(cv::Mat &model, cv::Mat &roi) {
    for (int row = 0; row < model.rows; ++row) {
        for (int col = 0; col < model.cols; ++col) {
            float &vm = model.at<float>(row, col);
            float &vr = roi.at<float>(row, col);
            if (vm < vr)
                vr = vm;
        }
    }
}

/// LikehoodField::ModelPoint的静态成员
LikehoodField::ModelPoint::BuiltModelPts LikehoodField::ModelPoint::built_models_; ///< 构建过的似然场模版

} // namespace fos