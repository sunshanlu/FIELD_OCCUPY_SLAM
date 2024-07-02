#pragma once

#include <unordered_map>

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include "Common.h"
#include "Frame.h"
#include "OccupyMap.h"

namespace fos {

/// @brief 似然场类
class LikehoodField {
    friend class FieldEdge;
    friend class MLikehoodField;
    using BlockSolverX = g2o::BlockSolverX;
    using LinearSolverX = g2o::LinearSolverEigen<BlockSolverX::PoseMatrixType>;

public:
    struct ModelPoint {
        using BuiltModelPts = std::unordered_map<int, cv::Mat>;

        /// 根据range大小创建点模版
        static cv::Mat CreateModelPt(int range);

        static BuiltModelPts built_models_; ///< 构建过的似然场模版
    };

public:
    using Ptr = std::shared_ptr<LikehoodField>;

    LikehoodField() = default;

    /// 对frame进行配准，要求此时frame的pose_sub_肯定肯定有效
    float AddFrame(const Frame::Ptr &frame, const SE2 &Tws, const float &rk_delta);

    /// 以frame重置似然场，其中似然场的位姿与frame相同
    void ResetField(const Frame::Ptr &frame, int range = 20, float resolution = 20, int width = 1000,
                    int height = 1000);

    /// 以栅格地图重置似然场，其中似然场的位姿与栅格地图相同
    void ResetField(const OccupyMap::Ptr &map, int range = 20, float ratio = 1.0);

    /// 输入子地图中的点，返回对应在似然场中的像素坐标
    cv::Point2i SubMap2Field(const Vec2 &Ps) const {
        Eigen::Vector2i pt = (Ps * resolution_ + origin_).cast<int>();
        return cv::Point2i(pt.x(), pt.y());
    }

    /// 似然场坐标转换为子地图物理坐标
    Vec2 Field2SubMap(const cv::Point2i &Pf) const { return (Vec2(Pf.x, Pf.y) - origin_) / resolution_; }

    /// 获取似然场图像
    const cv::Mat &GetFieldImg() const { return field_; }

private:
    /// 使用g2o的方式进行似然场优化配准
    float AlignG2O(const std::vector<Vec2> &scan_, SE2 &pose, float rk_delta);

    void SetField(cv::Mat &template_pt, const int &row, const int &col);

    cv::Mat field_;    ///< 似然场
    float resolution_; ///< 似然场分辨率 (px/m)
    Vec2 origin_;      ///< 似然场中心点
    int range_;        ///< 点模版宽度
};

/// @brief SE2优化顶点
class SE2Vertex : public g2o::BaseVertex<3, SE2> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool read(std::istream &is) override { return false; }

    bool write(std::ostream &os) const override { return false; }

    void setToOriginImpl() override { _estimate = SE2(); }

    void oplusImpl(const double *update) override;
};

/// @brief  似然场误差边，定义了残差计算方法和雅可比矩阵
class FieldEdge : public g2o::BaseUnaryEdge<1, Vec2, SE2Vertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit FieldEdge(LikehoodField *field)
        : field_(field) {}

    bool read(std::istream &is) override { return false; }

    bool write(std::ostream &os) const override { return false; }

    /// 判断点pt是否在field内
    bool IsValid(const cv::Point2i &pt) const {
        return pt.x < field_->field_.cols && pt.y < field_->field_.rows && pt.x >= 0 && pt.y >= 0;
    }

    /// 计算残差，即似然场值，_measurement是Pb
    void computeError() override;

    void linearizeOplus() override;

private:
    const LikehoodField *field_; ///< 误差边值指向的似然场
    cv::Point2i pf_;             ///< Pb对应似然场的坐标Pf
    cv::Point2i pf_add_x_;       ///< Pf在x方向加1
    cv::Point2i pf_sub_x_;       ///< Pf在x方向减1
    cv::Point2i pf_add_y_;       ///< Pf在y方向加1
    cv::Point2i pf_sub_y_;       ///< Pf在y方向减1
};

} // namespace fos