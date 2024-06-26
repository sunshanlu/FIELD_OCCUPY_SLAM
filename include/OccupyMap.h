#pragma once

#include "Common.h"
#include "Frame.h"

namespace fos {

class LikehoodField;
class SubMap;

class OccupyMap {
    friend class LikehoodField;
    friend class SubMap;

public:
    struct Point2iCompare {
        bool operator()(const cv::Point2i &lhs, const cv::Point2i &rhs) const {
            return std::tie(lhs.x, lhs.y) < std::tie(rhs.x, rhs.y);
        }
    };

    using Ptr = std::shared_ptr<OccupyMap>;
    using Area = std::multiset<cv::Point2i, Point2iCompare>;

    enum class Method {
        BRESENHAM, ///< bresenham栅格化算法
        TEMPLATE   ///< 模版匹配栅格化算法
    };

    /// @brief 模版法中的模版点定义
    struct TemplatePoint {
        int dx_;      ///< x方向偏移
        int dy_;      ///< y方向偏移
        float range_; ///< 模版点范围(m)
        float angle_; ///< 模版点角度(rad)
    };
    using TemplatePts = std::vector<TemplatePoint>;

    OccupyMap(int width, int height, int resolution, float half_rwid, float half_rhei, Method method, int range = 200)
        : map_(height, width, CV_8U, cv::Scalar(127))
        , origin_(width / 2, height / 2)
        , resolution_(resolution)
        , method_(method)
        , half_rwid_(half_rwid)
        , half_rhei_(half_rhei) {
        if (method_ == Method::TEMPLATE)
            BuildTemplate(range);
    }

    /// 向栅格地图中添加一帧数据，使用时保证frame的Tsb位姿有效
    bool AddFrame(const Frame::Ptr &frame);

    /// 获取栅格地图
    const cv::Mat &GetOccuImg() const { return map_; }

private:
    /// 子地图坐标系转换到栅格坐标系
    cv::Point2i SubMap2Occupy(const Vec2 &Ps) {
        Eigen::Vector2i pt = (Ps * resolution_ + origin_).cast<int>();
        return cv::Point2i(pt.x(), pt.y());
    }

    /// 构建模版
    void BuildTemplate(int Range);

    /// BRESENHAM栅格化算法
    void Bresenham(const Frame::Ptr &frame, const std::vector<cv::Point2i> &EndPts);

    /// 针对单次栅格化，End部分不做操作
    void BresenhamOnce(const Frame::Ptr &frame, const cv::Point2i &Start, const cv::Point2i &End, const int &idx);

    /// 模版匹配栅格化算法，使用懒加载的方式
    void Template(const Frame::Ptr &frame, const Area &EndPts);

    /// 设置栅格点
    bool SetPoint(const int &x, const int &y, bool Occupied);

    /// 输入Pb来判断，因此需要转换到Pb坐标系下
    bool IsNotOccupy(const Vec2 &Pb) {
        if (Pb[0] <= half_rhei_ && Pb[0] >= -half_rhei_ && Pb[1] <= half_rwid_ && Pb[1] >= -half_rwid_) {
            return true;
        }
        return false;
    }

    cv::Mat map_;                 ///< 栅格地图
    Vec2 origin_;                 ///< 地图原点
    int resolution_;              ///< 栅格地图分辨率(px/m)
    Method method_;               ///< 栅格化方法
    TemplatePts template_pts_;    ///< 模版点集合
    float half_rwid_, half_rhei_; ///< 机器人尺寸
};

} // namespace fos