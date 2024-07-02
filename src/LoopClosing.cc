#include <rclcpp/rclcpp.hpp>

#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/vertex_se2.h>

#include "LoopClosing.h"

namespace fos {
using namespace std::chrono_literals;

void LoopClosing::Run() {
    while (!resq_stop_.load()) {
        RunOnce();
    }
}

void LoopClosing::RunOnce() {
    decltype(all_maps_) all_maps;
    decltype(fields_) fields;
    {
        std::lock_guard<std::mutex> lock(mutex_submap_);
        all_maps = all_maps_;
        fields = fields_;
    }

    if (!GetKeyframe()) {
        std::this_thread::sleep_for(10ms);
        return;
    }

    if (!DetectCandidates(all_maps))
        return;

    if (!ComputeSE2(all_maps, fields))
        return;
    
    Optimize(all_maps);

    /// 向可视化线程中添加回环信息
    if (options_->use_viewer_) {
        std::multimap<int, int> loop_edges;
        for (const auto &edge : loop_submaps_)
            loop_edges.insert({edge.first, edge.second.first});
        viewer_->SetLoopEdges(loop_edges);
    }
}

/**
 * @brief 检测回环闭合候选子地图
 * @details
 *      1. 引入间隔子地图id超参数，要求kf所属子地图id与候选id相差至少id_gap_;
 *      2. 引入距离阈值超参数，要求kf与候选子地图最近点距离大于阈值;
 *      3. 在判断子地图时，同时跳过与当前子地图产生闭环关系的子地图；
 * @return true     检测到了候选子地图
 * @return false    没有检测到候选子地图
 */
bool LoopClosing::DetectCandidates(std::map<int, SubMap::Ptr> &all_maps) {
    candidates_.clear();
    int curr_sub_id_ = curr_keyframe_->submap_->GetID();
    std::set<int> skip_sub_id;

    for (auto iter = loop_submaps_.begin(); iter != loop_submaps_.end(); ++iter) {
        if (iter->first == curr_sub_id_)
            skip_sub_id.insert(iter->second.first);
    }

    for (auto &submap : all_maps) {
        if (skip_sub_id.find(submap.first) != skip_sub_id.end())
            continue;

        if (curr_sub_id_ - submap.first <= submap_gap_)
            continue;
        Vec2 Twb = curr_keyframe_->GetPose().translation();
        Vec2 Tws = submap.second->GetPose().translation();
        float dist = (Twb - Tws).lpNorm<2>();
        if (dist < distance_th_)
            candidates_.push_back(submap.first);
    }
    return !candidates_.empty();
}

/**
 * @brief 计算SE2回环闭合位姿
 *
 * @return true     找到了回环闭合位姿
 * @return false    没有找到回环闭合位姿
 */
bool LoopClosing::ComputeSE2(std::map<int, SubMap::Ptr> &all_maps, MLikehoodFields &fields) {
    for (const auto &candidate : candidates_) {
        SubMap::Ptr submap_can = all_maps[candidate];
        MLikehoodField::Ptr field = fields[candidate];
        SE2 Tscb = submap_can->GetPose().inverse() * curr_keyframe_->GetPose();
        bool ret = field->AddKeyframe(Tscb, curr_keyframe_);
        if (ret) {
            SE2 Tssc = curr_keyframe_->GetPoseSub() * Tscb.inverse();
            candidate_ = curr_keyframe_->submap_->GetID();
            loop_submaps_.insert({curr_keyframe_->submap_->GetID(), {submap_can->GetID(), Tssc}});
            return true;
        }
    }
    return false;
}

/**
 * @brief 当明确检查到回环闭合时，进行位姿图优化
 * @details
 *      1. 相邻子地图位姿约束
 *      2. 闭环地图位姿约束
 */
void LoopClosing::Optimize(std::map<int, SubMap::Ptr> &all_maps) {
    Optimizer optimizer;
    auto lm =
        new g2o::OptimizationAlgorithmLevenberg(std::make_unique<BlockSolverX>(std::make_unique<LinearSolverX>()));
    optimizer.setAlgorithm(lm);

    std::map<int, g2o::VertexSE2 *> vertices;
    std::map<int, g2o::SE2> pose_unopt;
    for (auto &submap : all_maps) {
        const auto &Tws = submap.second->GetPose();
        auto v = new g2o::VertexSE2;
        v->setId(submap.first);
        g2o::SE2 g2o_pose(Tws.translation()[0], Tws.translation()[1], Tws.so2().log());
        v->setEstimate(g2o_pose);
        pose_unopt.insert({submap.first, g2o_pose});
        if (submap.first == candidate_)
            v->setFixed(true);
        optimizer.addVertex(v);
        vertices.insert({submap.first, v});
    }

    /// 1. 相邻子地图位姿约束
    int id1 = all_maps.begin()->first;
    for (auto iter = ++all_maps.begin(); iter != all_maps.end(); ++iter) {
        int id2 = iter->first;
        auto e = new g2o::EdgeSE2;
        auto rk = new g2o::RobustKernelCauchy;
        rk->setDelta(loop_rk_delta_);
        e->setVertex(0, vertices[id1]);
        e->setVertex(1, vertices[id2]);
        SE2 Ts1s2 = all_maps[id1]->GetPose().inverse() * all_maps[id2]->GetPose();
        e->setMeasurement(g2o::SE2(Ts1s2.translation()[0], Ts1s2.translation()[1], Ts1s2.so2().log()));
        e->setInformation(Eigen::Matrix3d::Identity());
        e->setRobustKernel(rk);
        optimizer.addEdge(e);
        id1 = id2;
    }

    /// 2. 闭环地图位姿约束
    std::vector<g2o::EdgeSE2 *> loop_edges; ///< 顺序与loop_submaps_的顺序一致
    for (const auto &loop : loop_submaps_) {
        auto &submap = vertices[loop.first];
        auto &submap_can = vertices[loop.second.first];
        const auto &Tssc = loop.second.second;
        auto e = new g2o::EdgeSE2;
        auto rk = new g2o::RobustKernelCauchy;
        rk->setDelta(loop_rk_delta_);
        e->setVertex(1, submap_can);
        e->setVertex(0, submap);
        e->setMeasurement(g2o::SE2(Tssc.translation()[0], Tssc.translation()[1], Tssc.so2().log()));
        e->setInformation(Eigen::Matrix3d::Identity());
        e->setRobustKernel(rk);
        optimizer.addEdge(e);
        loop_edges.push_back(e);
    }

    std::vector<float> err;
    for (const auto &e : loop_edges) {
        e->computeError();
        err.push_back(e->chi2());
    }

    optimizer.setVerbose(true);
    optimizer.initializeOptimization(0);
    optimizer.optimize(10);

    auto loop_iter = loop_submaps_.begin();
    for (int i = 0; i < loop_edges.size(); ++i) {
        if (loop_edges[i]->chi2() > loop_rk_delta_) {
            loop_edges[i]->setLevel(1);
            loop_iter = loop_submaps_.erase(loop_iter);
        } else
            ++loop_iter;
    }

    for (const auto &pose : pose_unopt)
        vertices.at(pose.first)->setEstimate(pose.second);

    optimizer.optimize(10);
    for (auto &vertex : vertices) {
        g2o::Vector3 Tws = vertex.second->estimate().toVector();
        SubMap::Ptr submap = all_maps[vertex.first];
        submap->SetPose(SE2(Tws[2], Vec2(Tws[0], Tws[1])));
        submap->UpdateFramePose();
    }
}

} // namespace fos