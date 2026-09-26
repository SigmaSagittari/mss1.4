#pragma once

#include "algo/probability_engine/shape_solver/shape_solver_common.h"

//==============================================================================
#include "algo/probability_engine/shape_solver/dfs_solver.h"
#include "algo/probability_engine/shape_solver/graph_solver/graph_solver.h"

namespace mss {

inline ShapeSolver::Distribution::Result::Result(int start, int boxCount, std::vector<long double> ways,
                                                 RawGrid<long double> perBoxExpectations)
    : start_(start), boxCount_(boxCount), ways_(std::move(ways)), perBoxExpectationData_(std::move(perBoxExpectations)) {
    // 为每个总雷数建立指向连续期望值数组的 span。
    perBoxExpectations_.reserve(ways_.size());
    for (int i = 0; i < (int)(ways_.size()); ++i)
        perBoxExpectations_.emplace_back(perBoxExpectationData_[i], boxCount_);
}

inline ShapeSolver::DistributionId ShapeSolver::Distribution::Pool::find(U128 hash) const {
    // 按结构哈希查找分布缓存，不命中时返回 -1。
    if (const ShapeSolver::DistributionId *found = index_.find(hash))
        return *found;
    return -1;
}

inline ShapeSolver::DistributionId ShapeSolver::Distribution::Pool::insert(U128 hash, ShapeSolver::Distribution::Result result) {
    // 将新分布加入缓存，并复用已存在的同哈希结果。
    if (const ShapeSolver::DistributionId *found = index_.find(hash))
        return *found;
    const ShapeSolver::DistributionId id = results_.size();
    results_.push_back(std::move(result));
    index_.emplace(hash, id);
    return id;
}

inline void ShapeSolver::Distribution::Pool::clear() {
    // 清空所有分布结果和索引。
    results_.clear();
    index_.clear();
}

inline ShapeSolver::DistributionId ShapeSolver::analyze(const Structure::Shape &shape, const Structure::Pool &shapes,
                                           ShapeSolver::Distribution::Pool &pool, ShapeSolver::Workspace &workspace, const OrderAlgo &algo) {
    // 根据 Box 数量选择分布求解后端。
    if (shape.boxes.size < graphThreshold)
        return ShapeSolver::DfsSolver::analyze(shape, shapes, pool, workspace.dfsSolver);
    return ShapeSolver::GraphSolver::analyze(shape, shapes, pool, algo, workspace.graphSolver);
}

} // namespace mss
