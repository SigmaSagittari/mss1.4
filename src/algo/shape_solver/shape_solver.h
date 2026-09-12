#pragma once

#include <array>

#include "algo/shape_solver/shape_solver_common.h"

//==============================================================================
#include "algo/shape_solver/dfs_solver.h"
#include "algo/shape_solver/graph_solver.h"

namespace mss {

inline long double ShapeSolver::binom(int n, int k) {
    constexpr int max = 9;
    static constexpr std::array<std::array<long double, max + 1>, max + 1>
        table = [] {
            std::array<std::array<long double, max + 1>, max + 1> result{};
            for (int i = 0; i <= max; ++i) {
                result[i][0] = 1;
                result[i][i] = 1;
                for (int j = 1; j < i; ++j)
                    result[i][j] = result[i - 1][j - 1] + result[i - 1][j];
            }
            return result;
        }();
    return table[n][k];
}

inline ShapeSolver::Distribution::Result::Result(
    int start, int boxCount, std::vector<long double> ways,
    std::vector<long double> perBoxExpectations)
    : start_(start), boxCount_(boxCount), ways_(std::move(ways)),
      perBoxExpectationData_(std::move(perBoxExpectations)) {
    perBoxExpectations_.reserve(ways_.size());
    const std::span<const long double> allExpectations = perBoxExpectationData_;
    for (std::size_t i = 0; i < ways_.size(); ++i)
        perBoxExpectations_.push_back(
            allExpectations.subspan(i * boxCount_, boxCount_));
}

inline DistributionId ShapeSolver::Distribution::Pool::find(U128 hash) const {
    if (const DistributionId* found = index_.find(hash)) return *found;
    return -1;
}

inline DistributionId ShapeSolver::Distribution::Pool::insert(
    U128 hash, ShapeSolver::Distribution::Result result) {
    if (const DistributionId* found = index_.find(hash)) return *found;
    const DistributionId id = static_cast<DistributionId>(results_.size());
    results_.push_back(std::move(result));
    index_.emplace(hash, id);
    return id;
}

inline void ShapeSolver::Distribution::Pool::clear() {
    results_.clear();
    index_.clear();
}

inline DistributionId ShapeSolver::analyze(
    const Structure::Shape& shape, ShapeSolver::Distribution::Pool& pool) {
    if (static_cast<int>(shape.boxes.size()) < graphThreshold)
        return ShapeSolver::DfsSolver::analyze(shape, pool);
    return ShapeSolver::GraphSolver::analyze(
        shape, pool, ShapeSolver::GraphSolver::PolishKind::Adjacent);
}

}  // namespace mss
