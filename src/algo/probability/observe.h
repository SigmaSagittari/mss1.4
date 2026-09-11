#pragma once

#include <array>
#include <span>
#include <vector>

#include "algo/probability/probability.h"
#include "algo/shape_solver/dfs_solver.h"
#include "algo/shape_solver/graph_solver.h"

namespace mss::Probability {

struct ObserveTransfer {
    int neighborMines = 0;
    int componentMines = 0;
    long double ways = 0.0L;
};

void buildObserveTable(const Structure::Shape& shape,
                       std::span<const int> adjacentBoxCells, int xBox,
                       std::vector<ObserveTransfer>& out);

// 点开一个隐藏格后的结果分布。
// 下标 0..8 为显示数字，下标 9 为爆炸。
struct ObserveResult {
    std::array<long double, 10> probability = {};
};

// 计算点开 cell 后的结果分布。
// distributions 可被补充组件分布缓存；其内容不代表 observe 的临时状态。
ObserveResult observe(const ObservedBoard::Result& board,
                      const Basic::Result& basic,
                      const Structure::Result& structure,
                      const Structure::ShapePool& shapes,
                      const Result& probability,
                      ShapeSolver::Distribution::Pool& distributions,
                      CellId cell);

}  // namespace mss::Probability

namespace mss::ShapeSolver::DfsSolver {

void buildObserveTable(const Structure::Shape& shape,
                       std::span<const int> adjacentBoxCells, int xBox,
                       std::vector<Probability::ObserveTransfer>& out);

}  // namespace mss::ShapeSolver::DfsSolver

namespace mss::ShapeSolver::GraphSolver {

void buildObserveTable(const Structure::Shape& shape,
                       std::span<const int> adjacentBoxCells, int xBox,
                       std::vector<Probability::ObserveTransfer>& out);

}  // namespace mss::ShapeSolver::GraphSolver
