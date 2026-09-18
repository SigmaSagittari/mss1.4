#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <span>
#include <utility>
#include <vector>

#include "algo/probability/probability_external.h"
#include "algo/shape_solver/dfs_solver.h"
#include "algo/shape_solver/graph_solver/graph_solver.h"
#include "core/assert.h"
#include "core/utility/combinatorics.h"

namespace mss {

// 点开一个隐藏格后的结果分布。
// 下标 0..8 为显示数字，下标 9 为爆炸。
struct Probability::ObserveResult {
    std::array<long double, 10> probability = {};
};

// 计算点开 cell 后的结果分布。
// cell 必须是 Hidden；结果下标 9 表示爆炸，0..8 表示点开后数字。
// distributions 可被补充组件分布缓存；其内容不代表 observe 的临时状态。

inline void Probability::observePolyMultiply(int leftStart, std::span<const long double> left, int rightStart,
                                             std::span<const long double> right, ObservePoly &out) {
    // 卷积两个点开结果多项式并写入 out。
    const int size = left.size() + right.size() - 1;
    out.coeffs.assign(size, 0.0L);
    for (int i = 0; i < (int)(left.size()); ++i)
        for (int j = 0; j < (int)(right.size()); ++j)
            out.coeffs[i + j] += left[i] * right[j];
    out.start = leftStart + rightStart;
}

inline void Probability::observePolyMultiplyInto(ObservePoly &accumulator, int sourceStart, std::span<const long double> source,
                                                 ObservePoly &mult) {
    // 将 source 多项式乘入 accumulator，并交换临时系数缓冲。
    observePolyMultiply(accumulator.start, accumulator.coeffs, sourceStart, source, mult);
    accumulator.coeffs.swap(mult.coeffs);
    accumulator.start = mult.start;
}

inline long double Probability::observeDenominator(const ObservePoly &polynomial, int totalMines, int tSum) {
    // 计算点开条件下满足总雷数的加权方案数。
    long double result = 0.0L;
    for (int i = 0; i < (int)(polynomial.coeffs.size()); ++i) {
        const int componentMines = polynomial.start + i;
        const int tMines = totalMines - componentMines;
        if (tMines >= 0 && tMines <= tSum)
            result += polynomial.coeffs[i] * combLog(tSum, tMines);
    }
    return result;
}

inline void Probability::buildObserveTable(const Structure::Shape &shape, std::span<const int> adjacentBoxCells, int xBox,
                                           std::vector<ObserveTransfer> &out) {
    // 根据组件规模选择 DFS 或 Graph 后端生成点开转移表；xBox>=0 时排除被点击
    // Box 的具体格子，xBox=-1 表示该组件只通过邻居数字影响点开结果。
    out.clear();
    if (shape.boxes.size() < ShapeSolver::graphThreshold)
        return Probability::buildDfsTable(shape, adjacentBoxCells, xBox, out);
    Probability::buildGraphTable(shape, adjacentBoxCells, xBox, out);
}

inline Probability::ObserveResult Probability::observe(const ObservedBoard::Result &board, const Basic::Result &basic,
                                                       const Structure::Result &structure, const Structure::Pool &shapes,
                                                       const Result &probability, ShapeSolver::Distribution::Pool &distributions,
                                                       CellId cell, const ShapeSolver::OrderAlgo &algo) {
    // 设计目的：这里严格对当前 board/basic/structure/probability 做条件化；这些对象
    // 必须来自同一次分析刷新，才能让点开结果与当前盘面保持一致。
    using Mark = Basic::Mark;
    const auto [x, y] = board.pos(cell);
    const int tSum = basic.unknownSum;
    const int totalMines = board.totalMines - basic.mineSum;
    ObserveResult result;
    assert_(board.board[x][y] == ObservedBoard::CellState::Hidden, "Probability::observe: cell 必须是 Hidden");
    if (basic.marks[x][y] == Mark::Mine) {
        result.probability[9] = 1.0L;
        return result;
    }

    ObserveWorkspace &ws = workspace::ProbabilityObserve::observeWorkspace;
    const bool xInUnknown = basic.marks[x][y] == Mark::Unknown;
    const CellLocation xLocation = structure.cellLoc[cell];
    const bool xInBox = xLocation.component >= 0;
    const ComponentId xComponent = xLocation.component;
    const BoxId xBox = xLocation.box;
    int fixedMines = 0;
    int unknownNeighbors = 0;
    ws.captured.clear();
    ws.seen.assign(structure.components.size(), 0);
    forEachAdjacent(x, y, board.rows, board.cols, [&](int nx, int ny) {
        if (basic.marks[nx][ny] == Mark::Mine) {
            ++fixedMines;
            return;
        }
        if (basic.marks[nx][ny] == Mark::Unknown) {
            ++unknownNeighbors;
            return;
        }
        const CellLocation location = structure.cellLoc[board.id(nx, ny)];
        if (location.component == -1 || ws.seen[location.component])
            return;
        ws.seen[location.component] = 1;
        ws.captured.push_back(location.component);
    });
    assert_(!xInBox || ws.seen[xComponent] != 0, "Probability::observe: x 所在组件未被捕获");

    int maxCapturedMines = unknownNeighbors;
    for (const ComponentId component : ws.captured) {
        const Structure::Instance &instance = shapes.getInstance(structure.components[component]);
        const Structure::Shape &shape = shapes.get(instance.shape);
        for (const Structure::Shape::Box &box : shape.boxes)
            maxCapturedMines += box.size;
    }
    const int stride = maxCapturedMines + 1;
    ws.dp.resize(9, stride, 0.0L);
    ws.dp[0][0] = 1.0L;
    // 把一个组件/Unknown 的联合转移卷入 dp；第一维是点击格邻居雷数，第二维是
    // 已捕获组件的雷数，后续 restWays 再补齐未捕获组件和远端 Unknown。
    auto applyTransfer = [&](const ObserveTransfer &transfer) {
        for (int neighborMines = 0; neighborMines + transfer.neighborMines <= 8; ++neighborMines)
            for (int capturedMines = 0; capturedMines + transfer.componentMines <= maxCapturedMines; ++capturedMines) {
                const long double base = ws.dp[neighborMines][capturedMines];
                if (base == 0.0L)
                    continue;
                ws.nextDp[neighborMines + transfer.neighborMines][capturedMines + transfer.componentMines] += base * transfer.ways;
            }
    };

    // 只捕获与点击格相邻的组件；不相邻组件不会影响点开数字，但仍影响总雷数。
    for (const ComponentId component : ws.captured) {
        const Structure::Instance &instance = shapes.getInstance(structure.components[component]);
        const Structure::Shape &shape = shapes.get(instance.shape);
        ws.adjacentBoxCells.assign(shape.boxes.size(), 0);
        for (int box = 0; box < (int)(shape.boxes.size()); ++box)
            for (int i = instance.boxes.boxOf[box]; i < instance.boxes.boxOf[box + 1]; ++i) {
                const auto [cx, cy] = board.pos(instance.boxes.cells[i]);
                if ((std::abs(cx - x) <= 1) && (std::abs(cy - y) <= 1) && !(cx == x && cy == y))
                    ++ws.adjacentBoxCells[box];
            }
        ws.nextDp.resize(9, stride, 0.0L);
        Probability::buildObserveTable(shape, ws.adjacentBoxCells, component == xComponent ? xBox : -1, ws.transfers);
        for (const ObserveTransfer &transfer : ws.transfers)
            applyTransfer(transfer);
        ws.dp.swap(ws.nextDp);
    }

    if (unknownNeighbors > 0) {
        ws.nextDp.resize(9, stride, 0.0L);
        for (int mines = 0; mines <= unknownNeighbors; ++mines)
            applyTransfer({mines, mines, combLog(unknownNeighbors, mines)});
        ws.dp.swap(ws.nextDp);
    }

    const int tPool = tSum - unknownNeighbors - (xInUnknown ? 1 : 0);
    ws.rest.start = 0;
    ws.rest.coeffs.assign(1, 1.0L);
    // restWays 汇总未捕获组件与剩余 Unknown 的雷数，负责把局部点开事件重新
    // 条件化到整张盘面的 totalMines。
    for (ComponentId component = 0; component < (int)(structure.components.size()); ++component) {
        if (ws.seen[component])
            continue;
        const DistributionId id =
            ShapeSolver::analyze(shapes.get(shapes.getInstance(structure.components[component]).shape), distributions, algo);
        const ShapeSolver::Distribution::Result &distribution = distributions.get(id);
        observePolyMultiplyInto(ws.rest, distribution.start(), distribution.ways(), ws.mult);
    }
    const int restMax = ws.rest.start + ws.rest.coeffs.size() - 1;
    ws.restWays.assign(tPool + restMax + 1, 0.0L);
    for (int i = 0; i < (int)(ws.rest.coeffs.size()); ++i) {
        const long double ways = ws.rest.coeffs[i];
        const int componentMines = ws.rest.start + i;
        for (int tMines = 0; tMines <= tPool; ++tMines)
            ws.restWays[componentMines + tMines] += ways * combLog(tPool, tMines);
    }
    std::array<long double, 9> neighborWays{};
    for (int neighborMines = 0; neighborMines <= 8; ++neighborMines)
        for (int capturedMines = 0; capturedMines <= maxCapturedMines; ++capturedMines) {
            const long double ways = ws.dp[neighborMines][capturedMines];
            const int restMines = totalMines - capturedMines;
            if (ways == 0.0L || restMines < 0 || restMines >= (int)(ws.restWays.size()))
                continue;
            neighborWays[neighborMines] += ways * ws.restWays[restMines];
        }

    // 爆炸项直接复用当前格的全局雷概率；数字项则由邻居雷数分布归一化得到。
    result.probability[9] = probability.mineProbability(cell, board, basic, structure);
    ws.all.coeffs.assign(ws.rest.coeffs.begin(), ws.rest.coeffs.end());
    ws.all.start = ws.rest.start;
    for (const ComponentId component : ws.captured) {
        const DistributionId id =
            ShapeSolver::analyze(shapes.get(shapes.getInstance(structure.components[component]).shape), distributions, algo);
        const ShapeSolver::Distribution::Result &distribution = distributions.get(id);
        observePolyMultiplyInto(ws.all, distribution.start(), distribution.ways(), ws.mult);
    }
    const long double candidates = observeDenominator(ws.all, totalMines, tSum);
    for (int neighborMines = 0; neighborMines + fixedMines <= 8; ++neighborMines)
        result.probability[fixedMines + neighborMines] = neighborWays[neighborMines] / candidates;
    return result;
}

inline void Probability::buildDfsTable(const Structure::Shape &shape, std::span<const int> adjacentBoxCells, int xBox,
                                       std::vector<Probability::ObserveTransfer> &out) {
    // 枚举组件 Box 雷数，统计点开格邻居数字与组件雷数的联合权重。
    int maxMineCount = 0;
    for (const Structure::Shape::Box &box : shape.boxes)
        maxMineCount += box.size;
    using Workspace = workspace::ProbabilityObserve::BuildDfsTable;
    Workspace &dfsWorkspace = workspace::ProbabilityObserve::buildDfsWorkspace;
    std::vector<std::array<long double, 9>> &accumulated = dfsWorkspace.accumulated;
    accumulated.assign(maxMineCount + 1, {});
    ShapeSolver::DfsSolver::forEachAssignment(shape, [&](std::span<const char> assignment, long double weight) {
        int componentMines = 0;
        std::array<long double, 9> convolution{};
        convolution[0] = 1.0L;
        for (int boxId = 0; boxId < (int)(assignment.size()); ++boxId) {
            const int mines = assignment[boxId];
            componentMines += mines;
            const int adjacent = adjacentBoxCells[boxId];
            const bool isXBox = boxId == xBox;
            if (adjacent == 0 && !isXBox)
                continue;
            const int size = shape.boxes[boxId].size;
            const int pool = isXBox ? size - 1 : size;
            std::array<long double, 9> local{};
            const int maxAdjacent = (std::min)(adjacent, mines);
            for (int adjacentMines = 0; adjacentMines <= maxAdjacent; ++adjacentMines) {
                const int remaining = mines - adjacentMines;
                if (remaining > pool - adjacent)
                    continue;
                local[adjacentMines] = combLog(adjacent, adjacentMines) * combLog(pool - adjacent, remaining) / combLog(size, mines);
            }
            std::array<long double, 9> next{};
            for (int h = 0; h <= 8; ++h)
                if (convolution[h] != 0.0L)
                    for (int adjacentMines = 0; adjacentMines <= 8 - h; ++adjacentMines)
                        next[h + adjacentMines] += convolution[h] * local[adjacentMines];
            convolution = next;
        }
        for (int h = 0; h <= 8; ++h)
            if (convolution[h] != 0.0L)
                accumulated[componentMines][h] += weight * convolution[h];
    });
    for (int componentMines = 0; componentMines <= maxMineCount; ++componentMines)
        for (int neighborMines = 0; neighborMines <= 8; ++neighborMines)
            if (accumulated[componentMines][neighborMines] != 0.0L)
                out.push_back({neighborMines, componentMines, accumulated[componentMines][neighborMines]});
}

template <typename Plan>
inline void workspace::ProbabilityObserve::BuildGraphTable::Layer::advance(
    const Plan &plan, workspace::ProbabilityObserve::BuildGraphTable::Layer &nextLayer, std::span<const int> adjacentBoxCells,
    int xBox) const {
    // 按一步 Box 计划推进点开专用 Graph DP，并累计转移权重。
    nextLayer.states.clear();
    nextLayer.counts.clear();
    nextLayer.frontierValues.clear();
    nextLayer.index.clear();
    nextLayer.states.reserve(states.size() * (plan.boxSize + 1));
    nextLayer.counts.reserve(counts.size() * (plan.boxSize + 1));
    nextLayer.frontierValues.reserve(frontierValues.size() + plan.boxSize + 1);
    const int adjacent = adjacentBoxCells[plan.box];
    const bool isXBox = plan.box == xBox;
    const int size = plan.boxSize;
    const int pool = isXBox ? size - 1 : size;
    for (const State &state : states) {
        int minMine = 0;
        int maxMine = plan.boxSize;
        for (const typename Plan::Check &check : plan.checks) {
            int partial = 0;
            for (int i = 0; i < check.readCount; ++i)
                partial += frontierValues[state.frontierOffset + check.readSlots[i]];
            minMine = (std::max)(minMine, check.sum - partial - check.remainingSize);
            maxMine = (std::min)(maxMine, check.sum - partial);
        }
        for (int mine = minMine; mine <= maxMine; ++mine) {
            U128Hasher hasher;
            for (int source : plan.gather) {
                const char value = source < 0 ? mine : frontierValues[state.frontierOffset + source];
                hasher.mix((std::uint64_t)(unsigned char)(value));
            }
            const U128 hash = hasher.finalize();
            State *target;
            if (const std::size_t *found = nextLayer.index.find(hash))
                target = &nextLayer.states[*found];
            else {
                const std::size_t id = nextLayer.states.size();
                nextLayer.states.push_back({nextLayer.frontierValues.size(), -1, -1});
                for (int source : plan.gather)
                    nextLayer.frontierValues.push_back(source < 0 ? mine : frontierValues[state.frontierOffset + source]);
                nextLayer.index.emplace(hash, id);
                target = &nextLayer.states.back();
            }
            if (!isXBox && adjacent == 0) {
                const long double factor = ShapeSolver::binom(size, mine);
                for (int sourceIndex = state.firstCount; sourceIndex >= 0; sourceIndex = counts[sourceIndex].next) {
                    const Count &source = counts[sourceIndex];
                    Count &destination = nextLayer.findOrAddCount(*target, source.componentMines + mine, source.neighborMines);
                    destination.ways += source.ways * factor;
                }
                continue;
            }
            const int maxAdjacent = (std::min)(adjacent, mine);
            for (int neighborMines = 0; neighborMines <= maxAdjacent; ++neighborMines) {
                const int remaining = mine - neighborMines;
                if (remaining > pool - adjacent)
                    continue;
                const long double factor = ShapeSolver::binom(adjacent, neighborMines) * ShapeSolver::binom(pool - adjacent, remaining);
                for (int sourceIndex = state.firstCount; sourceIndex >= 0; sourceIndex = counts[sourceIndex].next) {
                    const Count &source = counts[sourceIndex];
                    if (source.neighborMines + neighborMines > 8)
                        continue;
                    Count &destination =
                        nextLayer.findOrAddCount(*target, source.componentMines + mine, source.neighborMines + neighborMines);
                    destination.ways += source.ways * factor;
                }
            }
        }
    }
}

inline void Probability::buildGraphTable(const Structure::Shape &shape, std::span<const int> adjacentBoxCells, int xBox,
                                         std::vector<Probability::ObserveTransfer> &out) {
    // 使用结构图的消元顺序构建点开专用 Graph DP 转移表。
    const ShapeSolver::GraphSolver::Graph graph = ShapeSolver::GraphSolver::Graph::fromShape(shape);
    const std::vector<BoxId> order = ShapeSolver::GraphSolver::makeOrder(graph, ShapeSolver::GraphSolver::OrderAlgo::Adjacent);
    GraphLayer current;
    GraphLayer next;
    current.reset();
    ShapeSolver::GraphSolver::walkSteps(shape, order, [&](const ShapeSolver::GraphSolver::StepPlan &plan) {
        current.advance(plan, next, adjacentBoxCells, xBox);
        std::swap(current, next);
    });
    current.emit(out);
}

} // namespace mss
