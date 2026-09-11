#include "algo/probability/observe.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <utility>

#include "algo/shape_solver/dfs_solver.h"
#include "algo/shape_solver/graph_solver.h"
#include "core/assert.h"
#include "core/utility/combinatorics.h"

namespace mss::Probability {

namespace {

struct Poly {
    int start = 0;
    std::vector<long double> coeffs;
};

struct Workspace {
    Poly rest;
    Poly all;
    Poly mult;
    std::vector<ComponentId> captured;
    std::vector<char> seen;
    std::vector<int> adjacentBoxCells;
    std::vector<long double> dp;
    std::vector<long double> nextDp;
    std::vector<long double> restWays;
    std::vector<ObserveTransfer> transfers;
};

thread_local Workspace workspace;

void polyMultiply(int leftStart, std::span<const long double> left,
                  int rightStart, std::span<const long double> right,
                  Poly& out) {
    const int size = static_cast<int>(left.size()) +
                     static_cast<int>(right.size()) - 1;
    out.coeffs.assign(size, 0.0L);
    for (int i = 0; i < static_cast<int>(left.size()); ++i)
        for (int j = 0; j < static_cast<int>(right.size()); ++j)
            out.coeffs[i + j] += left[i] * right[j];
    out.start = leftStart + rightStart;
}

void polyMultiplyInto(Poly& accumulator, int sourceStart,
                      std::span<const long double> source, Poly& mult) {
    polyMultiply(accumulator.start, accumulator.coeffs, sourceStart, source,
                 mult);
    accumulator.coeffs.swap(mult.coeffs);
    accumulator.start = mult.start;
}

long double denominator(const Poly& polynomial, int totalMines, int tSum) {
    long double result = 0.0L;
    for (int i = 0; i < static_cast<int>(polynomial.coeffs.size()); ++i) {
        const int componentMines = polynomial.start + i;
        const int tMines = totalMines - componentMines;
        if (tMines >= 0 && tMines <= tSum)
            result += polynomial.coeffs[i] * combLog(tSum, tMines);
    }
    return result;
}

}  // namespace

void buildObserveTable(const Structure::Shape& shape,
                       std::span<const int> adjacentBoxCells, int xBox,
                       std::vector<ObserveTransfer>& out) {
    out.clear();
    if (static_cast<int>(shape.boxes.size()) < ShapeSolver::graphThreshold)
        return ShapeSolver::DfsSolver::buildObserveTable(
            shape, adjacentBoxCells, xBox, out);
    ShapeSolver::GraphSolver::buildObserveTable(
        shape, adjacentBoxCells, xBox, out);
}

ObserveResult observe(const ObservedBoard::Result& board,
                      const Basic::Result& basic,
                      const Structure::Result& structure,
                      const Structure::ShapePool& shapes,
                      const Result& probability,
                      ShapeSolver::Distribution::Pool& distributions,
                      CellId cell) {
    using Mark = Basic::Mark;
    const auto [x, y] = board.pos(cell);
    const int tSum = basic.unknownSum;
    const int totalMines = board.totalMines - basic.mineSum;
    ObserveResult result;

    assert_(board.board[x][y] == ObservedBoard::CellState::Hidden,
            "Probability::observe: cell 必须是 Hidden");
    if (basic.marks[x][y] == Mark::Mine) {
        result.probability[9] = 1.0L;
        return result;
    }

    Workspace& ws = workspace;
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
        if (location.component == -1 || ws.seen[location.component]) return;
        ws.seen[location.component] = 1;
        ws.captured.push_back(location.component);
    });
    assert_(!xInBox || ws.seen[xComponent] != 0,
            "Probability::observe: x 所在组件未被捕获");

    int maxCapturedMines = unknownNeighbors;
    for (const ComponentId component : ws.captured) {
        const Structure::Instance& instance = structure.components[component];
        const Structure::Shape& shape = shapes.get(instance.shape);
        for (const Structure::Shape::Box& box : shape.boxes)
            maxCapturedMines += box.size;
    }

    const int stride = maxCapturedMines + 1;
    ws.dp.assign(9 * stride, 0.0L);
    ws.dp[0] = 1.0L;
    auto applyTransfer = [&](const ObserveTransfer& transfer) {
        for (int neighborMines = 0;
             neighborMines + transfer.neighborMines <= 8;
             ++neighborMines)
            for (int capturedMines = 0;
                 capturedMines + transfer.componentMines <= maxCapturedMines;
                 ++capturedMines) {
                const long double base =
                    ws.dp[neighborMines * stride + capturedMines];
                if (base == 0.0L) continue;
                ws.nextDp[(neighborMines + transfer.neighborMines) * stride +
                          capturedMines + transfer.componentMines] +=
                    base * transfer.ways;
            }
    };

    for (const ComponentId component : ws.captured) {
        const Structure::Instance& instance = structure.components[component];
        const Structure::Shape& shape = shapes.get(instance.shape);

        ws.adjacentBoxCells.assign(shape.boxes.size(), 0);
        for (std::size_t box = 0; box < shape.boxes.size(); ++box)
            for (std::size_t i = instance.boxes.boxOf[box];
                 i < instance.boxes.boxOf[box + 1]; ++i) {
                const auto [cx, cy] = board.pos(instance.boxes.cells[i]);
                if ((std::abs(cx - x) <= 1) && (std::abs(cy - y) <= 1) &&
                    !(cx == x && cy == y))
                    ++ws.adjacentBoxCells[box];
            }

        ws.nextDp.assign(9 * stride, 0.0L);
        buildObserveTable(shape, ws.adjacentBoxCells,
                          component == xComponent ? static_cast<int>(xBox) : -1,
                          ws.transfers);
        for (const ObserveTransfer& transfer : ws.transfers)
            applyTransfer(transfer);
        ws.dp.swap(ws.nextDp);
    }

    if (unknownNeighbors > 0) {
        ws.nextDp.assign(9 * stride, 0.0L);
        for (int mines = 0; mines <= unknownNeighbors; ++mines)
            applyTransfer({mines, mines, combLog(unknownNeighbors, mines)});
        ws.dp.swap(ws.nextDp);
    }

    const int tPool = tSum - unknownNeighbors - (xInUnknown ? 1 : 0);
    ws.rest.start = 0;
    ws.rest.coeffs.assign(1, 1.0L);
    for (ComponentId component = 0;
         component < static_cast<ComponentId>(structure.components.size());
         ++component) {
        if (ws.seen[component]) continue;
        const DistributionId id = ShapeSolver::analyze(
            shapes.get(structure.components[component].shape), distributions);
        const auto& distribution = distributions.get(id);
        polyMultiplyInto(ws.rest, distribution.start(), distribution.ways(),
                         ws.mult);
    }

    const int restMax = ws.rest.start +
                        static_cast<int>(ws.rest.coeffs.size()) - 1;
    ws.restWays.assign(tPool + restMax + 1, 0.0L);
    for (std::size_t i = 0; i < ws.rest.coeffs.size(); ++i) {
        const long double ways = ws.rest.coeffs[i];
        const int componentMines = ws.rest.start + static_cast<int>(i);
        for (int tMines = 0; tMines <= tPool; ++tMines)
            ws.restWays[componentMines + tMines] +=
                ways * combLog(tPool, tMines);
    }

    std::array<long double, 9> neighborWays{};
    for (int neighborMines = 0; neighborMines <= 8; ++neighborMines)
        for (int capturedMines = 0; capturedMines <= maxCapturedMines;
             ++capturedMines) {
            const long double ways =
                ws.dp[neighborMines * stride + capturedMines];
            const int restMines = totalMines - capturedMines;
            if (ways == 0.0L || restMines < 0 ||
                restMines >= static_cast<int>(ws.restWays.size()))
                continue;
            neighborWays[neighborMines] += ways * ws.restWays[restMines];
        }

    result.probability[9] =
        probability.mineProbability(cell, board, basic, structure);
    ws.all.coeffs.assign(ws.rest.coeffs.begin(), ws.rest.coeffs.end());
    ws.all.start = ws.rest.start;
    for (const ComponentId component : ws.captured) {
        const DistributionId id = ShapeSolver::analyze(
            shapes.get(structure.components[component].shape), distributions);
        const auto& distribution = distributions.get(id);
        polyMultiplyInto(ws.all, distribution.start(), distribution.ways(),
                         ws.mult);
    }
    const long double candidates = denominator(ws.all, totalMines, tSum);
    for (int neighborMines = 0;
         neighborMines + fixedMines <= 8; ++neighborMines)
        result.probability[fixedMines + neighborMines] =
            neighborWays[neighborMines] / candidates;
    return result;
}

}  // namespace mss::Probability

namespace mss::ShapeSolver::DfsSolver {

void buildObserveTable(
    const Structure::Shape& shape, std::span<const int> adjacentBoxCells,
    int xBox, std::vector<Probability::ObserveTransfer>& out) {
    int maxMineCount = 0;
    for (const Structure::Shape::Box& box : shape.boxes)
        maxMineCount += box.size;

    thread_local std::vector<std::array<long double, 9>> accumulated;
    accumulated.assign(maxMineCount + 1, {});

    forEachAssignment(shape, [&](std::span<const char> assignment,
                                 long double weight) {
        int componentMines = 0;
        std::array<long double, 9> convolution{};
        convolution[0] = 1.0L;

        for (std::size_t boxId = 0; boxId < assignment.size(); ++boxId) {
            const int mines = assignment[boxId];
            componentMines += mines;
            const int adjacent = adjacentBoxCells[boxId];
            const bool isXBox = static_cast<int>(boxId) == xBox;
            if (adjacent == 0 && !isXBox) continue;

            const int size = shape.boxes[boxId].size;
            const int pool = isXBox ? size - 1 : size;
            std::array<long double, 9> local{};
            const int maxAdjacent = (std::min)(adjacent, mines);
            for (int adjacentMines = 0;
                 adjacentMines <= maxAdjacent; ++adjacentMines) {
                const int remaining = mines - adjacentMines;
                if (remaining > pool - adjacent) continue;
                local[adjacentMines] =
                    combLog(adjacent, adjacentMines) *
                    combLog(pool - adjacent, remaining) /
                    combLog(size, mines);
            }

            std::array<long double, 9> next{};
            for (int h = 0; h <= 8; ++h)
                if (convolution[h] != 0.0L)
                    for (int adjacentMines = 0; adjacentMines <= 8 - h;
                         ++adjacentMines)
                        next[h + adjacentMines] +=
                            convolution[h] * local[adjacentMines];
            convolution = next;
        }

        for (int h = 0; h <= 8; ++h)
            if (convolution[h] != 0.0L)
                accumulated[componentMines][h] += weight * convolution[h];
    });

    for (int componentMines = 0; componentMines <= maxMineCount;
         ++componentMines)
        for (int neighborMines = 0; neighborMines <= 8; ++neighborMines)
            if (accumulated[componentMines][neighborMines] != 0.0L)
                out.push_back({neighborMines, componentMines,
                               accumulated[componentMines][neighborMines]});
}

}  // namespace mss::ShapeSolver::DfsSolver

namespace mss::ShapeSolver::GraphSolver {

namespace {

using detail::StepPlan;

struct ObserveLayer {
    struct Count {
        int componentMines = 0;
        int neighborMines = 0;
        long double ways = 0.0L;
        int next = -1;
    };

    struct State {
        std::size_t frontierOffset = 0;
        int firstCount = -1;
        int lastCount = -1;
    };

    std::vector<State> states;
    std::vector<Count> counts;
    std::vector<char> frontierValues;
    FlatHashTable<U128, std::size_t, U128Hash> index;

    void reset() {
        states.clear();
        counts.clear();
        frontierValues.clear();
        index.clear();

        states.push_back({0, 0, 0});
        counts.push_back({0, 0, 1.0L, -1});
    }

    Count& findOrAddCount(State& state, int componentMines,
                          int neighborMines) {
        for (int i = state.firstCount; i >= 0; i = counts[i].next)
            if (counts[i].componentMines == componentMines &&
                counts[i].neighborMines == neighborMines)
                return counts[i];

        const int index = static_cast<int>(counts.size());
        counts.push_back({componentMines, neighborMines, 0.0L, -1});
        if (state.lastCount >= 0)
            counts[state.lastCount].next = index;
        else
            state.firstCount = index;
        state.lastCount = index;
        return counts.back();
    }

    void advance(const StepPlan& plan, ObserveLayer& nextLayer,
                 std::span<const int> adjacentBoxCells, int xBox) const {
        nextLayer.states.clear();
        nextLayer.counts.clear();
        nextLayer.frontierValues.clear();
        nextLayer.index.clear();
        nextLayer.states.reserve(states.size() * (plan.boxSize + 1));
        nextLayer.counts.reserve(counts.size() * (plan.boxSize + 1));
        nextLayer.frontierValues.reserve(
            frontierValues.size() + plan.boxSize + 1);

        const int adjacent = adjacentBoxCells[plan.box];
        const bool isXBox = static_cast<int>(plan.box) == xBox;
        const int size = plan.boxSize;
        const int pool = isXBox ? size - 1 : size;

        for (const State& state : states) {
            int minMine = 0;
            int maxMine = plan.boxSize;
            for (const StepPlan::Check& check : plan.checks) {
                int partial = 0;
                for (int i = 0; i < check.readCount; ++i)
                    partial += frontierValues[state.frontierOffset +
                                              check.readSlots[i]];
                minMine = (std::max)(minMine,
                                     check.sum - partial - check.remainingSize);
                maxMine = (std::min)(maxMine, check.sum - partial);
            }

            for (int mine = minMine; mine <= maxMine; ++mine) {
                U128Hasher hasher;
                for (int source : plan.gather) {
                    const char value = source < 0
                                           ? static_cast<char>(mine)
                                           : frontierValues[state.frontierOffset +
                                                            source];
                    hasher.mix(static_cast<std::uint64_t>(
                        static_cast<unsigned char>(value)));
                }
                const U128 hash = hasher.finalize();

                State* target;
                if (const std::size_t* found = nextLayer.index.find(hash)) {
                    target = &nextLayer.states[*found];
                } else {
                    const std::size_t id = nextLayer.states.size();
                    nextLayer.states.push_back(
                        {nextLayer.frontierValues.size(), -1, -1});
                    for (int source : plan.gather)
                        nextLayer.frontierValues.push_back(
                            source < 0
                                ? static_cast<char>(mine)
                                : frontierValues[state.frontierOffset + source]);
                    nextLayer.index.emplace(hash, id);
                    target = &nextLayer.states.back();
                }

                if (!isXBox && adjacent == 0) {
                    const long double factor =
                        DfsSolver::detail::binom(size, mine);
                    for (int sourceIndex = state.firstCount;
                         sourceIndex >= 0;
                         sourceIndex = counts[sourceIndex].next) {
                        const Count& source = counts[sourceIndex];
                        Count& destination = nextLayer.findOrAddCount(
                            *target, source.componentMines + mine,
                            source.neighborMines);
                        destination.ways += source.ways * factor;
                    }
                    continue;
                }

                const int maxAdjacent = (std::min)(adjacent, mine);
                for (int neighborMines = 0;
                     neighborMines <= maxAdjacent; ++neighborMines) {
                    const int remaining = mine - neighborMines;
                    if (remaining > pool - adjacent) continue;
                    const long double factor =
                        DfsSolver::detail::binom(adjacent, neighborMines) *
                        DfsSolver::detail::binom(pool - adjacent, remaining);
                    for (int sourceIndex = state.firstCount;
                         sourceIndex >= 0;
                         sourceIndex = counts[sourceIndex].next) {
                        const Count& source = counts[sourceIndex];
                        if (source.neighborMines + neighborMines > 8) continue;
                        Count& destination = nextLayer.findOrAddCount(
                            *target, source.componentMines + mine,
                            source.neighborMines + neighborMines);
                        destination.ways += source.ways * factor;
                    }
                }
            }
        }
    }

    void emit(std::vector<Probability::ObserveTransfer>& out) const {
        for (const State& state : states)
            for (int index = state.firstCount; index >= 0;
                 index = counts[index].next) {
                const Count& count = counts[index];
                if (count.ways == 0.0L) continue;
                out.push_back({count.neighborMines, count.componentMines,
                               count.ways});
            }
    }
};

}  // namespace

void buildObserveTable(
    const Structure::Shape& shape, std::span<const int> adjacentBoxCells,
    int xBox, std::vector<Probability::ObserveTransfer>& out) {
    const detail::Graph graph = detail::Graph::fromShape(shape);
    const std::vector<BoxId> order =
        detail::makeOrder(graph, PolishKind::Adjacent);

    ObserveLayer current;
    ObserveLayer next;
    current.reset();
    detail::walkSteps(shape, order, [&](const detail::StepPlan& plan) {
        current.advance(plan, next, adjacentBoxCells, xBox);
        std::swap(current, next);
    });
    current.emit(out);
}

}  // namespace mss::ShapeSolver::GraphSolver
