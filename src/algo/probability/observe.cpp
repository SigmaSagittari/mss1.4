#include "algo/probability/observe.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "algo/shape_solver/dfs_solver.h"
#include "core/assert.h"
#include "core/utility/combinatorics.h"

namespace mss {

namespace Probability {

namespace {

struct Poly {
    int start = 0;
    std::vector<long double> coeffs;
};

struct Transfer {
    int h = 0;
    int y = 0;
    long double ways = 0.0L;
};

struct Workspace {
    Poly rest;
    Poly all;
    Poly mult;
    std::vector<ComponentId> captured;
    std::vector<char> seen;
    std::vector<int> adjacentBoxCells;
    std::vector<Transfer> transfers;
    std::vector<std::pair<int, int>> transferRanges;
    std::vector<std::array<long double, 9>> accumulated;
    std::vector<long double> dp;
    std::vector<long double> nextDp;
    std::vector<long double> restWays;
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

void buildTransfer(const Structure::Shape& shape, std::span<const int> adjacent,
                   int xBox, std::vector<Transfer>& out) {
    int maxTotal = 0;
    for (const auto& box : shape.boxes) maxTotal += box.size;
    Workspace& ws = workspace;
    ws.accumulated.assign(maxTotal + 1, {});

    ShapeSolver::DfsSolver::forEachAssignment(
        shape, [&](std::span<const char> assignment, long double ways) {
            int totalMines = 0;
            std::array<long double, 9> convolution{};
            convolution[0] = 1.0L;
            for (std::size_t boxId = 0; boxId < assignment.size(); ++boxId) {
                const int mines = assignment[boxId];
                totalMines += mines;
                const int adjacentCount = adjacent[boxId];
                const bool isXBox = static_cast<int>(boxId) == xBox;
                if (adjacentCount == 0 && !isXBox) continue;

                const int size = shape.boxes[boxId].size;
                const int pool = isXBox ? size - 1 : size;
                std::array<long double, 9> local{};
                const int maxAdjacentMines = (std::min)(adjacentCount, mines);
                for (int adjacentMines = 0;
                     adjacentMines <= maxAdjacentMines; ++adjacentMines) {
                    const int remaining = mines - adjacentMines;
                    if (remaining > pool - adjacentCount) continue;
                    local[adjacentMines] =
                        combLog(adjacentCount, adjacentMines) *
                        combLog(pool - adjacentCount, remaining) /
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
                    ws.accumulated[totalMines][h] += ways * convolution[h];
        });

    for (int totalMines = 0; totalMines <= maxTotal; ++totalMines)
        for (int h = 0; h <= 8; ++h)
            if (ws.accumulated[totalMines][h] != 0.0L)
                out.push_back(
                    {h, totalMines, ws.accumulated[totalMines][h]});
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

    if (board.board[x][y] != ObservedBoard::CellState::Hidden) return result;
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
    ws.transfers.clear();
    ws.transferRanges.clear();
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
        for (const auto& box : shape.boxes) maxCapturedMines += box.size;

        ws.adjacentBoxCells.assign(shape.boxes.size(), 0);
        for (std::size_t box = 0; box < shape.boxes.size(); ++box)
            for (std::size_t i = instance.boxes.boxOf[box];
                 i < instance.boxes.boxOf[box + 1]; ++i) {
                const auto [cx, cy] = board.pos(instance.boxes.cells[i]);
                if ((std::abs(cx - x) <= 1) && (std::abs(cy - y) <= 1) &&
                    !(cx == x && cy == y))
                    ++ws.adjacentBoxCells[box];
            }

        const int offset = static_cast<int>(ws.transfers.size());
        buildTransfer(shape, ws.adjacentBoxCells,
                      component == xComponent ? xBox : -1, ws.transfers);
        ws.transferRanges.emplace_back(
            offset, static_cast<int>(ws.transfers.size()) - offset);
    }

    if (unknownNeighbors > 0) {
        const int offset = static_cast<int>(ws.transfers.size());
        for (int mines = 0; mines <= unknownNeighbors; ++mines)
            ws.transfers.push_back(
                {mines, mines, combLog(unknownNeighbors, mines)});
        ws.transferRanges.emplace_back(
            offset, static_cast<int>(ws.transfers.size()) - offset);
    }

    const int stride = maxCapturedMines + 1;
    ws.dp.assign(9 * stride, 0.0L);
    ws.dp[0] = 1.0L;
    for (const auto [offset, count] : ws.transferRanges) {
        ws.nextDp.assign(9 * stride, 0.0L);
        for (int i = 0; i < count; ++i) {
            const Transfer& transfer = ws.transfers[offset + i];
            for (int neighborMines = 0;
                 neighborMines + transfer.h <= 8; ++neighborMines)
                for (int capturedMines = 0;
                     capturedMines + transfer.y <= maxCapturedMines;
                     ++capturedMines) {
                    const long double base =
                        ws.dp[neighborMines * stride + capturedMines];
                    if (base == 0.0L) continue;
                    ws.nextDp[(neighborMines + transfer.h) * stride +
                              capturedMines + transfer.y] +=
                        base * transfer.ways;
                }
        }
        ws.dp.swap(ws.nextDp);
    }

    const int tPool = tSum - unknownNeighbors - (xInUnknown ? 1 : 0);
    ws.rest.start = 0;
    ws.rest.coeffs.assign(1, 1.0L);
    for (ComponentId component = 0;
         component < static_cast<ComponentId>(structure.components.size());
         ++component) {
        if (ws.seen[component]) continue;
        const Structure::Shape& shape =
            shapes.get(structure.components[component].shape);
        const DistributionId id =
            ShapeSolver::analyze(shape, distributions);
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

}  // namespace Probability

}  // namespace mss
