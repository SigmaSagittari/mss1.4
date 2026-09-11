#include "algo/probability/global_solver.h"

#include <bit>

#include "core/assert.h"
#include "core/utility/combinatorics.h"

namespace mss {

namespace Probability {

namespace GlobalSolver {

thread_local Workspace workspace;

void polyMultiply(int leftStart, std::span<const long double> left,
                  int rightStart, std::span<const long double> right,
                  Poly& out) {
    const int size = static_cast<int>(left.size()) +
                     static_cast<int>(right.size()) - 1;
    out.view = {};
    out.coeffs.assign(size, 0.0L);
    for (int i = 0; i < static_cast<int>(left.size()); ++i)
        for (int j = 0; j < static_cast<int>(right.size()); ++j)
            out.coeffs[i + j] += left[i] * right[j];
    out.start = leftStart + rightStart;
}

long double denominator(const Poly& polynomial, int totalMines, int tSum) {
    long double result = 0.0L;
    const auto coefficients = polynomial.coefficients();
    for (int i = 0; i < static_cast<int>(coefficients.size()); ++i) {
        const int componentMines = polynomial.start + i;
        const int tMines = totalMines - componentMines;
        if (tMines >= 0 && tMines <= tSum)
            result += coefficients[i] * combLog(tSum, tMines);
    }
    return result;
}

long double unknownMineProbability(const Poly& polynomial, int totalMines,
                                   int tSum, long double denom) {
    assert_(denom > 0.0L,
            "Probability::GlobalSolver::unknownMineProbability: 分母为零");
    long double result = 0.0L;
    const auto coefficients = polynomial.coefficients();
    for (int i = 0; i < static_cast<int>(coefficients.size()); ++i) {
        const int componentMines = polynomial.start + i;
        const int tMines = totalMines - 1 - componentMines;
        if (tMines >= 0 && tMines <= tSum - 1)
            result += coefficients[i] * combLog(tSum - 1, tMines);
    }
    return result / denom;
}

}  // namespace GlobalSolver

Result analyze(const ObservedBoard::Result& board, const Basic::Result& basic,
               const Structure::Result& structure,
               const Structure::ShapePool& shapes,
               ShapeSolver::Distribution::Pool& distributions) {
    Result result;
    analyze(board, basic, structure, shapes, distributions, result);
    return result;
}

void analyze(const ObservedBoard::Result& board, const Basic::Result& basic,
             const Structure::Result& structure,
             const Structure::ShapePool& shapes,
             ShapeSolver::Distribution::Pool& distributions, Result& result) {
    GlobalSolver::Workspace& ws = GlobalSolver::workspace;
    ws.distributions.clear();
    for (const Structure::Instance& instance : structure.components)
        ws.distributions.push_back(ShapeSolver::analyze(
            shapes.get(instance.shape), distributions));

    for (const DistributionId id : ws.distributions)
        if (distributions.get(id).ways().empty()) {
            result.reset({});
            result.tCellProbability_ = 0.0L;
            result.candidates_ = 0.0L;
            return;
        }

    const int totalMines = board.totalMines - basic.mineSum;
    const int tSum = basic.unknownSum;
    const std::size_t componentCount = ws.distributions.size();

    ws.identity.start = 0;
    ws.identity.coeffs.assign(1, 1.0L);

    ws.componentBoxCounts.resize(componentCount);
    for (std::size_t i = 0; i < componentCount; ++i)
        ws.componentBoxCounts[i] =
            shapes.get(structure.components[i].shape).boxes.size();
    result.reset(ws.componentBoxCounts);

    long double candidates;
    long double tCellProbability;
    if (componentCount == 0) {
        candidates = GlobalSolver::denominator(ws.identity, totalMines, tSum);
        tCellProbability = GlobalSolver::unknownMineProbability(
            ws.identity, totalMines, tSum, candidates);
        result.tCellProbability_ = limitProbability(tCellProbability);
        result.candidates_ = candidates;
        return;
    }

    const std::size_t leafBase = std::bit_ceil(componentCount);
    const std::size_t treeSize = 2 * leafBase;
    if (ws.tree.size() < treeSize) ws.tree.resize(treeSize);
    if (ws.outside.size() < treeSize) ws.outside.resize(treeSize);
    for (std::size_t i = 0; i < leafBase; ++i) {
        if (i < componentCount) {
            const auto& distribution =
                distributions.get(ws.distributions[i]);
            ws.tree[leafBase + i].setView(distribution.start(),
                                          distribution.ways());
        } else {
            ws.tree[leafBase + i].setView(ws.identity.start,
                                          ws.identity.coeffs);
        }
    }
    for (std::size_t i = leafBase - 1; i > 0; --i) {
        const GlobalSolver::Poly& left = ws.tree[i << 1];
        const GlobalSolver::Poly& right = ws.tree[i << 1 | 1];
        GlobalSolver::polyMultiply(left.start, left.coefficients(),
                                   right.start, right.coefficients(),
                                   ws.tree[i]);
    }

    candidates = GlobalSolver::denominator(ws.tree[1], totalMines, tSum);
    assert_(candidates > 0.0L,
            "Probability::analyze: 当前盘面不存在全局可行方案");
    tCellProbability = GlobalSolver::unknownMineProbability(
        ws.tree[1], totalMines, tSum, candidates);
    tCellProbability = limitProbability(tCellProbability);

    ws.outside[1].setView(ws.identity.start, ws.identity.coeffs);
    for (std::size_t i = 1; i < leafBase; ++i) {
        const GlobalSolver::Poly& right = ws.tree[i << 1 | 1];
        GlobalSolver::polyMultiply(ws.outside[i].start,
                                   ws.outside[i].coefficients(), right.start,
                                   right.coefficients(),
                                   ws.outside[i << 1]);
        const GlobalSolver::Poly& left = ws.tree[i << 1];
        GlobalSolver::polyMultiply(ws.outside[i].start,
                                   ws.outside[i].coefficients(), left.start,
                                   left.coefficients(),
                                   ws.outside[i << 1 | 1]);
    }

    std::size_t boxOffset = 0;
    for (ComponentId cid = 0; cid < static_cast<ComponentId>(componentCount);
         ++cid) {
        const auto& distribution = distributions.get(ws.distributions[cid]);
        const auto ways = distribution.ways();
        const GlobalSolver::Poly& others = ws.outside[leafBase + cid];
        ws.entryProbabilities.assign(ways.size(), 0.0L);
        for (std::size_t i = 0; i < ways.size(); ++i) {
            const int componentMines = distribution.start() + i;
            long double numerator = 0.0L;
            const auto otherCoefficients = others.coefficients();
            for (int k = 0; k < static_cast<int>(otherCoefficients.size());
                 ++k) {
                const int tMines = totalMines - componentMines -
                                   others.start - k;
                if (tMines >= 0 && tMines <= tSum)
                    numerator += otherCoefficients[k] * combLog(tSum, tMines);
            }
            ws.entryProbabilities[i] = ways[i] * numerator / candidates;
        }

        const Structure::Shape& shape =
            shapes.get(structure.components[cid].shape);
        for (std::size_t box = 0; box < shape.boxes.size(); ++box) {
            long double probability = 0.0L;
            for (std::size_t i = 0; i < ways.size(); ++i)
                probability += ws.entryProbabilities[i] *
                               distribution.perBoxExpectation(i)[box];
            result.boxProbabilities_[boxOffset + box] =
                probability / static_cast<long double>(shape.boxes[box].size);
        }
        boxOffset += shape.boxes.size();
    }
    result.tCellProbability_ = tCellProbability;
    result.candidates_ = candidates;
}

}  // namespace Probability

}  // namespace mss
