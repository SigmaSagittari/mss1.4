#pragma once

#include <bit>
#include <span>
#include <vector>

#include "algo/probability/probability.h"
#include "core/assert.h"
#include "core/utility/combinatorics.h"

namespace mss {

// 生成函数的稀疏区间表示：coeffs[i] 对应 x^(start + i)。
struct Probability::Poly {
    int start = 0;
    std::vector<long double> coeffs;
    std::span<const long double> view;

    std::span<const long double> coefficients() const {
        return view.empty() ? std::span<const long double>(coeffs) : view;
    }

    void setView(int newStart, std::span<const long double> newCoefficients) {
        start = newStart;
        coeffs.clear();
        view = newCoefficients;
    }
};

struct Probability::Workspace {
    Poly identity;
    std::vector<DistributionId> distributions;
    std::vector<std::size_t> componentBoxCounts;
    std::vector<long double> entryProbabilities;
    std::vector<Poly> tree;
    std::vector<Poly> outside;
};

// 根据盘面约束计算全局雷概率并返回结果。

// 高性能复用入口：result 由本函数完全重建，内部容量可跨次调用复用。

//==============================================================================
inline thread_local Probability::Workspace Probability::globalWorkspace;

inline void Probability::polyMultiply(
    int leftStart, std::span<const long double> left, int rightStart,
    std::span<const long double> right, Poly& out) {
    const int size = static_cast<int>(left.size()) +
                     static_cast<int>(right.size()) - 1;
    out.view = {};
    out.coeffs.assign(size, 0.0L);
    for (int i = 0; i < static_cast<int>(left.size()); ++i)
        for (int j = 0; j < static_cast<int>(right.size()); ++j)
            out.coeffs[i + j] += left[i] * right[j];
    out.start = leftStart + rightStart;
}

inline long double Probability::denominator(
    const Poly& polynomial, int totalMines, int tSum) {
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

inline long double Probability::unknownMineProbability(
    const Poly& polynomial, int totalMines, int tSum, long double denom) {
    assert_(denom > 0.0L,
            "Probability::unknownMineProbability: 分母为零");
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

inline Probability::Result Probability::analyze(
    const ObservedBoard::Result& board, const Basic::Result& basic,
    const Structure::Result& structure, const Structure::Pool& shapes,
    ShapeSolver::Distribution::Pool& distributions) {
    Result result;
    analyze(board, basic, structure, shapes, distributions, result);
    return result;
}

inline void Probability::analyze(
    const ObservedBoard::Result& board, const Basic::Result& basic,
    const Structure::Result& structure, const Structure::Pool& shapes,
    ShapeSolver::Distribution::Pool& distributions, Result& result) {
    Workspace& ws = globalWorkspace;
    ws.distributions.clear();
    for (InstanceId instanceId : structure.components) {
        const Structure::Instance& instance = shapes.getInstance(instanceId);
        ws.distributions.push_back(ShapeSolver::analyze(
            shapes.get(instance.shape), distributions));
    }
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
            shapes.get(shapes.getInstance(structure.components[i]).shape)
                .boxes.size();
    result.reset(ws.componentBoxCounts);

    long double candidates;
    long double tCellProbability;
    if (componentCount == 0) {
        candidates = denominator(ws.identity, totalMines, tSum);
        if (candidates == 0.0L) {
            result.tCellProbability_ = 0.0L;
            result.candidates_ = 0.0L;
            return;
        }
        tCellProbability = unknownMineProbability(
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
            const auto& distribution = distributions.get(ws.distributions[i]);
            ws.tree[leafBase + i].setView(distribution.start(), distribution.ways());
        } else {
            ws.tree[leafBase + i].setView(ws.identity.start, ws.identity.coeffs);
        }
    }
    for (std::size_t i = leafBase - 1; i > 0; --i) {
        const Poly& left = ws.tree[i << 1];
        const Poly& right = ws.tree[i << 1 | 1];
        polyMultiply(left.start, left.coefficients(), right.start,
                     right.coefficients(), ws.tree[i]);
    }

    candidates = denominator(ws.tree[1], totalMines, tSum);
    if (candidates == 0.0L) {
        result.tCellProbability_ = 0.0L;
        result.candidates_ = 0.0L;
        return;
    }
    tCellProbability = unknownMineProbability(
        ws.tree[1], totalMines, tSum, candidates);
    tCellProbability = limitProbability(tCellProbability);
    ws.outside[1].setView(ws.identity.start, ws.identity.coeffs);
    for (std::size_t i = 1; i < leafBase; ++i) {
        const Poly& right = ws.tree[i << 1 | 1];
        polyMultiply(ws.outside[i].start, ws.outside[i].coefficients(),
                     right.start, right.coefficients(), ws.outside[i << 1]);
        const Poly& left = ws.tree[i << 1];
        polyMultiply(ws.outside[i].start, ws.outside[i].coefficients(),
                     left.start, left.coefficients(), ws.outside[i << 1 | 1]);
    }

    std::size_t boxOffset = 0;
    for (ComponentId cid = 0; cid < static_cast<ComponentId>(componentCount); ++cid) {
        const auto& distribution = distributions.get(ws.distributions[cid]);
        const auto ways = distribution.ways();
        const Poly& others = ws.outside[leafBase + cid];
        ws.entryProbabilities.assign(ways.size(), 0.0L);
        for (std::size_t i = 0; i < ways.size(); ++i) {
            const int componentMines = distribution.start() + (int)i;
            long double numerator = 0.0L;
            const auto otherCoefficients = others.coefficients();
            for (int k = 0; k < static_cast<int>(otherCoefficients.size()); ++k) {
                const int tMines = totalMines - componentMines - others.start - k;
                if (tMines >= 0 && tMines <= tSum)
                    numerator += otherCoefficients[k] * combLog(tSum, tMines);
            }
            ws.entryProbabilities[i] = ways[i] * numerator / candidates;
        }
        const Structure::Shape& shape =
            shapes.get(shapes.getInstance(structure.components[cid]).shape);
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

}  // namespace mss
