#pragma once

#include <bit>
#include <span>
#include <vector>

#include "algo/probability_engine/probability/probability_external.h"
#include "core/assert.h"
#include "core/utility/combinatorics.h"

namespace mss {

// 根据盘面约束计算全局雷概率并返回结果。
// candidates 是满足总雷数的加权方案数，不是去重后的整数布局数；组件 ways 已经
// 把同一 Box 雷数对应的具体格子布局数量计入权重。

// 高性能复用入口：result 由本函数完全重建，内部容量可跨次调用复用。

inline void Probability::polyMultiply(Poly left, Poly right, TreePoly &out) {
    // 卷积两个稀疏区间多项式，结果由树节点拥有。
    const std::span<const long double> leftCoefficients = left.coefficients();
    const std::span<const long double> rightCoefficients = right.coefficients();
    const int size = leftCoefficients.size() + rightCoefficients.size() - 1;
    out.view = {};
    out.coeffs.assign(size, 0.0L);
    for (int i = 0; i < (int)(leftCoefficients.size()); ++i)
        for (int j = 0; j < (int)(rightCoefficients.size()); ++j)
            out.coeffs[i + j] += leftCoefficients[i] * rightCoefficients[j];
    out.start = left.start + right.start;
}

inline Probability::DistributionProbabilityResult Probability::analyzeDistributions(std::span<const Poly> distributions, int tMines,
                                                                                    int totalMines, Workspace &workspace) {
    std::size_t probabilityCount = 0;
    for (const Poly &distribution : distributions) {
        if (distribution.coefficients().empty())
            return {{}, 0.0L, 0.0L};
        probabilityCount += distribution.coefficients().size();
    }

    AnalyzeWorkspace &ws = workspace.analyze.buffers;
    const Poly identity{0, std::span<const long double>(ws.identity)};
    if (distributions.empty()) {
        const long double candidates = denominator(identity, totalMines, tMines, workspace);
        if (candidates == 0.0L)
            return {{}, 0.0L, 0.0L};
        const long double tCellProbability = unknownMineProbability(identity, totalMines, tMines, candidates, workspace);
        return {{}, candidates, limitProbability(tCellProbability)};
    }

    const std::size_t leafBase = std::bit_ceil(distributions.size());
    const std::size_t treeSize = 2 * leafBase;
    if (ws.tree.size() < treeSize)
        ws.tree.resize(treeSize);
    if (ws.outside.size() < treeSize)
        ws.outside.resize(treeSize);
    for (int i = 0; i < (int)(leafBase); ++i) {
        if (i < (int)(distributions.size()))
            ws.tree[leafBase + i].setView(distributions[i]);
        else
            ws.tree[leafBase + i].setView(identity);
    }
    for (int i = (int)(leafBase)-1; i > 0; --i) {
        const Poly left = ws.tree[i << 1].asPoly();
        const Poly right = ws.tree[i << 1 | 1].asPoly();
        polyMultiply(left, right, ws.tree[i]);
    }

    ws.outside[1].setView(identity);
    for (int i = 1; i < (int)(leafBase); ++i) {
        const Poly outside = ws.outside[i].asPoly();
        polyMultiply(outside, ws.tree[i << 1 | 1].asPoly(), ws.outside[i << 1]);
        polyMultiply(outside, ws.tree[i << 1].asPoly(), ws.outside[i << 1 | 1]);
    }
    const Poly all = ws.tree[1].asPoly();
    const long double candidates = denominator(all, totalMines, tMines, workspace);
    if (candidates == 0.0L)
        return {{}, 0.0L, 0.0L};
    const long double tCellProbability = limitProbability(unknownMineProbability(all, totalMines, tMines, candidates, workspace));

    ws.distributionProbabilities.resize(probabilityCount);
    ws.distributionProbabilityViews.resize(distributions.size());
    std::size_t probabilityOffset = 0;
    for (int i = 0; i < (int)(distributions.size()); ++i) {
        const Poly &distribution = distributions[i];
        const Poly dominator = ws.outside[leafBase + i].asPoly();
        const std::span<const long double> dominatorWays = dominator.coefficients();
        const std::span<long double> componentProbabilities(ws.distributionProbabilities.data() + probabilityOffset,
                                                            distribution.coefficients().size());
        for (int k = 0; k < (int)(distribution.coefficients().size()); ++k) {
            const int componentMines = distribution.start + k;
            long double numerator = 0.0L;
            for (int j = 0; j < (int)(dominatorWays.size()); ++j) {
                const int freeMines = totalMines - componentMines - dominator.start - j;
                if (freeMines >= 0 && freeMines <= tMines)
                    numerator += dominatorWays[j] * binom(tMines, freeMines, workspace.binom);
            }
            componentProbabilities[k] = distribution.coefficients()[k] * numerator / candidates;
        }
        ws.distributionProbabilityViews[i] = componentProbabilities;
        probabilityOffset += distribution.coefficients().size();
    }
    return {ws.distributionProbabilityViews, candidates, tCellProbability};
}

inline long double Probability::denominator(Poly polynomial, int totalMines, int tSum, Workspace &workspace) {
    // 用组件雷数多项式与 Unknown 的 C(tSum,tMines) 组合数相乘，得到全局条件化
    // 分母；同一分母同时归一化组件和组件外格子的概率。
    long double result = 0.0L;
    const std::span<const long double> coefficients = polynomial.coefficients();
    for (int i = 0; i < (int)(coefficients.size()); ++i) {
        const int componentMines = polynomial.start + i;
        const int tMines = totalMines - componentMines;
        if (tMines >= 0 && tMines <= tSum)
            result += coefficients[i] * binom(tSum, tMines, workspace.binom);
    }
    return result;
}

inline long double Probability::unknownMineProbability(Poly polynomial, int totalMines, int tSum, long double denom, Workspace &workspace) {
    // 固定一个组件外 Unknown 为雷，把组合数改为 C(tSum-1,tMines)，计算该格的
    // 条件雷概率；denom 必须是同一 polynomial 的总方案数。
    assert_(denom > 0.0L, "Probability::unknownMineProbability: 分母为零");
    long double result = 0.0L;
    const std::span<const long double> coefficients = polynomial.coefficients();
    for (int i = 0; i < (int)(coefficients.size()); ++i) {
        const int componentMines = polynomial.start + i;
        const int tMines = totalMines - 1 - componentMines;
        if (tMines >= 0 && tMines <= tSum - 1)
            result += coefficients[i] * binom(tSum - 1, tMines, workspace.binom);
    }
    return result / denom;
}

inline Probability::Result Probability::analyze(const ObservedBoard::Result &board, const Basic::Result &basic,
                                                const Structure::Result &structure, const Structure::Pool &shapes,
                                                ShapeSolver::Distribution::Pool &distributions, Workspace &workspace,
                                                const ShapeSolver::OrderAlgo &algo) {
    // 创建并返回一次全局概率分析结果。
    Result result;
    analyze(board, basic, structure, shapes, distributions, result, workspace, algo);
    return result;
}

inline void Probability::analyze(const ObservedBoard::Result &board, const Basic::Result &basic, const Structure::Result &structure,
                                 const Structure::Pool &shapes, ShapeSolver::Distribution::Pool &distributions, Result &result,
                                 Workspace &workspace, const ShapeSolver::OrderAlgo &algo) {
    // 重建 result 的组件概率、Unknown 概率和加权方案总数；先用乘积树得到总分母，
    // 再用 outside 树为每个组件排除自身，避免为每个组件重复卷积其余组件。
    // 工作区按所属对象复用多项式和临时数组，避免热路径反复分配；分析结果本身
    // 拥有自己的存储，不依赖该工作区的生命周期。
    AnalyzeWorkspace &ws = workspace.analyze.buffers;
    ws.distributions.clear();
    for (Structure::InstanceId instanceId : structure.components) {
        const Structure::Instance &instance = shapes.getInstance(instanceId);
        ws.distributions.push_back(ShapeSolver::analyze(shapes.get(instance.shape), shapes, distributions, workspace.shapeSolver, algo));
    }
    for (const ShapeSolver::DistributionId id : ws.distributions)
        if (distributions.get(id).ways().empty()) {
            result.reset({});
            result.tCellProbability_ = 0.0L;
            result.candidates_ = 0.0L;
            return;
        }

    const int totalMines = board.totalMines - basic.mineSum;
    const int tSum = basic.unknownSum;
    const std::size_t componentCount = ws.distributions.size();
    ws.componentBoxCounts.resize(componentCount);
    ws.factors.resize(componentCount);
    for (int i = 0; i < (int)(componentCount); ++i) {
        const ShapeSolver::Distribution::Result &distribution = distributions.get(ws.distributions[i]);
        ws.factors[i] = {distribution.start(), distribution.ways()};
        ws.componentBoxCounts[i] = shapes.get(shapes.getInstance(structure.components[i]).shape).boxes.size;
    }
    result.reset(ws.componentBoxCounts);

    const DistributionProbabilityResult distributionResult = analyzeDistributions(ws.factors, tSum, totalMines, workspace);
    result.tCellProbability_ = distributionResult.tCellProbability;
    result.candidates_ = distributionResult.candidates;
    if (distributionResult.candidates == 0.0L || componentCount == 0)
        return;

    std::size_t boxOffset = 0;
    for (int cid = 0; cid < (int)(componentCount); ++cid) {
        const ShapeSolver::Distribution::Result &distribution = distributions.get(ws.distributions[cid]);
        const std::span<const long double> mineCountProbabilities = distributionResult.mineCountProbabilities[cid];
        const Structure::Shape &shape = shapes.get(shapes.getInstance(structure.components[cid]).shape);
        for (int box = 0; box < shape.boxes.size; ++box) {
            long double probability = 0.0L;
            for (int i = 0; i < (int)(mineCountProbabilities.size()); ++i)
                probability += mineCountProbabilities[i] * distribution.perBoxExpectation(i)[box];
            result.boxProbabilities_[boxOffset + box] = probability / shape.boxes.span(shapes.boxes)[box].size;
        }
        boxOffset += shape.boxes.size;
    }
}

} // namespace mss
