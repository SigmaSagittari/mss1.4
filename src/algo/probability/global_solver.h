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

    // 返回当前多项式的有效系数视图：叶节点借用组件分布的 ways，卷积节点使用
    // 自己的 coeffs；调用方只需依赖这个统一入口。
    std::span<const long double> coefficients() const {
        return view.empty() ? std::span<const long double>(coeffs) : view;
    }

    // 将多项式切换为指定指数起点和外部系数视图；清空自有 coeffs 表示该节点
    // 只是借用叶节点/恒等多项式的存储。
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
// candidates 是满足总雷数的加权方案数，不是去重后的整数布局数；组件 ways 已经
// 把同一 Box 雷数对应的具体格子布局数量计入权重。

// 高性能复用入口：result 由本函数完全重建，内部容量可跨次调用复用。

//==============================================================================
inline thread_local Probability::Workspace Probability::globalWorkspace;

inline void Probability::polyMultiply(int leftStart, std::span<const long double> left, int rightStart, std::span<const long double> right,
                                      Poly &out) {
    // 卷积两个稀疏区间多项式；树节点用它合并左右组件，out 必须拥有自己的系数，
    // 不能继续借用任一输入视图。
    const int size = left.size() + right.size() - 1;
    out.view = {};
    out.coeffs.assign(size, 0.0L);
    for (int i = 0; i < (int)(left.size()); ++i)
        for (int j = 0; j < (int)(right.size()); ++j)
            out.coeffs[i + j] += left[i] * right[j];
    out.start = leftStart + rightStart;
}

inline long double Probability::denominator(const Poly &polynomial, int totalMines, int tSum) {
    // 用组件雷数多项式与 Unknown 的 C(tSum,tMines) 组合数相乘，得到全局条件化
    // 分母；同一分母同时归一化组件和组件外格子的概率。
    long double result = 0.0L;
    const std::span<const long double> coefficients = polynomial.coefficients();
    for (int i = 0; i < (int)(coefficients.size()); ++i) {
        const int componentMines = polynomial.start + i;
        const int tMines = totalMines - componentMines;
        if (tMines >= 0 && tMines <= tSum)
            result += coefficients[i] * combLog(tSum, tMines);
    }
    return result;
}

inline long double Probability::unknownMineProbability(const Poly &polynomial, int totalMines, int tSum, long double denom) {
    // 固定一个组件外 Unknown 为雷，把组合数改为 C(tSum-1,tMines)，计算该格的
    // 条件雷概率；denom 必须是同一 polynomial 的总方案数。
    assert_(denom > 0.0L, "Probability::unknownMineProbability: 分母为零");
    long double result = 0.0L;
    const std::span<const long double> coefficients = polynomial.coefficients();
    for (int i = 0; i < (int)(coefficients.size()); ++i) {
        const int componentMines = polynomial.start + i;
        const int tMines = totalMines - 1 - componentMines;
        if (tMines >= 0 && tMines <= tSum - 1)
            result += coefficients[i] * combLog(tSum - 1, tMines);
    }
    return result / denom;
}

inline Probability::Result Probability::analyze(const ObservedBoard::Result &board, const Basic::Result &basic,
                                                const Structure::Result &structure, const Structure::Pool &shapes,
                                                ShapeSolver::Distribution::Pool &distributions) {
    // 创建并返回一次全局概率分析结果。
    Result result;
    analyze(board, basic, structure, shapes, distributions, result);
    return result;
}

inline void Probability::analyze(const ObservedBoard::Result &board, const Basic::Result &basic, const Structure::Result &structure,
                                 const Structure::Pool &shapes, ShapeSolver::Distribution::Pool &distributions, Result &result) {
    // 重建 result 的组件概率、Unknown 概率和加权方案总数；先用乘积树得到总分母，
    // 再用 outside 树为每个组件排除自身，避免为每个组件重复卷积其余组件。
    // 设计目的：globalWorkspace 按线程复用多项式和临时数组，避免热路径反复分配；
    // 分析结果本身拥有自己的存储，不依赖该工作区的生命周期。
    Workspace &ws = globalWorkspace;
    ws.distributions.clear();
    for (InstanceId instanceId : structure.components) {
        const Structure::Instance &instance = shapes.getInstance(instanceId);
        ws.distributions.push_back(ShapeSolver::analyze(shapes.get(instance.shape), distributions));
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
    for (int i = 0; i < (int)(componentCount); ++i)
        ws.componentBoxCounts[i] = shapes.get(shapes.getInstance(structure.components[i]).shape).boxes.size();
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
        tCellProbability = unknownMineProbability(ws.identity, totalMines, tSum, candidates);
        result.tCellProbability_ = limitProbability(tCellProbability);
        result.candidates_ = candidates;
        return;
    }

    const std::size_t leafBase = std::bit_ceil(componentCount);
    const std::size_t treeSize = 2 * leafBase;
    if (ws.tree.size() < treeSize)
        ws.tree.resize(treeSize);
    if (ws.outside.size() < treeSize)
        ws.outside.resize(treeSize);
    for (int i = 0; i < (int)(leafBase); ++i) {
        if (i < (int)(componentCount)) {
            const ShapeSolver::Distribution::Result &distribution = distributions.get(ws.distributions[i]);
            ws.tree[leafBase + i].setView(distribution.start(), distribution.ways());
        } else {
            ws.tree[leafBase + i].setView(ws.identity.start, ws.identity.coeffs);
        }
    }
    for (int i = (int)(leafBase)-1; i > 0; --i) {
        const Poly &left = ws.tree[i << 1];
        const Poly &right = ws.tree[i << 1 | 1];
        polyMultiply(left.start, left.coefficients(), right.start, right.coefficients(), ws.tree[i]);
    }

    candidates = denominator(ws.tree[1], totalMines, tSum);
    if (candidates == 0.0L) {
        result.tCellProbability_ = 0.0L;
        result.candidates_ = 0.0L;
        return;
    }
    tCellProbability = unknownMineProbability(ws.tree[1], totalMines, tSum, candidates);
    tCellProbability = limitProbability(tCellProbability);
    ws.outside[1].setView(ws.identity.start, ws.identity.coeffs);
    for (int i = 1; i < (int)(leafBase); ++i) {
        const Poly &right = ws.tree[i << 1 | 1];
        polyMultiply(ws.outside[i].start, ws.outside[i].coefficients(), right.start, right.coefficients(), ws.outside[i << 1]);
        const Poly &left = ws.tree[i << 1];
        polyMultiply(ws.outside[i].start, ws.outside[i].coefficients(), left.start, left.coefficients(), ws.outside[i << 1 | 1]);
    }

    std::size_t boxOffset = 0;
    for (int cid = 0; cid < (int)(componentCount); ++cid) {
        const ShapeSolver::Distribution::Result &distribution = distributions.get(ws.distributions[cid]);
        const std::span<const long double> ways = distribution.ways();
        const Poly &others = ws.outside[leafBase + cid];
        ws.entryProbabilities.assign(ways.size(), 0.0L);
        for (int i = 0; i < (int)(ways.size()); ++i) {
            const int componentMines = distribution.start() + i;
            long double numerator = 0.0L;
            const std::span<const long double> otherCoefficients = others.coefficients();
            for (int k = 0; k < (int)(otherCoefficients.size()); ++k) {
                const int tMines = totalMines - componentMines - others.start - k;
                if (tMines >= 0 && tMines <= tSum)
                    numerator += otherCoefficients[k] * combLog(tSum, tMines);
            }
            ws.entryProbabilities[i] = ways[i] * numerator / candidates;
        }
        const Structure::Shape &shape = shapes.get(shapes.getInstance(structure.components[cid]).shape);
        for (int box = 0; box < (int)(shape.boxes.size()); ++box) {
            long double probability = 0.0L;
            for (int i = 0; i < (int)(ways.size()); ++i)
                probability += ws.entryProbabilities[i] * distribution.perBoxExpectation(i)[box];
            result.boxProbabilities_[boxOffset + box] = probability / shape.boxes[box].size;
        }
        boxOffset += shape.boxes.size();
    }
    result.tCellProbability_ = tCellProbability;
    result.candidates_ = candidates;
}

} // namespace mss
