#pragma once

#include <span>
#include <vector>

#include "algo/probability/probability.h"

namespace mss {

namespace Probability {

namespace GlobalSolver {

// 生成函数的稀疏区间表示：coeffs[i] 对应 x^(start + i)。
struct Poly {
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

struct Workspace {
    Poly identity;
    std::vector<DistributionId> distributions;
    std::vector<std::size_t> componentBoxCounts;
    std::vector<long double> entryProbabilities;
    std::vector<Poly> tree;
    std::vector<Poly> outside;
};

extern thread_local Workspace workspace;

// 以下三个函数是全局概率引擎的可测试算法接口，不负责管理工作区。
void polyMultiply(int leftStart, std::span<const long double> left,
                  int rightStart, std::span<const long double> right,
                  Poly& out);
long double denominator(const Poly& polynomial, int totalMines, int tSum);
long double unknownMineProbability(const Poly& polynomial, int totalMines,
                                   int tSum, long double candidates);

}  // namespace GlobalSolver

// 根据盘面约束计算全局雷概率并返回结果。
Result analyze(const ObservedBoard::Result& board, const Basic::Result& basic,
               const Structure::Result& structure,
               const Structure::ShapePool& shapes,
               ShapeSolver::Distribution::Pool& distributions);

// 高性能复用入口：result 由本函数完全重建，内部容量可跨次调用复用。
void analyze(const ObservedBoard::Result& board, const Basic::Result& basic,
             const Structure::Result& structure,
             const Structure::ShapePool& shapes,
             ShapeSolver::Distribution::Pool& distributions, Result& result);

}  // namespace Probability

}  // namespace mss
