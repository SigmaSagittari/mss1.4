#pragma once

#include <array>
#include <cstddef>
#include <span>
#include <vector>

#include "algo/basic.h"
#include "algo/observed_board.h"
#include "algo/shape_solver/shape_solver.h"
#include "algo/structure.h"

namespace mss {

struct Probability {

    // 概率层把每个结构组件的分布与组件外的 Unknown 组合起来，按总雷数做全局条件化。
    // Result 内的 span 指向 Result 自己的连续存储；移动后只能使用目标对象。

    struct ObserveTransfer;
    struct ObserveResult;

public:

    // 逻辑确定的雷/安全格必须输出浮点精确的 1.0L/0.0L；只有未确定格
    // 才允许保留 (0, 1) 内的概率，尾差只向精确的一收敛。
    inline static long double limitProbability(long double probability) {
        return probability >= 1.0L - 1e-10L ? 1.0L : probability;
    }

    class Result {
    public:
        struct Component {
            std::span<const long double> boxProbabilities;
        };

        // 创建空的概率结果。
        Result() = default;

        // 禁止复制拥有 span 视图的结果。
        Result(const Result&) = delete;
        // 禁止复制赋值概率结果。
        Result& operator=(const Result&) = delete;
        // 移动概率结果及其内部存储。
        Result(Result&&) noexcept = default;
        // 移动赋值概率结果及其内部存储。
        Result& operator=(Result&&) noexcept = default;

        // 返回各连通组件的 Box 概率视图。
        std::span<const Component> components() const { return components_; }
        // 返回组件外 Unknown 格子的统一雷概率。
        long double tCellProbability() const { return tCellProbability_; }
        // 返回满足总雷数约束的加权方案数。
        long double candidates() const { return candidates_; }

        // 根据 CellLocation 查询指定格子的条件雷概率。
        long double mineProbability(CellId cell,
                                    const ObservedBoard::Result& board,
                                    const Basic::Result& basic,
                                    const Structure::Result& structure) const;

        template <typename Callback>
        // 遍历所有前沿 Box 中的格子及其对应雷概率。
        void frontierCells(const ObservedBoard::Result& board,
                           const Structure::Result& structure,
                           const Structure::Pool& shapes,
                           Callback&& callback) const;

    private:
        friend struct Probability;

        // 按组件 Box 数量重建结果视图和连续概率存储。
        void reset(std::span<const std::size_t> componentBoxCounts);

        std::vector<Component> components_;
        std::vector<long double> boxProbabilities_;
        long double tCellProbability_ = 0.0L;
        long double candidates_ = 0.0L;
    };

private:
    struct ObservePoly;
    struct ObserveWorkspace;
    struct GraphLayer;
    static thread_local ObserveWorkspace observeWorkspace;

    // 卷积两个点开专用多项式。
    static void observePolyMultiply(
        int leftStart, std::span<const long double> left, int rightStart,
        std::span<const long double> right, ObservePoly& out);
    // 将点开多项式乘入累加器并复用临时缓冲。
    static void observePolyMultiplyInto(
        ObservePoly& accumulator, int sourceStart,
        std::span<const long double> source, ObservePoly& mult);
    // 计算点开条件下的全局方案数分母。
    static long double observeDenominator(const ObservePoly& polynomial,
                                          int totalMines, int tSum);
    // 用 DFS 枚举组件并生成点开转移表。
    static void buildDfsTable(
        const Structure::Shape& shape, std::span<const int> adjacentBoxCells,
        int xBox, std::vector<ObserveTransfer>& out);
    // 用 Graph DP 生成点开转移表。
    static void buildGraphTable(
        const Structure::Shape& shape, std::span<const int> adjacentBoxCells,
        int xBox, std::vector<ObserveTransfer>& out);

    struct Poly;
    struct Workspace;
    static thread_local Workspace globalWorkspace;
    // 卷积全局概率使用的两个生成函数多项式。
    static void polyMultiply(int leftStart, std::span<const long double> left,
                             int rightStart, std::span<const long double> right,
                             Poly& out);
    // 计算全局总雷数条件下的加权方案数。
    static long double denominator(const Poly& polynomial, int totalMines,
                                   int tSum);
    // 计算组件外 Unknown 格子的条件雷概率。
    static long double unknownMineProbability(const Poly& polynomial,
                                              int totalMines, int tSum,
                                              long double candidates);

public:

    // 根据组件分布创建并返回一次全局概率分析结果。
    static Result analyze(const ObservedBoard::Result& board,
                          const Basic::Result& basic,
                          const Structure::Result& structure,
                          const Structure::Pool& shapes,
                          ShapeSolver::Distribution::Pool& distributions);
    // 重建传入 Result，复用已有容量以减少热路径分配。
    static void analyze(const ObservedBoard::Result& board,
                        const Basic::Result& basic,
                        const Structure::Result& structure,
                        const Structure::Pool& shapes,
                        ShapeSolver::Distribution::Pool& distributions,
                        Result& result);

    // 为指定点开格生成组件转移表。
    static void buildObserveTable(
        const Structure::Shape& shape, std::span<const int> adjacentBoxCells,
        int xBox, std::vector<ObserveTransfer>& out);
    // 计算点开指定 Hidden 格后的数字/爆炸概率分布。
    static ObserveResult observe(
        const ObservedBoard::Result& board, const Basic::Result& basic,
        const Structure::Result& structure, const Structure::Pool& shapes,
        const Result& probability,
        ShapeSolver::Distribution::Pool& distributions, CellId cell);
};

}  // namespace mss

//==============================================================================
namespace mss {

inline void Probability::Result::reset(
    std::span<const std::size_t> componentBoxCounts) {
    // 重排组件 span，使每个组件指向连续总概率数组的一段。
    components_.reserve(componentBoxCounts.size());
    components_.resize(componentBoxCounts.size());
    std::size_t totalBoxCount = 0;
    for (int i = 0; i < (int)(componentBoxCounts.size()); ++i)
        totalBoxCount += componentBoxCounts[i];
    boxProbabilities_.resize(totalBoxCount);
    const std::span<const long double> allProbabilities = boxProbabilities_;
    std::size_t offset = 0;
    for (int i = 0; i < (int)(componentBoxCounts.size()); ++i) {
        const std::size_t count = componentBoxCounts[i];
        components_[i].boxProbabilities = allProbabilities.subspan(offset, count);
        offset += count;
    }
}

inline long double Probability::Result::mineProbability(
    CellId cell, const ObservedBoard::Result& board, const Basic::Result& basic,
    const Structure::Result& structure) const {
    // 按格子的分析归属返回 Mine、Unknown、Safe 或前沿 Box 的概率；逻辑推出
    // 的确定雷/安全格在这里分别表现为浮点精确的 1.0L/0.0L。
    const auto [x, y] = board.pos(cell);
    const CellLocation loc = structure.cellLoc[cell];
    if (loc.component == -1) {
        if (basic.marks[x][y] == Basic::Mark::Mine) return 1.0L;
        if (basic.marks[x][y] == Basic::Mark::Unknown)
            return Probability::limitProbability(tCellProbability_);
        return 0.0L;
    }
    if (loc.box == -1) return 0.0L;
    const long double probability =
        components_[loc.component].boxProbabilities[loc.box];
    return Probability::limitProbability(probability);
}

}  // namespace mss

template <typename Callback>
inline void mss::Probability::Result::frontierCells(
    const mss::ObservedBoard::Result& board,
    const mss::Structure::Result& structure,
    const mss::Structure::Pool& shapes, Callback&& callback) const {
    // 展开每个前沿 Box 的成员格，并把 Box 概率传给回调。
    for (int cid = 0; cid < (int)(components_.size()); ++cid) {
        const Component& component = components_[cid];
        const Structure::Instance& instance = shapes.getInstance(
            structure.components[cid]);
        for (int box = 0; box < (int)(component.boxProbabilities.size()); ++box) {
            const long double probability = component.boxProbabilities[box];
            for (int i = instance.boxes.boxOf[box];
                 i < instance.boxes.boxOf[box + 1]; ++i) {
                const auto [x, y] = board.pos(instance.boxes.cells[i]);
                callback(x, y, probability);
            }
        }
    }
}
