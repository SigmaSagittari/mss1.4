#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "algo/probability_engine/basic.h"
#include "algo/probability_engine/observed_board.h"
#include "algo/probability_engine/shape_solver/shape_solver.h"
#include "algo/probability_engine/structure.h"
#include "core/utility/combinatorics.h"
#include "core/utility/flat_hashtable.h"
#include "core/utility/grid.h"
#include "core/utility/hash.h"

namespace mss {
struct Probability {

    // 概率层把每个结构组件的分布与组件外的 Unknown 组合起来，按总雷数做全局条件化。
    // Result 内的 span 指向 Result 自己的连续存储；移动后只能使用目标对象。

    struct Workspace {
        ShapeSolver::Workspace &shapeSolver;
        Binom::Workspace &binom;

        struct Analyze {
            struct Poly {
                int start = 0;
                std::span<const long double> coeffs;

                std::span<const long double> coefficients() const {
                    return coeffs;
                }
            };

            struct TreePoly {
                int start = 0;
                std::vector<long double> coeffs;
                std::span<const long double> view;

                std::span<const long double> coefficients() const {
                    return view.empty() ? std::span<const long double>(coeffs) : view;
                }

                Poly asPoly() const {
                    return {start, coefficients()};
                }

                void setView(Poly poly) {
                    start = poly.start;
                    coeffs.clear();
                    view = poly.coefficients();
                }
            };

            struct Buffers {
                std::array<long double, 1> identity{1.0L};
                std::vector<ShapeSolver::DistributionId> distributions;
                std::vector<std::size_t> componentBoxCounts;
                std::vector<Poly> factors;
                std::vector<long double> distributionProbabilities;
                std::vector<std::span<const long double>> distributionProbabilityViews;
                std::vector<TreePoly> tree;
                std::vector<TreePoly> outside;
            } buffers;
        } analyze{};

        struct Observe {
            struct Transfer {
                int neighborMines = 0;
                int componentMines = 0;
                long double ways = 0.0L;
            };

            struct Poly {
                int start = 0;
                std::vector<long double> coeffs;
            };

            struct Buffers {
                Poly rest;
                Poly all;
                Poly mult;
                std::vector<Structure::ComponentId> captured;
                std::vector<char> seen;
                std::vector<int> adjacentBoxCells;
                RawGrid<long double> dp;
                RawGrid<long double> nextDp;
                std::vector<long double> restWays;
                std::vector<Transfer> transfers;
            } buffers;

            struct BuildDfsTable {
                std::vector<std::array<long double, 9>> accumulated;
            } buildDfsTable;

            struct BuildGraphTable {
                struct Layer {
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

                    Count &findOrAddCount(State &state, int componentMines, int neighborMines) {
                        for (int i = state.firstCount; i >= 0; i = counts[i].next)
                            if (counts[i].componentMines == componentMines && counts[i].neighborMines == neighborMines)
                                return counts[i];
                        const int index = counts.size();
                        counts.push_back({componentMines, neighborMines, 0.0L, -1});
                        if (state.lastCount >= 0)
                            counts[state.lastCount].next = index;
                        else
                            state.firstCount = index;
                        state.lastCount = index;
                        return counts.back();
                    }

                    template <typename Plan>
                    void advance(const Plan &plan, Layer &nextLayer, std::span<const int> adjacentBoxCells, int xBox) const;

                    void emit(std::vector<Transfer> &out) const {
                        for (const State &state : states)
                            for (int index = state.firstCount; index >= 0; index = counts[index].next) {
                                const Count &count = counts[index];
                                if (count.ways == 0.0L)
                                    continue;
                                out.push_back({count.neighborMines, count.componentMines, count.ways});
                            }
                    }
                };
            };
        } observe{};
    };

    using ObserveTransfer = Workspace::Observe::Transfer;
    struct ObserveResult;

  public:
    // coeffs[k] 是恰有 start + k 雷的前沿方案数。
    using Poly = Workspace::Analyze::Poly;

    struct DistributionProbabilityResult {
        std::span<const std::span<const long double>> mineCountProbabilities;
        long double candidates;
        long double tCellProbability;
    };

    inline static long double limitProbability(long double probability) {
        return probability >= 1.0L - 1e-10L ? 1.0L : probability;
    }

    // tMines 是可自由分配的 T 格数量，totalMines 是剩余总雷数；mineCountProbabilities[i][k] 对应 distributions[i].start + k 雷。
    // 返回的分布视图只能使用到同一 Workspace 下一次调用此函数或 analyze。
    static DistributionProbabilityResult analyzeDistributions(std::span<const Poly> distributions, int tMines, int totalMines,
                                                              Workspace &workspace);

    class Result {
      public:
        struct Component {
            std::span<const long double> boxProbabilities;
        };

        // 创建空的概率结果。
        Result() = default;

        // 禁止复制拥有 span 视图的结果。
        Result(const Result &) = delete;
        // 禁止复制赋值概率结果。
        Result &operator=(const Result &) = delete;
        // 移动概率结果及其内部存储。
        Result(Result &&) noexcept = default;
        // 移动赋值概率结果及其内部存储。
        Result &operator=(Result &&) noexcept = default;

        // 返回各连通组件的 Box 概率视图。
        std::span<const Component> components() const {
            return components_;
        }
        // 返回组件外 Unknown 格子的统一雷概率。
        long double tCellProbability() const {
            return tCellProbability_;
        }
        // 返回满足总雷数约束的加权方案数。
        long double candidates() const {
            return candidates_;
        }

        // 根据 Structure::CellLocation 查询指定格子的条件雷概率。
        long double mineProbability(ObservedBoard::CellId cell, const ObservedBoard::Result &board, const Basic::Result &basic,
                                    const Structure::Result &structure) const;

        template <typename Callback>
        // 遍历所有前沿 Box 中的格子及其对应雷概率。
        void frontierCells(const ObservedBoard::Result &board, const Structure::Result &structure, const Structure::Pool &shapes,
                           Callback &&callback) const;

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
    using ObservePoly = Workspace::Observe::Poly;
    using ObserveWorkspace = Workspace::Observe::Buffers;
    using GraphLayer = Workspace::Observe::BuildGraphTable::Layer;

    // 卷积两个点开专用多项式。
    static void observePolyMultiply(int leftStart, std::span<const long double> left, int rightStart, std::span<const long double> right,
                                    ObservePoly &out);
    // 将点开多项式乘入累加器并复用临时缓冲。
    static void observePolyMultiplyInto(ObservePoly &accumulator, int sourceStart, std::span<const long double> source, ObservePoly &mult);
    // 计算点开条件下的全局方案数分母。
    static long double observeDenominator(const ObservePoly &polynomial, int totalMines, int tSum, Workspace &workspace);
    // 用 DFS 枚举组件并生成点开转移表。
    static void buildDfsTable(const Structure::Shape &shape, const Structure::Pool &shapes, std::span<const int> adjacentBoxCells,
                              int xBox, std::vector<ObserveTransfer> &out, Workspace &workspace);
    // 用 Graph DP 生成点开转移表。
    static void buildGraphTable(const Structure::Shape &shape, const Structure::Pool &shapes, std::span<const int> adjacentBoxCells,
                                int xBox, std::vector<ObserveTransfer> &out, Workspace &workspace);

    using TreePoly = Workspace::Analyze::TreePoly;
    using AnalyzeWorkspace = Workspace::Analyze::Buffers;
    // 卷积全局概率使用的两个生成函数多项式。
    static void polyMultiply(Poly left, Poly right, TreePoly &out);
    // 计算全局总雷数条件下的加权方案数。
    static long double denominator(Poly polynomial, int totalMines, int tSum, Workspace &workspace);
    // 计算组件外 Unknown 格子的条件雷概率。
    static long double unknownMineProbability(Poly polynomial, int totalMines, int tSum, long double candidates, Workspace &workspace);

  public:
    // 根据组件分布创建并返回一次全局概率分析结果。
    static Result analyze(const ObservedBoard::Result &board, const Basic::Result &basic, const Structure::Result &structure,
                          const Structure::Pool &shapes, ShapeSolver::Distribution::Pool &distributions, Workspace &workspace,
                          const ShapeSolver::OrderAlgo &algo = ShapeSolver::OrderAlgo::Auto);
    // 重建传入 Result，复用已有容量以减少热路径分配。
    static void analyze(const ObservedBoard::Result &board, const Basic::Result &basic, const Structure::Result &structure,
                        const Structure::Pool &shapes, ShapeSolver::Distribution::Pool &distributions, Result &result,
                        Workspace &workspace, const ShapeSolver::OrderAlgo &algo = ShapeSolver::OrderAlgo::Auto);

    // 为指定点开格生成组件转移表。
    static void buildObserveTable(const Structure::Shape &shape, const Structure::Pool &shapes, std::span<const int> adjacentBoxCells,
                                  int xBox, std::vector<ObserveTransfer> &out, Workspace &workspace);
    // 计算点开指定 Hidden 格后的数字/爆炸概率分布。
    static ObserveResult observe(const ObservedBoard::Result &board, const Basic::Result &basic, const Structure::Result &structure,
                                 const Structure::Pool &shapes, const Result &probability, ShapeSolver::Distribution::Pool &distributions,
                                 ObservedBoard::CellId cell, Workspace &workspace,
                                 const ShapeSolver::OrderAlgo &algo = ShapeSolver::OrderAlgo::Auto);
};

} // namespace mss

//==============================================================================
namespace mss {

inline void Probability::Result::reset(std::span<const std::size_t> componentBoxCounts) {
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

inline long double Probability::Result::mineProbability(ObservedBoard::CellId cell, const ObservedBoard::Result &board, const Basic::Result &basic,
                                                        const Structure::Result &structure) const {
    // 按格子的分析归属返回 Mine、Unknown、Safe 或前沿 Box 的概率；逻辑推出
    // 的确定雷/安全格在这里分别表现为浮点精确的 1.0L/0.0L。
    const auto [x, y] = board.pos(cell);
    const Structure::CellLocation loc = structure.cellLoc[cell];
    if (loc.component == -1) {
        if (basic.marks[x][y] == Basic::Mark::Mine)
            return 1.0L;
        if (basic.marks[x][y] == Basic::Mark::Unknown)
            return Probability::limitProbability(tCellProbability_);
        return 0.0L;
    }
    if (loc.box == -1)
        return 0.0L;
    const long double probability = components_[loc.component].boxProbabilities[loc.box];
    return Probability::limitProbability(probability);
}

} // namespace mss

template <typename Callback>
inline void mss::Probability::Result::frontierCells(const mss::ObservedBoard::Result &board, const mss::Structure::Result &structure,
                                                    const mss::Structure::Pool &shapes, Callback &&callback) const {
    // 展开每个前沿 Box 的成员格，并把 Box 概率传给回调。
    for (int cid = 0; cid < (int)(components_.size()); ++cid) {
        const Component &component = components_[cid];
        const Structure::Instance &instance = shapes.getInstance(structure.components[cid]);
        for (int box = 0; box < (int)(component.boxProbabilities.size()); ++box) {
            const long double probability = component.boxProbabilities[box];
            for (int i = instance.boxes.boxOf.span(shapes.boxOf)[box]; i < instance.boxes.boxOf.span(shapes.boxOf)[box + 1]; ++i) {
                const auto [x, y] = board.pos(instance.boxes.cells.span(shapes.cells)[i]);
                callback(x, y, probability);
            }
        }
    }
}
