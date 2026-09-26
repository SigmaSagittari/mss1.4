#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <span>
#include <utility>
#include <vector>

#include "algo/probability_engine/shape_solver/shape_solver_common.h"
#include "core/assert.h"
#include "core/utility/rng.h"

namespace mss {

namespace ShapeSolver {

struct GraphSolver {

    // 图 DP 只保留当前“尚未闭合”的边界状态；order/algo 影响状态峰值，
    // 不改变最终分布。该后端由 ShapeSolver::analyze 在 Box 较多时选用。

  public:
    using OrderAlgo = ShapeSolver::OrderAlgo;
    using Workspace = ShapeSolver::Workspace::Graph;

    struct Graph {
        std::vector<int> offsets;
        std::vector<Structure::BoxId> adjacent;

        static Graph fromShape(const Structure::Shape &shape, const Structure::Pool &shapes);
        std::span<const Structure::BoxId> neighbors(Structure::BoxId box) const;
    };

  private:
    static std::pair<int, int> orderScore(const Graph &graph, const std::vector<Structure::BoxId> &order, Workspace &workspace);
    static Structure::BoxId farthestBox(const Graph &graph, Structure::BoxId source);
    static std::vector<Structure::BoxId> makeGreedyOrder(const Graph &graph, Structure::BoxId first, bool lookahead);
    static std::vector<Structure::BoxId> makeWindow3Order(const Graph &graph, std::vector<Structure::BoxId> order, Workspace &workspace);
    static std::vector<Structure::BoxId> makeSAOrder(const Graph &graph, std::vector<Structure::BoxId> seed, Workspace &workspace);
    static std::vector<Structure::BoxId> makeAutoOrder(const Graph &graph, Workspace &workspace);
    static std::vector<Structure::BoxId> makeAutoSAOrder(const Graph &graph, Workspace &workspace);

  public:
    static std::vector<Structure::BoxId> makeOrder(const Graph &graph, const OrderAlgo &algo, Workspace &workspace);

    // 一个消元步骤的完整转移计划：处理一个 Box，读取旧 frontier，生成新
    // frontier，并记录本步刚好闭合的 Box。
    struct StepPlan {
        struct Check {
            // 该约束要求的总雷数。
            int sum = 0;
            // 该约束中尚未处理的 Box 最多还能贡献的雷数。
            int remainingSize = 0;
            // 该约束中已经处理的 Box 在旧 frontierValues 中的 slot 下标。
            std::array<int, 8> readSlots{};
            // readSlots 中实际有效的下标数量。
            int readCount = 0;
        };

        struct Closing {
            // 已闭合 Box 的 ID。
            Structure::BoxId box = 0;
            // 该 Box 在旧 frontier 中的 slot；当前正在处理的 Box 用 -1 表示，
            // 因为它的雷数来自本步骤的 mine，而不是旧 frontier。
            int oldSlot = -1;
        };

        // 本步骤正在分配雷数的 Box。
        Structure::BoxId box = 0;
        // box 中的格子数；mine 的取值范围首先是 [0, boxSize]。
        int boxSize = 0;
        // 所有包含 box 的约束；advance 用它们从旧 frontier 计算 mine 的合法范围。
        std::vector<Check> checks;
        // 新 frontier 的来源，按新 frontier 顺序排列：非负值表示复制旧
        // frontierValues 的对应 slot，-1 表示插入本步骤的 mine。
        std::vector<int> gather;
        // 本步骤结束时闭合的所有 Box；它们不再进入新 frontier，但要生成
        // 对应的 momentValues。
        std::vector<Closing> closings;
    };

    template <typename Callback>
    static void walkSteps(const Structure::Shape &shape, const Structure::Pool &shapes, const std::vector<Structure::BoxId> &order,
                          Callback &&callback);

  private:
    using Layer = Workspace::Layer;
    static ShapeSolver::Distribution::Result materialize(const Layer &layer, int boxCount);

  public:
    // Graph DP 后端的普通分布求解。
    static ShapeSolver::DistributionId analyze(const Structure::Shape &shape, const Structure::Pool &shapes, Distribution::Pool &pool,
                                  const OrderAlgo &algo, Workspace &workspace);
};

} // namespace ShapeSolver

} // namespace mss

#include "algo/probability_engine/shape_solver/graph_solver/graph_solver_dp.h"
#include "algo/probability_engine/shape_solver/graph_solver/graph_solver_order.h"
