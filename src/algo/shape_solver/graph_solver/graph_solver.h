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

#include "algo/shape_solver/shape_solver_common.h"
#include "core/assert.h"
#include "core/utility/rng.h"

namespace mss {

struct ShapeSolver::GraphSolver {

    // 图 DP 只保留当前“尚未闭合”的边界状态；order/algo 影响状态峰值，
    // 不改变最终分布。该后端由 ShapeSolver::analyze 在 Box 较多时选用。

public:
    enum class OrderAlgo {
        Adjacent,
        Window3,
        SA,
        Auto,
    };

    struct Graph {
        std::vector<int> offsets;
        std::vector<BoxId> adjacent;

        static Graph fromShape(const Structure::Shape& shape);
        std::span<const BoxId> neighbors(BoxId box) const;
    };

private:
    static std::pair<int, int> orderScore(
        const Graph& graph, const std::vector<BoxId>& order);
    static BoxId farthestBox(const Graph& graph, BoxId source);
    static std::vector<BoxId> makeGreedyOrder(
        const Graph& graph, BoxId first, bool lookahead);
    static std::vector<BoxId> makeWindow3Order(
        const Graph& graph, std::vector<BoxId> order);
    static std::vector<BoxId> makeSAOrder(
        const Graph& graph, std::vector<BoxId> seed);
    static std::vector<BoxId> makeAutoOrder(const Graph& graph);

public:
    static std::vector<BoxId> makeOrder(const Graph& graph,
                                        OrderAlgo algo);

    struct StepPlan {
        struct Check {
            int sum = 0;
            int remainingSize = 0;
            std::array<int, 8> readSlots{};
            int readCount = 0;
        };

        struct Closing {
            BoxId box = 0;
            int oldSlot = -1;
        };

        BoxId box = 0;
        int boxSize = 0;
        std::vector<Check> checks;
        std::vector<int> gather;
        std::vector<Closing> closings;
    };

    template <typename Callback>
    static void walkSteps(const Structure::Shape& shape,
                          const std::vector<BoxId>& order, Callback&& callback);

private:
    struct Layer;
    static ShapeSolver::Distribution::Result materialize(const Layer& layer,
                                                         int boxCount);

public:

    // Graph DP 后端的普通分布求解。
    static DistributionId analyze(const Structure::Shape& shape,
                                  Distribution::Pool& pool,
                                  OrderAlgo algo);
};

}  // namespace mss

#include "algo/shape_solver/graph_solver/graph_solver_order.h"
#include "algo/shape_solver/graph_solver/graph_solver_dp.h"
