#pragma once

#include <algorithm>
#include <array>
#include <numeric>
#include <span>
#include <utility>
#include <vector>

#include "algo/shape_solver/shape_solver_common.h"

namespace mss {

struct ShapeSolver::GraphSolver {

    // 图 DP 只保留当前“尚未闭合”的边界状态；order/polish 影响状态峰值，
    // 不改变最终分布。该后端由 ShapeSolver::analyze 在 Box 较多时选用。

public:
    enum class PolishKind {
        Adjacent,
        Window3,
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

public:
    static std::vector<BoxId> makeOrder(const Graph& graph,
                                        PolishKind polish);

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
                                  PolishKind polish);
};

//==============================================================================
inline ShapeSolver::GraphSolver::Graph
ShapeSolver::GraphSolver::Graph::fromShape(const Structure::Shape& shape) {
    // 将每条约束中的 Box 两两连接，构建消元排序使用的邻接图；同一约束中的
    // 任意两个 Box 必须在消元前互相可见，才能在局部状态中检查约束剩余量。
    const int boxCount = static_cast<int>(shape.boxes.size());
    Graph graph;
    graph.offsets.assign(boxCount + 1, 0);

    std::vector<int> head(boxCount, -1);
    std::vector<BoxId> to;
    std::vector<int> next;
    std::vector<char> marked(boxCount, 0);

    auto addEdge = [&](BoxId from, BoxId target) {
        next.push_back(head[from]);
        to.push_back(target);
        head[from] = static_cast<int>(to.size()) - 1;
    };

    for (int i = 0; i < static_cast<int>(shape.constraintCount()); ++i) {
        const Structure::Shape::ConstraintView constraint = shape.constraint(i);
        for (int a = 0; a < static_cast<int>(constraint.boxIds.size()); ++a)
            for (int b = a + 1; b < static_cast<int>(constraint.boxIds.size()); ++b) {
                addEdge(constraint.boxIds[a], constraint.boxIds[b]);
                addEdge(constraint.boxIds[b], constraint.boxIds[a]);
            }
    }

    for (BoxId box = 0; box < boxCount; ++box) {
        for (int edge = head[box]; edge >= 0; edge = next[edge]) {
            const BoxId target = to[edge];
            if (marked[target]) continue;
            marked[target] = 1;
            ++graph.offsets[box + 1];
        }
        for (int edge = head[box]; edge >= 0; edge = next[edge])
            marked[to[edge]] = 0;
    }
    for (int box = 0; box < boxCount; ++box)
        graph.offsets[box + 1] += graph.offsets[box];

    graph.adjacent.resize(graph.offsets.back());
    for (BoxId box = 0; box < boxCount; ++box) {
        int write = graph.offsets[box];
        for (int edge = head[box]; edge >= 0; edge = next[edge]) {
            const BoxId target = to[edge];
            if (marked[target]) continue;
            marked[target] = 1;
            graph.adjacent[write++] = target;
        }
        for (int edge = head[box]; edge >= 0; edge = next[edge])
            marked[to[edge]] = 0;
    }
    return graph;
}

inline std::span<const BoxId>
ShapeSolver::GraphSolver::Graph::neighbors(BoxId box) const {
    // 返回指定 Box 在压缩邻接数组中的邻居视图。
    const int begin = offsets[box];
    const int end = offsets[box + 1];
    return {adjacent.data() + begin, static_cast<std::size_t>(end - begin)};
}

inline std::pair<int, int> ShapeSolver::GraphSolver::orderScore(
    const Graph& graph, const std::vector<BoxId>& order) {
    // 评估一个 Box 顺序的峰值边界宽度和累计边界面积；makeOrder 用它比较
    // Window3 局部排列，优先降低 Graph DP 的峰值状态数。
    const int boxCount = static_cast<int>(order.size());
    std::vector<int> remaining(graph.offsets.size() - 1);
    std::vector<char> selected(remaining.size(), 0);
    for (BoxId box = 0; box < boxCount; ++box)
        remaining[box] = static_cast<int>(graph.neighbors(box).size());

    int frontier = 0;
    int peak = 0;
    int area = 0;
    for (BoxId box : order) {
        if (remaining[box] != 0) ++frontier;
        for (BoxId neighbor : graph.neighbors(box))
            if (selected[neighbor] && remaining[neighbor] == 1)
                --frontier;
        selected[box] = 1;
        for (BoxId neighbor : graph.neighbors(box)) --remaining[neighbor];
        peak = (std::max)(peak, frontier);
        area += frontier;
    }
    return {peak, area};
}

inline std::vector<BoxId> ShapeSolver::GraphSolver::makeOrder(
    const Graph& graph, PolishKind polish) {
    // 用贪心闭合度生成消元顺序，并按策略做局部优化；顺序只影响中间边界，
    // 不改变 walkSteps 最终枚举的赋值集合。
    const int boxCount = static_cast<int>(graph.offsets.size()) - 1;
    std::vector<BoxId> order;
    order.reserve(boxCount);
    std::vector<int> remaining(boxCount);
    std::vector<char> selected(boxCount, 0);
    for (BoxId box = 0; box < boxCount; ++box)
        remaining[box] = static_cast<int>(graph.neighbors(box).size());

    for (int step = 0; step < boxCount; ++step) {
        BoxId best = -1;
        int bestDelta = 0;
        for (BoxId candidate = 0; candidate < boxCount; ++candidate) {
            if (selected[candidate]) continue;
            int closes = 0;
            for (BoxId neighbor : graph.neighbors(candidate))
                if (selected[neighbor] && remaining[neighbor] == 1)
                    ++closes;
            const int delta = static_cast<int>(remaining[candidate] != 0) - closes;
            if (best < 0 || delta < bestDelta) {
                best = candidate;
                bestDelta = delta;
            }
        }

        selected[best] = 1;
        order.push_back(best);
        for (BoxId neighbor : graph.neighbors(best)) --remaining[neighbor];
    }

    if (polish == PolishKind::Window3) {
        for (int start = 0; start < boxCount; start += 3) {
            const int length = (std::min)(3, boxCount - start);
            std::array<BoxId, 3> candidate{};
            std::array<BoxId, 3> best{};
            for (int i = 0; i < length; ++i) {
                candidate[i] = order[start + i];
                best[i] = candidate[i];
            }
            std::pair<int, int> bestScore = orderScore(graph, order);

            auto consider = [&] {
                for (int i = 0; i < length; ++i) order[start + i] = candidate[i];
                const std::pair<int, int> score = orderScore(graph, order);
                if (score < bestScore) {
                    bestScore = score;
                    best = candidate;
                }
            };

            if (length == 1) {
                consider();
            } else if (length == 2) {
                if (candidate[1] < candidate[0]) std::swap(candidate[0], candidate[1]);
                consider();
                std::swap(candidate[0], candidate[1]);
                consider();
            } else {
                std::sort(candidate.begin(), candidate.end());
                do {
                    consider();
                } while (std::next_permutation(candidate.begin(), candidate.end()));
            }
            for (int i = 0; i < length; ++i) order[start + i] = best[i];
        }
    }
    return order;
}

template <typename Callback>
inline void ShapeSolver::GraphSolver::walkSteps(
    const Structure::Shape& shape, const std::vector<BoxId>& order,
    Callback&& callback) {
    // 把消元顺序转换为逐步的读取、收集和闭合计划；StepPlan 让 Layer 只读取
    // 当前检查所需的旧槽位，并在约束关闭时输出对应 Box 的矩。
    const int boxCount = static_cast<int>(shape.boxes.size());
    std::vector<int> position(boxCount);
    for (int step = 0; step < boxCount; ++step) position[order[step]] = step;

    const int constraintCount = static_cast<int>(shape.constraintCount());
    std::vector<int> constraintLast(constraintCount, -1);
    std::vector<int> boxHead(boxCount, -1);
    std::vector<int> nextLink;
    std::vector<int> constraintIds;
    for (int constraint = 0; constraint < constraintCount; ++constraint) {
        const Structure::Shape::ConstraintView view = shape.constraint(constraint);
        for (BoxId box : view.boxIds)
            constraintLast[constraint] =
                (std::max)(constraintLast[constraint], position[box]);
        for (BoxId box : view.boxIds) {
            nextLink.push_back(boxHead[box]);
            constraintIds.push_back(constraint);
            boxHead[box] = static_cast<int>(constraintIds.size()) - 1;
        }
    }

    std::vector<int> closeStep = position;
    for (int constraint = 0; constraint < constraintCount; ++constraint)
        for (BoxId box : shape.constraint(constraint).boxIds)
            closeStep[box] =
                (std::max)(closeStep[box], constraintLast[constraint]);

    std::vector<int> closeHead(boxCount, -1);
    std::vector<int> closeNext(boxCount, -1);
    for (BoxId box = 0; box < boxCount; ++box) {
        closeNext[box] = closeHead[closeStep[box]];
        closeHead[closeStep[box]] = box;
    }

    std::vector<BoxId> layout;
    std::vector<BoxId> nextLayout;
    std::vector<int> slotOf(boxCount, -1);
    StepPlan plan;
    for (int step = 0; step < boxCount; ++step) {
        plan.box = order[step];
        plan.boxSize = shape.boxes[plan.box].size;
        plan.checks.clear();
        plan.gather.clear();
        plan.closings.clear();

        for (int link = boxHead[plan.box]; link >= 0; link = nextLink[link]) {
            const Structure::Shape::ConstraintView view =
                shape.constraint(constraintIds[link]);
            StepPlan::Check check;
            check.sum = view.sum;
            for (BoxId member : view.boxIds) {
                if (position[member] < step)
                    check.readSlots[check.readCount++] = slotOf[member];
                else if (position[member] > step)
                    check.remainingSize += shape.boxes[member].size;
            }
            plan.checks.push_back(check);
        }

        nextLayout.clear();
        for (std::size_t oldSlot = 0; oldSlot < layout.size(); ++oldSlot) {
            const BoxId box = layout[oldSlot];
            if (closeStep[box] == step) continue;
            plan.gather.push_back(static_cast<int>(oldSlot));
            nextLayout.push_back(box);
        }
        if (closeStep[plan.box] > step) {
            plan.gather.push_back(-1);
            nextLayout.push_back(plan.box);
        }
        for (BoxId box = closeHead[step]; box >= 0; box = closeNext[box])
            plan.closings.push_back({box, box == plan.box ? -1 : slotOf[box]});

        callback(plan);

        std::fill(slotOf.begin(), slotOf.end(), -1);
        for (int slot = 0; slot < static_cast<int>(nextLayout.size()); ++slot)
            slotOf[nextLayout[slot]] = slot;
        layout.swap(nextLayout);
    }
}

struct ShapeSolver::GraphSolver::Layer {
    public:

    struct Count {
        int mineCount = 0;
        long double ways = 0;
        std::size_t momentOffset = 0;
        int next = -1;
    };
    struct State {
        std::size_t frontierOffset = 0;
        int firstCount = -1;
        int lastCount = -1;
    };
    std::vector<State> states;
    std::vector<Count> counts;
    std::vector<BoxId> momentBoxes;
    std::vector<long double> momentValues;
    std::vector<char> frontierValues;
    FlatHashTable<U128, std::size_t, U128Hash> index;

    void reset() {
        // 清空 Graph DP 层并恢复“0 个 Box、0 个雷、1 种方式”的初始状态。
        states.clear();
        counts.clear();
        momentBoxes.clear();
        momentValues.clear();
        frontierValues.clear();
        index.clear();
        states.push_back({0, 0, 0});
        counts.push_back({0, 1.0L, 0, -1});
    }

    Count& findOrAddCount(State& state, int mineCount) {
        // 在状态的链表中查找或创建指定累计雷数的计数项。
        for (int i = state.firstCount; i >= 0; i = counts[i].next)
            if (counts[i].mineCount == mineCount) return counts[i];
        const int index = static_cast<int>(counts.size());
        counts.push_back({mineCount, 0.0L, momentValues.size(), -1});
        if (state.lastCount >= 0) counts[state.lastCount].next = index;
        else state.firstCount = index;
        state.lastCount = index;
        momentValues.resize(momentValues.size() + momentBoxes.size(), 0.0L);
        return counts.back();
    }

    void advance(const StepPlan& plan, Layer& nextLayer) const {
        // 执行一步 Graph DP：按当前 Box 可取的雷数转移，按前沿赋值哈希合并
        // 等价状态，并累计 ways 与每个关闭 Box 的雷数矩。
        nextLayer.states.clear();
        nextLayer.counts.clear();
        nextLayer.momentValues.clear();
        nextLayer.frontierValues.clear();
        nextLayer.index.clear();
        nextLayer.momentBoxes = momentBoxes;
        for (const StepPlan::Closing& closing : plan.closings)
            nextLayer.momentBoxes.push_back(closing.box);
        for (const State& state : states) {
            int minMine = 0;
            int maxMine = plan.boxSize;
            for (const StepPlan::Check& check : plan.checks) {
                int partial = 0;
                for (int i = 0; i < check.readCount; ++i)
                    partial += frontierValues[state.frontierOffset + check.readSlots[i]];
                minMine = (std::max)(minMine, check.sum - partial - check.remainingSize);
                maxMine = (std::min)(maxMine, check.sum - partial);
            }
            for (int mine = minMine; mine <= maxMine; ++mine) {
                U128Hasher hasher;
                for (int source : plan.gather) {
                    const char value = source < 0
                        ? static_cast<char>(mine)
                        : frontierValues[state.frontierOffset + source];
                    hasher.mix(static_cast<std::uint64_t>(static_cast<unsigned char>(value)));
                }
                const U128 hash = hasher.finalize();
                State* target;
                if (const std::size_t* found = nextLayer.index.find(hash))
                    target = &nextLayer.states[*found];
                else {
                    const std::size_t id = nextLayer.states.size();
                    nextLayer.states.push_back({nextLayer.frontierValues.size(), -1, -1});
                    for (int source : plan.gather)
                        nextLayer.frontierValues.push_back(source < 0
                            ? static_cast<char>(mine)
                            : frontierValues[state.frontierOffset + source]);
                    nextLayer.index.emplace(hash, id);
                    target = &nextLayer.states.back();
                }
                const long double factor = ShapeSolver::binom(plan.boxSize, mine);
                for (int sourceIndex = state.firstCount; sourceIndex >= 0;
                     sourceIndex = counts[sourceIndex].next) {
                    const Count& source = counts[sourceIndex];
                    Count& targetCount = nextLayer.findOrAddCount(
                        *target, source.mineCount + mine);
                    const long double ways = source.ways * factor;
                    targetCount.ways += ways;
                    for (std::size_t slot = 0; slot < momentBoxes.size(); ++slot)
                        nextLayer.momentValues[targetCount.momentOffset + slot] +=
                            momentValues[source.momentOffset + slot] * factor;
                    for (std::size_t i = 0; i < plan.closings.size(); ++i) {
                        const StepPlan::Closing& closing = plan.closings[i];
                        const long double boxMine = closing.oldSlot < 0
                            ? static_cast<long double>(mine)
                            : static_cast<long double>(
                                frontierValues[state.frontierOffset + closing.oldSlot]);
                        nextLayer.momentValues[targetCount.momentOffset +
                                                momentBoxes.size() + i] += boxMine * ways;
                    }
                }
            }
        }
    }
};

inline ShapeSolver::Distribution::Result
ShapeSolver::GraphSolver::materialize(const Layer& layer, int boxCount) {
    // 将 Graph DP 的最终层展开为公开的按总雷数分布结果；moment/ways 的比值
    // 还原每个 Box 在该总雷数条件下的期望雷数。
    if (layer.states.empty()) return {0, boxCount, {}, {}};
    std::vector<int> countIds;
    int start = 0;
    int end = 0;
    bool first = true;
    for (int index = layer.states[0].firstCount; index >= 0;
         index = layer.counts[index].next) {
        const Layer::Count& count = layer.counts[index];
        countIds.push_back(index);
        if (first) {
            start = count.mineCount;
            end = count.mineCount;
            first = false;
        } else {
            start = (std::min)(start, count.mineCount);
            end = (std::max)(end, count.mineCount);
        }
    }
    if (first) return {0, boxCount, {}, {}};
    std::vector<std::size_t> order(countIds.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](std::size_t lhs, std::size_t rhs) {
        return layer.counts[countIds[lhs]].mineCount <
               layer.counts[countIds[rhs]].mineCount;
    });
    std::vector<long double> sortedWays(end - start + 1, 0.0L);
    std::vector<long double> moments(sortedWays.size() * boxCount, 0.0L);
    for (const std::size_t id : order) {
        const Layer::Count& count = layer.counts[countIds[id]];
        sortedWays[count.mineCount - start] = count.ways;
        const std::size_t offset =
            static_cast<std::size_t>(count.mineCount - start) * boxCount;
        for (int box = 0; box < boxCount; ++box) {
            long double value = 0;
            for (std::size_t slot = 0; slot < layer.momentBoxes.size(); ++slot)
                if (layer.momentBoxes[slot] == box)
                    value = layer.momentValues[count.momentOffset + slot] / count.ways;
            moments[offset + box] = value;
        }
    }
    return {start, boxCount, std::move(sortedWays), std::move(moments)};
}

inline DistributionId ShapeSolver::GraphSolver::analyze(
    const Structure::Shape& shape, ShapeSolver::Distribution::Pool& pool,
    ShapeSolver::GraphSolver::PolishKind polish) {
    // 构建消元图、执行 Graph DP，并把结果写入分布缓存；先查缓存，命中时不再
    // 重算同一 Shape，未命中时用 polish 控制消元顺序的局部优化。
    const DistributionId cached = pool.find(shape.hash);
    if (cached >= 0) return cached;
    const Graph graph = Graph::fromShape(shape);
    const std::vector<BoxId> order = makeOrder(graph, polish);
    Layer current;
    Layer next;
    current.reset();
    walkSteps(shape, order, [&](const StepPlan& plan) {
        current.advance(plan, next);
        std::swap(current, next);
    });
    ShapeSolver::Distribution::Result result = materialize(
        current, static_cast<int>(shape.boxes.size()));
    return pool.insert(shape.hash, std::move(result));
}

}  // namespace mss
