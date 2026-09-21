#pragma once

#include "algo/shape_solver/graph_solver/graph_solver.h"

namespace mss {

template <typename Callback>
inline void ShapeSolver::GraphSolver::walkSteps(const Structure::Shape &shape, const Structure::Pool &shapes,
                                                 const std::vector<BoxId> &order, Callback &&callback) {
    // 把消元顺序转换为逐步的读取、收集和闭合计划；StepPlan 让 Layer 只读取
    // 当前检查所需的旧槽位，并在约束关闭时输出对应 Box 的矩。
    const int boxCount = shape.boxes.size;
    std::vector<int> position(boxCount);
    for (int step = 0; step < boxCount; ++step)
        position[order[step]] = step;

    const int constraintCount = shape.constraintCount();
    std::vector<int> constraintLast(constraintCount, -1);
    std::vector<int> boxHead(boxCount, -1);
    std::vector<int> nextLink;
    std::vector<int> constraintIds;
    for (int constraint = 0; constraint < constraintCount; ++constraint) {
        const Structure::Shape::ConstraintView view = shape.constraint(shapes, constraint);
        for (BoxId box : view.boxIds)
            constraintLast[constraint] = (std::max)(constraintLast[constraint], position[box]);
        for (BoxId box : view.boxIds) {
            nextLink.push_back(boxHead[box]);
            constraintIds.push_back(constraint);
            boxHead[box] = constraintIds.size() - 1;
        }
    }

    std::vector<int> closeStep = position;
    for (int constraint = 0; constraint < constraintCount; ++constraint)
        for (BoxId box : shape.constraint(shapes, constraint).boxIds)
            closeStep[box] = (std::max)(closeStep[box], constraintLast[constraint]);

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
        plan.boxSize = shape.boxes.span(shapes.boxes)[plan.box].size;
        plan.checks.clear();
        plan.gather.clear();
        plan.closings.clear();

        for (int link = boxHead[plan.box]; link >= 0; link = nextLink[link]) {
            const Structure::Shape::ConstraintView view = shape.constraint(shapes, constraintIds[link]);
            StepPlan::Check check;
            check.sum = view.sum;
            for (BoxId member : view.boxIds) {
                if (position[member] < step)
                    check.readSlots[check.readCount++] = slotOf[member];
                else if (position[member] > step)
                    check.remainingSize += shape.boxes.span(shapes.boxes)[member].size;
            }
            plan.checks.push_back(check);
        }

        nextLayout.clear();
        for (int oldSlot = 0; oldSlot < (int)(layout.size()); ++oldSlot) {
            const BoxId box = layout[oldSlot];
            if (closeStep[box] == step)
                continue;
            plan.gather.push_back(oldSlot);
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
        for (int slot = 0; slot < (int)(nextLayout.size()); ++slot)
            slotOf[nextLayout[slot]] = slot;
        layout.swap(nextLayout);
    }
}

template <typename Plan>
inline void workspace::GraphSolverDp::Layer::advance(const Plan &plan, Layer &nextLayer) const {
        // 执行一步 Graph DP：按当前 Box 可取的雷数转移，按
        // (frontier assignment, total mine count) 合并等价状态，并累计 ways
        // 与每个关闭 Box 的雷数矩。
        nextLayer.states.clear();
        nextLayer.counts.clear();
        nextLayer.momentValues.clear();
        nextLayer.frontierWords.clear();
        nextLayer.index.clear();
        nextLayer.momentBoxes = momentBoxes;
        for (const typename Plan::Closing &closing : plan.closings)
            nextLayer.momentBoxes.push_back(closing.box);
        for (const State &state : states) {
            int minMine = 0;
            int maxMine = plan.boxSize;
            for (const typename Plan::Check &check : plan.checks) {
                int partial = 0;
                for (int i = 0; i < check.readCount; ++i)
                    partial += frontierValue(state, check.readSlots[i]);
                minMine = (std::max)(minMine, check.sum - partial - check.remainingSize);
                maxMine = (std::min)(maxMine, check.sum - partial);
            }
            for (int mine = minMine; mine <= maxMine; ++mine) {
                const std::size_t packedOffset = nextLayer.frontierWords.size();
                for (std::size_t slot = 0; slot < plan.gather.size(); ++slot) {
                    if ((slot & 15) == 0)
                        nextLayer.frontierWords.push_back(0);
                    const int source = plan.gather[slot];
                    const std::uint64_t value = source < 0 ? mine : frontierValue(state, source);
                    nextLayer.frontierWords.back() |= value << ((slot & 15) * 4);
                }
                U128Hasher hasher;
                for (std::size_t i = packedOffset; i < nextLayer.frontierWords.size(); ++i)
                    hasher.mix(nextLayer.frontierWords[i]);
                const U128 hash = hasher.finalize();
                State *target;
                if (const std::size_t *found = nextLayer.index.find(hash)) {
                    nextLayer.frontierWords.resize(packedOffset);
                    target = &nextLayer.states[*found];
                } else {
                    const std::size_t id = nextLayer.states.size();
                    nextLayer.states.push_back({packedOffset, -1, -1});
                    nextLayer.index.emplace(hash, id);
                    target = &nextLayer.states.back();
                }
                const long double factor = binomSmall(plan.boxSize, mine);
                // factor 为 1 时跳过乘法。
                const bool factorIsOne = factor == 1.0L;
                for (int sourceIndex = state.firstCount; sourceIndex >= 0; sourceIndex = counts[sourceIndex].next) {
                    const Count &source = counts[sourceIndex];
                    const int mineCount = source.mineCount + mine;
                    int targetIndex = target->firstCount;
                    while (targetIndex >= 0 && nextLayer.counts[targetIndex].mineCount != mineCount)
                        targetIndex = nextLayer.counts[targetIndex].next;
                    // 新 Count 的首次贡献直接写入未初始化块，后续贡献才累加。
                    const bool firstContribution = targetIndex < 0;
                    if (firstContribution) {
                        targetIndex = nextLayer.counts.size();
                        nextLayer.counts.push_back({mineCount, 0.0L, nextLayer.momentValues.size(), -1});
                        if (target->lastCount >= 0)
                            nextLayer.counts[target->lastCount].next = targetIndex;
                        else
                            target->firstCount = targetIndex;
                        target->lastCount = targetIndex;
                        nextLayer.momentValues.resize(nextLayer.momentValues.size() + nextLayer.momentBoxes.size());
                    }
                    Count &targetCount = nextLayer.counts[targetIndex];
                    const long double ways = factorIsOne ? source.ways : source.ways * factor;
                    targetCount.ways += ways;
                    if (factorIsOne) {
                        for (int slot = 0; slot < (int)(momentBoxes.size()); ++slot) {
                            const long double contribution = momentValues[source.momentOffset + slot].value;
                            MomentValue &targetValue = nextLayer.momentValues[targetCount.momentOffset + slot];
                            targetValue.value = firstContribution ? contribution : targetValue.value + contribution;
                        }
                    } else {
                        for (int slot = 0; slot < (int)(momentBoxes.size()); ++slot) {
                            const long double contribution = momentValues[source.momentOffset + slot].value * factor;
                            MomentValue &targetValue = nextLayer.momentValues[targetCount.momentOffset + slot];
                            targetValue.value = firstContribution ? contribution : targetValue.value + contribution;
                        }
                    }
                    for (int i = 0; i < (int)(plan.closings.size()); ++i) {
                        const typename Plan::Closing &closing = plan.closings[i];
                        const long double boxMine = closing.oldSlot < 0 ? mine : frontierValue(state, closing.oldSlot);
                        const long double contribution = boxMine * ways;
                        MomentValue &targetValue = nextLayer.momentValues[targetCount.momentOffset + momentBoxes.size() + i];
                        targetValue.value = firstContribution ? contribution : targetValue.value + contribution;
                    }
                }
            }
        }
}

inline ShapeSolver::Distribution::Result ShapeSolver::GraphSolver::materialize(const Layer &layer, int boxCount) {
    // 将 Graph DP 的最终层展开为公开的按总雷数分布结果；moment/ways 的比值
    // 还原每个 Box 在该总雷数条件下的期望雷数。
    if (layer.states.empty())
        return {0, boxCount, {}, {}};
    std::vector<int> countIds;
    int start = 0;
    int end = 0;
    bool first = true;
    for (int index = layer.states[0].firstCount; index >= 0; index = layer.counts[index].next) {
        const Layer::Count &count = layer.counts[index];
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
    if (first)
        return {0, boxCount, {}, {}};
    std::vector<std::size_t> order(countIds.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](std::size_t lhs, std::size_t rhs) {
        return layer.counts[countIds[lhs]].mineCount < layer.counts[countIds[rhs]].mineCount;
    });
    std::vector<long double> sortedWays(end - start + 1, 0.0L);
    RawGrid<long double> moments((int)(sortedWays.size()), boxCount, 0.0L);
    for (const BoxId id : order) {
        const Layer::Count &count = layer.counts[countIds[id]];
        sortedWays[count.mineCount - start] = count.ways;
        for (int box = 0; box < boxCount; ++box) {
            long double value = 0;
            for (int slot = 0; slot < (int)(layer.momentBoxes.size()); ++slot)
                if (layer.momentBoxes[slot] == box)
                    value = layer.momentValues[count.momentOffset + slot].value / count.ways;
            moments[count.mineCount - start][box] = value;
        }
    }
    return {start, boxCount, std::move(sortedWays), std::move(moments)};
}

inline DistributionId ShapeSolver::GraphSolver::analyze(const Structure::Shape &shape, const Structure::Pool &shapes,
                                                        ShapeSolver::Distribution::Pool &pool,
                                                        const ShapeSolver::GraphSolver::OrderAlgo &algo) {
    // 构建消元图、执行 Graph DP，并把结果写入分布缓存；先查缓存，命中时不再
    // 重算同一 Shape，未命中时用 algo 控制消元顺序的优化策略。
    const DistributionId cached = pool.find(shape.hash);
    if (cached >= 0)
        return cached;
    const Graph graph = Graph::fromShape(shape, shapes);
    const std::vector<BoxId> order = makeOrder(graph, algo);
    // 当前线程复用两层 DP 容量；reset 只清空逻辑元素。
    Layer &current = workspace::GraphSolverDp::current;
    Layer &next = workspace::GraphSolverDp::next;
    current.reset();
    next.reset();
    walkSteps(shape, shapes, order, [&](const StepPlan &plan) {
        current.advance(plan, next);
        std::swap(current, next);
    });
    ShapeSolver::Distribution::Result result = materialize(current, shape.boxes.size);
    return pool.insert(shape.hash, std::move(result));
}

} // namespace mss
