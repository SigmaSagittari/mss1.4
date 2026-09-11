#include "algo/shape_solver/graph_solver.h"

#include <algorithm>
#include <array>
#include <numeric>
#include <utility>

#include "algo/shape_solver/dfs_solver.h"

namespace mss {

namespace ShapeSolver {

namespace GraphSolver {

namespace {

using detail::StepPlan;

struct Layer {
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
        for (int i = state.firstCount; i >= 0; i = counts[i].next)
            if (counts[i].mineCount == mineCount) return counts[i];

        const int index = static_cast<int>(counts.size());
        counts.push_back({mineCount, 0.0L, momentValues.size(), -1});
        momentValues.resize(momentValues.size() + momentBoxes.size(), 0.0L);
        if (state.lastCount >= 0)
            counts[state.lastCount].next = index;
        else
            state.firstCount = index;
        state.lastCount = index;
        return counts.back();
    }

    void advance(const StepPlan& plan, Layer& nextLayer) const {
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
                    hasher.mix(static_cast<std::uint64_t>(
                        static_cast<unsigned char>(value)));
                }
                const U128 hash = hasher.finalize();

                State* target;
                if (const std::size_t* found = nextLayer.index.find(hash)) {
                    target = &nextLayer.states[*found];
                } else {
                    const std::size_t id = nextLayer.states.size();
                    nextLayer.states.push_back({nextLayer.frontierValues.size(), -1, -1});
                    for (int source : plan.gather)
                        nextLayer.frontierValues.push_back(
                            source < 0
                                ? static_cast<char>(mine)
                                : frontierValues[state.frontierOffset + source]);
                    nextLayer.index.emplace(hash, id);
                    target = &nextLayer.states.back();
                }

                const long double factor = DfsSolver::detail::binom(plan.boxSize, mine);
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




Distribution::Result materialize(const Layer& layer, int boxCount) {
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

}  // namespace


DistributionId analyze(const Structure::Shape& shape, Distribution::Pool& pool,
                       PolishKind polish) {
    const DistributionId cached = pool.find(shape.hash);
    if (cached >= 0) return cached;

    const detail::Graph graph = detail::Graph::fromShape(shape);
    const std::vector<BoxId> order = detail::makeOrder(graph, polish);

    Layer current;
    Layer next;
    current.reset();
    detail::walkSteps(shape, order, [&](const StepPlan& plan) {
        current.advance(plan, next);
        std::swap(current, next);
    });

    Distribution::Result result = materialize(current,
                                              static_cast<int>(shape.boxes.size()));
    return pool.insert(shape.hash, std::move(result));
}

}  // namespace GraphSolver

}  // namespace ShapeSolver

}  // namespace mss
