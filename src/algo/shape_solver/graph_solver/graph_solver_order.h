#pragma once

#include "algo/shape_solver/graph_solver/graph_solver.h"

namespace mss {

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
    // 局部排列和模拟退火结果，优先降低 Graph DP 的峰值状态数。
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

inline BoxId ShapeSolver::GraphSolver::farthestBox(
    const Graph& graph, BoxId source) {
    const int boxCount = static_cast<int>(graph.offsets.size()) - 1;
    std::vector<int> distance(boxCount, -1);
    std::vector<BoxId> queue;
    queue.reserve(boxCount);
    distance[source] = 0;
    queue.push_back(source);
    BoxId farthest = source;
    for (int head = 0; head < static_cast<int>(queue.size()); ++head) {
        const BoxId box = queue[head];
        if (distance[box] > distance[farthest] ||
            (distance[box] == distance[farthest] && box < farthest))
            farthest = box;
        for (BoxId neighbor : graph.neighbors(box))
            if (distance[neighbor] < 0) {
                distance[neighbor] = distance[box] + 1;
                queue.push_back(neighbor);
            }
    }
    return farthest;
}

inline std::vector<BoxId> ShapeSolver::GraphSolver::makeGreedyOrder(
    const Graph& graph, BoxId first, bool lookahead) {
    const int boxCount = static_cast<int>(graph.offsets.size()) - 1;
    const BoxId fallback = first >= 0 ? first : farthestBox(graph, 0);
    std::vector<BoxId> order;
    order.reserve(boxCount);
    std::vector<int> remaining(boxCount);
    std::vector<char> selected(boxCount, 0);
    for (BoxId box = 0; box < boxCount; ++box)
        remaining[box] = static_cast<int>(graph.neighbors(box).size());

    int frontier = 0;
    for (int step = 0; step < boxCount; ++step) {
        BoxId best = -1;
        if (step == 0 && first >= 0) {
            best = first;
        } else {
            int bestDelta = 0;
            std::vector<BoxId> ties;
            for (BoxId candidate = 0; candidate < boxCount; ++candidate) {
                if (selected[candidate]) continue;
                int closes = 0;
                for (BoxId neighbor : graph.neighbors(candidate))
                    if (selected[neighbor] && remaining[neighbor] == 1)
                        ++closes;
                const int delta =
                    static_cast<int>(remaining[candidate] != 0) - closes;
                if (best < 0 || delta < bestDelta) {
                    best = candidate;
                    bestDelta = delta;
                    if (lookahead) {
                        ties.clear();
                        ties.push_back(candidate);
                    }
                } else if (lookahead && delta == bestDelta) {
                    ties.push_back(candidate);
                }
            }

            if (lookahead && ties.size() > 1) {
                std::pair<int, int> previewBest{boxCount + 1, boxCount + 1};
                for (BoxId candidate : ties) {
                    std::vector<int> nextRemaining = remaining;
                    std::vector<char> nextSelected = selected;
                    int nextFrontier = frontier;
                    if (nextRemaining[candidate] != 0) ++nextFrontier;
                    for (BoxId neighbor : graph.neighbors(candidate))
                        if (nextSelected[neighbor] &&
                            nextRemaining[neighbor] == 1)
                            --nextFrontier;
                    nextSelected[candidate] = 1;
                    for (BoxId neighbor : graph.neighbors(candidate))
                        --nextRemaining[neighbor];

                    int nextDelta = 0;
                    BoxId nextBest = -1;
                    for (BoxId follow = 0; follow < boxCount; ++follow) {
                        if (nextSelected[follow]) continue;
                        int closes = 0;
                        for (BoxId neighbor : graph.neighbors(follow))
                            if (nextSelected[neighbor] &&
                                nextRemaining[neighbor] == 1)
                                ++closes;
                        const int delta = static_cast<int>(
                            nextRemaining[follow] != 0) - closes;
                        if (nextBest < 0 || delta < nextDelta) {
                            nextBest = follow;
                            nextDelta = delta;
                        }
                    }
                    const std::pair<int, int> preview{
                        (std::max)(nextFrontier, nextFrontier + nextDelta),
                        nextDelta};
                    if (preview < previewBest) {
                        previewBest = preview;
                        best = candidate;
                    }
                }
            }
        }

        if (best < 0) best = fallback;
        if (remaining[best] != 0) ++frontier;
        for (BoxId neighbor : graph.neighbors(best))
            if (selected[neighbor] && remaining[neighbor] == 1)
                --frontier;
        selected[best] = 1;
        order.push_back(best);
        for (BoxId neighbor : graph.neighbors(best)) --remaining[neighbor];
    }
    return order;
}

inline std::vector<BoxId> ShapeSolver::GraphSolver::makeWindow3Order(
    const Graph& graph, std::vector<BoxId> order) {
    const int boxCount = static_cast<int>(graph.offsets.size()) - 1;
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
            for (int i = 0; i < length; ++i)
                order[start + i] = candidate[i];
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
    return order;
}

inline std::vector<BoxId> ShapeSolver::GraphSolver::makeSAOrder(
    const Graph& graph, std::vector<BoxId> seed) {
    const int boxCount = static_cast<int>(graph.offsets.size()) - 1;
    auto nextRandom = [](std::uint64_t& state) {
        const std::uint64_t value = splitmix64(state);
        state += 0x9e3779b97f4a7c15ULL;
        return value;
    };
    auto unitRandom = [&](std::uint64_t& state) {
        return static_cast<double>(nextRandom(state) >> 11) *
               (1.0 / 9007199254740992.0);
    };
    auto energy = [](const std::pair<int, int> score) {
        return score.first * 20000 + score.second;
    };

    constexpr int rounds = 8;
    constexpr int iterations = 50000;
    constexpr double startTemperature = 40000.0;
    constexpr double endTemperature = 0.1;
    const double cooling = std::pow(
        endTemperature / startTemperature,
        1.0 / static_cast<double>(iterations - 1));
    std::vector<BoxId> bestOrder = seed;
    std::pair<int, int> bestScore = orderScore(graph, bestOrder);
    for (int round = 0; round < rounds; ++round) {
        std::uint64_t state = 0x9e3779b97f4a7c15ULL +
                              static_cast<std::uint64_t>(round) *
                                  0x6a09e667f3bcc909ULL;
        std::vector<BoxId> current = seed;
        for (int perturb = 0; perturb < round; ++perturb) {
            const int left = static_cast<int>(nextRandom(state) % boxCount);
            const int right = static_cast<int>(nextRandom(state) % boxCount);
            std::swap(current[left], current[right]);
        }
        std::pair<int, int> currentScore = orderScore(graph, current);
        int currentEnergy = energy(currentScore);
        double temperature = startTemperature;
        for (int iteration = 0; iteration < iterations; ++iteration) {
            std::vector<BoxId> candidate = current;
            const int left = static_cast<int>(nextRandom(state) % boxCount);
            const int right = static_cast<int>(nextRandom(state) % boxCount);
            const bool swap = nextRandom(state) % 2 == 0;
            if (swap) {
                std::swap(candidate[left], candidate[right]);
            } else {
                const BoxId value = candidate[left];
                if (left < right) {
                    for (int i = left; i < right; ++i)
                        candidate[i] = candidate[i + 1];
                } else if (right < left) {
                    for (int i = left; i > right; --i)
                        candidate[i] = candidate[i - 1];
                }
                candidate[right] = value;
            }
            const std::pair<int, int> candidateScore =
                orderScore(graph, candidate);
            const int candidateEnergy = energy(candidateScore);
            const int difference = currentEnergy - candidateEnergy;
            const bool accept =
                difference >= 0 ||
                unitRandom(state) <
                    std::exp(static_cast<double>(difference) / temperature);
            if (candidateScore < bestScore) {
                bestOrder = candidate;
                bestScore = candidateScore;
            }
            if (accept) {
                current = std::move(candidate);
                currentScore = candidateScore;
                currentEnergy = candidateEnergy;
            }
            temperature *= cooling;
        }
    }
    return bestOrder;
}

inline std::vector<BoxId> ShapeSolver::GraphSolver::makeAutoOrder(
    const Graph& graph) {
    const BoxId initial = farthestBox(graph, 0);
    std::vector<BoxId> best = makeGreedyOrder(graph, initial, false);
    std::pair<int, int> bestScore = orderScore(graph, best);
    if (bestScore.first <= 15) return best;

    std::vector<BoxId> candidate = makeWindow3Order(graph, best);
    std::pair<int, int> candidateScore = orderScore(graph, candidate);
    if (candidateScore < bestScore) {
        best = candidate;
        bestScore = candidateScore;
    }
    if (bestScore.first <= 15) return best;

    candidate = makeSAOrder(graph, makeGreedyOrder(graph, -1, true));
    candidateScore = orderScore(graph, candidate);
    if (candidateScore < bestScore) best = std::move(candidate);
    return best;
}

inline std::vector<BoxId> ShapeSolver::GraphSolver::makeOrder(
    const Graph& graph, OrderAlgo algo) {
    const BoxId initial = farthestBox(graph, 0);
    switch (algo) {
    case OrderAlgo::Adjacent:
        return makeGreedyOrder(graph, initial, false);
    case OrderAlgo::Window3:
        return makeWindow3Order(graph, makeGreedyOrder(graph, initial, false));
    case OrderAlgo::SA:
        return makeSAOrder(graph, makeGreedyOrder(graph, -1, true));
    case OrderAlgo::Auto:
        return makeAutoOrder(graph);
    }
    std::abort();
}

}  // namespace mss
