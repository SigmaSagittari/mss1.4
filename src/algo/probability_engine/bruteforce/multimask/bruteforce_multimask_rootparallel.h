#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <span>
#include <thread>
#include <vector>

#include "algo/probability_engine/bruteforce/multimask/bruteforce_multimask.h"
#include "core/config.h"
#include "core/utility/flat_hashtable.h"
#include "core/utility/hash.h"
#include "core/utility/rng.h"

namespace mss {

template <typename Mask>
template <bool CheckAllMoves>
inline int BruteForce::MultiMaskSolver<Mask>::solveCandidatesParallel(const Common &common, Session &s,
                                                                     std::span<ConfigId> configs, int need, Result &result,
                                                                     Worker &workspace) {
    static_assert(kMaxCores > 0);
    MultiMaskWorkspace &maskWorkspace = *workspace.owner;
    const Layer &root = workspace.scratch.layer(0);
    const std::vector<int> order = root.order;
    const std::vector<int> deaths = root.deaths;
    const int count = order.size();
    const int n = configs.size();
    const int threadCount = (std::min)(kMaxCores, count);
    std::atomic<int> next{0};
    // 高位为胜局数，低位为原搜索顺序的反序；同分始终保留原来先尝试的候选。
    std::atomic<long long> best{0};
    std::vector<int> uppers(count);
    std::vector<long long> nodes(threadCount);
    SharedCache shared(common.possibilityCount);
    for (int worker = 0; worker < threadCount; ++worker)
        maskWorkspace.getWorker(worker);
    auto work = [&](int worker) {
        Session session = s;
        session.nodes = 0;
        Result localResult;
        Worker &workerWorkspace = worker == 0 ? maskWorkspace.main : maskWorkspace.workers[worker - 1];
        workerWorkspace.scratch.reset();
        workerWorkspace.cache.clear();
        workerWorkspace.sharedCache = &shared;
        Layer &buf = workerWorkspace.scratch.layer(0);
        for (;;) {
            const int index = next.fetch_add(1, std::memory_order_relaxed);
            if (index >= count)
                break;
            const int candidate = order[index];
            const long long previous = best.load(std::memory_order_relaxed);
            const int bestWins = previous / (count + 1);
            const int bestIndex = count - previous % (count + 1);
            const int target = CheckAllMoves ? 1 : (std::max)(need, bestWins + (index > bestIndex));
            if (!CheckAllMoves && n - deaths[candidate] < target) {
                uppers[index] = n - deaths[candidate];
                continue;
            }
            for (auto &group : buf.groups)
                group.clear();
            const int groupCount = groupByReveal(session, configs, candidate, buf.groups);
            if (!CheckAllMoves && groupCount <= 1)
                continue;
            buf.groupList.clear();
            for (int reveal = 0; reveal < 9; ++reveal)
                if (!buf.groups[reveal].empty())
                    buf.groupList.push_back({reveal, (int)buf.groups[reveal].size()});
            std::sort(buf.groupList.begin(), buf.groupList.end(), [](const auto &a, const auto &b) {
                if (a.second != b.second)
                    return a.second > b.second;
                return a.first < b.first;
            });
            session.unopenedCandidates.reset(candidate);
            BranchRun run;
            run.reached = true;
            int remaining = n - deaths[candidate];
            for (const auto &[reveal, size] : buf.groupList) {
                // 每个数字分支开始前读取其他线程已证明的最优值，及时收紧阈值。
                const long long current = best.load(std::memory_order_relaxed);
                const int required = CheckAllMoves ? 1 : (std::max)(need, (int)(current / (count + 1)) + (index > count - current % (count + 1)));
                remaining -= size;
                run.upper = run.wins + size + remaining;
                if (run.upper < required) {
                    run.reached = false;
                    break;
                }
                const int value = solve<false, false>(common, session, buf.groups[reveal],
                                                      (std::max)(1, required - run.wins - remaining), 1,
                                                      workerWorkspace.cache, localResult, workerWorkspace);
                if (value <= 0) {
                    run.upper = run.wins - value + remaining;
                    run.reached = false;
                    break;
                }
                run.wins += value;
            }
            session.unopenedCandidates.set(candidate);
            if constexpr (CheckAllMoves)
                result.moves[candidate] = {common.candidates[candidate].x, common.candidates[candidate].y, run.wins};
            if (!run.reached) {
                uppers[index] = run.upper;
                continue;
            }
            const long long value = (long long)run.wins * (count + 1) + count - index;
            long long old = best.load(std::memory_order_relaxed);
            while (old < value && !best.compare_exchange_weak(old, value, std::memory_order_relaxed)) {}
        }
        nodes[worker] = session.nodes;
        workerWorkspace.sharedCache = nullptr;
    };
    // 调用线程也参与搜索，总活跃搜索线程数不会超过配置上限。
    std::vector<std::jthread> threads;
    for (int worker = 1; worker < threadCount; ++worker)
        threads.emplace_back(work, worker);
    work(0);
    for (auto &thread : threads)
        thread.join();
    for (long long visited : nodes)
        s.nodes += visited;
    const long long value = best.load(std::memory_order_relaxed);
    const int wins = value / (count + 1);
    if constexpr (CheckAllMoves)
        return wins;
    if (wins >= need) {
        const int candidate = order[count - value % (count + 1)];
        result.moves[0] = {common.candidates[candidate].x, common.candidates[candidate].y, wins};
        return wins;
    }
    const int upper = *std::max_element(uppers.begin(), uppers.end());
    if (wins == 0 && upper == 0) {
        writeFirstSafeMove(common, s, configs[0], result);
        return 1;
    }
    return -(std::max)(wins, upper);
}

} // namespace mss
