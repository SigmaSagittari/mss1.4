#pragma once

#include <algorithm>
#include <array>
#include <deque>
#include <utility>

#include "algo/bruteforce/bruteforce_common.h"

namespace mss {

//==============================================================================

inline workspace::BruteForceNormal::Scratch::Layer &workspace::BruteForceNormal::Scratch::layer(int depth) {
    // 取得指定搜索深度的普通后端临时缓冲层。
    if ((int)(layers.size()) <= depth)
        layers.emplace_back();
    return layers[depth];
}

inline void workspace::BruteForceNormal::Scratch::reset() {
    // 清空所有搜索临时容器并保留容量，供下一次残局搜索复用。
    for (Layer &l : layers) {
        l.deaths.clear();
        l.safeCells.clear();
        l.order.clear();
        for (std::vector<std::uint32_t> &g : l.groups)
            g.clear();
        l.groupList.clear();
        l.safeHashes.clear();
        l.safeGroupIds.clear();
        l.safeGroupSizes.clear();
        l.safeGroupOffsets.clear();
        l.safeGroupedConfigs.clear();
        l.safeGroupList.clear();
    }
    safeGroupTable.clear();
}

inline BruteForce::Session BruteForce::buildSession(const CommonSession &common) {
    // 构建普通后端的“方案×候选格”雷表和揭示数字表；这是空间换时间，
    // 让 solve 的递归只处理方案分组，不重复沿 links 计算揭示数字。
    Session session;
    session.mineByConfig.resize(common.possibilityCount, common.candidateCount, 0);
    for (int config = 0; config < common.possibilityCount; ++config) {
        for (int i = 0; i < common.minesPerConfig; ++i)
            session.mineByConfig[config][common.mineCandidateIds[config][i]] = 1;
    }
    session.revealByConfig.resize(common.possibilityCount, common.candidateCount, 0);
    for (int config = 0; config < common.possibilityCount; ++config)
        for (int candidate = 0; candidate < common.candidateCount; ++candidate) {
            int value = common.candidates[candidate].fixedMines;
            const CommonSession::Candidate &current = common.candidates[candidate];
            for (std::uint32_t i = 0; i < current.linksCount; ++i)
                value += session.mineByConfig[config][common.links[current.linksOffset + i]];
            session.revealByConfig[config][candidate] = value;
        }
    return session;
}

template <bool CheckAllMoves, bool IsRoot>
inline int BruteForce::solve(const CommonSession &common, Session &s, std::span<ConfigId> configs, int need, int depth,
                             FlatHashTable<U128, int, U128Hash> &table, Result &result) {
    // 返回值采用带符号的阈值协议：正数表示当前 configs 的精确可赢方案数；
    // 负数表示无法达到 need，绝对值是已证明的可赢上界。上层只在 value>0 时
    // 把该分支计入 wins，因此负数不会被误当成“负的胜局数”。
    // CheckAllMoves 只在根节点展开所有首步；IsRoot 控制是否把推荐动作写入 result。
    if constexpr (CheckAllMoves && IsRoot) {
        ++s.nodes;
        const int n = configs.size();
        result.moves.clear();
        // 方案总数本身不足 need，直接返回失败上界 -n。
        if (need > n)
            return -n;
        ScratchBuffers::Layer &buf = workspace::BruteForceNormal::scratch.layer(depth);
        const int m = common.candidates.size();
        std::vector<int> &deaths = buf.deaths;
        deaths.assign(m, 0);
        for (ConfigId ci : configs) {
            for (int i = 0; i < common.minesPerConfig; ++i)
                ++deaths[common.mineCandidateIds[ci][i]];
        }
        int best = 0;
        std::array<std::vector<ConfigId>, 9> &groups = buf.groups;
        s.unopenedCandidates.for_each([&](std::size_t j) {
            const CandidateId candidate = (CandidateId)j;
            for (std::vector<ConfigId> &g : groups)
                g.clear();
            for (ConfigId ci : configs)
                if (!s.mineByConfig[ci][candidate])
                    groups[s.revealByConfig[ci][candidate]].push_back(ci);
            s.unopenedCandidates.reset(j);
            int wins = 0;
            for (int r = 0; r < 9; ++r)
                if (!groups[r].empty()) {
                    const int value = solve<false, false>(common, s, groups[r], 1, depth + 1, table, result);
                    if (value > 0)
                        wins += value;
                }
            s.unopenedCandidates.set(j);
            result.moves.push_back({common.candidates[j].x, common.candidates[j].y, wins});
            best = (std::max)(best, wins);
        });
        return best;
    } else {
        ++s.nodes;
        const int n = configs.size();
        if (n <= 1) {
            // 单方案节点只能贡献 0/1；不足 need 时仍按同一负上界协议返回。
            if (need > n)
                return -n;
            if constexpr (IsRoot)
                if (n == 1)
                    for (int j = 0; j < (int)(common.candidates.size()); ++j)
                        if (!s.mineByConfig[configs[0]][j]) {
                            result.moves[0].x = common.candidates[j].x;
                            result.moves[0].y = common.candidates[j].y;
                            break;
                        }
            return n;
        }
        // 不可能从 n 个方案中拿到 need 个胜利方案。
        if (need > n)
            return -n;
        const U128 key = hashConfigs(configs);
        // 缓存正值可以直接作为精确结果；负值只有在其上界仍小于 need 时才足以
        // 证明本次调用失败，否则必须继续搜索更高的阈值。
        if (const int *cached = table.find(key)) {
            if (*cached >= 0)
                return *cached >= need ? *cached : -*cached;
            if (-*cached < need)
                return *cached;
        }
        ScratchBuffers::Layer &buf = workspace::BruteForceNormal::scratch.layer(depth);
        const int m = common.candidates.size();
        std::vector<int> &deaths = buf.deaths;
        deaths.assign(m, 0);
        for (ConfigId ci : configs) {
            for (int i = 0; i < common.minesPerConfig; ++i)
                ++deaths[common.mineCandidateIds[ci][i]];
        }
        std::vector<int> &safeCells = buf.safeCells;
        safeCells.clear();
        s.unopenedCandidates.for_each([&](std::size_t j) {
            if (deaths[j] == 0)
                safeCells.push_back((int)j);
        });
        if (!safeCells.empty()) {
            // 所有方案都认为这些格安全；一次同时打开它们后，只需按揭示向量分组，
            // 不必逐格创建等价的递归子问题。
            if constexpr (IsRoot) {
                result.moves[0].x = common.candidates[safeCells[0]].x;
                result.moves[0].y = common.candidates[safeCells[0]].y;
            }
            for (int j : safeCells)
                s.unopenedCandidates.reset(j);
            std::vector<U128> &hashes = buf.safeHashes;
            hashes.clear();
            const std::size_t keyLen = safeCells.size();
            for (ConfigId ci : configs) {
                U128Hasher hasher;
                for (int i = 0; i < (int)(keyLen); ++i)
                    hasher.mix((std::uint64_t)(s.revealByConfig[ci][safeCells[i]]) * (keyLen + 1) + i);
                hashes.push_back(hasher.finalize());
            }
            std::vector<std::span<ConfigId>> &groupList = buf.safeGroupList;
            groupList.clear();
            FlatHashTable<U128, int, U128Hash> &groupTable = workspace::BruteForceNormal::scratch.safeGroupTable;
            std::vector<int> &groupIds = buf.safeGroupIds;
            std::vector<int> &groupSizes = buf.safeGroupSizes;
            std::vector<int> &groupOffsets = buf.safeGroupOffsets;
            std::vector<ConfigId> &groupedConfigs = buf.safeGroupedConfigs;
            groupTable.clear();
            groupTable.reserve(hashes.size());
            groupIds.resize(hashes.size());
            groupSizes.clear();
            for (int i = 0; i < (int)(hashes.size()); ++i) {
                int &slot = groupTable[hashes[i]];
                if (slot == 0) {
                    slot = (int)groupSizes.size() + 1;
                    groupSizes.push_back(0);
                }
                groupIds[i] = slot - 1;
                ++groupSizes[groupIds[i]];
            }
            groupOffsets.resize(groupSizes.size() + 1);
            groupOffsets[0] = 0;
            for (int i = 0; i < (int)(groupSizes.size()); ++i)
                groupOffsets[i + 1] = groupOffsets[i] + groupSizes[i];
            groupedConfigs.resize(configs.size());
            for (int i = 0; i < (int)(groupSizes.size()); ++i)
                groupSizes[i] = groupOffsets[i];
            for (int i = 0; i < (int)(configs.size()); ++i)
                groupedConfigs[groupSizes[groupIds[i]]++] = configs[i];
            for (int i = 0; i < (int)(groupSizes.size()); ++i)
                groupList.emplace_back(groupedConfigs.data() + groupOffsets[i], groupOffsets[i + 1] - groupOffsets[i]);
            std::sort(groupList.begin(), groupList.end(), [](auto a, auto b) {
                if (a.size() != b.size())
                    return a.size() > b.size();
                return a.data() < b.data();
            });
            int wins = 0;
            bool bailed = false;
            int upper = 0;
            // remaining 是【尚未展开的桶】里的方案数上界，初值 n（一个桶都还没算）；
            // 每轮先扣掉本桶，于是 wins + size + remaining 是乐观上界。
            int remaining = n;
            for (int i = 0; i < (int)(groupList.size()); ++i) {
                const int size = groupList[i].size();
                remaining -= size;
                if (wins + size + remaining < need) {
                    upper = wins + size + remaining;
                    bailed = true;
                    break;
                }
                const int value =
                    solve<false, false>(common, s, groupList[i], (std::max)(1, need - wins - remaining), depth + 1, table, result);
                if (value <= 0) {
                    upper = wins - value + remaining;
                    bailed = true;
                    break;
                }
                wins += value;
            }
            for (int j : safeCells)
                s.unopenedCandidates.set(j);
            if (bailed) {
                // 当前安全集合无法达到 need；upper 是尚未展开分支也不可能超过的总上界。
                saveFail(key, upper, n, table);
                return -upper;
            }
            table[key] = wins;
            return wins;
        }

        std::vector<int> &order = buf.order;
        order.clear();
        s.unopenedCandidates.for_each([&](std::size_t j) {
            order.push_back((int)j);
        });
        std::sort(order.begin(), order.end(), [&](int a, int b) {
            if (deaths[a] != deaths[b])
                return deaths[a] < deaths[b];
            return a < b;
        });
        int best = 0;
        int upper = 0;
        for (int j : order) {
            const int target = (std::max)(best + 1, need);
            if (n - deaths[j] < target) {
                upper = (std::max)(upper, n - deaths[j]);
                break;
            }
            std::array<std::vector<ConfigId>, 9> &groups = buf.groups;
            for (std::vector<ConfigId> &g : groups)
                g.clear();
            int groupCount = 0;
            for (ConfigId ci : configs)
                if (!s.mineByConfig[ci][j]) {
                    const int r = s.revealByConfig[ci][j];
                    if (groups[r].empty())
                        ++groupCount;
                    groups[r].push_back(ci);
                }
            if (groupCount <= 1) {
                // 该操作平白丢掉候选为雷的配置，却没有把存活配置分成不同数字分支，
                // 所以是纯亏：它可能是最优操作，但一定不是唯一的最优操作，可以跳过。
                continue;
            }
            std::vector<std::pair<int, int>> &groupList = buf.groupList;
            groupList.clear();
            for (int r = 0; r < 9; ++r)
                if (!groups[r].empty())
                    groupList.push_back({r, (int)groups[r].size()});
            std::sort(groupList.begin(), groupList.end(), [](const auto &a, const auto &b) {
                if (a.second != b.second)
                    return a.second > b.second;
                return a.first < b.first;
            });
            s.unopenedCandidates.reset(j);
            int wins = 0;
            bool bailed = false;
            int moveUpper = 0;
            // 与 safe 分支同一套乐观上界，只是初值换成 n - deaths[j]：
            // 认为该候选是雷的方案已经出局，本来就不该计入任何桶。
            int remaining = n - deaths[j];
            for (int i = 0; i < (int)(groupList.size()); ++i) {
                std::vector<ConfigId> &group = groups[groupList[i].first];
                remaining -= groupList[i].second;
                if (wins + groupList[i].second + remaining < target) {
                    moveUpper = wins + groupList[i].second + remaining;
                    bailed = true;
                    break;
                }
                const int value =
                    solve<false, false>(common, s, group, (std::max)(1, target - wins - remaining), depth + 1, table, result);
                if (value <= 0) {
                    moveUpper = wins - value + remaining;
                    bailed = true;
                    break;
                }
                wins += value;
            }
            s.unopenedCandidates.set(j);
            if (bailed)
                upper = (std::max)(upper, moveUpper);
            if (!bailed && wins > best) {
                best = wins;
                if constexpr (IsRoot) {
                    result.moves[0].x = common.candidates[j].x;
                    result.moves[0].y = common.candidates[j].y;
                }
            }
        }
        if (best >= need) {
            table[key] = best;
            return best;
        }
        // upper == 0 说明所有操作都没有信息增益；此时只能在全部 configs 中猜中一条，
        // 因而胜利线数精确为 1，与本次 need 无关。
        if (best == 0 && upper == 0) {
            if constexpr (IsRoot)
                for (int j = 0; j < m; ++j)
                    if (!s.mineByConfig[configs[0]][j]) {
                        result.moves[0].x = common.candidates[j].x;
                        result.moves[0].y = common.candidates[j].y;
                        break;
                    }
            return 1;
        }
        saveFail(key, (std::max)(best, upper), n, table);
        return -(std::max)(best, upper);
    }
}

} // namespace mss
