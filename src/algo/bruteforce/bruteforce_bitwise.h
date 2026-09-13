#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <deque>
#include <utility>
#include <vector>

#include "algo/bruteforce/bruteforce_common.h"

namespace mss {

struct BruteForce::BitwiseSolver {
    using ConfigId = std::uint32_t;

    struct Layer {
        std::vector<int> deaths;
        std::vector<int> order;
        std::vector<int> suffix;
        std::array<std::vector<ConfigId>, 9> groups;
        std::vector<std::pair<int, int>> groupList;
        std::vector<U128> safeHashes;
        std::vector<int> safeGroupIds;
        std::vector<int> safeGroupSizes;
        std::vector<int> safeGroupOffsets;
        std::vector<ConfigId> safeGroupedConfigs;
        std::vector<std::span<ConfigId>> safeGroupList;
    };

    struct Scratch {
        Layer& layer(int depth);
        void reset();

        std::deque<Layer> layers;
        FlatHashTable<U128, int, U128Hash> safeGroupTable;
    };

    inline static thread_local Scratch scratch;
    inline static thread_local FlatHashTable<U128, int, U128Hash> cache;

    static int revealAt(const BruteForce::CommonSession& common,
                        const BruteForce::BitwiseSession& session,
                        ConfigId config, ConfigId candidate);
    static bool mineAt(const BruteForce::BitwiseSession& session,
                       ConfigId config, ConfigId candidate);

    template <bool CheckAllMoves, bool IsRoot>
    static int solve(const BruteForce::CommonSession& common,
                     BruteForce::BitwiseSession& s,
                     std::span<ConfigId> configs, int need, int depth,
                     FlatHashTable<U128, int, U128Hash>& table,
                     BruteForce::Result& result);

    static BruteForce::Result solve(const BruteForce::CommonSession& common,
                                    BruteForce::BitwiseSession& session,
                                    const BruteForce::Config& config);
};

//==============================================================================

inline BruteForce::BitwiseSession BruteForce::buildBitwiseSession(
    const CommonSession& common) {
    BitwiseSession session;
    session.mineMasks.resize(common.possibilityCount, 0);
    for (ConfigId config = 0;
         config < static_cast<ConfigId>(common.possibilityCount); ++config)
        for (std::uint32_t i = common.mineOffsets[config];
             i < common.mineOffsets[config + 1]; ++i)
            session.mineMasks[config] |= 1ULL << common.mineCells[i];
    session.reveal.assign(static_cast<std::size_t>(common.possibilityCount) *
                              common.candidateCount, 0);
    for (ConfigId config = 0;
         config < static_cast<ConfigId>(common.possibilityCount); ++config)
        for (CandidateId candidate = 0;
             candidate < static_cast<CandidateId>(common.candidateCount); ++candidate) {
            int value = common.candidates[candidate].fixedMines;
            const CommonSession::Candidate& current = common.candidates[candidate];
            for (std::uint32_t i = 0; i < current.linksCount; ++i)
                value += (session.mineMasks[config] >>
                          common.links[current.linksOffset + i]) & 1;
            session.reveal[static_cast<std::size_t>(config) *
                           common.candidateCount + candidate] = value;
        }
    return session;
}

inline BruteForce::BitwiseSolver::Layer&
BruteForce::BitwiseSolver::Scratch::layer(int depth) {
    if (static_cast<int>(layers.size()) <= depth) layers.emplace_back();
    return layers[depth];
}

inline void BruteForce::BitwiseSolver::Scratch::reset() {
    for (Layer& layer : layers) {
        layer.deaths.clear();
        layer.order.clear();
        layer.suffix.clear();
        for (std::vector<ConfigId>& group : layer.groups) group.clear();
        layer.groupList.clear();
        layer.safeHashes.clear();
        layer.safeGroupIds.clear();
        layer.safeGroupSizes.clear();
        layer.safeGroupOffsets.clear();
        layer.safeGroupedConfigs.clear();
        layer.safeGroupList.clear();
    }
    safeGroupTable.clear();
}

inline int BruteForce::BitwiseSolver::revealAt(
    const BruteForce::CommonSession& common,
    const BruteForce::BitwiseSession& session,
    ConfigId config, ConfigId candidate) {
    return session.reveal[static_cast<std::size_t>(config) *
                              common.candidateCount + candidate];
}

inline bool BruteForce::BitwiseSolver::mineAt(
    const BruteForce::BitwiseSession& session,
    ConfigId config, ConfigId candidate) {
    return (session.mineMasks[config] >> candidate) & 1;
}

template <bool CheckAllMoves, bool IsRoot>
inline int BruteForce::BitwiseSolver::solve(
    const BruteForce::CommonSession& common,
    BruteForce::BitwiseSession& s,
    std::span<ConfigId> configs, int need, int depth,
    FlatHashTable<U128, int, U128Hash>& table,
    BruteForce::Result& result) {
    if constexpr (CheckAllMoves && IsRoot) {
        ++s.nodes;
        const int n = static_cast<int>(configs.size());
        result.moves.clear();
        if (need > n) return -n;
        Layer& buf = scratch.layer(depth);
        int best = 0;
        std::array<std::vector<ConfigId>, 9>& groups = buf.groups;
        for (std::uint64_t unopened = s.unopened; unopened != 0;
             unopened &= unopened - 1) {
            const ConfigId candidate = std::countr_zero(unopened);
            for (std::vector<ConfigId>& group : groups) group.clear();
            for (ConfigId config : configs)
                if (!mineAt(s, config, candidate))
                    groups[revealAt(common, s, config, candidate)].push_back(config);
            s.unopened &= ~(1ULL << candidate);
            int wins = 0;
            for (int reveal = 0; reveal < 9; ++reveal)
                if (!groups[reveal].empty()) {
                    const int value = solve<false, false>(
                        common, s, groups[reveal], 1, depth + 1, table, result);
                    if (value > 0) wins += value;
                }
            s.unopened |= 1ULL << candidate;
            result.moves.push_back({common.candidates[candidate].x,
                                    common.candidates[candidate].y, wins});
            best = (std::max)(best, wins);
        }
        return best;
    } else {
        ++s.nodes;
        const int n = static_cast<int>(configs.size());
        if (n <= 1) {
            if (need > n) return -n;
            if constexpr (IsRoot) if (n == 1)
                for (int candidate = 0; candidate < common.candidateCount; ++candidate)
                    if (!mineAt(s, configs[0], candidate)) {
                        result.moves[0].x = common.candidates[candidate].x;
                        result.moves[0].y = common.candidates[candidate].y;
                        break;
                    }
            return n;
        }
        if (need > n) return -n;
        const U128 key = BruteForce::hashConfigs(configs);
        if (const int* cached = table.find(key)) {
            if (*cached >= 0) return *cached >= need ? *cached : -*cached;
            if (-*cached < need) return *cached;
        }
        Layer& buf = scratch.layer(depth);
        const int m = common.candidateCount;
        std::vector<int>& deaths = buf.deaths;
        std::uint64_t safeMask = s.unopened;
        for (ConfigId config : configs) {
            safeMask &= ~s.mineMasks[config];
            if (safeMask == 0) break;
        }
        if (safeMask != 0) {
            if constexpr (IsRoot) {
                const ConfigId candidate = std::countr_zero(safeMask);
                result.moves[0].x = common.candidates[candidate].x;
                result.moves[0].y = common.candidates[candidate].y;
            }
            s.unopened &= ~safeMask;
            std::vector<U128>& hashes = buf.safeHashes;
            hashes.clear();
            const std::size_t wordCount =
                (std::popcount(safeMask) + 15) / 16;
            for (ConfigId config : configs) {
                std::array<std::uint64_t, 4> packed{};
                int word = 0;
                int shift = 0;
                std::uint64_t safe = safeMask;
                while (safe != 0) {
                    const ConfigId candidate = std::countr_zero(safe);
                    const std::uint64_t value = static_cast<std::uint64_t>(
                        revealAt(common, s, config, candidate));
                    packed[word] |= value << shift;
                    shift += 4;
                    if (shift == 64) {
                        ++word;
                        shift = 0;
                    }
                    safe &= safe - 1;
                }
                U128Hasher hasher;
                for (std::size_t i = 0; i < wordCount; ++i)
                    hasher.mix(packed[i]);
                hashes.push_back(hasher.finalize());
            }
            std::vector<std::span<ConfigId>>& groupList = buf.safeGroupList;
            groupList.clear();
            FlatHashTable<U128, int, U128Hash>& groupTable =
                scratch.safeGroupTable;
            std::vector<int>& groupIds = buf.safeGroupIds;
            std::vector<int>& groupSizes = buf.safeGroupSizes;
            std::vector<int>& groupOffsets = buf.safeGroupOffsets;
            std::vector<ConfigId>& groupedConfigs = buf.safeGroupedConfigs;
            groupTable.clear();
            groupTable.reserve(hashes.size());
            groupIds.resize(hashes.size());
            groupSizes.clear();
            for (std::size_t i = 0; i < hashes.size(); ++i) {
                int& slot = groupTable[hashes[i]];
                if (slot == 0) {
                    slot = static_cast<int>(groupSizes.size()) + 1;
                    groupSizes.push_back(0);
                }
                groupIds[i] = slot - 1;
                ++groupSizes[groupIds[i]];
            }
            groupOffsets.resize(groupSizes.size() + 1);
            groupOffsets[0] = 0;
            for (std::size_t i = 0; i < groupSizes.size(); ++i)
                groupOffsets[i + 1] = groupOffsets[i] + groupSizes[i];
            groupedConfigs.resize(configs.size());
            for (std::size_t i = 0; i < groupSizes.size(); ++i)
                groupSizes[i] = groupOffsets[i];
            for (std::size_t i = 0; i < configs.size(); ++i)
                groupedConfigs[groupSizes[groupIds[i]]++] = configs[i];
            for (std::size_t i = 0; i < groupSizes.size(); ++i)
                groupList.emplace_back(groupedConfigs.data() + groupOffsets[i],
                                       groupOffsets[i + 1] - groupOffsets[i]);
            std::sort(groupList.begin(), groupList.end(),
                      [](auto a, auto b) {
                          if (a.size() != b.size()) return a.size() > b.size();
                          return a.data() < b.data();
                      });
            std::vector<int>& suffix = buf.suffix;
            suffix.resize(groupList.size() + 1);
            suffix.back() = 0;
            for (int i = static_cast<int>(groupList.size()) - 1; i >= 0; --i)
                suffix[i] = suffix[i + 1] + static_cast<int>(groupList[i].size());
            int wins = 0;
            bool bailed = false;
            int upper = 0;
            for (int i = 0; i < static_cast<int>(groupList.size()); ++i) {
                const int size = static_cast<int>(groupList[i].size());
                if (wins + size + suffix[i + 1] < need) {
                    upper = wins + size + suffix[i + 1];
                    bailed = true;
                    break;
                }
                const int value = solve<false, false>(
                    common, s, groupList[i],
                    (std::max)(1, need - wins - suffix[i + 1]),
                    depth + 1, table, result);
                if (value <= 0) {
                    upper = wins - value + suffix[i + 1];
                    bailed = true;
                    break;
                }
                wins += value;
            }
            s.unopened |= safeMask;
            if (bailed) {
                BruteForce::saveFail(key, upper, n, table);
                return -upper;
            }
            table[key] = wins;
            return wins;
        }

        deaths.assign(m, 0);
        for (ConfigId config : configs)
            for (std::uint32_t i = common.mineOffsets[config];
                 i < common.mineOffsets[config + 1]; ++i)
                ++deaths[common.mineCells[i]];
        std::vector<int>& order = buf.order;
        order.clear();
        for (std::uint64_t unopened = s.unopened; unopened != 0;
             unopened &= unopened - 1)
            order.push_back(std::countr_zero(unopened));
        std::sort(order.begin(), order.end(),
                  [&](int a, int b) {
                      if (deaths[a] != deaths[b]) return deaths[a] < deaths[b];
                      return a < b;
                  });
        int best = 0;
        int upper = 0;
        for (int candidate : order) {
            const int target = (std::max)(best + 1, need);
            if (n - deaths[candidate] < target) {
                upper = (std::max)(upper, n - deaths[candidate]);
                break;
            }
            std::array<std::vector<ConfigId>, 9>& groups = buf.groups;
            for (std::vector<ConfigId>& group : groups) group.clear();
            int groupCount = 0;
            for (ConfigId config : configs) if (!mineAt(s, config, candidate)) {
                const int reveal = revealAt(common, s, config, candidate);
                if (groups[reveal].empty()) ++groupCount;
                groups[reveal].push_back(config);
            }
            if (groupCount <= 1) {
                // 不分裂的候选不产生失败上界；保持 upper 为 0，供末尾判断
                // “所有候选都不分裂”，此时返回值精确为 1。
                continue;
            }
            std::vector<std::pair<int, int>>& groupList = buf.groupList;
            groupList.clear();
            for (int reveal = 0; reveal < 9; ++reveal)
                if (!groups[reveal].empty())
                    groupList.push_back({reveal, static_cast<int>(groups[reveal].size())});
            std::sort(groupList.begin(), groupList.end(),
                      [](const auto& a, const auto& b) {
                          if (a.second != b.second) return a.second > b.second;
                          return a.first < b.first;
                      });
            std::vector<int>& suffix = buf.suffix;
            suffix.resize(groupList.size() + 1);
            suffix.back() = 0;
            for (int i = static_cast<int>(groupList.size()) - 1; i >= 0; --i)
                suffix[i] = suffix[i + 1] + groupList[i].second;
            s.unopened &= ~(1ULL << candidate);
            int wins = 0;
            bool bailed = false;
            int moveUpper = 0;
            for (int i = 0; i < static_cast<int>(groupList.size()); ++i) {
                std::vector<ConfigId>& group = groups[groupList[i].first];
                if (wins + groupList[i].second + suffix[i + 1] < target) {
                    moveUpper = wins + groupList[i].second + suffix[i + 1];
                    bailed = true;
                    break;
                }
                const int value = solve<false, false>(
                    common, s, group,
                    (std::max)(1, target - wins - suffix[i + 1]),
                    depth + 1, table, result);
                if (value <= 0) {
                    moveUpper = wins - value + suffix[i + 1];
                    bailed = true;
                    break;
                }
                wins += value;
            }
            s.unopened |= 1ULL << candidate;
            if (bailed) upper = (std::max)(upper, moveUpper);
            if (!bailed && wins > best) {
                best = wins;
                if constexpr (IsRoot) {
                    result.moves[0].x = common.candidates[candidate].x;
                    result.moves[0].y = common.candidates[candidate].y;
                }
            }
        }
        if (best >= need) {
            table[key] = best;
            return best;
        }
        // upper == 0 证明没有可分裂候选进入失败路径；need 只是阈值，
        // 不能用来判断这个终局。
        if (best == 0 && upper == 0) {
            // 这里是精确结果，可供任意 need 直接复用。
            table[key] = 1;
            if constexpr (IsRoot)
                for (int candidate = 0; candidate < m; ++candidate)
                    if (!mineAt(s, configs[0], candidate)) {
                        result.moves[0].x = common.candidates[candidate].x;
                        result.moves[0].y = common.candidates[candidate].y;
                        break;
                    }
            return 1;
        }
        BruteForce::saveFail(key, (std::max)(best, upper), n, table);
        return -(std::max)(best, upper);
    }
}

inline BruteForce::Result BruteForce::BitwiseSolver::solve(
    const BruteForce::CommonSession& common,
    BruteForce::BitwiseSession& session,
    const BruteForce::Config& config) {
    BruteForce::Result result;
    result.possibilities = common.possibilityCount;
    session.unopened = (1ULL << common.candidateCount) - 1;
    scratch.reset();
    cache.clear();
    std::vector<ConfigId> configs(common.possibilityCount);
    for (ConfigId i = 0;
         static_cast<int>(i) < common.possibilityCount; ++i)
        configs[i] = i;
    if (config.checkAllMoves) {
        solve<true, true>(common, session, configs, 1, 0, cache, result);
    } else {
        result.moves.resize(1);
        const int wins = solve<false, true>(
            common, session, configs, config.minWins, 0, cache, result);
        if (wins >= config.minWins) result.moves[0].wins = wins;
        else result.moves.clear();
    }
    result.nodes = session.nodes;
    return result;
}

}  // namespace mss
