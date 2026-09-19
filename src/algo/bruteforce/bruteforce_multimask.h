#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdlib>
#include <deque>
#include <utility>
#include <vector>

#include "algo/bruteforce/bruteforce_common.h"

namespace mss {

// 定宽掩码后端把每个完整雷位方案压成候选格位图，最多覆盖 512 个候选格；
// 高位未使用部分必须始终保持为 0，这样 ~mask 才能安全表示“其余候选”。
template <std::size_t WordCount> struct bitMask {
    static_assert(WordCount > 0);
    static constexpr std::size_t kWordCount = WordCount;
    static constexpr int kBitCount = WordCount * 64;

    std::array<std::uint64_t, WordCount> words{};

    // 创建只包含一个指定候选位的掩码。
    static bitMask bit(int index) {
        // index 必须落在该 Mask 的可表示范围内；此函数刻意不做边界检查。
        bitMask result;
        result.words[index / 64] = std::uint64_t{1} << (index % 64);
        return result;
    }

    // 创建低 bitCount 位为 1 的候选集合掩码。
    static bitMask all(int bitCount) {
        // bitCount <= kBitCount；最后一个有效 word 之外的位保持为 0。
        bitMask result;
        int remaining = bitCount;
        for (std::uint64_t &word : result.words) {
            if (remaining >= 64) {
                word = ~std::uint64_t{};
                remaining -= 64;
            } else if (remaining > 0) {
                word = (std::uint64_t{1} << remaining) - 1;
                break;
            } else {
                break;
            }
        }
        return result;
    }

    // 判断掩码中是否至少有一个有效位。
    bool any() const {
        for (std::uint64_t word : words)
            if (word != 0)
                return true;
        return false;
    }

    // 返回最低位的置位下标；调用方保证掩码非空。
    int firstSetBit() const {
        int index = 0;
        for (std::uint64_t word : words) {
            if (word != 0)
                return index * 64 + std::countr_zero(word);
            ++index;
        }
        std::exit(1);
    }

    // 查询指定候选位是否置位。
    bool test(int index) const {
        return (words[index / 64] >> (index % 64)) & 1;
    }

    // 将指定候选位设置为 1。
    void set(int index) {
        words[index / 64] |= std::uint64_t{1} << (index % 64);
    }

    // 将指定候选位清为 0。
    void reset(int index) {
        words[index / 64] &= ~(std::uint64_t{1} << (index % 64));
    }

    // 统计掩码中置位候选的数量。
    int popcount() const {
        int result = 0;
        for (std::uint64_t word : words)
            result += std::popcount(word);
        return result;
    }

    // 保留两个候选集合的交集。
    bitMask &operator&=(const bitMask &other) {
        for (int i = 0; i < (int)(WordCount); ++i)
            words[i] &= other.words[i];
        return *this;
    }

    // 将另一个候选集合并入当前集合。
    bitMask &operator|=(const bitMask &other) {
        for (int i = 0; i < (int)(WordCount); ++i)
            words[i] |= other.words[i];
        return *this;
    }

    // 按 word 取反，得到候选全集的补集掩码。
    bitMask operator~() const {
        bitMask result;
        for (int i = 0; i < (int)(WordCount); ++i)
            result.words[i] = ~words[i];
        return result;
    }

    template <typename Callback>
    // 按下标递增顺序访问所有置位候选。
    void forEachSetBit(Callback &&callback) const {
        int index = 0;
        for (std::uint64_t word : words) {
            while (word != 0) {
                callback(index * 64 + std::countr_zero(word));
                word &= word - 1;
            }
            ++index;
        }
    }
};

template <> struct bitMask<1> {
    static constexpr std::size_t kWordCount = 1;
    static constexpr int kBitCount = 64;

    std::uint64_t word = 0;

    // 创建只包含一个指定候选位的单 word 掩码。
    static bitMask bit(int index) {
        // 设计目的：此特化直接使用单个 64 位 word；index 的范围由调用方的候选格契约保证。
        bitMask result;
        result.word = std::uint64_t{1} << index;
        return result;
    }

    // 创建低 bitCount 位为 1 的单 word 掩码。
    static bitMask all(int bitCount) {
        bitMask result;
        if (bitCount == 64)
            result.word = ~std::uint64_t{};
        else if (bitCount != 0)
            result.word = (std::uint64_t{1} << bitCount) - 1;
        return result;
    }

    // 判断单 word 掩码中是否至少有一个置位候选。
    bool any() const {
        return word != 0;
    }

    // 返回最低置位候选的下标；调用方保证掩码非空。
    int firstSetBit() const {
        if (word != 0)
            return std::countr_zero(word);
        std::exit(1);
    }

    // 查询单 word 中指定候选位是否置位。
    bool test(int index) const {
        return (word >> index) & 1;
    }

    // 将单 word 中指定候选位置为 1。
    void set(int index) {
        word |= std::uint64_t{1} << index;
    }

    // 将单 word 中指定候选位清为 0。
    void reset(int index) {
        word &= ~(std::uint64_t{1} << index);
    }

    // 统计单 word 掩码中的置位候选数量。
    int popcount() const {
        return std::popcount(word);
    }

    // 保留两个单 word 候选集合的交集。
    bitMask &operator&=(const bitMask &other) {
        word &= other.word;
        return *this;
    }

    // 将另一个单 word 候选集合并入当前集合。
    bitMask &operator|=(const bitMask &other) {
        word |= other.word;
        return *this;
    }

    // 返回单 word 候选集合的补集。
    bitMask operator~() const {
        bitMask result;
        result.word = ~word;
        return result;
    }

    template <typename Callback>
    // 按下标递增顺序访问单 word 中所有置位候选。
    void forEachSetBit(Callback &&callback) const {
        std::uint64_t bits = word;
        while (bits != 0) {
            callback(std::countr_zero(bits));
            bits &= bits - 1;
        }
    }
};

using u64 = bitMask<1>;
using u128 = bitMask<2>;
using u256 = bitMask<4>;
using u512 = bitMask<8>;

template <typename Mask> struct BruteForce::MultiMaskSolver {
    using ConfigId = std::uint32_t;
    using Common = BruteForce::CommonSession;
    using Session = BruteForce::MultiMaskSession<Mask>;
    using Result = BruteForce::Result;
    inline static constexpr int kJavaLiteConfigThreshold = 10000;

    using Layer = workspace::BruteForceMultiMask::Layer<Mask>;
    using Scratch = workspace::BruteForceMultiMask::Scratch<Mask>;

    // 把完整方案表转换成多 word 雷掩码和揭示数字表。
    static Session buildSession(const Common &common);
    // 用 Java-lite 启发式给同死亡数候选排序。
    static int javaLiteScore(const Common &common, const Session &session, std::span<const ConfigId> configs, int candidate);
    // 统一生成当前 unopened 候选的搜索顺序；root 也必须使用它来预热共享缓存。
    static std::vector<int> &orderCandidates(const Common &common, const Session &session, std::span<const ConfigId> configs, int depth);
    // 按同时揭示的安全格向量对方案分组。
    template <bool Small>
    static void groupSafeConfigs(const Session &session, std::span<ConfigId> configs, const Mask &safeMask, Layer &buf);

    template <bool CheckAllMoves, bool IsRoot>
    // 递归搜索当前方案集合，按 need 返回可保证的胜局数或失败上界。
    static int solve(const Common &common, Session &s, std::span<ConfigId> configs, int need, int depth,
                     FlatHashTable<U128, int, U128Hash> &table, Result &result);

    // 在多掩码状态上执行完整残局搜索并生成结果。
    static Result solve(const Common &common, Session &session, const BruteForce::Config &config);
};

//==============================================================================

template <typename Mask>
inline BruteForce::MultiMaskSession<Mask> BruteForce::MultiMaskSolver<Mask>::buildSession(const BruteForce::CommonSession &common) {
    // 把 CommonSession 的 CSR 雷位列表展开为 Mask、按候选字节打包的雷表，
    // 并预计算每个方案点击每格后的数字；雷位置在 revealByConfig 中记为 9。
    Session session;
    session.mineMaskByConfig.resize(common.possibilityCount);
    session.mineByteWordsByConfig.resize(common.possibilityCount, (common.candidateCount + 7) / 8, 0);
    for (int config = 0; config < common.possibilityCount; ++config) {
        for (int i = 0; i < common.minesPerConfig; ++i) {
            const CandidateId candidate = common.mineCandidateIds[config][i];
            session.mineMaskByConfig[config].set(candidate);
            session.mineByteWordsByConfig[config][candidate / 8] |= std::uint64_t{1} << (8 * (candidate % 8));
        }
    }
    session.revealByConfig.resize(common.possibilityCount, common.candidateCount, 0);
    for (int config = 0; config < common.possibilityCount; ++config)
        for (int candidate = 0; candidate < common.candidateCount; ++candidate) {
            int value = common.candidates[candidate].fixedMines;
            const CommonSession::Candidate &current = common.candidates[candidate];
            for (std::uint32_t i = 0; i < current.linksCount; ++i)
                value += session.mineMaskByConfig[config].test(common.links[current.linksOffset + i]);
            session.revealByConfig[config][candidate] = session.mineMaskByConfig[config].test(candidate) ? 9 : value;
        }
    return session;
}

// 返回 Java-lite 的评分分子；实际分母恒为 5 * configs.size()。
// 只统计点击 candidate 后仍可点击的候选，不包含 50/50、热点和进度项。
template <typename Mask>
inline int BruteForce::MultiMaskSolver<Mask>::javaLiteScore(const BruteForce::CommonSession &common,
                                                            const BruteForce::MultiMaskSession<Mask> &session,
                                                            std::span<const ConfigId> configs, int candidate) {
    // 在死亡数相同的候选中，按“点击后各数字分支还能暴露多少低风险格”排序；
    // 这只改变搜索顺序，不改变 solve 的返回值协议。
    std::array<std::array<int, Mask::kBitCount>, 9> &mineCounts = workspace::BruteForceMultiMask::scratch<Mask>.javaLiteMineCounts;
    std::array<int, 9> &groupSizes = workspace::BruteForceMultiMask::scratch<Mask>.javaLiteGroupSizes;
    for (std::array<int, Mask::kBitCount> &counts : mineCounts)
        counts.fill(0);
    groupSizes.fill(0);
    Mask remaining = session.unopenedCandidates;
    remaining.reset(candidate);
    for (ConfigId config : configs) {
        const int reveal = session.revealByConfig[config][candidate];
        if (reveal == 9)
            continue;
        const Mask &mines = session.mineMaskByConfig[config];
        ++groupSizes[reveal];
        Mask nextMines = mines;
        nextMines &= remaining;
        nextMines.forEachSetBit([&](int next) {
            ++mineCounts[reveal][next];
        });
    }

    int score = 0;
    for (int reveal = 0; reveal < 9; ++reveal) {
        const int groupSize = groupSizes[reveal];
        int bestMine = groupSize;
        int secondMine = groupSize;
        for (int next = 0; next < common.candidateCount; ++next) {
            if (!remaining.test(next))
                continue;
            const int mines = mineCounts[reveal][next];
            if (mines < bestMine) {
                secondMine = bestMine;
                bestMine = mines;
            } else if (mines < secondMine) {
                secondMine = mines;
            }
        }
        score += 4 * (groupSize - bestMine) + (groupSize - secondMine);
    }
    return score;
}

template <typename Mask>
inline std::vector<int> &BruteForce::MultiMaskSolver<Mask>::orderCandidates(const BruteForce::CommonSession &common,
                                                                            const BruteForce::MultiMaskSession<Mask> &session,
                                                                            std::span<const ConfigId> configs, int depth) {
    Layer &buf = workspace::BruteForceMultiMask::scratch<Mask>.layer(depth);
    std::vector<int> &deaths = buf.deaths;
    deaths.assign(common.candidateCount, 0);
    {
        // 使用预处理与 SIMD 加速代码：
        // for (ConfigId config : configs) {
        //     for (int i = 0; i < common.minesPerConfig; ++i)
        //         ++deaths[common.mineCandidateIds[config][i]];
        // }
        const int wordCount = session.mineByteWordsByConfig.cols();
        std::array<std::uint64_t, Mask::kBitCount / 8> sums{};
        for (int begin = 0; begin < (int)configs.size(); begin += 255) {
            sums.fill(0);
            const int end = (std::min)(begin + 255, (int)configs.size());
            for (int i = begin; i < end; ++i) {
                const std::uint64_t *row = session.mineByteWordsByConfig[configs[i]];
                for (int word = 0; word < wordCount; ++word)
                    sums[word] += row[word];
            }
            for (int word = 0; word < wordCount; ++word) {
                std::uint64_t value = sums[word];
                const int endCandidate = (std::min)(word * 8 + 8, common.candidateCount);
                for (int candidate = word * 8; candidate < endCandidate; ++candidate) {
                    deaths[candidate] += value & 255;
                    value >>= 8;
                }
            }
        }
    }
    std::vector<int> &order = buf.order;
    order.clear();
    session.unopenedCandidates.forEachSetBit([&](int candidate) {
        order.push_back(candidate);
    });
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        if (deaths[a] != deaths[b])
            return deaths[a] < deaths[b];
        return a < b;
    });
    // 首轮竞争只在最低 death 的候选之间使用 Java-lite；其余 death 层仍按
    // death 递增，避免为不会先被尝试的候选支付额外评分成本。
    if (configs.size() >= kJavaLiteConfigThreshold && order.size() > 1) {
        std::size_t tieEnd = 1;
        while (tieEnd < order.size() && deaths[order[tieEnd]] == deaths[order[0]])
            ++tieEnd;
        if (tieEnd > 1) {
            std::array<int, Mask::kBitCount> javaLiteScores{};
            for (int i = 0; i < (int)(tieEnd); ++i)
                javaLiteScores[order[i]] = javaLiteScore(common, session, configs, order[i]);
            std::sort(order.begin(), order.begin() + tieEnd, [&](int a, int b) {
                if (javaLiteScores[a] != javaLiteScores[b])
                    return javaLiteScores[a] > javaLiteScores[b];
                return a < b;
            });
        }
    }
    return order;
}

template <typename Mask>
template <bool Small>
inline void BruteForce::MultiMaskSolver<Mask>::groupSafeConfigs(const BruteForce::MultiMaskSession<Mask> &session,
                                                                std::span<ConfigId> configs, const Mask &safeMask, Layer &buf) {
    std::vector<std::span<ConfigId>> &groupList = buf.safeGroupList;
    groupList.clear();
    std::vector<int> &groupOffsets = buf.safeGroupOffsets;
    std::vector<ConfigId> &groupedConfigs = buf.safeGroupedConfigs;
    std::array<int, Small ? 3 : Mask::kBitCount> safeCandidates;
    int safeCount = 0;
    safeMask.forEachSetBit([&](int candidate) {
        safeCandidates[safeCount++] = candidate;
    });
    if constexpr (Small) {
        static thread_local std::array<std::vector<ConfigId>, 729> groups;
        static thread_local std::vector<int> keys;
        for (ConfigId config : configs) {
            int key = 0;
            int factor = 1;
            for (int i = 0; i < safeCount; ++i) {
                key += session.revealByConfig[config][safeCandidates[i]] * factor;
                factor *= 9;
            }
            if (groups[key].empty())
                keys.push_back(key);
            groups[key].push_back(config);
        }
        groupOffsets.resize(keys.size() + 1);
        groupOffsets[0] = 0;
        groupedConfigs.resize(configs.size());
        for (int i = 0; i < (int)(keys.size()); ++i) {
            std::vector<ConfigId> &group = groups[keys[i]];
            groupOffsets[i + 1] = groupOffsets[i] + group.size();
            std::copy(group.begin(), group.end(), groupedConfigs.begin() + groupOffsets[i]);
            groupList.emplace_back(groupedConfigs.data() + groupOffsets[i], group.size());
        }
        for (int key : keys)
            groups[key].clear();
        keys.clear();
    } else {
        FlatHashTable<U128, int, U128Hash> &groupTable = workspace::BruteForceMultiMask::scratch<Mask>.safeGroupTable;
        std::vector<int> &groupIds = buf.safeGroupIds;
        std::vector<int> &groupSizes = buf.safeGroupSizes;
        groupTable.clear();
        groupTable.reserve(configs.size());
        groupIds.resize(configs.size());
        groupSizes.clear();
        const int full = safeCount / 16 * 16;
        for (int i = 0; i < (int)(configs.size()); ++i) {
            const auto *row = session.revealByConfig[configs[i]];
            U128Hasher hasher;
            for (int j = 0; j < full; j += 16) {
                std::uint64_t packed = 0;
                for (int k = 0; k < 16; ++k)
                    packed |= (std::uint64_t)row[safeCandidates[j + k]] << (4 * k);
                hasher.mix(packed);
            }
            if (full < safeCount) {
                std::uint64_t packed = 0;
                for (int j = full; j < safeCount; ++j)
                    packed |= (std::uint64_t)row[safeCandidates[j]] << (4 * (j - full));
                hasher.mix(packed);
            }
            int &slot = groupTable[hasher.finalize()];
            if (slot == 0) {
                slot = groupSizes.size() + 1;
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
    }
}

template <typename Mask>
template <bool CheckAllMoves, bool IsRoot>
inline int BruteForce::MultiMaskSolver<Mask>::solve(const BruteForce::CommonSession &common, BruteForce::MultiMaskSession<Mask> &s,
                                                    std::span<ConfigId> configs, int need, int depth,
                                                    FlatHashTable<U128, int, U128Hash> &table, BruteForce::Result &result) {
    // 与普通后端相同，正数是当前方案集合的精确可赢数，负数是未达到 need
    // 时的可赢上界编码；Mask 只改变雷位/安全集合的表示，不改变这个调用协议。
    // 根节点在 CheckAllMoves 模式下逐候选汇总各数字分支，否则只求一条推荐路径。
    if constexpr (CheckAllMoves && IsRoot) {
        ++s.nodes;
        const int n = configs.size();
        result.moves.clear();
        // 方案总数不足 need，-n 表示这是本节点可达到的最大上界。
        if (need > n)
            return -n;
        Layer &buf = workspace::BruteForceMultiMask::scratch<Mask>.layer(depth);
        result.moves.resize(common.candidateCount);
        int best = 0;
        std::array<std::vector<ConfigId>, 9> &groups = buf.groups;
        std::vector<int> &order = orderCandidates(common, s, configs, depth);
        for (int candidate : order) {
            for (std::vector<ConfigId> &group : groups)
                group.clear();
            for (ConfigId config : configs) {
                const int reveal = s.revealByConfig[config][candidate];
                if (reveal == 9)
                    continue;
                groups[reveal].push_back(config);
            }
            s.unopenedCandidates.reset(candidate);
            int wins = 0;
            for (int reveal = 0; reveal < 9; ++reveal)
                if (!groups[reveal].empty()) {
                    const int value = solve<false, false>(common, s, groups[reveal], 1, depth + 1, table, result);
                    if (value > 0)
                        wins += value;
                }
            s.unopenedCandidates.set(candidate);
            result.moves[candidate] = {common.candidates[candidate].x, common.candidates[candidate].y, wins};
            best = (std::max)(best, wins);
        }
        return best;
    } else {
        ++s.nodes;
        const int n = configs.size();
        if (n <= 1) {
            // 单方案节点至多贡献一个胜利方案；不足 need 时返回负上界。
            if (need > n)
                return -n;
            if constexpr (IsRoot)
                if (n == 1)
                    for (int candidate = 0; candidate < common.candidateCount; ++candidate)
                        if (s.revealByConfig[configs[0]][candidate] != 9) {
                            result.moves[0].x = common.candidates[candidate].x;
                            result.moves[0].y = common.candidates[candidate].y;
                            break;
                        }
            return n;
        }
        // 当前方案集合规模不足以满足目标阈值。
        if (need > n)
            return -n;
        const U128 key = BruteForce::hashConfigs(configs);
        // 正缓存值可直接回答阈值查询；负缓存值只有在绝对值仍低于 need 时
        // 才能直接证明失败，否则该缓存只对更低阈值有效。
        if (const int *cached = table.find(key)) {
            if (*cached >= 0)
                return *cached >= need ? *cached : -*cached;
            if (-*cached < need)
                return *cached;
        }
        Layer &buf = workspace::BruteForceMultiMask::scratch<Mask>.layer(depth);
        const int m = common.candidateCount;
        std::vector<int> &deaths = buf.deaths;
        Mask safeMask = s.unopenedCandidates;
        for (ConfigId config : configs) {
            safeMask &= ~s.mineMaskByConfig[config];
            if (!safeMask.any())
                break;
        }
        if (safeMask.any()) {
            // safeMask 是所有 configs 的交集安全格；将其整体消去后按整组揭示向量
            // 聚类，复用“同时打开安全格”的递归子问题。
            if constexpr (IsRoot) {
                const ConfigId candidate = safeMask.firstSetBit();
                result.moves[0].x = common.candidates[candidate].x;
                result.moves[0].y = common.candidates[candidate].y;
            }
            s.unopenedCandidates &= ~safeMask;
            if (safeMask.popcount() <= 3)
                groupSafeConfigs<true>(s, configs, safeMask, buf);
            else
                groupSafeConfigs<false>(s, configs, safeMask, buf);
            std::vector<std::span<ConfigId>> &groupList = buf.safeGroupList;
            std::sort(groupList.begin(), groupList.end(), [](auto a, auto b) {
                if (a.size() != b.size())
                    return a.size() > b.size();
                return a.data() < b.data();
            });
            int wins = 0;
            bool bailed = false;
            int upper = 0;
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
            s.unopenedCandidates |= safeMask;
            if (bailed) {
                // 仍无法达到 need；upper 包含已累计结果和未展开分支的最大贡献。
                BruteForce::saveFail(key, upper, n, table);
                return -upper;
            }
            table[key] = wins;
            return wins;
        }

        std::vector<int> &order = orderCandidates(common, s, configs, depth);
        int best = 0;
        int upper = 0;
        for (int candidate : order) {
            const int target = (std::max)(best + 1, need);
            if (n - deaths[candidate] < target) {
                upper = (std::max)(upper, n - deaths[candidate]);
                break;
            }
            std::array<std::vector<ConfigId>, 9> &groups = buf.groups;
            for (std::vector<ConfigId> &group : groups)
                group.clear();
            int groupCount = 0;
            for (ConfigId config : configs) {
                const int reveal = s.revealByConfig[config][candidate];
                if (reveal == 9)
                    continue;
                if (groups[reveal].empty())
                    ++groupCount;
                groups[reveal].push_back(config);
            }
            if (groupCount <= 1) {
                // 不分裂候选不会产生新的信息分支；不更新 upper，供末尾识别所有
                // 候选都不分裂的精确终局。
                continue;
            }
            std::vector<std::pair<int, int>> &groupList = buf.groupList;
            groupList.clear();
            for (int reveal = 0; reveal < 9; ++reveal)
                if (!groups[reveal].empty())
                    groupList.push_back({reveal, groups[reveal].size()});
            std::sort(groupList.begin(), groupList.end(), [](const auto &a, const auto &b) {
                if (a.second != b.second)
                    return a.second > b.second;
                return a.first < b.first;
            });
            s.unopenedCandidates.reset(candidate);
            int wins = 0;
            bool bailed = false;
            int moveUpper = 0;
            int remaining = n - deaths[candidate];
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
            s.unopenedCandidates.set(candidate);
            if (bailed)
                upper = (std::max)(upper, moveUpper);
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
        // upper == 0 说明没有可分裂候选进入失败路径；所有候选行为等价，
        // 因而该状态的精确可赢数是 1，而不是依赖本次 need 的失败上界。
        if (best == 0 && upper == 0) {
            // 这里是精确结果，可供任意 need 直接复用。
            table[key] = 1;
            if constexpr (IsRoot)
                    for (int candidate = 0; candidate < m; ++candidate)
                    if (s.revealByConfig[configs[0]][candidate] != 9) {
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

template <typename Mask>
inline BruteForce::Result BruteForce::MultiMaskSolver<Mask>::solve(const BruteForce::CommonSession &common,
                                                                   BruteForce::MultiMaskSession<Mask> &session,
                                                                   const BruteForce::Config &config) {
    // 初始化全候选 unopened 集合并建立根方案 span；外层负责把带符号递归结果
    // 翻译成公开的 moves，失败的负上界不会泄漏为公开的 wins。
    BruteForce::Result result;
    result.possibilities = common.possibilityCount;
    session.unopenedCandidates = Mask::all(common.candidateCount);
    workspace::BruteForceMultiMask::scratch<Mask>.reset();
    workspace::BruteForceMultiMask::cache<Mask>.clear();
    std::vector<ConfigId> configs(common.possibilityCount);
    for (int i = 0; i < (int)(configs.size()); ++i)
        configs[i] = i;
    if (config.checkAllMoves) {
        // 该模式直接暴露根节点各候选的可赢数；递归负值只是阈值失败上界，
        // 不能写进公开的 Move::wins。
        solve<true, true>(common, session, configs, 1, 0, workspace::BruteForceMultiMask::cache<Mask>, result);
    } else {
        result.moves.resize(1);
        // 这里把 minWins 交给递归做阈值剪枝；只有正返回值才形成推荐步，
        // 负值说明最多只能赢 abs(value) 局，因此清空公开动作结果。
        const int wins = solve<false, true>(common, session, configs, config.minWins, 0, workspace::BruteForceMultiMask::cache<Mask>, result);
        if (wins >= config.minWins)
            result.moves[0].wins = wins;
        else
            result.moves.clear();
    }
    result.nodes = session.nodes;
    return result;
}

} // namespace mss
