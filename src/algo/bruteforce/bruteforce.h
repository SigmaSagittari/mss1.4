#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <span>
#include <utility>
#include <vector>

#include "algo/basic.h"
#include "algo/observed_board.h"
#include "algo/shape_solver/shape_solver.h"
#include "algo/structure.h"
#include "core/utility/dynamic_bitset.h"



namespace mss {

struct BruteForce {

public:

struct Config {
    bool checkAllMoves;
    int minWins;
};

struct Result {
    struct Move {
        int x = 0;
        int y = 0;
        int wins = 0;
    };

    int possibilities = 0;
    long long nodes = 0;
    std::vector<Move> moves;
};

// 在当前盘面可能性上进行残局搜索。
// Config 的所有字段必须由调用方显式指定；minWins 只影响非 checkAllMoves 模式。
    static Result solve(const ObservedBoard::Result& board,
                        const Basic::Result& basic,
                        const Structure::Result& structure,
                        const Structure::ShapePool& shapes,
                        const Config& config);

private:
    using ConfigId = std::uint32_t;
    using CandidateId = std::uint32_t;

    struct Session;
    struct ScratchBuffers;

    static thread_local ScratchBuffers scratch;
    static thread_local FlatHashTable<U128, int, U128Hash> cache;

    static int revealAt(const Session& session, ConfigId config,
                        CandidateId candidate);
    static bool mineAt(const Session& session, ConfigId config,
                       CandidateId candidate);
    static U128 hashConfigs(std::span<const ConfigId> configs);
    static void saveFail(const U128& key, int upper, int count,
                         FlatHashTable<U128, int, U128Hash>& table);
    static Session buildSession(
        const ObservedBoard::Result& board, const Basic::Result& basic,
        const Structure::Result& structure, const Structure::ShapePool& shapes);
    template <bool CheckAllMoves, bool IsRoot>
    static int solve(Session& s, std::span<ConfigId> configs, int need,
                     int depth, FlatHashTable<U128, int, U128Hash>& table,
                     Result& result);
};

}  // namespace mss

//==============================================================================
namespace mss {

struct BruteForce::Session {
    struct Candidate {
        int x = 0;
        int y = 0;
        std::uint32_t linksOffset = 0;
        std::uint8_t linksCount = 0;
        std::uint8_t fixedMines = 0;
    };
    int candidateCount = 0;
    int possibilityCount = 0;
    std::vector<Candidate> candidates;
    std::vector<int> links;
    std::vector<std::uint8_t> mine;
    std::vector<std::uint8_t> reveal;
    std::vector<std::uint32_t> mineOffsets;
    std::vector<CandidateId> mineCells;
    DynamicBitset unopened;
    long long nodes = 0;
    long long smallEntries = 0;
    long long smallNodes = 0;
    long long smallHashRequests = 0;
    long long smallHashedConfigs = 0;
    long long smallEntryConfigTotal = 0;
    long long smallEntryMaskWork = 0;
    long long smallCandidateEvals = 0;
    long long smallCandidateConfigChecks = 0;
    long long smallSafeCells = 0;
    long long smallSafeCellConfigChecks = 0;
    std::array<long long, 32> cacheQueries{};
    std::array<long long, 32> cacheHits{};
};

struct BruteForce::ScratchBuffers {
    struct Layer {
        std::vector<int> deaths;
        std::vector<int> safeCells;
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
    Layer& layer(int depth) {
        if (static_cast<int>(layers.size()) <= depth) layers.emplace_back();
        return layers[depth];
    }
    void reset() {
        for (Layer& l : layers) {
            l.deaths.clear();
            l.safeCells.clear();
            l.order.clear();
            l.suffix.clear();
            for (std::vector<ConfigId>& g : l.groups) g.clear();
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
    FlatHashTable<U128, int, U128Hash> safeGroupTable;
    std::deque<Layer> layers;
};

inline thread_local BruteForce::ScratchBuffers BruteForce::scratch;
inline thread_local FlatHashTable<U128, int, U128Hash> BruteForce::cache;

inline int BruteForce::revealAt(const Session& session, ConfigId config,
                                CandidateId candidate) {
    return session.reveal[static_cast<std::size_t>(config) *
                          session.candidateCount + candidate];
}

inline bool BruteForce::mineAt(const Session& session, ConfigId config,
                               CandidateId candidate) {
    return session.mine[static_cast<std::size_t>(config) *
                        session.candidateCount + candidate] != 0;
}

inline U128 BruteForce::hashConfigs(std::span<const ConfigId> configs) {
    U128 hash{configs.size(), configs.size()};
    for (ConfigId config : configs)
        hash += {splitmix64(config), splitmix64(config + 0x9e3779b97f4a7c15ULL)};
    return hash;
}

inline void BruteForce::saveFail(
    const U128& key, int upper, int count,
    FlatHashTable<U128, int, U128Hash>& table) {
    if (upper <= 0 || upper >= count) return;
    int* old = table.find(key);
    if (old == nullptr) {
        table[key] = -upper;
        return;
    }
    if (*old < 0) *old = (std::max)(*old, -upper);
}

inline BruteForce::Session BruteForce::buildSession(
    const ObservedBoard::Result& board, const Basic::Result& basic,
    const Structure::Result& structure, const Structure::ShapePool& shapes) {
    Session session;
    const int cellCount = (board.rows + 1) * (board.cols + 1);
    std::vector<int> candidateAt(cellCount, -1);
    for (int x = 1; x <= board.rows; ++x)
        for (int y = 1; y <= board.cols; ++y) {
            if (board.board[x][y] != ObservedBoard::CellState::Hidden) continue;
            const Basic::Mark mark = basic.marks[x][y];
            if (mark != Basic::Mark::H && mark != Basic::Mark::T) continue;
            candidateAt[board.id(x, y)] = session.candidateCount++;
            session.candidates.push_back({x, y, 0, 0, 0});
        }
    for (CandidateId candidate = 0;
         candidate < static_cast<CandidateId>(session.candidates.size()); ++candidate) {
        Session::Candidate& current = session.candidates[candidate];
        current.linksOffset = static_cast<std::uint32_t>(session.links.size());
        forEachAdjacent(current.x, current.y, board.rows, board.cols,
                        [&](int x, int y) {
            if (basic.marks[x][y] == Basic::Mark::F) ++current.fixedMines;
            const int linked = candidateAt[board.id(x, y)];
            if (linked >= 0) session.links.push_back(linked);
        });
        current.linksCount = static_cast<std::uint8_t>(
            session.links.size() - current.linksOffset);
    }
    std::vector<CandidateId> tCells;
    for (CandidateId candidate = 0;
         candidate < static_cast<CandidateId>(session.candidates.size()); ++candidate)
        if (basic.marks[session.candidates[candidate].x]
                        [session.candidates[candidate].y] == Basic::Mark::T)
            tCells.push_back(candidate);

    session.mineOffsets.push_back(0);
    std::vector<CandidateId> placed;
    const int mines = board.totalMines - basic.mineSum;
    const int componentCount = static_cast<int>(structure.components.size());
    std::vector<std::uint32_t> assignmentOffsets(componentCount + 1);
    std::vector<std::uint32_t> assignmentCounts(componentCount);
    std::vector<char> assignments;
    for (int component = 0; component < componentCount; ++component) {
        assignmentOffsets[component] = static_cast<std::uint32_t>(assignments.size());
        const Structure::Instance& instance = structure.components[component];
        const Structure::Shape& shape = shapes.get(instance.shape);
        const int boxCount = static_cast<int>(instance.boxes.count());
        ShapeSolver::DfsSolver::forEachAssignment(
            shape, [&](auto assignment, long double) {
                for (int box = 0; box < boxCount; ++box)
                    assignments.push_back(assignment[box]);
                ++assignmentCounts[component];
            });
    }
    assignmentOffsets[componentCount] = static_cast<std::uint32_t>(assignments.size());

    auto enumerateComponents = [&](auto&& self, int component, int used) -> void {
        if (component == componentCount) {
            const int left = mines - used;
            if (left < 0 || left > static_cast<int>(tCells.size())) return;
            auto chooseT = [&](auto&& choose, int start, int remaining) -> void {
                if (remaining == 0) {
                    ++session.possibilityCount;
                    const std::size_t row = session.mine.size();
                    session.mine.resize(row + session.candidateCount, 0);
                    for (CandidateId candidate : placed) {
                        session.mineCells.push_back(candidate);
                        session.mine[row + candidate] = 1;
                    }
                    session.mineOffsets.push_back(
                        static_cast<std::uint32_t>(session.mineCells.size()));
                    return;
                }
                for (int i = start; i <= static_cast<int>(tCells.size()) - remaining; ++i) {
                    placed.push_back(tCells[i]);
                    choose(choose, i + 1, remaining - 1);
                    placed.pop_back();
                }
            };
            chooseT(chooseT, 0, left);
            return;
        }
        const Structure::Instance& instance = structure.components[component];
        const int boxCount = static_cast<int>(instance.boxes.count());
        const std::uint32_t assignmentOffset = assignmentOffsets[component];
        for (std::uint32_t index = 0; index < assignmentCounts[component]; ++index) {
            const int assignmentStart =
                static_cast<int>(assignmentOffset + index * boxCount);
            int componentMines = 0;
            for (int box = 0; box < boxCount; ++box)
                componentMines += assignments[assignmentStart + box];
            if (used + componentMines > mines) continue;
            auto chooseCells = [&](auto&& choose, int box, int start,
                                   int remaining) -> void {
                if (remaining == 0) {
                    if (box + 1 == boxCount) {
                        self(self, component + 1, used + componentMines);
                        return;
                    }
                    choose(choose, box + 1, 0,
                           assignments[assignmentStart + box + 1]);
                    return;
                }
                const int first = instance.boxes.boxOf[box];
                const int count = instance.boxes.boxOf[box + 1] - first;
                for (int i = start; i <= count - remaining; ++i) {
                    placed.push_back(static_cast<CandidateId>(
                        candidateAt[instance.boxes.cells[first + i]]));
                    choose(choose, box, i + 1, remaining - 1);
                    placed.pop_back();
                }
            };
            if (boxCount == 0) self(self, component + 1, used);
            else chooseCells(chooseCells, 0, 0, assignments[assignmentStart]);
        }
    };
    enumerateComponents(enumerateComponents, 0, 0);

    session.reveal.assign(static_cast<std::size_t>(session.possibilityCount) *
                              session.candidateCount, 0);
    for (ConfigId config = 0;
         config < static_cast<ConfigId>(session.possibilityCount); ++config)
        for (CandidateId candidate = 0;
             candidate < static_cast<CandidateId>(session.candidateCount); ++candidate) {
            int value = session.candidates[candidate].fixedMines;
            const Session::Candidate& current = session.candidates[candidate];
            for (std::uint32_t i = 0; i < current.linksCount; ++i)
                value += mineAt(session, config,
                                session.links[current.linksOffset + i]);
            session.reveal[static_cast<std::size_t>(config) *
                           session.candidateCount + candidate] = value;
        }
    return session;
}

template <bool CheckAllMoves, bool IsRoot>
inline int BruteForce::solve(
    Session& s, std::span<ConfigId> configs, int need, int depth,
    FlatHashTable<U128, int, U128Hash>& table, Result& result) {
    if constexpr (CheckAllMoves && IsRoot) {
        ++s.nodes;
        const int n = (int)configs.size();
        result.moves.clear();
        if (need > n) return -n;
        ScratchBuffers::Layer& buf = scratch.layer(depth);
        const int m = (int)s.candidates.size();
        std::vector<int>& deaths = buf.deaths;
        deaths.assign(m, 0);
        for (ConfigId ci : configs)
            for (std::uint32_t i = s.mineOffsets[ci]; i < s.mineOffsets[ci + 1]; ++i)
                ++deaths[s.mineCells[i]];
        int best = 0;
        std::array<std::vector<ConfigId>, 9>& groups = buf.groups;
        s.unopened.for_each([&](std::size_t j) {
            const CandidateId candidate = (CandidateId)j;
            for (std::vector<ConfigId>& g : groups) g.clear();
            for (ConfigId ci : configs)
                if (!mineAt(s, ci, candidate)) groups[revealAt(s, ci, candidate)].push_back(ci);
            s.unopened.reset(j);
            int wins = 0;
            for (int r = 0; r < 9; ++r) if (!groups[r].empty()) {
                if (configs.size() >= 32 && groups[r].size() < 32)
                {
                    ++s.smallEntries;
                    s.smallEntryConfigTotal += groups[r].size();
                    s.smallEntryMaskWork +=
                        static_cast<long long>(groups[r].size()) * s.candidateCount;
                }
                const int value = solve<false, false>(s, groups[r], 1, depth + 1,
                                                      table, result);
                if (value > 0) wins += value;
            }
            s.unopened.set(j);
            result.moves.push_back({s.candidates[j].x, s.candidates[j].y, wins});
            best = (std::max)(best, wins);
        });
        return best;
    } else {
        ++s.nodes;
        const int n = (int)configs.size();
        if (n < 32) ++s.smallNodes;
        if (n <= 1) {
            if (need > n) return -n;
            if constexpr (IsRoot) if (n == 1)
                for (int j = 0; j < static_cast<int>(s.candidates.size()); ++j)
                    if (!mineAt(s, configs[0], j)) {
                        result.moves[0].x = s.candidates[j].x;
                        result.moves[0].y = s.candidates[j].y;
                        break;
                    }
            return n;
        }
        if (need > n) return -n;
        if (n < 32) {
            ++s.smallHashRequests;
            s.smallHashedConfigs += n;
        }
        const U128 key = hashConfigs(configs);
        if (n < 32) ++s.cacheQueries[n];
        if (const int* cached = table.find(key)) {
            if (n < 32) ++s.cacheHits[n];
            if (*cached >= 0) return *cached >= need ? *cached : -*cached;
            if (-*cached < need) return *cached;
        }
        ScratchBuffers::Layer& buf = scratch.layer(depth);
        const int m = (int)s.candidates.size();
        std::vector<int>& deaths = buf.deaths;
        deaths.assign(m, 0);
        for (ConfigId ci : configs)
            for (std::uint32_t i = s.mineOffsets[ci]; i < s.mineOffsets[ci + 1]; ++i)
                ++deaths[s.mineCells[i]];
        std::vector<int>& safeCells = buf.safeCells;
        safeCells.clear();
        s.unopened.for_each([&](std::size_t j) {
            if (deaths[j] == 0) safeCells.push_back((int)j);
        });
        if (n < 32) {
            s.smallSafeCells += safeCells.size();
            s.smallSafeCellConfigChecks +=
                static_cast<long long>(n) * safeCells.size();
        }
        if (!safeCells.empty()) {
            if constexpr (IsRoot) {
                result.moves[0].x = s.candidates[safeCells[0]].x;
                result.moves[0].y = s.candidates[safeCells[0]].y;
            }
            for (int j : safeCells) s.unopened.reset(j);
            std::vector<U128>& hashes = buf.safeHashes;
            hashes.clear();
            const std::size_t keyLen = safeCells.size();
            for (ConfigId ci : configs) {
                U128Hasher hasher;
                for (std::size_t i = 0; i < keyLen; ++i)
                    hasher.mix(static_cast<std::uint64_t>(revealAt(
                        s, ci, safeCells[i])) * (keyLen + 1) + i);
                hashes.push_back(hasher.finalize());
            }
            std::vector<std::span<ConfigId>>& groupList = buf.safeGroupList;
            groupList.clear();
            FlatHashTable<U128, int, U128Hash>& groupTable = scratch.safeGroupTable;
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
                    slot = (int)groupSizes.size() + 1;
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
                      [](auto a, auto b) { return a.size() > b.size(); });
            std::vector<int>& suffix = buf.suffix;
            suffix.assign(groupList.size() + 1, 0);
            for (int i = static_cast<int>(groupList.size()) - 1; i >= 0; --i)
                suffix[i] = suffix[i + 1] + (int)groupList[i].size();
            int wins = 0;
            bool bailed = false;
            int upper = 0;
            for (int i = 0; i < static_cast<int>(groupList.size()); ++i) {
                const int size = (int)groupList[i].size();
                if (wins + size + suffix[i + 1] < need) {
                    upper = wins + size + suffix[i + 1];
                    bailed = true;
                    break;
                }
                if (configs.size() >= 32 && groupList[i].size() < 32)
                {
                    ++s.smallEntries;
                    s.smallEntryConfigTotal += groupList[i].size();
                    s.smallEntryMaskWork +=
                        static_cast<long long>(groupList[i].size()) * s.candidateCount;
                }
                const int value = solve<false, false>(
                    s, groupList[i], (std::max)(1, need - wins - suffix[i + 1]),
                    depth + 1, table, result);
                if (value <= 0) {
                    upper = wins - value + suffix[i + 1];
                    bailed = true;
                    break;
                }
                wins += value;
            }
            for (int j : safeCells) s.unopened.set(j);
            if (bailed) {
                saveFail(key, upper, n, table);
                return -upper;
            }
            table[key] = wins;
            return wins;
        }

        std::vector<int>& order = buf.order;
        order.clear();
        s.unopened.for_each([&](std::size_t j) { order.push_back((int)j); });
        std::sort(order.begin(), order.end(),
                  [&](int a, int b) { return deaths[a] < deaths[b]; });
        int best = 0;
        int upper = 0;
        for (int j : order) {
            if (n < 32) {
                ++s.smallCandidateEvals;
                s.smallCandidateConfigChecks += n;
            }
            const int target = (std::max)(best + 1, need);
            if (n - deaths[j] < target) {
                upper = (std::max)(upper, n - deaths[j]);
                break;
            }
            std::array<std::vector<ConfigId>, 9>& groups = buf.groups;
            for (std::vector<ConfigId>& g : groups) g.clear();
            int groupCount = 0;
            for (ConfigId ci : configs) if (!mineAt(s, ci, j)) {
                const int r = revealAt(s, ci, j);
                if (groups[r].empty()) ++groupCount;
                groups[r].push_back(ci);
            }
            if (groupCount <= 1) {
                upper = (std::max)(upper, n - deaths[j]);
                continue;
            }
            std::vector<std::pair<int, int>>& groupList = buf.groupList;
            groupList.clear();
            for (int r = 0; r < 9; ++r)
                if (!groups[r].empty()) groupList.push_back({r, (int)groups[r].size()});
            std::sort(groupList.begin(), groupList.end(),
                      [](const auto& a, const auto& b) { return a.second > b.second; });
            std::vector<int>& suffix = buf.suffix;
            suffix.assign(groupList.size() + 1, 0);
            for (int i = static_cast<int>(groupList.size()) - 1; i >= 0; --i)
                suffix[i] = suffix[i + 1] + groupList[i].second;
            s.unopened.reset(j);
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
                if (configs.size() >= 32 && groupList[i].second < 32)
                {
                    ++s.smallEntries;
                    s.smallEntryConfigTotal += groupList[i].second;
                    s.smallEntryMaskWork +=
                        static_cast<long long>(groupList[i].second) * s.candidateCount;
                }
                const int value = solve<false, false>(
                    s, group, (std::max)(1, target - wins - suffix[i + 1]),
                    depth + 1, table, result);
                if (value <= 0) {
                    moveUpper = wins - value + suffix[i + 1];
                    bailed = true;
                    break;
                }
                wins += value;
            }
            s.unopened.set(j);
            if (bailed) upper = (std::max)(upper, moveUpper);
            if (!bailed && wins > best) {
                best = wins;
                if constexpr (IsRoot) {
                    result.moves[0].x = s.candidates[j].x;
                    result.moves[0].y = s.candidates[j].y;
                }
            }
        }
        if (best >= need) {
            table[key] = best;
            return best;
        }
        if (best == 0 && need <= 1) {
            if constexpr (IsRoot)
                for (int j = 0; j < m; ++j) if (!mineAt(s, configs[0], j)) {
                    result.moves[0].x = s.candidates[j].x;
                    result.moves[0].y = s.candidates[j].y;
                    break;
                }
            return 1;
        }
        saveFail(key, (std::max)(best, upper), n, table);
        return -(std::max)(best, upper);
    }
}

inline BruteForce::Result BruteForce::solve(
    const ObservedBoard::Result& board, const Basic::Result& basic,
    const Structure::Result& structure, const Structure::ShapePool& shapes,
    const Config& config) {
    Session session = buildSession(board, basic, structure, shapes);
    scratch.reset();
    cache.clear();
    session.unopened.resize(session.candidateCount);
    session.unopened.setAll();
    Result result;
    result.possibilities = session.possibilityCount;
    if (session.possibilityCount == 0 || session.candidateCount == 0) return result;
    std::vector<ConfigId> configs(session.possibilityCount);
    for (ConfigId i = 0;
         static_cast<int>(i) < session.possibilityCount; ++i)
        configs[i] = i;
    if (config.checkAllMoves) {
        solve<true, true>(session, configs, 1, 0, cache, result);
    } else {
        result.moves.resize(1);
        const int wins = solve<false, true>(
            session, configs, config.minWins, 0, cache, result);
        if (wins >= config.minWins) result.moves[0].wins = wins;
        else result.moves.clear();
    }
    result.nodes = session.nodes;
    if (session.possibilityCount < 32) {
        ++session.smallEntries;
        session.smallEntryConfigTotal += session.possibilityCount;
        session.smallEntryMaskWork =
            static_cast<long long>(session.possibilityCount) * session.candidateCount;
    }
    std::fprintf(stderr,
                 "bruteforce stats: smallEntries=%lld smallNodes=%lld "
                 "smallHashRequests=%lld smallHashedConfigs=%lld "
                 "entryConfigs=%lld entryMaskWork=%lld\n",
                 session.smallEntries, session.smallNodes,
                 session.smallHashRequests, session.smallHashedConfigs,
                 session.smallEntryConfigTotal, session.smallEntryMaskWork);
    std::fprintf(stderr,
                 "small grouping: candidateEvals=%lld candidateConfigChecks=%lld "
                 "safeCells=%lld safeCellConfigChecks=%lld\n",
                 session.smallCandidateEvals, session.smallCandidateConfigChecks,
                 session.smallSafeCells, session.smallSafeCellConfigChecks);
    std::fprintf(stderr, "small cache hits/queries:");
    for (int n = 2; n <= 8; ++n)
        std::fprintf(stderr, " n%d=%lld/%lld", n,
                     session.cacheHits[n], session.cacheQueries[n]);
    std::fputc('\n', stderr);
    cache.clear();
    return result;
}

}  // namespace mss
