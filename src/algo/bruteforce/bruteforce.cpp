#include "algo/bruteforce/bruteforce.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <deque>
#include <span>
#include <utility>

#include "algo/shape_solver/dfs_solver.h"
#include "core/utility/dynamic_bitset.h"
#include "core/utility/flat_hashtable.h"
#include "core/utility/hash.h"
#include "core/utility/radix_sort.h"

namespace mss {

namespace BruteForce {

namespace {

using ConfigId = std::uint32_t;
using CandidateId = std::uint32_t;

struct Session {
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
    // 当前路径仍可点击的格；DFS 下潜 reset，返回时 set 恢复。
    DynamicBitset unopened;
    long long nodes = 0;
};

struct ScratchBuffers {
    struct Layer {
        std::vector<int> deaths;
        std::vector<int> safeCells;
        std::vector<int> order;
        std::vector<int> suffix;
        std::array<std::vector<ConfigId>, 9> groups;
        std::vector<std::pair<int, int>> groupList;
        std::vector<U128> safeHashes;
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
            l.safeGroupList.clear();
        }
    }

    std::deque<Layer> layers;
};

thread_local ScratchBuffers scratch;
thread_local FlatHashTable<U128, int, U128Hash> cache;

int revealAt(const Session& session, ConfigId config, CandidateId candidate) {
    return session.reveal[static_cast<std::size_t>(config) *
                              session.candidateCount + candidate];
}

bool mineAt(const Session& session, ConfigId config, CandidateId candidate) {
    return session.mine[static_cast<std::size_t>(config) *
                            session.candidateCount + candidate] != 0;
}

U128 hashConfigs(std::span<const ConfigId> configs) {
    U128Hasher hasher;
    const std::uint64_t count = configs.size();
    for (int i = 0; i < static_cast<int>(configs.size()); ++i)
        hasher.mix(static_cast<std::uint64_t>(configs[i]) * (count + 1) + i);
    return hasher.finalize();
}

// 正数是精确 wins；负数 -n 表示真实 wins 不超过 n；零不进缓存。
// 配置数是没有更精确信息时的天然上界。
// 多次失败只保留更低的上界；天然上界不能剪枝，也不写入。
void saveFail(const U128& key, int upper, int count,
              FlatHashTable<U128, int, U128Hash>& cache) {
    if (upper <= 0 || upper >= count) return;
    int* old = cache.find(key);
    if (old == nullptr) {
        cache[key] = -upper;
        return;
    }
    if (*old < 0) *old = (std::max)(*old, -upper);
}

Session buildSession(const ObservedBoard::Result& board,
                     const Basic::Result& basic,
                     const Structure::Result& structure,
                     const Probability::Result& probability,
                     const Structure::ShapePool& shapes,
                     ShapeSolver::Distribution::Pool& distributions) {
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
         candidate < static_cast<CandidateId>(session.candidates.size());
         ++candidate) {
        Session::Candidate& current = session.candidates[candidate];
        current.linksOffset = static_cast<std::uint32_t>(session.links.size());
        forEachAdjacent(current.x, current.y, board.rows, board.cols,
                        [&](int x, int y) {
                            if (basic.marks[x][y] == Basic::Mark::F)
                                ++current.fixedMines;
                            const int linked = candidateAt[board.id(x, y)];
                            if (linked >= 0) session.links.push_back(linked);
                        });
        current.linksCount = static_cast<std::uint8_t>(
            session.links.size() - current.linksOffset);
    }

    std::vector<CandidateId> tCells;
    for (CandidateId candidate = 0;
         candidate < static_cast<CandidateId>(session.candidates.size());
         ++candidate) {
        const Session::Candidate& current = session.candidates[candidate];
        if (basic.marks[current.x][current.y] == Basic::Mark::T)
            tCells.push_back(candidate);
    }

    session.mineOffsets.push_back(0);
    std::vector<CandidateId> placed;
    const int mines = board.totalMines - basic.mineSum;

    const int componentCount = static_cast<int>(structure.components.size());
    std::vector<std::uint32_t> assignmentOffsets(componentCount + 1);
    std::vector<std::uint32_t> assignmentCounts(componentCount);
    std::vector<char> assignments;
    for (int component = 0; component < componentCount; ++component) {
        assignmentOffsets[component] =
            static_cast<std::uint32_t>(assignments.size());
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
    assignmentOffsets[componentCount] =
        static_cast<std::uint32_t>(assignments.size());

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
                for (int i = start;
                     i <= static_cast<int>(tCells.size()) - remaining; ++i) {
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
        for (std::uint32_t index = 0;
             index < assignmentCounts[component]; ++index) {
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

            if (boxCount == 0)
                self(self, component + 1, used);
            else
                chooseCells(chooseCells, 0, 0, assignments[assignmentStart]);
        }
    };
    enumerateComponents(enumerateComponents, 0, 0);

    session.reveal.assign(static_cast<std::size_t>(session.possibilityCount) *
                              session.candidateCount,
                          0);
    for (ConfigId config = 0;
         config < static_cast<ConfigId>(session.possibilityCount); ++config)
        for (CandidateId candidate = 0;
             candidate < static_cast<CandidateId>(session.candidateCount);
             ++candidate) {
            int value = session.candidates[candidate].fixedMines;
            const Session::Candidate& current = session.candidates[candidate];
            for (std::uint32_t i = 0; i < current.linksCount; ++i)
                value += mineAt(session, config,
                                session.links[current.linksOffset + i]);
            session.reveal[static_cast<std::size_t>(config) *
                               session.candidateCount + candidate] = value;
        }

    (void)probability;
    (void)distributions;
    return session;
}

template <bool CheckAllMoves, bool IsRoot>
int solve(Session& s, std::span<ConfigId> configs, int need, int depth,
          FlatHashTable<U128, int, U128Hash>& cache, Result& result) {
    // 正数是满足 need 的精确 wins；负数表示未求出精确值，-返回值是真实上界；
    // 零表示上界为零。负值让上界沿递归直接向上传递，避免父节点再次查缓存。
    if constexpr (CheckAllMoves && IsRoot) {
        ++s.nodes;
        const int n = configs.size();
        result.moves.clear();
        if (need > n) return -n;
        ScratchBuffers::Layer& buf = scratch.layer(depth);
        const int m = s.candidates.size();
        std::vector<int>& deaths = buf.deaths;
        deaths.assign(m, 0);
        for (ConfigId ci : configs)
            for (std::uint32_t i = s.mineOffsets[ci]; i < s.mineOffsets[ci + 1]; ++i)
                ++deaths[s.mineCells[i]];

        int best = 0;
        std::array<std::vector<ConfigId>, 9>& groups = buf.groups;
        s.unopened.for_each([&](std::size_t j) {
            for (std::vector<ConfigId>& g : groups) g.clear();
            for (ConfigId ci : configs)
                if (!mineAt(s, ci, j)) groups[revealAt(s, ci, j)].push_back(ci);
            s.unopened.reset(j);
            int wins = 0;
            for (int r = 0; r < 9; ++r) if (!groups[r].empty()) {
                const int value = solve<false, false>(s, groups[r], 1, depth + 1, cache, result);
                if (value > 0) wins += value;
            }
            s.unopened.set(j);
            result.moves.push_back({s.candidates[j].x, s.candidates[j].y, wins});
            best = (std::max)(best, wins);
        });
        return best;
    } else {
        ++s.nodes;
        const int n = configs.size();
        if (n <= 1) {
            // n <= 1 是刻意的终止语义，不是遗漏检查：唯一配置已经确定，
            // 此处直接返回其可胜配置数；根节点仅顺便回填落子位置。
            if (need > n) return -n;
            if constexpr (IsRoot) if (n == 1)
                for (int j = 0; j < (int)s.candidates.size(); ++j)
                    if (!mineAt(s, configs[0], j)) {
                        result.moves[0].x = s.candidates[j].x;
                        result.moves[0].y = s.candidates[j].y;
                        break;
                    }
            return n;
        }
        if (need > n) return -n;

        const U128 key = hashConfigs(configs);
        if (const int* cached = cache.find(key)) {
            if (*cached >= 0) return *cached >= need ? *cached : -*cached;
            if (-*cached < need) return *cached;
        }
        ScratchBuffers::Layer& buf = scratch.layer(depth);
        const int m = s.candidates.size();
        std::vector<int>& deaths = buf.deaths;
        deaths.assign(m, 0);
        for (ConfigId ci : configs)
            for (std::uint32_t i = s.mineOffsets[ci]; i < s.mineOffsets[ci + 1]; ++i)
                ++deaths[s.mineCells[i]];

        // death 为零的未开格可免费获得信息，必须先于风险分支处理。
        std::vector<int>& safeCells = buf.safeCells;
        safeCells.clear();
        s.unopened.for_each([&](std::size_t j) {
            if (deaths[j] == 0) safeCells.push_back(j);
        });
        if (!safeCells.empty()) {
            if constexpr (IsRoot) {
                result.moves[0].x = s.candidates[safeCells[0]].x;
                result.moves[0].y = s.candidates[safeCells[0]].y;
            }
            // 安全格在所有配置中都不会死，先统一点开再按观测向量分支。
            for (int j : safeCells) s.unopened.reset(j);
            std::vector<U128>& hashes = buf.safeHashes;
            hashes.clear();
            const std::size_t keyLen = safeCells.size();
            for (ConfigId ci : configs) {
                U128Hasher hasher;
                for (std::size_t i = 0; i < keyLen; ++i)
                    hasher.mix(static_cast<std::uint64_t>(revealAt(s, ci, safeCells[i])) * (keyLen + 1) + i);
                hashes.push_back(hasher.finalize());
            }
            radix_sort::sortBy(hashes.size(),
                               [&](std::size_t i) { return hashes[i]; },
                               [&](std::size_t dst, std::size_t src) {
                                   hashes[dst] = hashes[src];
                                   configs[dst] = configs[src];
                               },
                               [&](std::size_t i, std::size_t j) {
                                   std::swap(hashes[i], hashes[j]);
                                   std::swap(configs[i], configs[j]);
                               });
            // radix 后相同观测向量已相邻；只需切连续 span，不再建分组 vector。
            std::vector<std::span<ConfigId>>& groupList = buf.safeGroupList;
            groupList.clear();
            for (std::size_t i = 0; i < hashes.size();) {
                std::size_t j = i + 1;
                while (j < hashes.size() && hashes[j] == hashes[i]) ++j;
                groupList.emplace_back(configs.data() + i, j - i);
                i = j;
            }
            std::sort(groupList.begin(), groupList.end(),
                      [](auto a, auto b) { return a.size() > b.size(); });
            std::vector<int>& suffix = buf.suffix;
            suffix.assign(groupList.size() + 1, 0);
            for (int i = (int)groupList.size() - 1; i >= 0; --i) suffix[i] = suffix[i + 1] + groupList[i].size();
            int wins = 0;
            bool bailed = false;
            int upper = 0;
            for (int i = 0; i < (int)groupList.size(); ++i) {
                const int size = groupList[i].size();
                if (wins + size + suffix[i + 1] < need) {
                    upper = wins + size + suffix[i + 1];
                    bailed = true;
                    break;
                }
                const int value = solve<false, false>(s, groupList[i], (std::max)(1, need - wins - suffix[i + 1]), depth + 1, cache, result);
                if (value <= 0) {
                    upper = wins - value + suffix[i + 1];
                    bailed = true;
                    break;
                }
                wins += value;
            }
            for (int j : safeCells) s.unopened.set(j);
            if (bailed) {
                saveFail(key, upper, n, cache);
                return -upper;
            }
            cache[key] = wins;
            return wins;
        }

        // 没有免费信息时，按死亡配置数从少到多尝试九路观测分组。
        std::vector<int>& order = buf.order;
        order.clear();
        // 只枚举尚未点开的候选格；bitset 枚举不扫描已打开格。
        s.unopened.for_each([&](std::size_t j) { order.push_back(j); });
        std::sort(order.begin(), order.end(), [&](int a, int b) { return deaths[a] < deaths[b]; });
        int best = 0;
        int upper = 0;
        for (int j : order) {
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
            for (int r = 0; r < 9; ++r) if (!groups[r].empty()) groupList.push_back({r, groups[r].size()});
            std::sort(groupList.begin(), groupList.end(), [](const auto& a, const auto& b) { return a.second > b.second; });
            std::vector<int>& suffix = buf.suffix;
            suffix.assign(groupList.size() + 1, 0);
            for (int i = (int)groupList.size() - 1; i >= 0; --i) suffix[i] = suffix[i + 1] + groupList[i].second;
            s.unopened.reset(j);
            int wins = 0;
            bool bailed = false;
            int moveUpper = 0;
            for (int i = 0; i < (int)groupList.size(); ++i) {
                std::vector<ConfigId>& group = groups[groupList[i].first];
                if (wins + groupList[i].second + suffix[i + 1] < target) {
                    moveUpper = wins + groupList[i].second + suffix[i + 1];
                    bailed = true;
                    break;
                }
                const int value = solve<false, false>(s, group, (std::max)(1, target - wins - suffix[i + 1]), depth + 1, cache, result);
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
        if (best >= need) { cache[key] = best; return best; }
        if (best == 0 && need <= 1) {
            if constexpr (IsRoot)
                for (int j = 0; j < m; ++j) if (!mineAt(s, configs[0], j)) {
                    result.moves[0].x = s.candidates[j].x;
                    result.moves[0].y = s.candidates[j].y;
                    break;
                }
            return 1;
        }
        saveFail(key, (std::max)(best, upper), n, cache);
        return -(std::max)(best, upper);
    }
}

}  // namespace

Result solve(const ObservedBoard::Result& board,
             const Basic::Result& basic,
             const Structure::Result& structure,
             const Probability::Result& probability,
             const Structure::ShapePool& shapes,
             ShapeSolver::Distribution::Pool& distributions,
             const Config& config) {
    Session session = buildSession(board, basic, structure, probability,
                                   shapes, distributions);
    scratch.reset();
    cache.clear();
    session.unopened.resize(session.candidateCount);
    session.unopened.setAll();

    Result result;
    result.possibilities = session.possibilityCount;
    if (session.possibilityCount == 0 || session.candidateCount == 0) return result;
    std::vector<ConfigId> configs(session.possibilityCount);
    for (ConfigId i = 0; (int)i < session.possibilityCount; ++i) configs[i] = i;

    if (config.checkAllMoves) {
        solve<true, true>(session, configs, 1, 0, cache, result);
    } else {
        result.moves.resize(1);
        const int wins = solve<false, true>(session, configs, config.minWins, 0, cache, result);
        if (wins >= config.minWins)
            result.moves[0].wins = wins;
        else
            result.moves.clear();
    }

    result.nodes = session.nodes;
    cache.clear();
    return result;
}

}  // namespace BruteForce

}  // namespace mss
