#pragma once

#include <algorithm>

#include "algo/bruteforce/bruteforce_common.h"
#include "algo/bruteforce/bruteforce_normal.h"
#include "algo/bruteforce/multimask/bruteforce_multimask.h"
#include "core/assert.h"

//==============================================================================

namespace mss {

// 自动路径在候选格不多时使用定宽多掩码后端；Common 路径保留给对拍/基准。

inline U128 BruteForce::hashConfigs(std::span<const ConfigId> configs) {
    // 这是无序集合哈希：递归分组的顺序可能变化，但缓存键必须保持相同。
    // 缓存键故意不含 unopened：同一 configs 下，已经从 unopened 移除的格子
    // 必然对所有方案都安全；solve 会在继续分支前统一消掉这些共同安全格，
    // 所以 exact 结果只由 configs 决定。upper 是带 need 的剪枝上界，取决于
    // 本次搜索停在哪里，不是 configs 的固有结果。
    // 同一个 splitmix 值同时进入 sum/xor 两个无序通道；最后只混合 sum，
    // 打断它与 xor 低位之间的相关性。
    U128 hash{configs.size(), configs.size()};
    for (ConfigId config : configs) {
        const std::uint64_t value = splitmix64(config);
        hash.lo += value;
        hash.hi ^= value;
    }
    hash.lo = splitmix64(hash.lo);
    return hash;
}

inline void BruteForce::saveFail(const U128 &key, int upper, int count, FlatHashTable<U128, int, U128Hash> &table) {
    // 保存当前方案集合的可证明失败上界。负值只表示“当前阈值未达到”，不是
    // 负的胜局数；solve 读取它时会比较绝对值与新的 need。
    if (upper <= 0 || upper >= count)
        return;
    int *old = table.find(key);
    if (old == nullptr) {
        table[key] = -upper;
        return;
    }
    // 负值编码上界；对负数取 max 等价于保留更小的正上界。
    if (*old < 0)
        *old = (std::max)(*old, -upper);
}

template <typename Solver, typename SessionT>
inline BruteForce::Result BruteForce::runSolver(const CommonSession &common, SessionT &session, const Config &config,
                                                FlatHashTable<U128, int, U128Hash> &table) {
    // 外层负责把带符号的递归结果翻译成公开的 moves：负值只是"未达到 minWins 的
    // 可赢上界"，绝不能泄漏成公开的 Move::wins。多掩码与普通后端共用这段驱动。
    Result result;
    result.possibilities = common.possibilityCount;
    std::vector<ConfigId> configs(common.possibilityCount);
    for (int i = 0; i < (int)(configs.size()); ++i)
        configs[i] = i;
    if (config.checkAllMoves) {
        // 该模式直接暴露根节点各候选的可赢数；递归负值不写进公开的 Move::wins。
        Solver::template solve<true, true>(common, session, configs, 1, 0, table, result);
    } else {
        result.moves.resize(1);
        // 单推荐格模式把 minWins 交给递归做阈值剪枝；只有正返回值才形成推荐步，
        // 负值说明最多只能赢 abs(value) 局，因此清空公开动作结果。
        const int wins = Solver::template solve<false, true>(common, session, configs, config.minWins, 0, table, result);
        if (wins >= config.minWins)
            result.moves[0].wins = wins;
        else
            result.moves.clear();
    }
    result.nodes = session.nodes;
    return result;
}

template <typename Mask>
inline BruteForce::Result BruteForce::solveWithMask(const CommonSession &common, const Config &config) {
    // 掩码后端需要自己的 scratch/cache，二者都是跨调用复用的 thread_local。
    MultiMaskSession<Mask> session = MultiMaskSolver<Mask>::buildSession(common);
    session.unopenedCandidates = Mask::all(common.candidateCount);
    workspace::BruteForceMultiMask::scratch<Mask>.reset();
    workspace::BruteForceMultiMask::cache<Mask>.clear();
    return runSolver<MultiMaskSolver<Mask>>(common, session, config, workspace::BruteForceMultiMask::cache<Mask>);
}

inline BruteForce::CommonSession BruteForce::buildCommonSession(const ObservedBoard::Result &board, const Basic::Result &basic,
                                                                const Structure::Result &structure, const Structure::Pool &shapes) {
    // 从分析结果枚举完整雷位方案，并建立残局搜索的稠密索引。
    CommonSession session;
    const int cellCount = (board.rows + 1) * (board.cols + 1);
    std::vector<int> candidateAt(cellCount, -1);
    for (int x = 1; x <= board.rows; ++x)
        for (int y = 1; y <= board.cols; ++y) {
            if (board.board[x][y] != ObservedBoard::CellState::Hidden)
                continue;
            const Basic::Mark mark = basic.marks[x][y];
            // TODO: 在候选建立前排除全局概率为 0 或 1 的格子；当前完整 config
            // 尚未枚举，强制雷仍可能流入 candidate，造成无效搜索和排序开销。
            if (mark != Basic::Mark::H && mark != Basic::Mark::T)
                continue;
            candidateAt[board.id(x, y)] = session.candidateCount++;
            session.candidates.push_back({x, y, 0, 0, 0});
        }
    for (int candidate = 0; candidate < (int)session.candidates.size(); ++candidate) {
        CommonSession::Candidate &current = session.candidates[candidate];
        current.linksOffset = session.links.size();
        forEachAdjacent(current.x, current.y, board.rows, board.cols, [&](int x, int y) {
            if (basic.marks[x][y] == Basic::Mark::F)
                ++current.fixedMines;
            const int linked = candidateAt[board.id(x, y)];
            if (linked >= 0)
                session.links.push_back(linked);
        });
        current.linksCount = session.links.size() - current.linksOffset;
    }
    std::vector<CandidateId> tCells;
    for (int candidate = 0; candidate < (int)session.candidates.size(); ++candidate)
        if (basic.marks[session.candidates[candidate].x][session.candidates[candidate].y] == Basic::Mark::T)
            tCells.push_back(candidate);

    std::vector<CandidateId> placed;
    const int mines = board.totalMines - basic.mineSum;
    session.minesPerConfig = mines;
    const int componentCount = structure.components.size();
    std::vector<std::uint32_t> assignmentOffsets(componentCount + 1);
    std::vector<std::uint32_t> assignmentCounts(componentCount);
    std::vector<char> assignments;
    for (int component = 0; component < componentCount; ++component) {
        assignmentOffsets[component] = assignments.size();
        const Structure::Instance &instance = shapes.getInstance(structure.components[component]);
        const Structure::Shape &shape = shapes.get(instance.shape);
        const int boxCount = instance.boxes.count();
        ShapeSolver::DfsSolver::forEachAssignment(shape, shapes, [&](auto assignment, long double) {
            for (int box = 0; box < boxCount; ++box)
                assignments.push_back(assignment[box]);
            ++assignmentCounts[component];
        });
    }
    assignmentOffsets[componentCount] = assignments.size();

    auto enumerateComponents = [&](auto &&self, int component, int used, auto &&emit) -> void {
        if (component == componentCount) {
            const int left = mines - used;
            if (left < 0 || left > (int)tCells.size())
                return;
            auto chooseT = [&](auto &&choose, int start, int remaining) -> void {
                if (remaining == 0) {
                    emit(placed);
                    return;
                }
                for (int i = start; i <= (int)tCells.size() - remaining; ++i) {
                    placed.push_back(tCells[i]);
                    choose(choose, i + 1, remaining - 1);
                    placed.pop_back();
                }
            };
            chooseT(chooseT, 0, left);
            return;
        }
        const Structure::Instance &instance = shapes.getInstance(structure.components[component]);
        const int boxCount = instance.boxes.count();
        const std::uint32_t assignmentOffset = assignmentOffsets[component];
        for (std::uint32_t index = 0; index < assignmentCounts[component]; ++index) {
            const int assignmentStart = assignmentOffset + index * boxCount;
            int componentMines = 0;
            for (int box = 0; box < boxCount; ++box)
                componentMines += assignments[assignmentStart + box];
            if (used + componentMines > mines)
                continue;
            auto chooseCells = [&](auto &&choose, int box, int start, int remaining) -> void {
                if (remaining == 0) {
                    if (box + 1 == boxCount) {
                        self(self, component + 1, used + componentMines, emit);
                        return;
                    }
                    choose(choose, box + 1, 0, assignments[assignmentStart + box + 1]);
                    return;
                }
                const int first = instance.boxes.boxOf.span(shapes.boxOf)[box];
                const int count = instance.boxes.boxOf.span(shapes.boxOf)[box + 1] - first;
                for (int i = start; i <= count - remaining; ++i) {
                    placed.push_back(candidateAt[instance.boxes.cells.span(shapes.cells)[first + i]]);
                    choose(choose, box, i + 1, remaining - 1);
                    placed.pop_back();
                }
            };
            if (boxCount == 0)
                self(self, component + 1, used, emit);
            else
                chooseCells(chooseCells, 0, 0, assignments[assignmentStart]);
        }
    };
    // 先数出完整方案数，再一次性分配 [方案][第几个雷] 表，避免动态追加行。
    auto countConfigs = [&](const std::vector<CandidateId> &) {
        ++session.possibilityCount;
    };
    enumerateComponents(enumerateComponents, 0, 0, countConfigs);
    session.mineCandidateIds.resize(session.possibilityCount, mines, 0);
    int config = 0;
    auto storeConfig = [&](const std::vector<CandidateId> &mineCandidates) {
        for (int i = 0; i < mines; ++i)
            session.mineCandidateIds[config][i] = mineCandidates[i];
        ++config;
    };
    enumerateComponents(enumerateComponents, 0, 0, storeConfig);
    return session;
}

inline BruteForce::Result BruteForce::solve(const ObservedBoard::Result &board, const Basic::Result &basic,
                                            const Structure::Result &structure, const Structure::Pool &shapes, const Config &config) {
    // 按 config.solver 选择后端，返回候选动作及可赢方案数。分流必须发生在构建
    // 递归 Session 之前，因为普通后端与掩码后端的 unopened / mine 存储完全不同。
    CommonSession common = buildCommonSession(board, basic, structure, shapes);
    Result result;
    result.possibilities = common.possibilityCount;
    if (common.possibilityCount == 0 || common.candidateCount == 0)
        return result;
    // bitwise 三兄弟共用的入口：Mask 宽度按候选数选最小够用的那档，四档求解流程
    // 完全相同，只是根节点是否并行由 rootParallel 决定（见 multimask 后端）。
    // 候选数超过阈值时 Mask::all 会断言，所以这里必须先退化成普通后端。
    MultiMaskSolver<u64>::rootParallel = config.solver == Solver::BitwiseRootParallel || config.solver == Solver::BitwiseMultithread;
    if (config.solver != Solver::Common && common.possibilityCount > 1 && common.candidateCount <= multiMaskCandidateThreshold) {
        if (common.candidateCount <= 64)
            return solveWithMask<u64>(common, config);
        if (common.candidateCount <= 128)
            return solveWithMask<u128>(common, config);
        if (common.candidateCount <= 256)
            return solveWithMask<u256>(common, config);
        return solveWithMask<u512>(common, config);
    }
    // 普通后端：scratch/cache 都是跨调用复用的 thread_local，必须先清空。
    workspace::BruteForceNormal::scratch.reset();
    workspace::BruteForceNormal::cache.clear();
    Session session = buildSession(common);
    session.unopenedCandidates.resize(common.candidateCount);
    session.unopenedCandidates.setAll();
    return runSolver<BruteForce>(common, session, config, workspace::BruteForceNormal::cache);
}

} // namespace mss
