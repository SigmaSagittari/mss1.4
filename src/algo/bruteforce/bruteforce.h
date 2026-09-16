#pragma once

#include <algorithm>

#include "algo/bruteforce/bruteforce_common.h"
#include "algo/bruteforce/bruteforce_normal.h"
#include "algo/bruteforce/bruteforce_multimask.h"

//==============================================================================

namespace mss {

// 自动路径在候选格不多时使用定宽多掩码后端；Common 路径保留给对拍/基准。

inline U128 BruteForce::hashConfigs(std::span<const ConfigId> configs) {
    // 这是无序集合哈希：递归分组的顺序可能变化，但缓存键必须保持相同。
    // 缓存键故意不含 unopened：同一 configs 下，已经从 unopened 移除的格子
    // 必然对所有方案都安全；solve 会在继续分支前统一消掉这些共同安全格，
    // 所以 exact 结果只由 configs 决定。upper 是带 need 的剪枝上界，取决于
    // 本次搜索停在哪里，不是 configs 的固有结果。
    // 将当前方案下标集合压缩成搜索缓存使用的 128 位键。
    U128 hash{configs.size(), configs.size()};
    for (ConfigId config : configs)
        hash += {splitmix64(config), splitmix64(config + 0x9e3779b97f4a7c15ULL)};
    return hash;
}

inline void BruteForce::saveFail(
    const U128& key, int upper, int count,
    FlatHashTable<U128, int, U128Hash>& table) {
    // 保存当前方案集合的可证明失败上界。负值只表示“当前阈值未达到”，不是
    // 负的胜局数；solve 读取它时会比较绝对值与新的 need。
    if (upper <= 0 || upper >= count) return;
    int* old = table.find(key);
    if (old == nullptr) {
        table[key] = -upper;
        return;
    }
    // 负值编码上界；对负数取 max 等价于保留更小的正上界。
    if (*old < 0) *old = (std::max)(*old, -upper);
}

inline BruteForce::CommonSession BruteForce::buildCommonSession(
    const ObservedBoard::Result& board, const Basic::Result& basic,
    const Structure::Result& structure, const Structure::Pool& shapes) {
    // 从分析结果枚举完整雷位方案，并建立残局搜索的稠密索引。
    CommonSession session;
    const int cellCount = (board.rows + 1) * (board.cols + 1);
    std::vector<int> candidateAt(cellCount, -1);
    for (int x = 1; x <= board.rows; ++x)
        for (int y = 1; y <= board.cols; ++y) {
            if (board.board[x][y] != ObservedBoard::CellState::Hidden) continue;
            const Basic::Mark mark = basic.marks[x][y];
            // TODO: 在候选建立前排除全局概率为 0 或 1 的格子；当前完整 config
            // 尚未枚举，强制雷仍可能流入 candidate，造成无效搜索和排序开销。
            if (mark != Basic::Mark::H && mark != Basic::Mark::T) continue;
            candidateAt[board.id(x, y)] = session.candidateCount++;
            session.candidates.push_back({x, y, 0, 0, 0});
        }
    for (int candidate = 0;
         candidate < (int)session.candidates.size(); ++candidate) {
        CommonSession::Candidate& current = session.candidates[candidate];
        current.linksOffset = session.links.size();
        forEachAdjacent(current.x, current.y, board.rows, board.cols,
                        [&](int x, int y) {
            if (basic.marks[x][y] == Basic::Mark::F) ++current.fixedMines;
            const int linked = candidateAt[board.id(x, y)];
            if (linked >= 0) session.links.push_back(linked);
        });
        current.linksCount = session.links.size() - current.linksOffset;
    }
    std::vector<CandidateId> tCells;
    for (int candidate = 0;
         candidate < (int)session.candidates.size(); ++candidate)
        if (basic.marks[session.candidates[candidate].x]
                        [session.candidates[candidate].y] == Basic::Mark::T)
            tCells.push_back(candidate);

    session.mineOffsets.push_back(0);
    std::vector<CandidateId> placed;
    const int mines = board.totalMines - basic.mineSum;
    const int componentCount = structure.components.size();
    std::vector<std::uint32_t> assignmentOffsets(componentCount + 1);
    std::vector<std::uint32_t> assignmentCounts(componentCount);
    std::vector<char> assignments;
    for (int component = 0; component < componentCount; ++component) {
        assignmentOffsets[component] = assignments.size();
        const Structure::Instance& instance = shapes.getInstance(
            structure.components[component]);
        const Structure::Shape& shape = shapes.get(instance.shape);
        const int boxCount = instance.boxes.count();
        ShapeSolver::DfsSolver::forEachAssignment(
            shape, [&](auto assignment, long double) {
                for (int box = 0; box < boxCount; ++box)
                    assignments.push_back(assignment[box]);
                ++assignmentCounts[component];
            });
    }
    assignmentOffsets[componentCount] = assignments.size();

    auto enumerateComponents = [&](auto&& self, int component, int used) -> void {
        if (component == componentCount) {
            const int left = mines - used;
            if (left < 0 || left > (int)tCells.size()) return;
            auto chooseT = [&](auto&& choose, int start, int remaining) -> void {
                if (remaining == 0) {
                    ++session.possibilityCount;
                    for (CandidateId candidate : placed)
                        session.mineCells.push_back(candidate);
                    session.mineOffsets.push_back(
                        session.mineCells.size());
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
        const Structure::Instance& instance = shapes.getInstance(
            structure.components[component]);
        const int boxCount = instance.boxes.count();
        const std::uint32_t assignmentOffset = assignmentOffsets[component];
        for (std::uint32_t index = 0; index < assignmentCounts[component]; ++index) {
            const int assignmentStart =
                assignmentOffset + index * boxCount;
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
                    placed.push_back(candidateAt[instance.boxes.cells[first + i]]);
                    choose(choose, box, i + 1, remaining - 1);
                    placed.pop_back();
                }
            };
            if (boxCount == 0) self(self, component + 1, used);
            else chooseCells(chooseCells, 0, 0, assignments[assignmentStart]);
        }
    };
    enumerateComponents(enumerateComponents, 0, 0);
    return session;
}

inline BruteForce::Result BruteForce::solve(
    const ObservedBoard::Result& board, const Basic::Result& basic,
    const Structure::Result& structure, const Structure::Pool& shapes,
    const Config& config) {
    // 按配置选择多掩码或普通递归后端，返回候选动作及可赢方案数。
    // Automatic 只在候选格不超过 512 且方案数大于 1 时走多掩码；Common 用于
    // 与新后端对拍。这里的分流必须发生在构建递归 Session 之前，因为两套 Session
    // 的 unopened 和 mine 存储完全不同。
    CommonSession common = buildCommonSession(board, basic, structure, shapes);
    Result result;
    result.possibilities = common.possibilityCount;
    if (common.possibilityCount == 0 || common.candidateCount == 0) return result;
    if (config.route != Config::Route::Common &&
        common.possibilityCount > 1 &&
        common.candidateCount <= multiMaskCandidateThreshold) {
        if (common.candidateCount <= 64) {
            MultiMaskSession<u64> session =
                MultiMaskSolver<u64>::buildSession(common);
            return MultiMaskSolver<u64>::solve(common, session, config);
        }
        if (common.candidateCount <= 128) {
            MultiMaskSession<u128> session =
                MultiMaskSolver<u128>::buildSession(common);
            return MultiMaskSolver<u128>::solve(common, session, config);
        }
        if (common.candidateCount <= 256) {
            MultiMaskSession<u256> session =
                MultiMaskSolver<u256>::buildSession(common);
            return MultiMaskSolver<u256>::solve(common, session, config);
        }
        MultiMaskSession<u512> session =
            MultiMaskSolver<u512>::buildSession(common);
        return MultiMaskSolver<u512>::solve(common, session, config);
    }
    scratch.reset();
    cache.clear();
    Session session = buildSession(common);
    session.unopened.resize(common.candidateCount);
    session.unopened.setAll();
    std::vector<ConfigId> configs(common.possibilityCount);
    for (int i = 0; i < (int)configs.size(); ++i)
        configs[i] = i;
    if (config.checkAllMoves) {
        // 该模式直接把根节点每个候选的可赢数写入 result；递归中的负数只作为
        // 未达到阈值时的上界参与剪枝，不会进入公开的 Move::wins。
        solve<true, true>(common, session, configs, 1, 0, cache, result);
    } else {
        result.moves.resize(1);
        // 单步模式把 minWins 传入递归：正返回值才是推荐步的可赢数；负返回值
        // 表示当前残局最多只能赢 abs(value) 局，故公开结果必须保持为空。
        const int wins = solve<false, true>(
            common, session, configs, config.minWins, 0, cache, result);
        if (wins >= config.minWins) result.moves[0].wins = wins;
        else result.moves.clear();
    }
    result.nodes = session.nodes;
    cache.clear();
    return result;
}

}  // namespace mss
