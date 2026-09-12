#pragma once

#include <algorithm>

#include "algo/bruteforce/bruteforce_common.h"
#include "algo/bruteforce/bruteforce_normal.h"
#include "algo/bruteforce/bruteforce_bitwise.h"

//==============================================================================

namespace mss {

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

inline BruteForce::CommonSession BruteForce::buildCommonSession(
    const ObservedBoard::Result& board, const Basic::Result& basic,
    const Structure::Result& structure, const Structure::ShapePool& shapes) {
    CommonSession session;
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
        CommonSession::Candidate& current = session.candidates[candidate];
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
                    for (CandidateId candidate : placed)
                        session.mineCells.push_back(candidate);
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
    return session;
}

inline BruteForce::Result BruteForce::solve(
    const ObservedBoard::Result& board, const Basic::Result& basic,
    const Structure::Result& structure, const Structure::ShapePool& shapes,
    const Config& config) {
    CommonSession common = buildCommonSession(board, basic, structure, shapes);
    Result result;
    result.possibilities = common.possibilityCount;
    if (common.possibilityCount == 0 || common.candidateCount == 0) return result;
    if (common.possibilityCount > 1 &&
        common.candidateCount < bitwiseCandidateThreshold) {
        BitwiseSession session = buildBitwiseSession(common);
        return BitwiseSolver::solve(common, session, config);
    }
    scratch.reset();
    cache.clear();
    Session session = buildSession(common);
    session.unopened.resize(common.candidateCount);
    session.unopened.setAll();
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
    cache.clear();
    return result;
}

}  // namespace mss
