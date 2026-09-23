#pragma once

#include <cstddef>
#include <span>
#include <vector>

#include "algo/probability_engine/basic.h"
#include "algo/probability_engine/probability/probability.h"
#include "algo/probability_engine/probability/probability_external.h"
#include "algo/probability_engine/structure.h"
#include "core/types.h"

namespace mss {

struct LogicStructure {

    struct Result {
        struct LogicConstraint {
            CellId number = 0; // 数字格 ID。
            int mines = 0; // 扣除确定雷后，cells 中应有的雷数。
            std::span<const CellId> cells;
        };

        struct LogicComponent {
            std::span<const CellId> cells;
            std::span<const LogicConstraint> constraints; // 所有相邻数字限制。
        };

        Result() = default;
        Result(const Result &) = delete;
        Result &operator=(const Result &) = delete;
        Result(Result &&) noexcept = default;
        Result &operator=(Result &&) noexcept = default;

        std::vector<CellId> cells;
        std::vector<CellId> constraintCells; // LogicConstraint::cells 的连续存储。
        std::vector<LogicConstraint> constraints;
        std::vector<LogicComponent> components;
        std::vector<ComponentId> cell2Component;
    };

    // 概率严格处于 (0, 1) 的隐藏格入图；共用数字约束的格归入同一分量。
    static Result analyze(const ObservedBoard::Result &board, const Basic::Result &basic, const Structure::Result &structure,
                          const Probability::Result &probability);
};

} // namespace mss

//==============================================================================
namespace mss {

inline LogicStructure::Result LogicStructure::analyze(const ObservedBoard::Result &board, const Basic::Result &basic,
                                                       const Structure::Result &structure, const Probability::Result &probability) {
    const int cellCount = (board.rows + 1) * (board.cols + 1);
    Result result;
    result.cell2Component.resize(cellCount, -1);

    int unknownCount = 0;
    for (int x = 1; x <= board.rows; ++x)
        for (int y = 1; y <= board.cols; ++y) {
            if (board.board[x][y] != ObservedBoard::CellState::Hidden)
                continue;
            const CellId cell = board.id(x, y);
            const long double mineProbability = probability.mineProbability(cell, board, basic, structure);
            if (mineProbability > 0.0L && mineProbability < 1.0L) {
                result.cell2Component[cell] = -2;
                ++unknownCount;
            }
        }
    if (unknownCount == 0)
        return result;

    result.cells.reserve(unknownCount);
    result.constraintCells.reserve((std::size_t)(unknownCount) * 8);
    result.constraints.reserve(cellCount);
    result.components.reserve(unknownCount);
    std::vector<char> visitedNumbers(cellCount, 0);
    for (int x = 1; x <= board.rows; ++x)
        for (int y = 1; y <= board.cols; ++y) {
            const CellId start = board.id(x, y);
            if (result.cell2Component[start] != -2)
                continue;
            const ComponentId component = result.components.size();
            const int begin = result.cells.size();
            const int constraintBegin = result.constraints.size();
            result.cell2Component[start] = component;
            result.cells.push_back(start);
            for (int i = begin; i < (int)(result.cells.size()); ++i) {
                const auto [cx, cy] = board.pos(result.cells[i]);
                forEachAdjacent(cx, cy, board.rows, board.cols, [&](int nx, int ny) {
                    if ((int)(board.board[nx][ny]) > (int)(ObservedBoard::CellState::Num8))
                        return;
                    const CellId number = board.id(nx, ny);
                    if (visitedNumbers[number])
                        return;
                    visitedNumbers[number] = 1;
                    const int constraintCellsBegin = result.constraintCells.size();
                    int mines = (int)(board.board[nx][ny]) - basic.mineAround[nx][ny];
                    forEachAdjacent(nx, ny, board.rows, board.cols, [&](int ax, int ay) {
                        const CellId cell = board.id(ax, ay);
                        if (result.cell2Component[cell] == -2) {
                            result.cell2Component[cell] = component;
                            result.cells.push_back(cell);
                        }
                        if (result.cell2Component[cell] == component) {
                            result.constraintCells.push_back(cell);
                            return;
                        }
                        if (board.board[ax][ay] != ObservedBoard::CellState::Hidden || basic.marks[ax][ay] == Basic::Mark::F)
                            return;
                        if (probability.mineProbability(cell, board, basic, structure) == 1.0L)
                            --mines;
                    });
                    result.constraints.push_back(
                        {number, mines, std::span<const CellId>(result.constraintCells).subspan(constraintCellsBegin)});
                });
            }
            result.components.push_back({std::span<const CellId>(result.cells).subspan(begin, result.cells.size() - begin),
                                         std::span<const Result::LogicConstraint>(result.constraints).subspan(
                                             constraintBegin, result.constraints.size() - constraintBegin)});
        }

    return result;
}

} // namespace mss
