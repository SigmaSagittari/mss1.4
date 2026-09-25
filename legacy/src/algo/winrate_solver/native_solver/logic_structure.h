#pragma once

#include <span>
#include <vector>

#include "algo/probability_engine/basic.h"
#include "algo/probability_engine/observed_board.h"
#include "algo/probability_engine/structure.h"
#include "core/types.h"

namespace mss {

struct LogicStructure {

    struct Result {
        // LogicComponent 是相邻 Hidden H/T 候选构成的信息连通块：点开其中一个
        // 翻出的数字可能包含相邻 Hidden 格的雷。共享数字约束已由 Structure 合并，
        // 这里只需对 H/T 补 8 邻接闭包，并把 H 所属的 Structure 实例整块并入。
        struct LogicComponent {
            // 本块包含的前沿 Structure 实例；切片引用 Result::structComponents 中的元素。
            std::span<const Structure::Instance> structComponents;
            // 本块包含的非前沿 T 格 CellID；切片引用 Result::offFrontierCells 中的元素。
            std::span<const CellId> offFrontierCells;
        };

        Result() = default;
        Result(const Result &) = delete;
        Result &operator=(const Result &) = delete;
        Result(Result &&) noexcept = default;
        Result &operator=(Result &&) noexcept = default;

        std::vector<Structure::Instance> structComponents; // 所有逻辑块的前沿组件连续存储区。
        std::vector<CellId> offFrontierCells; // 所有逻辑块的非前沿格连续存储区。
        std::vector<LogicComponent> components; // 当前盘面的逻辑连通块。
    };

    // Structure 已按共享数字把 H 连成实例。LogicStructure 在其上补 Hidden 候选的
    // 8 邻接闭包：T-T、T-H、H-H 都合并；T 格邻到 H 格时把 H 所属实例并入。
    static Result analyze(const ObservedBoard::Result &board, const Basic::Result &basic, const Structure::Result &structure,
                          const Structure::Pool &shapes);
};

} // namespace mss

//==============================================================================
namespace mss {

inline LogicStructure::Result LogicStructure::analyze(const ObservedBoard::Result &board, const Basic::Result &basic,
                                                       const Structure::Result &structure, const Structure::Pool &shapes) {
    Result result;
    std::vector<char> visitedCells((board.rows + 1) * (board.cols + 1), 0), visitedComponents(structure.components.size(), 0);
    std::size_t candidateCount = basic.unknownSum;
    for (InstanceId instance : structure.components)
        candidateCount += shapes.getInstance(instance).boxes.cells.size;
    std::vector<CellId> cells;
    cells.reserve(candidateCount);
    result.structComponents.reserve(structure.components.size());
    result.offFrontierCells.reserve(basic.unknownSum);
    result.components.reserve(candidateCount);
    auto addCell = [&](CellId cell) {
        if (visitedCells[cell])
            return;
        visitedCells[cell] = 1;
        cells.push_back(cell);
    };
    for (int x = 1; x <= board.rows; ++x)
        for (int y = 1; y <= board.cols; ++y) {
            const CellId start = board.id(x, y);
            if (board.board[x][y] != ObservedBoard::CellState::Hidden ||
                (basic.marks[x][y] != Basic::Mark::H && basic.marks[x][y] != Basic::Mark::Unknown) || visitedCells[start])
                continue;
            const std::size_t structBegin = result.structComponents.size();
            const std::size_t offFrontierBegin = result.offFrontierCells.size();
            cells.clear();
            addCell(start);
            for (int i = 0; i < (int)(cells.size()); ++i) {
                const CellId cell = cells[i];
                const auto [cx, cy] = board.pos(cell);
                const Basic::Mark mark = basic.marks[cx][cy];
                if (mark == Basic::Mark::H) {
                    const ComponentId component = structure.cellLoc[cell].component;
                    if (!visitedComponents[component]) {
                        visitedComponents[component] = 1;
                        const Structure::Instance &instance = shapes.getInstance(structure.components[component]);
                        result.structComponents.push_back(instance);
                        for (CellId member : instance.boxes.cells.span(shapes.cells))
                            addCell(member);
                    }
                } else
                    result.offFrontierCells.push_back(cell);
                forEachAdjacent(cx, cy, board.rows, board.cols, [&](int nx, int ny) {
                    if (board.board[nx][ny] != ObservedBoard::CellState::Hidden)
                        return;
                    // H/T 都是未确定候选；只要直接相邻，点开其中一个翻出的数字就可能
                    // 包含另一个的雷，因此 T-T、T-H、H-H 都必须并入同一个信息连通块。
                    if (basic.marks[nx][ny] == Basic::Mark::H || basic.marks[nx][ny] == Basic::Mark::Unknown)
                        addCell(board.id(nx, ny));
                });
            }
            result.components.push_back({
                std::span<const Structure::Instance>(result.structComponents).subspan(
                    structBegin, result.structComponents.size() - structBegin),
                std::span<const CellId>(result.offFrontierCells).subspan(
                    offFrontierBegin, result.offFrontierCells.size() - offFrontierBegin)});
        }
    return result;
}

} // namespace mss
