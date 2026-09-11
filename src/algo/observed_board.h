#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "core/types.h"

namespace mss {

struct ObservedBoard {
    enum class CellState : std::uint8_t {
        Num0 = 0,
        Num1,
        Num2,
        Num3,
        Num4,
        Num5,
        Num6,
        Num7,
        Num8,
        Hidden,
        ForcedMine,
        ForcedSafe,
    };

    struct Result {
        int rows = 0;
        int cols = 0;
        int totalMines = 0;
        Grid<CellState> board;

        Result() = default;
        Result(int rows, int cols, int mines)
            : rows(rows), cols(cols), totalMines(mines), board(rows, cols, CellState::Hidden) {}

        CellId id(int x, int y) const { return x * (cols + 1) + y; }
        std::pair<int, int> pos(CellId cell) const { return {cell / (cols + 1), cell % (cols + 1)}; }
    };

    struct Change {
        CellId cell = -1;
        CellState next = CellState::Hidden;
        CellState previous = CellState::Hidden;
    };

    struct Delta {
        std::vector<Change> changes;
        void clear() { changes.clear(); }
    };

    static Result analyze(int rows, int cols, int mines) { return Result(rows, cols, mines); }
    static Delta update(Result& board, Delta delta);
    static void applyDelta(Result& board, const Delta& delta, bool reverse = true);
};

}  // namespace mss
