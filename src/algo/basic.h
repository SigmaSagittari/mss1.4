#pragma once

#include <cstdint>
#include <vector>

#include "algo/observed_board.h"
#include "core/types.h"

namespace mss {

struct Basic {
    // H: 前沿，T: 非前沿，S: 安全，F: 危险。
    enum class Mark : std::uint8_t {
        H = 0,
        T = 1,
        S = 2,
        F = 3,

        Frontier = H,
        Unknown = T,
        Safe = S,
        Mine = F,
    };

    struct Result {
        int rows = 0;
        int cols = 0;
        Grid<Mark> marks;
        int unknownSum = 0;
        int mineSum = 0;
        int safeCount = 0;
        bool valid = true;

        Grid<std::int8_t> mineAround;
        Grid<std::int8_t> hideAround;
    };

    struct Delta {
        struct Change {
            CellId cell = -1;
            Mark old = Mark::T;
            Mark now = Mark::T;
        };

        std::vector<Change> changes;
        int unknownSum = 0;
        int mineSum = 0;
        int safeCount = 0;
        bool valid = true;
        int oldUnknownSum = 0;
        int oldMineSum = 0;
        int oldSafeCount = 0;
        bool oldValid = true;
    };

    static Result analyze(const ObservedBoard::Result& board);
    static Delta update(const ObservedBoard::Result& board, Result& result,
                        const ObservedBoard::Delta& updates, Delta delta);
    static void applyDelta(Result& result, const Delta& delta, bool reverse = true);
};

}  // namespace mss
