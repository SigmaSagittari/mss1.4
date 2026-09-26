#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "core/assert.h"
#include "core/utility/grid.h"

namespace mss {

struct ObservedBoard {
    using CellId = int;

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

        Result();
        Result(int rows, int cols, int mines);

        ObservedBoard::CellId id(int x, int y) const;
        std::pair<int, int> pos(ObservedBoard::CellId cell) const;
    };

    struct Change {
        ObservedBoard::CellId cell = -1;
        CellState next = CellState::Hidden;
        CellState previous = CellState::Hidden;
    };

    struct Delta {
        std::vector<Change> changes;
        void clear();
    };

    // analyze 只创建全 Hidden 的分析视图，不推断任何数字或雷位。

    static Result analyze(int rows, int cols, int mines);
    static void update(Result &board, Delta &delta);
    // update 会回写每个 Change::previous；同一 Delta 不能重复作为“新更新”提交。
    // reverse=true 的 applyDelta 必须在该 Delta 对应的子状态上调用，且按逆序撤销。
    static void applyDelta(Result &board, const Delta &delta, bool reverse = true);
};

//==============================================================================
// 创建空的观测结果，供后续赋值或移动构造使用。
inline ObservedBoard::Result::Result() = default;

// 创建指定尺寸和雷数的全 Hidden 观测盘面。
inline ObservedBoard::Result::Result(int rows, int cols, int mines)
    : rows(rows), cols(cols), totalMines(mines), board(rows, cols, CellState::Hidden) {
}

// 将 1-based 坐标编码成与 Grid 存储一致的 ObservedBoard::CellId。
inline ObservedBoard::CellId ObservedBoard::Result::id(int x, int y) const {
    return x * (cols + 1) + y;
}

// 将 ObservedBoard::CellId 解码回 1-based 坐标。
inline std::pair<int, int> ObservedBoard::Result::pos(ObservedBoard::CellId cell) const {
    return {cell / (cols + 1), cell % (cols + 1)};
}

// 清空本次观测变化记录。
inline void ObservedBoard::Delta::clear() {
    changes.clear();
}

// 创建全 Hidden 的初始观测结果。
inline ObservedBoard::Result ObservedBoard::analyze(int rows, int cols, int mines) {
    return Result(rows, cols, mines);
}

// 按 Delta 将 Hidden 格子更新为数字或强制状态，并回写旧状态；这是生产分析
// 管线的第一步，后续 Basic/Structure 必须使用同一批 updates。
inline void ObservedBoard::update(Result &board, Delta &delta) {
    for (Change &change : delta.changes) {
        assert_(change.next != CellState::Hidden, "ObservedBoard::update: next state must not be Hidden");
        const auto [x, y] = board.pos(change.cell);
        assert_(board.board[x][y] == CellState::Hidden, "ObservedBoard::update: target must be Hidden");
        change.previous = board.board[x][y];
        board.board[x][y] = change.next;
    }
}

// 将观测 Delta 正向应用或按逆序恢复到指定观测结果；reverse=true 用于搜索/测试
// 返回父状态，且必须从 changes 的末尾开始撤销。
inline void ObservedBoard::applyDelta(Result &board, const Delta &delta, bool reverse) {
    if (reverse) {
        for (int i = delta.changes.size(); i-- > 0;) {
            const Change &change = delta.changes[i];
            const auto [x, y] = board.pos(change.cell);
            board.board[x][y] = change.previous;
        }
        return;
    }
    for (const Change &change : delta.changes) {
        const auto [x, y] = board.pos(change.cell);
        board.board[x][y] = change.next;
    }
}

} // namespace mss
