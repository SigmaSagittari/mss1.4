#include "board/observed_board.h"

namespace mss {

ObservedBoard::Result::Result() = default;

ObservedBoard::Result::Result(int rows, int cols, int mines)
    : rows(rows), cols(cols), totalMines(mines), board(rows, cols, CellState::Hidden) {
}

ObservedBoard::CellId ObservedBoard::Result::id(int x, int y) const {
    return x * cols + y;
}

std::pair<int, int> ObservedBoard::Result::pos(CellId cell) const {
    return {cell / cols, cell % cols};
}

void ObservedBoard::Delta::clear() {
    changes.clear();
}

ObservedBoard::Result ObservedBoard::analyze(int rows, int cols, int mines) {
    return Result(rows, cols, mines);
}

void ObservedBoard::update(Result &board, Delta &delta) {
    for (Change &change : delta.changes) {
        const auto [x, y] = board.pos(change.cell);
        const CellState from = board.board[x][y];
        // 触发条件与 1.4 完全一致（next != Hidden 且 target == Hidden 的否定），
        // 只是把两条断言合成一条、并给出更明确的补救提示。
        assert_(isLegalTransition(from, change.next),
                "ObservedBoard::update: 非法迁移；只允许 Hidden -> 非 Hidden，撤销请用 applyDelta(reverse=true)");
        change.previous = from;
        board.board[x][y] = change.next;
    }
}

void ObservedBoard::applyDelta(Result &board, const Delta &delta, bool reverse) {
    if (reverse) {
        for (int i = static_cast<int>(delta.changes.size()); i-- > 0;) {
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