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

void ObservedBoard::update(Result &board, const Delta &delta) {
    for (const Change &change : delta.changes) {
        // CellId 就是 Grid 的线性下标（契约），这里直接线性访问，不需要 pos()。
        CellState &target = board.board.data()[change.cell];
        // 【唯一合法迁移】Hidden -> 非 Hidden。触发条件与 1.4 完全一致
        // （next != Hidden 且 target == Hidden 的否定），只是信息更明确。
        assert_(target == CellState::Hidden && change.next != CellState::Hidden,
                "ObservedBoard::update: 非法迁移；只允许 Hidden -> 非 Hidden，撤销请用 reverseDelta");
        target = change.next;
    }
}

void ObservedBoard::applyDelta(Result &board, const Delta &delta) {
    for (const Change &change : delta.changes)
        board.board.data()[change.cell] = change.next;
}

void ObservedBoard::reverseDelta(Result &board, const Delta &delta) {
    // 从后往前：Delta 的逆序撤销语义（多格时顺序是契约的一部分）。
    for (int i = static_cast<int>(delta.changes.size()); i-- > 0;)
        board.board.data()[delta.changes[i].cell] = CellState::Hidden;
}

} // namespace mss