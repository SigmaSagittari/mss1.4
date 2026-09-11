#include "algo/observed_board.h"

#include "core/assert.h"

namespace mss {

ObservedBoard::Delta ObservedBoard::update(Result& board, Delta delta) {
    for (Change& change : delta.changes) {
        assert_(change.next != CellState::Hidden,
                "ObservedBoard::update: next state must not be Hidden");
        const auto [x, y] = board.pos(change.cell);
        assert_(board.board[x][y] == CellState::Hidden,
                "ObservedBoard::update: target must be Hidden");
        change.previous = board.board[x][y];
        board.board[x][y] = change.next;
    }
    return delta;
}

void ObservedBoard::applyDelta(Result& board, const Delta& delta, bool reverse) {
    if (reverse) {
        for (std::size_t i = delta.changes.size(); i-- > 0;) {
            const Change& change = delta.changes[i];
            const auto [x, y] = board.pos(change.cell);
            board.board[x][y] = change.previous;
        }
        return;
    }
    for (const Change& change : delta.changes) {
        const auto [x, y] = board.pos(change.cell);
        board.board[x][y] = change.next;
    }
}

}  // namespace mss
