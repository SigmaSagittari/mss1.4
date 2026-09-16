#pragma once

#include <cstdint>
#include <iostream>

#include "algo/observed_board.h"
#include "test/common.h"

namespace test {

// 覆盖坐标编码以及 ObservedBoard Delta 的正向/逆向应用。
inline void observedBoard() {
    using State = mss::ObservedBoard::CellState;
    static_assert(sizeof(State) == sizeof(std::uint8_t));
    mss::ObservedBoard::Result board = mss::ObservedBoard::analyze(3, 3, 1);
    mss::ObservedBoard::Delta delta;
    delta.changes.push_back({board.id(1, 1), State::Num1});
    delta.changes.push_back({board.id(1, 2), State::ForcedMine});
    const std::size_t capacity = delta.changes.capacity();
    mss::ObservedBoard::update(board, delta);
    check(board.board[1][1] == State::Num1, "revealed state was not applied");
    check(board.board[1][2] == State::ForcedMine, "forced state was not applied");
    check(delta.changes[0].previous == State::Hidden, "previous state was not recorded");
    mss::ObservedBoard::applyDelta(board, delta);
    check(board.board[1][1] == State::Hidden && board.board[1][2] == State::Hidden, "reverse apply did not restore Hidden");
    mss::ObservedBoard::applyDelta(board, delta, false);
    check(board.board[1][1] == State::Num1 && board.board[1][2] == State::ForcedMine, "forward apply did not restore next states");
    delta.clear();
    check(delta.changes.capacity() == capacity, "Delta capacity was discarded by clear");
    delta.changes.push_back({board.id(2, 2), State::Num0});
    mss::ObservedBoard::update(board, delta);
    check(board.board[2][2] == State::Num0, "reused Delta was not applied");
    std::cout << "test/observed_board: packed state, external Delta and reuse passed\n";
}

} // namespace test
