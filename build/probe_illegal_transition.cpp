#include "board/observed_board.h"

int main() {
    mss::ObservedBoard::Result board = mss::ObservedBoard::analyze(3, 3, 1);
    mss::ObservedBoard::Delta first;
    first.changes.push_back({board.id(1, 1), mss::ObservedBoard::CellState::Num3, mss::ObservedBoard::CellState::Hidden});
    mss::ObservedBoard::update(board, first);
    mss::ObservedBoard::Delta again;  // 同一格重复 update —— 1.4 的崩溃形态
    again.changes.push_back({board.id(1, 1), mss::ObservedBoard::CellState::Num3, mss::ObservedBoard::CellState::Hidden});
    mss::ObservedBoard::update(board, again);
    return 0;  // 不应到达
}