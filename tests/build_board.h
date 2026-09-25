#pragma once

#include <initializer_list>
#include <tuple>
#include <utility>

#include "board/observed_board.h"

namespace test {

// 构造观测盘面：先建全 Hidden，再按列表翻开数字或施加断言。
inline mss::ObservedBoard::Result makeBoard(
    int rows, int cols, int mines,
    std::initializer_list<std::tuple<int, int, mss::ObservedBoard::CellState>> cells) {
    mss::ObservedBoard::Result board = mss::ObservedBoard::analyze(rows, cols, mines);
    mss::ObservedBoard::Delta delta;
    for (const std::tuple<int, int, mss::ObservedBoard::CellState> &cell : cells)
        delta.changes.push_back({board.id(std::get<0>(cell), std::get<1>(cell)), std::get<2>(cell)});
    mss::ObservedBoard::update(board, delta);
    return board;
}

} // namespace test