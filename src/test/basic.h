#pragma once

#include <iostream>

#include "core/workspace.h"
#include "test/common.h"

namespace test {

// 覆盖 Basic 的初始传播、矛盾检测和增量 Delta 回放。
inline void basic() {
    mss::Workspace workspace;
    using Mark = mss::Basic::Mark;
    using State = mss::ObservedBoard::CellState;
    mss::ObservedBoard::Result board = mss::ObservedBoard::analyze(3, 3, 1);
    mss::ObservedBoard::Delta boardDelta;
    boardDelta.changes.push_back({board.id(2, 2), State::Num1});
    boardDelta.changes.push_back({board.id(1, 1), State::ForcedMine});
    mss::ObservedBoard::update(board, boardDelta);
    mss::Basic::Result result = mss::Basic::analyze(board, workspace.basic);
    check(result.valid, "deterministic board was marked invalid");
    check(result.marks[1][1] == Mark::F, "forced mine was not marked F");
    check(result.marks[2][2] == Mark::S, "revealed number was not marked S");
    check(result.marks[1][2] == Mark::S && result.marks[2][1] == Mark::S, "deterministic safe cells were not marked S");
    check(result.unknownSum == 0 && result.mineSum == 1 && result.safeCount == 8, "H/T/S/F counts are incorrect");
    mss::ObservedBoard::Result emptyBoard = mss::ObservedBoard::analyze(3, 3, 1);
    mss::Basic::Result emptyResult = mss::Basic::analyze(emptyBoard, workspace.basic);
    mss::Basic::Delta delta;
    mss::Basic::update(emptyResult, delta, board, boardDelta, workspace.basic);
    check(emptyResult.marks[1][1] == Mark::F && emptyResult.marks[1][2] == Mark::S, "incremental propagation disagrees with full analysis");
    check(emptyResult.safeCount == 8 && emptyResult.mineSum == 1, "incremental counts are incorrect");
    mss::Basic::applyDelta(emptyResult, delta);
    check(emptyResult.unknownSum == 9 && emptyResult.mineSum == 0 && emptyResult.safeCount == 0 && emptyResult.valid,
          "default reverse apply did not restore the parent");
    mss::Basic::applyDelta(emptyResult, delta, false);
    check(emptyResult.marks[1][1] == Mark::F && emptyResult.marks[1][2] == Mark::S, "forward apply did not restore the child");
    std::cout << "test/basic: H/T/S/F analysis, update and reverse apply passed\n";
}

} // namespace test
