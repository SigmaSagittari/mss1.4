#pragma once

#include <iostream>

#include "algo/structure.h"
#include "test/common.h"

namespace test {

// 覆盖组件发现、Box/constraint 构造和结构结果的基本不变量。
inline void structure() {
    using State = mss::ObservedBoard::CellState;
    mss::ObservedBoard::Result board = mss::ObservedBoard::analyze(3, 3, 1);
    mss::ObservedBoard::Delta boardDelta;
    boardDelta.changes.push_back({board.id(2, 2), State::Num1});
    mss::ObservedBoard::update(board, boardDelta);
    const mss::Basic::Result basic = mss::Basic::analyze(board);
    mss::Structure::Pool pool;
    mss::Structure::Result result = mss::Structure::analyze(board, basic, pool);
    check(result.components.size() == 1, "one frontier component was not built");
    const mss::Structure::Instance& firstInstance =
        pool.getInstance(result.components[0]);
    check(firstInstance.boxes.count() == 1,
          "identical frontier neighborhoods were not merged");
    check(firstInstance.boxes.cellCount(0) == 8,
          "frontier box size is incorrect");
    check(firstInstance.constraintCells.size() == 1,
          "number constraint was not recorded");
    check(pool.size() == 1, "shape was not interned");
    check(pool.getShape(pool.getInstance(result.components[0]).shape)
              .constraintCount() == 1,
          "interned shape is incomplete");
    const mss::Structure::Result second = mss::Structure::analyze(board, basic, pool);
    check(pool.getInstance(second.components[0]).shape == firstInstance.shape,
          "identical shapes were not deduplicated");
    check(second.components[0] == result.components[0],
          "identical instance data was not deduplicated");
    check(pool.size() == 1, "duplicate shape expanded the pool");
    const mss::Structure::Result parent = result;
    mss::ObservedBoard::Result childBoard = board;
    mss::ObservedBoard::Delta childBoardDelta;
    childBoardDelta.changes.push_back({childBoard.id(1, 1), State::Num0});
    mss::ObservedBoard::update(childBoard, childBoardDelta);
    mss::Basic::Result childBasic = basic;
    mss::Basic::Delta childBasicDelta;
    mss::Basic::update(childBasic, childBasicDelta, childBoard, childBoardDelta);
    mss::Structure::Delta structureDelta;
    mss::Structure::update(result, structureDelta, childBoard, childBasic, pool,
                           childBoardDelta);
    check(result.components.size() == 1 &&
              pool.getInstance(result.components[0]).boxes.cellCount(0) == 5,
          "incremental component rebuild is incorrect");
    mss::Structure::applyDelta(result, pool, structureDelta);
    check(result.components.size() == parent.components.size() &&
              result.components[0] == parent.components[0] &&
              pool.getInstance(result.components[0]).boxes.cellCount(0) == 8,
          "default reverse did not restore the component");
    mss::Structure::applyDelta(result, pool, structureDelta, false);
    check(pool.getInstance(result.components[0]).boxes.cellCount(0) == 5,
          "forward replay did not restore the child component");
    std::cout << "test/structure: Pool, full analysis and delta smoke passed\n";
}

}  // namespace test
