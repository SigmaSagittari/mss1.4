#include "test/structure.h"

#include <iostream>

#include "algo/structure.h"
#include "test/common.h"

namespace test {

void structure() {
    using State = mss::ObservedBoard::CellState;

    auto board = mss::ObservedBoard::analyze(3, 3, 1);
    mss::ObservedBoard::Delta boardDelta;
    boardDelta.changes.push_back({board.id(2, 2), State::Num1});
    boardDelta = mss::ObservedBoard::update(board, std::move(boardDelta));
    const auto basic = mss::Basic::analyze(board);

    mss::Structure::ShapePool pool;
    auto result = mss::Structure::analyze(board, basic, pool);
    check(result.components.size() == 1, "one frontier component was not built");
    check(result.components[0].boxes.count() == 1,
          "identical frontier neighborhoods were not merged");
    check(result.components[0].boxes.cellCount(0) == 8,
          "frontier box size is incorrect");
    check(result.components[0].constraintCells.size() == 1,
          "number constraint was not recorded");
    check(pool.size() == 1, "shape was not interned");
    check(pool.get(result.components[0].shape).constraintCount() == 1,
          "interned shape is incomplete");

    const auto second = mss::Structure::analyze(board, basic, pool);
    check(second.components[0].shape == result.components[0].shape,
          "identical shapes were not deduplicated");
    check(pool.size() == 1, "duplicate shape expanded the pool");

    const auto parent = result;
    auto childBoard = board;
    mss::ObservedBoard::Delta childBoardDelta;
    childBoardDelta.changes.push_back({childBoard.id(1, 1), State::Num0});
    childBoardDelta =
        mss::ObservedBoard::update(childBoard, std::move(childBoardDelta));
    auto childBasic = basic;
    mss::Basic::Delta childBasicDelta;
    childBasicDelta = mss::Basic::update(
        childBoard, childBasic, childBoardDelta, std::move(childBasicDelta));
    const auto structureDelta = mss::Structure::update(
        childBoard, childBasic, result, pool, childBoardDelta);
    check(result.components.size() == 1 &&
              result.components[0].boxes.cellCount(0) == 5,
          "incremental component rebuild is incorrect");

    mss::Structure::applyDelta(result, structureDelta);
    check(result.components.size() == parent.components.size() &&
              result.components[0].shape == parent.components[0].shape &&
              result.components[0].boxes.cellCount(0) == 8,
          "default reverse did not restore the component");
    mss::Structure::applyDelta(result, structureDelta, false);
    check(result.components[0].boxes.cellCount(0) == 5,
          "forward replay did not restore the child component");

    std::cout << "test/structure: ShapePool, full analysis and delta smoke passed\n";
}

}  // namespace test
