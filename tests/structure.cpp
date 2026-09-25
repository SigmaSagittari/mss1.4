// Structure 契约的可执行形式：Box 压缩、约束、cellLoc、增量 == 全量重建、Delta 双向回放、interning。
#include <algorithm>
#include <array>
#include <cstddef>
#include <vector>

#include "basic/basic.h"
#include "build_board.h"
#include "harness.h"
#include "structure/structure.h"
#include "workspace.h"

namespace test {

namespace {

using Cell = mss::ObservedBoard::CellId;
using State = mss::ObservedBoard::CellState;
using Structure = mss::Structure;

Structure::ShapeId shapeOf(const Structure::Pool &pool, const Structure::Result &result, int component) {
    return pool.instanceShape(result.components[static_cast<std::size_t>(component)]);
}

// ───────────────────────── 基础正确性 ─────────────────────────

void testEmptyBoard() {
    mss::Workspace workspace;
    Structure::Pool pool;
    const mss::ObservedBoard::Result board = mss::ObservedBoard::analyze(3, 3, 2); // 全 Hidden：没有数字
    const mss::Basic::Result basic = mss::Basic::analyze(board, workspace.basic);
    const Structure::Result result = Structure::analyze(board, basic, pool, workspace.structure);
    check(result.components.empty(), "没有数字就没有组件");
    check(pool.shapeCount() == 0, "池里不应该有 Shape");
    for (Cell cell = 0; cell < 9; ++cell)
        check(result.cellLoc[static_cast<std::size_t>(cell)].component == -1, "无组件的格子 cellLoc = {-1,-1}");
}

void testSingleBox() {
    // 3x3、2 雷，翻开 (0,0)=1：三个邻居都只邻接这一个数字 → 同签名 → 一个 Box。
    mss::Workspace workspace;
    Structure::Pool pool;
    const mss::ObservedBoard::Result board = makeBoard(3, 3, 2, {{0, 0, State::Num1}});
    const mss::Basic::Result basic = mss::Basic::analyze(board, workspace.basic);
    const Structure::Result result = Structure::analyze(board, basic, pool, workspace.structure);

    check(result.components.size() == 1, "应该恰好一个组件");
    const Structure::ShapeId shape = shapeOf(pool, result, 0);
    check(pool.shapeBoxCount(shape) == 1, "三个同签名格应该压成一个 Box");
    check(pool.shapeBoxSize(shape, 0) == 3, "Box 尺寸 = 3");
    check(pool.shapeConstraintCount(shape) == 1, "一个数字一条约束");
    const Structure::Shape::ConstraintView constraint = pool.shapeConstraint(shape, 0);
    check(constraint.sum == 1, "sum = 数字值 - 邻域已知雷数");
    check(constraint.boxIds.size() == 1, "约束引用 1 个 Box");
    check(constraint.boxIds[0] == 0, "引用的就是那个 Box");

    const Structure::InstanceId instance = result.components[0];
    check(pool.instanceBoxCount(instance) == 1, "实例 1 个 Box");
    check(pool.instanceBoxCellCount(instance, 0) == 3, "实例 Box 里 3 个格子");
    check(pool.instanceConstraintCell(instance, 0) == board.id(0, 0), "约束对应数字格 (0,0)");

    // Box 内格子集合（与顺序无关地比较）
    std::vector<Cell> cells;
    for (int i = 0; i < pool.instanceBoxCellCount(instance, 0); ++i)
        cells.push_back(pool.instanceBoxCell(instance, 0, i));
    std::sort(cells.begin(), cells.end());
    const std::vector<Cell> expected{board.id(0, 1), board.id(1, 0), board.id(1, 1)};
    check(cells == expected, "Box 内就是那三个候选格");

    for (Cell cell : cells)
        check(result.cellLoc[static_cast<std::size_t>(cell)] == Structure::CellLocation{0, 0}, "候选格 cellLoc = {组件 0, Box 0}");
    check(result.cellLoc[static_cast<std::size_t>(board.id(0, 0))] == Structure::CellLocation{0, -1}, "数字格 cellLoc = {组件, -1}");
    check(result.cellLoc[static_cast<std::size_t>(board.id(2, 2))].component == -1, "T 格不参与组件");
}

void testBoxSizeEight() {
    // 3x3、2 雷，翻开中心 = 2：八个邻居签名全相同 → 一个尺寸 8 的 Box（上限用例）。
    mss::Workspace workspace;
    Structure::Pool pool;
    const mss::ObservedBoard::Result board = makeBoard(3, 3, 2, {{1, 1, State::Num2}});
    const mss::Basic::Result basic = mss::Basic::analyze(board, workspace.basic);
    const Structure::Result result = Structure::analyze(board, basic, pool, workspace.structure);

    check(result.components.size() == 1, "一个组件");
    const Structure::ShapeId shape = shapeOf(pool, result, 0);
    check(pool.shapeBoxCount(shape) == 1 && pool.shapeBoxSize(shape, 0) == 8, "八个同签名格压成一个尺寸 8 的 Box");
    const Structure::Shape::ConstraintView center = pool.shapeConstraint(shape, 0);
    check(center.sum == 2 && center.boxIds.size() == 1, "约束 sum=2、引用 1 个 Box");
}

void testTwoNumbersThreeBoxes() {
    // 3x3、2 雷，翻开 (0,0)=1 与 (2,2)=1：
    //   A = 只邻接 (0,0) 的 2 格；B = 同时邻接两个数字的 1 格 (1,1)；C = 只邻接 (2,2) 的 2 格。
    mss::Workspace workspace;
    Structure::Pool pool;
    const mss::ObservedBoard::Result board = makeBoard(3, 3, 2, {{0, 0, State::Num1}, {2, 2, State::Num1}});
    const mss::Basic::Result basic = mss::Basic::analyze(board, workspace.basic);
    const Structure::Result result = Structure::analyze(board, basic, pool, workspace.structure);

    check(result.components.size() == 1, "两个数字通过共同的候选格连成一个组件");
    const Structure::ShapeId shape = shapeOf(pool, result, 0);
    check(pool.shapeBoxCount(shape) == 3, "三种签名 → 三个 Box");

    std::vector<int> sizes;
    for (Structure::BoxId box = 0; box < pool.shapeBoxCount(shape); ++box)
        sizes.push_back(pool.shapeBoxSize(shape, box));
    std::sort(sizes.begin(), sizes.end());
    check(sizes == std::vector<int>({1, 2, 2}), "Box 尺寸多重集 = {1,2,2}");
    check(pool.shapeConstraintCount(shape) == 2, "两个数字两条约束");

    for (std::size_t i = 0; i < pool.shapeConstraintCount(shape); ++i) {
        const Structure::Shape::ConstraintView constraint = pool.shapeConstraint(shape, i);
        check(constraint.sum == 1, "两条约束 sum 都是 1");
        check(constraint.boxIds.size() == 2, "每个数字都邻接两个 Box");
        int total = 0;
        for (Structure::BoxId box : constraint.boxIds)
            total += pool.shapeBoxSize(shape, box);
        check(total == 3, "约束引用的 Box 一共覆盖 3 个候选格");
    }

    const Structure::BoxId middle = result.cellLoc[static_cast<std::size_t>(board.id(1, 1))].box;
    check(middle >= 0, "(1,1) 属于某个 Box");
    for (std::size_t i = 0; i < pool.shapeConstraintCount(shape); ++i) {
        bool referenced = false;
        for (Structure::BoxId box : pool.shapeConstraint(shape, i).boxIds)
            if (box == middle)
                referenced = true;
        check(referenced, "(1,1) 所在的 Box 必须被两条约束同时引用");
    }
}

// ───────────────────────── 增量 vs 全量 ─────────────────────────

constexpr int kRows = 4;
constexpr int kCols = 4;
constexpr char kLayout[kRows][kCols] = {
    {'.', '.', '.', '.'},
    {'.', '*', '.', '.'},
    {'.', '.', '*', '.'},
    {'.', '.', '.', '.'},
};

bool isMine(int x, int y) {
    return kLayout[x][y] == '*';
}

int digitAt(int x, int y) {
    int mines = 0;
    for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy) {
            const int nx = x + dx;
            const int ny = y + dy;
            if ((dx != 0 || dy != 0) && nx >= 0 && nx < kRows && ny >= 0 && ny < kCols && isMine(nx, ny))
                ++mines;
        }
    return mines;
}

// 规范化：组件按"最小格子"排序，并把 cellLoc 里的组件下标重新编号 ——
// 这样比较就与"组件在数组里的顺序"无关（增量重建的追加顺序和全量扫描不同）。
Structure::Result canonicalize(const Structure::Result &result, const Structure::Pool &pool) {
    const std::size_t count = result.components.size();
    std::vector<Cell> keys(count, -1);
    for (std::size_t i = 0; i < count; ++i) {
        const Structure::InstanceId instance = result.components[i];
        Cell best = -1;
        const int boxCount = pool.instanceBoxCount(instance);
        for (Structure::BoxId box = 0; box < boxCount; ++box) {
            const int cellCount = pool.instanceBoxCellCount(instance, box);
            for (int k = 0; k < cellCount; ++k) {
                const Cell cell = pool.instanceBoxCell(instance, box, k);
                if (best == -1 || cell < best)
                    best = cell;
            }
        }
        const std::size_t constraintCount = pool.instanceConstraintCellCount(instance);
        for (std::size_t k = 0; k < constraintCount; ++k) {
            const Cell cell = pool.instanceConstraintCell(instance, k);
            if (best == -1 || cell < best)
                best = cell;
        }
        keys[i] = best;
    }
    std::vector<std::size_t> order(count);
    for (std::size_t i = 0; i < count; ++i)
        order[i] = i;
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return keys[a] < keys[b]; });

    std::vector<Structure::ComponentId> remap(count, -1);
    Structure::Result out;
    out.components.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        remap[order[i]] = static_cast<Structure::ComponentId>(i);
        out.components.push_back(result.components[order[i]]);
    }
    out.cellLoc = result.cellLoc;
    for (Structure::CellLocation &location : out.cellLoc)
        if (location.component >= 0)
            location.component = remap[static_cast<std::size_t>(location.component)];
    return out;
}

void testIncrementalMatchesRebuild() {
    mss::ObservedBoard::Result board = mss::ObservedBoard::analyze(kRows, kCols, 2);
    mss::Workspace live;
    mss::Workspace rebuilt;
    Structure::Pool pool;
    mss::Basic::Result basic = mss::Basic::analyze(board, live.basic);
    Structure::Result structure = Structure::analyze(board, basic, pool, live.structure);

    int reveals = 0;
    for (int x = 0; x < kRows; ++x)
        for (int y = 0; y < kCols; ++y) {
            if (isMine(x, y))
                continue;
            mss::ObservedBoard::Delta boardDelta;
            boardDelta.changes.push_back({board.id(x, y), static_cast<State>(digitAt(x, y))});
            mss::ObservedBoard::update(board, boardDelta);
            mss::Basic::Delta basicDelta;
            mss::Basic::update(basic, basicDelta, board, boardDelta, live.basic);
            Structure::Delta structureDelta;
            Structure::update(structure, structureDelta, board, basic, pool, boardDelta, live.structure);
            ++reveals;

            const mss::Basic::Result basicRebuilt = mss::Basic::analyze(board, rebuilt.basic);
            const Structure::Result structureRebuilt = Structure::analyze(board, basicRebuilt, pool, rebuilt.structure);
            check(structure.components.size() == structureRebuilt.components.size(), "组件数必须一致");
            check(canonicalize(structure, pool).sameAs(canonicalize(structureRebuilt, pool)),
                  "增量结构必须与全量重建一致（组件集合 + Box 分组 + cellLoc）");
        }
    check(reveals == kRows * kCols - 2, "翻开的非雷格数量");
}

void testDeltaRoundTrip() {
    mss::ObservedBoard::Result board = mss::ObservedBoard::analyze(kRows, kCols, 2);
    mss::Workspace workspace;
    Structure::Pool pool;
    mss::Basic::Result basic = mss::Basic::analyze(board, workspace.basic);
    Structure::Result structure = Structure::analyze(board, basic, pool, workspace.structure);
    const Structure::Result initial = structure;

    constexpr int kReveal[8][2] = {{0, 0}, {0, 1}, {0, 2}, {0, 3}, {1, 0}, {1, 2}, {1, 3}, {2, 1}};
    std::vector<Structure::Delta> deltas;
    for (const auto &cell : kReveal) {
        mss::ObservedBoard::Delta boardDelta;
        boardDelta.changes.push_back({board.id(cell[0], cell[1]), static_cast<State>(digitAt(cell[0], cell[1]))});
        mss::ObservedBoard::update(board, boardDelta);
        mss::Basic::Delta basicDelta;
        mss::Basic::update(basic, basicDelta, board, boardDelta, workspace.basic);
        Structure::Delta structureDelta;
        Structure::update(structure, structureDelta, board, basic, pool, boardDelta, workspace.structure);
        deltas.push_back(std::move(structureDelta));
    }
    const Structure::Result finalState = structure;

    for (std::size_t i = deltas.size(); i-- > 0;)
        Structure::reverseDelta(structure, pool, deltas[i]);
    check(structure.sameAs(initial), "全部反向回放必须回到初始状态");

    for (const Structure::Delta &delta : deltas)
        Structure::applyDelta(structure, pool, delta);
    check(structure.sameAs(finalState), "正向重放必须回到最终状态");
}

void testInternDedup() {
    mss::Workspace workspace;
    Structure::Pool pool;
    const mss::ObservedBoard::Result board = makeBoard(3, 3, 2, {{0, 0, State::Num1}});
    const mss::Basic::Result basic = mss::Basic::analyze(board, workspace.basic);
    const Structure::Result first = Structure::analyze(board, basic, pool, workspace.structure);
    const std::size_t shapesAfterFirst = pool.shapeCount();
    const Structure::Result second = Structure::analyze(board, basic, pool, workspace.structure);
    check(pool.shapeCount() == shapesAfterFirst, "同一盘面二次 analyze 不应新增 Shape（interning 命中）");
    check(first.sameAs(second), "同一盘面两次 analyze 必须逐字段一致");
}

} // namespace

void structure() {
    testEmptyBoard();
    testSingleBox();
    testBoxSizeEight();
    testTwoNumbersThreeBoxes();
    testIncrementalMatchesRebuild();
    testDeltaRoundTrip();
    testInternDedup();
}

} // namespace test