// Basic 契约的可执行形式：初始标记、传播到不动点、valid 的四个来源、
// 增量与全量重建一致、Delta 往返。失败即终止（见 harness.h）。
#include <array>
#include <cstddef>
#include <initializer_list>
#include <tuple>
#include <utility>
#include <vector>

#include "basic/basic.h"
#include "build_board.h"
#include "workspace.h"
#include "harness.h"

namespace test {

namespace {

using State = mss::ObservedBoard::CellState;
using Mark = mss::Basic::Mark;



void testInitialMarks() {
    mss::Workspace workspace;
    const mss::ObservedBoard::Result board = makeBoard(3, 3, 1, {{0, 0, State::Num1}});
    const mss::Basic::Result basic = mss::Basic::analyze(board, workspace.basic);
    check(basic.valid, "单个数字的盘面必须有效");
    check(basic.marks[0][0] == Mark::S, "已翻开的数字格标记为 S（非候选填充值）");
    check(basic.marks[0][1] == Mark::H && basic.marks[1][0] == Mark::H && basic.marks[1][1] == Mark::H,
          "邻接数字的隐藏格必须是 H");
    check(basic.marks[2][2] == Mark::T, "不邻接数字的隐藏格必须是 T");
    check(basic.unknownSum == 5, "unknownSum = #T（不含 H）");
    check(basic.mineSum == 0, "mineSum = #F");
    check(basic.safeHideCount == 0, "safeHideCount 不含已翻开的数字格");
}

void testPropagation() {
    mss::Workspace workspace;
    // 2x3、1 雷（在 (1,2)）：第一行全部翻开 → 0 推出两个安全格，1 再推出雷。
    const mss::ObservedBoard::Result board =
        makeBoard(2, 3, 1, {{0, 0, State::Num0}, {0, 1, State::Num1}, {0, 2, State::Num1}});
    const mss::Basic::Result basic = mss::Basic::analyze(board, workspace.basic);
    check(basic.valid, "链式传播后必须有效");
    check(basic.marks[1][0] == Mark::S, "(0,0)=0 推出 (1,0) 安全");
    check(basic.marks[1][1] == Mark::S, "(0,0)=0 推出 (1,1) 安全");
    check(basic.marks[1][2] == Mark::F, "(0,1)=1 且只剩一个候选 → (1,2) 是雷");
    check(basic.mineSum == 1 && basic.safeHideCount == 2 && basic.unknownSum == 0, "传播后的三个计数");
}

void testInvalidSources() {
    mss::Workspace workspace;
    {
        // ① 传播中越界：0 的邻居被断言为雷。
        const mss::ObservedBoard::Result board = makeBoard(2, 2, 1, {{0, 0, State::Num0}, {0, 1, State::ForcedMine}});
        check(!mss::Basic::analyze(board, workspace.basic).valid, "数字 0 旁边出现确定雷 → invalid");
    }
    {
        // ③ 总雷数终检：2x2 要放 3 颗雷，但 0 把三个邻居全推成安全。
        const mss::ObservedBoard::Result board = makeBoard(2, 2, 3, {{0, 0, State::Num0}});
        check(!mss::Basic::analyze(board, workspace.basic).valid, "推完后剩余候选放不下总雷数 → invalid");
    }
}

// 固定雷局：4x4、2 颗雷，按行序翻开全部非雷格，每步与全量重建对比。
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

void testIncrementalMatchesRebuild() {
    mss::Workspace workspace;
    mss::ObservedBoard::Result board = mss::ObservedBoard::analyze(kRows, kCols, 2);
    mss::Basic::Result incremental = mss::Basic::analyze(board, workspace.basic);
    int revealed = 0;
    for (int x = 0; x < kRows; ++x)
        for (int y = 0; y < kCols; ++y) {
            if (isMine(x, y))
                continue;
            mss::ObservedBoard::Delta boardDelta;
            boardDelta.changes.push_back({board.id(x, y), static_cast<State>(digitAt(x, y))});
            mss::ObservedBoard::update(board, boardDelta);
            mss::Basic::Delta basicDelta;
            mss::Basic::update(incremental, basicDelta, board, boardDelta, workspace.basic);
            ++revealed;
            const mss::Basic::Result rebuilt = mss::Basic::analyze(board, workspace.basic);
            check(rebuilt.valid, "真实雷局的每一步都必须有效");
            check(incremental.sameAs(rebuilt), "增量结果必须与全量重建逐字段一致（含内部加速表）");
        }
    check(revealed == kRows * kCols - 2, "翻开的非雷格数量");
}

void testDeltaRoundTrip() {
    // 3x3、1 雷（在 (0,2)）：逐步翻开 (0,0)=0、(0,1)=1、(1,1)=1。
    mss::Workspace workspace;
    mss::ObservedBoard::Result board = mss::ObservedBoard::analyze(3, 3, 1);
    mss::Basic::Result live = mss::Basic::analyze(board, workspace.basic);
    const mss::Basic::Result initial = live;
    std::vector<mss::Basic::Delta> deltas;

    constexpr int kReveal[3][2] = {{0, 0}, {0, 1}, {1, 1}};
    constexpr State kStates[3] = {State::Num0, State::Num1, State::Num1};
    for (int i = 0; i < 3; ++i) {
        mss::ObservedBoard::Delta boardDelta;
        boardDelta.changes.push_back({board.id(kReveal[i][0], kReveal[i][1]), kStates[i]});
        mss::ObservedBoard::update(board, boardDelta);
        mss::Basic::Delta basicDelta;
        mss::Basic::update(live, basicDelta, board, boardDelta, workspace.basic);
        check(live.valid, "这一步必须有效");
        deltas.push_back(std::move(basicDelta));
    }
    const mss::Basic::Result finalState = live;

    for (int i = static_cast<int>(deltas.size()); i-- > 0;)
        mss::Basic::reverseDelta(live, deltas[static_cast<std::size_t>(i)]);
    check(live.sameAs(initial), "全部反向回放后必须回到初始状态");

    for (const mss::Basic::Delta &delta : deltas)
        mss::Basic::applyDelta(live, delta);
    check(live.sameAs(finalState), "正向重放后必须回到最终状态");
}

void testForcedConflict() {
    // 0 颗雷：0 把三个邻居推成安全，此时盘面自洽；随后把其中一格断言为雷才产生冲突。
    mss::Workspace workspace;
    mss::ObservedBoard::Result board = makeBoard(2, 2, 0, {{0, 0, State::Num0}});
    mss::Basic::Result basic = mss::Basic::analyze(board, workspace.basic);
    check(basic.valid && basic.marks[0][1] == Mark::S, "先决条件：(0,1) 已被推为安全");

    mss::ObservedBoard::Delta boardDelta;
    boardDelta.changes.push_back({board.id(0, 1), State::ForcedMine});
    mss::ObservedBoard::update(board, boardDelta);
    mss::Basic::Delta basicDelta;
    mss::Basic::update(basic, basicDelta, board, boardDelta, workspace.basic);
    check(!basic.valid, "把已知安全的格断言为雷 → invalid（④）");
}

} // namespace

void basic() {
    testInitialMarks();
    testPropagation();
    testInvalidSources();
    testIncrementalMatchesRebuild();
    testDeltaRoundTrip();
    testForcedConflict();
}

} // namespace test