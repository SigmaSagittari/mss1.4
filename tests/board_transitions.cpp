// ObservedBoard 状态迁移契约的可执行形式：表即文档，改契约必须改这张表。
#include <array>
#include <cstddef>
#include <string_view>

#include "board/observed_board.h"
#include "harness.h"

namespace test {

namespace {

using State = mss::ObservedBoard::CellState;

struct TransitionCase {
    State from;
    State to;
    bool legal;
    std::string_view why;
};

constexpr std::array<State, 12> kAllStates{
    State::Num0, State::Num1, State::Num2, State::Num3, State::Num4, State::Num5, State::Num6,
    State::Num7, State::Num8, State::Hidden, State::ForcedMine, State::ForcedSafe};

// 契约表：每一行 = 一条必须成立或必须非法的迁移。
constexpr std::array<TransitionCase, 13> kTransitions{{
    // ── 合法：Hidden → 非 Hidden，唯一形态 ──
    {State::Hidden, State::Num0, true, "翻开 0"},
    {State::Hidden, State::Num3, true, "翻开数字"},
    {State::Hidden, State::Num8, true, "翻开 8"},
    {State::Hidden, State::ForcedMine, true, "外部断言：雷"},
    {State::Hidden, State::ForcedSafe, true, "外部断言：安全"},
    // ── 非法：原地重放 / 撤销走错路 —— 1.4 里观测到的崩溃形态 ──
    {State::Num3, State::Num3, false, "同一格重复 update（1.4 的 target must be Hidden）"},
    {State::ForcedMine, State::ForcedMine, false, "重复断言同一事实"},
    {State::ForcedMine, State::ForcedSafe, false, "改判（1.4 不允许，见 D2）"},
    {State::Num0, State::Num1, false, "已翻开的数字不能再被 update 改写"},
    {State::Hidden, State::Hidden, false, "update 永远不能把格子置回 Hidden"},
    {State::Num3, State::Hidden, false, "撤销只能走 applyDelta(reverse=true)"},
    {State::ForcedSafe, State::Hidden, false, "同上"},
    {State::ForcedMine, State::Num1, false, "断言过的格子不能再翻开"},
}};

static_assert(sizeof(State) == 1, "CellState 必须是一个字节（跨模块存储契约）");

void testTransitionTable() {
    for (const TransitionCase &c : kTransitions)
        check(mss::ObservedBoard::isLegalTransition(c.from, c.to) == c.legal, c.why);

    // 穷举 12×12：合法集合必须恰好等于 {Hidden} × {非 Hidden}，不是抽样而是全表。
    for (const State from : kAllStates)
        for (const State to : kAllStates) {
            const bool expected = from == State::Hidden && to != State::Hidden;
            check(mss::ObservedBoard::isLegalTransition(from, to) == expected, "12x12 穷举：迁移规则不一致");
        }
}

void testGeometry() {
    const mss::ObservedBoard::Result board = mss::ObservedBoard::analyze(3, 3, 1);
    check(board.id(0, 0) == 0, "CellId 编码：左上角必须是 0");
    check(board.id(board.rows - 1, board.cols - 1) == board.rows * board.cols - 1, "CellId 编码：右下角必须是 rows*cols-1");
    for (int x = 0; x < board.rows; ++x)
        for (int y = 0; y < board.cols; ++y) {
            const mss::ObservedBoard::CellId cell = board.id(x, y);
            check(cell == x * board.cols + y, "CellId 公式（0-based，无 padding）");
            check(cell >= 0 && cell < board.rows * board.cols, "CellId 必须落在 [0, rows*cols)");
            const auto [px, py] = board.pos(cell);
            check(px == x && py == y, "id/pos 往返");
            check(board.board[x][y] == State::Hidden, "analyze 必须产出全 Hidden");
        }
}

void testDeltaRoundTrip() {
    mss::ObservedBoard::Result board = mss::ObservedBoard::analyze(3, 3, 1);
    mss::ObservedBoard::Delta delta;
    delta.changes.push_back({board.id(0, 0), State::Num1, State::Hidden});
    delta.changes.push_back({board.id(0, 1), State::ForcedMine, State::Hidden});
    const std::size_t capacity = delta.changes.capacity();

    mss::ObservedBoard::update(board, delta);
    check(board.board[0][0] == State::Num1, "正向应用：数字");
    check(board.board[0][1] == State::ForcedMine, "正向应用：断言");
    for (const mss::ObservedBoard::Change &change : delta.changes)
        check(change.previous == State::Hidden, "previous 必须被 update 回写（当前契约下恒为 Hidden）");

    mss::ObservedBoard::applyDelta(board, delta);  // reverse=true
    check(board.board[0][0] == State::Hidden && board.board[0][1] == State::Hidden, "逆序撤销必须回到 Hidden");

    mss::ObservedBoard::applyDelta(board, delta, false);
    check(board.board[0][0] == State::Num1 && board.board[0][1] == State::ForcedMine, "正序重放");

    delta.clear();
    check(delta.changes.empty() && delta.changes.capacity() == capacity, "clear 必须保留 capacity");
    delta.changes.push_back({board.id(1, 1), State::Num0, State::Hidden});
    mss::ObservedBoard::update(board, delta);
    check(board.board[1][1] == State::Num0, "Delta 复用");
}

} // namespace

void boardTransitions() {
    testTransitionTable();
    testGeometry();
    testDeltaRoundTrip();
}

} // namespace test