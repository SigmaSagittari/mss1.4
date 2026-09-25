// 本文件是 ObservedBoard 状态迁移契约的可执行形式：表即文档，改契约必须改这张表。
#include <array>
#include <cstddef>
#include <cstdio>
#include <string_view>

#include "board/observed_board.h"
#include "version.h"

namespace {

using State = mss::ObservedBoard::CellState;

int g_checks = 0;
int g_failures = 0;

void check(bool ok, std::string_view what, int line) {
    ++g_checks;
    if (ok)
        return;
    ++g_failures;
    std::printf("[FAIL] %.*s  (board_transitions.cpp:%d)\n", static_cast<int>(what.size()), what.data(), line);
}

#define CHECK(cond, why) check((cond), (why), __LINE__)

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
        CHECK(mss::ObservedBoard::isLegalTransition(c.from, c.to) == c.legal, c.why);

    // 穷举 12×12：合法集合必须恰好等于 {Hidden} × {非 Hidden}，不是抽样而是全表。
    for (const State from : kAllStates)
        for (const State to : kAllStates) {
            const bool expected = from == State::Hidden && to != State::Hidden;
            CHECK(mss::ObservedBoard::isLegalTransition(from, to) == expected, "12x12 穷举：迁移规则不一致");
        }
}

void testGeometry() {
    const mss::ObservedBoard::Result board = mss::ObservedBoard::analyze(3, 3, 1);
    for (int x = 1; x <= board.rows; ++x)
        for (int y = 1; y <= board.cols; ++y) {
            const mss::CellId cell = board.id(x, y);
            CHECK(cell == x * (board.cols + 1) + y, "CellId 公式");
            const auto [px, py] = board.pos(cell);
            CHECK(px == x && py == y, "id/pos 往返");
            CHECK(board.board[x][y] == State::Hidden, "analyze 必须产出全 Hidden");
        }
}

void testDeltaRoundTrip() {
    mss::ObservedBoard::Result board = mss::ObservedBoard::analyze(3, 3, 1);
    mss::ObservedBoard::Delta delta;
    delta.changes.push_back({board.id(1, 1), State::Num1, State::Hidden});
    delta.changes.push_back({board.id(1, 2), State::ForcedMine, State::Hidden});
    const std::size_t capacity = delta.changes.capacity();

    mss::ObservedBoard::update(board, delta);
    CHECK(board.board[1][1] == State::Num1, "正向应用：数字");
    CHECK(board.board[1][2] == State::ForcedMine, "正向应用：断言");
    for (const mss::ObservedBoard::Change &change : delta.changes)
        CHECK(change.previous == State::Hidden, "previous 必须被 update 回写（当前契约下恒为 Hidden）");

    mss::ObservedBoard::applyDelta(board, delta);  // reverse=true
    CHECK(board.board[1][1] == State::Hidden && board.board[1][2] == State::Hidden, "逆序撤销必须回到 Hidden");

    mss::ObservedBoard::applyDelta(board, delta, false);
    CHECK(board.board[1][1] == State::Num1 && board.board[1][2] == State::ForcedMine, "正序重放");

    delta.clear();
    CHECK(delta.changes.empty() && delta.changes.capacity() == capacity, "clear 必须保留 capacity");
    delta.changes.push_back({board.id(2, 2), State::Num0, State::Hidden});
    mss::ObservedBoard::update(board, delta);
    CHECK(board.board[2][2] == State::Num0, "Delta 复用");
}

} // namespace

int main() {
    std::printf("=== mss %s | tests/board_transitions ===\n", mss::kVersion);
    testTransitionTable();
    testGeometry();
    testDeltaRoundTrip();
    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}