// ObservedBoard 迁移契约的可执行形式：合法迁移走行为级验证，非法迁移是规范表（待子进程测试器）。
#include <array>
#include <cstddef>
#include <string_view>

#include "board/observed_board.h"
#include "harness.h"

namespace test {

namespace {

using State = mss::ObservedBoard::CellState;

// 非法迁移规范表。update 在这些输入上必须 assert 杀死进程 —— 进程内验证不了
// （assert 会 exit），需要一个跑子进程的测试器；在那之前这几行是规范文档，
// 不会红（明确标记，避免被误当成已覆盖的用例）。合法迁移那一侧见
// testLegalTransitions()，它是行为级验证，不依赖本表。
struct TransitionCase {
    State from;
    State to;
    std::string_view why;
};

[[maybe_unused]] constexpr std::array<TransitionCase, 8> kIllegalTransitions{{
    {State::Num3, State::Num3, "同一格重复 update（1.4 的 target must be Hidden）"},
    {State::ForcedMine, State::ForcedMine, "重复断言同一事实"},
    {State::ForcedMine, State::ForcedSafe, "改判（1.4 不允许）"},
    {State::Num0, State::Num1, "已翻开的数字不能再被 update 改写"},
    {State::Hidden, State::Hidden, "update 永远不能把格子置回 Hidden"},
    {State::Num3, State::Hidden, "撤销只能走 reverseDelta"},
    {State::ForcedSafe, State::Hidden, "同上"},
    {State::ForcedMine, State::Num1, "断言过的格子不能再翻开"},
}};

constexpr std::array<State, 12> kAllStates{
    State::Num0, State::Num1, State::Num2, State::Num3, State::Num4, State::Num5, State::Num6,
    State::Num7, State::Num8, State::Hidden, State::ForcedMine, State::ForcedSafe};

// 合法迁移：从 Hidden 出发，除 Hidden 之外的每个状态都必须能真正应用成功。
// 行为级验证（走真的 update 与 reverseDelta），不依赖任何公开谓词。
void testLegalTransitions() {
    for (const State to : kAllStates) {
        if (to == State::Hidden)
            continue; // Hidden -> Hidden 在非法规范表里
        mss::ObservedBoard::Result board = mss::ObservedBoard::analyze(2, 2, 1);
        mss::ObservedBoard::Delta delta;
        delta.changes.push_back({board.id(0, 0), to});
        mss::ObservedBoard::update(board, delta);
        check(board.board[0][0] == to, "Hidden -> 任意非 Hidden 都必须可应用");
        check(delta.changes[0].next == to, "Delta 不得被 update 改写（只读契约）");
        mss::ObservedBoard::reverseDelta(board, delta);
        check(board.board[0][0] == State::Hidden, "reverseDelta 必须回到 Hidden");
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
    delta.changes.push_back({board.id(0, 0), State::Num1});
    delta.changes.push_back({board.id(0, 1), State::ForcedMine});
    const std::size_t capacity = delta.changes.capacity();

    mss::ObservedBoard::update(board, delta);
    check(board.board[0][0] == State::Num1, "正向应用：数字");
    check(board.board[0][1] == State::ForcedMine, "正向应用：断言");

    // 编译期契约：update 只读 Delta —— 传 const Delta 必须能编译并通过。
    const mss::ObservedBoard::Delta constDelta = delta;
    mss::ObservedBoard::reverseDelta(board, constDelta);
    check(board.board[0][0] == State::Hidden && board.board[0][1] == State::Hidden, "reverseDelta = 恢复为 Hidden");
    mss::ObservedBoard::update(board, constDelta);
    check(board.board[0][0] == State::Num1 && board.board[0][1] == State::ForcedMine, "只读 Delta 也能应用");

    // 反向撤销（多格时从后往前；单格场景下顺序不可观测，但契约如此）
    mss::ObservedBoard::reverseDelta(board, delta);
    check(board.board[0][0] == State::Hidden && board.board[0][1] == State::Hidden, "reverseDelta 必须回到 Hidden");

    mss::ObservedBoard::applyDelta(board, delta);
    check(board.board[0][0] == State::Num1 && board.board[0][1] == State::ForcedMine, "applyDelta 正向重放");

    delta.clear();
    check(delta.changes.empty() && delta.changes.capacity() == capacity, "clear 必须保留 capacity");
    delta.changes.push_back({board.id(1, 1), State::Num0});
    mss::ObservedBoard::update(board, delta);
    check(board.board[1][1] == State::Num0, "Delta 复用");
}

} // namespace

void boardTransitions() {
    testLegalTransitions();
    testGeometry();
    testDeltaRoundTrip();
}

} // namespace test