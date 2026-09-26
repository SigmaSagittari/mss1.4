#pragma once

namespace mss {

// ─────────────────────────────────────────────────────────────
// neighbors.h — 棋盘邻居遍历工具。
// ─────────────────────────────────────────────────────────────

// 遍历 (x, y) 的 8 个邻居，fn 收到的都是合法坐标；Basic、Structure 和测试模拟器
// 共用这个顺序，因此不要在调用层自行补边界或改变邻居定义。
template <typename Func> inline void forEachAdjacent(int x, int y, int rows, int cols, Func &&fn) {
    // 调用方必须先保证 (x, y) 是盘面内坐标；这里刻意不做边界防御。
    bool up = x > 1;
    bool down = x < rows;
    bool left = y > 1;
    bool right = y < cols;

    if (up && left)
        fn(x - 1, y - 1);
    if (up)
        fn(x - 1, y);
    if (up && right)
        fn(x - 1, y + 1);
    if (left)
        fn(x, y - 1);
    if (right)
        fn(x, y + 1);
    if (down && left)
        fn(x + 1, y - 1);
    if (down)
        fn(x + 1, y);
    if (down && right)
        fn(x + 1, y + 1);
}

} // namespace mss
