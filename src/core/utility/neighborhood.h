#pragma once

namespace mss {

// 遍历 (x,y) 的 8 邻域：0-based，只回调落在 [0,rows)×[0,cols) 内的坐标。
// 顺序固定（上、左、右、下），全体调用方共享 —— 因此 Delta 的内容也是确定的。
// 调用方必须保证 (x,y) 本身在界内；这里刻意不做该检查。
template <typename Func> inline void forEachAdjacent(int x, int y, int rows, int cols, Func &&fn) {
    const bool up = x > 0;
    const bool down = x + 1 < rows;
    const bool left = y > 0;
    const bool right = y + 1 < cols;

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