#pragma once

namespace mss {

// ─────────────────────────────────────────────────────────────
// types.h — core 层基础类型（跨层共享的身份与状态）。
//
//   - CellId 等整数句柄  稠密 int，一律以 vector 下标形式存储/传递（越界即 bug）
//   - CellLocation       格子 → 所属连通块 + 单位格
//
// 约定：
//   - 身份不用裸指针：句柄生命周期由所属池管理（Structure::Pool / DistPool
//     只增不删、地址稳定，见 structure.h / distribution.h）。
//   - core 层不含分析概念（连通块/分布/概率在 analysis 层），也不含游戏
//     规则（雷位布局/翻开逻辑在 game 层）。
// ─────────────────────────────────────────────────────────────

// ── 整数身份 ──
// 稠密 int 句柄，全部以 vector 下标形式存储与传递，越界即 bug。
using CellId = int;          // 棋盘格：x*(cols+1)+y（即 Grid 存储下标，可直接索引 cellLoc）
using ComponentId = int;     // 连通块实例（Structure::Result::components 下标）
using BoxId = int;           // 单位格（Shape 内局部下标，0..boxes.size()-1）
using ShapeId = int;         // interned 不可变结构（Structure::Pool 句柄）
using InstanceId = int;      // interned 不可变布局（Structure::Pool 句柄）
using DistributionId = int;  // 分布缓存句柄（DistPool 句柄）

// 坐标永远是 1-based；0 行/列只存在于 Grid 的 padding 中，不能当作真实格子。

// 格子 → (所属连通块, shape 内单位格下标)。
// 不在任何连通块的格子（Safe/Mine/Unknown）component = -1。
struct CellLocation {
    ComponentId component = -1;
    BoxId box = -1;
};

// 遍历 (x, y) 的 8 个邻居，fn 收到的都是合法坐标；Basic、Structure 和测试模拟器
// 共用这个顺序，因此不要在调用层自行补边界或改变邻居定义。
template <typename Func>
inline void forEachAdjacent(int x, int y, int rows, int cols, Func&& fn) {
    // 调用方必须先保证 (x, y) 是盘面内坐标；这里刻意不做边界防御。
    bool up = x > 1;
    bool down = x < rows;
    bool left = y > 1;
    bool right = y < cols;

    if (up && left) fn(x - 1, y - 1);
    if (up) fn(x - 1, y);
    if (up && right) fn(x - 1, y + 1);
    if (left) fn(x, y - 1);
    if (right) fn(x, y + 1);
    if (down && left) fn(x + 1, y - 1);
    if (down) fn(x + 1, y);
    if (down && right) fn(x + 1, y + 1);
}

}  // namespace mss
