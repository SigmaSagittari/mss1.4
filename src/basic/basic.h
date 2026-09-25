#pragma once

#include <cstdint>
#include <vector>

#include "board/observed_board.h"
#include "core/utility/grid.h"
#include "core/utility/neighborhood.h"

namespace mss {

// ═══════════════════════════════════════════════════════════════════════
// Basic 契约
//
// 【四个标记】Mark : uint8_t，取值顺序是跨模块契约，不得重排。
//   H = 0  前沿候选：未翻开，且至少邻接一个已翻开的数字。
//   T = 1  非前沿候选：未翻开，且不邻接任何已翻开的数字。
//          H 与 T 都表示"仍可能是雷"（isCandidate = H|T）。
//   S = 2  确定不是雷。三种来源：(a) 被传播推出的安全格，(b) 外部断言
//          ForcedSafe，(c) 已翻开的数字格（只作为"非候选"的填充值，无独立含义）。
//   F = 3  确定是雷：被传播推出的雷，或外部断言 ForcedMine。
//
// 【三个计数】都属于契约，外部会读。
//   unknownSum    = #T（不含 H）。概率层用 board.totalMines - mineSum - unknownSum
//                   之外的组合数算 T 格分布，所以 T 与 H 必须分开数。
//   mineSum       = #F。外部用 board.totalMines - mineSum 得到剩余雷数。
//   safeHideCount = #(S ∧ 未翻开)。即"现在就能安全点开的格数"。
//                   已翻开的数字格虽然也标 S，但不计入。
//
// 【两张加速表】Result::mineAround / hideAround —— 内部状态，外部不得读，
//   由 analyze 建立、由 update / applyDelta / reverseDelta 增量维护。
//   mineAround[x][y] = (x,y) 的 8 邻域中 F 的个数
//   hideAround[x][y] = (x,y) 的 8 邻域中候选（H|T）的个数
//   只有数字格会读它们；存在的唯一理由是让增量传播 O(1) 拿到邻域计数。
//
// 【传播规则】反复应用到不动点：
//   对每个数字格 n：remaining = 数值 - mineAround[n]，candidates = hideAround[n]
//     remaining < 0 或 > candidates  → 矛盾（valid = false）
//     remaining == 0                 → n 的全部候选标 S
//     remaining == candidates        → n 的全部候选标 F
//     任何标记变化后，受影响的数字格重新入队
//
// 【valid 的四个来源】矛盾时 valid = false：
//   ① 传播中 remaining 越界  ② 数字约束终检  ③ 总雷数终检  ④ 强制标记与已知 S/F 冲突
//
// 【valid = false 的后果】（比 1.4 更严格）
//   一旦 valid = false：传播立即中断，Result 的其余字段全部作废（内容未定义）。
//   该 Result 永久不可再 update —— 只能 reverseDelta 回放回父状态，或 analyze 重建。
//   这是有意的：在无效状态上继续传播只会把错误扩散到后续层。
//
// 【Delta 方向】与 ObservedBoard 相反：
//   Basic::Delta 是 update 的输出（调用方只读），记录逐格 {cell, old, now}
//   以及前后两份计数快照。applyDelta / reverseDelta 只回放标记与计数，
//   不重新推理。
//
// 【调用前置条件】update 之前，updates 必须已经由 ObservedBoard::update 应用到
//   board，且顺序一致。
// ═══════════════════════════════════════════════════════════════════════

struct Basic {
    enum class Mark : std::uint8_t {
        H = 0,
        T = 1,
        S = 2,
        F = 3,
    };

    // 增量传播的工作区：跨调用复用容量，显式传入（不用 thread_local 全局）。
    struct Workspace {
        std::vector<ObservedBoard::CellId> pending;
        std::vector<unsigned char> queued; // 尺寸 rows*cols，按 CellId 直接索引
    };

    struct Result {
        int rows = 0;
        int cols = 0;
        Grid<Mark> marks;
        int unknownSum = 0;    // #T
        int mineSum = 0;       // #F
        int safeHideCount = 0; // #(S ∧ 未翻开)
        bool valid = true;

        // 内部加速状态（见契约）。外部不得读。
        Grid<std::int8_t> mineAround;
        Grid<std::int8_t> hideAround;
    };

    struct Delta {
        struct Change {
            ObservedBoard::CellId cell = -1;
            Mark old = Mark::T;
            Mark now = Mark::T;
        };

        struct Snapshot {
            int unknownSum = 0;
            int mineSum = 0;
            int safeHideCount = 0;
            bool valid = true;
        };

        std::vector<Change> changes;
        Snapshot before;
        Snapshot after;
    };

    // 全量构建：初始标记 → 传播到不动点 → 建加速表 → 数字约束与总雷数终检。
    static Result analyze(const ObservedBoard::Result &board);

    // 增量传播一批新观测，并写出可回放的 Delta。
    static void update(Result &result, Delta &delta, const ObservedBoard::Result &board,
                       const ObservedBoard::Delta &updates, Workspace &workspace);

    // 正向重放 / 反序回放：只回放标记与计数，不重新推理。
    static void applyDelta(Result &result, const Delta &delta);
    static void reverseDelta(Result &result, const Delta &delta);
};

} // namespace mss