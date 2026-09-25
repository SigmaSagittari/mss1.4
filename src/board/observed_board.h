#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "core/assert.h"
#include "core/utility/grid.h"

namespace mss {

// ═══════════════════════════════════════════════════════════════════════
// ObservedBoard 契约
//
// 【状态语义】
//   Num0..Num8 (0..8)  已翻开的数字，是事实。
//   Hidden     (9)     未翻开。
//   ForcedMine (10)    外部断言"此格是雷"。是断言不是事实，不产生数字推断。
//   ForcedSafe (11)    外部断言"此格安全"。同上。
//   枚举取值是跨模块契约（Basic/Structure 依赖 (int) 比较），顺序不得重排。
//
// 【坐标与存储】
//   坐标 0-based：x ∈ [0,rows)、y ∈ [0,cols)。
//   ObservedBoard::CellId = x*cols + y，与 Grid<CellState> 的线性存储下标一致，
//   可直接当 Grid 的一维下标（Structure::cellLoc 依赖这一点）。
//   没有 padding：越界即 UB，调用方负责坐标合法。
//   Result::pos() 的前置条件：cols > 0（默认构造的 Result 为 0x0，不得调用）。
//
// 【唯一合法迁移】
//   Hidden → 任意非 Hidden。没有第二条。
//   撤销只能走 reverseDelta。
//   重复 update 同一格（Num3 → Num3）、改判（ForcedMine → ForcedSafe）、
//   把格子置回 Hidden —— 全部是调用方 bug，assert 杀死进程，绝不静默回退。
//   规则只写在 update 内部，不对外暴露查询接口：本项目不给"先问能不能做、
//   再决定要不要做"的防御性预检查留口子。
//
// 【Delta 生命周期】
//   changes 由调用方填充 {cell, next}。Delta 交给 update 之后即视为只读
//   （update 收 const Delta&），update 不回写任何字段。
//   一个 Delta 被 update 之后只能交给 applyDelta / reverseDelta；再次当作
//   "新更新"提交给 update 是 bug。
//   applyDelta  ：从前往后把每格写成 next（不检查当前状态）。
//   reverseDelta：从后往前把每格恢复成 Hidden。
//   两者都不修改 Delta。
//   clear() 只清空、保留 capacity（热路径复用契约）。
//
// 【Result】
//   值类型（可拷贝/可移动）。analyze 只创建全 Hidden 视图，不做任何推断、
//   不会失败（与 Basic 不同，它没有 valid 标记）。
// ═══════════════════════════════════════════════════════════════════════

struct ObservedBoard {
    // 棋盘格句柄：Grid<CellState> 的线性存储下标，即 x*cols+y。
    // 合法范围 [0, rows*cols)；-1 表示"无此格"。
    // 注意：它是 int 别名 —— 嵌套只表达归属，不提供类型安全（编译器把它和任何 int 视为同一种类型）。
    using CellId = int;

    enum class CellState : std::uint8_t {
        Num0 = 0,
        Num1,
        Num2,
        Num3,
        Num4,
        Num5,
        Num6,
        Num7,
        Num8,
        Hidden,
        ForcedMine,
        ForcedSafe,
    };

    struct Result {
        int rows = 0;
        int cols = 0;
        int totalMines = 0;
        Grid<CellState> board;

        Result();
        Result(int rows, int cols, int mines);

        CellId id(int x, int y) const;
        std::pair<int, int> pos(CellId cell) const;
    };

    struct Change {
        CellId cell = -1;
        CellState next = CellState::Hidden;
    };

    struct Delta {
        std::vector<Change> changes;
        void clear();
    };

    static Result analyze(int rows, int cols, int mines);

    // 把 Delta 施加到盘面：每格必须处于 Hidden（见【唯一合法迁移】）。
    static void update(Result &board, const Delta &delta);
    // 正向重放：每格写成 change.next（不检查当前状态；用于撤销后重放与测试）。
    static void applyDelta(Result &board, const Delta &delta);
    // 反向撤销：每格恢复成 Hidden，从 changes 末尾往前处理。
    // 前提是【唯一合法迁移】（只允许 Hidden → 非 Hidden），所以"撤销"等价于
    // "恢复 Hidden"，不必记录旧状态。一旦放开该规则（例如允许改判），本函数
    // 必须改成依据 Delta 里显式记录的旧状态回滚。
    static void reverseDelta(Result &board, const Delta &delta);
};

} // namespace mss
