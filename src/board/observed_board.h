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
//   撤销只能走 applyDelta(reverse=true)。
//   重复 update 同一格（Num3 → Num3）、改判（ForcedMine → ForcedSafe）、
//   把格子置回 Hidden —— 全部是调用方 bug，assert 杀死进程，绝不静默回退。
//   规则的唯一真相是 isLegalTransition()；测试穷举 12×12 全表。
//
// 【Delta 生命周期】
//   changes 由调用方填充 {cell, next}；previous 是 update 的输出，
//   调用 update 之前它的值无意义。
//   一个 Delta 被 update 之后只能用于 applyDelta；再次当作"新更新"提交是 bug。
//   applyDelta(reverse=true)：必须在该 Delta 对应的子状态上、从后往前撤销。
//   applyDelta(reverse=false)：从前往后重放（测试/重放用）。
//   applyDelta 不修改 Delta，也不修改 previous。
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

    // 唯一的迁移合法性规则。update 的断言与测试的穷举都读这一个函数。
    static constexpr bool isLegalTransition(CellState from, CellState to) {
        return from == CellState::Hidden && to != CellState::Hidden;
    }

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
        // update 的输出：应用前的状态。在【唯一合法迁移】下恒为 Hidden，
        // 因此这是一个冗余字段 —— 见文件末尾 D1。
        CellState previous = CellState::Hidden;
    };

    struct Delta {
        std::vector<Change> changes;
        void clear();
    };

    static Result analyze(int rows, int cols, int mines);
    static void update(Result &board, Delta &delta);
    static void applyDelta(Result &board, const Delta &delta, bool reverse = true);
};

// ═══════════════════════════════════════════════════════════════════════
// 【已定决策】（D1–D4 已拍板，以下即契约；改动需显式记录）
//
//   D1  Change::previous 保留。当前恒为 Hidden，为将来可能的"允许改判"留位置。
//   D2  只允许 Hidden → 非 Hidden。幂等重复断言（ForcedMine → ForcedMine）
//       与改判（ForcedMine → ForcedSafe）一律 assert —— 即 1.4 的行为。
//       注：1.4 的 6 次 target must be Hidden 崩溃与这条同源，属"调用方违约"，
//       不是本模块的 bug；重现与修复在调用方（见后续 policy 层审计）。
//   D3  applyDelta 不加校验：在错误状态上回滚仍会静默污染。契约要求调用方
//       "在该 Delta 对应的子状态上、从后往前撤销"。加校验留待诊断模块统一做。
//   D4  update 的 Delta 参数保持 Delta&（要回写 previous）。
//
//   错误信息与 1.4 不同（合并为一条更明确的），触发条件完全相同。
//   进程内无法验证"必须 abort"的用例（assert 会 exit）——留待子进程测试。
// ═══════════════════════════════════════════════════════════════════════

} // namespace mss