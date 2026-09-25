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
//   坐标 1-based；Grid<CellState> 内部垫一行一列，下标 = x*(cols+1)+y。
//   ObservedBoard::CellId 就是这个下标，可直接作为 Grid/RawGrid 的索引（Structure::cellLoc 依赖这一点）。
//   padding（x==0 或 y==0）不是真实格子，分析层不得访问。
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
    // 棋盘格句柄：Grid<CellState> 的存储下标，即 x*(cols+1)+y。
    // 合法范围 1..rows × 1..cols；-1 表示"无此格"。
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
// 【待审决定】本轮 review 的唯一目的：这四条你拍板
//
//   D1  Change::previous 是否保留？
//       现状恒为 Hidden（唯一合法迁移的直接后果），是冗余字段。
//       保留 = 为将来的"允许改判"留位置；删除 = 更诚实的类型（同时解锁 D4）。
//
//   D2  是否允许"幂等重复断言"（ForcedMine → ForcedMine）与"改判"
//       （ForcedMine → ForcedSafe）？
//       现状：一律 assert。1.4 那 6 次 target must be Hidden 很可能就来自
//       "同一格被断言两次" —— 如果确认，这里选"允许"就是修 bug，而不是改行为。
//       （改这一条会让旧实现不再是 oracle，属左栏变更，必须显式记录。）
//
//   D3  applyDelta 是否加断言？
//       现状：逆序撤销时不做任何检查，在错误的状态上回滚会静默污染。
//       建议加：正确程序不受影响（合法程序在撤销时该格必然等于 change.next），
//       错误程序立刻死。
//
//   D4  update 的 Delta 参数现在是 Delta&（因为要回写 previous）。
//       若 D1 选删除，可改成 const Delta&（调用方意图更清楚）。
//
//   另有两点仅为记录：错误信息与 1.4 不同（合并为一条更明确的），
//   触发条件完全相同；测试里无法在进程内验证"必须 abort"的用例，
//   因为 assert 会 exit —— 计划用子进程测试覆盖（后续阶段）。
// ═══════════════════════════════════════════════════════════════════════

} // namespace mss