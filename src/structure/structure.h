#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "basic/basic.h"
#include "board/observed_board.h"
#include "core/utility/flat_hashtable.h"
#include "core/utility/grid.h"
#include "core/utility/hash.h"
#include "core/utility/rng.h"
#include "core/utility/vector_pool.h"

namespace mss {

// ═══════════════════════════════════════════════════════════════════════
// Structure 契约
//
// 【职责】
//   把 Basic 的"数字 ↔ H 候选"二部图切成互不相连的组件；组件内部再把"邻接数字
//   集合完全相同"的 H 格压成一个 Box。下游只枚举 Box 的雷数分配，而不是每个格子
//   的雷位布局 —— 这是整个求解器的性能地基。
//
// 【四种句柄】都是 int，且只在自己所属的容器内有意义
//   ComponentId  Result::components 的下标。**会因删改而变**，不得跨 update 保存。
//   BoxId        **Shape 内局部下标**。单独拿出来无意义：不同 Shape 的 BoxId 会撞。
//   ShapeId      Pool 内 interned Shape：按内容去重、只增不删，句柄稳定。
//   InstanceId   Pool 内 interned Instance：同上。
//   四个句柄都是 int 的别名 —— 编译器把它们视作同一类型，因此：
//     · 禁止用重载区分句柄（`boxCount(ShapeId)` 与 `boxCount(InstanceId)` 会直接编译失败）；
//     · 访问器函数名必须自带对象前缀（shapeXxx / instanceXxx）；
//     · 传错句柄编译器不会报错，靠调用点自查。
//
// 【Shape / Instance 分工】
//   Shape    与坐标无关：只有 Box 尺寸与约束 —— 所以同一形状出现在盘面不同位置时只存一份。
//   Instance 是 Shape 在具体盘面上的落位：哪个 Box 有哪些真实格子、哪条约束对应哪个数字格。
//   对应关系：两者 Box 数相同；Shape 的第 i 条约束 ↔ Instance 的第 i 个 constraintCell（同序）。
//
// 【Box 与约束】
//   Box    邻接数字集合完全相同的 H 格集合。
//   约束   sum    = 数字值 − 该数字邻域内已被确定的雷数（Basic 的 F 标记）
//          boxIds = 该数字邻接到的 Box 集合（按邻域遍历顺序去重；区间长度可为 0，
//                   表示该数字周围没有候选 Box，此时 ConstraintView 的 boxIds 为空）
//
//   约束的 boxIds 是池里一段连续区间，用项目统一的 vectorPool<BoxId>::vector 句柄表示
//   —— 和 boxes_ / cells_ / boxOf_ 同一套词汇，不手搓 offset/count。
//   两条不变量来自同一个事实 —— **一个格子的邻居上限是 8**：
//     · Box::size <= kMaxBoxSize(8)：同签名格子都是同一数字的邻居，交集不超过 8；
//     · 一条约束引用的 Box 数 <= kMaxConstraintBoxes(8)：邻居格至多 8 个，各自落在一个 Box 里。
//   它们是**构建期校验**（assert），不是存储形态：区间长度由句柄的 size 自带，
//   按实际引用数分配 → 没有定长浪费。
//
//   Shape 里**不存数字格** —— 这是 Shape 能按内容去重的前提；数字格在 Instance 里。
//
// 【Result::cellLoc】按 CellId 稠密索引（长度 rows*cols）
//   {component >= 0, box >= 0}   该格是某 Box 的候选格（H）
//   {component >= 0, box == -1}  该格是某组件的数字格
//   {-1, -1}                     不参与任何组件（S / F / T，或周围候选已被推成 S 的数字格）
//
// 【Pool：读写边界与 span 生命周期】
//   · 写入（internShape / internInstance / clear）只发生在 Structure::analyze、
//     Structure::update 与 Pool::clear() 内部；intern* 是 private + friend Structure，
//     上层在**语法上无法**让池变更。
//   · 于是上层从 Pool 拿到的 span / 裸指针，有效期 = 到下一次 analyze / update / clear 为止。
//     在分析阶段（ShapeSolver / Probability / BruteForce / 逻辑块求解）可以任意保存、
//     跨函数传递、在循环外提升 base 指针、内层裸下标 —— 不会悬空。
//   · 分析阶段结束、准备下一次 analyze/update 时，必须丢弃全部 span。
//   · 去重命中时 intern 内部仍会先 push_back 再 pop_back → 同样可能扩容，旧 span 一样失效。
//   · clear() 只保留容量、元素已析构：旧 span 的内容未定义，不是"旧数据"。
//   · 不提供运行时护栏（generation 之类）；正确性由契约 + 测试保证
//     （增量 vs 全量重建、Delta 双向回放）。
//
// 【Delta 顺序契约】
//   · update 用"尾元素搬移"把失效组件从 components 里删掉以保持稠密；搬移顺序 =
//     **下标降序**，removed 就按这个顺序记录（removed[i] 与 removedData[i] 一一对应）。
//   · applyDelta（正向）：先按 removed 的记录顺序重放搬移，再按 addedData 顺序追加。
//   · reverseDelta（反向）：先倒序弹出 addedData，再**倒序**处理 removed（即下标升序），
//     每个恢复的组件都要重新映射 cellLoc。
//   · 一个 Delta 只能在同一状态上回放一次（正向或反向），不可与其它 Delta 交叉。
//   · 在错误状态上回放会静默污染 —— 与 board / basic 一致，不加校验。
//
// 【Pool 的不变量】
//   · 只增不删：intern* 只追加；只有 clear() 会重置（此时全部句柄作废）。
//   · clear() 保留全部底层容量（重置游戏时不重新分配）。
//   · analyze 不清空 Pool：跨盘面复用同一个池是允许的（按内容去重）。
//
// 【analyze / update 的前置条件】
//   · update 之前，updates 必须已由 ObservedBoard::update 应用到 board、且 Basic 已同步更新。
//   · update 只重建受 updates 影响的组件；未受影响的组件与其 cellLoc 映射保持原样（原地不动）。
// ═══════════════════════════════════════════════════════════════════════

struct Structure {
    // 一个格子的邻居上限是 8：Box 尺寸与每条约束引用的 Box 数都受它约束。
    static constexpr int kMaxBoxSize = 8;
    static constexpr int kMaxConstraintBoxes = 8;

    using ComponentId = int;
    using BoxId = int;
    using ShapeId = int;
    using InstanceId = int;

    struct Box {
        int size = 0;
    };

    struct Constraint {
        int sum = 0;
        vectorPool<BoxId>::vector boxIds; // 本约束引用到的 Box 集合（池里一段区间）
    };

    struct ConstraintView {
        int sum = 0;
        std::span<const BoxId> boxIds;
    };

    struct CellLocation {
        ComponentId component = -1;
        BoxId box = -1;
    };

    struct Pool;

    // 与坐标无关的约束形状（按内容 interning 去重）。
    struct Shape {
      private:
        vectorPool<Box>::vector boxes_;
        vectorPool<Constraint>::vector constraints_;
        vectorPool<BoxId>::vector boxIds_; // 本 Shape 全部约束的 Box 引用，首尾相接
        U128 hash_ = {};

        friend struct Pool;
        friend struct Structure;
    };

    // Shape 在具体盘面上的落位。
    struct Instance {
      private:
        ShapeId shape_ = -1;
        vectorPool<ObservedBoard::CellId>::vector cells_;
        vectorPool<int>::vector boxOf_;
        vectorPool<ObservedBoard::CellId>::vector constraintCells_;

        friend struct Pool;
        friend struct Structure;
    };

    struct Result {
        std::vector<InstanceId> components;
        std::vector<CellLocation> cellLoc;

        // 全字段比较：增量维护的回归检查用它。
        bool sameAs(const Result &other) const;
    };

    struct Delta {
        // 见【Delta 顺序契约】。removed 按**下标降序**记录。
        std::vector<ComponentId> removed;
        std::vector<InstanceId> removedData;
        std::vector<InstanceId> addedData;
    };

    struct Pool {
        // ── 读：Shape 侧 ──
        std::size_t shapeCount() const;
        int shapeBoxCount(ShapeId shape) const;
        int shapeBoxSize(ShapeId shape, BoxId box) const;
        std::size_t shapeConstraintCount(ShapeId shape) const;
        ConstraintView shapeConstraint(ShapeId shape, std::size_t index) const;

        // ── 读：Instance 侧 ──
        ShapeId instanceShape(InstanceId instance) const;
        int instanceBoxCount(InstanceId instance) const;
        int instanceBoxCellCount(InstanceId instance, BoxId box) const;
        ObservedBoard::CellId instanceBoxCell(InstanceId instance, BoxId box, int index) const;
        const ObservedBoard::CellId *instanceCellsData(InstanceId instance) const;         // 循环外提升 base 用
        std::span<const ObservedBoard::CellId> instanceCells(InstanceId instance) const;   // 表达式内短暂使用
        std::size_t instanceConstraintCellCount(InstanceId instance) const;
        ObservedBoard::CellId instanceConstraintCell(InstanceId instance, std::size_t index) const;

        // 保留全部底层容量，使所有句柄与 span 失效（重置游戏用）。
        void clear();

      private:
        ShapeId internShape(Shape shape);
        InstanceId internInstance(Instance instance);
        static U128 computeInstanceHash(const Instance &instance, const Pool &pool);

        std::vector<Shape> shapes_;
        FlatHashTable<U128, ShapeId, U128Hash> shapeIndex_;
        std::vector<Instance> instances_;
        FlatHashTable<U128, InstanceId, U128Hash> instanceIndex_;

        vectorPool<Box> boxes_;
        vectorPool<Constraint> constraints_;
        vectorPool<BoxId> boxIds_;
        vectorPool<ObservedBoard::CellId> cells_;
        vectorPool<int> boxOf_;
        vectorPool<ObservedBoard::CellId> constraintCells_;

        friend struct Structure;
    };

    // analyze / update 的复用缓冲（聚合在 mss::Workspace 的 structure 成员）。
    // analyze 与 update 不会并发，visited / cellHash / cells 由两者共用。
    struct Scratch {
        Grid<char> visited;
        Grid<U128> cellHash;
        std::vector<ObservedBoard::CellId> cells;

        Grid<char> dirty;
        std::vector<ObservedBoard::CellId> dirtyCells;
        std::vector<char> removed;
        std::vector<InstanceId> staged;

        FlatHashTable<U128, BoxId, U128Hash> hashBox;
        std::vector<int> boxOfCells;
        std::vector<std::array<ObservedBoard::CellId, 8>> buckets;
        std::vector<std::uint8_t> bucketSize;
        std::vector<char> boxUsed; // 按 BoxId 索引：本条约束里该 Box 是否已记过
    };

    // 全量构建：扫出所有组件 → 压缩 Box → 建约束 → 写池 → 铺 cellLoc。
    static Result analyze(const ObservedBoard::Result &board, const Basic::Result &basic, Pool &pool, Scratch &scratch);

    // 增量重建受影响的组件并写出 Delta。调用前 board/basic 必须已同步。
    static void update(Result &result, Delta &delta, const ObservedBoard::Result &board, const Basic::Result &basic,
                       Pool &pool, const ObservedBoard::Delta &updates, Scratch &scratch);

    // 回放：只搬组件与重映射 cellLoc，不重新推理。
    static void applyDelta(Result &result, const Pool &pool, const Delta &delta);
    static void reverseDelta(Result &result, const Pool &pool, const Delta &delta);
};

} // namespace mss