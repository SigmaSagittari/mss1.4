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
// ── 30 秒导读（细节见下方完整契约）─────────────────────────────────
//   干什么  把"数字 ↔ H 候选"的约束图切成组件；组件内把邻接数字集合相同的
//           H 格压成一个 Box。下游只枚举 Box 雷数，不枚举每格雷位。
//   怎么读  消费者只用 Pool 的访问器（不再自己解句柄）：
//             Shape 侧：shapeCount / shapeBoxCount / shapeBoxSize /
//                       shapeConstraintCount / shapeConstraint（返回 {sum, span<BoxId>}）
//             Instance 侧：instanceBoxCount / instanceBoxCellCount / instanceBoxCell /
//                       instanceCells（span）/ instanceCellsData（裸指针）/ instanceShape
//           遍历组件：result.components（InstanceId 列表）+ result.cellLoc[cell]
//   记四句  · ComponentId 会变；BoxId 是 Shape 内局部；ShapeId/InstanceId 稳定。
//           · 存的是句柄（Shape::Constraint），取的是投影（Shape::ConstraintView）。
//           · span 有效期 = 到下一次 analyze / update / clear；分析阶段可随便存。
//           · 顺序契约：removed 按下标降序记录；正向按记录走，反向倒着走。
// ─────────────────────────────────────────────────────────────────────
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
// 【Box 与约束】两者都挂在 Shape 下：Shape::Box / Shape::Constraint（存储态）+
//   Shape::ConstraintView（投影态）。四个**句柄**留在 Structure 层，因为它们是
//   跨 Shape / Instance / Pool / Result 的共享词汇。
//   Box    邻接数字集合完全相同的 H 格集合。
//   约束   sum    = 数字值 − 该数字邻域内已被确定的雷数（Basic 的 F 标记）
//          boxIds = 该数字邻接到的 Box 集合（按邻域遍历顺序去重；区间长度可为 0，
//                   表示该数字周围没有候选 Box，此时 ConstraintView 的 boxIds 为空）
//
//   约束的 boxIds 是池里一段连续区间，用项目统一的 vectorPool<BoxId>::vector 句柄表示
//   —— 和 boxes_ / cells_ / boxOf_ 同一套词汇，不手搓 offset/count。
//   两条上限来自同一个事实 —— **一个格子的邻居上限是 8**（kMaxNeighbors）：
//     · Box::size <= kMaxNeighbors：同签名的格子都是同一个数字的邻居，交集不超过它的邻居数；
//     · 一条约束引用的 Box 数 <= kMaxNeighbors：邻居至多 8 个，每个邻居落在一个 Box 里。
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
//
// 【实现要点】（声明看不出内部在干什么，这里说清楚；代码在 structure.cpp）
//   组件发现  从任一 H 格出发，在"数字 ↔ H 候选"二部图上 BFS（数字找 H 邻居、H 找数字邻居），
//              得到一个连通的约束组件。
//   Box 压缩  每个 H 格算一个 128 位签名 = Σ splitmix64(位置种子(每个相邻数字))；签名相同
//              等价于"邻接数字集合相同"（128 位哈希，碰撞可忽略），这些格合并成一个 Box。
//   cellHash  一物两用：先存签名（分组用），分组完成后同一张表改存该格的 BoxId，
//              供紧接的约束构造回查 —— 省一张按格的表。
//   约束      sum = 数字值 − 邻域内 Basic 判定的雷数（F）；boxIds = 邻域里出现过的 Box（去重）。
//   实例布局  格子按 Box 连续存放，另存偏移表 boxOf —— 于是"第 b 个 Box 的格子"是一段
//              连续区间，暴力枚举可以顺着扫。
//   interning Shape / Instance 按内容哈希去重；命中时把刚写进池尾的内容弹回去（pop_back），
//              于是"同一形状/布局只存一份"，且句柄稳定。
//   update    三段式：① 观测变化的格 + 八邻域 = 脏区起点；② 脏区里凡属于某组件的格子，
//              整个组件失效（清它的 cellLoc，并把它的全部格子补进脏区）；③ 从脏区重新
//              发现组件并重建。
//   删组件    用尾元素搬移：失效组件的空位由数组最后一个元素填上，保持 components 稠密；
//              Delta 按**下标降序**记录被搬走的顺序，反向回放倒着走即可还原。
//   Scratch   复用而非重建：analyze 与 update 不会并发，共用 visited / cellHash / cells；
//              脏标记在 update 结束按脏格清单逐格清掉，不整表 fill。
// ═══════════════════════════════════════════════════════════════════════

struct Structure {
    // 一个格子的邻居上限是 8：Box 尺寸与每条约束引用的 Box 数都受它约束。
    // 一个格子的邻居上限。两条推论都从这里来：
    //   · 一个 Box 至多装 kMaxNeighbors 个格子（同签名 = 同一个数字的邻居）；
    //   · 一条约束至多引用 kMaxNeighbors 个 Box（每个邻居落在一个 Box 里）。
    static constexpr int kMaxNeighbors = 8;

    using ComponentId = int;
    using BoxId = int;
    using ShapeId = int;
    using InstanceId = int;

    struct CellLocation {
        ComponentId component = -1;
        BoxId box = -1;

        bool operator==(const CellLocation &) const = default;
    };

    struct Pool;

    // 与坐标无关的约束形状（按内容 interning 去重）。
    struct Shape {
        // 一个 Box：邻接数字集合完全相同的 H 格集合。
        struct Box {
            int size = 0;
        };

        // 存储态：sum + 池里一段 BoxId 区间的句柄。
        struct Constraint {
            int sum = 0;
            vectorPool<BoxId>::vector boxIds; // 本约束引用到的 Box 集合（池里一段区间）
        };

        // 投影态：同一个 Constraint 解析后的样子。消费者用这个遍历，别自己解句柄。
        struct ConstraintView {
            int sum = 0;
            std::span<const BoxId> boxIds;
        };

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
        // 全部 inline 在头文件里：消费者的内层循环要能真正内联掉它们。
        std::size_t shapeCount() const {
            return shapes_.size();
        }
        int shapeBoxCount(ShapeId shape) const {
            return static_cast<int>(shapes_[shape].boxes_.size);
        }
        int shapeBoxSize(ShapeId shape, BoxId box) const {
            return shapes_[shape].boxes_.span(boxes_)[box].size;
        }
        std::size_t shapeConstraintCount(ShapeId shape) const {
            return shapes_[shape].constraints_.size;
        }
        Shape::ConstraintView shapeConstraint(ShapeId shape, std::size_t index) const {
            const Shape::Constraint &constraint = shapes_[shape].constraints_.span(constraints_)[index];
            return {constraint.sum, constraint.boxIds.span(boxIds_)};
        }

        // ── 读：Instance 侧 ──
        ShapeId instanceShape(InstanceId instance) const {
            return instances_[instance].shape_;
        }
        int instanceBoxCount(InstanceId instance) const {
            return static_cast<int>(instances_[instance].boxOf_.size) - 1;
        }
        int instanceBoxCellCount(InstanceId instance, BoxId box) const {
            const std::span<const int> offsets = instances_[instance].boxOf_.span(boxOf_);
            return offsets[box + 1] - offsets[box];
        }
        ObservedBoard::CellId instanceBoxCell(InstanceId instance, BoxId box, int index) const {
            const std::span<const int> offsets = instances_[instance].boxOf_.span(boxOf_);
            return instances_[instance].cells_.span(cells_)[offsets[box] + index];
        }
        // 循环外提升 base 用；有效期见【Pool：读写边界与 span 生命周期】
        const ObservedBoard::CellId *instanceCellsData(InstanceId instance) const {
            return instances_[instance].cells_.span(cells_).data();
        }
        std::span<const ObservedBoard::CellId> instanceCells(InstanceId instance) const {
            return instances_[instance].cells_.span(cells_);
        }
        std::size_t instanceConstraintCellCount(InstanceId instance) const {
            return instances_[instance].constraintCells_.size;
        }
        ObservedBoard::CellId instanceConstraintCell(InstanceId instance, std::size_t index) const {
            return instances_[instance].constraintCells_.span(constraintCells_)[index];
        }

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

        vectorPool<Shape::Box> boxes_;
        vectorPool<Shape::Constraint> constraints_;
        vectorPool<BoxId> boxIds_;
        vectorPool<ObservedBoard::CellId> cells_;
        vectorPool<int> boxOf_;
        vectorPool<ObservedBoard::CellId> constraintCells_;

        friend struct Structure;
    };

    // analyze / update 的复用缓冲（聚合在 mss::Workspace 的 structure 成员）。
    // analyze 与 update 不会并发，visited / cellHash / cells 由两者共用。
    struct Scratch {
        // ── 组件发现（analyze / update 共用）──
        Grid<char> visited;                       // BFS 去重标记（按格）
        Grid<U128> cellHash;                      // 按格一物两用：先存邻接签名，再改存 BoxId
        std::vector<ObservedBoard::CellId> cells; // 当前组件的格子（BFS 结果，含数字格）

        // ── update 的脏区 ──
        Grid<char> dirty;                         // 脏标记（按格）
        std::vector<ObservedBoard::CellId> dirtyCells; // 脏格清单；invalidate 会边扫边追加
        std::vector<char> removed;                // 按组件下标：本轮是否失效
        std::vector<InstanceId> staged;           // 本轮重建出的实例，最后统一追加

        // ── buildComponent 的临时表（每个组件开工前清空）──
        FlatHashTable<U128, BoxId, U128Hash> hashBox; // 签名 → BoxId
        std::vector<int> boxOfCells;                  // 每个格子 → 它的 BoxId（-1 = 数字格）
        std::vector<std::array<ObservedBoard::CellId, kMaxNeighbors>> buckets; // 按 Box 分桶
        std::vector<std::uint8_t> bucketSize;         // 每桶实际格数（<= kMaxNeighbors）
        std::vector<char> boxUsed;                    // 按 BoxId：当前约束是否已记过它
    };

    // 全量构建：扫出所有组件 → 压缩 Box → 建约束 → 写池 → 铺 cellLoc。
    static Result analyze(const ObservedBoard::Result &board, const Basic::Result &basic, Pool &pool, Scratch &scratch);

    // 增量重建受影响的组件并写出 Delta。调用前 board/basic 必须已同步。
    static void update(Result &result, Delta &delta, const ObservedBoard::Result &board, const Basic::Result &basic,
                       Pool &pool, const ObservedBoard::Delta &updates, Scratch &scratch);

    // 回放：只搬组件与重映射 cellLoc，不重新推理。
    static void applyDelta(Result &result, const Pool &pool, const Delta &delta);
    static void reverseDelta(Result &result, const Pool &pool, const Delta &delta);

  private:
    // 实现细节。必须挂在 Structure 下：需要访问 Shape / Instance / Pool 的私有成员。
    static std::uint64_t positionSeed(int x, int y, int rows, int cols);
    static U128 cellSignature(int x, int y, const ObservedBoard::Result &board);
    static void collectComponent(ObservedBoard::CellId start, const ObservedBoard::Result &board, const Basic::Result &basic,
                                 Grid<char> &visited, std::vector<ObservedBoard::CellId> &cells);
    static InstanceId buildComponent(const std::vector<ObservedBoard::CellId> &cells, const ObservedBoard::Result &board,
                                     const Basic::Result &basic, Grid<U128> &cellHash, Pool &pool, Scratch &scratch);
    static U128 computeHash(const Shape &shape, const Pool &pool);
    static void remapInstance(InstanceId instance, ComponentId component, const Pool &pool, std::vector<CellLocation> &cellLoc);
    static void clearInstance(InstanceId instance, const Pool &pool, std::vector<CellLocation> &cellLoc);
};

} // namespace mss