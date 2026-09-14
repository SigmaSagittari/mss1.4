#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

#include "algo/basic.h"
#include "core/types.h"
#include "core/utility/flat_hashtable.h"
#include "core/utility/hash.h"

namespace mss {

struct Structure {

    // Structure 把 Basic 的 H/数字二部图压缩成独立连通组件；同一组件内，
    // 具有相同数字邻接签名的隐藏格共享一个 Box，后续分布层只枚举 Box 的雷数。

    struct Shape {
        struct Box {
            int size = 0;
        };

        struct ConstraintView {
            int sum = 0;
            std::span<const BoxId> boxIds;
        };

        std::vector<Box> boxes;
        U128 hash = {};

        // 返回该 Shape 保存的数字约束数量。
        std::size_t constraintCount() const { return constraints_.size(); }
        // 返回指定约束的雷数和 Box 成员视图。
        ConstraintView constraint(std::size_t i) const {
            const Constraint& c = constraints_[i];
            if (c.count == 0) return {};
            return {c.sum,
                    std::span<const BoxId>(boxIds_.data() + c.offset, c.count)};
        }

        struct Constraint {
            int sum = 0;
            std::uint32_t offset = 0;
            std::uint8_t count = 0;
        };

        std::vector<Constraint> constraints_;
        std::vector<BoxId> boxIds_;
    };

    struct Instance {
        ShapeId shape = -1;
        struct Boxes {
            std::vector<CellId> cells;
            // 设计目的：用 16 位累计偏移压缩组件布局；组件规模受项目盘面约束，
            // 不在此处引入更宽整数，以保持 Instance 的紧凑存储。
            std::vector<std::uint16_t> boxOf;

            // 返回实例中的 Box 数量。
            std::size_t count() const { return boxOf.empty() ? 0 : boxOf.size() - 1; }
            // 返回指定 Box 包含的真实格子数量。
            std::size_t cellCount(std::size_t box) const {
                return boxOf[box + 1] - boxOf[box];
            }
        };

        Boxes boxes;
        std::vector<CellId> constraintCells;
    };

    struct Result {
        std::vector<InstanceId> components;
        std::vector<CellLocation> cellLoc;
    };

    struct Delta {
        // removed 按降序记录旧组件下标，forward 回放依赖这个顺序避免尾部搬移覆盖。
        std::vector<ComponentId> removed;
        std::vector<InstanceId> removedData;
        std::vector<ComponentId> added;
        std::vector<InstanceId> addedData;
    };

    struct Pool {
        // 通过内容哈希插入或复用一个不可变 Shape。
        ShapeId internShape(Shape shape);
        // 通过内容哈希插入或复用一个不可变 Instance。
        InstanceId internInstance(Instance instance);
        // 读取 Shape 池中的指定句柄。
        const Shape& getShape(ShapeId id) const { return shapes_[id]; }
        // 读取 Shape 池中的指定句柄（兼容旧接口名称）。
        const Shape& get(ShapeId id) const { return getShape(id); }
        // 读取 Instance 池中的指定句柄。
        const Instance& getInstance(InstanceId id) const {
            return instances_[id];
        }
        // 返回已缓存 Shape 的数量。
        std::size_t size() const { return shapes_.size(); }

    private:
        // 计算 Instance 的完整内容哈希，用于布局池去重。
        static U128 computeInstanceHash(const Instance& instance);

        std::vector<Shape> shapes_;
        FlatHashTable<U128, ShapeId, U128Hash> shapeIndex_;
        std::vector<Instance> instances_;
        FlatHashTable<U128, InstanceId, U128Hash> instanceIndex_;
    };

    using ShapePool = Pool;

    struct Workspace {
        struct Analyze {
            Grid<char> visited;
            Grid<U128> cellHash;
            std::vector<CellId> cells;
        } analyze;

        struct Update {
            Grid<char> visited;
            Grid<U128> cellHash;
            std::vector<CellId> cells;
            Grid<char> dirty;
            std::vector<CellId> dirtyCells;
            std::vector<char> removed;
            std::vector<InstanceId> staged;
        } update;

        FlatHashTable<U128, BoxId, U128Hash> hashBox;
        std::vector<BoxId> boxOfCells;
        std::vector<std::array<CellId, 9>> buckets;
        std::vector<std::uint8_t> bucketSize;
        std::vector<BoxId> boxCursor;
        std::vector<char> boxUsed;
        std::vector<BoxId> allBoxIds;
    };

private:
    static thread_local Workspace workspace;

    // 判断观测状态是否为已翻开的数字。
    static bool isNumber(ObservedBoard::CellState state);
    // 将数字观测状态转换为整数值。
    static int numberValue(ObservedBoard::CellState state);
    // 将坐标映射为稳定的位置种子。
    static std::uint64_t positionSeed(int x, int y, int rows, int cols);
    // 计算格子周围数字位置组成的邻接签名。
    static U128 cellSignature(int x, int y,
                              const ObservedBoard::Result& board);
    // 将实例中的格子位置映射到当前组件和 Box。
    static void remapInstance(InstanceId instance, ComponentId component,
                              const Pool& pool,
                              std::vector<CellLocation>& cellLoc);
    // 清除实例在 cellLoc 中留下的组件和 Box 映射。
    static void clearInstance(InstanceId instance,
                              const Pool& pool,
                              std::vector<CellLocation>& cellLoc);

    // 从起始格遍历一个数字/H 候选连通组件。
    static void collectComponent(CellId start, const ObservedBoard::Result& board,
                                 const Basic::Result& basic, Grid<char>& visited,
                                 std::vector<CellId>& cells);
    // 根据组件格子构造 Box、数字约束和实例布局并写入池。
    static InstanceId buildComponent(const std::vector<CellId>& cells,
                                   const ObservedBoard::Result& board,
                                   const Basic::Result& basic, Grid<U128>& cellHash,
                                   Pool& pool);
    // 计算 Shape 内容哈希，用于结构池去重。
    static U128 computeHash(const Shape& shape);

public:
    // 从完整盘面构建所有独立约束组件：数字与 H 候选先按邻接关系连通，再把
    // 邻接签名相同的 H 压成 Box，供 ShapeSolver 枚举 Box 雷数而非逐格枚举。
    static Result analyze(const ObservedBoard::Result& board,
                          const Basic::Result& basic, Pool& pool);

    // 只重建受观测更新影响的组件，并生成结构 Delta；受影响旧组件先整体失效，
    // 再从 dirty 区域发现新组件，保证 cellLoc 与 components 的下标同步。
    static void update(Result& result, Delta& delta,
                       const ObservedBoard::Result& board,
                       const Basic::Result& basic, Pool& pool,
                       const ObservedBoard::Delta& updates);
    // update 只重建受 updates 影响的组件；调用方必须同步更新 board/basic 后再调用。

    // 正向应用或逆向恢复结构组件 Delta；组件删除会用尾元素搬移保持 vector 紧凑，
    // 因而回放顺序和 cellLoc 重映射是这个接口的核心语义。
    static void applyDelta(Result& result, const Pool& pool, const Delta& delta,
                           bool reverse = true);
    // 设计目的：applyDelta 只服务于同一条分析管线的父子 Result 回放；组件删改使用
    // “最后一个元素搬移”维持连续存储，因此调用方必须传入对应的状态。

};

}  // namespace mss

//==============================================================================
namespace mss {

inline bool Structure::isNumber(ObservedBoard::CellState state) {
    return static_cast<int>(state) <=
           static_cast<int>(ObservedBoard::CellState::Num8);
}

inline int Structure::numberValue(ObservedBoard::CellState state) {
    return static_cast<int>(state);
}

inline std::uint64_t Structure::positionSeed(int x, int y, int rows, int cols) {
    return static_cast<std::uint64_t>(x) * (cols + rows + 3) + y;
}

inline U128 Structure::cellSignature(int x, int y,
                                     const ObservedBoard::Result& board) {
    U128 hash;
    forEachAdjacent(x, y, board.rows, board.cols, [&](int nx, int ny) {
        if (!Structure::isNumber(board.board[nx][ny])) return;
        const std::uint64_t position = positionSeed(nx, ny, board.rows, board.cols);
        hash += U128{splitmix64(position),
                     splitmix64(position + 0x9e3779b97f4a7c15ULL)};
    });
    return hash;
}

inline void Structure::remapInstance(InstanceId instance,
                                     ComponentId component,
                                     const Structure::Pool& pool,
                                     std::vector<CellLocation>& cellLoc) {
    const Instance& data = pool.getInstance(instance);
    for (std::size_t box = 0; box < data.boxes.count(); ++box)
        for (std::size_t i = data.boxes.boxOf[box];
             i < data.boxes.boxOf[box + 1]; ++i)
            cellLoc[data.boxes.cells[i]] =
                CellLocation{component, static_cast<BoxId>(box)};
    for (CellId cell : data.constraintCells)
        cellLoc[cell] = CellLocation{component, -1};
}

inline void Structure::clearInstance(InstanceId instance,
                                     const Structure::Pool& pool,
                                     std::vector<CellLocation>& cellLoc) {
    const Instance& data = pool.getInstance(instance);
    for (CellId cell : data.boxes.cells) cellLoc[cell] = CellLocation{};
    for (CellId cell : data.constraintCells) cellLoc[cell] = CellLocation{};
}

inline thread_local Structure::Workspace Structure::workspace;

inline ShapeId Structure::Pool::internShape(Shape shape) {
    shape.hash = Structure::computeHash(shape);
    if (const ShapeId* found = shapeIndex_.find(shape.hash)) return *found;
    const ShapeId id = static_cast<ShapeId>(shapes_.size());
    shapes_.push_back(std::move(shape));
    shapeIndex_.emplace(shapes_[id].hash, id);
    return id;
}

inline InstanceId Structure::Pool::internInstance(Instance data) {
    const U128 hash = Structure::Pool::computeInstanceHash(data);
    if (const InstanceId* found = instanceIndex_.find(hash)) return *found;
    const InstanceId id = static_cast<InstanceId>(instances_.size());
    instances_.push_back(std::move(data));
    instanceIndex_.emplace(hash, id);
    return id;
}

inline U128 Structure::Pool::computeInstanceHash(const Instance& data) {
    U128Hasher hasher;
    hasher.mix(static_cast<std::uint64_t>(data.shape));
    hasher.mix(static_cast<std::uint64_t>(data.boxes.cells.size()));
    for (CellId cell : data.boxes.cells) hasher.mix(cell);
    hasher.mix(static_cast<std::uint64_t>(data.boxes.boxOf.size()));
    for (std::uint16_t offset : data.boxes.boxOf)
        hasher.mix(static_cast<std::uint64_t>(offset));
    hasher.mix(static_cast<std::uint64_t>(data.constraintCells.size()));
    for (CellId cell : data.constraintCells) hasher.mix(cell);
    return hasher.finalize();
}

inline Structure::Result Structure::analyze(
    const ObservedBoard::Result& board, const Basic::Result& basic, Pool& pool) {
    const int rows = board.rows;
    const int cols = board.cols;
    Result result;
    result.cellLoc.assign((rows + 1) * (cols + 1), CellLocation{});

    if (Structure::workspace.analyze.visited.rows() != rows ||
        Structure::workspace.analyze.visited.cols() != cols) {
        Structure::workspace.analyze.visited.resize(rows, cols, 0);
        Structure::workspace.analyze.cellHash.resize(rows, cols, U128{});
        Structure::workspace.analyze.cells.reserve(rows * cols / 2);
    } else {
        Structure::workspace.analyze.visited.fill(0);
        Structure::workspace.analyze.cellHash.fill(U128{});
    }
    for (int x = 1; x <= rows; ++x)
        for (int y = 1; y <= cols; ++y)
            if (Structure::isNumber(board.board[x][y])) {
                const std::uint64_t position = Structure::positionSeed(x, y, rows, cols);
                const U128 seed{splitmix64(position),
                                splitmix64(position + 0x9e3779b97f4a7c15ULL)};
                forEachAdjacent(x, y, rows, cols, [&](int nx, int ny) {
                    Structure::workspace.analyze.cellHash[nx][ny] += seed;
                });
            }
    for (int x = 1; x <= rows; ++x)
        for (int y = 1; y <= cols; ++y)
            if (basic.marks[x][y] == Basic::Mark::H &&
                !Structure::workspace.analyze.visited[x][y]) {
                Structure::workspace.analyze.cells.clear();
                Structure::collectComponent(board.id(x, y), board, basic,
                                            Structure::workspace.analyze.visited,
                                            Structure::workspace.analyze.cells);
                result.components.push_back(Structure::buildComponent(
                    Structure::workspace.analyze.cells, board, basic,
                    Structure::workspace.analyze.cellHash, pool));
            }
    for (ComponentId component = 0;
         component < static_cast<ComponentId>(result.components.size()); ++component)
        Structure::remapInstance(result.components[component], component, pool,
                                 result.cellLoc);
    return result;
}

inline void Structure::collectComponent(CellId start,
                                         const ObservedBoard::Result& board,
                                         const Basic::Result& basic,
                                         Grid<char>& visited,
                                         std::vector<CellId>& cells) {
    const auto [startX, startY] = board.pos(start);
    visited[startX][startY] = 1;
    cells.push_back(start);
    for (std::size_t i = 0; i < cells.size(); ++i) {
        const auto [x, y] = board.pos(cells[i]);
        if (Structure::isNumber(board.board[x][y]))
            forEachAdjacent(x, y, board.rows, board.cols, [&](int nx, int ny) {
                if (basic.marks[nx][ny] == Basic::Mark::H && !visited[nx][ny]) {
                    visited[nx][ny] = 1;
                    cells.push_back(board.id(nx, ny));
                }
            });
        if (basic.marks[x][y] == Basic::Mark::H)
            forEachAdjacent(x, y, board.rows, board.cols, [&](int nx, int ny) {
                if (Structure::isNumber(board.board[nx][ny]) && !visited[nx][ny]) {
                    visited[nx][ny] = 1;
                    cells.push_back(board.id(nx, ny));
                }
            });
    }
}

inline InstanceId Structure::buildComponent(
    const std::vector<CellId>& cells, const ObservedBoard::Result& board,
    const Basic::Result& basic, Grid<U128>& cellHash, Pool& pool) {
    Structure::workspace.hashBox.clear();
    Structure::workspace.boxOfCells.assign(cells.size(), -1);
    Shape shape;
    for (std::size_t i = 0; i < cells.size(); ++i) {
        const auto [x, y] = board.pos(cells[i]);
        if (basic.marks[x][y] != Basic::Mark::H) continue;
        const U128 hash = cellHash[x][y];
        BoxId box;
        if (const BoxId* found = Structure::workspace.hashBox.find(hash)) box = *found;
        else {
            box = static_cast<BoxId>(shape.boxes.size());
            shape.boxes.push_back({0});
            Structure::workspace.hashBox.emplace(hash, box);
        }
        ++shape.boxes[box].size;
        Structure::workspace.boxOfCells[i] = box;
        cellHash[x][y] = U128{static_cast<std::uint64_t>(box), 0};
    }
    Structure::workspace.bucketSize.assign(shape.boxes.size(), 0);
    if (Structure::workspace.buckets.size() < shape.boxes.size())
        Structure::workspace.buckets.resize(shape.boxes.size());
    std::size_t numberCells = 0;
    for (std::size_t i = 0; i < cells.size(); ++i) {
        const BoxId box = Structure::workspace.boxOfCells[i];
        if (box == -1) {
            ++numberCells;
            continue;
        }
        Structure::workspace.buckets[box][Structure::workspace.bucketSize[box]++] = cells[i];
    }
    Instance instance;
    instance.boxes.boxOf.resize(shape.boxes.size() + 1);
    for (std::size_t box = 0; box < shape.boxes.size(); ++box)
        instance.boxes.boxOf[box + 1] = static_cast<std::uint16_t>(
            instance.boxes.boxOf[box] + Structure::workspace.bucketSize[box]);
    instance.boxes.cells.resize(instance.boxes.boxOf.back());
    Structure::workspace.boxCursor.assign(shape.boxes.size(), 0);
    for (std::size_t i = 0; i < cells.size(); ++i) {
        const BoxId box = Structure::workspace.boxOfCells[i];
        if (box == -1) continue;
        instance.boxes.cells[instance.boxes.boxOf[box] +
                             Structure::workspace.boxCursor[box]++] =
            cells[i];
    }
    shape.constraints_.reserve(numberCells);
    instance.constraintCells.reserve(numberCells);
    Structure::workspace.boxUsed.assign(shape.boxes.size(), 0);
    Structure::workspace.allBoxIds.clear();
    for (CellId cell : cells) {
        const auto [x, y] = board.pos(cell);
        if (!Structure::isNumber(board.board[x][y])) continue;
        int sum = Structure::numberValue(board.board[x][y]);
        const std::uint32_t start =
            static_cast<std::uint32_t>(Structure::workspace.allBoxIds.size());
        forEachAdjacent(x, y, board.rows, board.cols, [&](int nx, int ny) {
            if (basic.marks[nx][ny] == Basic::Mark::F) --sum;
            if (basic.marks[nx][ny] != Basic::Mark::H) return;
            const BoxId box = static_cast<BoxId>(cellHash[nx][ny].lo);
            if (!Structure::workspace.boxUsed[box]) {
                Structure::workspace.boxUsed[box] = 1;
                Structure::workspace.allBoxIds.push_back(box);
            }
        });
        for (std::size_t i = start; i < Structure::workspace.allBoxIds.size(); ++i)
            Structure::workspace.boxUsed[Structure::workspace.allBoxIds[i]] = 0;
        shape.constraints_.push_back({sum, start, static_cast<std::uint8_t>(
            Structure::workspace.allBoxIds.size() - start)});
        instance.constraintCells.push_back(cell);
    }
    shape.boxIds_.insert(shape.boxIds_.end(), Structure::workspace.allBoxIds.begin(),
                         Structure::workspace.allBoxIds.end());
    instance.shape = pool.internShape(std::move(shape));
    return pool.internInstance(std::move(instance));
}

inline U128 Structure::computeHash(const Shape& shape) {
    U128Hasher hasher;
    for (const Shape::Box& box : shape.boxes)
        hasher.mix(static_cast<std::uint64_t>(box.size));
    for (std::size_t i = 0; i < shape.constraints_.size(); ++i) {
        const Shape::Constraint& constraint = shape.constraints_[i];
        hasher.mix(static_cast<std::uint64_t>(constraint.sum));
        for (std::uint32_t k = 0; k < constraint.count; ++k)
            hasher.mix(static_cast<std::uint64_t>(
                shape.boxIds_[constraint.offset + k]) + 0x9e3779b9ULL);
    }
    return hasher.finalize();
}

inline void Structure::update(
    Result& result, Delta& delta, const ObservedBoard::Result& board,
    const Basic::Result& basic, Pool& pool,
    const ObservedBoard::Delta& updates) {
    const int rows = board.rows;
    const int cols = board.cols;
    auto& scratch = Structure::workspace.update;
    if (scratch.dirty.rows() != rows || scratch.dirty.cols() != cols) {
        scratch.dirty.resize(rows, cols, 0);
        scratch.visited.resize(rows, cols, 0);
        scratch.cellHash.resize(rows, cols, U128{});
        scratch.cells.reserve(rows * cols / 2);
    }
    scratch.dirtyCells.clear();
    scratch.staged.clear();
    scratch.removed.assign(result.components.size(), 0);
    auto markDirty = [&](int x, int y) {
        if (scratch.dirty[x][y]) return;
        scratch.dirty[x][y] = 1;
        scratch.dirtyCells.push_back(board.id(x, y));
    };
    auto invalidate = [&](ComponentId component) {
        scratch.removed[component] = 1;
        const Instance& instance = pool.getInstance(result.components[component]);
        for (CellId cell : instance.boxes.cells) {
            const auto [x, y] = board.pos(cell);
            markDirty(x, y);
            result.cellLoc[cell] = CellLocation{};
        }
        for (CellId cell : instance.constraintCells) {
            const auto [x, y] = board.pos(cell);
            markDirty(x, y);
            result.cellLoc[cell] = CellLocation{};
        }
    };
    for (const ObservedBoard::Change& change : updates.changes) {
        const auto [x, y] = board.pos(change.cell);
        markDirty(x, y);
        forEachAdjacent(x, y, rows, cols, [&](int nx, int ny) { markDirty(nx, ny); });
    }
    for (std::size_t i = 0; i < scratch.dirtyCells.size(); ++i) {
        const CellLocation location = result.cellLoc[scratch.dirtyCells[i]];
        if (location.component == -1 || scratch.removed[location.component]) continue;
        invalidate(location.component);
    }
    scratch.visited.fill(0);
    auto hashAt = [&](int x, int y) { return Structure::cellSignature(x, y, board); };
    for (std::size_t i = 0; i < scratch.dirtyCells.size(); ++i) {
        const CellId start = scratch.dirtyCells[i];
        const auto [x, y] = board.pos(start);
        if (basic.marks[x][y] != Basic::Mark::H || scratch.visited[x][y]) continue;
        scratch.cells.clear();
        Structure::collectComponent(start, board, basic, scratch.visited,
                                    scratch.cells);
        for (CellId cell : scratch.cells) {
            const auto [cx, cy] = board.pos(cell);
            if (basic.marks[cx][cy] == Basic::Mark::H)
                scratch.cellHash[cx][cy] = hashAt(cx, cy);
        }
        scratch.staged.push_back(Structure::buildComponent(
            scratch.cells, board, basic, scratch.cellHash, pool));
    }
    delta.removed.clear();
    delta.removedData.clear();
    delta.added.clear();
    delta.addedData.clear();
    for (int i = static_cast<int>(result.components.size()) - 1; i >= 0; --i) {
        if (!scratch.removed[i]) continue;
        delta.removed.push_back(i);
        delta.removedData.push_back(result.components[i]);
        const ComponentId last = static_cast<ComponentId>(result.components.size()) - 1;
        if (i != last) {
            result.components[i] = std::move(result.components[last]);
            scratch.removed[i] = scratch.removed[last];
            Structure::remapInstance(result.components[i], i, pool, result.cellLoc);
        }
        result.components.pop_back();
    }
    for (InstanceId instance : scratch.staged) {
        const ComponentId component = static_cast<ComponentId>(result.components.size());
        result.components.push_back(std::move(instance));
        delta.added.push_back(component);
        delta.addedData.push_back(result.components.back());
        Structure::remapInstance(result.components.back(), component, pool,
                                 result.cellLoc);
    }
    for (CellId cell : scratch.dirtyCells) {
        const auto [x, y] = board.pos(cell);
        scratch.dirty[x][y] = 0;
        scratch.visited[x][y] = 0;
        scratch.cellHash[x][y] = U128{};
    }
}

inline void Structure::applyDelta(Result& result, const Pool& pool,
                                  const Delta& delta, bool reverse) {
    if (reverse) {
        for (std::size_t i = delta.addedData.size(); i-- > 0;) {
            Structure::clearInstance(result.components.back(), pool, result.cellLoc);
            result.components.pop_back();
        }
        for (std::size_t i = delta.removed.size(); i-- > 0;) {
            const ComponentId component = delta.removed[i];
            const ComponentId tail = static_cast<ComponentId>(result.components.size());
            if (component != tail) {
                result.components.push_back(std::move(result.components[component]));
                Structure::remapInstance(result.components.back(), tail, pool,
                                         result.cellLoc);
            }
            if (component == tail) result.components.push_back(delta.removedData[i]);
            else result.components[component] = delta.removedData[i];
            Structure::remapInstance(result.components[component], component, pool,
                                     result.cellLoc);
        }
        return;
    }
    for (ComponentId component : delta.removed) {
        Structure::clearInstance(result.components[component], pool, result.cellLoc);
        const ComponentId last = static_cast<ComponentId>(result.components.size()) - 1;
        if (component != last) {
            result.components[component] = std::move(result.components[last]);
            Structure::remapInstance(result.components[component], component, pool,
                                     result.cellLoc);
        }
        result.components.pop_back();
    }
    for (InstanceId instance : delta.addedData) {
        const ComponentId component = static_cast<ComponentId>(result.components.size());
        result.components.push_back(instance);
        Structure::remapInstance(result.components.back(), component, pool,
                                 result.cellLoc);
    }
}

}  // namespace mss
