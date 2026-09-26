#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

#include "algo/probability_engine/basic.h"
#include "core/utility/neighbors.h"
#include "core/utility/flat_hashtable.h"
#include "core/utility/hash.h"
#include "core/utility/vector_pool.h"

namespace mss {

struct Structure {
    using ComponentId = int;
    using BoxId = int;
    using ShapeId = int;
    using InstanceId = int;

    struct CellLocation {
        Structure::ComponentId component = -1;
        Structure::BoxId box = -1;
    };

    // Structure 把 Basic 的 H/数字二部图压缩成独立连通组件；同一组件内，
    // 具有相同数字邻接签名的隐藏格共享一个 Box，后续分布层只枚举 Box 的雷数。

    struct Pool;

    struct Shape {
        struct Box {
            int size = 0;
        };

        struct ConstraintView {
            int sum = 0;
            std::span<const Structure::BoxId> boxIds;
        };

        vectorPool<Box>::vector boxes;
        U128 hash = {};

        struct Constraint {
            int sum = 0;
            std::uint32_t offset = 0;
            std::uint8_t count = 0;
        };

        // 返回该 Shape 保存的数字约束数量。
        std::size_t constraintCount() const {
            return constraints_.size;
        }
        // 返回指定约束的雷数和 Box 成员视图。
        ConstraintView constraint(const Pool &pool, std::size_t i) const;

        vectorPool<Constraint>::vector constraints_;
        vectorPool<Structure::BoxId>::vector boxIds_;
    };

    struct Instance {
        Structure::ShapeId shape = -1;
        struct Boxes {
            vectorPool<ObservedBoard::CellId>::vector cells;
            vectorPool<int>::vector boxOf;

            // 返回实例中的 Box 数量。
            std::size_t count() const {
                return boxOf.size == 0 ? 0 : boxOf.size - 1;
            }
            // 返回指定 Box 包含的真实格子数量。
            std::size_t cellCount(const Pool &pool, std::size_t box) const;
        };

        Boxes boxes;
        vectorPool<ObservedBoard::CellId>::vector constraintCells;
    };

    struct Result {
        std::vector<Structure::InstanceId> components;
        std::vector<Structure::CellLocation> cellLoc;
    };

    struct Delta {
        // removed 按降序记录旧组件下标，forward 回放依赖这个顺序避免尾部搬移覆盖。
        std::vector<Structure::ComponentId> removed;
        std::vector<Structure::InstanceId> removedData;
        std::vector<Structure::ComponentId> added;
        std::vector<Structure::InstanceId> addedData;
    };

    struct Pool {
        vectorPool<Shape::Box> boxes;
        vectorPool<Shape::Constraint> constraints;
        vectorPool<Structure::BoxId> boxIds;
        vectorPool<ObservedBoard::CellId> cells;
        vectorPool<int> boxOf;
        vectorPool<ObservedBoard::CellId> constraintCells;

        // 通过内容哈希插入或复用一个不可变 Shape。
        Structure::ShapeId internShape(Shape shape);
        // 通过内容哈希插入或复用一个不可变 Instance。
        Structure::InstanceId internInstance(Instance instance);
        // 读取 Shape 池中的指定句柄。
        const Shape &getShape(Structure::ShapeId id) const {
            return shapes_[id];
        }
        // 读取 Shape 池中的指定句柄（兼容旧接口名称）。
        const Shape &get(Structure::ShapeId id) const {
            return getShape(id);
        }
        // 读取 Instance 池中的指定句柄。
        const Instance &getInstance(Structure::InstanceId id) const {
            return instances_[id];
        }
        // 返回已缓存 Shape 的数量。
        std::size_t size() const {
            return shapes_.size();
        }
        // 清空逻辑内容并保留所有底层容量。
        void clear();

      private:
        // 计算 Instance 的完整内容哈希，用于布局池去重。
        static U128 computeInstanceHash(const Instance &instance, const Pool &pool);

        std::vector<Shape> shapes_;
        FlatHashTable<U128, Structure::ShapeId, U128Hash> shapeIndex_;
        std::vector<Instance> instances_;
        FlatHashTable<U128, Structure::InstanceId, U128Hash> instanceIndex_;
    };

    using structPool = Pool;

    struct Workspace {
        struct Analyze {
            Grid<char> visited;
            Grid<U128> cellHash;
            std::vector<ObservedBoard::CellId> cells;
        } analyze;

        struct Update {
            Grid<char> visited;
            Grid<U128> cellHash;
            std::vector<ObservedBoard::CellId> cells;
            Grid<char> dirty;
            std::vector<ObservedBoard::CellId> dirtyCells;
            std::vector<char> removed;
            std::vector<Structure::InstanceId> staged;
        } update;

        FlatHashTable<U128, Structure::BoxId, U128Hash> hashBox;
        std::vector<Structure::BoxId> boxOfCells;
        std::vector<std::array<ObservedBoard::CellId, 9>> buckets;
        std::vector<std::uint8_t> bucketSize;
        std::vector<char> boxUsed;
        std::vector<Structure::BoxId> allBoxIds;
    };

  private:
    // 判断观测状态是否为已翻开的数字。
    static bool isNumber(ObservedBoard::CellState state);
    // 将数字观测状态转换为整数值。
    static int numberValue(ObservedBoard::CellState state);
    // 将坐标映射为稳定的位置种子。
    static std::uint64_t positionSeed(int x, int y, int rows, int cols);
    // 计算格子周围数字位置组成的邻接签名。
    static U128 cellSignature(int x, int y, const ObservedBoard::Result &board);
    // 将实例中的格子位置映射到当前组件和 Box。
    static void remapInstance(Structure::InstanceId instance, Structure::ComponentId component, const Pool &pool, std::vector<Structure::CellLocation> &cellLoc);
    // 清除实例在 cellLoc 中留下的组件和 Box 映射。
    static void clearInstance(Structure::InstanceId instance, const Pool &pool, std::vector<Structure::CellLocation> &cellLoc);
    // 从起始格遍历一个数字/H 候选连通组件。
    static void collectComponent(ObservedBoard::CellId start, const ObservedBoard::Result &board, const Basic::Result &basic, Grid<char> &visited,
                                 std::vector<ObservedBoard::CellId> &cells);
    // 根据组件格子构造 Box、数字约束和实例布局并写入池。
    static Structure::InstanceId buildComponent(const std::vector<ObservedBoard::CellId> &cells, const ObservedBoard::Result &board, const Basic::Result &basic,
                                     Grid<U128> &cellHash, Pool &pool, Workspace &workspace);
    // 计算 Shape 内容哈希，用于结构池去重。
    static U128 computeHash(const Shape &shape, const Pool &pool);

  public:
    // 从完整盘面构建所有独立约束组件：数字与 H 候选先按邻接关系连通，再把
    // 邻接签名相同的 H 压成 Box，供 ShapeSolver 枚举 Box 雷数而非逐格枚举。
    static Result analyze(const ObservedBoard::Result &board, const Basic::Result &basic, Pool &pool, Workspace &workspace);

    // 只重建受观测更新影响的组件，并生成结构 Delta；受影响旧组件先整体失效，
    // 再从 dirty 区域发现新组件，保证 cellLoc 与 components 的下标同步。
    static void update(Result &result, Delta &delta, const ObservedBoard::Result &board, const Basic::Result &basic, Pool &pool,
                       const ObservedBoard::Delta &updates, Workspace &workspace);
    // update 只重建受 updates 影响的组件；调用方必须同步更新 board/basic 后再调用。

    // 正向应用或逆向恢复结构组件 Delta；组件删除会用尾元素搬移保持 vector 紧凑，
    // 因而回放顺序和 cellLoc 重映射是这个接口的核心语义。
    static void applyDelta(Result &result, const Pool &pool, const Delta &delta, bool reverse = true);
    // 设计目的：applyDelta 只服务于同一条分析管线的父子 Result 回放；组件删改使用
    // “最后一个元素搬移”维持连续存储，因此调用方必须传入对应的状态。
};

} // namespace mss

//==============================================================================
namespace mss {

inline bool Structure::isNumber(ObservedBoard::CellState state) {
    return (int)(state) <= (int)(ObservedBoard::CellState::Num8);
}

inline int Structure::numberValue(ObservedBoard::CellState state) {
    return (int)(state);
}

inline std::uint64_t Structure::positionSeed(int x, int y, int rows, int cols) {
    return (std::uint64_t)(x) * (cols + rows + 3) + y;
}

inline U128 Structure::cellSignature(int x, int y, const ObservedBoard::Result &board) {
    U128 hash;
    forEachAdjacent(x, y, board.rows, board.cols, [&](int nx, int ny) {
        if (!Structure::isNumber(board.board[nx][ny]))
            return;
        const std::uint64_t position = positionSeed(nx, ny, board.rows, board.cols);
        hash += U128{splitmix64(position), splitmix64(position + 0x9e3779b97f4a7c15ULL)};
    });
    return hash;
}

inline Structure::Shape::ConstraintView Structure::Shape::constraint(const Pool &pool, std::size_t i) const {
    const Constraint &c = constraints_.span(pool.constraints)[i];
    if (c.count == 0)
        return {};
    return {c.sum, boxIds_.span(pool.boxIds).subspan(c.offset, c.count)};
}

inline std::size_t Structure::Instance::Boxes::cellCount(const Pool &pool, std::size_t box) const {
    const std::span<const int> offsets = boxOf.span(pool.boxOf);
    return offsets[box + 1] - offsets[box];
}

inline void Structure::remapInstance(Structure::InstanceId instance, Structure::ComponentId component, const Structure::Pool &pool,
                                     std::vector<Structure::CellLocation> &cellLoc) {
    const Instance &data = pool.getInstance(instance);
    const std::span<const int> boxOf = data.boxes.boxOf.span(pool.boxOf);
    const std::span<const ObservedBoard::CellId> cells = data.boxes.cells.span(pool.cells);
    for (int box = 0; box < (int)(data.boxes.count()); ++box)
        for (int i = boxOf[box]; i < boxOf[box + 1]; ++i)
            cellLoc[cells[i]] = Structure::CellLocation{component, box};
    for (ObservedBoard::CellId cell : data.constraintCells.span(pool.constraintCells))
        cellLoc[cell] = Structure::CellLocation{component, -1};
}

inline void Structure::clearInstance(Structure::InstanceId instance, const Structure::Pool &pool, std::vector<Structure::CellLocation> &cellLoc) {
    const Instance &data = pool.getInstance(instance);
    for (ObservedBoard::CellId cell : data.boxes.cells.span(pool.cells))
        cellLoc[cell] = Structure::CellLocation{};
    for (ObservedBoard::CellId cell : data.constraintCells.span(pool.constraintCells))
        cellLoc[cell] = Structure::CellLocation{};
}

inline Structure::ShapeId Structure::Pool::internShape(Shape shape) {
    shape.hash = Structure::computeHash(shape, *this);
    if (const Structure::ShapeId *found = shapeIndex_.find(shape.hash)) {
        boxes.pop_back(shape.boxes);
        constraints.pop_back(shape.constraints_);
        boxIds.pop_back(shape.boxIds_);
        return *found;
    }
    const Structure::ShapeId id = shapes_.size();
    shapes_.push_back(std::move(shape));
    shapeIndex_.emplace(shapes_[id].hash, id);
    return id;
}

inline Structure::InstanceId Structure::Pool::internInstance(Instance data) {
    const U128 hash = Structure::Pool::computeInstanceHash(data, *this);
    if (const Structure::InstanceId *found = instanceIndex_.find(hash)) {
        cells.pop_back(data.boxes.cells);
        boxOf.pop_back(data.boxes.boxOf);
        constraintCells.pop_back(data.constraintCells);
        return *found;
    }
    const Structure::InstanceId id = instances_.size();
    instances_.push_back(std::move(data));
    instanceIndex_.emplace(hash, id);
    return id;
}

inline void Structure::Pool::clear() {
    shapes_.clear();
    instances_.clear();
    shapeIndex_.clear();
    instanceIndex_.clear();
    boxes.clear();
    constraints.clear();
    boxIds.clear();
    cells.clear();
    boxOf.clear();
    constraintCells.clear();
}

inline U128 Structure::Pool::computeInstanceHash(const Instance &data, const Pool &pool) {
    U128Hasher hasher;
    hasher.mix((std::uint64_t)(data.shape));
    hasher.mix((std::uint64_t)(data.boxes.cells.size));
    for (ObservedBoard::CellId cell : data.boxes.cells.span(pool.cells))
        hasher.mix(cell);
    hasher.mix((std::uint64_t)(data.boxes.boxOf.size));
    for (int offset : data.boxes.boxOf.span(pool.boxOf))
        hasher.mix((std::uint64_t)(offset));
    hasher.mix((std::uint64_t)(data.constraintCells.size));
    for (ObservedBoard::CellId cell : data.constraintCells.span(pool.constraintCells))
        hasher.mix(cell);
    return hasher.finalize();
}

inline Structure::Result Structure::analyze(const ObservedBoard::Result &board, const Basic::Result &basic, Pool &pool,
                                            Structure::Workspace &workspace) {
    const int rows = board.rows;
    const int cols = board.cols;
    Result result;
    result.cellLoc.assign((rows + 1) * (cols + 1), Structure::CellLocation{});

    if (workspace.analyze.visited.rows() != rows || workspace.analyze.visited.cols() != cols) {
        workspace.analyze.visited.resize(rows, cols, 0);
        workspace.analyze.cellHash.resize(rows, cols, U128{});
        workspace.analyze.cells.reserve(rows * cols / 2);
    } else {
        workspace.analyze.visited.fill(0);
        workspace.analyze.cellHash.fill(U128{});
    }
    for (int x = 1; x <= rows; ++x)
        for (int y = 1; y <= cols; ++y)
            if (Structure::isNumber(board.board[x][y])) {
                const std::uint64_t position = Structure::positionSeed(x, y, rows, cols);
                const U128 seed{splitmix64(position), splitmix64(position + 0x9e3779b97f4a7c15ULL)};
                forEachAdjacent(x, y, rows, cols, [&](int nx, int ny) {
                    workspace.analyze.cellHash[nx][ny] += seed;
                });
            }
    for (int x = 1; x <= rows; ++x)
        for (int y = 1; y <= cols; ++y)
            if (basic.marks[x][y] == Basic::Mark::H && !workspace.analyze.visited[x][y]) {
                workspace.analyze.cells.clear();
                Structure::collectComponent(board.id(x, y), board, basic, workspace.analyze.visited, workspace.analyze.cells);
                result.components.push_back(Structure::buildComponent(workspace.analyze.cells, board, basic,
                                                                      workspace.analyze.cellHash, pool, workspace));
            }
    for (Structure::ComponentId component = 0; component < (int)(result.components.size()); ++component)
        Structure::remapInstance(result.components[component], component, pool, result.cellLoc);
    return result;
}

inline void Structure::collectComponent(ObservedBoard::CellId start, const ObservedBoard::Result &board, const Basic::Result &basic, Grid<char> &visited,
                                        std::vector<ObservedBoard::CellId> &cells) {
    const auto [startX, startY] = board.pos(start);
    visited[startX][startY] = 1;
    cells.push_back(start);
    for (int i = 0; i < (int)(cells.size()); ++i) {
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

inline Structure::InstanceId Structure::buildComponent(const std::vector<ObservedBoard::CellId> &cells, const ObservedBoard::Result &board,
                                            const Basic::Result &basic, Grid<U128> &cellHash, Pool &pool, Workspace &workspace) {
    workspace.hashBox.clear();
    workspace.boxOfCells.assign(cells.size(), -1);
    Shape shape;
    shape.boxes = pool.boxes.push_back();
    shape.constraints_ = pool.constraints.push_back();
    shape.boxIds_ = pool.boxIds.push_back();
    for (int i = 0; i < (int)(cells.size()); ++i) {
        const auto [x, y] = board.pos(cells[i]);
        if (basic.marks[x][y] != Basic::Mark::H)
            continue;
        const U128 hash = cellHash[x][y];
        Structure::BoxId box;
        if (const Structure::BoxId *found = workspace.hashBox.find(hash))
            box = *found;
        else {
            box = shape.boxes.size;
            shape.boxes.push_back(pool.boxes, {0});
            workspace.hashBox.emplace(hash, box);
        }
        ++shape.boxes.span(pool.boxes)[box].size;
        workspace.boxOfCells[i] = box;
        cellHash[x][y] = U128{(std::uint64_t)(box), 0};
    }
    workspace.bucketSize.assign(shape.boxes.size, 0);
    if ((int)(workspace.buckets.size()) < shape.boxes.size)
        workspace.buckets.resize(shape.boxes.size);
    for (int i = 0; i < (int)(cells.size()); ++i) {
        const Structure::BoxId box = workspace.boxOfCells[i];
        if (box == -1)
            continue;
        workspace.buckets[box][workspace.bucketSize[box]++] = cells[i];
    }
    Instance instance;
    instance.boxes.cells = pool.cells.push_back();
    instance.boxes.boxOf = pool.boxOf.push_back();
    instance.constraintCells = pool.constraintCells.push_back();
    int boxOffset = 0;
    instance.boxes.boxOf.push_back(pool.boxOf, 0);
    for (int box = 0; box < shape.boxes.size; ++box) {
        for (int i = 0; i < workspace.bucketSize[box]; ++i)
            instance.boxes.cells.push_back(pool.cells, workspace.buckets[box][i]);
        boxOffset += workspace.bucketSize[box];
        instance.boxes.boxOf.push_back(pool.boxOf, boxOffset);
    }
    workspace.boxUsed.assign(shape.boxes.size, 0);
    workspace.allBoxIds.clear();
    for (ObservedBoard::CellId cell : cells) {
        const auto [x, y] = board.pos(cell);
        if (!Structure::isNumber(board.board[x][y]))
            continue;
        int sum = Structure::numberValue(board.board[x][y]);
        const std::uint32_t start = workspace.allBoxIds.size();
        forEachAdjacent(x, y, board.rows, board.cols, [&](int nx, int ny) {
            if (basic.marks[nx][ny] == Basic::Mark::F)
                --sum;
            if (basic.marks[nx][ny] != Basic::Mark::H)
                return;
            const Structure::BoxId box = cellHash[nx][ny].lo;
            if (!workspace.boxUsed[box]) {
                workspace.boxUsed[box] = 1;
                workspace.allBoxIds.push_back(box);
            }
        });
        for (int i = start; i < (int)(workspace.allBoxIds.size()); ++i)
            workspace.boxUsed[workspace.allBoxIds[i]] = 0;
        const std::uint8_t count = workspace.allBoxIds.size() - start;
        shape.constraints_.push_back(pool.constraints, {sum, start, count});
        instance.constraintCells.push_back(pool.constraintCells, cell);
    }
    for (Structure::BoxId box : workspace.allBoxIds)
        shape.boxIds_.push_back(pool.boxIds, box);
    instance.shape = pool.internShape(std::move(shape));
    return pool.internInstance(std::move(instance));
}

inline U128 Structure::computeHash(const Shape &shape, const Pool &pool) {
    U128Hasher hasher;
    for (const Shape::Box &box : shape.boxes.span(pool.boxes))
        hasher.mix((std::uint64_t)(box.size));
    for (int i = 0; i < shape.constraints_.size; ++i) {
        const Shape::Constraint &constraint = shape.constraints_.span(pool.constraints)[i];
        hasher.mix((std::uint64_t)(constraint.sum));
        for (std::uint32_t k = 0; k < constraint.count; ++k)
            hasher.mix((std::uint64_t)(shape.boxIds_.span(pool.boxIds)[constraint.offset + k]) + 0x9e3779b9ULL);
    }
    return hasher.finalize();
}

inline void Structure::update(Result &result, Delta &delta, const ObservedBoard::Result &board, const Basic::Result &basic, Pool &pool,
                              const ObservedBoard::Delta &updates, Structure::Workspace &workspace) {
    const int rows = board.rows;
    const int cols = board.cols;
    Workspace::Update &scratch = workspace.update;
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
        if (scratch.dirty[x][y])
            return;
        scratch.dirty[x][y] = 1;
        scratch.dirtyCells.push_back(board.id(x, y));
    };
    auto invalidate = [&](Structure::ComponentId component) {
        scratch.removed[component] = 1;
        const Instance &instance = pool.getInstance(result.components[component]);
        for (ObservedBoard::CellId cell : instance.boxes.cells.span(pool.cells)) {
            const auto [x, y] = board.pos(cell);
            markDirty(x, y);
            result.cellLoc[cell] = Structure::CellLocation{};
        }
        for (ObservedBoard::CellId cell : instance.constraintCells.span(pool.constraintCells)) {
            const auto [x, y] = board.pos(cell);
            markDirty(x, y);
            result.cellLoc[cell] = Structure::CellLocation{};
        }
    };
    for (const ObservedBoard::Change &change : updates.changes) {
        const auto [x, y] = board.pos(change.cell);
        markDirty(x, y);
        forEachAdjacent(x, y, rows, cols, [&](int nx, int ny) {
            markDirty(nx, ny);
        });
    }
    for (int i = 0; i < (int)(scratch.dirtyCells.size()); ++i) {
        const Structure::CellLocation location = result.cellLoc[scratch.dirtyCells[i]];
        if (location.component == -1 || scratch.removed[location.component])
            continue;
        invalidate(location.component);
    }
    scratch.visited.fill(0);
    auto hashAt = [&](int x, int y) {
        return Structure::cellSignature(x, y, board);
    };
    for (int i = 0; i < (int)(scratch.dirtyCells.size()); ++i) {
        const ObservedBoard::CellId start = scratch.dirtyCells[i];
        const auto [x, y] = board.pos(start);
        if (basic.marks[x][y] != Basic::Mark::H || scratch.visited[x][y])
            continue;
        scratch.cells.clear();
        Structure::collectComponent(start, board, basic, scratch.visited, scratch.cells);
        for (ObservedBoard::CellId cell : scratch.cells) {
            const auto [cx, cy] = board.pos(cell);
            if (basic.marks[cx][cy] == Basic::Mark::H)
                scratch.cellHash[cx][cy] = hashAt(cx, cy);
        }
        scratch.staged.push_back(Structure::buildComponent(scratch.cells, board, basic, scratch.cellHash, pool, workspace));
    }
    delta.removed.clear();
    delta.removedData.clear();
    delta.added.clear();
    delta.addedData.clear();
    for (int i = (int)(result.components.size()) - 1; i >= 0; --i) {
        if (!scratch.removed[i])
            continue;
        delta.removed.push_back(i);
        delta.removedData.push_back(result.components[i]);
        const Structure::ComponentId last = (Structure::ComponentId)(result.components.size()) - 1;
        if (i != last) {
            result.components[i] = std::move(result.components[last]);
            scratch.removed[i] = scratch.removed[last];
            Structure::remapInstance(result.components[i], i, pool, result.cellLoc);
        }
        result.components.pop_back();
    }
    for (Structure::InstanceId instance : scratch.staged) {
        const Structure::ComponentId component = result.components.size();
        result.components.push_back(std::move(instance));
        delta.added.push_back(component);
        delta.addedData.push_back(result.components.back());
        Structure::remapInstance(result.components.back(), component, pool, result.cellLoc);
    }
    for (ObservedBoard::CellId cell : scratch.dirtyCells) {
        const auto [x, y] = board.pos(cell);
        scratch.dirty[x][y] = 0;
        scratch.visited[x][y] = 0;
        scratch.cellHash[x][y] = U128{};
    }
}

inline void Structure::applyDelta(Result &result, const Pool &pool, const Delta &delta, bool reverse) {
    if (reverse) {
        for (int i = delta.addedData.size(); i-- > 0;) {
            Structure::clearInstance(result.components.back(), pool, result.cellLoc);
            result.components.pop_back();
        }
        for (int i = delta.removed.size(); i-- > 0;) {
            const Structure::ComponentId component = delta.removed[i];
            const Structure::ComponentId tail = result.components.size();
            if (component != tail) {
                result.components.push_back(std::move(result.components[component]));
                Structure::remapInstance(result.components.back(), tail, pool, result.cellLoc);
            }
            if (component == tail)
                result.components.push_back(delta.removedData[i]);
            else
                result.components[component] = delta.removedData[i];
            Structure::remapInstance(result.components[component], component, pool, result.cellLoc);
        }
        return;
    }
    for (Structure::ComponentId component : delta.removed) {
        Structure::clearInstance(result.components[component], pool, result.cellLoc);
        const Structure::ComponentId last = (Structure::ComponentId)(result.components.size()) - 1;
        if (component != last) {
            result.components[component] = std::move(result.components[last]);
            Structure::remapInstance(result.components[component], component, pool, result.cellLoc);
        }
        result.components.pop_back();
    }
    for (Structure::InstanceId instance : delta.addedData) {
        const Structure::ComponentId component = result.components.size();
        result.components.push_back(instance);
        Structure::remapInstance(result.components.back(), component, pool, result.cellLoc);
    }
}

} // namespace mss
