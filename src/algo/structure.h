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

        std::size_t constraintCount() const { return constraints_.size(); }
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
            // 累计偏移使用 16 位；极端组件超过 65535 个格子时布局无法表示。
            std::vector<std::uint16_t> boxOf;

            std::size_t count() const { return boxOf.empty() ? 0 : boxOf.size() - 1; }
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
        std::vector<ComponentId> removed;
        std::vector<InstanceId> removedData;
        std::vector<ComponentId> added;
        std::vector<InstanceId> addedData;
    };

    struct Pool {
        ShapeId internShape(Shape shape);
        InstanceId internInstance(Instance instance);
        const Shape& getShape(ShapeId id) const { return shapes_[id]; }
        const Shape& get(ShapeId id) const { return getShape(id); }
        const Instance& getInstance(InstanceId id) const {
            return instances_[id];
        }
        std::size_t size() const { return shapes_.size(); }

    private:
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

    static bool isNumber(ObservedBoard::CellState state);
    static int numberValue(ObservedBoard::CellState state);
    static std::uint64_t positionSeed(int x, int y, int rows, int cols);
    static U128 cellSignature(int x, int y,
                              const ObservedBoard::Result& board);
    static void remapInstance(InstanceId instance, ComponentId component,
                              const Pool& pool,
                              std::vector<CellLocation>& cellLoc);
    static void clearInstance(InstanceId instance,
                              const Pool& pool,
                              std::vector<CellLocation>& cellLoc);

    static void collectComponent(CellId start, const ObservedBoard::Result& board,
                                 const Basic::Result& basic, Grid<char>& visited,
                                 std::vector<CellId>& cells);
    static InstanceId buildComponent(const std::vector<CellId>& cells,
                                   const ObservedBoard::Result& board,
                                   const Basic::Result& basic, Grid<U128>& cellHash,
                                   Pool& pool);
    static U128 computeHash(const Shape& shape);

public:
    static Result analyze(const ObservedBoard::Result& board,
                          const Basic::Result& basic, Pool& pool);

    static void update(Result& result, Delta& delta,
                       const ObservedBoard::Result& board,
                       const Basic::Result& basic, Pool& pool,
                       const ObservedBoard::Delta& updates);

    static void applyDelta(Result& result, const Pool& pool, const Delta& delta,
                           bool reverse = true);

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
