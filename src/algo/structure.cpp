#include "algo/structure.h"

#include <utility>

namespace mss {

namespace {

using State = ObservedBoard::CellState;

static thread_local Structure::Workspace workspace;

bool isNumber(State state) {
    return static_cast<int>(state) <= static_cast<int>(State::Num8);
}

int numberValue(State state) {
    return static_cast<int>(state);
}

std::uint64_t positionSeed(int x, int y, int rows, int cols) {
    return static_cast<std::uint64_t>(x) * (cols + rows + 3) + y;
}

U128 cellSignature(int x, int y, const ObservedBoard::Result& board) {
    U128 hash;
    forEachAdjacent(x, y, board.rows, board.cols, [&](int nx, int ny) {
        if (!isNumber(board.board[nx][ny])) return;
        const std::uint64_t position = positionSeed(nx, ny, board.rows, board.cols);
        hash += U128{splitmix64(position),
                     splitmix64(position + 0x9e3779b97f4a7c15ULL)};
    });
    return hash;
}

void remapInstance(const Structure::Instance& instance, ComponentId component,
                   std::vector<CellLocation>& cellLoc) {
    for (std::size_t box = 0; box < instance.boxes.count(); ++box)
        for (std::size_t i = instance.boxes.boxOf[box];
             i < instance.boxes.boxOf[box + 1]; ++i)
            cellLoc[instance.boxes.cells[i]] =
                CellLocation{component, static_cast<BoxId>(box)};
    for (CellId cell : instance.constraintCells)
        cellLoc[cell] = CellLocation{component, -1};
}

void clearInstance(const Structure::Instance& instance,
                   std::vector<CellLocation>& cellLoc) {
    for (CellId cell : instance.boxes.cells) cellLoc[cell] = CellLocation{};
    for (CellId cell : instance.constraintCells) cellLoc[cell] = CellLocation{};
}

}  // namespace

namespace Structure {

void collectComponent(CellId start, const ObservedBoard::Result& board,
                      const Basic::Result& basic, Grid<char>& visited,
                      std::vector<CellId>& cells);
Instance buildComponent(const std::vector<CellId>& cells,
                        const ObservedBoard::Result& board,
                        const Basic::Result& basic, Grid<U128>& cellHash,
                        ShapePool& pool);
U128 computeHash(const Shape& shape);

ShapeId ShapePool::intern(Shape shape) {
    shape.hash = computeHash(shape);
    if (const ShapeId* found = index_.find(shape.hash)) return *found;

    const ShapeId id = static_cast<ShapeId>(shapes_.size());
    shapes_.push_back(std::move(shape));
    index_.emplace(shapes_[id].hash, id);
    return id;
}

Result analyze(const ObservedBoard::Result& board, const Basic::Result& basic,
               ShapePool& pool) {
    const int rows = board.rows;
    const int cols = board.cols;
    Result result;
    result.cellLoc.assign((rows + 1) * (cols + 1), CellLocation{});

    if (workspace.analyze.visited.rows() != rows ||
        workspace.analyze.visited.cols() != cols) {
        workspace.analyze.visited.resize(rows, cols, 0);
        workspace.analyze.cellHash.resize(rows, cols, U128{});
        workspace.analyze.cells.reserve(rows * cols / 2);
    } else {
        workspace.analyze.visited.fill(0);
        workspace.analyze.cellHash.fill(U128{});
    }

    for (int x = 1; x <= rows; ++x)
        for (int y = 1; y <= cols; ++y)
            if (isNumber(board.board[x][y])) {
                const std::uint64_t position = positionSeed(x, y, rows, cols);
                const U128 seed{splitmix64(position),
                                splitmix64(position + 0x9e3779b97f4a7c15ULL)};
                forEachAdjacent(x, y, rows, cols, [&](int nx, int ny) {
                    workspace.analyze.cellHash[nx][ny] += seed;
                });
            }

    for (int x = 1; x <= rows; ++x)
        for (int y = 1; y <= cols; ++y)
            if (basic.marks[x][y] == Basic::Mark::H &&
                !workspace.analyze.visited[x][y]) {
                workspace.analyze.cells.clear();
                collectComponent(board.id(x, y), board, basic,
                                 workspace.analyze.visited,
                                 workspace.analyze.cells);
                result.components.push_back(buildComponent(
                    workspace.analyze.cells, board, basic,
                    workspace.analyze.cellHash, pool));
            }

    for (ComponentId component = 0;
         component < static_cast<ComponentId>(result.components.size()); ++component)
        remapInstance(result.components[component], component, result.cellLoc);

    return result;
}

void collectComponent(CellId start, const ObservedBoard::Result& board,
                      const Basic::Result& basic, Grid<char>& visited,
                      std::vector<CellId>& cells) {
    const auto [startX, startY] = board.pos(start);
    visited[startX][startY] = 1;
    cells.push_back(start);

    for (std::size_t i = 0; i < cells.size(); ++i) {
        const auto [x, y] = board.pos(cells[i]);
        if (isNumber(board.board[x][y]))
            forEachAdjacent(x, y, board.rows, board.cols, [&](int nx, int ny) {
                if (basic.marks[nx][ny] == Basic::Mark::H &&
                    !visited[nx][ny]) {
                    visited[nx][ny] = 1;
                    cells.push_back(board.id(nx, ny));
                }
            });
        if (basic.marks[x][y] == Basic::Mark::H)
            forEachAdjacent(x, y, board.rows, board.cols, [&](int nx, int ny) {
                if (isNumber(board.board[nx][ny]) && !visited[nx][ny]) {
                    visited[nx][ny] = 1;
                    cells.push_back(board.id(nx, ny));
                }
            });
    }
}

Instance buildComponent(const std::vector<CellId>& cells,
                        const ObservedBoard::Result& board,
                        const Basic::Result& basic, Grid<U128>& cellHash,
                        ShapePool& pool) {
    workspace.hashBox.clear();
    workspace.boxOfCells.assign(cells.size(), -1);
    Shape shape;

    for (std::size_t i = 0; i < cells.size(); ++i) {
        const auto [x, y] = board.pos(cells[i]);
        if (basic.marks[x][y] != Basic::Mark::H) continue;

        const U128 hash = cellHash[x][y];
        BoxId box;
        if (const BoxId* found = workspace.hashBox.find(hash)) {
            box = *found;
        } else {
            box = static_cast<BoxId>(shape.boxes.size());
            shape.boxes.push_back({0});
            workspace.hashBox.emplace(hash, box);
        }
        ++shape.boxes[box].size;
        workspace.boxOfCells[i] = box;
        cellHash[x][y] = U128{static_cast<std::uint64_t>(box), 0};
    }

    workspace.bucketSize.assign(shape.boxes.size(), 0);
    if (workspace.buckets.size() < shape.boxes.size())
        workspace.buckets.resize(shape.boxes.size());

    std::size_t numberCells = 0;
    for (std::size_t i = 0; i < cells.size(); ++i) {
        const BoxId box = workspace.boxOfCells[i];
        if (box == -1) {
            ++numberCells;
            continue;
        }
        workspace.buckets[box][workspace.bucketSize[box]++] = cells[i];
    }

    Instance instance;
    instance.boxes.boxOf.resize(shape.boxes.size() + 1);
    for (std::size_t box = 0; box < shape.boxes.size(); ++box)
        instance.boxes.boxOf[box + 1] =
            static_cast<std::uint16_t>(instance.boxes.boxOf[box] +
                                       workspace.bucketSize[box]);
    instance.boxes.cells.resize(instance.boxes.boxOf.back());
    workspace.boxCursor.assign(shape.boxes.size(), 0);
    for (std::size_t i = 0; i < cells.size(); ++i) {
        const BoxId box = workspace.boxOfCells[i];
        if (box == -1) continue;
        instance.boxes.cells[instance.boxes.boxOf[box] + workspace.boxCursor[box]++] =
            cells[i];
    }

    shape.constraints_.reserve(numberCells);
    instance.constraintCells.reserve(numberCells);
    workspace.boxUsed.assign(shape.boxes.size(), 0);
    workspace.allBoxIds.clear();
    for (CellId cell : cells) {
        const auto [x, y] = board.pos(cell);
        if (!isNumber(board.board[x][y])) continue;

        int sum = numberValue(board.board[x][y]);
        const std::uint32_t start =
            static_cast<std::uint32_t>(workspace.allBoxIds.size());
        forEachAdjacent(x, y, board.rows, board.cols, [&](int nx, int ny) {
            if (basic.marks[nx][ny] == Basic::Mark::F) --sum;
            if (basic.marks[nx][ny] != Basic::Mark::H) return;
            const BoxId box = static_cast<BoxId>(cellHash[nx][ny].lo);
            if (!workspace.boxUsed[box]) {
                workspace.boxUsed[box] = 1;
                workspace.allBoxIds.push_back(box);
            }
        });
        for (std::size_t i = start; i < workspace.allBoxIds.size(); ++i)
            workspace.boxUsed[workspace.allBoxIds[i]] = 0;
        shape.constraints_.push_back({
            sum, start,
            static_cast<std::uint8_t>(workspace.allBoxIds.size() - start)});
        instance.constraintCells.push_back(cell);
    }
    shape.boxIds_.insert(shape.boxIds_.end(), workspace.allBoxIds.begin(),
                         workspace.allBoxIds.end());

    instance.shape = pool.intern(std::move(shape));
    return instance;
}

U128 computeHash(const Shape& shape) {
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

Delta update(const ObservedBoard::Result& board, const Basic::Result& basic,
             Result& result, ShapePool& pool,
             const ObservedBoard::Delta& updates) {
    const int rows = board.rows;
    const int cols = board.cols;
    auto& scratch = workspace.update;

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
        const Instance& instance = result.components[component];
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
        forEachAdjacent(x, y, rows, cols,
                        [&](int nx, int ny) { markDirty(nx, ny); });
    }

    for (std::size_t i = 0; i < scratch.dirtyCells.size(); ++i) {
        const CellLocation location = result.cellLoc[scratch.dirtyCells[i]];
        if (location.component == -1 || scratch.removed[location.component]) continue;
        invalidate(location.component);
    }

    scratch.visited.fill(0);
    auto hashAt = [&](int x, int y) { return cellSignature(x, y, board); };
    for (std::size_t i = 0; i < scratch.dirtyCells.size(); ++i) {
        const CellId start = scratch.dirtyCells[i];
        const auto [x, y] = board.pos(start);
        if (basic.marks[x][y] != Basic::Mark::H || scratch.visited[x][y]) continue;

        scratch.cells.clear();
        collectComponent(start, board, basic, scratch.visited, scratch.cells);
        for (CellId cell : scratch.cells) {
            const auto [cx, cy] = board.pos(cell);
            if (basic.marks[cx][cy] == Basic::Mark::H)
                scratch.cellHash[cx][cy] = hashAt(cx, cy);
        }
        scratch.staged.push_back(buildComponent(
            scratch.cells, board, basic, scratch.cellHash, pool));
    }

    Delta delta;
    for (int i = static_cast<int>(result.components.size()) - 1; i >= 0; --i) {
        if (!scratch.removed[i]) continue;
        delta.removed.push_back(i);
        delta.removedData.push_back(result.components[i]);
        const ComponentId last =
            static_cast<ComponentId>(result.components.size()) - 1;
        if (i != last) {
            result.components[i] = std::move(result.components[last]);
            scratch.removed[i] = scratch.removed[last];
            remapInstance(result.components[i], i, result.cellLoc);
        }
        result.components.pop_back();
    }

    for (Instance& instance : scratch.staged) {
        const ComponentId component =
            static_cast<ComponentId>(result.components.size());
        result.components.push_back(std::move(instance));
        delta.added.push_back(component);
        delta.addedData.push_back(result.components.back());
        remapInstance(result.components.back(), component, result.cellLoc);
    }

    for (CellId cell : scratch.dirtyCells) {
        const auto [x, y] = board.pos(cell);
        scratch.dirty[x][y] = 0;
        scratch.visited[x][y] = 0;
        scratch.cellHash[x][y] = U128{};
    }
    return delta;
}

void applyDelta(Result& result, const Delta& delta, bool reverse) {
    if (reverse) {
        for (std::size_t i = delta.addedData.size(); i-- > 0;) {
            clearInstance(result.components.back(), result.cellLoc);
            result.components.pop_back();
        }
        for (std::size_t i = delta.removed.size(); i-- > 0;) {
            const ComponentId component = delta.removed[i];
            const ComponentId tail =
                static_cast<ComponentId>(result.components.size());
            if (component != tail) {
                result.components.push_back(std::move(result.components[component]));
                remapInstance(result.components.back(), tail, result.cellLoc);
            }
            if (component == tail)
                result.components.push_back(delta.removedData[i]);
            else
                result.components[component] = delta.removedData[i];
            remapInstance(result.components[component], component, result.cellLoc);
        }
        return;
    }

    for (ComponentId component : delta.removed) {
        clearInstance(result.components[component], result.cellLoc);
        const ComponentId last =
            static_cast<ComponentId>(result.components.size()) - 1;
        if (component != last) {
            result.components[component] = std::move(result.components[last]);
            remapInstance(result.components[component], component, result.cellLoc);
        }
        result.components.pop_back();
    }
    for (const Instance& instance : delta.addedData) {
        const ComponentId component =
            static_cast<ComponentId>(result.components.size());
        result.components.push_back(instance);
        remapInstance(result.components.back(), component, result.cellLoc);
    }
}

}  // namespace Structure

}  // namespace mss
