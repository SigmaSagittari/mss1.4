#include "structure/structure.h"

#include "core/assert.h"
#include "core/utility/neighborhood.h"

namespace mss {

// 邻接签名的混合步长：与 splitmix64 用的常数不同，避免两个通道相关。
static constexpr std::uint64_t kMixStride = 0x9e3779b97f4a7c15ULL;

// ── 文件级工具 ──

static bool isNumber(ObservedBoard::CellState state) {
    return static_cast<int>(state) <= static_cast<int>(ObservedBoard::CellState::Num8);
}

static int numberValue(ObservedBoard::CellState state) {
    return static_cast<int>(state);
}

// ── Pool：访问器以外的部分 ──

Structure::ShapeId Structure::Pool::internShape(Shape shape) {
    shape.hash_ = Structure::computeHash(shape, *this);
    if (const ShapeId *found = shapeIndex_.find(shape.hash_)) {
        // 去重命中：把刚写进段里的内容撤回（只弹尾部）。
        boxes_.pop_back(shape.boxes_);
        constraints_.pop_back(shape.constraints_);
        boxIds_.pop_back(shape.boxIds_);
        return *found;
    }
    const ShapeId id = static_cast<ShapeId>(shapes_.size());
    shapes_.push_back(std::move(shape));
    shapeIndex_.emplace(shapes_[static_cast<std::size_t>(id)].hash_, id);
    return id;
}

Structure::InstanceId Structure::Pool::internInstance(Instance instance) {
    const U128 hash = Pool::computeInstanceHash(instance, *this);
    if (const InstanceId *found = instanceIndex_.find(hash)) {
        cells_.pop_back(instance.cells_);
        boxOf_.pop_back(instance.boxOf_);
        constraintCells_.pop_back(instance.constraintCells_);
        return *found;
    }
    const InstanceId id = static_cast<InstanceId>(instances_.size());
    instances_.push_back(std::move(instance));
    instanceIndex_.emplace(hash, id);
    return id;
}

U128 Structure::Pool::computeInstanceHash(const Instance &instance, const Pool &pool) {
    U128Hasher hasher;
    hasher.mix(static_cast<std::uint64_t>(instance.shape_));
    hasher.mix(static_cast<std::uint64_t>(instance.cells_.size));
    for (ObservedBoard::CellId cell : instance.cells_.span(pool.cells_))
        hasher.mix(static_cast<std::uint64_t>(cell));
    hasher.mix(static_cast<std::uint64_t>(instance.boxOf_.size));
    for (int offset : instance.boxOf_.span(pool.boxOf_))
        hasher.mix(static_cast<std::uint64_t>(offset));
    hasher.mix(static_cast<std::uint64_t>(instance.constraintCells_.size));
    for (ObservedBoard::CellId cell : instance.constraintCells_.span(pool.constraintCells_))
        hasher.mix(static_cast<std::uint64_t>(cell));
    return hasher.finalize();
}

void Structure::Pool::clear() {
    shapes_.clear();
    instances_.clear();
    shapeIndex_.clear();
    instanceIndex_.clear();
    boxes_.clear();
    constraints_.clear();
    boxIds_.clear();
    cells_.clear();
    boxOf_.clear();
    constraintCells_.clear();
}

// ── 私有实现 ──

std::uint64_t Structure::positionSeed(int x, int y, int rows, int cols) {
    // 0-based 下仍对盘面内所有 (x,y) 单射：乘子大于最大列号。
    return static_cast<std::uint64_t>(x) * static_cast<std::uint64_t>(cols + rows + 3) + static_cast<std::uint64_t>(y);
}

U128 Structure::cellSignature(int x, int y, const ObservedBoard::Result &board) {
    U128 hash;
    forEachAdjacent(x, y, board.rows, board.cols, [&](int nx, int ny) {
        if (!isNumber(board.board[nx][ny]))
            return;
        const std::uint64_t position = Structure::positionSeed(nx, ny, board.rows, board.cols);
        hash += U128{splitmix64(position), splitmix64(position + kMixStride)};
    });
    return hash;
}

U128 Structure::computeHash(const Shape &shape, const Pool &pool) {
    U128Hasher hasher;
    const std::span<const Shape::Box> boxes = shape.boxes_.span(pool.boxes_);
    hasher.mix(static_cast<std::uint64_t>(boxes.size()));
    for (const Shape::Box &box : boxes)
        hasher.mix(static_cast<std::uint64_t>(box.size));
    const std::span<const Shape::Constraint> constraints = shape.constraints_.span(pool.constraints_);
    hasher.mix(static_cast<std::uint64_t>(constraints.size()));
    for (const Shape::Constraint &constraint : constraints) {
        hasher.mix(static_cast<std::uint64_t>(constraint.sum));
        const std::span<const BoxId> boxIds = constraint.boxIds.span(pool.boxIds_);
        hasher.mix(static_cast<std::uint64_t>(boxIds.size()));
        for (BoxId box : boxIds)
            hasher.mix(static_cast<std::uint64_t>(box) + kMixStride);
    }
    return hasher.finalize();
}

void Structure::collectComponent(ObservedBoard::CellId start, const ObservedBoard::Result &board, const Basic::Result &basic,
                                 Grid<char> &visited, std::vector<ObservedBoard::CellId> &cells) {
    const auto [startX, startY] = board.pos(start);
    visited[startX][startY] = 1;
    cells.push_back(start);
    for (std::size_t i = 0; i < cells.size(); ++i) {
        const auto [x, y] = board.pos(cells[i]);
        if (isNumber(board.board[x][y]))
            forEachAdjacent(x, y, board.rows, board.cols, [&](int nx, int ny) {
                if (basic.marks[nx][ny] == Basic::Mark::H && !visited[nx][ny]) {
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

Structure::InstanceId Structure::buildComponent(const std::vector<ObservedBoard::CellId> &cells, const ObservedBoard::Result &board,
                                                const Basic::Result &basic, Grid<U128> &cellHash, Pool &pool, Scratch &scratch) {
    scratch.hashBox.clear();
    scratch.boxOfCells.assign(cells.size(), -1);

    Shape shape;
    shape.boxes_ = pool.boxes_.push_back();
    shape.constraints_ = pool.constraints_.push_back();
    shape.boxIds_ = pool.boxIds_.push_back();

    // 1) 按邻接签名把 H 格分组为 Box。签名相等 => 邻接数字集合相同 => 同一个 Box。
    for (std::size_t i = 0; i < cells.size(); ++i) {
        const auto [x, y] = board.pos(cells[i]);
        if (basic.marks[x][y] != Basic::Mark::H)
            continue;
        const U128 hash = cellHash[x][y];
        BoxId box = -1;
        if (const BoxId *found = scratch.hashBox.find(hash)) {
            box = *found;
        } else {
            box = static_cast<BoxId>(shape.boxes_.size);
            shape.boxes_.push_back(pool.boxes_, Shape::Box{0});
            scratch.hashBox.emplace(hash, box);
        }
        ++shape.boxes_.span(pool.boxes_)[static_cast<std::size_t>(box)].size;
        assert_(shape.boxes_.span(pool.boxes_)[static_cast<std::size_t>(box)].size <= kMaxBoxSize,
                "Structure::buildComponent: Box 尺寸超过 8（同签名格子不可能超过一个数字的邻居数）");
        scratch.boxOfCells[i] = box;
        // 这一格的签名槽改存它的 BoxId（lo 位），供约束构造回查。
        cellHash[x][y] = U128{static_cast<std::uint64_t>(box), 0};
    }

    // 2) 按 Box 把格子分桶（每桶 <= kMaxBoxSize）。
    scratch.bucketSize.assign(shape.boxes_.size, 0);
    if (scratch.buckets.size() < static_cast<std::size_t>(shape.boxes_.size))
        scratch.buckets.resize(shape.boxes_.size);
    for (std::size_t i = 0; i < cells.size(); ++i) {
        const BoxId box = scratch.boxOfCells[i];
        if (box == -1)
            continue;
        scratch.buckets[static_cast<std::size_t>(box)][scratch.bucketSize[static_cast<std::size_t>(box)]++] = cells[i];
    }

    // 3) 写实例：按 Box 顺序连续存放格子 + 偏移表。
    Instance instance;
    instance.cells_ = pool.cells_.push_back();
    instance.boxOf_ = pool.boxOf_.push_back();
    instance.constraintCells_ = pool.constraintCells_.push_back();
    int boxOffset = 0;
    instance.boxOf_.push_back(pool.boxOf_, 0);
    for (BoxId box = 0; box < static_cast<BoxId>(shape.boxes_.size); ++box) {
        const std::uint8_t size = scratch.bucketSize[static_cast<std::size_t>(box)];
        for (std::uint8_t i = 0; i < size; ++i)
            instance.cells_.push_back(pool.cells_, scratch.buckets[static_cast<std::size_t>(box)][i]);
        boxOffset += size;
        instance.boxOf_.push_back(pool.boxOf_, boxOffset);
    }

    // 4) 每个数字格一条约束：sum 扣除邻域内已确定的雷，boxIds 记邻接到的 Box（去重）。
    scratch.boxUsed.assign(shape.boxes_.size, 0);
    for (ObservedBoard::CellId cell : cells) {
        const auto [x, y] = board.pos(cell);
        if (!isNumber(board.board[x][y]))
            continue;
        int sum = numberValue(board.board[x][y]);
        vectorPool<BoxId>::vector boxIds = pool.boxIds_.push_back();
        forEachAdjacent(x, y, board.rows, board.cols, [&](int nx, int ny) {
            if (basic.marks[nx][ny] == Basic::Mark::F)
                --sum;
            if (basic.marks[nx][ny] != Basic::Mark::H)
                return;
            const BoxId box = static_cast<BoxId>(cellHash[nx][ny].lo);
            if (!scratch.boxUsed[static_cast<std::size_t>(box)]) {
                scratch.boxUsed[static_cast<std::size_t>(box)] = 1;
                boxIds.push_back(pool.boxIds_, box);
            }
        });
        for (BoxId box : boxIds.span(pool.boxIds_))
            scratch.boxUsed[static_cast<std::size_t>(box)] = 0;
        assert_(boxIds.size <= kMaxConstraintBoxes,
                "Structure::buildComponent: 约束引用的 Box 数超过 8（一个数字最多 8 个邻居）");
        shape.constraints_.push_back(pool.constraints_, Shape::Constraint{sum, boxIds});
        instance.constraintCells_.push_back(pool.constraintCells_, cell);
    }

    instance.shape_ = pool.internShape(std::move(shape));
    return pool.internInstance(std::move(instance));
}

void Structure::remapInstance(InstanceId instance, ComponentId component, const Pool &pool, std::vector<CellLocation> &cellLoc) {
    const int boxCount = pool.instanceBoxCount(instance);
    for (BoxId box = 0; box < boxCount; ++box) {
        const int count = pool.instanceBoxCellCount(instance, box);
        for (int i = 0; i < count; ++i)
            cellLoc[pool.instanceBoxCell(instance, box, i)] = CellLocation{component, box};
    }
    const std::size_t constraintCount = pool.instanceConstraintCellCount(instance);
    for (std::size_t i = 0; i < constraintCount; ++i)
        cellLoc[pool.instanceConstraintCell(instance, i)] = CellLocation{component, -1};
}

void Structure::clearInstance(InstanceId instance, const Pool &pool, std::vector<CellLocation> &cellLoc) {
    for (ObservedBoard::CellId cell : pool.instanceCells(instance))
        cellLoc[cell] = CellLocation{};
    const std::size_t constraintCount = pool.instanceConstraintCellCount(instance);
    for (std::size_t i = 0; i < constraintCount; ++i)
        cellLoc[pool.instanceConstraintCell(instance, i)] = CellLocation{};
}

// ── 公开接口 ──

Structure::Result Structure::analyze(const ObservedBoard::Result &board, const Basic::Result &basic, Pool &pool, Scratch &scratch) {
    const int rows = board.rows;
    const int cols = board.cols;
    Result result;
    result.cellLoc.assign(static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols), CellLocation{});

    if (scratch.visited.rows() != rows || scratch.visited.cols() != cols) {
        scratch.visited.resize(rows, cols, 0);
        scratch.cellHash.resize(rows, cols, U128{});
        scratch.cells.reserve(static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols) / 2);
    } else {
        scratch.visited.fill(0);
        scratch.cellHash.fill(U128{});
    }

    // 每个数字格把自己的位置种子（经 splitmix）加进八邻域 -> 每格的邻接数字签名。
    for (int x = 0; x < rows; ++x)
        for (int y = 0; y < cols; ++y)
            if (isNumber(board.board[x][y])) {
                const std::uint64_t position = positionSeed(x, y, rows, cols);
                const U128 seed{splitmix64(position), splitmix64(position + kMixStride)};
                forEachAdjacent(x, y, rows, cols, [&](int nx, int ny) { scratch.cellHash[nx][ny] += seed; });
            }

    for (int x = 0; x < rows; ++x)
        for (int y = 0; y < cols; ++y)
            if (basic.marks[x][y] == Basic::Mark::H && !scratch.visited[x][y]) {
                scratch.cells.clear();
                collectComponent(board.id(x, y), board, basic, scratch.visited, scratch.cells);
                result.components.push_back(buildComponent(scratch.cells, board, basic, scratch.cellHash, pool, scratch));
            }

    for (ComponentId component = 0; component < static_cast<ComponentId>(result.components.size()); ++component)
        remapInstance(result.components[static_cast<std::size_t>(component)], component, pool, result.cellLoc);
    return result;
}

void Structure::update(Result &result, Delta &delta, const ObservedBoard::Result &board, const Basic::Result &basic, Pool &pool,
                       const ObservedBoard::Delta &updates, Scratch &scratch) {
    const int rows = board.rows;
    const int cols = board.cols;
    if (scratch.dirty.rows() != rows || scratch.dirty.cols() != cols) {
        scratch.dirty.resize(rows, cols, 0);
        scratch.visited.resize(rows, cols, 0);
        scratch.cellHash.resize(rows, cols, U128{});
        scratch.cells.reserve(static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols) / 2);
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

    // 让一个组件整体失效：抹掉它的 cellLoc 映射，并把它所有格子标脏。
    auto invalidate = [&](ComponentId component) {
        scratch.removed[static_cast<std::size_t>(component)] = 1;
        const InstanceId instance = result.components[static_cast<std::size_t>(component)];
        const int boxCount = pool.instanceBoxCount(instance);
        for (BoxId box = 0; box < boxCount; ++box) {
            const int count = pool.instanceBoxCellCount(instance, box);
            for (int i = 0; i < count; ++i) {
                const ObservedBoard::CellId cell = pool.instanceBoxCell(instance, box, i);
                const auto [x, y] = board.pos(cell);
                markDirty(x, y);
                result.cellLoc[cell] = CellLocation{};
            }
        }
        const std::size_t constraintCount = pool.instanceConstraintCellCount(instance);
        for (std::size_t i = 0; i < constraintCount; ++i) {
            const ObservedBoard::CellId cell = pool.instanceConstraintCell(instance, i);
            const auto [x, y] = board.pos(cell);
            markDirty(x, y);
            result.cellLoc[cell] = CellLocation{};
        }
    };

    // 观测变化的格子与它的邻居是脏区起点。
    for (const ObservedBoard::Change &change : updates.changes) {
        const auto [x, y] = board.pos(change.cell);
        markDirty(x, y);
        forEachAdjacent(x, y, rows, cols, [&](int nx, int ny) { markDirty(nx, ny); });
    }
    // 脏区里凡属于某组件的格子 -> 整个组件失效（invalidate 会继续扩大脏区）。
    for (std::size_t i = 0; i < scratch.dirtyCells.size(); ++i) {
        const CellLocation location = result.cellLoc[scratch.dirtyCells[i]];
        if (location.component == -1 || scratch.removed[static_cast<std::size_t>(location.component)])
            continue;
        invalidate(location.component);
    }

    // 从脏区重新发现组件并重建。
    scratch.visited.fill(0);
    for (std::size_t i = 0; i < scratch.dirtyCells.size(); ++i) {
        const ObservedBoard::CellId start = scratch.dirtyCells[i];
        const auto [x, y] = board.pos(start);
        if (basic.marks[x][y] != Basic::Mark::H || scratch.visited[x][y])
            continue;
        scratch.cells.clear();
        collectComponent(start, board, basic, scratch.visited, scratch.cells);
        for (ObservedBoard::CellId cell : scratch.cells) {
            const auto [cx, cy] = board.pos(cell);
            if (basic.marks[cx][cy] == Basic::Mark::H)
                scratch.cellHash[cx][cy] = cellSignature(cx, cy, board);
        }
        scratch.staged.push_back(buildComponent(scratch.cells, board, basic, scratch.cellHash, pool, scratch));
        for (ObservedBoard::CellId cell : scratch.cells) {
            const auto [cx, cy] = board.pos(cell);
            scratch.cellHash[cx][cy] = U128{};
        }
    }

    // 尾元素搬移删掉失效组件；removed 按**下标降序**记录。
    delta.removed.clear();
    delta.removedData.clear();
    delta.addedData.clear();
    for (ComponentId i = static_cast<ComponentId>(result.components.size()) - 1; i >= 0; --i) {
        if (!scratch.removed[static_cast<std::size_t>(i)])
            continue;
        delta.removed.push_back(i);
        delta.removedData.push_back(result.components[static_cast<std::size_t>(i)]);
        const ComponentId last = static_cast<ComponentId>(result.components.size()) - 1;
        if (i != last) {
            result.components[static_cast<std::size_t>(i)] = result.components[static_cast<std::size_t>(last)];
            scratch.removed[static_cast<std::size_t>(i)] = scratch.removed[static_cast<std::size_t>(last)];
            remapInstance(result.components[static_cast<std::size_t>(i)], i, pool, result.cellLoc);
        }
        result.components.pop_back();
    }
    for (InstanceId instance : scratch.staged) {
        const ComponentId component = static_cast<ComponentId>(result.components.size());
        result.components.push_back(instance);
        delta.addedData.push_back(instance);
        remapInstance(result.components.back(), component, pool, result.cellLoc);
    }

    for (ObservedBoard::CellId cell : scratch.dirtyCells) {
        const auto [x, y] = board.pos(cell);
        scratch.dirty[x][y] = 0;
        scratch.visited[x][y] = 0;
        scratch.cellHash[x][y] = U128{};
    }
}

void Structure::applyDelta(Result &result, const Pool &pool, const Delta &delta) {
    for (ComponentId component : delta.removed) {
        clearInstance(result.components[static_cast<std::size_t>(component)], pool, result.cellLoc);
        const ComponentId last = static_cast<ComponentId>(result.components.size()) - 1;
        if (component != last) {
            result.components[static_cast<std::size_t>(component)] = result.components[static_cast<std::size_t>(last)];
            remapInstance(result.components[static_cast<std::size_t>(component)], component, pool, result.cellLoc);
        }
        result.components.pop_back();
    }
    for (InstanceId instance : delta.addedData) {
        const ComponentId component = static_cast<ComponentId>(result.components.size());
        result.components.push_back(instance);
        remapInstance(result.components.back(), component, pool, result.cellLoc);
    }
}

void Structure::reverseDelta(Result &result, const Pool &pool, const Delta &delta) {
    for (int i = static_cast<int>(delta.addedData.size()); i-- > 0;) {
        clearInstance(result.components.back(), pool, result.cellLoc);
        result.components.pop_back();
    }
    for (int i = static_cast<int>(delta.removed.size()); i-- > 0;) {
        const ComponentId component = delta.removed[static_cast<std::size_t>(i)];
        const ComponentId tail = static_cast<ComponentId>(result.components.size());
        if (component != tail) {
            result.components.push_back(result.components[static_cast<std::size_t>(component)]);
            remapInstance(result.components.back(), tail, pool, result.cellLoc);
        }
        const InstanceId instance = delta.removedData[static_cast<std::size_t>(i)];
        if (component == tail)
            result.components.push_back(instance);
        else
            result.components[static_cast<std::size_t>(component)] = instance;
        remapInstance(result.components[static_cast<std::size_t>(component)], component, pool, result.cellLoc);
    }
}

bool Structure::Result::sameAs(const Result &other) const {
    return components == other.components && cellLoc == other.cellLoc;
}

} // namespace mss