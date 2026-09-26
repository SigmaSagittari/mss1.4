#pragma once

#include <array>
#include <span>
#include <vector>

#include "algo/probability_engine/shape_solver/shape_solver_common.h"

namespace mss {

namespace ShapeSolver {

struct DfsSolver {
    using Workspace = ShapeSolver::Workspace::Dfs;

  private:
  public:
    // 回调收到一个 Box->雷数赋值及其具体布局权重；weight 是各 Box 内 C(size,k)
    // 的乘积，调用方用它累计 ways/矩。回调期间 assignment 有效，返回后不能保存 span。
    template <typename Callback>
    static void forEachAssignment(const Structure::Shape &shape, const Structure::Pool &shapes, Workspace &workspace,
                                  Callback &&callback);

    static ShapeSolver::DistributionId analyze(const Structure::Shape &shape, const Structure::Pool &shapes, Distribution::Pool &pool,
                                  Workspace &workspace);
};

} // namespace ShapeSolver

template <typename Callback>
inline void mss::ShapeSolver::DfsSolver::forEachAssignment(const Structure::Shape &shape, const Structure::Pool &shapes,
                                                           Workspace &workspace, Callback &&callback) {
    // 深度优先枚举满足全部约束的 Box 雷数赋值；每加入一个 Box 就用当前和与
    // 剩余容量剪枝，因此 callback 只看合法 assignment，不需要再次检查约束。
    Workspace::ForEachAssignment &ws = workspace.forEachAssignment;
    const int boxCount = shape.boxes.size;
    const int constraintCount = shape.constraintCount();

    ws.boxHead.assign(boxCount, -1);
    ws.constraintNext.clear();
    ws.constraintIds.clear();
    ws.constraintSum.assign(constraintCount, 0);
    ws.constraintMaxAdd.assign(constraintCount, 0);
    ws.currentSum.assign(constraintCount, 0);
    ws.assignedSize.assign(constraintCount, 0);
    ws.assignment.assign(boxCount, 0);

    std::size_t incidenceCount = 0;
    for (int constraint = 0; constraint < constraintCount; ++constraint)
        incidenceCount += shape.constraint(shapes, constraint).boxIds.size();
    ws.constraintNext.reserve(incidenceCount);
    ws.constraintIds.reserve(incidenceCount);

    for (int constraint = 0; constraint < constraintCount; ++constraint) {
        const Structure::Shape::ConstraintView view = shape.constraint(shapes, constraint);
        ws.constraintSum[constraint] = view.sum;
        for (Structure::BoxId box : view.boxIds) {
            ws.constraintMaxAdd[constraint] += shape.boxes.span(shapes.boxes)[box].size;
            ws.constraintNext.push_back(ws.boxHead[box]);
            ws.constraintIds.push_back(constraint);
            ws.boxHead[box] = ws.constraintIds.size() - 1;
        }
    }

    ws.frames.clear();
    ws.frames.reserve(boxCount + 1);
    ws.frames.push_back({0, 0, -1, 0, 1.0L});

    while (!ws.frames.empty()) {
        Workspace::ForEachAssignment::Frame &frame = ws.frames.back();
        if (frame.index == boxCount) {
            callback(std::span<const char>(ws.assignment), frame.ways);
            const int appliedIndex = frame.appliedIndex;
            const int appliedMine = frame.appliedMine;
            ws.frames.pop_back();
            if (appliedIndex >= 0) {
                const int maxMine = shape.boxes.span(shapes.boxes)[appliedIndex].size;
                for (int link = ws.boxHead[appliedIndex]; link >= 0; link = ws.constraintNext[link]) {
                    const int constraint = ws.constraintIds[link];
                    ws.currentSum[constraint] -= appliedMine;
                    ws.assignedSize[constraint] -= maxMine;
                }
            }
            continue;
        }

        const int index = frame.index;
        const int maxMine = shape.boxes.span(shapes.boxes)[index].size;
        if (frame.nextMine > maxMine) {
            const int appliedIndex = frame.appliedIndex;
            const int appliedMine = frame.appliedMine;
            ws.frames.pop_back();
            if (appliedIndex >= 0) {
                const int appliedMaxMine = shape.boxes.span(shapes.boxes)[appliedIndex].size;
                for (int link = ws.boxHead[appliedIndex]; link >= 0; link = ws.constraintNext[link]) {
                    const int constraint = ws.constraintIds[link];
                    ws.currentSum[constraint] -= appliedMine;
                    ws.assignedSize[constraint] -= appliedMaxMine;
                }
            }
            continue;
        }

        const int mine = frame.nextMine++;
        ws.assignment[index] = mine;
        bool valid = true;
        for (int link = ws.boxHead[index]; link >= 0; link = ws.constraintNext[link]) {
            const int constraint = ws.constraintIds[link];
            const int sum = ws.currentSum[constraint] + mine;
            const int remaining = ws.constraintMaxAdd[constraint] - (ws.assignedSize[constraint] + maxMine);
            if (sum > ws.constraintSum[constraint] || sum + remaining < ws.constraintSum[constraint]) {
                valid = false;
                break;
            }
        }
        if (!valid)
            continue;

        for (int link = ws.boxHead[index]; link >= 0; link = ws.constraintNext[link]) {
            const int constraint = ws.constraintIds[link];
            ws.currentSum[constraint] += mine;
            ws.assignedSize[constraint] += maxMine;
        }
        ws.frames.push_back({index + 1, 0, index, mine, frame.ways * binomSmall(maxMine, mine)});
    }
}

inline ShapeSolver::DistributionId mss::ShapeSolver::DfsSolver::analyze(const Structure::Shape &shape, const Structure::Pool &shapes,
                                                           Distribution::Pool &pool, Workspace &workspace) {
    // 汇总每个合法赋值在各总雷数下的布局权重和 Box 期望雷数，再压缩掉空的
    // 雷数区间并交给 Distribution::Pool 缓存。
    const ShapeSolver::DistributionId cached = pool.find(shape.hash);
    if (cached >= 0)
        return cached;

    const int boxCount = shape.boxes.size;
    int maxMineCount = 0;
    for (const Structure::Shape::Box &box : shape.boxes.span(shapes.boxes))
        maxMineCount += box.size;

    std::vector<long double> &ways = workspace.analyze.ways;
    RawGrid<long double> &moments = workspace.analyze.moments;
    ways.assign(maxMineCount + 1, 0.0L);
    moments.resize(maxMineCount + 1, boxCount, 0.0L);
    forEachAssignment(shape, shapes, workspace, [&](std::span<const char> assignment, long double weight) {
        int mineCount = 0;
        for (char mine : assignment)
            mineCount += mine;
        ways[mineCount] += weight;
        for (int box = 0; box < boxCount; ++box)
            moments[mineCount][box] += weight * assignment[box];
    });

    int start = 0;
    while (start <= maxMineCount && ways[start] == 0.0L)
        ++start;
    if (start > maxMineCount)
        return pool.insert(shape.hash, Distribution::Result(0, boxCount, {}, {}));

    int end = maxMineCount;
    while (ways[end] == 0.0L)
        --end;
    std::vector<long double> compactWays(ways.begin() + start, ways.begin() + end + 1);
    RawGrid<long double> expectation((int)(compactWays.size()), boxCount, 0.0L);
    for (int mineCount = start; mineCount <= end; ++mineCount) {
        if (ways[mineCount] == 0.0L)
            continue;
        for (int box = 0; box < boxCount; ++box)
            expectation[mineCount - start][box] = moments[mineCount][box] / ways[mineCount];
    }
    Distribution::Result result(start, boxCount, std::move(compactWays), std::move(expectation));
    return pool.insert(shape.hash, std::move(result));
}

} // namespace mss
