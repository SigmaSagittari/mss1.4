#pragma once

#include <array>
#include <span>
#include <vector>

#include "algo/shape_solver/shape_solver_common.h"
#include "core/workspace.h"

namespace mss {

struct ShapeSolver::DfsSolver {
  private:
    using AssignmentWorkspace = workspace::DfsSolver::ForEachAssignment;
    using AnalyzeWorkspace = workspace::DfsSolver::Analyze;

  public:
    // 回调收到一个 Box->雷数赋值及其具体布局权重；weight 是各 Box 内 C(size,k)
    // 的乘积，调用方用它累计 ways/矩。回调期间 assignment 有效，返回后不能保存 span。
    template <typename Callback>
    static void forEachAssignment(const Structure::Shape &shape, const Structure::Pool &shapes, Callback &&callback);

    static DistributionId analyze(const Structure::Shape &shape, const Structure::Pool &shapes, Distribution::Pool &pool);
};

template <typename Callback>
inline void mss::ShapeSolver::DfsSolver::forEachAssignment(const Structure::Shape &shape, const Structure::Pool &shapes,
                                                           Callback &&callback) {
    // 深度优先枚举满足全部约束的 Box 雷数赋值；每加入一个 Box 就用当前和与
    // 剩余容量剪枝，因此 callback 只看合法 assignment，不需要再次检查约束。
    AssignmentWorkspace &workspace = workspace::DfsSolver::forEachAssignment;
    const int boxCount = shape.boxes.size;
    const int constraintCount = shape.constraintCount();

    workspace.boxHead.assign(boxCount, -1);
    workspace.constraintNext.clear();
    workspace.constraintIds.clear();
    workspace.constraintSum.assign(constraintCount, 0);
    workspace.constraintMaxAdd.assign(constraintCount, 0);
    workspace.currentSum.assign(constraintCount, 0);
    workspace.assignedSize.assign(constraintCount, 0);
    workspace.assignment.assign(boxCount, 0);

    std::size_t incidenceCount = 0;
    for (int constraint = 0; constraint < constraintCount; ++constraint)
        incidenceCount += shape.constraint(shapes, constraint).boxIds.size();
    workspace.constraintNext.reserve(incidenceCount);
    workspace.constraintIds.reserve(incidenceCount);

    for (int constraint = 0; constraint < constraintCount; ++constraint) {
        const Structure::Shape::ConstraintView view = shape.constraint(shapes, constraint);
        workspace.constraintSum[constraint] = view.sum;
        for (BoxId box : view.boxIds) {
            workspace.constraintMaxAdd[constraint] += shape.boxes.span(shapes.boxes)[box].size;
            workspace.constraintNext.push_back(workspace.boxHead[box]);
            workspace.constraintIds.push_back(constraint);
            workspace.boxHead[box] = workspace.constraintIds.size() - 1;
        }
    }

    workspace.frames.clear();
    workspace.frames.reserve(boxCount + 1);
    workspace.frames.push_back({0, 0, -1, 0, 1.0L});

    while (!workspace.frames.empty()) {
        AssignmentWorkspace::Frame &frame = workspace.frames.back();
        if (frame.index == boxCount) {
            callback(std::span<const char>(workspace.assignment), frame.ways);
            const int appliedIndex = frame.appliedIndex;
            const int appliedMine = frame.appliedMine;
            workspace.frames.pop_back();
            if (appliedIndex >= 0) {
                const int maxMine = shape.boxes.span(shapes.boxes)[appliedIndex].size;
                for (int link = workspace.boxHead[appliedIndex]; link >= 0; link = workspace.constraintNext[link]) {
                    const int constraint = workspace.constraintIds[link];
                    workspace.currentSum[constraint] -= appliedMine;
                    workspace.assignedSize[constraint] -= maxMine;
                }
            }
            continue;
        }

        const int index = frame.index;
        const int maxMine = shape.boxes.span(shapes.boxes)[index].size;
        if (frame.nextMine > maxMine) {
            const int appliedIndex = frame.appliedIndex;
            const int appliedMine = frame.appliedMine;
            workspace.frames.pop_back();
            if (appliedIndex >= 0) {
                const int appliedMaxMine = shape.boxes.span(shapes.boxes)[appliedIndex].size;
                for (int link = workspace.boxHead[appliedIndex]; link >= 0; link = workspace.constraintNext[link]) {
                    const int constraint = workspace.constraintIds[link];
                    workspace.currentSum[constraint] -= appliedMine;
                    workspace.assignedSize[constraint] -= appliedMaxMine;
                }
            }
            continue;
        }

        const int mine = frame.nextMine++;
        workspace.assignment[index] = mine;
        bool valid = true;
        for (int link = workspace.boxHead[index]; link >= 0; link = workspace.constraintNext[link]) {
            const int constraint = workspace.constraintIds[link];
            const int sum = workspace.currentSum[constraint] + mine;
            const int remaining = workspace.constraintMaxAdd[constraint] - (workspace.assignedSize[constraint] + maxMine);
            if (sum > workspace.constraintSum[constraint] || sum + remaining < workspace.constraintSum[constraint]) {
                valid = false;
                break;
            }
        }
        if (!valid)
            continue;

        for (int link = workspace.boxHead[index]; link >= 0; link = workspace.constraintNext[link]) {
            const int constraint = workspace.constraintIds[link];
            workspace.currentSum[constraint] += mine;
            workspace.assignedSize[constraint] += maxMine;
        }
        workspace.frames.push_back({index + 1, 0, index, mine, frame.ways * ShapeSolver::binom(maxMine, mine)});
    }
}

inline DistributionId mss::ShapeSolver::DfsSolver::analyze(const Structure::Shape &shape, const Structure::Pool &shapes,
                                                           Distribution::Pool &pool) {
    // 汇总每个合法赋值在各总雷数下的布局权重和 Box 期望雷数，再压缩掉空的
    // 雷数区间并交给 Distribution::Pool 缓存。
    const DistributionId cached = pool.find(shape.hash);
    if (cached >= 0)
        return cached;

    const int boxCount = shape.boxes.size;
    int maxMineCount = 0;
    for (const Structure::Shape::Box &box : shape.boxes.span(shapes.boxes))
        maxMineCount += box.size;

    AnalyzeWorkspace &analyzeWorkspace = workspace::DfsSolver::analyze;
    std::vector<long double> &ways = analyzeWorkspace.ways;
    RawGrid<long double> &moments = analyzeWorkspace.moments;
    ways.assign(maxMineCount + 1, 0.0L);
    moments.resize(maxMineCount + 1, boxCount, 0.0L);
    forEachAssignment(shape, shapes, [&](std::span<const char> assignment, long double weight) {
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
