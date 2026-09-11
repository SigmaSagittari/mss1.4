#include "algo/shape_solver/dfs_solver.h"

#include <algorithm>

namespace mss {

namespace ShapeSolver {

namespace DfsSolver {

namespace {

thread_local detail::AssignmentWorkspace workspace;

}  // namespace

namespace detail {

AssignmentWorkspace& assignmentWorkspace() {
    return workspace;
}

}  // namespace detail

DistributionId analyze(const Structure::Shape& shape, Distribution::Pool& pool) {
    const DistributionId cached = pool.find(shape.hash);
    if (cached >= 0) return cached;

    const int boxCount = static_cast<int>(shape.boxes.size());
    int maxMineCount = 0;
    for (const Structure::Shape::Box& box : shape.boxes)
        maxMineCount += box.size;

    thread_local std::vector<long double> ways;
    thread_local std::vector<long double> moments;
    ways.assign(maxMineCount + 1, 0.0L);
    moments.assign((maxMineCount + 1) * boxCount, 0.0L);

    forEachAssignment(shape, [&](std::span<const char> assignment, long double weight) {
        int mineCount = 0;
        for (char mine : assignment) mineCount += mine;
        ways[mineCount] += weight;
        const std::size_t offset = mineCount * boxCount;
        for (int box = 0; box < boxCount; ++box)
            moments[offset + box] += weight * assignment[box];
    });

    int start = 0;
    while (start <= maxMineCount && ways[start] == 0.0L) ++start;
    if (start > maxMineCount)
        return pool.insert(shape.hash, Distribution::Result(
            0, boxCount, {}, {}));

    int end = maxMineCount;
    while (ways[end] == 0.0L) --end;

    std::vector<long double> compactWays(ways.begin() + start,
                                         ways.begin() + end + 1);
    std::vector<long double> expectation(
        compactWays.size() * boxCount, 0.0L);
    for (int mineCount = start; mineCount <= end; ++mineCount) {
        if (ways[mineCount] == 0.0L) continue;
        const std::size_t sourceOffset = mineCount * boxCount;
        const std::size_t targetOffset = (mineCount - start) * boxCount;
        for (int box = 0; box < boxCount; ++box)
            expectation[targetOffset + box] =
                moments[sourceOffset + box] / ways[mineCount];
    }

    Distribution::Result result(start, boxCount, std::move(compactWays),
                                std::move(expectation));
    return pool.insert(shape.hash, std::move(result));
}

}  // namespace DfsSolver

}  // namespace ShapeSolver

}  // namespace mss
