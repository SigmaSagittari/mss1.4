#pragma once

#include <array>
#include <span>
#include <vector>

#include "algo/shape_solver/shape_solver.h"

namespace mss {

namespace ShapeSolver {

namespace DfsSolver {

namespace detail {

struct AssignmentWorkspace {
    struct Frame {
        int index = 0;
        int nextMine = 0;
        int appliedIndex = -1;
        int appliedMine = 0;
        long double ways = 0;
    };

    std::vector<int> boxHead;
    std::vector<int> constraintNext;
    std::vector<int> constraintIds;
    std::vector<int> constraintSum;
    std::vector<int> constraintMaxAdd;
    std::vector<int> currentSum;
    std::vector<int> assignedSize;
    std::vector<char> assignment;
    std::vector<Frame> frames;
};

AssignmentWorkspace& assignmentWorkspace();

inline long double binom(int n, int k) {
    constexpr int max = 9;
    static constexpr std::array<std::array<long double, max + 1>, max + 1> table = [] {
        std::array<std::array<long double, max + 1>, max + 1> result{};
        for (int i = 0; i <= max; ++i) {
            result[i][0] = 1;
            result[i][i] = 1;
            for (int j = 1; j < i; ++j)
                result[i][j] = result[i - 1][j - 1] + result[i - 1][j];
        }
        return result;
    }();
    return table[n][k];
}

}  // namespace detail

// 枚举满足 Shape 约束的 box 雷数分配。
// callback 接收按 BoxId 编号的只读 assignment，以及该分配的组合权重。
template <typename Callback>
void forEachAssignment(const Structure::Shape& shape, Callback&& callback);

// DFS 后端的普通分布求解。
DistributionId analyze(const Structure::Shape& shape, Distribution::Pool& pool);

}  // namespace DfsSolver

}  // namespace ShapeSolver

}  // namespace mss

template <typename Callback>
inline void mss::ShapeSolver::DfsSolver::forEachAssignment(
    const Structure::Shape& shape, Callback&& callback) {
    detail::AssignmentWorkspace& workspace = detail::assignmentWorkspace();
    const int boxCount = static_cast<int>(shape.boxes.size());
    const int constraintCount = static_cast<int>(shape.constraintCount());

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
        incidenceCount += shape.constraint(constraint).boxIds.size();
    workspace.constraintNext.reserve(incidenceCount);
    workspace.constraintIds.reserve(incidenceCount);

    for (int constraint = 0; constraint < constraintCount; ++constraint) {
        const Structure::Shape::ConstraintView view = shape.constraint(constraint);
        workspace.constraintSum[constraint] = view.sum;
        for (BoxId box : view.boxIds) {
            workspace.constraintMaxAdd[constraint] += shape.boxes[box].size;
            workspace.constraintNext.push_back(workspace.boxHead[box]);
            workspace.constraintIds.push_back(constraint);
            workspace.boxHead[box] = static_cast<int>(workspace.constraintIds.size()) - 1;
        }
    }

    workspace.frames.clear();
    workspace.frames.reserve(boxCount + 1);
    workspace.frames.push_back({0, 0, -1, 0, 1.0L});

    while (!workspace.frames.empty()) {
        detail::AssignmentWorkspace::Frame& frame = workspace.frames.back();
        if (frame.index == boxCount) {
            callback(std::span<const char>(workspace.assignment), frame.ways);
            const int appliedIndex = frame.appliedIndex;
            const int appliedMine = frame.appliedMine;
            workspace.frames.pop_back();
            if (appliedIndex >= 0) {
                const int maxMine = shape.boxes[appliedIndex].size;
                for (int link = workspace.boxHead[appliedIndex]; link >= 0;
                     link = workspace.constraintNext[link]) {
                    const int constraint = workspace.constraintIds[link];
                    workspace.currentSum[constraint] -= appliedMine;
                    workspace.assignedSize[constraint] -= maxMine;
                }
            }
            continue;
        }

        const int index = frame.index;
        const int maxMine = shape.boxes[index].size;
        if (frame.nextMine > maxMine) {
            const int appliedIndex = frame.appliedIndex;
            const int appliedMine = frame.appliedMine;
            workspace.frames.pop_back();
            if (appliedIndex >= 0) {
                const int appliedMaxMine = shape.boxes[appliedIndex].size;
                for (int link = workspace.boxHead[appliedIndex]; link >= 0;
                     link = workspace.constraintNext[link]) {
                    const int constraint = workspace.constraintIds[link];
                    workspace.currentSum[constraint] -= appliedMine;
                    workspace.assignedSize[constraint] -= appliedMaxMine;
                }
            }
            continue;
        }

        const int mine = frame.nextMine++;
        workspace.assignment[index] = static_cast<char>(mine);
        bool valid = true;
        for (int link = workspace.boxHead[index]; link >= 0;
             link = workspace.constraintNext[link]) {
            const int constraint = workspace.constraintIds[link];
            const int sum = workspace.currentSum[constraint] + mine;
            const int remaining = workspace.constraintMaxAdd[constraint] -
                                  (workspace.assignedSize[constraint] + maxMine);
            if (sum > workspace.constraintSum[constraint] ||
                sum + remaining < workspace.constraintSum[constraint]) {
                valid = false;
                break;
            }
        }
        if (!valid) continue;

        for (int link = workspace.boxHead[index]; link >= 0;
             link = workspace.constraintNext[link]) {
            const int constraint = workspace.constraintIds[link];
            workspace.currentSum[constraint] += mine;
            workspace.assignedSize[constraint] += maxMine;
        }
        workspace.frames.push_back(
            {index + 1, 0, index, mine, frame.ways * detail::binom(maxMine, mine)});
    }
}
