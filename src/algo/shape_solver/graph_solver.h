#pragma once

#include "algo/shape_solver/shape_solver.h"

namespace mss {

namespace ShapeSolver {

namespace GraphSolver {

enum class PolishKind {
    Adjacent,
    Window3,
};

// Graph DP 后端的普通分布求解。
DistributionId analyze(const Structure::Shape& shape, Distribution::Pool& pool,
                       PolishKind polish);

}  // namespace GraphSolver

}  // namespace ShapeSolver

}  // namespace mss
