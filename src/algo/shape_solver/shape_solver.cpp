#include "algo/shape_solver/shape_solver.h"

#include "algo/shape_solver/dfs_solver.h"
#include "algo/shape_solver/graph_solver.h"

namespace mss {

namespace ShapeSolver {

namespace Distribution {

Result::Result(int start, int boxCount, std::vector<long double> ways,
               std::vector<long double> perBoxExpectations)
    : start_(start), boxCount_(boxCount), ways_(std::move(ways)),
      perBoxExpectationData_(std::move(perBoxExpectations)) {
    perBoxExpectations_.reserve(ways_.size());
    const std::span<const long double> allExpectations =
        perBoxExpectationData_;
    for (std::size_t i = 0; i < ways_.size(); ++i)
        perBoxExpectations_.push_back(
            allExpectations.subspan(i * boxCount_, boxCount_));
}

DistributionId Pool::find(U128 hash) const {
    if (const DistributionId* found = index_.find(hash)) return *found;
    return -1;
}

DistributionId Pool::insert(U128 hash, Result result) {
    if (const DistributionId* found = index_.find(hash)) return *found;

    const DistributionId id = static_cast<DistributionId>(results_.size());
    results_.push_back(std::move(result));
    index_.emplace(hash, id);
    return id;
}

void Pool::clear() {
    results_.clear();
    index_.clear();
}

}  // namespace Distribution

DistributionId analyze(const Structure::Shape& shape, Distribution::Pool& pool) {
    if (static_cast<int>(shape.boxes.size()) < graphThreshold)
        return DfsSolver::analyze(shape, pool);
    return GraphSolver::analyze(shape, pool, GraphSolver::PolishKind::Adjacent);
}

}  // namespace ShapeSolver

}  // namespace mss
