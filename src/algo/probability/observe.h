#pragma once

#include "algo/probability/probability.h"

namespace mss {

namespace Probability {

ObserveResult observe(const ObservedBoard::Result& board,
                      const Basic::Result& basic,
                      const Structure::Result& structure,
                      const Structure::ShapePool& shapes,
                      const Result& probability,
                      ShapeSolver::Distribution::Pool& distributions,
                      CellId cell);

}  // namespace Probability

}  // namespace mss
