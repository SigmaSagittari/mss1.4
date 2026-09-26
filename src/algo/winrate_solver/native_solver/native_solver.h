#pragma once

#include "algo/probability_engine/bruteforce/bruteforce.h"
#include "algo/probability_engine/basic.h"
#include "algo/probability_engine/probability/probability.h"
#include "algo/probability_engine/probability/probability_external.h"
#include "algo/probability_engine/shape_solver/shape_solver.h"
#include "algo/probability_engine/structure.h"

namespace mss {
struct NativeSolver {
    struct Result {
        int x = 0;
        int y = 0;
    };

        static Result solve(ObservedBoard::Result &board, Basic::Result &basic, Structure::Result &structure,
                        const Probability::Result &probability, Structure::structPool &shapes,
                        ShapeSolver::Distribution::Pool &distributions, BruteForce::Workspace &bruteForceWorkspace);
};

} // namespace mss
