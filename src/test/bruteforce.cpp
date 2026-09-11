#include "test/bruteforce.h"

#include <chrono>
#include <iostream>

#include "algo/basic.h"
#include "algo/bruteforce/bruteforce.h"
#include "algo/observed_board.h"
#include "algo/shape_solver/shape_solver.h"
#include "algo/structure.h"
#include "test/common.h"

namespace test {

void bruteforce() {
    auto board = mss::ObservedBoard::analyze(5, 5, 4);
    const auto basic = mss::Basic::analyze(board);
    mss::Structure::ShapePool shapes;
    const auto structure = mss::Structure::analyze(board, basic, shapes);
    mss::Probability::Result probability;
    mss::ShapeSolver::Distribution::Pool distributions;
    const mss::BruteForce::Config config{false, 1};

    const auto start = std::chrono::steady_clock::now();
    const auto result = mss::BruteForce::solve(
        board, basic, structure, probability, shapes, distributions, config);
    const double milliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();

    check(result.possibilities == 12650, "5x5/4 possibility count changed");
    check(result.moves.size() == 1 && result.moves[0].wins == 8236,
          "5x5/4 exact bruteforce win count changed");
    std::cout << "test/bruteforce: 5x5/4 wins=" << result.moves[0].wins
              << " nodes=" << result.nodes << " time_ms=" << milliseconds << '\n';
}

}  // namespace test
