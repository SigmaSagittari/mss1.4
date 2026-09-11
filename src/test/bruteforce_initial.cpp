#include <chrono>
#include <iomanip>
#include <iostream>

#ifdef MSS_OLD
#include "analysis/bruteforce/endgame_bruteforce.h"
#else
#include "algo/bruteforce/bruteforce.h"
#endif

namespace {

struct TestCase {
    int rows;
    int cols;
    int mines;
};

#ifdef MSS_OLD

void run(const TestCase testCase) {
    auto board = mss::ObservedBoard::analyze(
        testCase.rows, testCase.cols, testCase.mines);
    auto basic = mss::Basic::analyze(board);

    mss::Structure::ShapePool shapes;
    const auto structure = mss::Structure::analyze(board, basic, shapes);
    mss::Distribution::DistPool distributions;
    mss::Grid<long double> probability(testCase.rows, testCase.cols, 0.5L);
    mss::EndgameBruteforce::Config config;
    config.checkAllMoves = false;

    const auto start = std::chrono::steady_clock::now();
    const auto result = mss::EndgameBruteforce::solveEndgame(
        board, basic, structure, distributions, probability, config, 1);
    const double milliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    const int wins = result.result.empty() ? 0 : result.result[0].wins;

    std::cout << testCase.rows << 'x' << testCase.cols << '/' << testCase.mines
              << " total=" << result.totalPossibilities
              << " wins=" << wins
              << " winrate=" << std::setprecision(12)
              << static_cast<double>(wins) / result.totalPossibilities
              << " time_ms=" << std::fixed << std::setprecision(3)
              << milliseconds << " nodes=" << result.nodes << '\n';
}

#else

void run(const TestCase testCase) {
    auto board = mss::ObservedBoard::analyze(
        testCase.rows, testCase.cols, testCase.mines);
    auto basic = mss::Basic::analyze(board);

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
    const int wins = result.moves.empty() ? 0 : result.moves[0].wins;
    std::cout << testCase.rows << 'x' << testCase.cols << '/' << testCase.mines
              << " total=" << result.possibilities
              << " wins=" << wins
              << " winrate=" << std::setprecision(12)
              << static_cast<double>(wins) / result.possibilities
              << " time_ms=" << std::fixed << std::setprecision(3)
              << milliseconds << " nodes=" << result.nodes << '\n';
}

#endif

}  // namespace

int main(int argc, char** argv) {
    if (argc == 1 || argv[1][0] == '4') run({4, 4, 5});
    if (argc == 1 || argv[1][0] == '5') run({5, 5, 5});
    return 0;
}
