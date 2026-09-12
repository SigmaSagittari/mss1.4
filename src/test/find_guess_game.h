#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>

#include "algo/ref/java_evaluate.h"
#include "algo/ref/long_term_risk_helper.h"
#include "algo/ref/pseudo_helper.h"
#include "test/common.h"

namespace test {

struct FindGuessGameConfig {
    int rows;
    int cols;
    int mines;
    int games;
    double seconds;
    std::uint64_t seed;
};

inline void findGuessGame(const FindGuessGameConfig& config) {
    if (config.rows <= 0 || config.cols <= 0 || config.mines <= 0 ||
        config.mines >= config.rows * config.cols || config.games < 0 ||
        config.seconds < 0.0)
        std::abort();

    struct Sample {
        long double solutions;
        int cells;
    };
    std::vector<Sample> samples;
    std::array<long long, 5> solutionBuckets{};
    std::array<long long, 6> cellBuckets{};
    const auto start = std::chrono::steady_clock::now();
    const auto deadline = start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                      std::chrono::duration<double>(config.seconds));
    GameRng rng(config.seed);
    long long played = 0;
    long long losses = 0;
    long long guessStates = 0;
    long long pseudoStates = 0;
    long long javaStates = 0;
    long long rangeStates = 0;
    long long rangePseudoStates = 0;

    while (played < config.games && std::chrono::steady_clock::now() < deadline) {
        ++played;
        Game game({config.rows, config.cols, config.mines});
        game.placeMines(rng);
        mss::ObservedBoard::Delta first;
        check(game.reveal(1, 1, first), "first move was a mine");
        mss::ObservedBoard::update(game.board, std::move(first));

        while (!game.won()) {
            Analysis analysis(game.board);
            const std::vector<mss::CellId> safe = hiddenSafeCells(game, analysis);
            if (!safe.empty()) {
                const auto [x, y] = game.board.pos(safe.front());
                mss::ObservedBoard::Delta updates;
                if (!game.reveal(x, y, updates)) {
                    ++losses;
                    break;
                }
                mss::ObservedBoard::update(game.board, std::move(updates));
                continue;
            }

            ++guessStates;
            const mss::LongTermRiskReference::Config riskConfig{};
            ++javaStates;
            mss::ObservedBoard::Result javaBoard = game.board;
            mss::Basic::Result javaBasic = analysis.basic;
            mss::Structure::ShapePool javaShapes;
            mss::Structure::Result javaStructure =
                mss::Structure::analyze(javaBoard, javaBasic, javaShapes);
            mss::ShapeSolver::Distribution::Pool javaDistributions;
            mss::Probability::Result javaProbability = mss::Probability::analyze(
                javaBoard, javaBasic, javaStructure, javaShapes, javaDistributions);
            const mss::LongTermRiskReference::Influence javaRisk =
                mss::LongTermRiskReference::findInfluence(
                    javaBoard, javaBasic, javaStructure, javaProbability, javaShapes,
                    javaDistributions, {}, riskConfig);
            std::vector<mss::CellId> pseudos = javaRisk.pseudos;
            if (pseudos.empty())
                pseudos = mss::PseudoReference::findPseudo5050(
                    javaBoard, javaBasic, javaStructure, javaShapes, javaProbability);
            if (!pseudos.empty()) ++pseudoStates;

            const long double solutions = analysis.probability.candidates();
            const bool inRange = solutions > 10000.0L && solutions < 200000.0L;
            if (inRange) {
                ++rangeStates;
                if (!pseudos.empty()) ++rangePseudoStates;
            }
            if (inRange && pseudos.empty()) {
                int cells = 0;
                for (int x = 1; x <= game.board.rows; ++x)
                    for (int y = 1; y <= game.board.cols; ++y) {
                        if (game.board.board[x][y] != mss::ObservedBoard::CellState::Hidden)
                            continue;
                        const auto mark = analysis.basic.marks[x][y];
                        if (mark == mss::Basic::Mark::Frontier ||
                            mark == mss::Basic::Mark::Unknown)
                            ++cells;
                    }
                samples.push_back({solutions, cells});
                if (solutions < 25000.0L) ++solutionBuckets[0];
                else if (solutions < 50000.0L) ++solutionBuckets[1];
                else if (solutions < 100000.0L) ++solutionBuckets[2];
                else if (solutions < 150000.0L) ++solutionBuckets[3];
                else ++solutionBuckets[4];
                if (cells <= 32) ++cellBuckets[0];
                else if (cells <= 64) ++cellBuckets[1];
                else if (cells <= 128) ++cellBuckets[2];
                else if (cells <= 256) ++cellBuckets[3];
                else if (cells <= 512) ++cellBuckets[4];
                else ++cellBuckets[5];
            }

            const mss::JavaEvaluate::Result evaluation = mss::JavaEvaluate::solve(
                javaBoard, javaBasic, javaStructure, javaProbability, javaShapes,
                javaDistributions, javaRisk, {}, {});
            check(evaluation.x >= 1 && evaluation.x <= game.board.rows &&
                      evaluation.y >= 1 && evaluation.y <= game.board.cols,
                  "Java reference returned no move");
            const auto [x, y] = std::pair{evaluation.x, evaluation.y};
            const mss::CellId cell = game.board.id(x, y);
            mss::ObservedBoard::Delta updates;
            if (!game.reveal(x, y, updates)) {
                ++losses;
                break;
            }
            mss::ObservedBoard::update(game.board, std::move(updates));
        }
    }

    std::sort(samples.begin(), samples.end(),
              [](const Sample& lhs, const Sample& rhs) { return lhs.cells < rhs.cells; });
    auto quantile = [&](double p) {
        if (samples.empty()) return Sample{};
        return samples[static_cast<std::size_t>(p * (samples.size() - 1))];
    };
    const auto q50 = quantile(0.50);
    const auto q90 = quantile(0.90);
    const auto q99 = quantile(0.99);
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    std::cout << "find-guess-game " << config.rows << 'x' << config.cols << '/'
              << config.mines << ": games=" << played << " losses=" << losses
              << " guessStates=" << guessStates << " pseudoStates=" << pseudoStates
              << " javaStates=" << javaStates << " samples=" << samples.size()
              << " elapsed=" << std::fixed << std::setprecision(3) << elapsed << "s\n";
    std::cout << "  solutions [10k,25k,50k,100k,150k,200k): "
              << solutionBuckets[0] << ' ' << solutionBuckets[1] << ' '
              << solutionBuckets[2] << ' ' << solutionBuckets[3] << ' '
              << solutionBuckets[4] << '\n';
    std::cout << "  target range states/pseudo-free/pseudo: " << rangeStates << '/'
              << (rangeStates - rangePseudoStates) << '/' << rangePseudoStates << '\n';
    std::cout << "  candidate cells [1,33,65,129,257,513,+): "
              << cellBuckets[0] << ' ' << cellBuckets[1] << ' ' << cellBuckets[2]
              << ' ' << cellBuckets[3] << ' ' << cellBuckets[4] << ' ' << cellBuckets[5]
              << '\n';
    if (!samples.empty())
        std::cout << "  candidate cells min/median/p90/p99/max: "
                  << samples.front().cells << '/' << q50.cells << '/' << q90.cells << '/'
                  << q99.cells << '/' << samples.back().cells << '\n';
}

}  // namespace test
