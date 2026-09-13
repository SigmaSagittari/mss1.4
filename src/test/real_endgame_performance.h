#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <vector>

#include "algo/bruteforce/bruteforce.h"
#include "algo/ref/java_evaluate.h"
#include "algo/ref/long_term_risk_helper.h"
#include "algo/ref/pseudo_helper.h"
#include "test/common.h"

namespace test {

inline int bruteForceCandidateCount(const Game& game, const Analysis& analysis) {
    int count = 0;
    for (int x = 1; x <= game.board.rows; ++x)
        for (int y = 1; y <= game.board.cols; ++y)
            if (game.board.board[x][y] == mss::ObservedBoard::CellState::Hidden &&
                (analysis.basic.marks[x][y] == mss::Basic::Mark::H ||
                 analysis.basic.marks[x][y] == mss::Basic::Mark::T))
                ++count;
    return count;
}

inline bool hasHiddenSafeCell(const Game& game, const Analysis& analysis) {
    for (int x = 1; x <= game.board.rows; ++x)
        for (int y = 1; y <= game.board.cols; ++y)
            if (game.board.board[x][y] == mss::ObservedBoard::CellState::Hidden &&
                analysis.basic.marks[x][y] == mss::Basic::Mark::Safe)
                return true;
    return false;
}

inline void real_endgame_performance(const int l, const int r,
                                     const double seconds) {
    constexpr int kRows = 30;
    constexpr int kCols = 16;
    constexpr int kMines = 99;
    constexpr std::uint64_t kSeed = 0xC0FFEE12345ULL;

    struct PossibilityBucket {
        int low;
        int high;
        long long calls = 0;
        long long nodes = 0;
        long long possibilities = 0;
        double milliseconds = 0.0;
    };
    const int range = r - l;
    const std::array<PossibilityBucket, 5> bucketTemplate{{
        {l, l + range / 10},
        {l + range / 10, l + range / 5},
        {l + range / 5, l + range * 2 / 5},
        {l + range * 2 / 5, l + range * 7 / 10},
        {l + range * 7 / 10, r + 1}}};
    std::array<PossibilityBucket, 5> possibilityBuckets = bucketTemplate;

    TimeBox timebox(seconds);
    GameRng rng(kSeed);
    long long games = 0;
    long long wins = 0;
    long long calls = 0;
    long long gamesWithCalls = 0;
    long long totalNodes = 0;
    long long totalPossibilities = 0;
    long long positions = 0;
    long long noSafePositions = 0;
    long long noFiftyFiftyPositions = 0;
    long long eligiblePositions = 0;
    long long moves = 0;
    long long safeMoves = 0;
    long long javaMoves = 0;
    long long javaSafeMoves = 0;
    long long lowestSafeMoves = 0;
    long long noSafeMoves = 0;
    long long noSafeLowestSafeMoves = 0;
    long double javaSafetyTotal = 0;
    long double lowestSafetyTotal = 0;
    long double javaSafetyMinimum = 1;
    long double lowestSafetyMinimum = 1;
    int maxOpened = 0;
    int minAllCandidates = 0;
    int maxAllCandidates = 0;
    int minEligibleCandidates = 0;
    int maxEligibleCandidates = 0;
    long double minEligiblePossibilities = 0;
    long double maxEligiblePossibilities = 0;
    double totalMilliseconds = 0.0;
    int minCandidates = r + 1;
    int maxCandidates = 0;
    int minPossibilities = 0;
    int maxPossibilities = 0;
    double slowestMilliseconds = 0.0;
    long long slowestGame = 0;
    int slowestMove = 0;
    int slowestOpened = 0;
    int slowestCandidates = 0;
    int slowestPossibilities = 0;
    long long slowestNodes = 0;
    mss::ObservedBoard::Result slowestBoard;
    std::vector<char> slowestMines;

    while (!timebox.expired()) {
        const long long gameNumber = games + 1;
        Game game({kRows, kCols, kMines});
        game.placeMines(rng, true);
        mss::ObservedBoard::Delta updates;
        if (!game.reveal(1, 1, updates)) std::abort();
        mss::ObservedBoard::update(game.board, updates);
        Analysis analysis(game.board);
        bool tracked = false;
        int moveNumber = 0;

        while (!game.won()) {
            ++moveNumber;
            const mss::LongTermRiskReference::Config riskConfig{};
            mss::LongTermRiskReference::Influence risk =
                mss::LongTermRiskReference::findInfluence(
                    game.board, analysis.basic, analysis.structure,
                    analysis.probability, analysis.shapes, analysis.distributions,
                    {}, riskConfig);
            const int candidates = bruteForceCandidateCount(game, analysis);
            const bool noSafe = !hasHiddenSafeCell(game, analysis);
            const bool noFiftyFifty =
                risk.pseudos.empty() && risk.hotspots.empty() && risk.possible.empty();
            const mss::JavaEvaluate::Config evaluateConfig{};
            const mss::JavaEvaluate::Result java = mss::JavaEvaluate::solve(
                game.board, analysis.basic, analysis.structure, analysis.probability,
                analysis.shapes, analysis.distributions, risk, {}, evaluateConfig);
            if (java.x == 0 || java.y == 0) std::abort();
            const Move next = {java.x, java.y,
                               1.0L - analysis.probability.mineProbability(
                                   game.board.id(java.x, java.y), game.board,
                                   analysis.basic, analysis.structure)};
            ++javaMoves;
            const Move lowest = lowestRiskMove(game, analysis);
            ++moves;
            if (!game.mine(next.x, next.y)) ++safeMoves;
            if (noSafe) {
                ++noSafeMoves;
                if (!game.mine(next.x, next.y)) ++javaSafeMoves;
            }
            if (!game.mine(lowest.x, lowest.y)) ++lowestSafeMoves;
            if (noSafe && !game.mine(lowest.x, lowest.y)) ++noSafeLowestSafeMoves;
            javaSafetyTotal += next.mineProbability;
            lowestSafetyTotal += 1.0L - lowest.mineProbability;
            javaSafetyMinimum = (std::min)(javaSafetyMinimum, next.mineProbability);
            lowestSafetyMinimum = (std::min)(lowestSafetyMinimum, 1.0L - lowest.mineProbability);
            maxOpened = (std::max)(maxOpened, game.opened);
            if (minAllCandidates == 0 || candidates < minAllCandidates)
                minAllCandidates = candidates;
            maxAllCandidates = (std::max)(maxAllCandidates, candidates);
            ++positions;
            if (noSafe) ++noSafePositions;
            if (noFiftyFifty) ++noFiftyFiftyPositions;
            if (noSafe && noFiftyFifty) {
                ++eligiblePositions;
                if (minEligibleCandidates == 0 || candidates < minEligibleCandidates)
                    minEligibleCandidates = candidates;
                maxEligibleCandidates = (std::max)(maxEligibleCandidates, candidates);
                const long double possibilities = analysis.probability.candidates();
                if (minEligiblePossibilities == 0 || possibilities < minEligiblePossibilities)
                    minEligiblePossibilities = possibilities;
                maxEligiblePossibilities = (std::max)(maxEligiblePossibilities, possibilities);
            }
            const long double possibilities = analysis.probability.candidates();
            if (noSafe && noFiftyFifty && l <= possibilities && possibilities <= r) {
                const auto started = std::chrono::steady_clock::now();
                const mss::BruteForce::Result result = mss::BruteForce::solve(
                    game.board, analysis.basic, analysis.structure, analysis.shapes,
                    {false, 1});
                const double milliseconds = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - started).count();
                totalMilliseconds += milliseconds;
                ++calls;
                totalNodes += result.nodes;
                totalPossibilities += result.possibilities;
                for (PossibilityBucket& bucket : possibilityBuckets)
                    if (bucket.low <= result.possibilities &&
                        result.possibilities < bucket.high) {
                        ++bucket.calls;
                        bucket.nodes += result.nodes;
                        bucket.possibilities += result.possibilities;
                        bucket.milliseconds += milliseconds;
                        break;
                    }
                std::cout << "performance/real_endgame/search game=" << gameNumber
                          << " move=" << moveNumber << " opened=" << game.opened
                          << " candidates=" << candidates
                          << " possibilities=" << result.possibilities
                          << " nodes=" << result.nodes << " time_ms="
                          << std::fixed << std::setprecision(3) << milliseconds << '\n';
                if (milliseconds > slowestMilliseconds) {
                    slowestMilliseconds = milliseconds;
                    slowestGame = gameNumber;
                    slowestMove = moveNumber;
                    slowestOpened = game.opened;
                    slowestCandidates = candidates;
                    slowestPossibilities = result.possibilities;
                    slowestNodes = result.nodes;
                    slowestBoard = game.board;
                    slowestMines = game.mines;
                }
                if (!tracked) {
                    ++gamesWithCalls;
                    tracked = true;
                }
                minCandidates = (std::min)(minCandidates, candidates);
                maxCandidates = (std::max)(maxCandidates, candidates);
                if (minPossibilities == 0 || result.possibilities < minPossibilities)
                    minPossibilities = result.possibilities;
                maxPossibilities = (std::max)(maxPossibilities, result.possibilities);
            }

            updates.clear();
            if (!game.reveal(next.x, next.y, updates)) break;
            if (game.won()) break;
            analysis.update(game.board, updates);
        }
        if (game.won()) ++wins;
        ++games;
    }

    std::cout << "performance/real_endgame: " << kRows << 'x' << kCols << '/'
              << kMines << " games=" << games << " possibility_range=[" << l << ','
              << r << "] wins=" << wins << " ("
              << (games == 0 ? 0 : 100.0 * wins / games) << "%)"
              << " moves=" << moves << " safe_moves=" << safeMoves
              << " java_moves=" << javaMoves << " java_safe_moves=" << javaSafeMoves
              << " no_safe_moves=" << noSafeMoves << " no_safe_lowest_safe_moves="
              << noSafeLowestSafeMoves
              << " lowest_safe_moves=" << lowestSafeMoves
              << " java_safety_avg=" << (javaMoves == 0 ? 0 : javaSafetyTotal / javaMoves)
              << " java_safety_min=" << javaSafetyMinimum
              << " lowest_safety_avg=" << (moves == 0 ? 0 : lowestSafetyTotal / moves)
              << " lowest_safety_min=" << lowestSafetyMinimum
              << " opened_max=" << maxOpened << " all_candidates=["
              << minAllCandidates << ',' << maxAllCandidates << "]"
              << " positions=" << positions << " no_safe=" << noSafePositions
              << " no_5050=" << noFiftyFiftyPositions
              << " eligible=" << eligiblePositions << " eligible_candidates=["
              << minEligibleCandidates << ',' << maxEligibleCandidates
              << "] eligible_possibilities=[" << minEligiblePossibilities << ','
              << maxEligiblePossibilities << "]"
              << " calls=" << calls << " games_with_calls=" << gamesWithCalls
              << " candidates=[" << (calls == 0 ? 0 : minCandidates) << ','
              << maxCandidates << "] possibilities=[" << minPossibilities << ','
              << maxPossibilities << "] possibilities_total=" << totalPossibilities
              << " nodes=" << totalNodes << " search_time_ms="
              << std::fixed << std::setprecision(3) << totalMilliseconds;
    std::cout << " wall_time_ms=" << timebox.elapsedSeconds() * 1000.0;
    if (calls != 0)
        std::cout << " avg_nodes=" << totalNodes / calls
                  << " avg_time_ms=" << totalMilliseconds / calls;
    std::cout << " by_possibilities=";
    bool firstBucket = true;
    for (const PossibilityBucket& bucket : possibilityBuckets) {
        if (bucket.calls == 0) continue;
        if (!firstBucket) std::cout << ';';
        firstBucket = false;
        std::cout << '[' << bucket.low << ',' << bucket.high << ")="
                  << bucket.calls << ",nodes=" << bucket.nodes
                  << ",configs=" << bucket.possibilities
                  << ",time_ms=" << bucket.milliseconds
                  << ",avg_nodes=" << bucket.nodes / bucket.calls
                  << ",avg_configs=" << bucket.possibilities / bucket.calls
                  << ",avg_time_ms=" << bucket.milliseconds / bucket.calls;
    }
    std::cout << '\n';
    if (slowestGame != 0) {
        std::cout << "performance/real_endgame/slowest game=" << slowestGame
                  << " move=" << slowestMove << " opened=" << slowestOpened
                  << " candidates=" << slowestCandidates
                  << " possibilities=" << slowestPossibilities
                  << " nodes=" << slowestNodes << " time_ms="
                  << slowestMilliseconds << " observed_board:\n";
        for (int x = 1; x <= slowestBoard.rows; ++x) {
            for (int y = 1; y <= slowestBoard.cols; ++y) {
                const mss::ObservedBoard::CellState state = slowestBoard.board[x][y];
                if (state == mss::ObservedBoard::CellState::Hidden)
                    std::cout << '?';
                else if (state == mss::ObservedBoard::CellState::ForcedMine)
                    std::cout << 'F';
                else if (state == mss::ObservedBoard::CellState::ForcedSafe)
                    std::cout << 'S';
                else
                    std::cout << static_cast<int>(state);
            }
            std::cout << '\n';
        }
        std::cout << "performance/real_endgame/slowest mines:\n";
        for (int x = 1; x <= slowestBoard.rows; ++x) {
            for (int y = 1; y <= slowestBoard.cols; ++y)
                std::cout << (slowestMines[(x - 1) * slowestBoard.cols + y - 1] ? '*' : '.');
            std::cout << '\n';
        }
    }
}

}  // namespace test
