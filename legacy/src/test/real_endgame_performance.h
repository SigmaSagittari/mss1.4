#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>

#include "algo/probability_engine/bruteforce/bruteforce.h"
#include "core/utility/rng.h"
#include "test/common.h"

namespace test {

inline int bruteForceCandidateCount(const mss::GameControl::Game &game) {
    // 统计当前测试局面中可交给残局搜索的候选格数量。
    int count = 0;
    const mss::GameControl::Position &position = game.position;
    for (int x = 1; x <= position.observedBoard.rows; ++x)
        for (int y = 1; y <= position.observedBoard.cols; ++y)
            if (position.observedBoard.board[x][y] == mss::ObservedBoard::CellState::Hidden &&
                (position.basic().marks[x][y] == mss::Basic::Mark::H || position.basic().marks[x][y] == mss::Basic::Mark::T))
                ++count;
    return count;
}

inline bool hasZeroProbabilityCell(mss::GameControl::Game &game) {
    // 只按全局概率判断确定安全格；Basic 的局部传播不能覆盖完整逻辑推导。
    const mss::Probability::Result &probability = game.probability();
    const mss::GameControl::Position &position = game.position;
    for (int x = 1; x <= position.observedBoard.rows; ++x)
        for (int y = 1; y <= position.observedBoard.cols; ++y)
            if (position.observedBoard.board[x][y] == mss::ObservedBoard::CellState::Hidden &&
                probability.mineProbability(position.observedBoard.id(x, y), position.observedBoard, position.basic(), position.structure()) == 0.0L)
                return true;
    return false;
}

inline void real_endgame_performance(const int l, const int r, const double seconds, const bool compareCommon = false) {
    // 在指定样本区间内比较残局后端的结果一致性和运行性能。
    constexpr int kRows = 30;
    constexpr int kCols = 16;
    constexpr int kMines = 99;

    struct LayoutBucket {
        int low;
        int high;
        long long calls = 0;
        long long nodes = 0;
        long long commonNodes = 0;
        long double layouts = 0.0L;
        double milliseconds = 0.0;
        double commonMilliseconds = 0.0;
    };
    const int range = r - l;
    const std::array<LayoutBucket, 5> bucketTemplate{{{l, l + range / 10},
                                                       {l + range / 10, l + range / 5},
                                                       {l + range / 5, l + range * 2 / 5},
                                                       {l + range * 2 / 5, l + range * 7 / 10},
                                                       {l + range * 7 / 10, r + 1}}};
    std::array<LayoutBucket, 5> layoutBuckets = bucketTemplate;

    TimeBox timebox(seconds);
    mss::Random rng(0xC0FFEE12345ULL, 0xD1B54A32D192ED03ULL);
    long long games = 0;
    long long wins = 0;
    long long calls = 0;
    long long gamesWithCalls = 0;
    long long totalNodes = 0;
    long long commonTotalNodes = 0;
    long double totalLayouts = 0.0L;
    long long positions = 0;
    long long noSafePositions = 0;
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
    int minAllCandidates = 0;
    int maxAllCandidates = 0;
    double totalMilliseconds = 0.0;
    double commonTotalMilliseconds = 0.0;
    long long mismatches = 0;
    long long tieMoveDifferences = 0;
    int minCandidates = r + 1;
    int maxCandidates = 0;
    long double minLayouts = 0.0L;
    long double maxLayouts = 0.0L;
    double slowestMilliseconds = 0.0;
    long long slowestGame = 0;
    int slowestMove = 0;
    int slowestCandidates = 0;
    long double slowestLayouts = 0.0L;
    long long slowestNodes = 0;
    mss::ObservedBoard::Result slowestBoard;

    while (!timebox.expired()) {
        const long long gameNumber = games + 1;
        mss::GameControl::mineBoard mineBoard;
        mineBoard.generate(kRows, kCols, kMines, mss::U128{rng.next(), rng.next()});
        mss::GameControl::Game game(std::move(mineBoard), mss::ObservedBoard::Result(kRows, kCols, kMines));
        mss::ObservedBoard::Delta updates;
        const std::vector<mss::CellId> firstMove = game.Suggest(mss::GameControl::Position::SuggestMode::Java);
        mss::assert_(!firstMove.empty(), "test::real_endgame_performance: initial Suggest returned no move");
        game.makeFirstMoveSafe(firstMove.front(), mss::U128{rng.next(), rng.next()});
        const mss::CellId first = firstMove.front();
        const auto [firstX, firstY] = game.position.observedBoard.pos(first);
        updates.changes.push_back({first, (mss::ObservedBoard::CellState)game.number(firstX, firstY)});
        game.update(updates);
        const mss::GameControl::Position &position = game.position;
        bool tracked = false;
        int moveNumber = 0;

        while (!game.won()) {
            ++moveNumber;
            const int candidates = bruteForceCandidateCount(game);
            const std::vector<mss::CellId> java = game.Suggest(mss::GameControl::Position::SuggestMode::Java);
            mss::assert_(!java.empty(), "test::real_endgame_performance: Java Suggest returned no move");
            const bool noSafe = !hasZeroProbabilityCell(game);
            const mss::CellId javaCell = java.front();
            const auto [javaX, javaY] = position.observedBoard.pos(javaCell);
            const long double javaMineProbability = 1.0L - game.mineProbability(javaCell);
            const std::vector<mss::CellId> lowest = game.Suggest(mss::GameControl::Position::SuggestMode::LowRisk);
            mss::assert_(!lowest.empty(), "test::real_endgame_performance: LowRisk Suggest returned no move");
            const mss::CellId lowestCell = lowest.front();
            const auto [lowestX, lowestY] = position.observedBoard.pos(lowestCell);
            const long double lowestMineProbability = game.mineProbability(lowestCell);
            ++javaMoves;
            ++moves;
            if (game.number(javaX, javaY) != 9)
                ++safeMoves;
            if (noSafe) {
                ++noSafeMoves;
                if (game.number(javaX, javaY) != 9)
                    ++javaSafeMoves;
            }
            if (game.number(lowestX, lowestY) != 9)
                ++lowestSafeMoves;
            if (noSafe && game.number(lowestX, lowestY) != 9)
                ++noSafeLowestSafeMoves;
            javaSafetyTotal += javaMineProbability;
            lowestSafetyTotal += 1.0L - lowestMineProbability;
            javaSafetyMinimum = (std::min)(javaSafetyMinimum, javaMineProbability);
            lowestSafetyMinimum = (std::min)(lowestSafetyMinimum, 1.0L - lowestMineProbability);
            if (minAllCandidates == 0 || candidates < minAllCandidates)
                minAllCandidates = candidates;
            maxAllCandidates = (std::max)(maxAllCandidates, candidates);
            ++positions;
            if (noSafe)
                ++noSafePositions;
            const long double layoutCount = game.probability().candidates();
            if (noSafe && l <= layoutCount && layoutCount <= r) {
                const std::chrono::steady_clock::time_point multimaskStarted = std::chrono::steady_clock::now();
                const mss::BruteForce::Result multimaskResult =
                    mss::BruteForce::solve(position.observedBoard, position.basic(), position.structure(), game.shapePool(),
                                           {false, 1, mss::BruteForce::Solver::BitwiseRootParallel});
                const double multimaskMilliseconds =
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - multimaskStarted).count();
                mss::BruteForce::Result commonResult;
                double commonMilliseconds = 0.0;
                bool same = true;
                bool sameValue = true;
                if (compareCommon) {
                    const std::chrono::steady_clock::time_point commonStarted = std::chrono::steady_clock::now();
                    commonResult = mss::BruteForce::solve(position.observedBoard, position.basic(), position.structure(), game.shapePool(),
                                                          {false, 1, mss::BruteForce::Solver::Common});
                    commonMilliseconds =
                        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - commonStarted).count();
                    commonTotalMilliseconds += commonMilliseconds;
                    commonTotalNodes += commonResult.nodes;
                    sameValue = multimaskResult.moves.size() == commonResult.moves.size() &&
                                std::equal(multimaskResult.moves.begin(), multimaskResult.moves.end(), commonResult.moves.begin(),
                                           [](const auto &a, const auto &b) {
                                               return a.wins == b.wins;
                                           });
                    same = sameValue && std::equal(multimaskResult.moves.begin(), multimaskResult.moves.end(), commonResult.moves.begin(),
                                                   [](const auto &a, const auto &b) {
                                                       return a.x == b.x && a.y == b.y;
                                                   });
                    if (!sameValue) {
                        ++mismatches;
                        std::cout << "performance/real_endgame/mismatch game=" << gameNumber << " move=" << moveNumber
                                  << " multimask_moves=" << multimaskResult.moves.size() << " common_moves=" << commonResult.moves.size()
                                  << '\n';
                    } else if (!same) {
                        ++tieMoveDifferences;
                    }
                }
                totalMilliseconds += multimaskMilliseconds;
                ++calls;
                totalNodes += multimaskResult.nodes;
                totalLayouts += layoutCount;
                for (LayoutBucket &bucket : layoutBuckets)
                    if (bucket.low <= layoutCount && layoutCount < bucket.high) {
                        ++bucket.calls;
                        bucket.nodes += multimaskResult.nodes;
                        bucket.commonNodes += commonResult.nodes;
                        bucket.layouts += layoutCount;
                        bucket.milliseconds += multimaskMilliseconds;
                        bucket.commonMilliseconds += commonMilliseconds;
                        break;
                    }
                std::cout << "performance/real_endgame/search game=" << gameNumber << " move=" << moveNumber
                          << " candidates=" << candidates << " layouts=" << layoutCount
                          << " multimask_nodes=" << multimaskResult.nodes << " multimask_time_ms=" << std::fixed << std::setprecision(3)
                          << multimaskMilliseconds;
                if (compareCommon)
                    std::cout << " common_nodes=" << commonResult.nodes << " common_time_ms=" << commonMilliseconds
                              << " same_value=" << (sameValue ? 1 : 0) << " same_move=" << (same ? 1 : 0);
                std::cout << '\n';
                if (multimaskMilliseconds > slowestMilliseconds) {
                    slowestMilliseconds = multimaskMilliseconds;
                    slowestGame = gameNumber;
                    slowestMove = moveNumber;
                    slowestCandidates = candidates;
                    slowestLayouts = layoutCount;
                    slowestNodes = multimaskResult.nodes;
                    slowestBoard = position.observedBoard;
                }
                if (!tracked) {
                    ++gamesWithCalls;
                    tracked = true;
                }
                minCandidates = (std::min)(minCandidates, candidates);
                maxCandidates = (std::max)(maxCandidates, candidates);
                if (minLayouts == 0.0L || layoutCount < minLayouts)
                    minLayouts = layoutCount;
                maxLayouts = (std::max)(maxLayouts, layoutCount);
            }

            updates.clear();
            const int javaNumber = game.number(javaX, javaY);
            if (javaNumber == 9)
                break;
            updates.changes.push_back({javaCell, (mss::ObservedBoard::CellState)(javaNumber)});
            game.update(updates);
            if (game.won())
                break;
        }
        if (game.won())
            ++wins;
        ++games;
    }

    std::cout << "performance/real_endgame: " << kRows << 'x' << kCols << '/' << kMines << " games=" << games << " layout_count_range=[" << l
              << ',' << r << "] wins=" << wins << " (" << (games == 0 ? 0 : 100.0 * wins / games) << "%)"
              << " moves=" << moves << " safe_moves=" << safeMoves << " java_moves=" << javaMoves << " java_safe_moves=" << javaSafeMoves
              << " no_safe_moves=" << noSafeMoves << " no_safe_lowest_safe_moves=" << noSafeLowestSafeMoves
              << " lowest_safe_moves=" << lowestSafeMoves << " java_safety_avg=" << (javaMoves == 0 ? 0 : javaSafetyTotal / javaMoves)
              << " java_safety_min=" << javaSafetyMinimum << " lowest_safety_avg=" << (moves == 0 ? 0 : lowestSafetyTotal / moves)
              << " lowest_safety_min=" << lowestSafetyMinimum << " all_candidates=[" << minAllCandidates
              << ',' << maxAllCandidates << "]"
              << " positions=" << positions << " no_safe=" << noSafePositions
              << " calls=" << calls << " games_with_calls=" << gamesWithCalls << " candidates=[" << (calls == 0 ? 0 : minCandidates) << ','
              << maxCandidates << "] layouts=[" << minLayouts << ',' << maxLayouts
              << "] layout_count_total=" << totalLayouts << " nodes=" << totalNodes << " search_time_ms=" << std::fixed
              << std::setprecision(3) << totalMilliseconds;
    if (compareCommon)
        std::cout << " common_nodes=" << commonTotalNodes << " common_search_time_ms=" << commonTotalMilliseconds
                  << " value_mismatches=" << mismatches << " tie_move_differences=" << tieMoveDifferences;
    std::cout << " wall_time_ms=" << timebox.elapsedSeconds() * 1000.0;
    if (calls != 0)
        std::cout << " avg_nodes=" << totalNodes / calls << " avg_time_ms=" << totalMilliseconds / calls;
    if (compareCommon && calls != 0)
        std::cout << " common_avg_nodes=" << commonTotalNodes / calls << " common_avg_time_ms=" << commonTotalMilliseconds / calls;
    std::cout << " by_layout_count=";
    bool firstBucket = true;
    for (const LayoutBucket &bucket : layoutBuckets) {
        if (bucket.calls == 0)
            continue;
        if (!firstBucket)
            std::cout << ';';
        firstBucket = false;
        std::cout << '[' << bucket.low << ',' << bucket.high << ")=" << bucket.calls << ",nodes=" << bucket.nodes
                  << ",layouts=" << bucket.layouts << ",time_ms=" << bucket.milliseconds
                  << ",avg_nodes=" << bucket.nodes / bucket.calls << ",avg_layouts=" << bucket.layouts / bucket.calls
                  << ",avg_time_ms=" << bucket.milliseconds / bucket.calls;
        if (compareCommon)
            std::cout << ",common_nodes=" << bucket.commonNodes << ",common_time_ms=" << bucket.commonMilliseconds
                      << ",common_avg_nodes=" << bucket.commonNodes / bucket.calls
                      << ",common_avg_time_ms=" << bucket.commonMilliseconds / bucket.calls;
    }
    std::cout << '\n';
    if (slowestGame != 0) {
        std::cout << "performance/real_endgame/slowest game=" << slowestGame << " move=" << slowestMove
                  << " candidates=" << slowestCandidates << " layouts=" << slowestLayouts << " nodes=" << slowestNodes
                  << " time_ms=" << slowestMilliseconds << " observed_board:\n";
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
                    std::cout << (int)(state);
            }
            std::cout << '\n';
        }
    }
}

} // namespace test
