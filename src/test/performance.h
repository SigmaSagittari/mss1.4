#pragma once

#include <iomanip>
#include <iostream>

#include "test/common.h"

namespace test {

// 只测核心热路径耗时；输出用于比较实现，不是功能断言。
inline void performance() {
    const TestConfig config{
        .rows = 30,
        .cols = 16,
        .mines = 99,
        .seconds = 1.0,
        .games = -1,
        .filter = PositionFilter::All,
        .maxRestarts = 10000,
        .requireWinningGame = false,
        .firstMoveSafe = true,
    };
    GameRng rng(0xC0FFEE12345ULL);
    long long steps = 0;
    const RunSummary summary = runGames(
        config, rng, [&](const Snapshot&) { ++steps; });
    const double winRate = 100.0 * summary.wins / summary.games;
    const double gamesPerSecond = summary.games / summary.elapsedSeconds;
    std::cout << "performance/real-game: " << summary.games << " games, "
              << summary.wins << " wins (" << std::fixed << std::setprecision(3)
              << winRate << "%), " << gamesPerSecond << " games/s, " << steps
              << " steps, " << summary.elapsedSeconds << "s\n";
}

}  // namespace test
