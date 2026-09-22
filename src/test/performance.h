#pragma once

#include <iomanip>
#include <iostream>

#include "core/utility/rng.h"
#include "test/common.h"

namespace test {

// 只测核心热路径耗时；输出用于比较实现，不是功能断言。
inline void performance() {
    const TestConfig config{
        .rows = 30,
        .cols = 16,
        .mines = 99,
        .seconds = 40.0,
        .games = -1,
        .filter = PositionFilter::All,
        .firstMoveSafe = true,
    };
    mss::Random rng(0xC0FFEE12345ULL, 0xD1B54A32D192ED03ULL);
    const mss::GameControl::Position::SuggestMode mode = mss::GameControl::Position::SuggestMode::Java;
    long long steps = 0;
    const RunSummary summary = runGames(config, rng, mode, [&](const Snapshot &) {
        ++steps;
    });
    const double winRate = 100.0 * summary.wins / summary.games;
    const double gamesPerSecond = summary.games / summary.elapsedSeconds;
    std::cout << "performance/real-game: mode=" << (mode == mss::GameControl::Position::SuggestMode::Java ? "Java" : "LowRisk") << ", "
              << summary.games << " games, " << summary.wins << " wins (" << std::fixed << std::setprecision(3)
              << winRate << "%), " << gamesPerSecond << " games/s, " << steps << " steps, " << summary.elapsedSeconds << "s\n";
}

} // namespace test
