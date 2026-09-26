#pragma once

#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

#include "core/config.h"
#include "core/utility/rng.h"
#include "test/common.h"

namespace test {

// 只测核心热路径耗时；输出用于比较实现，不是功能断言。
inline void performance() {
    const TestConfig config{
        .rows = 17,
        .cols = 17,
        .mines = 104,
        .seconds = 40.0,
        .games = -1,
        .filter = PositionFilter::All,
        .firstMoveSafe = true,
    };
    mss::Random rng(0xC0FFEE12345ULL, 0xD1B54A32D192ED03ULL);
    const mss::GameControl::Position::SuggestMode mode = mss::GameControl::Position::SuggestMode::Java;
    std::atomic<long long> steps{0};
    std::vector<RunSummary> workerSummaries(mss::kMaxCores);
    std::vector<std::jthread> workers;
    workers.reserve(mss::kMaxCores);
    const std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
    for (int worker = 0; worker < mss::kMaxCores; ++worker) {
        mss::Random workerRng(rng.next(), rng.next());
        workers.emplace_back([&, worker, workerRng]() mutable {
            workerSummaries[worker] = runGames(config, workerRng, mode, [&](const Snapshot &) {
                steps.fetch_add(1, std::memory_order_relaxed);
            });
        });
    }
    for (std::jthread &worker : workers)
        worker.join();
    RunSummary summary;
    for (const RunSummary &workerSummary : workerSummaries) {
        summary.games += workerSummary.games;
        summary.wins += workerSummary.wins;
        summary.losses += workerSummary.losses;
    }
    summary.elapsedSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    const double winRate = 100.0 * summary.wins / summary.games;
    const double gamesPerSecond = summary.games / summary.elapsedSeconds;
    std::cout << "performance/real-game: mode=" << (mode == mss::GameControl::Position::SuggestMode::Java ? "Java" : "LowRisk") << ", "
              << summary.games << " games, " << summary.wins << " wins (" << std::fixed << std::setprecision(3)
              << winRate << "%), " << gamesPerSecond << " games/s, " << steps.load(std::memory_order_relaxed) << " steps, " << summary.elapsedSeconds
              << "s\n";
}

} // namespace test
