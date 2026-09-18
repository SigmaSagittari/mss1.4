#pragma once

#include <array>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string_view>
#include <vector>

#include "algo/observed_board.h"
#include "test/common.h"

namespace test {

// 按固定安全点序列逐点打开，并在每次打开后重算概率和耗时。
inline void probabilityCase() {
    constexpr int kRows = 10;
    constexpr int kCols = 21;
    constexpr int kMines = 78;
    constexpr double kTimeoutSeconds = 100.0;
    constexpr std::array<std::string_view, kRows> kRowsData{
        {"HMHMHMHMHMHMHMHMHMHHH", "HMHHHMHHHMHHHMHHHMHMH", "HMHMHMHMHMHMHMHMHMHMH", "HMHHHMHHHMHHHMHHHMHHH", "HMHMHMHMHMHMHMHMHMHMH",
         "HMHHHMHHHMHHHMHHHMHMH", "HMHMHMHMHMHMHMHMHMHMH", "HMHHHMHHHMHHHMHHHMHMH", "HMHMHMHMHMHMHMHMHMHMH", "HMHHHMHHHMHHHMHHHMHMH"}};

    mss::ObservedBoard::Result board = mss::ObservedBoard::analyze(kRows, kCols, kMines);
    mss::RawGrid<char> mines(kRows, kCols, 0);
    int mineCount = 0;
    for (int x = 1; x <= kRows; ++x) {
        check(kRowsData[x - 1].size() == kCols, "probability case row width is incorrect");
        for (int y = 1; y <= kCols; ++y) {
            const char cell = kRowsData[x - 1][y - 1];
            check(cell == 'H' || cell == 'M', "probability case contains an invalid cell");
            if (cell == 'M') {
                mines[x - 1][y - 1] = 1;
                ++mineCount;
            }
        }
    }
    check(mineCount == kMines, "probability case mine count is incorrect");

    const TimeBox timebox(kTimeoutSeconds);
    Analysis analysis(board, mss::ShapeSolver::OrderAlgo::AutoSA);
    long long totalSteps = 0;
    double totalCalculationMilliseconds = 0.0;
    bool timedOut = false;
    std::cout << "test/probability-case: " << kCols << 'x' << kRows << 'x' << kMines << ", sequence=(1,1),(1,3),...,(9,21)\n";
    std::cout << std::fixed << std::setprecision(6);
    for (int x = 1; x <= kRows && !timedOut; x += 2)
        for (int y = 1; y <= kCols && !timedOut; y += 2) {
            check(mines[x - 1][y - 1] == 0, "probability case opened a mine");
            const long double clickProbability =
                analysis.probability.mineProbability(board.id(x, y), board, analysis.basic, analysis.structure);
            const std::chrono::steady_clock::time_point stepStart = std::chrono::steady_clock::now();
            int adjacentMines = 0;
            mss::forEachAdjacent(x, y, kRows, kCols, [&](int nx, int ny) {
                adjacentMines += mines[nx - 1][ny - 1];
            });
            mss::ObservedBoard::Delta updates;
            updates.changes.push_back({board.id(x, y), (mss::ObservedBoard::CellState)(adjacentMines)});
            analysis.update(board, updates);
            const double calculationMilliseconds =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - stepStart).count();
            check(analysis.basic.valid, "probability case became invalid");
            check(analysis.probability.candidates() > 0.0L, "probability case has no consistent mine layouts");
            ++totalSteps;
            totalCalculationMilliseconds += calculationMilliseconds;
            timedOut = timebox.expired();
            std::cout << "  step=" << totalSteps << " open=(" << x << ',' << y << ") number=" << adjacentMines
                      << " click-risk=" << clickProbability * 100.0L << "% candidates=" << std::setprecision(18)
                      << analysis.probability.candidates() << std::setprecision(6)
                      << " t-cell=" << analysis.probability.tCellProbability() * 100.0L << "% calc_ms=" << calculationMilliseconds << '\n';
        }
    std::cout << "  steps=" << totalSteps << ", calc_total_ms=" << totalCalculationMilliseconds
              << ", elapsed_ms=" << timebox.elapsedSeconds() * 1000.0 << ", timeout_s=" << kTimeoutSeconds
              << ", timed_out=" << std::boolalpha << timedOut << '\n';
}

} // namespace test
