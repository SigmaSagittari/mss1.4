#pragma once

#include <array>
#include <chrono>
#include <iostream>

#include "algo/bruteforce/bruteforce.h"
#include "algo/observed_board.h"
#include "test/common.h"

namespace test {

// 暴力残局测试同时对照普通后端与多掩码后端的推荐动作和胜局数。
inline void bruteforce() {
    constexpr std::array<const char*, 30> rows{{
        "00012?211111012?",
        "0001?33?11?101??",
        "112222?33321013?",
        "2?2?1123??11223?",
        "?332101?4321??3?",
        "12?100112?11222?",
        "0111000011100011",
        "111000000011112?",
        "1?210011113?33??",
        "12?2122?22???6??",
        "0112?3?33?45????",
        "00024?5?32?224??",
        "0001??4?32222332",
        "110122213?32??10",
        "?11110113??22210",
        "223?201?22221111",
        "1?3?20112112?21?",
        "112110001?12?211",
        "0122111122322111",
        "12??11?22?4?201?",
        "?333112?44??2011",
        "23?21012??332211",
        "?23?21122211?2?1",
        "222?32?211232211",
        "?223??3?11??2111",
        "?3?????22233?11?",
        "???????22?211111",
        "???????33?311011",
        "????????323?101?",
        "????????21?21011"}};
    mss::ObservedBoard::Result board =
        mss::ObservedBoard::analyze(30, 16, 99);
    for (int x = 1; x <= board.rows; ++x)
        for (int y = 1; y <= board.cols; ++y) {
            const char cell = rows[x - 1][y - 1];
            board.board[x][y] = cell == '?'
                                    ? mss::ObservedBoard::CellState::Hidden
                                    : (mss::ObservedBoard::CellState)(cell - '0');
        }
    const mss::Basic::Result basic = mss::Basic::analyze(board);
    mss::Structure::ShapePool shapes;
    const mss::Structure::Result structure =
        mss::Structure::analyze(board, basic, shapes);
    const mss::BruteForce::Config config{false, 1};
    const std::chrono::steady_clock::time_point start =
        std::chrono::steady_clock::now();
    const mss::BruteForce::Result result = mss::BruteForce::solve(
        board, basic, structure, shapes, config);
    const double milliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    check(result.possibilities == 134550, "slow endgame possibility count changed");
    std::cout << "test/bruteforce: slow-endgame wins="
              << (result.moves.empty() ? 0 : result.moves[0].wins)
              << " nodes=" << result.nodes << " time_ms=" << milliseconds << '\n';
}

}  // namespace test
