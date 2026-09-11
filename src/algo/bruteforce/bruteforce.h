#pragma once

#include <vector>

#include "algo/basic.h"
#include "algo/observed_board.h"
#include "algo/probability/probability.h"
#include "algo/shape_solver/shape_solver.h"
#include "algo/structure.h"

namespace mss {

namespace BruteForce {

struct Config {
    bool checkAllMoves;
    int minWins;
};

struct Result {
    struct Move {
        int x = 0;
        int y = 0;
        int wins = 0;
    };

    int possibilities = 0;
    long long nodes = 0;
    std::vector<Move> moves;
};

// 在当前盘面可能性上进行残局搜索。
// Config 的所有字段必须由调用方显式指定；minWins 只影响非 checkAllMoves 模式。
Result solve(const ObservedBoard::Result& board,
             const Basic::Result& basic,
             const Structure::Result& structure,
             const Probability::Result& probability,
             const Structure::ShapePool& shapes,
             ShapeSolver::Distribution::Pool& distributions,
             const Config& config);

}  // namespace BruteForce

}  // namespace mss
