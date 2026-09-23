#pragma once

#include <vector>

#include "algo/probability_engine/bruteforce/bruteforce_common.h"
#include "algo/probability_engine/shape_solver/shape_solver.h"
#include "algo/winrate_solver/native_solver/logic_structure.h"

namespace mss {

struct LogicComponentSolver {
    struct MineCountResult {
        int mines = 0; // 此档固定的组件雷数 k。
        long double probability = 0.0L; // 满足整盘约束的布局中，组件恰有 k 雷的比例。
        BruteForce::Result bruteForce; // 固定 k 雷并以 checkAllMoves=true 搜索；Move::wins / layoutCount 是此档逐格胜率。
    };

    // 对同一格累加“该雷数档的 probability × 该格在该档的胜率”，返回总胜率最高的格。
    struct Recommendation {
        CellId cell = -1; // 总加权胜率最高的候选格。
        long double winProbability = 0.0L; // cell 的总加权胜率。
    };

    struct ComponentResult {
        enum class Status { Calculated, TooLarge };

        Status status = Status::TooLarge; // TooLarge 时 mineCounts 仍有概率，但不含逐格搜索结果；recommendation 无效。
        Recommendation recommendation;
        std::vector<MineCountResult> mineCounts;
    };

    struct Result {
        // 与 LogicStructure::Result::components 一一对应，顺序相同。
        std::vector<ComponentResult> components;
    };

    static Result analyze(const ObservedBoard::Result &board, const Basic::Result &basic, const Structure::Result &structure,
                         const Probability::Result &probability, const LogicStructure::Result &logicStructure,
                         const Structure::structPool &shapes, ShapeSolver::Distribution::Pool &distributions);
};

} // namespace mss
