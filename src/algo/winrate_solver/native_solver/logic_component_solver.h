#pragma once

#include <span>
#include <vector>

#include "core/assert.h"
#include "algo/probability_engine/bruteforce/bruteforce.h"
#include "algo/probability_engine/probability/probability.h"
#include "algo/probability_engine/shape_solver/shape_solver.h"
#include "algo/winrate_solver/native_solver/logic_structure.h"

namespace mss {
struct LogicComponentSolver {
    static constexpr long double maxBruteForceWays = 1000.0L;

    struct Workspace {
        Probability::Workspace &probability;
        BruteForce::Workspace &bruteForce;
    };

    struct MineCountResult {
        int mines = 0; // 此档固定的组件雷数 k。
        long double probability = 0.0L; // 满足整盘约束的布局中，组件恰有 k 雷的比例。
        BruteForce::Result bruteForce; // 固定 k 雷时各格的可赢布局数；对应布局数为该档 ways[k]。
    };

    // 对同一格累加“该雷数档的 probability × 该格在该档的胜率”，返回总胜率最高的格。
    struct Recommendation {
        ObservedBoard::CellId cell = -1; // 总加权胜率最高的候选格。
        long double winProbability = 0.0L; // cell 的总加权胜率。
    };

    struct ComponentResult {
        enum class Status { Calculated, TooLarge };

        Status status = Status::TooLarge; // 任一全局可行雷数档 ways[k] 超过上限时保留概率、不跑暴力；recommendation 无效。
        Recommendation recommendation;
        std::vector<MineCountResult> mineCounts; // 只保留整盘概率非零的雷数档。
    };

    struct Result {
        // 与 LogicStructure::Result::components 一一对应，顺序相同。
        std::vector<ComponentResult> components;
    };

    static Result analyze(const ObservedBoard::Result &board, const Basic::Result &basic, const Structure::Result &structure,
                         const Probability::Result &probability, const LogicStructure::Result &logicStructure,
                         const Structure::structPool &shapes, ShapeSolver::Distribution::Pool &distributions, Workspace &workspace);
};

} // namespace mss

namespace mss {
inline LogicComponentSolver::Result LogicComponentSolver::analyze(
    const ObservedBoard::Result &board, const Basic::Result &basic, const Structure::Result &, const Probability::Result &probability,
    const LogicStructure::Result &logicStructure, const Structure::structPool &shapes, ShapeSolver::Distribution::Pool &distributions,
    Workspace &workspace) {
    Result result;
    if (logicStructure.components.empty())
        return result;
    assert_(probability.candidates() > 0.0L, "LogicComponentSolver::analyze: board has no valid layout");

    std::vector<std::vector<long double>> ways(logicStructure.components.size());
    std::vector<int> starts(logicStructure.components.size(), 0);
    for (int cid = 0; cid < (int)(logicStructure.components.size()); ++cid) {
        ways[cid].push_back(1.0L);
        const LogicStructure::Result::LogicComponent &component = logicStructure.components[cid];
        for (const Structure::Instance &instance : component.structComponents) {
            const ShapeSolver::DistributionId id =
                ShapeSolver::analyze(shapes.getShape(instance.shape), shapes, distributions, workspace.probability.shapeSolver);
            const ShapeSolver::Distribution::Result &distribution = distributions.get(id);
            const std::span<const long double> factor = distribution.ways();
            std::vector<long double> product(ways[cid].size() + factor.size() - 1, 0.0L);
            for (int i = 0; i < (int)(ways[cid].size()); ++i)
                for (int j = 0; j < (int)(factor.size()); ++j)
                    product[i + j] += ways[cid][i] * factor[j];
            ways[cid].swap(product);
            starts[cid] += distribution.start();
        }
        const int offFrontierCount = (int)(component.offFrontierCells.size());
        if (offFrontierCount > 0) {
            std::vector<long double> product(ways[cid].size() + offFrontierCount, 0.0L);
            long double combinations = 1.0L;
            for (int tMines = 0; tMines <= offFrontierCount; ++tMines) {
                for (int i = 0; i < (int)(ways[cid].size()); ++i)
                    product[i + tMines] += ways[cid][i] * combinations;
                combinations *= (long double)(offFrontierCount - tMines) / (tMines + 1);
            }
            ways[cid].swap(product);
        }
    }

    std::vector<Probability::Poly> polynomials;
    polynomials.reserve(ways.size());
    for (int cid = 0; cid < (int)(ways.size()); ++cid)
        polynomials.push_back({starts[cid], std::span<const long double>(ways[cid])});
    const Probability::DistributionProbabilityResult probabilities =
        Probability::analyzeDistributions(polynomials, 0, board.totalMines - basic.mineSum, workspace.probability);
    result.components.resize(ways.size());
    for (int cid = 0; cid < (int)(ways.size()); ++cid) {
        const std::span<const long double> mineCountProbabilities = probabilities.mineCountProbabilities[cid];
        ComponentResult &componentResult = result.components[cid];
        componentResult.mineCounts.reserve(mineCountProbabilities.size());
        bool tooLarge = false;
        for (int i = 0; i < (int)(mineCountProbabilities.size()); ++i) {
            if (mineCountProbabilities[i] == 0.0L)
                continue;
            componentResult.mineCounts.push_back({starts[cid] + i, mineCountProbabilities[i], {}});
            if (ways[cid][i] > maxBruteForceWays)
                tooLarge = true;
        }
        if (tooLarge)
            continue;

        const LogicStructure::Result::LogicComponent &logicComponent = logicStructure.components[cid];
        std::vector<ObservedBoard::CellId> recommendedCells;
        std::vector<long double> winProbabilities;
        for (MineCountResult &mineCount : componentResult.mineCounts) {
            const int i = mineCount.mines - starts[cid];
            mineCount.bruteForce = BruteForce::solveComponent(board, basic, logicComponent.structComponents,
                                                              logicComponent.offFrontierCells, shapes, mineCount.mines,
                                                              BruteForce::Config::allMoves(), workspace.bruteForce);
            const std::vector<BruteForce::Result::Move> &moves = mineCount.bruteForce.moves;
            if (recommendedCells.empty()) {
                recommendedCells.reserve(moves.size());
                winProbabilities.resize(moves.size(), 0.0L);
                for (const BruteForce::Result::Move &move : moves)
                    recommendedCells.push_back(board.id(move.x, move.y));
            }
            for (int move = 0; move < (int)(moves.size()); ++move)
                winProbabilities[move] += mineCount.probability * moves[move].wins / ways[cid][i];
        }
        for (int move = 0; move < (int)(recommendedCells.size()); ++move)
            if (move == 0 || winProbabilities[move] > componentResult.recommendation.winProbability)
                componentResult.recommendation = {recommendedCells[move], winProbabilities[move]};
        componentResult.status = ComponentResult::Status::Calculated;
    }
    return result;
}
} // namespace mss
