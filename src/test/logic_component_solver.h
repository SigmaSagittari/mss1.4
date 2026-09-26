#pragma once

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <span>
#include <utility>
#include <vector>

#include "core/workspace.h"
#include "test/common.h"

namespace test {

struct ExtractedLogicComponent {
    mss::ObservedBoard::Result board;
    int firstRow;
    int firstCol;
};

// 新盘只保留组件候选、相邻数字和这些约束周围的已知雷，其余原盘信息不参与计算。
inline ExtractedLogicComponent extractLogicComponent(const mss::ObservedBoard::Result &source, const mss::Basic::Result &basic,
                                                     const mss::LogicStructure::Result::LogicComponent &component,
                                                     const mss::Structure::Pool &shapes, int mines) {
    using State = mss::ObservedBoard::CellState;
    const int cellCount = (source.rows + 1) * (source.cols + 1);
    std::vector<char> componentCells(cellCount, 0), frontierCells(cellCount, 0), clueCells(cellCount, 0), relevantCells(cellCount, 0);
    int minRow = source.rows, maxRow = 1, minCol = source.cols, maxCol = 1;
    auto addCell = [&](mss::ObservedBoard::CellId cell, bool frontier) {
        componentCells[cell] = 1;
        frontierCells[cell] = frontier ? 1 : 0;
        const auto [x, y] = source.pos(cell);
        minRow = (std::min)(minRow, x);
        maxRow = (std::max)(maxRow, x);
        minCol = (std::min)(minCol, y);
        maxCol = (std::max)(maxCol, y);
    };
    for (const mss::Structure::Instance &instance : component.structComponents)
        for (mss::ObservedBoard::CellId cell : instance.boxes.cells.span(shapes.cells))
            addCell(cell, true);
    for (mss::ObservedBoard::CellId cell : component.offFrontierCells)
        addCell(cell, false);

    for (int x = 1; x <= source.rows; ++x)
        for (int y = 1; y <= source.cols; ++y)
            if (componentCells[source.id(x, y)])
                mss::forEachAdjacent(x, y, source.rows, source.cols, [&](int nx, int ny) {
                    relevantCells[source.id(nx, ny)] = 1;
                    if (frontierCells[source.id(x, y)] && (int)(source.board[nx][ny]) <= (int)(State::Num8)) {
                        clueCells[source.id(nx, ny)] = 1;
                        mss::forEachAdjacent(nx, ny, source.rows, source.cols,
                                             [&](int mx, int my) { relevantCells[source.id(mx, my)] = 1; });
                    }
                });

    const int firstRow = (std::max)(1, minRow - 2), firstCol = (std::max)(1, minCol - 2);
    const int lastRow = (std::min)(source.rows, maxRow + 2), lastCol = (std::min)(source.cols, maxCol + 2);
    mss::ObservedBoard::Result board = mss::ObservedBoard::analyze(lastRow - firstRow + 1, lastCol - firstCol + 1, 0);
    int fixedMines = 0;
    for (int x = 1; x <= board.rows; ++x)
        for (int y = 1; y <= board.cols; ++y) {
            const int sourceX = firstRow + x - 1, sourceY = firstCol + y - 1;
            const mss::ObservedBoard::CellId cell = source.id(sourceX, sourceY);
            State state = State::ForcedSafe;
            if (componentCells[cell])
                state = State::Hidden;
            else if (clueCells[cell])
                state = source.board[sourceX][sourceY];
            else if (relevantCells[cell] && basic.marks[sourceX][sourceY] == mss::Basic::Mark::F) {
                state = State::ForcedMine;
                ++fixedMines;
            }
            board.board[x][y] = state;
        }
    board.totalMines = fixedMines + mines;
    return {std::move(board), firstRow, firstCol};
}

inline mss::ObservedBoard::Result removeLogicComponent(const mss::ObservedBoard::Result &source,
                                                        const mss::Basic::Result &basic,
                                                        const mss::LogicStructure::Result::LogicComponent &component,
                                                        const mss::Structure::Pool &shapes, int mines) {
    using State = mss::ObservedBoard::CellState;
    mss::ObservedBoard::Result board = mss::ObservedBoard::analyze(source.rows, source.cols, source.totalMines - mines);
    for (int x = 1; x <= source.rows; ++x)
        for (int y = 1; y <= source.cols; ++y)
            if (source.board[x][y] == State::Hidden && basic.marks[x][y] == mss::Basic::Mark::F)
                board.board[x][y] = State::ForcedMine;
            else if (source.board[x][y] == State::Hidden && basic.marks[x][y] == mss::Basic::Mark::S)
                board.board[x][y] = State::ForcedSafe;
            else
                board.board[x][y] = source.board[x][y];
    auto removeCell = [&](mss::ObservedBoard::CellId cell) {
        const auto [x, y] = source.pos(cell);
        board.board[x][y] = State::ForcedSafe;
    };
    for (const mss::Structure::Instance &instance : component.structComponents)
        for (mss::ObservedBoard::CellId cell : instance.boxes.cells.span(shapes.cells)) {
            const auto [x, y] = source.pos(cell);
            removeCell(cell);
            mss::forEachAdjacent(x, y, source.rows, source.cols, [&](int nx, int ny) {
            if ((int)(source.board[nx][ny]) <= (int)(State::Num8))
                    board.board[nx][ny] = State::ForcedSafe;
            });
        }
    for (mss::ObservedBoard::CellId cell : component.offFrontierCells)
        removeCell(cell);
    return board;
}

struct LayoutDistribution {
    int start;
    std::vector<long double> ways;
};

inline LayoutDistribution multiplyLayoutWays(const LayoutDistribution &left, int rightStart, std::span<const long double> right) {
    if (left.ways.empty() || right.empty())
        return {left.start + rightStart, {}};
    std::vector<long double> ways(left.ways.size() + right.size() - 1, 0.0L);
    for (int i = 0; i < (int)(left.ways.size()); ++i)
        for (int j = 0; j < (int)(right.size()); ++j)
            ways[i + j] += left.ways[i] * right[j];
    return {left.start + rightStart, std::move(ways)};
}

inline LayoutDistribution countLayoutWays(const mss::Basic::Result &basic, const mss::Structure::Result &structure,
                                          const mss::Structure::Pool &shapes,
                                          mss::ShapeSolver::Distribution::Pool &distributions,
                                          mss::ShapeSolver::Workspace &shapeWorkspace, mss::Binom::Workspace &binomWorkspace) {
    LayoutDistribution result{0, {1.0L}};
    for (mss::Structure::InstanceId id : structure.components) {
        const mss::Structure::Instance &instance = shapes.getInstance(id);
        const mss::ShapeSolver::DistributionId distributionId =
            mss::ShapeSolver::analyze(shapes.getShape(instance.shape), shapes, distributions, shapeWorkspace);
        const mss::ShapeSolver::Distribution::Result &distribution = distributions.get(distributionId);
        result = multiplyLayoutWays(result, distribution.start(), distribution.ways());
    }
    if (basic.unknownSum > 0) {
        std::vector<long double> tWays(basic.unknownSum + 1);
        for (int mines = 0; mines <= basic.unknownSum; ++mines)
            tWays[mines] = mss::binom(basic.unknownSum, mines, binomWorkspace);
        result = multiplyLayoutWays(result, 0, tWays);
    }
    return result;
}

template <typename Fn>
inline RunSummary runNoSafeGames(const TestConfig &config, mss::Random &rng, mss::Workspace &workspace,
                                 mss::GameControl::Position::SuggestMode mode, Fn &&consume) {
    TimeBox timebox(config.seconds);
    RunSummary summary;
    mss::ObservedBoard::Delta updates;
    while ((config.games < 0 || summary.games < config.games) && !timebox.expired()) {
        mss::GameControl::Game game(config.rows, config.cols, config.mines, mss::U128{rng.next(), rng.next()}, workspace);
        const mss::ObservedBoard::CellId first = game.position.observedBoard.id((config.rows + 1) / 2, (config.cols + 1) / 2);
        game.makeFirstMoveSafe(first, mss::U128{rng.next(), rng.next()});
        const auto [x, y] = game.position.observedBoard.pos(first);
        updates.changes = {{first, (mss::ObservedBoard::CellState)(game.number(x, y))}};
        game.update(updates);
        game.playUntilNoSafe();
        if (!game.won()) {
            for (;;) {
                const std::vector<mss::ObservedBoard::CellId> next = game.Suggest(mode);
                mss::assert_(!next.empty(), "test::runNoSafeGames: Suggest returned no move");
                if (game.mineProbability(next.front()) != 0.0L)
                    break;
                game.playUntilNoSafe();
                if (game.won())
                    break;
            }
            if (!game.won())
                consume(game);
        }
        ++summary.games;
        if (game.won())
            ++summary.wins;
    }
    summary.elapsedSeconds = timebox.elapsedSeconds();
    return summary;
}

inline void compareExtractedMoves(const mss::ObservedBoard::Result &source, const ExtractedLogicComponent &extracted,
                                  const std::vector<int> &cellIndices, int candidateCount,
                                  const mss::BruteForce::Result &componentResult,
                                  const mss::BruteForce::Result &extractedResult) {
    check(componentResult.moves.size() == extractedResult.moves.size(), "component and extracted-board move counts differ");
    std::vector<int> wins(candidateCount, -1);
    for (const mss::BruteForce::Result::Move &move : extractedResult.moves) {
        const int x = extracted.firstRow + move.x - 1, y = extracted.firstCol + move.y - 1;
        const int index = cellIndices[source.id(x, y)];
        check(index >= 0 && wins[index] == -1, "extracted board returned a non-component or duplicate move");
        wins[index] = move.wins;
    }
    for (const mss::BruteForce::Result::Move &move : componentResult.moves) {
        const int index = cellIndices[source.id(move.x, move.y)];
        check(index >= 0 && wins[index] == move.wins, "component and extracted-board win counts differ");
        wins[index] = -2;
    }
    for (int value : wins)
        check(value == -2, "extracted-board reference omitted a component move");
}

inline void logicComponentSolver() {
    mss::Workspace workspace;
    const TestConfig config{
        .rows = 30,
        .cols = 16,
        .mines = 99,
        .seconds = 30.0,
        .games = -1,
        .filter = PositionFilter::All,
        .firstMoveSafe = true,
    };
    mss::Random rng(0xC0FFEE12345ULL, 0xD1B54A32D192ED03ULL);
    int noSafePositions = 0, checkedComponents = 0, tooLargeComponents = 0, checkedMineCounts = 0, offFrontierComponents = 0;
    int calculatedOffFrontierComponents = 0;
    double solverSeconds = 0.0, referenceSeconds = 0.0, bruteForceSeconds = 0.0;
    const RunSummary summary = runNoSafeGames(config, rng, workspace, mss::GameControl::Position::SuggestMode::LowRisk,
                                              [&](mss::GameControl::Game &game) {
        const mss::ObservedBoard::Result &board = game.position.observedBoard;
        const mss::Basic::Result &basic = game.position.basic();
        const mss::Structure::Result &structure = game.position.structure();
        const mss::Probability::Result &probability = game.probability();
        const mss::Structure::Pool &shapes = game.shapePool();
        mss::ShapeSolver::Distribution::Pool distributions;
        ++noSafePositions;

        auto phaseStart = std::chrono::steady_clock::now();
        const mss::LogicStructure::Result logicStructure = mss::LogicStructure::analyze(board, basic, structure, shapes);
        const mss::LogicComponentSolver::Result result =
            mss::LogicComponentSolver::analyze(board, basic, structure, probability, logicStructure, shapes, distributions,
                                               workspace.logicComponentSolver);
        solverSeconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - phaseStart).count();
        check(result.components.size() == logicStructure.components.size(), "logic-component result count differs");
        mss::BruteForce::Config options = mss::BruteForce::Config::allMoves();
        options.solver = mss::BruteForce::Solver::Common;
        const int remainingMines = board.totalMines - basic.mineSum;
        for (int cid = 0; cid < (int)(logicStructure.components.size()); ++cid) {
            phaseStart = std::chrono::steady_clock::now();
            const mss::LogicStructure::Result::LogicComponent &component = logicStructure.components[cid];
            const mss::LogicComponentSolver::ComponentResult &componentResult = result.components[cid];
            check(!componentResult.mineCounts.empty(), "logic component has no mine-count probabilities");
            if (!component.offFrontierCells.empty())
                ++offFrontierComponents;
            int candidateCount = (int)(component.offFrontierCells.size());
            for (const mss::Structure::Instance &instance : component.structComponents)
                candidateCount += (int)(instance.boxes.cells.size);
            ExtractedLogicComponent extractedAll = extractLogicComponent(board, basic, component, shapes, candidateCount);
            const mss::Basic::Result extractedAllBasic = mss::Basic::analyze(extractedAll.board, workspace.basic);
            check(extractedAllBasic.valid, "extracted component board is invalid");
            mss::Structure::Pool extractedShapes, complementShapes;
            mss::ShapeSolver::Distribution::Pool extractedDistributions, complementDistributions;
            const mss::Structure::Result extractedAllStructure =
                mss::Structure::analyze(extractedAll.board, extractedAllBasic, extractedShapes, workspace.structure);
            const LayoutDistribution componentWays =
                countLayoutWays(extractedAllBasic, extractedAllStructure, extractedShapes, extractedDistributions, workspace.shapeSolver,
                                workspace.binom);
            const mss::ObservedBoard::Result complement = removeLogicComponent(board, basic, component, shapes, 0);
            const mss::Basic::Result complementBasic = mss::Basic::analyze(complement, workspace.basic);
            const mss::Structure::Result complementStructure =
                mss::Structure::analyze(complement, complementBasic, complementShapes, workspace.structure);
            const LayoutDistribution complementWays =
                countLayoutWays(complementBasic, complementStructure, complementShapes, complementDistributions, workspace.shapeSolver,
                                workspace.binom);
            long double totalWays = 0.0L;
            for (int i = 0; i < (int)(componentWays.ways.size()); ++i) {
                const int outsideMines = remainingMines - componentWays.start - i - complementWays.start;
                if (outsideMines >= 0 && outsideMines < (int)(complementWays.ways.size()))
                    totalWays += componentWays.ways[i] * complementWays.ways[outsideMines];
            }
            check(totalWays > 0.0L && probability.candidates() > 0.0L &&
                      std::fabs((totalWays - probability.candidates()) / probability.candidates()) < 1e-12L,
                  "component and complementary layout counts do not reproduce the whole board");
            int mineCountIndex = 0;
            for (int i = 0; i < (int)(componentWays.ways.size()); ++i) {
                const int mines = componentWays.start + i;
                const int outsideMines = remainingMines - mines - complementWays.start;
                const long double outsideWays = outsideMines >= 0 && outsideMines < (int)(complementWays.ways.size())
                                                    ? complementWays.ways[outsideMines]
                                                    : 0.0L;
                const long double expected = componentWays.ways[i] * outsideWays / totalWays;
                if (expected == 0.0L) {
                    check(mineCountIndex == (int)(componentResult.mineCounts.size()) || componentResult.mineCounts[mineCountIndex].mines != mines,
                          "zero-probability mine count was stored");
                } else {
                    check(mineCountIndex < (int)(componentResult.mineCounts.size()), "feasible mine count was omitted");
                    const mss::LogicComponentSolver::MineCountResult &mineCount = componentResult.mineCounts[mineCountIndex++];
                    check(mineCount.mines == mines && mineCount.probability > 0.0L && std::fabs(mineCount.probability - expected) < 1e-12L,
                          "component mine-count probability differs from extracted-board reference");
                }
            }
            check(mineCountIndex == (int)(componentResult.mineCounts.size()), "component contains an out-of-range mine count");
            referenceSeconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - phaseStart).count();

            const bool tooLarge = componentResult.status == mss::LogicComponentSolver::ComponentResult::Status::TooLarge;
            bool exceedsBruteForceLimit = false;
            for (const mss::LogicComponentSolver::MineCountResult &mineCount : componentResult.mineCounts)
                if (componentWays.ways[mineCount.mines - componentWays.start] > mss::LogicComponentSolver::maxBruteForceWays)
                    exceedsBruteForceLimit = true;
            check(tooLarge == exceedsBruteForceLimit, "component brute-force limit status differs");
            if (tooLarge) {
                ++tooLargeComponents;
                check(componentResult.recommendation.cell == -1, "too-large component has a recommendation");
                for (const mss::LogicComponentSolver::MineCountResult &mineCount : componentResult.mineCounts)
                    check(mineCount.bruteForce.moves.empty(), "too-large component contains brute-force moves");
                continue;
            }
            ++checkedComponents;
            if (!component.offFrontierCells.empty())
                ++calculatedOffFrontierComponents;
            std::vector<mss::ObservedBoard::CellId> cells;
            std::vector<int> cellIndices((board.rows + 1) * (board.cols + 1), -1);
            auto addCandidate = [&](mss::ObservedBoard::CellId cell) {
                check(cellIndices[cell] == -1, "logic component contains a duplicate candidate");
                cellIndices[cell] = cells.size();
                cells.push_back(cell);
            };
            for (const mss::Structure::Instance &instance : component.structComponents)
                for (mss::ObservedBoard::CellId cell : instance.boxes.cells.span(shapes.cells))
                    addCandidate(cell);
            for (mss::ObservedBoard::CellId cell : component.offFrontierCells)
                addCandidate(cell);
            check(!cells.empty(), "calculated logic component has no candidates");
            std::vector<long double> winProbabilities(cells.size(), 0.0L);
            for (const mss::LogicComponentSolver::MineCountResult &mineCount : componentResult.mineCounts) {
                const int i = mineCount.mines - componentWays.start;
                check(mineCount.probability > 0.0L && componentWays.ways[i] > 0.0L, "stored mine count is not feasible");
                ExtractedLogicComponent extracted = extractLogicComponent(board, basic, component, shapes, mineCount.mines);
                const mss::Basic::Result perMineBasic = mss::Basic::analyze(extracted.board, workspace.basic);
                check(perMineBasic.valid, "fixed-mine extracted component board is invalid");
                const mss::Structure::Result perMineStructure =
                    mss::Structure::analyze(extracted.board, perMineBasic, extractedShapes, workspace.structure);
                const mss::Probability::Result perMineProbability =
                    mss::Probability::analyze(extracted.board, perMineBasic, perMineStructure, extractedShapes, extractedDistributions,
                                              workspace.probability);
                check(std::fabs(perMineProbability.candidates() - componentWays.ways[i]) < 1e-12L,
                      "extracted fixed-mine layout count differs");
                const mss::BruteForce::Result extractedResult =
                    mss::BruteForce::solve(extracted.board, perMineBasic, perMineStructure, extractedShapes, options, workspace.bruteForce);
                bruteForceSeconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - phaseStart).count();
                phaseStart = std::chrono::steady_clock::now();
                compareExtractedMoves(board, extracted, cellIndices, (int)(cells.size()), mineCount.bruteForce, extractedResult);
                ++checkedMineCounts;
                for (const mss::BruteForce::Result::Move &move : extractedResult.moves) {
                    const int x = extracted.firstRow + move.x - 1, y = extracted.firstCol + move.y - 1;
                    const int index = cellIndices[board.id(x, y)];
                    winProbabilities[index] += mineCount.probability * move.wins / componentWays.ways[i];
                }
            }
            int best = 0;
            for (int i = 1; i < (int)(winProbabilities.size()); ++i)
                if (winProbabilities[i] > winProbabilities[best])
                    best = i;
            check(componentResult.recommendation.cell >= 0, "calculated logic component has no recommendation");
            const int recommendation = cellIndices[componentResult.recommendation.cell];
            check(recommendation >= 0 &&
                      std::fabs(componentResult.recommendation.winProbability - winProbabilities[best]) < 1e-12L &&
                      std::fabs(winProbabilities[recommendation] - winProbabilities[best]) < 1e-12L,
                  "component recommendation differs from extracted-board reference");
            referenceSeconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - phaseStart).count();
        }
    });
    std::cout << "test/logic_component_solver: games=" << summary.games << " no_safe=" << noSafePositions
              << " calculated_components=" << checkedComponents << " too_large=" << tooLargeComponents
              << " fixed_mine_counts=" << checkedMineCounts << " off_frontier_components=" << offFrontierComponents
              << " calculated_off_frontier=" << calculatedOffFrontierComponents << " elapsed=" << std::fixed << std::setprecision(3)
              << summary.elapsedSeconds << "s solver=" << solverSeconds << "s reference=" << referenceSeconds
              << "s fixed_mine_bf=" << bruteForceSeconds << "s\n";
    check(noSafePositions > 0 && checkedComponents > 12 && offFrontierComponents > 0 && calculatedOffFrontierComponents > 0,
          "batch did not calculate enough no-safe off-frontier components");
}
} // namespace test
