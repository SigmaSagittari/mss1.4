#pragma once

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "core/workspace.h"
#include "core/assert.h"
#include "core/utility/neighbors.h"
#include "core/utility/grid.h"
#include "core/utility/hash.h"
#include "core/utility/rng.h"

namespace mss {

struct GameControl {
  struct Game;

  struct mineBoard {
    mineBoard() = default;
    explicit mineBoard(Grid<char> board) : board_(std::move(board)) {
    }
    void generate(int rows, int cols, int mines, U128 seed);
    int number(int x, int y) const;

  private:
    void makeSafe(int x, int y, U128 seed);
    bool matches(const ObservedBoard::Result &observedBoard) const;

    friend struct Game;

    Grid<char> board_;
  };

  struct Position {
  public:
    enum class SuggestMode {
        Java,
        LowRisk,
    };

    ObservedBoard::Result observedBoard;

    struct Error {
        bool failed = false;
        std::string reason;
    } error;

    Position(ObservedBoard::Result observedBoard, Workspace &workspace,
             ShapeSolver::OrderAlgo orderAlgo = ShapeSolver::OrderAlgo::Auto);
    Position(ObservedBoard::Result observedBoard, Structure::structPool &structPool,
             ShapeSolver::Distribution::Pool &distributionPool,
             Workspace &workspace, ShapeSolver::OrderAlgo orderAlgo = ShapeSolver::OrderAlgo::Auto);
    void update(ObservedBoard::Delta &updates, Structure::structPool &structPool);
    const Basic::Result &basic() const;
    const Structure::Result &structure() const;
    const Probability::Result &probability(Structure::structPool &structPool,
                                           ShapeSolver::Distribution::Pool &distributionPool);
    long double mineProbability(ObservedBoard::CellId cell, Structure::structPool &structPool,
                                ShapeSolver::Distribution::Pool &distributionPool);
    // 一次返回希望打开的格子；确定安全时可多个，猜测时约定返回一个。
    std::vector<ObservedBoard::CellId> Suggest(SuggestMode mode, Structure::structPool &structPool,
                                ShapeSolver::Distribution::Pool &distributionPool);

  private:
    friend struct Game;

    std::vector<ObservedBoard::CellId> safeCell_;
    std::vector<ObservedBoard::CellId> basicSafeCell_;
    int remainingSafe_ = 0;

    std::unique_ptr<Basic::Result> basic_;
    std::unique_ptr<Structure::Result> structure_;
    std::unique_ptr<Probability::Result> probability_;

    ShapeSolver::OrderAlgo orderAlgo = ShapeSolver::OrderAlgo::Auto;
    Workspace &workspace_;
    BruteForce::Solver bruteForceAlgo = BruteForce::Solver::Bitwise;
    bool probabilityDirty_ = false;

    Basic::Delta basicDelta_;
    Structure::Delta structureDelta_;
    void initialize(Structure::structPool &structPool, ShapeSolver::Distribution::Pool &distributionPool);
    std::vector<ObservedBoard::CellId> safeMove(Structure::structPool &structPool, ShapeSolver::Distribution::Pool &distributionPool);
  };

  struct Game {
  private:
    std::unique_ptr<mineBoard> mineBoard_;
    std::unique_ptr<Structure::structPool> structPool_;
    std::unique_ptr<ShapeSolver::Distribution::Pool> distributionPool_;

  public:
    Position position;

    Game(mineBoard mineBoard, ObservedBoard::Result observedBoard, Workspace &workspace,
         ShapeSolver::OrderAlgo orderAlgo = ShapeSolver::OrderAlgo::Auto);
    Game(int rows, int cols, int mines, U128 seed, Workspace &workspace, const ObservedBoard::Result *observedBoard = nullptr,
         ShapeSolver::OrderAlgo orderAlgo = ShapeSolver::OrderAlgo::Auto);
    void reset(U128 seed);
    void update(ObservedBoard::Delta &updates);
    void makeFirstMoveSafe(ObservedBoard::CellId cell, U128 seed);
    int number(int x, int y) const;
    bool won() const;
    const Probability::Result &probability();
    long double mineProbability(ObservedBoard::CellId cell);
    const Structure::structPool &shapePool() const;
    std::vector<ObservedBoard::CellId> Suggest(Position::SuggestMode mode);
    void playUntilNoSafe();

  };
};

} // namespace mss

//==============================================================================
namespace mss {

inline GameControl::Position::Position(ObservedBoard::Result observedBoard, Workspace &workspace, ShapeSolver::OrderAlgo orderAlgo)
    : observedBoard(std::move(observedBoard)), orderAlgo(orderAlgo), workspace_(workspace) {
    Structure::structPool structPool;
    ShapeSolver::Distribution::Pool distributionPool;
    initialize(structPool, distributionPool);
    structure_.reset();
}

inline GameControl::Position::Position(ObservedBoard::Result observedBoard, Structure::structPool &structPool,
                                       ShapeSolver::Distribution::Pool &distributionPool, Workspace &workspace,
                                       ShapeSolver::OrderAlgo orderAlgo)
    : observedBoard(std::move(observedBoard)), orderAlgo(orderAlgo), workspace_(workspace) {
    initialize(structPool, distributionPool);
}

inline const Basic::Result &GameControl::Position::basic() const {
    return *basic_;
}

inline const Structure::Result &GameControl::Position::structure() const {
    return *structure_;
}

inline const Probability::Result &GameControl::Position::probability(Structure::structPool &structPool,
                                                                     ShapeSolver::Distribution::Pool &distributionPool) {
    if (probabilityDirty_) {
        Probability::analyze(observedBoard, *basic_, *structure_, structPool, distributionPool, *probability_, workspace_.probability,
                             orderAlgo);
        probabilityDirty_ = false;
    }
    return *probability_;
}

inline long double GameControl::Position::mineProbability(ObservedBoard::CellId cell, Structure::structPool &structPool,
                                                          ShapeSolver::Distribution::Pool &distributionPool) {
    return probability(structPool, distributionPool).mineProbability(cell, observedBoard, *basic_, *structure_);
}

inline void GameControl::Position::initialize(Structure::structPool &structPool,
                                               ShapeSolver::Distribution::Pool &distributionPool) {
    error = {};
    safeCell_.clear();
    basicSafeCell_.clear();
    basicDelta_ = {};
    structureDelta_ = {};
    probabilityDirty_ = false;
    basic_.reset();
    structure_.reset();
    probability_.reset();
    remainingSafe_ = observedBoard.rows * observedBoard.cols - observedBoard.totalMines;
    for (int x = 1; x <= observedBoard.rows; ++x)
        for (int y = 1; y <= observedBoard.cols; ++y)
            if ((int)observedBoard.board[x][y] < (int)ObservedBoard::CellState::Hidden)
                --remainingSafe_;
    basic_ = std::make_unique<Basic::Result>(Basic::analyze(observedBoard, workspace_.basic));
    if (!basic_->valid) {
        error.failed = true;
        error.reason = "observedBoard has no valid Basic solution";
        return;
    }
    structure_ = std::make_unique<Structure::Result>(Structure::analyze(observedBoard, *basic_, structPool, workspace_.structure));
    probability_ = std::make_unique<Probability::Result>();
    Probability::analyze(observedBoard, *basic_, *structure_, structPool, distributionPool, *probability_, workspace_.probability,
                         orderAlgo);
    if (probability_->candidates() == 0.0L) {
        error.failed = true;
        error.reason = "observedBoard has no possible mine distribution";
        return;
    }
    basicSafeCell_.clear();
    for (int x = 1; x <= observedBoard.rows; ++x)
        for (int y = 1; y <= observedBoard.cols; ++y)
            if (observedBoard.board[x][y] == ObservedBoard::CellState::Hidden && basic_->marks[x][y] == Basic::Mark::Safe)
                basicSafeCell_.push_back(observedBoard.id(x, y));
}

inline void GameControl::Position::update(ObservedBoard::Delta &updates, Structure::structPool &structPool) {
    for (const ObservedBoard::Change &change : updates.changes)
        if ((int)change.next < (int)ObservedBoard::CellState::Hidden)
            --remainingSafe_;
    if (!structure_)
        structure_ = std::make_unique<Structure::Result>(Structure::analyze(observedBoard, *basic_, structPool, workspace_.structure));
    ObservedBoard::update(observedBoard, updates);
    Basic::update(*basic_, basicDelta_, observedBoard, updates, workspace_.basic);
    Structure::update(*structure_, structureDelta_, observedBoard, *basic_, structPool, updates, workspace_.structure);
    structureDelta_.removed.clear();
    structureDelta_.removedData.clear();
    structureDelta_.added.clear();
    structureDelta_.addedData.clear();
    for (const Basic::Delta::Change &change : basicDelta_.changes) {
        if (change.now != Basic::Mark::Safe)
            continue;
        const auto [x, y] = observedBoard.pos(change.cell);
        if (observedBoard.board[x][y] == ObservedBoard::CellState::Hidden)
            basicSafeCell_.push_back(change.cell);
    }
    probabilityDirty_ = true;
}

inline std::vector<ObservedBoard::CellId> GameControl::Position::safeMove(Structure::structPool &structPool,
                                                           ShapeSolver::Distribution::Pool &distributionPool) {
    safeCell_.clear();
    if (error.failed)
        return std::move(safeCell_);
    for (ObservedBoard::CellId cell : basicSafeCell_) {
        const auto [x, y] = observedBoard.pos(cell);
        if (observedBoard.board[x][y] == ObservedBoard::CellState::Hidden && basic_->marks[x][y] == Basic::Mark::Safe)
            safeCell_.push_back(cell);
    }
    basicSafeCell_ = safeCell_;
    basicDelta_.changes.clear();
    if (!safeCell_.empty() || !basic_->valid)
        return std::move(safeCell_);
    if (!structure_)
        structure_ = std::make_unique<Structure::Result>(Structure::analyze(observedBoard, *basic_, structPool, workspace_.structure));

    for (Structure::InstanceId instanceId : structure_->components) {
        const Structure::Instance &instance = structPool.getInstance(instanceId);
        const Structure::Shape &shape = structPool.getShape(instance.shape);
        const ShapeSolver::DistributionId distributionId = ShapeSolver::analyze(shape, structPool, distributionPool, workspace_.shapeSolver, orderAlgo);
        const ShapeSolver::Distribution::Result &distribution = distributionPool.get(distributionId);
        const std::span<const long double> ways = distribution.ways();
        if (ways.empty())
            continue;
        const std::span<const int> offsets = instance.boxes.boxOf.span(structPool.boxOf);
        const std::span<const ObservedBoard::CellId> cells = instance.boxes.cells.span(structPool.cells);
        for (int box = 0; box < distribution.boxCount(); ++box) {
            bool safe = true;
            for (std::size_t i = 0; i < ways.size(); ++i)
                if (ways[i] != 0.0L && distribution.perBoxExpectation(i)[box] != 0.0L) {
                    safe = false;
                    break;
                }
            if (!safe)
                continue;
            for (int i = offsets[box]; i < offsets[box + 1]; ++i)
                safeCell_.push_back(cells[i]);
        }
        if (!safeCell_.empty())
            return std::move(safeCell_);
    }

    Probability::analyze(observedBoard, *basic_, *structure_, structPool, distributionPool, *probability_, workspace_.probability,
                         orderAlgo);
    probabilityDirty_ = false;
    if (probability_->candidates() == 0.0L)
        return std::move(safeCell_);
    probability_->frontierCells(observedBoard, *structure_, structPool, [&](int x, int y, long double probability) {
        if (probability == 0.0L)
            safeCell_.push_back(observedBoard.id(x, y));
    });
    if (probability_->tCellProbability() == 0.0L && basic_->unknownSum != 0)
        for (int x = 1; x <= observedBoard.rows; ++x)
            for (int y = 1; y <= observedBoard.cols; ++y)
                if (observedBoard.board[x][y] == ObservedBoard::CellState::Hidden && basic_->marks[x][y] == Basic::Mark::Unknown &&
                    structure_->cellLoc[observedBoard.id(x, y)].component == -1)
                    safeCell_.push_back(observedBoard.id(x, y));
    return std::move(safeCell_);
}

inline std::vector<ObservedBoard::CellId> GameControl::Position::Suggest(SuggestMode mode, Structure::structPool &structPool,
                                                          ShapeSolver::Distribution::Pool &distributionPool) {
    std::vector<ObservedBoard::CellId> safeCells = safeMove(structPool, distributionPool);
    if (!safeCells.empty() || error.failed || !basic_->valid)
        return safeCells;
    if (probability_->candidates() == 0.0L)
        return {};

    // 严禁任何 eps 判 0/1 的方法；确定安全/确定雷必须直接写成 == 0.0L / == 1.0L。
    ObservedBoard::Delta forcedMineUpdates;
    probability_->frontierCells(observedBoard, *structure_, structPool, [&](int x, int y, long double mineProbability) {
        if (mineProbability == 1.0L)
            forcedMineUpdates.changes.push_back({observedBoard.id(x, y), ObservedBoard::CellState::ForcedMine});
    });
    if (probability_->tCellProbability() == 1.0L && basic_->unknownSum != 0)
        for (int x = 1; x <= observedBoard.rows; ++x)
            for (int y = 1; y <= observedBoard.cols; ++y)
                if (observedBoard.board[x][y] == ObservedBoard::CellState::Hidden && basic_->marks[x][y] == Basic::Mark::Unknown)
                    forcedMineUpdates.changes.push_back({observedBoard.id(x, y), ObservedBoard::CellState::ForcedMine});
    if (!forcedMineUpdates.changes.empty()) {
        update(forcedMineUpdates, structPool);
        safeCells = safeMove(structPool, distributionPool);
        if (!safeCells.empty() || error.failed || !basic_->valid)
            return safeCells;
        if (probability_->candidates() == 0.0L)
            return {};
    }

    if (mode == SuggestMode::LowRisk) {
        ObservedBoard::CellId safest = -1;
        long double lowestMineProbability = 2.0L;
        probability_->frontierCells(observedBoard, *structure_, structPool, [&](int x, int y, long double probability) {
            if (probability < lowestMineProbability) {
                lowestMineProbability = probability;
                safest = observedBoard.id(x, y);
            }
        });
        if (basic_->unknownSum != 0) {
            ObservedBoard::CellId offEdge = -1;
            for (int x = 1; x <= observedBoard.rows && offEdge == -1; ++x)
                for (int y = 1; y <= observedBoard.cols; ++y)
                    if (observedBoard.board[x][y] == ObservedBoard::CellState::Hidden && basic_->marks[x][y] == Basic::Mark::Unknown &&
                        structure_->cellLoc[observedBoard.id(x, y)].component == -1) {
                        offEdge = observedBoard.id(x, y);
                        break;
                    }
            if (probability_->tCellProbability() < lowestMineProbability)
                safest = offEdge;
        }
        if (safest == -1)
            return {};
        return {safest};
    }

    if (mode == SuggestMode::Java) {
        if (probability_->candidates() < 5000.0L) {
            const BruteForce::Result result = BruteForce::solve(
                observedBoard, *basic_, *structure_, structPool, {false, 1, BruteForce::Solver::Bitwise}, workspace_.bruteForce);
            if (!result.moves.empty())
                return {observedBoard.id(result.moves.front().x, result.moves.front().y)};
        }
        const LongTermRiskReference::Config riskConfig{};
        const LongTermRiskReference::Influence risk = LongTermRiskReference::findInfluence(
            observedBoard, *basic_, *structure_, *probability_, structPool, distributionPool, {}, riskConfig, workspace_.longTermRisk);
        const JavaEvaluate::Config evaluateConfig{};
        const JavaEvaluate::Result result = JavaEvaluate::solve(
            observedBoard, *basic_, *structure_, *probability_, structPool, distributionPool, risk, {}, evaluateConfig,
            workspace_.javaEvaluate);
        if (result.x == 0 || result.y == 0)
            return {};
        return {observedBoard.id(result.x, result.y)};
    }

    assert_(false, "Position::Suggest: invalid SuggestMode");
#if defined(_MSC_VER)
    __assume(0);
#else
    __builtin_unreachable();
#endif
}

inline void GameControl::Game::update(ObservedBoard::Delta &updates) {
    position.update(updates, *structPool_);
}

inline void GameControl::Game::makeFirstMoveSafe(ObservedBoard::CellId cell, U128 seed) {
    const auto [x, y] = position.observedBoard.pos(cell);
    mineBoard_->makeSafe(x, y, seed);
}

inline void GameControl::Game::reset(U128 seed) {
    const int rows = position.observedBoard.rows;
    const int cols = position.observedBoard.cols;
    const int mines = position.observedBoard.totalMines;
    mineBoard_->generate(rows, cols, mines, seed);
    structPool_->clear();
    distributionPool_->clear();
    position.observedBoard = ObservedBoard::Result(rows, cols, mines);
    position.initialize(*structPool_, *distributionPool_);
}

inline int GameControl::Game::number(int x, int y) const {
    return mineBoard_->number(x, y);
}

inline bool GameControl::Game::won() const {
    return position.remainingSafe_ == 0;
}

inline const Probability::Result &GameControl::Game::probability() {
    return position.probability(*structPool_, *distributionPool_);
}

inline long double GameControl::Game::mineProbability(ObservedBoard::CellId cell) {
    return position.mineProbability(cell, *structPool_, *distributionPool_);
}

inline const Structure::structPool &GameControl::Game::shapePool() const {
    return *structPool_;
}

inline std::vector<ObservedBoard::CellId> GameControl::Game::Suggest(Position::SuggestMode mode) {
    return position.Suggest(mode, *structPool_, *distributionPool_);
}

inline void GameControl::Game::playUntilNoSafe() {
    ObservedBoard::Delta updates;
    for (;;) {
        const std::vector<ObservedBoard::CellId> safeCells = position.safeMove(*structPool_, *distributionPool_);
        if (safeCells.empty())
            return;
        updates.changes.clear();
        updates.changes.reserve(safeCells.size());
        for (ObservedBoard::CellId cell : safeCells) {
            const auto [x, y] = position.observedBoard.pos(cell);
            updates.changes.push_back({cell, (ObservedBoard::CellState)(mineBoard_->number(x, y))});
        }
        update(updates);
    }
}

inline GameControl::Game::Game(mineBoard board, ObservedBoard::Result observedBoard, Workspace &workspace,
                               ShapeSolver::OrderAlgo orderAlgo)
    : mineBoard_(std::make_unique<mineBoard>(std::move(board))),
      structPool_(std::make_unique<Structure::structPool>()),
      distributionPool_(std::make_unique<ShapeSolver::Distribution::Pool>()),
      position(std::move(observedBoard), *structPool_, *distributionPool_, workspace, orderAlgo) {
    if (!mineBoard_->matches(position.observedBoard)) {
        position.error.failed = true;
        position.error.reason = "observedBoard does not match mineBoard";
    }
}

inline GameControl::Game::Game(int rows, int cols, int mines, U128 seed, Workspace &workspace,
                               const ObservedBoard::Result *observedBoard, ShapeSolver::OrderAlgo orderAlgo)
    : mineBoard_(std::make_unique<mineBoard>()),
      structPool_(std::make_unique<Structure::structPool>()),
      distributionPool_(std::make_unique<ShapeSolver::Distribution::Pool>()),
      position(observedBoard ? *observedBoard : ObservedBoard::Result(rows, cols, mines), *structPool_, *distributionPool_, workspace,
               orderAlgo) {
    mineBoard_->generate(rows, cols, mines, seed);
    if (observedBoard && !mineBoard_->matches(position.observedBoard)) {
        position.error.failed = true;
        position.error.reason = "observedBoard does not match mineBoard";
    }
}

inline void GameControl::mineBoard::generate(int rows, int cols, int mines, U128 seed) {
    board_.resize(rows, cols, 0);
    const int cellCount = board_.rows() * board_.cols();
    std::vector<int> cells(cellCount);
    for (int i = 0; i < cellCount; ++i)
        cells[i] = i;

    Random random(seed.lo, seed.hi);
    for (int i = cellCount - 1; i > 0; --i) {
        const int j = (int)(random.next() % (std::uint64_t)(i + 1));
        std::swap(cells[i], cells[j]);
    }
    for (int i = 0; i < mines; ++i) {
        const int cell = cells[i];
        board_[cell / board_.cols() + 1][cell % board_.cols() + 1] = 1;
    }
}

inline void GameControl::mineBoard::makeSafe(int x, int y, U128 seed) {
    if (!board_[x][y])
        return;
    int safeCount = 0;
    for (int nx = 1; nx <= board_.rows(); ++nx)
        for (int ny = 1; ny <= board_.cols(); ++ny)
            if (!board_[nx][ny])
                ++safeCount;
    if (!safeCount)
        assert_(false, "mineBoard::makeSafe: no safe cell");
    int target = (int)(Random(seed.lo, seed.hi).next() % (std::uint64_t)safeCount);
    for (int nx = 1; nx <= board_.rows(); ++nx)
        for (int ny = 1; ny <= board_.cols(); ++ny)
            if (!board_[nx][ny] && target-- == 0) {
                board_[x][y] = 0;
                board_[nx][ny] = 1;
                return;
            }
    assert_(false, "mineBoard::makeSafe: safe cell not found");
}

inline bool GameControl::mineBoard::matches(const ObservedBoard::Result &observedBoard) const {
    if (board_.rows() != observedBoard.rows || board_.cols() != observedBoard.cols)
        return false;
    int mines = 0;
    for (int x = 1; x <= observedBoard.rows; ++x)
        for (int y = 1; y <= observedBoard.cols; ++y) {
            const bool mine = board_[x][y] != 0;
            if (mine)
                ++mines;
            switch (observedBoard.board[x][y]) {
            case ObservedBoard::CellState::Hidden:
                break;
            case ObservedBoard::CellState::ForcedMine:
                if (!mine)
                    return false;
                break;
            case ObservedBoard::CellState::ForcedSafe:
                if (mine)
                    return false;
                break;
            default:
                if (mine || number(x, y) != (int)(observedBoard.board[x][y]))
                    return false;
                break;
            }
        }
    return mines == observedBoard.totalMines;
}

inline int GameControl::mineBoard::number(int x, int y) const {
    if (board_[x][y])
        return 9;
    int result = 0;
    forEachAdjacent(x, y, board_.rows(), board_.cols(), [&](int nx, int ny) { result += board_[nx][ny]; });
    return result;
}

} // namespace mss
