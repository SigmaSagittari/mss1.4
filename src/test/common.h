#pragma once

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <numeric>
#include <source_location>
#include <utility>
#include <vector>

#include "algo/basic.h"
#include "algo/probability/global_solver.h"
#include "algo/probability/probability.h"
#include "algo/shape_solver/shape_solver.h"
#include "algo/structure.h"
#include "core/types.h"

#ifdef _WIN32
#include <windows.h>
#include <dbghelp.h>
#endif

namespace test {

#ifdef _WIN32

struct StackTrace {
private:
    // 打印当前线程的 Windows 符号化调用栈。
    static void print() {
        void* frames[32];
        const USHORT count = CaptureStackBackTrace(1, 32, frames, nullptr);
        HANDLE process = GetCurrentProcess();
        SymSetOptions(SYMOPT_LOAD_LINES | SYMOPT_UNDNAME);
        if (!SymInitialize(process, nullptr, TRUE)) {
            for (USHORT i = 0; i < count; ++i)
                std::cerr << "  " << frames[i] << '\n';
            return;
        }
        alignas(SYMBOL_INFO)
            unsigned char storage[sizeof(SYMBOL_INFO) + MAX_SYM_NAME]{};
        PSYMBOL_INFO symbol = reinterpret_cast<PSYMBOL_INFO>(storage);
        symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
        symbol->MaxNameLen = MAX_SYM_NAME;
        for (USHORT i = 0; i < count; ++i) {
            DWORD64 displacement = 0;
            if (SymFromAddr(process, reinterpret_cast<DWORD64>(frames[i]),
                            &displacement, symbol))
                std::cerr << "  " << symbol->Name;
            else
                std::cerr << "  " << frames[i];
            IMAGEHLP_LINE64 line{};
            line.SizeOfStruct = sizeof(line);
            DWORD lineDisplacement = 0;
            if (SymGetLineFromAddr64(process,
                                     reinterpret_cast<DWORD64>(frames[i]),
                                     &lineDisplacement, &line))
                std::cerr << " (" << line.FileName << ':' << line.LineNumber
                          << ')';
            std::cerr << '\n';
        }
        SymCleanup(process);
    }

    friend void check(bool, const char*, std::source_location);
};

#endif

inline void check(bool condition, const char* errmsg,
                  std::source_location location =
                      std::source_location::current()) {
    // 检查测试条件；失败时输出位置和调用栈并终止测试进程。
    if (condition) return;
    std::cerr << "[FAIL] " << errmsg << "\n"
              << "  at " << location.file_name() << ':' << location.line()
              << '\n'
              << "  function: " << location.function_name() << '\n'
              << "  stack:\n";
#ifdef _WIN32
    StackTrace::print();
#endif
    std::abort();
}

// Game simulation.

struct GameRng {
    std::uint64_t state;

    // 用确定性种子创建测试用伪随机数发生器。
    explicit GameRng(std::uint64_t seed) : state(seed) {}

    // 生成下一个确定性的 64 位伪随机值。
    std::uint64_t next() {
        state ^= state >> 12;
        state ^= state << 25;
        state ^= state >> 27;
        return state * 0x2545f4914f6cdd1dULL;
    }

    // 生成 [0,n) 范围内的测试随机下标。
    int below(int n) {
        return next() % n;
    }
};

struct GameConfig {
    int rows;
    int cols;
    int mines;
};

struct Game {
    mss::ObservedBoard::Result board;
    std::vector<char> mines;
    int opened = 0;

    // 按测试配置创建空雷盘和全 Hidden 观测盘面。
    explicit Game(const GameConfig& config)
        : board(config.rows, config.cols, config.mines),
          mines(config.rows * config.cols, 0) {}

    // 将 1-based 棋盘坐标转换为雷数组下标。
    int flat(int x, int y) const { return (x - 1) * board.cols + y - 1; }
    // 查询指定测试格是否有雷。
    bool mine(int x, int y) const { return mines[flat(x, y)] != 0; }

    void placeMines(GameRng& rng, bool firstMoveSafe = false) {
        // 设计目的：测试夹具固定保留 flat index 0（即 (1,1)）作为稳定的起始安全格；
        // firstMoveSafe 只控制后续是否再次执行安全交换，不改变这个固定测试布局。
        std::vector<int> cells(board.rows * board.cols - 1);
        std::iota(cells.begin(), cells.end(), 1);
        for (int i = (int)(cells.size()) - 1; i > 0; --i)
            std::swap(cells[i], cells[rng.below(i + 1)]);
        for (int i = 0; i < board.totalMines; ++i) mines[cells[i]] = 1;
        if (firstMoveSafe && mine(1, 1))
            for (int x = 1; x <= board.rows; ++x)
                for (int y = 1; y <= board.cols; ++y)
                    if (!mine(x, y)) {
                        std::swap(mines[flat(1, 1)], mines[flat(x, y)]);
                        return;
                    }
    }

    int adjacentMines(int x, int y) const {
        // 统计指定格八邻域中的雷数。
        int result = 0;
        mss::forEachAdjacent(x, y, board.rows, board.cols,
                             [&](int nx, int ny) { result += mine(nx, ny); });
        return result;
    }

    bool reveal(int x, int y, mss::ObservedBoard::Delta& updates) {
        // 模拟安全点击和零区域泛洪，并把新数字写入观测 Delta。
        if (mine(x, y)) return false;
        std::vector<char> queued((board.rows + 1) * (board.cols + 1), 0);
        std::deque<std::pair<int, int>> pending{{x, y}};
        while (!pending.empty()) {
            const auto [cx, cy] = pending.front();
            pending.pop_front();
            const mss::CellId cell = board.id(cx, cy);
            if (queued[cell] || board.board[cx][cy] != mss::ObservedBoard::CellState::Hidden ||
                mine(cx, cy))
                continue;
            queued[cell] = 1;
            const int digit = adjacentMines(cx, cy);
            ++opened;
            updates.changes.push_back(
                {cell, (mss::ObservedBoard::CellState)(digit)});
            if (digit == 0)
                mss::forEachAdjacent(cx, cy, board.rows, board.cols,
                                     [&](int nx, int ny) { pending.emplace_back(nx, ny); });
        }
        return true;
    }

    // 判断测试盘面是否已打开全部非雷格。
    bool won() const { return opened == board.rows * board.cols - board.totalMines; }
};

struct Analysis {
    mss::Basic::Result basic;
    mss::Structure::ShapePool shapes;
    mss::Structure::Result structure;
    mss::ShapeSolver::Distribution::Pool distributions;
    mss::Probability::Result probability;
    mss::Basic::Delta basicDelta;
    mss::Structure::Delta structureDelta;

    // 从当前观测盘面建立生产分析管线的测试副本。
    explicit Analysis(const mss::ObservedBoard::Result& board)
        : basic(mss::Basic::analyze(board)),
          structure(mss::Structure::analyze(board, basic, shapes)) {
        probability = mss::Probability::analyze(board, basic, structure, shapes,
                                                distributions);
    }

    void update(mss::ObservedBoard::Result& board,
                mss::ObservedBoard::Delta& updates) {
        // 测试分析器按生产管线的固定顺序回放 Delta：board → basic → structure → probability。
        mss::ObservedBoard::update(board, updates);
        mss::Basic::update(basic, basicDelta, board, updates);
        mss::Structure::update(structure, structureDelta, board, basic, shapes,
                               updates);
        mss::Probability::analyze(board, basic, structure, shapes, distributions,
                                  probability);
    }
};

inline std::vector<mss::CellId> hiddenSafeCells(const Game& game,
                                                const Analysis& analysis) {
    // 收集当前已被 Basic 推断为安全但仍未翻开的格子。
    std::vector<mss::CellId> result;
    for (int x = 1; x <= game.board.rows; ++x)
        for (int y = 1; y <= game.board.cols; ++y)
            if (game.board.board[x][y] == mss::ObservedBoard::CellState::Hidden &&
                analysis.basic.marks[x][y] == mss::Basic::Mark::Safe)
                result.push_back(game.board.id(x, y));
    return result;
}

// Batch test configuration and execution.

enum class PositionFilter { All, GuessOnly };

struct TestConfig {
    int rows;
    int cols;
    int mines;
    double seconds;
    int games;
    PositionFilter filter;
    int maxRestarts;
    bool requireWinningGame;
    bool firstMoveSafe;
};

struct TimeBox {
    std::chrono::steady_clock::time_point start;
    std::chrono::steady_clock::time_point deadline;

    // 创建按时限或按无限时长运行的计时盒。
    explicit TimeBox(double seconds)
        : start(std::chrono::steady_clock::now()),
          deadline(seconds < 0
                        ? (std::chrono::steady_clock::time_point::max)()
                       : start + std::chrono::duration_cast<
                             std::chrono::steady_clock::duration>(
                             std::chrono::duration<double>(seconds))) {}

    // 判断计时盒是否已经达到截止时间。
    bool expired() const { return std::chrono::steady_clock::now() >= deadline; }
    // 返回从计时盒创建到当前时刻经过的秒数。
    double elapsedSeconds() const {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - start)
            .count();
    }
};

struct Move {
    int x = -1;
    int y = -1;
    long double mineProbability = 1.0L;
};

inline Move lowestRiskMove(const Game& game, const Analysis& analysis) {
    // 扫描所有可点候选并返回条件雷概率最低的格子。
    Move result;
    for (int x = 1; x <= game.board.rows; ++x)
        for (int y = 1; y <= game.board.cols; ++y) {
            if (game.board.board[x][y] != mss::ObservedBoard::CellState::Hidden)
                continue;
            if (analysis.basic.marks[x][y] == mss::Basic::Mark::F) continue;
            const long double risk = analysis.probability.mineProbability(
                game.board.id(x, y), game.board, analysis.basic,
                analysis.structure);
            if (risk >= result.mineProbability) continue;
            result = {x, y, risk};
        }
    if (result.x == -1) std::abort();
    return result;
}

using MovePolicy = Move (*)(const Game&, const Analysis&);

inline Move defaultMovePolicy(const Game&, const Analysis&) {
    // 返回空动作，交给调用方的默认最低风险策略接管。
    return Move{};
}

struct Snapshot {
    const Game& game;
    const Analysis& analysis;
    Move next;
    bool mustGuess = false;
};

template <typename Policy, typename Fn>
inline bool generateGame(const TestConfig& config, GameRng& rng,
                         Policy&& movePolicy, Fn&& consume) {
    // 生成并运行一局测试游戏，在指定快照时机调用消费回调。
    if (config.rows <= 0 || config.cols <= 0 || config.mines < 0 ||
        config.mines >= config.rows * config.cols)
        std::abort();
    for (int restart = 0; restart < config.maxRestarts; ++restart) {
        Game game({config.rows, config.cols, config.mines});
        game.placeMines(rng, config.firstMoveSafe);
        mss::ObservedBoard::Delta updates;
        if (config.firstMoveSafe) {
            game.reveal(1, 1, updates);
            mss::ObservedBoard::update(game.board, updates);
        }
        Analysis analysis(game.board);
        auto chooseMove = [&](const Game& current,
                              const Analysis& currentAnalysis) {
            Move next = movePolicy(current, currentAnalysis);
            if (next.x == -1 && next.y == -1)
                next = lowestRiskMove(current, currentAnalysis);
            return next;
        };
        Move next = chooseMove(game, analysis);
        bool lost = false;
        while (!game.won()) {
            updates.clear();
            if (!game.reveal(next.x, next.y, updates)) {
                lost = true;
                break;
            }
            if (game.won()) break;
            analysis.update(game.board, updates);
            next = chooseMove(game, analysis);
            const Snapshot snapshot{game, analysis, next,
                                    next.mineProbability > 1e-15L};
            if (config.filter == PositionFilter::All || snapshot.mustGuess)
                consume(snapshot);
        }
        const bool won = !lost && game.won();
        if (won || !config.requireWinningGame) return won;
    }
    std::abort();
}

struct RunSummary {
    long long games = 0;
    long long wins = 0;
    long long losses = 0;
    double elapsedSeconds = 0.0;
};

template <typename Policy, typename SnapshotFn, typename GameFn>
inline RunSummary runGamesWithGameEnd(const TestConfig& config, GameRng& rng,
                                      Policy&& movePolicy,
                                      SnapshotFn&& perSnapshot,
                                      GameFn&& perGame) {
    // 按时间或局数限制批量运行测试，并分别回调快照和对局结束事件。
    if (config.seconds < 0 && config.games < 0) std::abort();
    TimeBox timebox(config.seconds);
    RunSummary summary;
    while ((config.games < 0 || summary.games < config.games) &&
           !timebox.expired()) {
        const bool won = generateGame(config, rng, movePolicy,
                                      [&](const Snapshot& snapshot) {
                                          if (!timebox.expired())
                                              perSnapshot(snapshot);
                                      });
        ++summary.games;
        if (won) ++summary.wins;
        else ++summary.losses;
        perGame(won);
    }
    summary.elapsedSeconds = timebox.elapsedSeconds();
    return summary;
}

template <typename Policy, typename Fn>
inline RunSummary runGames(const TestConfig& config, GameRng& rng,
                           Policy&& movePolicy, Fn&& perSnapshot) {
    // 批量运行测试并只提供逐快照回调。
    return runGamesWithGameEnd(config, rng, std::forward<Policy>(movePolicy),
                               std::forward<Fn>(perSnapshot), [](bool) {});
}

template <typename Fn>
inline RunSummary runGames(const TestConfig& config, GameRng& rng,
                           Fn&& perSnapshot) {
    // 使用默认最低风险策略批量运行测试。
    return runGames(config, rng, &defaultMovePolicy,
                    std::forward<Fn>(perSnapshot));
}

}  // namespace test
