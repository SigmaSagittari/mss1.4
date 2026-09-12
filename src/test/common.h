#pragma once

#include <algorithm>
#include <deque>
#include <cstdlib>
#include <iostream>
#include <source_location>
#include <numeric>
#include <vector>

#include "algo/basic.h"
#include "algo/probability/probability.h"
#include "algo/shape_solver/shape_solver.h"
#include "algo/structure.h"
#include "core/types.h"

#ifdef _WIN32
#include <windows.h>
#include <dbghelp.h>

namespace test {

struct StackTrace {
private:
    static void print() {
    void* frames[32];
    const USHORT count = CaptureStackBackTrace(1, 32, frames, nullptr);
    HANDLE process = GetCurrentProcess();
    SymSetOptions(SYMOPT_LOAD_LINES | SYMOPT_UNDNAME);
    if (!SymInitialize(process, nullptr, TRUE)) {
        for (USHORT i = 0; i < count; ++i) std::cerr << "  " << frames[i] << '\n';
        return;
    }
    alignas(SYMBOL_INFO) unsigned char storage[sizeof(SYMBOL_INFO) + MAX_SYM_NAME]{};
    auto* symbol = reinterpret_cast<PSYMBOL_INFO>(storage);
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
        if (SymGetLineFromAddr64(process, reinterpret_cast<DWORD64>(frames[i]),
                                 &lineDisplacement, &line))
            std::cerr << " (" << line.FileName << ':' << line.LineNumber << ')';
        std::cerr << '\n';
    }
    SymCleanup(process);
    }

    friend void check(bool, const char*, std::source_location);
};

}  // namespace test

#endif

namespace test {

struct GameRng {
    std::uint64_t state;

    explicit GameRng(std::uint64_t seed) : state(seed) {}

    std::uint64_t next() {
        state ^= state >> 12;
        state ^= state << 25;
        state ^= state >> 27;
        return state * 0x2545f4914f6cdd1dULL;
    }

    int below(int n) {
        return static_cast<int>(next() % static_cast<std::uint64_t>(n));
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

    explicit Game(const GameConfig& config)
        : board(config.rows, config.cols, config.mines),
          mines(config.rows * config.cols, 0) {}

    int flat(int x, int y) const { return (x - 1) * board.cols + y - 1; }
    bool mine(int x, int y) const { return mines[flat(x, y)] != 0; }

    void placeMines(GameRng& rng) {
        std::vector<int> cells(board.rows * board.cols - 1);
        std::iota(cells.begin(), cells.end(), 1);
        for (int i = static_cast<int>(cells.size()) - 1; i > 0; --i)
            std::swap(cells[i], cells[rng.below(i + 1)]);
        for (int i = 0; i < board.totalMines; ++i) mines[cells[i]] = 1;
    }

    int adjacentMines(int x, int y) const {
        int result = 0;
        mss::forEachAdjacent(x, y, board.rows, board.cols,
                             [&](int nx, int ny) { result += mine(nx, ny); });
        return result;
    }

    bool reveal(int x, int y, mss::ObservedBoard::Delta& updates) {
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
                {cell, static_cast<mss::ObservedBoard::CellState>(digit)});
            if (digit == 0)
                mss::forEachAdjacent(cx, cy, board.rows, board.cols,
                                     [&](int nx, int ny) { pending.emplace_back(nx, ny); });
        }
        return true;
    }

    bool won() const { return opened == board.rows * board.cols - board.totalMines; }
};

struct Analysis {
    mss::Basic::Result basic;
    mss::Structure::ShapePool shapes;
    mss::Structure::Result structure;
    mss::ShapeSolver::Distribution::Pool distributions;
    mss::Probability::Result probability;

    explicit Analysis(const mss::ObservedBoard::Result& board)
        : basic(mss::Basic::analyze(board)),
          structure(mss::Structure::analyze(board, basic, shapes)) {
        probability = mss::Probability::analyze(board, basic, structure, shapes,
                                                distributions);
    }
};

inline std::vector<mss::CellId> hiddenSafeCells(const Game& game,
                                                const Analysis& analysis) {
    std::vector<mss::CellId> result;
    for (int x = 1; x <= game.board.rows; ++x)
        for (int y = 1; y <= game.board.cols; ++y)
            if (game.board.board[x][y] == mss::ObservedBoard::CellState::Hidden &&
                analysis.basic.marks[x][y] == mss::Basic::Mark::Safe)
                result.push_back(game.board.id(x, y));
    return result;
}

inline mss::CellId lowestRiskCell(const Game& game, const Analysis& analysis,
                                  std::span<const mss::CellId> excluded = {}) {
    mss::CellId result = -1;
    long double risk = 1.0L;
    for (int x = 1; x <= game.board.rows; ++x)
        for (int y = 1; y <= game.board.cols; ++y) {
            if (game.board.board[x][y] != mss::ObservedBoard::CellState::Hidden) continue;
            const auto mark = analysis.basic.marks[x][y];
            if (mark != mss::Basic::Mark::Frontier && mark != mss::Basic::Mark::Unknown)
                continue;
            const mss::CellId cell = game.board.id(x, y);
            if (std::find(excluded.begin(), excluded.end(), cell) != excluded.end()) continue;
            const long double current = analysis.probability.mineProbability(
                cell, game.board, analysis.basic, analysis.structure);
            if (current < risk) {
                risk = current;
                result = cell;
            }
        }
    return result;
}

}  // namespace test

namespace test {

inline void check(bool condition, const char* errmsg,
                  std::source_location location = std::source_location::current()) {
    if (condition) return;
    std::cerr << "[FAIL] " << errmsg << "\n"
              << "  at " << location.file_name() << ':' << location.line() << '\n'
              << "  function: " << location.function_name() << '\n'
              << "  stack:\n";
#ifdef _WIN32
    StackTrace::print();
#endif
    std::abort();
}

}  // namespace test
