#pragma once

#include <chrono>
#include <iostream>
#include <memory>
#include <source_location>
#include <utility>
#include <vector>

#include "algo/observed_board.h"
#include "core/utility/hash.h"
#include "core/utility/rng.h"
#include "game/game.h"

#ifdef _WIN32
#include <dbghelp.h>
#include <windows.h>
#endif

namespace test {

#ifdef _WIN32

struct StackTrace {
  private:
    // 打印当前线程的 Windows 符号化调用栈。
    static void print() {
        void *frames[32];
        const USHORT count = CaptureStackBackTrace(1, 32, frames, nullptr);
        HANDLE process = GetCurrentProcess();
        SymSetOptions(SYMOPT_LOAD_LINES | SYMOPT_UNDNAME);
        if (!SymInitialize(process, nullptr, TRUE)) {
            for (USHORT i = 0; i < count; ++i)
                std::cerr << "  " << frames[i] << '\n';
        } else {
            alignas(SYMBOL_INFO) unsigned char storage[sizeof(SYMBOL_INFO) + MAX_SYM_NAME]{};
            PSYMBOL_INFO symbol = reinterpret_cast<PSYMBOL_INFO>(storage);
            symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
            symbol->MaxNameLen = MAX_SYM_NAME;
            for (USHORT i = 0; i < count; ++i) {
                DWORD64 displacement = 0;
                if (SymFromAddr(process, reinterpret_cast<DWORD64>(frames[i]), &displacement, symbol))
                    std::cerr << "  " << symbol->Name;
                else
                    std::cerr << "  " << frames[i];
                IMAGEHLP_LINE64 line{};
                line.SizeOfStruct = sizeof(line);
                DWORD lineDisplacement = 0;
                if (SymGetLineFromAddr64(process, reinterpret_cast<DWORD64>(frames[i]), &lineDisplacement, &line))
                    std::cerr << " (" << line.FileName << ':' << line.LineNumber << ')';
                std::cerr << '\n';
            }
            SymCleanup(process);
        }
    }

    friend void check(bool, const char *, std::source_location);
};

#endif

inline void check(bool condition, const char *errmsg, std::source_location location = std::source_location::current()) {
    // 检查测试条件；失败时输出位置和调用栈并终止测试进程。
    if (!condition) {
        std::cerr << "[FAIL] " << errmsg << "\n"
                  << "  at " << location.file_name() << ':' << location.line() << '\n'
                  << "  function: " << location.function_name() << '\n'
                  << "  stack:\n";
#ifdef _WIN32
        StackTrace::print();
#endif
        mss::assert_(false, errmsg, location);
    }
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
                       : start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(seconds))) {
    }

    // 判断计时盒是否已经达到截止时间。
    bool expired() const {
        return std::chrono::steady_clock::now() >= deadline;
    }
    // 返回从计时盒创建到当前时刻经过的秒数。
    double elapsedSeconds() const {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    }
};

struct Snapshot {
    const mss::GameControl::Game &game;
    const std::vector<mss::CellId> &next;
    bool mustGuess;
};

template <typename Fn>
inline bool generateGame(const TestConfig &config, mss::GameControl::Game &game, mss::Random &rng,
                         mss::GameControl::Position::SuggestMode mode, Fn &&consume) {
    // 生成并运行一局测试游戏，在指定快照时机调用消费回调。
    mss::assert_(config.rows > 0 && config.cols > 0 && config.mines >= 0 && config.mines < config.rows * config.cols,
                 "test::generateGame: invalid board configuration");
    mss::ObservedBoard::Delta updates;
    std::vector<mss::CellId> next = game.Suggest(mode);
    mss::assert_(!next.empty(), "test::generateGame: initial Suggest returned no move");
    if (config.firstMoveSafe)
        game.makeFirstMoveSafe(next.front(), mss::U128{rng.next(), rng.next()});
    bool lost = false;
    while (!game.won()) {
        for (const mss::CellId cell : next) {
            const auto [x, y] = game.position.observedBoard.pos(cell);
            updates.clear();
            const int number = game.number(x, y);
            if (number == 9) {
                lost = true;
                break;
            }
            updates.changes.push_back({cell, (mss::ObservedBoard::CellState)(number)});
            game.update(updates);
            if (game.won())
                break;
        }
        if (lost || game.won())
            break;
        next = game.Suggest(mode);
        mss::assert_(!next.empty(), "test::generateGame: Suggest returned no move");
        const Snapshot snapshot{game, next, next.size() == 1};
        if (config.filter == PositionFilter::All || snapshot.mustGuess)
            consume(snapshot);
    }
    return !lost && game.won();
}

struct RunSummary {
    long long games = 0;
    long long wins = 0;
    long long losses = 0;
    double elapsedSeconds = 0.0;
};

template <typename Fn>
inline RunSummary runGames(const TestConfig &config, mss::Random &rng, mss::GameControl::Position::SuggestMode mode, Fn &&perSnapshot) {
    mss::assert_(config.seconds >= 0 || config.games >= 0, "test::runGames: no stopping condition");
    TimeBox timebox(config.seconds);
    RunSummary summary;
    std::unique_ptr<mss::GameControl::Game> game;
    while ((config.games < 0 || summary.games < config.games) && !timebox.expired()) {
        if (game) {
            game->reset(mss::U128{rng.next(), rng.next()});
        } else {
            mss::GameControl::mineBoard mineBoard;
            mineBoard.generate(config.rows, config.cols, config.mines, mss::U128{rng.next(), rng.next()});
            game = std::make_unique<mss::GameControl::Game>(std::move(mineBoard),
                                                            mss::ObservedBoard::Result(config.rows, config.cols, config.mines));
        }
        const bool won = generateGame(config, *game, rng, mode, [&](const Snapshot &snapshot) {
            if (!timebox.expired())
                perSnapshot(snapshot);
        });
        ++summary.games;
        if (won)
            ++summary.wins;
        else
            ++summary.losses;
    }
    summary.elapsedSeconds = timebox.elapsedSeconds();
    return summary;
}

} // namespace test
