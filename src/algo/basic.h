#pragma once

#include <cstdint>
#include <vector>

#include "algo/observed_board.h"
#include "core/types.h"
#include "core/workspace.h"

namespace mss {

struct Basic {
    // H: 前沿，T: 非前沿，S: 安全，F: 危险。
    enum class Mark : std::uint8_t {
        H = 0,
        T = 1,
        S = 2,
        F = 3,

        Frontier = H,
        Unknown = T,
        Safe = S,
        Mine = F,
    };

    struct Result {
        int rows = 0;
        int cols = 0;
        Grid<Mark> marks;
        int unknownSum = 0;
        int mineSum = 0;
        int safeCount = 0;
        bool valid = true;

        Grid<std::int8_t> mineAround;
        Grid<std::int8_t> hideAround;
    };

    struct Delta {
        struct Change {
            CellId cell = -1;
            Mark old = Mark::T;
            Mark now = Mark::T;
        };

        std::vector<Change> changes;
        int unknownSum = 0;
        int mineSum = 0;
        int safeCount = 0;
        bool valid = true;
        int oldUnknownSum = 0;
        int oldMineSum = 0;
        int oldSafeCount = 0;
        bool oldValid = true;
    };

    // Basic 是局部约束传播结果：H/T 是仍可能为雷的候选，S/F 是已推出的安全/雷。
    // valid=false 表示观测值、强制标记或总雷数互相矛盾；不能继续喂给概率层。

  private:
    // 判断观测状态是否为 0..8 的已翻开数字。
    static bool isNumber(ObservedBoard::CellState state);
    // 将数字观测状态转换为对应的整数值。
    static int numberValue(ObservedBoard::CellState state);
    // 判断 Basic 标记是否仍属于可分配雷位的候选集合。
    static bool isCandidate(Mark mark);

  public:
    // 从完整观测盘面构建初始标记、邻域计数和合法性结果：先标出数字相邻的 H，
    // 再反复应用“剩余雷数为 0/候选数”的确定性约束，最后校验每条数字约束和总雷数。
    static Result analyze(const ObservedBoard::Result &state);
    // 将一批新观测增量传播到 Basic 结果中，并记录可逆 Delta；只把受影响数字放入
    // 队列，同时维护 mineAround/hideAround，避免每次点击都重扫整张盘面。
    static void update(Result &result, Delta &delta, const ObservedBoard::Result &board, const ObservedBoard::Delta &updates);
    // updates 必须已经由 ObservedBoard::update 应用到 board；Delta 的变更顺序也要保持。
    // 将 Basic Delta 正向或逆向回放到已有结果中；回放只恢复标记和计数，不重新推理。
    static void applyDelta(Result &result, const Delta &delta, bool reverse = true);
};

//==============================================================================
inline Basic::Result Basic::analyze(const ObservedBoard::Result &state) {
    // 这是全量基线构建；Analysis::update 只在增量路径无法复用时调用它。
    Result result;
    result.rows = state.rows;
    result.cols = state.cols;
    result.marks.resize(state.rows, state.cols, Mark::S);

    for (int i = 1; i <= state.rows; ++i)
        for (int j = 1; j <= state.cols; ++j) {
            switch (state.board[i][j]) {
            case ObservedBoard::CellState::Hidden:
                result.marks[i][j] = Mark::T;
                break;
            case ObservedBoard::CellState::ForcedMine:
                result.marks[i][j] = Mark::F;
                break;
            case ObservedBoard::CellState::ForcedSafe:
                result.marks[i][j] = Mark::S;
                break;
            default:
                break;
            }
        }

    for (int i = 1; i <= state.rows; ++i)
        for (int j = 1; j <= state.cols; ++j)
            if (isNumber(state.board[i][j]))
                forEachAdjacent(i, j, state.rows, state.cols, [&](int nx, int ny) {
                    if (state.board[nx][ny] == ObservedBoard::CellState::Hidden && result.marks[nx][ny] == Mark::T)
                        result.marks[nx][ny] = Mark::H;
                });

    std::vector<CellId> pending;
    for (int i = 1; i <= state.rows; ++i)
        for (int j = 1; j <= state.cols; ++j)
            if (isNumber(state.board[i][j]))
                pending.push_back(state.id(i, j));

    for (int head = 0; head < (int)(pending.size()); ++head) {
        const auto [x, y] = state.pos(pending[head]);
        int mineCount = 0;
        int candidateCount = 0;
        forEachAdjacent(x, y, state.rows, state.cols, [&](int nx, int ny) {
            if (result.marks[nx][ny] == Mark::F)
                ++mineCount;
            else if (state.board[nx][ny] == ObservedBoard::CellState::Hidden && result.marks[nx][ny] != Mark::S)
                ++candidateCount;
        });

        const int remaining = numberValue(state.board[x][y]) - mineCount;
        if (remaining < 0 || remaining > candidateCount) {
            result.valid = false;
            continue;
        }
        if (remaining != 0 && remaining != candidateCount)
            continue;

        const Mark mark = remaining == 0 ? Mark::S : Mark::F;
        forEachAdjacent(x, y, state.rows, state.cols, [&](int nx, int ny) {
            if (state.board[nx][ny] != ObservedBoard::CellState::Hidden || !isCandidate(result.marks[nx][ny]))
                return;
            result.marks[nx][ny] = mark;
            forEachAdjacent(nx, ny, state.rows, state.cols, [&](int ax, int ay) {
                if (isNumber(state.board[ax][ay]))
                    pending.push_back(state.id(ax, ay));
            });
        });
    }

    result.mineAround.resize(state.rows, state.cols, 0);
    result.hideAround.resize(state.rows, state.cols, 0);
    for (int i = 1; i <= state.rows; ++i)
        for (int j = 1; j <= state.cols; ++j)
            forEachAdjacent(i, j, state.rows, state.cols, [&](int nx, int ny) {
                if (result.marks[nx][ny] == Mark::F)
                    ++result.mineAround[i][j];
                else if (isCandidate(result.marks[nx][ny]))
                    ++result.hideAround[i][j];
            });

    for (int i = 1; i <= state.rows; ++i)
        for (int j = 1; j <= state.cols; ++j)
            if (isNumber(state.board[i][j])) {
                const int mineCount = result.mineAround[i][j];
                const int candidateCount = result.hideAround[i][j];
                const int value = numberValue(state.board[i][j]);
                if (value < mineCount || value > mineCount + candidateCount) {
                    result.valid = false;
                    break;
                }
            }

    int candidateSum = 0;
    for (int i = 1; i <= state.rows; ++i)
        for (int j = 1; j <= state.cols; ++j) {
            const Mark mark = result.marks[i][j];
            if (mark == Mark::F)
                ++result.mineSum;
            if (mark == Mark::T)
                ++result.unknownSum;
            if (mark == Mark::S)
                ++result.safeCount;
            if (isCandidate(mark))
                ++candidateSum;
        }
    if (result.mineSum > state.totalMines || result.mineSum + candidateSum < state.totalMines)
        result.valid = false;

    return result;
}

inline void Basic::update(Result &result, Delta &delta, const ObservedBoard::Result &board, const ObservedBoard::Delta &updates) {
    delta.changes.clear();
    delta.oldUnknownSum = result.unknownSum;
    delta.oldMineSum = result.mineSum;
    delta.oldSafeCount = result.safeCount;
    delta.oldValid = result.valid;

    const int rows = board.rows;
    const int cols = board.cols;
    using UpdateWorkspace = workspace::Basic::Update;
    UpdateWorkspace &updateWorkspace = workspace::Basic::update;
    std::vector<CellId> &pending = updateWorkspace.pending;
    std::vector<unsigned char> &queued = updateWorkspace.queued;
    pending.clear();
    const std::size_t queueSize = (rows + 1) * (cols + 1);
    if (queued.size() != queueSize)
        queued.resize(queueSize, 0);

    auto setMark = [&](int x, int y, Mark mark) {
        Mark &current = result.marks[x][y];
        if (current == mark)
            return;
        const Mark old = current;
        if (old == Mark::T)
            --result.unknownSum;
        if (old == Mark::F)
            --result.mineSum;
        if (old == Mark::S)
            --result.safeCount;
        if (mark == Mark::T)
            ++result.unknownSum;
        if (mark == Mark::F)
            ++result.mineSum;
        if (mark == Mark::S)
            ++result.safeCount;
        current = mark;
        delta.changes.push_back({board.id(x, y), old, mark});

        const bool oldMine = old == Mark::F;
        const bool newMine = mark == Mark::F;
        const bool oldCandidate = isCandidate(old);
        const bool newCandidate = isCandidate(mark);
        if (oldMine == newMine && oldCandidate == newCandidate)
            return;
        forEachAdjacent(x, y, rows, cols, [&](int nx, int ny) {
            if (oldMine)
                --result.mineAround[nx][ny];
            if (newMine)
                ++result.mineAround[nx][ny];
            if (oldCandidate)
                --result.hideAround[nx][ny];
            if (newCandidate)
                ++result.hideAround[nx][ny];
            const CellId neighbor = board.id(nx, ny);
            if (isNumber(board.board[nx][ny]) && !queued[neighbor]) {
                queued[neighbor] = 1;
                pending.push_back(neighbor);
            }
        });
    };

    bool forcedUpdate = false;
    for (const ObservedBoard::Change &change : updates.changes) {
        const auto [x, y] = board.pos(change.cell);
        if (change.next == ObservedBoard::CellState::ForcedMine || change.next == ObservedBoard::CellState::ForcedSafe) {
            forcedUpdate = true;
            const Mark forcedMark = change.next == ObservedBoard::CellState::ForcedMine ? Mark::F : Mark::S;
            const Mark current = result.marks[x][y];
            if ((forcedMark == Mark::F && current == Mark::S) || (forcedMark == Mark::S && current == Mark::F)) {
                result.valid = false;
                continue;
            }
            setMark(x, y, forcedMark);
            continue;
        }

        setMark(x, y, Mark::S);
        if (isNumber(change.next) && !queued[change.cell]) {
            queued[change.cell] = 1;
            pending.push_back(change.cell);
        }
        forEachAdjacent(x, y, rows, cols, [&](int nx, int ny) {
            if (board.board[nx][ny] == ObservedBoard::CellState::Hidden && result.marks[nx][ny] == Mark::T)
                setMark(nx, ny, Mark::H);
        });
    }

    for (int head = 0; head < (int)(pending.size()); ++head) {
        const auto [x, y] = board.pos(pending[head]);
        queued[pending[head]] = 0;
        const int remaining = numberValue(board.board[x][y]) - result.mineAround[x][y];
        if (remaining < 0 || remaining > result.hideAround[x][y]) {
            result.valid = false;
            continue;
        }
        if (remaining != 0 && remaining != result.hideAround[x][y])
            continue;
        const Mark mark = remaining == 0 ? Mark::S : Mark::F;
        forEachAdjacent(x, y, rows, cols, [&](int nx, int ny) {
            if (board.board[nx][ny] != ObservedBoard::CellState::Hidden || !isCandidate(result.marks[nx][ny]))
                return;
            setMark(nx, ny, mark);
        });
    }

    if (forcedUpdate) {
        const int candidateSum = rows * cols - result.safeCount - result.mineSum;
        if (result.mineSum > board.totalMines || result.mineSum + candidateSum < board.totalMines)
            result.valid = false;
    }
    delta.unknownSum = result.unknownSum;
    delta.mineSum = result.mineSum;
    delta.safeCount = result.safeCount;
    delta.valid = result.valid;
    return;
}

inline void Basic::applyDelta(Result &result, const Delta &delta, bool reverse) {
    // reverse 分支按逆序撤销标记变化，正向分支按记录顺序重做变化。
    auto account = [&](CellId cell, Mark old, Mark now) {
        if ((old == Mark::F) == (now == Mark::F) && isCandidate(old) == isCandidate(now))
            return;
        const int x = cell / (result.cols + 1);
        const int y = cell % (result.cols + 1);
        forEachAdjacent(x, y, result.rows, result.cols, [&](int nx, int ny) {
            if (old == Mark::F)
                --result.mineAround[nx][ny];
            if (now == Mark::F)
                ++result.mineAround[nx][ny];
            if (isCandidate(old))
                --result.hideAround[nx][ny];
            if (isCandidate(now))
                ++result.hideAround[nx][ny];
        });
    };
    if (reverse) {
        for (int i = delta.changes.size(); i-- > 0;) {
            const Delta::Change &change = delta.changes[i];
            const int x = change.cell / (result.cols + 1);
            const int y = change.cell % (result.cols + 1);
            result.marks[x][y] = change.old;
            account(change.cell, change.now, change.old);
        }
        result.unknownSum = delta.oldUnknownSum;
        result.mineSum = delta.oldMineSum;
        result.safeCount = delta.oldSafeCount;
        result.valid = delta.oldValid;
        return;
    }
    for (const Delta::Change &change : delta.changes) {
        const int x = change.cell / (result.cols + 1);
        const int y = change.cell % (result.cols + 1);
        result.marks[x][y] = change.now;
        account(change.cell, change.old, change.now);
    }
    result.unknownSum = delta.unknownSum;
    result.mineSum = delta.mineSum;
    result.safeCount = delta.safeCount;
    result.valid = delta.valid;
}

inline bool Basic::isNumber(ObservedBoard::CellState state) {
    return (int)(state) <= (int)(ObservedBoard::CellState::Num8);
}

inline int Basic::numberValue(ObservedBoard::CellState state) {
    return (int)(state);
}

inline bool Basic::isCandidate(Mark mark) {
    return mark == Mark::H || mark == Mark::T;
}

} // namespace mss
