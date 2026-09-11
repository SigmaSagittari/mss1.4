#include "algo/basic.h"

namespace mss {

namespace {

using Mark = Basic::Mark;
using State = ObservedBoard::CellState;

bool isNumber(State state) {
    return static_cast<int>(state) <= static_cast<int>(State::Num8);
}

int numberValue(State state) {
    return static_cast<int>(state);
}

bool isCandidate(Mark mark) {
    return mark == Mark::H || mark == Mark::T;
}

}  // namespace

Basic::Result Basic::analyze(const ObservedBoard::Result& state) {
    Result result;
    result.rows = state.rows;
    result.cols = state.cols;
    result.marks.resize(state.rows, state.cols, Mark::S);

    for (int i = 1; i <= state.rows; ++i)
        for (int j = 1; j <= state.cols; ++j) {
            switch (state.board[i][j]) {
            case State::Hidden:
                result.marks[i][j] = Mark::T;
                break;
            case State::ForcedMine:
                result.marks[i][j] = Mark::F;
                break;
            case State::ForcedSafe:
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
                    if (state.board[nx][ny] == State::Hidden &&
                        result.marks[nx][ny] == Mark::T)
                        result.marks[nx][ny] = Mark::H;
                });

    std::vector<CellId> pending;
    for (int i = 1; i <= state.rows; ++i)
        for (int j = 1; j <= state.cols; ++j)
            if (isNumber(state.board[i][j])) pending.push_back(state.id(i, j));

    for (std::size_t head = 0; head < pending.size(); ++head) {
        const auto [x, y] = state.pos(pending[head]);
        int mineCount = 0;
        int candidateCount = 0;
        forEachAdjacent(x, y, state.rows, state.cols, [&](int nx, int ny) {
            if (result.marks[nx][ny] == Mark::F)
                ++mineCount;
            else if (state.board[nx][ny] == State::Hidden &&
                     result.marks[nx][ny] != Mark::S)
                ++candidateCount;
        });

        const int remaining = numberValue(state.board[x][y]) - mineCount;
        if (remaining < 0 || remaining > candidateCount) {
            result.valid = false;
            continue;
        }
        if (remaining != 0 && remaining != candidateCount) continue;

        const Mark mark = remaining == 0 ? Mark::S : Mark::F;
        forEachAdjacent(x, y, state.rows, state.cols, [&](int nx, int ny) {
            if (state.board[nx][ny] != State::Hidden ||
                !isCandidate(result.marks[nx][ny]))
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
            if (mark == Mark::F) ++result.mineSum;
            if (mark == Mark::T) ++result.unknownSum;
            if (mark == Mark::S) ++result.safeCount;
            if (isCandidate(mark)) ++candidateSum;
        }
    if (result.mineSum > state.totalMines ||
        result.mineSum + candidateSum < state.totalMines)
        result.valid = false;

    return result;
}

Basic::Delta Basic::update(const ObservedBoard::Result& board, Result& result,
                           const ObservedBoard::Delta& updates, Delta delta) {
    delta.changes.clear();
    delta.oldUnknownSum = result.unknownSum;
    delta.oldMineSum = result.mineSum;
    delta.oldSafeCount = result.safeCount;
    delta.oldValid = result.valid;

    const int rows = board.rows;
    const int cols = board.cols;

    static thread_local std::vector<CellId> pending;
    static thread_local std::vector<unsigned char> queued;
    pending.clear();
    const auto queueSize = static_cast<std::vector<unsigned char>::size_type>(
        (rows + 1) * (cols + 1));
    if (queued.size() != queueSize)
        queued.resize(queueSize, 0);

    auto setMark = [&](int x, int y, Mark mark) {
        Mark& current = result.marks[x][y];
        if (current == mark) return;
        const Mark old = current;
        if (old == Mark::T) --result.unknownSum;
        if (old == Mark::F) --result.mineSum;
        if (old == Mark::S) --result.safeCount;
        if (mark == Mark::T) ++result.unknownSum;
        if (mark == Mark::F) ++result.mineSum;
        if (mark == Mark::S) ++result.safeCount;
        current = mark;
        delta.changes.push_back({board.id(x, y), old, mark});

        const bool oldMine = old == Mark::F;
        const bool newMine = mark == Mark::F;
        const bool oldCandidate = isCandidate(old);
        const bool newCandidate = isCandidate(mark);
        if (oldMine == newMine && oldCandidate == newCandidate) return;

        forEachAdjacent(x, y, rows, cols, [&](int nx, int ny) {
            if (oldMine) --result.mineAround[nx][ny];
            if (newMine) ++result.mineAround[nx][ny];
            if (oldCandidate) --result.hideAround[nx][ny];
            if (newCandidate) ++result.hideAround[nx][ny];
            const CellId neighbor = board.id(nx, ny);
            if (isNumber(board.board[nx][ny]) && !queued[neighbor]) {
                queued[neighbor] = 1;
                pending.push_back(board.id(nx, ny));
            }
        });
    };

    bool forcedUpdate = false;
    for (const ObservedBoard::Change& change : updates.changes) {
        const auto [x, y] = board.pos(change.cell);

        if (change.next == State::ForcedMine || change.next == State::ForcedSafe) {
            forcedUpdate = true;
            const Mark forcedMark = change.next == State::ForcedMine ? Mark::F : Mark::S;
            const Mark current = result.marks[x][y];
            if ((forcedMark == Mark::F && current == Mark::S) ||
                (forcedMark == Mark::S && current == Mark::F)) {
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
            if (board.board[nx][ny] == State::Hidden &&
                result.marks[nx][ny] == Mark::T)
                setMark(nx, ny, Mark::H);
        });
    }

    for (std::size_t head = 0; head < pending.size(); ++head) {
        const auto [x, y] = board.pos(pending[head]);
        queued[pending[head]] = 0;
        const int remaining = numberValue(board.board[x][y]) - result.mineAround[x][y];
        if (remaining < 0 || remaining > result.hideAround[x][y]) {
            result.valid = false;
            continue;
        }
        if (remaining != 0 && remaining != result.hideAround[x][y]) continue;

        const Mark mark = remaining == 0 ? Mark::S : Mark::F;
        forEachAdjacent(x, y, rows, cols, [&](int nx, int ny) {
            if (board.board[nx][ny] != State::Hidden ||
                !isCandidate(result.marks[nx][ny]))
                return;
            setMark(nx, ny, mark);
        });
    }

    if (forcedUpdate) {
        const int candidateSum = rows * cols - result.safeCount - result.mineSum;
        if (result.mineSum > board.totalMines ||
            result.mineSum + candidateSum < board.totalMines)
            result.valid = false;
    }

    delta.unknownSum = result.unknownSum;
    delta.mineSum = result.mineSum;
    delta.safeCount = result.safeCount;
    delta.valid = result.valid;
    return delta;
}

void Basic::applyDelta(Result& result, const Delta& delta, bool reverse) {
    auto account = [&](CellId cell, Mark old, Mark now) {
        if ((old == Mark::F) == (now == Mark::F) &&
            isCandidate(old) == isCandidate(now))
            return;
        const int x = cell / (result.cols + 1);
        const int y = cell % (result.cols + 1);
        forEachAdjacent(x, y, result.rows, result.cols, [&](int nx, int ny) {
            if (old == Mark::F) --result.mineAround[nx][ny];
            if (now == Mark::F) ++result.mineAround[nx][ny];
            if (isCandidate(old)) --result.hideAround[nx][ny];
            if (isCandidate(now)) ++result.hideAround[nx][ny];
        });
    };

    if (reverse) {
        for (std::size_t i = delta.changes.size(); i-- > 0;) {
            const Delta::Change& change = delta.changes[i];
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

    for (const Delta::Change& change : delta.changes) {
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

}  // namespace mss
