#include "basic/basic.h"

#include "core/assert.h"

namespace mss {

// 已翻开的数字格：CellState::Num0..Num8 取值 0..8 且连续。
static bool isNumber(ObservedBoard::CellState state) {
    return static_cast<int>(state) <= static_cast<int>(ObservedBoard::CellState::Num8);
}

static int numberValue(ObservedBoard::CellState state) {
    return static_cast<int>(state);
}

static bool isCandidate(Basic::Mark mark) {
    return mark == Basic::Mark::H || mark == Basic::Mark::T;
}

// 增量修补邻域加速表（applyDelta / reverseDelta 用）：old -> now 对邻居计数的影响。
// 只操作两张表本身，不需要访问 Result 的私有成员。
static void accountNeighbors(Grid<std::int8_t> &mineAround, Grid<std::int8_t> &hideAround, int rows, int cols,
                             ObservedBoard::CellId cell, Basic::Mark old, Basic::Mark now) {
    if ((old == Basic::Mark::F) == (now == Basic::Mark::F) && isCandidate(old) == isCandidate(now))
        return;
    const int x = cell / cols;
    const int y = cell % cols;
    forEachAdjacent(x, y, rows, cols, [&](int nx, int ny) {
        if (old == Basic::Mark::F)
            --mineAround[nx][ny];
        if (now == Basic::Mark::F)
            ++mineAround[nx][ny];
        if (isCandidate(old))
            --hideAround[nx][ny];
        if (isCandidate(now))
            ++hideAround[nx][ny];
    });
}

Basic::Result Basic::analyze(const ObservedBoard::Result &board, Workspace &workspace) {
    Result result;
    result.rows = board.rows;
    result.cols = board.cols;
    result.marks.resize(board.rows, board.cols, Mark::S);

    // 初始标记：三个隐藏状态各有对应标记，已翻开的数字格保持 resize 的默认 S
    // （"非候选"填充值）。末支断言兜住"将来新增 CellState 却忘了处理"。
    for (int x = 0; x < board.rows; ++x)
        for (int y = 0; y < board.cols; ++y) {
            const ObservedBoard::CellState state = board.board[x][y];
            if (state == ObservedBoard::CellState::Hidden)
                result.marks[x][y] = Mark::T;
            else if (state == ObservedBoard::CellState::ForcedMine)
                result.marks[x][y] = Mark::F;
            else if (state == ObservedBoard::CellState::ForcedSafe)
                result.marks[x][y] = Mark::S;
            else
                assert_(isNumber(state), "Basic::analyze: 未处理的 CellState");
        }

    // 前沿：邻接数字的隐藏格从 T 变 H。
    for (int x = 0; x < board.rows; ++x)
        for (int y = 0; y < board.cols; ++y)
            if (isNumber(board.board[x][y]))
                forEachAdjacent(x, y, board.rows, board.cols, [&](int nx, int ny) {
                    if (board.board[nx][ny] == ObservedBoard::CellState::Hidden && result.marks[nx][ny] == Mark::T)
                        result.marks[nx][ny] = Mark::H;
                });

    // 传播到不动点。pending 复用 workspace，跨调用不再分配。
    std::vector<ObservedBoard::CellId> &pending = workspace.pending;
    pending.clear();
    for (int x = 0; x < board.rows; ++x)
        for (int y = 0; y < board.cols; ++y)
            if (isNumber(board.board[x][y]))
                pending.push_back(board.id(x, y));

    for (std::size_t head = 0; head < pending.size(); ++head) {
        const auto [x, y] = board.pos(pending[head]);
        int mineCount = 0;
        int candidateCount = 0;
        forEachAdjacent(x, y, board.rows, board.cols, [&](int nx, int ny) {
            if (result.marks[nx][ny] == Mark::F)
                ++mineCount;
            else if (isCandidate(result.marks[nx][ny]))
                ++candidateCount;
        });
        const int remaining = numberValue(board.board[x][y]) - mineCount;
        if (remaining < 0 || remaining > candidateCount) {
            result.valid = false; // ① 传播中越界：立即中断，Result 作废
            return result;
        }
        if (remaining != 0 && remaining != candidateCount)
            continue;
        const Mark mark = remaining == 0 ? Mark::S : Mark::F;
        forEachAdjacent(x, y, board.rows, board.cols, [&](int nx, int ny) {
            if (!isCandidate(result.marks[nx][ny]))
                return;
            result.marks[nx][ny] = mark;
            forEachAdjacent(nx, ny, board.rows, board.cols, [&](int ax, int ay) {
                if (isNumber(board.board[ax][ay]))
                    pending.push_back(board.id(ax, ay));
            });
        });
    }

    // 加速表：全量重建。
    result.mineAround.resize(board.rows, board.cols, 0);
    result.hideAround.resize(board.rows, board.cols, 0);
    for (int x = 0; x < board.rows; ++x)
        for (int y = 0; y < board.cols; ++y)
            forEachAdjacent(x, y, board.rows, board.cols, [&](int nx, int ny) {
                if (result.marks[nx][ny] == Mark::F)
                    ++result.mineAround[x][y];
                else if (isCandidate(result.marks[nx][ny]))
                    ++result.hideAround[x][y];
            });

    // ② 数字约束终检。
    for (int x = 0; x < board.rows; ++x)
        for (int y = 0; y < board.cols; ++y)
            if (isNumber(board.board[x][y])) {
                const int value = numberValue(board.board[x][y]);
                if (value < result.mineAround[x][y] || value > result.mineAround[x][y] + result.hideAround[x][y]) {
                    result.valid = false;
                    return result;
                }
            }

    // 计数 + ③ 总雷数终检。
    int candidateSum = 0;
    for (int x = 0; x < board.rows; ++x)
        for (int y = 0; y < board.cols; ++y) {
            const Mark mark = result.marks[x][y];
            if (mark == Mark::F)
                ++result.mineSum;
            else if (mark == Mark::T)
                ++result.unknownSum;
            else if (mark == Mark::S && !isNumber(board.board[x][y]))
                ++result.safeHideCount;
            if (isCandidate(mark))
                ++candidateSum;
        }
    if (result.mineSum > board.totalMines || result.mineSum + candidateSum < board.totalMines)
        result.valid = false;
    return result;
}

void Basic::update(Result &result, Delta &delta, const ObservedBoard::Result &board,
                   const ObservedBoard::Delta &updates, Workspace &workspace) {
    delta.changes.clear();
    delta.before = {result.unknownSum, result.mineSum, result.safeHideCount, result.valid};

    const int rows = board.rows;
    const int cols = board.cols;
    std::vector<ObservedBoard::CellId> &pending = workspace.pending;
    std::vector<unsigned char> &queued = workspace.queued;
    pending.clear();
    const std::size_t cellCount = static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
    if (queued.size() != cellCount)
        queued.assign(cellCount, 0);

    // 唯一修改标记的地方：计数、Delta、加速表在这里同步维护。
    auto setMark = [&](int x, int y, Mark mark) {
        Mark &current = result.marks[x][y];
        if (current == mark)
            return;
        const Mark old = current;
        const bool revealed = isNumber(board.board[x][y]);
        if (old == Mark::T)
            --result.unknownSum;
        if (old == Mark::F)
            --result.mineSum;
        if (old == Mark::S && !revealed)
            --result.safeHideCount;
        if (mark == Mark::T)
            ++result.unknownSum;
        if (mark == Mark::F)
            ++result.mineSum;
        if (mark == Mark::S && !revealed)
            ++result.safeHideCount;
        current = mark;
        delta.changes.push_back({board.id(x, y), old, mark});

        const bool oldMine = old == Mark::F;
        const bool newMine = mark == Mark::F;
        const bool oldCandidate = isCandidate(old);
        const bool newCandidate = isCandidate(mark);
        if (oldMine == newMine && oldCandidate == newCandidate)
            return; // 对邻域计数无影响，也没有数字格需要重算
        forEachAdjacent(x, y, rows, cols, [&](int nx, int ny) {
            if (oldMine)
                --result.mineAround[nx][ny];
            if (newMine)
                ++result.mineAround[nx][ny];
            if (oldCandidate)
                --result.hideAround[nx][ny];
            if (newCandidate)
                ++result.hideAround[nx][ny];
            const ObservedBoard::CellId neighbor = board.id(nx, ny);
            if (isNumber(board.board[nx][ny]) && !queued[neighbor]) {
                queued[neighbor] = 1;
                pending.push_back(neighbor);
            }
        });
    };

    auto failInvalid = [&]() {
        result.valid = false;
        delta.after = {result.unknownSum, result.mineSum, result.safeHideCount, false};
    };

    bool forcedUpdate = false;
    for (const ObservedBoard::Change &change : updates.changes) {
        const auto [x, y] = board.pos(change.cell);
        if (change.next == ObservedBoard::CellState::ForcedMine || change.next == ObservedBoard::CellState::ForcedSafe) {
            forcedUpdate = true;
            const Mark forcedMark = change.next == ObservedBoard::CellState::ForcedMine ? Mark::F : Mark::S;
            const Mark current = result.marks[x][y];
            if ((forcedMark == Mark::F && current == Mark::S) || (forcedMark == Mark::S && current == Mark::F)) {
                failInvalid(); // ④ 强制标记与已知 S/F 冲突
                return;
            }
            setMark(x, y, forcedMark);
            continue;
        }

        // 数字：这一格被翻开了。它若原来是"已推出来的安全格"，就不再属于 safeHideCount。
        if (result.marks[x][y] == Mark::S)
            --result.safeHideCount;
        setMark(x, y, Mark::S);
        if (!queued[change.cell]) {
            queued[change.cell] = 1;
            pending.push_back(change.cell);
        }
        forEachAdjacent(x, y, rows, cols, [&](int nx, int ny) {
            if (board.board[nx][ny] == ObservedBoard::CellState::Hidden && result.marks[nx][ny] == Mark::T)
                setMark(nx, ny, Mark::H);
        });
    }

    // 传播到不动点：读加速表，O(1) 拿到邻域计数。
    for (std::size_t head = 0; head < pending.size(); ++head) {
        const ObservedBoard::CellId cell = pending[head];
        queued[cell] = 0;
        const auto [x, y] = board.pos(cell);
        const int remaining = numberValue(board.board[x][y]) - result.mineAround[x][y];
        if (remaining < 0 || remaining > result.hideAround[x][y]) {
            failInvalid(); // ① 传播中越界
            return;
        }
        if (remaining != 0 && remaining != result.hideAround[x][y])
            continue;
        const Mark mark = remaining == 0 ? Mark::S : Mark::F;
        forEachAdjacent(x, y, rows, cols, [&](int nx, int ny) {
            if (!isCandidate(result.marks[nx][ny]))
                return;
            setMark(nx, ny, mark);
        });
    }

    if (forcedUpdate) {
        // ③ 总雷数终检（只在强制更新时做；候选数直接数，冷路径）
        int candidateSum = 0;
        for (int x = 0; x < rows; ++x)
            for (int y = 0; y < cols; ++y)
                if (isCandidate(result.marks[x][y]))
                    ++candidateSum;
        if (result.mineSum > board.totalMines || result.mineSum + candidateSum < board.totalMines)
            result.valid = false;
    }

    delta.after = {result.unknownSum, result.mineSum, result.safeHideCount, result.valid};
}

bool Basic::Result::sameAs(const Result &other) const {
    if (rows != other.rows || cols != other.cols || unknownSum != other.unknownSum || mineSum != other.mineSum ||
        safeHideCount != other.safeHideCount || valid != other.valid)
        return false;
    for (int x = 0; x < rows; ++x)
        for (int y = 0; y < cols; ++y)
            if (marks[x][y] != other.marks[x][y] || mineAround[x][y] != other.mineAround[x][y] ||
                hideAround[x][y] != other.hideAround[x][y])
                return false;
    return true;
}

void Basic::applyDelta(Result &result, const Delta &delta) {
    for (const Delta::Change &change : delta.changes) {
        result.marks.data()[change.cell] = change.now;
        accountNeighbors(result.mineAround, result.hideAround, result.rows, result.cols, change.cell, change.old,change.now);
    }
    result.unknownSum = delta.after.unknownSum;
    result.mineSum = delta.after.mineSum;
    result.safeHideCount = delta.after.safeHideCount;
    result.valid = delta.after.valid;
}

void Basic::reverseDelta(Result &result, const Delta &delta) {
    for (int i = static_cast<int>(delta.changes.size()); i-- > 0;) {
        const Delta::Change &change = delta.changes[i];
        result.marks.data()[change.cell] = change.old;
        accountNeighbors(result.mineAround, result.hideAround, result.rows, result.cols, change.cell, change.now,change.old);
    }
    result.unknownSum = delta.before.unknownSum;
    result.mineSum = delta.before.mineSum;
    result.safeHideCount = delta.before.safeHideCount;
    result.valid = delta.before.valid;
}

} // namespace mss