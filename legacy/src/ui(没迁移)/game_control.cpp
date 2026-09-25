#include "ui/game_control.h"

namespace mss {

bool GameController::Analysis::initFromState(){
            basic_ = Basic::analyze(state_);
            structure_ = Structure::analyze(state_, basic_, shapes_);
            if (!basic_.valid) return false;
            rebuild();
            return true;
        }

void GameController::Analysis::update(const ObservedBoard::Delta& updates){
            const ObservedBoard::Delta boardDelta = ObservedBoard::update(state_, updates);
            Basic::update(state_, basic_, boardDelta);
            Structure::update(state_, basic_, structure_, shapes_, boardDelta);
            rebuild();
        }

void GameController::Analysis::setDistMode(DistMode mode){
            distMode_ = mode;
            dists_.clear();
            rebuild();
        }

void GameController::Analysis::rebuild(){
            fillDistributions();
            prob_ = Exact::analyze(state_, basic_, structure_, dists_);
        }

void GameController::Analysis::fillDistributions(){
            for (const Structure::Instance& inst : structure_.components) {
                switch (distMode_) {
                    case DistMode::Old:
                        Distribution::dfs_solver::analyze(*inst.shape, dists_);
                        break;
                    case DistMode::Graph:
                        Distribution::graph_solver::analyze(*inst.shape, dists_);
                        break;
                    default:
                        Distribution::graph_solver::analyze_auto(*inst.shape, dists_);
                        break;
                }
            }
        }

GameController::game_info GameController::info() const{
        return {layout_, revealed_, flags_, rows_, cols_, mines_,
                moves_, flagsRemaining_, status_, seed_};
    }

bool GameController::reveal(int x, int y){
        if (status_ != Status::Playing) return false;
        if (x < 1 || x > rows_ || y < 1 || y > cols_) return false;
        if (revealed_[x][y]) { chord(x, y); return status_ != Status::Lost; }
        if (flags_[x][y]) return false;

        ++moves_; ++revision_;
        if (firstMove_) {
            firstMove_ = false;
            if (layout_[x][y]) relocateMine(x, y);
        }

        ObservedBoard::Delta updates;
        openCell(x, y, updates);

        if (status_ == Status::Lost) return false;
        checkWin();
        if (!updates.upd.empty()) analysis_.update(updates);
        return true;
    }

void GameController::toggleFlag(int x, int y){
        if (status_ != Status::Playing) return;
        if (revealed_[x][y]) return;
        flags_[x][y] ^= 1;
        flagsRemaining_ += flags_[x][y] ? -1 : 1;
        ++revision_;
    }

int GameController::adjacentMines(int x, int y) const{
        int cnt = 0;
        forEachAdjacent(x, y, rows_, cols_, [&](int nx, int ny) { cnt += layout_[nx][ny]; });
        return cnt;
    }

void GameController::generate(){
        const int total = rows_ * cols_;
        std::vector<int> cells(total);
        for (int i = 0; i < total; ++i) cells[i] = i;
        for (int i = total - 1; i > 0; --i) {
            const std::uint64_t j = rng_.next() % (i + 1);
            std::swap(cells[i], cells[j]);
        }
        for (int k = 0; k < mines_ && k < total; ++k)
            layout_.at(cells[k] / cols_ + 1, cells[k] % cols_ + 1) = 1;
    }

void GameController::chord(int x, int y){
        if (status_ != Status::Playing || !revealed_[x][y]) return;
        const int num = adjacentMines(x, y);
        if (num == 0) return;
        int flagCount = 0;
        forEachAdjacent(x, y, rows_, cols_, [&](int nx, int ny) { flagCount += flags_[nx][ny]; });
        if (flagCount != num) return;

        ObservedBoard::Delta updates;
        bool any = false;
        forEachAdjacent(x, y, rows_, cols_, [&](int nx, int ny) {
            if (!flags_[nx][ny] && !revealed_[nx][ny]) {
                any = true;
                openCell(nx, ny, updates);
            }
        });
        if (any) { ++moves_; ++revision_; }
        checkWin();
        if (!updates.upd.empty()) analysis_.update(updates);
    }

void GameController::openCell(int x, int y, ObservedBoard::Delta& updates){
        if (revealed_[x][y] || flags_[x][y]) return;
        if (layout_[x][y]) {
            status_ = Status::Lost;
            explodedX_ = x; explodedY_ = y;
            return;
        }
        revealFlood(x, y, updates);
    }

void GameController::revealFlood(int x, int y, ObservedBoard::Delta& updates){
        if (revealed_[x][y] || layout_[x][y]) return;
        revealed_[x][y] = 1;
        ++revealedCount_;
        const Cell v = (Cell)(adjacentMines(x, y));
        updates.upd.push_back({analysis_.state().id(x, y), v});
        if (v == Cell::Num0)
            forEachAdjacent(x, y, rows_, cols_, [&](int nx, int ny) { revealFlood(nx, ny, updates); });
    }

void GameController::relocateMine(int x, int y){
        std::vector<std::pair<int, int>> empty;
        for (int i = 1; i <= rows_; ++i)
            for (int j = 1; j <= cols_; ++j)
                if (!layout_[i][j] && !(i == x && j == y)) empty.emplace_back(i, j);
        if (empty.empty()) return;
        const std::uint64_t pick = rng_.next() % empty.size();
        const auto [nx, ny] = empty[pick];
        layout_[x][y] = 0;
        layout_[nx][ny] = 1;
    }

void GameController::checkWin(){
        if (status_ != Status::Playing || revealedCount_ != rows_ * cols_ - mines_) return;
        status_ = Status::Won;
        for (int i = 1; i <= rows_; ++i)
            for (int j = 1; j <= cols_; ++j)
                if (layout_[i][j]) flags_[i][j] = 1;
        flagsRemaining_ = 0;
    }

}  // namespace mss
