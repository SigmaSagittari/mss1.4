#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

#include "analysis/basic.h"
#include "analysis/distribution/distribution.h"
#include "analysis/distribution/distribution_graph.h"
#include "analysis/probability/exact.h"
#include "analysis/probability/probability.h"
#include "analysis/structure.h"
#include "core/types.h"
#include "core/utility/rng.h"
#include "core/workspaces.h"

namespace mss {

// ─────────────────────────────────────────────────────────────
// game_control.h — 游戏控制器。
//
// 一个游戏 = 一个对象：持有真实雷位布局 + 分析视图，每次操作后驱动
// 分析管线刷新。控制"打开之后游戏显示什么"：reveal、泛洪、标旗、胜负。
//
// 对外只暴露 info()/reveal()/toggleFlag() 等游戏操作；
// 分析结果通过 analysis() 访问（getter 惰性刷新）。
// ─────────────────────────────────────────────────────────────

// 本版本 core/utility/rng.h 只保留了 splitmix64 函数（Rng 类已移除，
// 测试层另有 mss::test::Rng），UI 层自备一个最小实现，行为与 1.2 一致。
struct Rng {
    std::uint64_t state = 0;

    explicit Rng(std::uint64_t seed = 0) : state(seed) {
    }

    std::uint64_t next() {
        state = splitmix64(state);
        return state;
    }
};

struct GameController {
    enum class Status { Playing, Won, Lost };

    // 游戏信息视图（引用挂载，随调用方生命周期有效）。
    struct game_info {
        const Grid<char> &layout; // truth：雷位
        const Grid<char> &revealed;
        const Grid<char> &flags;
        int rows = 0, cols = 0, mines = 0, moves = 0, flagsRemaining = 0;
        Status status = Status::Playing;
        unsigned seed = 0;
    };

    // ── 分析管线 ──
    // 持有 ObservedBoard（数字/Hidden 分析视图）+ 各层结果 + 池 + 概率引擎。
    // 概率总是全量（Exact::analyze，热池 ~1.5μs）。结构层固定增量更新
    // （update 已彻底测试，全量重建无需保留）。
    struct Analysis {
        // 分布算法选择：auto = 小盘面用旧 DFS、大盘面转图分解（analyze_auto
        // 内置阈值）；old = 全量旧 DFS；graph = 全量图分解。三者产出数值
        // 一致（测试对拍），只在极端盘面的耗时/稳定性上有差别。
        enum class DistMode { Auto, Old, Graph };

        Analysis(int rows, int cols, int mines) : state_(ObservedBoard::analyze(rows, cols, mines)) {
        }

        // 首次全量建 basic + structure + probability（初始全隐藏盘面）。
        // 返回盘面合法性（Basic valid）：合法才继续 structure/probability，
        // 矛盾盘面（分析编辑可能产生）不进入 Exact，由调用方决定回滚。
        // 预检与重建共用同一遍 Basic::analyze，正常编辑零额外计算。
        bool initFromState();

        // 一次揭示后刷新全链：basic → structure → probability。
        // updates 是本次翻开产生的盘面事件。
        void update(const ObservedBoard::Delta &updates);

        const Basic::Result &basicMarks() const {
            return basic_;
        }
        Basic::Result &basicMarks() {
            return basic_;
        }
        const Structure::Result &structure() const {
            return structure_;
        }
        Structure::Result &structure() {
            return structure_;
        }

        Structure::ShapePool &shapes() {
            return shapes_;
        }
        const Probability::Result &probability() const {
            return prob_;
        }

        Distribution::DistPool &dists() {
            return dists_;
        }
        const Distribution::DistPool &dists() const {
            return dists_;
        }

        ObservedBoard::Result &state() {
            return state_;
        }
        const ObservedBoard::Result &state() const {
            return state_;
        }

        DistMode distMode() const {
            return distMode_;
        }

        // 切换分布算法：每次都清空分布池再全量重建概率。
        // 池缓存按算法归属，切走即失效，绝不允许旧算法算出的分布被新算法
        // 当作命中复用（数值虽一致，但"清池即重算"是切换的显式契约）。
        void setDistMode(DistMode mode);

      private:
        // 全量初始化概率状态（开局 / 编辑后 / 模式切换后）。
        void rebuild();

        // 按当前模式填充分布池：auto 自适应（小盘面 DFS / 大盘面图分解），
        // old 只走旧 DFS，graph 只走图分解。共享一个缓存池。
        void fillDistributions();

        ObservedBoard::Result state_; // 分析视图（数字/Hidden），经 update 维护
        Basic::Result basic_;
        Structure::Result structure_;
        Structure::ShapePool shapes_; // 结构池（本游戏生命周期）
        Distribution::DistPool dists_;
        Probability::Result prob_;           // Exact 产物（数值视图）
        DistMode distMode_ = DistMode::Auto; // 分布算法（开局默认 auto）
    };

    // ── 构造：开局即分析 ──
    GameController(int rows, int cols, int mines, unsigned seed)
        : analysis_(rows, cols, mines), layout_(rows, cols, 0), revealed_(rows, cols, 0), flags_(rows, cols, 0), rng_(seed) {
        rows_ = rows;
        cols_ = cols;
        mines_ = mines;
        seed_ = seed;
        flagsRemaining_ = mines;
        generate();
        // 初始盘面（全隐藏）：建一次 basic + structure（空结构）。
        analysis_.initFromState();
    }
    ~GameController() = default;
    GameController(const GameController &) = delete;
    GameController &operator=(const GameController &) = delete;

    game_info info() const;

    // false = 踩雷结束
    bool reveal(int x, int y);

    void toggleFlag(int x, int y);

    int revision() const {
        return revision_;
    }
    std::pair<int, int> exploded() const {
        return {explodedX_, explodedY_};
    }

    int adjacentMines(int x, int y) const;

    // 分析结果访问（getter，ui 层只读）。
    const Analysis &analysis() const {
        return analysis_;
    }
    Analysis &analysis() {
        return analysis_;
    }

  private:
    // ── 游戏 ──
    Analysis analysis_; // 分析管线（构造即初始化，游戏生命周期内持有）
    Grid<char> layout_;
    Grid<char> revealed_;
    Grid<char> flags_;
    int rows_ = 0, cols_ = 0, mines_ = 0, moves_ = 0, flagsRemaining_ = 0;
    Status status_ = Status::Playing;
    unsigned seed_ = 0;
    Rng rng_;
    bool firstMove_ = true;
    int revealedCount_ = 0;
    int explodedX_ = 0, explodedY_ = 0;
    int revision_ = 0;

    // ── 分析 ──
    // Analysis 持有全部分析状态；GameController 只负责游戏规则并驱动它。

    void generate();

    void chord(int x, int y);

    void openCell(int x, int y, ObservedBoard::Delta &updates);

    void revealFlood(int x, int y, ObservedBoard::Delta &updates);

    void relocateMine(int x, int y);

    void checkWin();
};

} // namespace mss
