#pragma once

#include <chrono>
#include <string>

#include "analysis/basic.h"
#include "analysis/distribution/distribution.h"
#include "analysis/probability/probability.h"
#include "analysis/probability/exact.h"
#include "analysis/structure.h"
#include "core/types.h"
#include "ui/game_control.h"

namespace mss {

// ─────────────────────────────────────────────────────────────
// interactive.h — 分析/游戏数据 → UI 可消费表示的翻译层（垃圾桶）。
//
// 各种"计算小垃圾"都扔这里：引擎无关的概率查询、整盘物化、精确对比。
// UiApp 只负责路由/JSON；GameController 只负责游戏规则（翻开/泛洪/标旗/
// 胜负）。本层不含游戏规则或 HTTP 逻辑，全是纯计算。
//
// 移植说明：1.2 的 MidgameSearch::Session 语义在本版本不存在
// （本版本是同步预算制 MidgameSearch::get()），对应的按会话物化函数已删除。
// ─────────────────────────────────────────────────────────────

namespace Interactive {

// ── 引擎无关查询（基于当前 Analysis 管线）──

// 单格雷概率（Mine→1，Unknown→tProb，Safe→0，前沿格→boxProbs）。
long double mineProbability(GameController::Analysis& an, int x, int y);

// 候选方案数。
long double candidates(const GameController::Analysis& an);

// 非前沿（Unknown）格雷密度。
long double tCellProbability(const GameController::Analysis& an);

// 整盘概率网格物化（1-based，与棋盘一致）。逐格查询，O(rows*cols)。
Grid<long double> materializeProbability(GameController::Analysis& an);

// 点开某格的结果分布（explosion + digit[0..8]），详情面板查询。
Probability::ObserveResult observe(GameController::Analysis& an, int x, int y);

// ── 实现区 ──











}  // namespace Interactive

}  // namespace mss
