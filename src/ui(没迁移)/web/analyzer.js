"use strict";

// ---------- 分析模式（盘面编辑） ----------
// 与游玩模式共享全局状态（state/prob/hover/settings 等，由 main.js 声明）。
// 本文件只负责分析模式的进入/退出与盘面编辑。
//
// 交互模型：进入分析模式只进入编辑态（快照盘面），不自动开始计算。
// 左键切换格子的开/关，悬停 + 按 0~8 设数字；编辑即重算概率/详情。
//
// 移植说明：1.2 的「开始分析」后台搜索（/api/analyze/start|stop|status|
// tree|moves）依赖的 MidgameSearch::Session API 在本版本不存在，已删除；
// 盘面编辑保留。

let analyzer = false;   // 分析模式：保留盘面，禁止改盘操作
let prevProb = false;   // 进入分析前的概率显示状态（退出时恢复）
let editBoard = null;   // 分析模式的编辑副本（进入时复制当前盘面）
let importMeta = null;  // 导入盘面的 {rows, cols, mines}（导出/详情用；分析态服务端持有）

// 当前渲染盘面格值：分析模式用编辑副本，否则用游戏盘面。
function boardAt(i, j) {
  return editBoard ? editBoard[i][j] : state.board[i][j];
}

// 进入分析模式：快照盘面 + 开启编辑。概率不自动计算，点「计算概率」按钮才算。
async function enterAnalyzer(btn) {
  analyzer = true;
  btn.classList.add("active");
  btn.textContent = "退出分析";
  await post("/api/analyzer", { active: true });  // 服务端快照原始盘面
  editBoard = state.board.map((row) => row.slice());  // 本地编辑副本
  prevProb = settings.prob;
  settings.prob = true;
  document.getElementById("opt-prob").checked = true;
  prob = null;  // 清除旧概率，避免编辑前后混合显示
  document.getElementById("calc-prob-btn").hidden = false;
  document.getElementById("java-evaluate-btn").hidden = false;
  document.getElementById("import-btn").hidden = false;
  document.getElementById("autoplay-btn").hidden = true;
  render();
}

// 退出分析模式：还原盘面 + 恢复概率状态 + 隐藏分析编辑态。
async function exitAnalyzer(btn) {
  analyzer = false;
  btn.classList.remove("active");
  btn.textContent = "分析";
  editBoard = null;
  importMeta = null;
  document.getElementById("calc-prob-btn").hidden = true;
  document.getElementById("java-evaluate-btn").hidden = true;
  document.getElementById("import-btn").hidden = true;
  document.getElementById("autoplay-btn").hidden = false;
  clearJavaAnalysis();
  settings.prob = prevProb;
  document.getElementById("opt-prob").checked = prevProb;
  prob = null;
  await post("/api/analyzer", { active: false });  // 服务端还原盘面
  state = await api("/api/state");                  // 重新拉取真实盘面
  setupCanvas();                                    // 画布尺寸回到游戏盘面
  render();
}

// 新开局重置分析模式（main.js newGame 调用；盘面被重置，分析上下文失效）。
function resetAnalyzer() {
  if (!analyzer) return;
  analyzer = false;
  editBoard = null;
  importMeta = null;
  document.getElementById("calc-prob-btn").hidden = true;
  document.getElementById("java-evaluate-btn").hidden = true;
  document.getElementById("import-btn").hidden = true;
  document.getElementById("autoplay-btn").hidden = false;
  clearJavaAnalysis();
  const abtn = document.getElementById("analyzer-btn");
  abtn.classList.remove("active");
  abtn.textContent = "分析";
}

// 编辑一格：更新本地编辑副本 + 同步后端分析视图。
// 后端协议 v=0..8 数字、v=9 盖上（Hidden）；本地 editBoard 用 -1 表示封闭。
// 服务端会预检合法性：矛盾编辑（数字超过周围格数）被回滚并返回 invalid=true，
// 本地同步回滚并提示。编辑后概率过期（棋盘概率层清除），点「计算概率」刷新。
async function editCell(x, y, next) {
  const oldV = editBoard[x - 1][y - 1];
  editBoard[x - 1][y - 1] = next;
  prob = null;
  clearJavaAnalysis();
  render();
  let invalid = false;
  try {
    const r = await post("/api/edit", { x, y, v: next === -1 ? 9 : next });
    invalid = !!(r && r.invalid);
  } catch (_) { /* 忽略瞬时错误 */ }
  if (invalid) {
    editBoard[x - 1][y - 1] = oldV;
    const el = document.getElementById("detail-text");
    if (el) el.textContent = "该编辑导致盘面矛盾（数字超过周围可放雷的格数），已回滚";
  }
}

// 手动计算概率：点「计算概率」按钮才请求并绘制（分析模式）。
async function calcProb() {
  const btn = document.getElementById("calc-prob-btn");
  if (btn) {
    btn.disabled = true;
    btn.textContent = "计算中…";
  }
  const ok = await refreshProb();
  render();
  if (btn) {
    btn.disabled = false;
    btn.textContent = "计算概率";
  }
  return ok;
}

// 分析模式悬停详情：从本地编辑盘面 + 最近一次的概率网格构建（不发请求）。
function analyzerDetailText(c) {
  const i = c.x - 1, j = c.y - 1;
  const v = boardAt(i, j);
  let stateLine;
  if (v === -2) stateLine = "已标旗";
  else if (v >= 0 && v <= 8) stateLine = "已翻开，数字 " + v;
  else stateLine = "未翻开";
  const lines = ["格子 (" + c.x + ", " + c.y + ")",
    "总雷数: " + (importMeta ? importMeta.mines : (state && state.mines != null ? state.mines : "?")),
    "状态: " + stateLine];
  if (prob && prob.prob) {
    const p = prob.prob[i][j];
    lines.push("雷概率: " + (p * 100).toFixed(2) + "%");
  } else {
    lines.push("概率未计算，点「计算概率」");
  }
  return lines.join("\n");
}

// 分析按钮：一键切换到分析编辑态（保留当前盘面，不开始计算）。
document.getElementById("analyzer-btn").addEventListener("click", async () => {
  const btn = document.getElementById("analyzer-btn");
  if (analyzer) await exitAnalyzer(btn);
  else await enterAnalyzer(btn);
});

// 「计算概率」按钮：分析模式下手动触发概率计算。
document.getElementById("calc-prob-btn").addEventListener("click", calcProb);

document.getElementById("java-evaluate-btn").addEventListener("click", async () => {
  const btn = document.getElementById("java-evaluate-btn");
  btn.disabled = true;
  btn.textContent = "分析中…";
  try {
    const data = await post("/api/java-evaluate", {});
    if (data && data.error) {
      document.getElementById("detail-text").textContent = "Java 分析失败: " + data.error;
      return;
    }
    showJavaEvaluate(data);
  } catch (err) {
    document.getElementById("detail-text").textContent = "Java 分析失败: " + err.message;
  } finally {
    btn.disabled = false;
    btn.textContent = "Java 分析";
  }
});

// ---------- 导入盘面（仅分析） ----------
// 本地编辑副本与导入文本对齐：F 只作前端旗标记（-2），服务端分析里仍是未开格。
function editBoardFromText(text) {
  const lines = text.replace(/\r/g, "").split("\n");
  const board = [];
  let header = true;
  for (const line of lines) {
    if (line.trim() === "") continue;  // 末尾空行
    if (header) { header = false; continue; }  // 首行是 行x列/雷数
    const row = [];
    for (let j = 0; j < line.length; j++) {
      const ch = line[j];
      if (ch === "F") row.push(-2);
      else if (ch === "H") row.push(-1);
      else row.push(+ch);
    }
    board.push(row);
  }
  return board;
}

async function importBoard() {
  const errEl = document.getElementById("import-error");
  const text = document.getElementById("import-text").value;
  if (!text.trim()) {
    errEl.textContent = "内容为空，请先粘贴盘面文本。";
    errEl.hidden = false;
    return;
  }
  const confirmBtn = document.getElementById("import-confirm");
  confirmBtn.disabled = true;
  try {
    const r = await post("/api/import", { text });
    if (r.ok) {
      editBoard = editBoardFromText(text);
      const m = text.match(/^\s*(\d+)\s*x\s*(\d+)\s*\/\s*(\d+)\s*$/m);
      importMeta = m ? { rows: +m[1], cols: +m[2], mines: +m[3] } : null;
      prob = null;
      clearJavaAnalysis();
      document.getElementById("import-modal").hidden = true;
      document.getElementById("import-text").value = "";
      errEl.hidden = true;
      setupCanvas();  // 画布尺寸跟随导入盘面（避免大棋盘被截断）
      document.getElementById("detail-text").textContent =
        "导入成功（仅分析视图，退出分析即还原）";
      render();
    } else {
      errEl.textContent = r.error || "导入失败";
      errEl.hidden = false;
    }
  } catch (err) {
    errEl.textContent = "导入失败: " + err.message;
    errEl.hidden = false;
  } finally {
    confirmBtn.disabled = false;
  }
}

document.getElementById("import-btn").addEventListener("click", () => {
  document.getElementById("import-error").hidden = true;
  document.getElementById("import-modal").hidden = false;
  document.getElementById("import-text").focus();
});
document.getElementById("import-confirm").addEventListener("click", importBoard);
document.getElementById("import-cancel").addEventListener("click", () => {
  document.getElementById("import-modal").hidden = true;
});
