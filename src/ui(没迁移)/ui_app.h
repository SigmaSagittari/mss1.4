#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "core/assert.h"
#include "ref/java_evaluate.h"
#include "ui/game_control.h"
#include "ui/http_server.h"
#include "ui/interactive.h"

#include <windows.h>
#include <shellapi.h>
#ifdef _MSC_VER
#pragma comment(lib, "shell32.lib")
#endif

namespace mss {

// UI 应用：本地 HTTP 服务 + 浏览器前端。
// 对外只暴露 run()；内部路由/JSON/静态文件全部私有。
//
// 移植说明：1.2 的后台分析模式（MidgameSearch::Session 渐进展开 + 搜索树
// 浏览）依赖的引擎 API 在本版本不存在（本版本是同步预算制
// MidgameSearch::get()），相关路由/线程/树展示已删除；分析模式的盘面编辑
// （快照/还原）保留。
class UiApp {
public:
    explicit UiApp(int port = 18080) : port_(port) {}

int run();

private:
    // ---------- 路由 ----------
HttpResponse handle(const HttpRequest& req);

    // ---------- 静态文件 ----------
static std::string exeDirectory();

HttpResponse serveFile(const std::string& name) const;

static std::string mimeOf(const std::string& name);

    // ---------- JSON 辅助 ----------
static std::string countDigits(long double v);

    // 候选方案数人类可读形式：小值完整显示，大值科学计数法。
    // long double 是浮点不是高精度整数，超过一定量级直接科学计数，
    // 避免显示一长串无意义的精确数字。
static std::string formatCount(long double v);

static std::string jsonString(const std::string& s);

static HttpResponse json(const std::string& body);

static int bodyInt(const std::string& body, const std::string& key);

static bool bodySeed(const std::string& body, const std::string& key, unsigned& out);

    // 取 JSON 字符串字段值（含 \n \r \t \" \\ 转义还原）。
static bool bodyString(const std::string& body, const std::string& key, std::string& out);

static int queryInt(const HttpRequest& req, const std::string& key, int def);

    // ---------- 盘面 JSON ----------
HttpResponse jsonState() const;

int cellValue(int x, int y) const;

const char* statusText() const;

HttpResponse jsonNew(const HttpRequest& req);

HttpResponse jsonReveal(const HttpRequest& req);

HttpResponse jsonFlag(const HttpRequest& req);

    // ---------- 分布算法配置 ----------
    // 三种模式：auto（小盘面 DFS + 大盘面图分解，默认）/ old（旧 DFS）/
    // graph（图分解）。切换在服务端清空分布池并全量重建概率；前端把
    // 本地概率/详情缓存一并失效。
    using DistMode = GameController::Analysis::DistMode;

std::string distModeName() const;

HttpResponse jsonConfigGet() const;

HttpResponse jsonConfig(const HttpRequest& req);

    // ---------- 概率 ----------
HttpResponse jsonProbability();

HttpResponse jsonDetail(const HttpRequest& req);

HttpResponse jsonJavaEvaluate();

    // 盘面导入/导出（文本格式：首行 行x列/雷数，其后每行一列字符 0-8/H/F）。
    // 导出：实战与分析模式都可用；导入：仅分析模式。
HttpResponse jsonExport();

HttpResponse jsonImport(const HttpRequest& req);

std::string getDetailInfo(int x, int y);

    // ---------- 全局分析（仅编辑器，无搜索） ----------
    // 分析模式：进入时快照原始盘面（退出还原），编辑直接改分析视图的
    // ObservedBoard；每次编辑后从（可能被编辑过的）盘面全量重建
    // basic/structure/probability，概率/详情立即反映编辑结果。
    // 1.2 的「开始分析」搜索（MidgameSearch::Session 渐进展开 + 搜索树浏览）
    // 在本版本不存在，已删除，盘面编辑保留。

    // 进入/退出分析模式：进入保存盘面快照，退出还原（编辑不污染真实游戏盘面）。
HttpResponse jsonAnalyzer(const HttpRequest& req);

    // 编辑：改分析视图的盘面格值（v=0..8 数字，9=盖上 Hidden）。仅分析模式生效。
    // initFromState 返回 false = 盘面矛盾（如数字超过周围格数）：不回滚的话
    // 分布层会枚举出空分布并触发引擎断言，必须在进入 Exact 前拦截——
    // 这里回滚该格并返回 invalid=true。合法性检查与重建共用同一遍计算。
HttpResponse jsonEdit(const HttpRequest& req);

void openBrowser() const;

static bool envFlagSet(const char* name);

    // 启动状态写盘（mss_run.log）：端口占用/崩溃等原因不明时方便排查。
static void writeRunLog(const std::string& msg);

    HttpServer server_;
    std::unique_ptr<GameController> game_;
    std::unique_ptr<ObservedBoard::Result> editSaved_;  // 分析模式进入时的盘面快照（退出还原）
    bool analyzerActive_ = false;
    int port_ = 18080;
    int computedMs_ = 0;
};

}  // namespace mss
