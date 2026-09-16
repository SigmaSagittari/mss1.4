#include "ui/ui_app.h"

namespace mss {
namespace {

// ── 盘面文本格式（导入/导出）──
//   首行：行x列/雷数（允许空格，如 "30 x 30 / 225"）
//   随后每行一列字符：0-8 = 已翻开数字，H = 未翻开，F = 标旗（仅前端标记，
//   分析层一律视为未开格——旗是猜测，不影响概率）。
// 解析失败返回 false 并给 error。严格校验，防止各种奇怪的输入。
struct ImportBoard {
    int rows = 0;
    int cols = 0;
    int mines = 0;
    std::vector<Cell> cells;   // 行主序，rows×cols
};

bool parseDigits(const std::string& str, size_t from, size_t to, int& out) {
    out = 0;
    if (to <= from) return false;
    for (int i = from; i < (int)(to); ++i) {
        if (str[i] < '0' || str[i] > '9') return false;
        out = out * 10 + (str[i] - '0');
        if (out > 100000) return false;
    }
    return true;
}

bool parseBoardText(const std::string& text, ImportBoard& out, std::string& error) {
    // 去 UTF-8 BOM。
    std::string s = text;
    if (s.size() >= 3 && (unsigned char)(s[0]) == 0xEF &&
        (unsigned char)(s[1]) == 0xBB &&
        (unsigned char)(s[2]) == 0xBF)
        s.erase(0, 3);

    // 分行（兼容 CRLF / LF）；末尾空行忽略。
    std::vector<std::string> lines;
    std::string cur;
    for (char ch : s) {
        if (ch == '\n') {
            lines.push_back(cur);
            cur.clear();
        } else {
            cur += ch;
        }
    }
    if (!cur.empty()) lines.push_back(cur);
    // 行首行尾的 CR/空白只允许出现在整行空白里；末尾空白行忽略（CRLF 的 \r 也算空行）。
    auto blankLine = [](const std::string& l) {
        for (char c : l)
            if (c != '\r' && c != ' ' && c != '\t') return false;
        return true;
    };
    while (!lines.empty() && blankLine(lines.back())) lines.pop_back();
    if (lines.size() < 2) {
        error = "内容太短（缺首行或盘面行）";
        return false;
    }

    // 首行：行x列/雷数。
    const std::string& head = lines[0];
    {
        const size_t b = head.find_first_not_of(" \t");
        const size_t e = head.find_last_not_of(" \t");
        if (b == std::string::npos) {
            error = "首行格式错误（应为 行x列/雷数）";
            return false;
        }
        const std::string h = head.substr(b, e - b + 1);
        std::string compact;
        for (char ch : h)
            if (ch != ' ' && ch != '\t' && ch != '\r') compact += ch;  // 允许组件间空格/CR
        const size_t x = compact.find('x');
        const size_t slash = compact.find('/');
        if (x == std::string::npos || slash == std::string::npos || x >= slash) {
            error = "首行格式错误（应为 行x列/雷数）";
            return false;
        }
        if (!parseDigits(compact, 0, x, out.rows) || !parseDigits(compact, x + 1, slash, out.cols) ||
            !parseDigits(compact, slash + 1, compact.size(), out.mines)) {
            error = "首行格式错误（应为 行x列/雷数）";
            return false;
        }
    }
    if (out.rows < 2 || out.rows > 100 || out.cols < 2 || out.cols > 100) {
        error = "行列数须在 2~100 之间";
        return false;
    }
    if (out.mines < 0 || out.mines > out.rows * out.cols - 1) {
        error = "雷数超出范围（须为 0~行×列−1）";
        return false;
    }
    if (lines.size() - 1 != out.rows) {
        error = "盘面行数与首行声明不符";
        return false;
    }

    const size_t cellCount = out.rows * out.cols;
    out.cells.assign(cellCount, Cell::Hidden);
    for (int i = 0; i < out.rows; ++i) {
        std::string row = lines[i + 1];
        while (!row.empty() && (row.back() == '\r' || row.back() == ' ' || row.back() == '\t'))
            row.pop_back();
        if (row.size() != out.cols) {
            error = "第 " + std::to_string(i + 1) + " 行长度应为 " + std::to_string(out.cols) +
                    "（实际 " + std::to_string(row.size()) + "）";
            return false;
        }
        for (int j = 0; j < out.cols; ++j) {
            const char ch = row[j];
            const size_t idx = i * out.cols + j;
            if (ch >= '0' && ch <= '8') {
                out.cells[idx] = (Cell)(ch - '0');
            } else if (ch == 'H' || ch == 'F') {  // F 只作前端标记，分析一律未开
                out.cells[idx] = Cell::Hidden;
            } else {
                error = "第 " + std::to_string(i + 1) + " 行含非法字符 '" + std::string(1, ch) +
                        "'（只允许 0-8/H/F）";
                return false;
            }
        }
    }
    return true;
}

}  // namespace

int UiApp::run(){
        server_.setHandler([this](const HttpRequest& req) { return handle(req); });
        if (!server_.start(port_)) {
            std::cerr << "无法在 127.0.0.1:" << port_ << " 启动服务（端口被占用？）\n";
            writeRunLog("failed: port " + std::to_string(port_) + " busy");
            return 1;
        }
        game_ = std::make_unique<GameController>(9, 9, 10, std::random_device{}());
        const std::string url = "http://127.0.0.1:" + std::to_string(port_) + "/";
        std::cout << "MSS 扫雷 UI 已启动: " << url << "\n";
        std::cout << "按 Ctrl+C 退出。\n";
        writeRunLog("started: " + url);
        // 设置环境变量 MSS_NO_BROWSER=1 可跳过自动打开浏览器（无头测试用）
        if (!envFlagSet("MSS_NO_BROWSER")) openBrowser();
        server_.run();
        server_.stop();
        return 0;
    }

HttpResponse UiApp::handle(const HttpRequest& req){
        const std::string& p = req.path;
        if (p == "/" || p == "/index.html") return serveFile("index.html");
        if (p == "/style.css") return serveFile("style.css");
        if (p == "/main.js") return serveFile("main.js");
        if (p == "/analyzer.js") return serveFile("analyzer.js");
        if (p == "/favicon.ico") return {204, "text/plain", ""};

        if (p == "/api/state") return jsonState();
        if (p == "/api/new" && req.method == "POST") return jsonNew(req);
        if (p == "/api/reveal" && req.method == "POST") return jsonReveal(req);
        if (p == "/api/flag" && req.method == "POST") return jsonFlag(req);
        if (p == "/api/config" && req.method == "GET") return jsonConfigGet();
        if (p == "/api/config" && req.method == "POST") return jsonConfig(req);
        if (p == "/api/probability") return jsonProbability();
        if (p == "/api/detail") return jsonDetail(req);
        if (p == "/api/java-evaluate" && req.method == "POST") return jsonJavaEvaluate();
        if (p == "/api/analyzer" && req.method == "POST") return jsonAnalyzer(req);
        if (p == "/api/edit" && req.method == "POST") return jsonEdit(req);
        if (p == "/api/export" && req.method == "POST") return jsonExport();
        if (p == "/api/import" && req.method == "POST") return jsonImport(req);
        return {404, "text/plain; charset=utf-8", "not found"};
    }

std::string UiApp::exeDirectory(){
        char buf[MAX_PATH];
        const DWORD n = GetModuleFileNameA(nullptr, buf, MAX_PATH);
        std::string path(buf, n > 0 ? n : 0);
        const size_t slash = path.find_last_of("\\/");
        return slash == std::string::npos ? std::string() : path.substr(0, slash + 1);
    }

HttpResponse UiApp::serveFile(const std::string& name) const{
        // 只服务源码树 src/ui/web（src 是唯一基准，不做构建期复制，杜绝配置间快照不一致）。
        // 沿 exe 目录向上找 src/ui/web：无论工作目录/输出布局怎么变都能命中。
        // 1.2 是平铺仓库（<root>/src/ui/web）；本仓库项目在子目录（<root>/mss/src/ui/web），
        // 两种布局都试。
        std::string dir = exeDirectory();
        for (int up = 0; up < 10 && !dir.empty(); ++up) {
            for (const std::string& root : {dir + "src/ui/web/", dir + "mss/src/ui/web/"}) {
                std::ifstream in(root + name, std::ios::binary);
                if (in) {
                    std::string body((std::istreambuf_iterator<char>(in)),
                                     std::istreambuf_iterator<char>());
                    return {200, mimeOf(name), std::move(body)};
                }
            }
            const size_t slash = dir.find_last_of("\\/", dir.size() - 2);
            dir = (slash == std::string::npos) ? std::string() : dir.substr(0, slash + 1);
        }
        return {404, "text/plain; charset=utf-8", "src/ui/web/" + name + " not found"};
    }

std::string UiApp::mimeOf(const std::string& name){
        if (name.ends_with(".html")) return "text/html; charset=utf-8";
        if (name.ends_with(".css")) return "text/css; charset=utf-8";
        if (name.ends_with(".js")) return "application/javascript; charset=utf-8";
        if (name.ends_with(".svg")) return "image/svg+xml";
        return "application/octet-stream";
    }

std::string UiApp::countDigits(long double v){
        assert_(std::isfinite(v) && v >= 0.0L, "UiApp::countDigits: 非有限候选数");
        std::ostringstream os;
        os << std::fixed << std::setprecision(0) << v;
        return os.str();
    }

std::string UiApp::formatCount(long double v){
        assert_(std::isfinite(v) && v >= 0.0L, "UiApp::formatCount: 非有限候选数");
        if (v < 1e6L) return countDigits(v);
        std::ostringstream os;
        os << std::setprecision(4) << v;
        return os.str();
    }

std::string UiApp::jsonString(const std::string& s){
        std::ostringstream os;
        os << '"';
        for (char ch : s) {
            switch (ch) {
                case '"': os << "\\\""; break;
                case '\\': os << "\\\\"; break;
                case '\n': os << "\\n"; break;
                case '\r': os << "\\r"; break;
                case '\t': os << "\\t"; break;
                default: os << ch;
            }
        }
        os << '"';
        return os.str();
    }

HttpResponse UiApp::json(const std::string& body){
        return {200, "application/json; charset=utf-8", body};
    }

int UiApp::bodyInt(const std::string& body, const std::string& key){
        std::string needle = "\"" + key + "\"";
        size_t pos = body.find(needle);
        if (pos == std::string::npos) return -1;
        size_t colon = body.find(':', pos);
        if (colon == std::string::npos) return -1;
        return std::atoi(body.c_str() + colon + 1);
    }

bool UiApp::bodySeed(const std::string& body, const std::string& key, unsigned& out){
        std::string needle = "\"" + key + "\"";
        size_t pos = body.find(needle);
        if (pos == std::string::npos) return false;
        size_t colon = body.find(':', pos);
        if (colon == std::string::npos) return false;
        out = std::strtoul(body.c_str() + colon + 1, nullptr, 10);
        return true;
    }

bool UiApp::bodyString(const std::string& body, const std::string& key, std::string& out){
        std::string needle = "\"" + key + "\"";
        size_t pos = body.find(needle);
        if (pos == std::string::npos) return false;
        size_t colon = body.find(':', pos);
        if (colon == std::string::npos) return false;
        size_t q = body.find('"', colon + 1);
        if (q == std::string::npos) return false;
        out.clear();
        for (int i = q + 1; i < (int)(body.size()); ++i) {
            const char ch = body[i];
            if (ch == '"') return true;
            if (ch == '\\' && i + 1 < body.size()) {
                const char esc = body[++i];
                switch (esc) {
                    case 'n': out += '\n'; break;
                    case 'r': out += '\r'; break;
                    case 't': out += '\t'; break;
                    case '"': out += '"'; break;
                    case '\\': out += '\\'; break;
                    default: out += esc; break;
                }
            } else {
                out += ch;
            }
        }
        return false;
    }

int UiApp::queryInt(const HttpRequest& req, const std::string& key, int def){
        const std::map<std::string, std::string>::const_iterator it = req.query.find(key);
        if (it == req.query.end()) return def;
        return std::atoi(it->second.c_str());
    }

HttpResponse UiApp::jsonState() const{
        const GameController::game_info& gi = game_->info();
        std::ostringstream os;
        os << "{\"status\":\"" << statusText() << "\""
           << ",\"rows\":" << gi.rows << ",\"cols\":" << gi.cols
           << ",\"mines\":" << gi.mines
           << ",\"flagsRemaining\":" << gi.flagsRemaining
           << ",\"moves\":" << gi.moves << ",\"seed\":" << gi.seed << ",\"board\":[";
        for (int i = 1; i <= gi.rows; ++i) {
            if (i > 1) os << ',';
            os << '[';
            for (int j = 1; j <= gi.cols; ++j) {
                if (j > 1) os << ',';
                os << cellValue(i, j);
            }
            os << ']';
        }
        os << "]}";
        return json(os.str());
    }

int UiApp::cellValue(int x, int y) const{
        const GameController::game_info& gi = game_->info();
        if (gi.status == GameController::Status::Lost && gi.layout[x][y]) {
            const auto [ex, ey] = game_->exploded();
            return (x == ex && y == ey) ? -4 : -3;
        }
        if (gi.revealed[x][y]) return game_->adjacentMines(x, y);
        if (gi.flags[x][y]) return -2;
        return -1;
    }

const char* UiApp::statusText() const{
        switch (game_->info().status) {
            case GameController::Status::Playing: return "playing";
            case GameController::Status::Won: return "won";
            case GameController::Status::Lost: return "lost";
        }
        return "playing";
    }

HttpResponse UiApp::jsonNew(const HttpRequest& req){
        int rows = std::clamp(bodyInt(req.body, "rows"), 2, 100);
        int cols = std::clamp(bodyInt(req.body, "cols"), 2, 100);
        int maxMines = rows * cols - 1;
        int mines = std::clamp(bodyInt(req.body, "mines"), 1, maxMines);
        unsigned seed;
        if (!bodySeed(req.body, "seed", seed)) seed = std::random_device{}();
        game_ = std::make_unique<GameController>(rows, cols, mines, seed);
        // 新局丢弃分析会话（编辑快照/分析态随旧局失效）。
        editSaved_.reset();
        analyzerActive_ = false;
        return jsonState();
    }

HttpResponse UiApp::jsonReveal(const HttpRequest& req){
        int x = bodyInt(req.body, "x");
        int y = bodyInt(req.body, "y");
        if (x >= 1 && x <= game_->info().rows && y >= 1 && y <= game_->info().cols) {
            const std::chrono::steady_clock::time_point t0 =
                std::chrono::steady_clock::now();
            game_->reveal(x, y);
            computedMs_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now() - t0)
                              .count();
        }
        return jsonState();
    }

HttpResponse UiApp::jsonFlag(const HttpRequest& req){
        int x = bodyInt(req.body, "x");
        int y = bodyInt(req.body, "y");
        if (x >= 1 && x <= game_->info().rows && y >= 1 && y <= game_->info().cols) {
            game_->toggleFlag(x, y);
        }
        return jsonState();
    }

std::string UiApp::distModeName() const{
        switch (game_->analysis().distMode()) {
            case DistMode::Old: return "old";
            case DistMode::Graph: return "graph";
            default: return "auto";
        }
    }

HttpResponse UiApp::jsonConfigGet() const{
        return json("{\"distMode\":\"" + distModeName() + "\"}");
    }

HttpResponse UiApp::jsonConfig(const HttpRequest& req){
        const bool graph = req.body.find("graph") != std::string::npos;
        const bool old = req.body.find("old") != std::string::npos;
        game_->analysis().setDistMode(graph ? DistMode::Graph : old ? DistMode::Old
                                                                    : DistMode::Auto);
        return json("{\"distMode\":\"" + distModeName() + "\",\"poolCleared\":true}");
    }

HttpResponse UiApp::jsonProbability(){
        GameController::Analysis& an = game_->analysis();
        const Grid<long double> grid = Interactive::materializeProbability(an);

        // 输出钳制：极端盘面（如编辑出的无解盘面）引擎可能产生 inf/nan，
        // 不许泄漏进 JSON（否则前端 res.json() 解析失败、概率整个消失）。
        auto safe = [](long double v) -> double {
            return std::isfinite(v) ? v : 0.0;
        };

        // 循环按分析视图尺寸（导入可改变盘面尺寸，游戏行/列会截断）。
        const int rows = an.state().rows;
        const int cols = an.state().cols;

        std::ostringstream os;
        os << std::setprecision(12);
        os << "{\"candidates\":\"" << formatCount(Interactive::candidates(an))
           << "\",\"tProb\":" << safe(Interactive::tCellProbability(an))
           << ",\"computedMs\":" << computedMs_ << ",\"prob\":[";
        for (int i = 1; i <= rows; ++i) {
            if (i > 1) os << ',';
            os << '[';
            for (int j = 1; j <= cols; ++j) {
                if (j > 1) os << ',';
                os << safe(grid[i][j]);
            }
            os << ']';
        }
        os << "]}";
        return json(os.str());
    }

HttpResponse UiApp::jsonDetail(const HttpRequest& req){
        int x = queryInt(req, "x", 0);
        int y = queryInt(req, "y", 0);
        const GameController::game_info& gi = game_->info();
        if (x < 1 || x > gi.rows || y < 1 || y > gi.cols)
            return json("{\"text\":\"（悬停在格子上查看）\"}");
        return json("{\"text\":" + jsonString(getDetailInfo(x, y)) + "}");
}

HttpResponse UiApp::jsonJavaEvaluate(){
        if (!analyzerActive_)
            return {409, "application/json; charset=utf-8", "{\"error\":\"Java analysis requires analyzer mode\"}"};
        GameController::Analysis& an = game_->analysis();
        // 无解盘面：引擎会产 NaN/Inf，直接显式报错，不静默钳 0 掩盖。
        if (an.probability().candidates == 0.0L)
            return json("{\"error\":\"盘面无可行的雷位方案（候选方案数 0），无法评估\"}");
        const std::chrono::steady_clock::time_point started =
            std::chrono::steady_clock::now();
        const LongTermRiskReference::Config riskConfig{};
        const JavaEvaluate::Config evaluateConfig{};
        const LongTermRiskReference::Influence risk = LongTermRiskReference::findInfluence(
            an.state(), an.basicMarks(), an.structure(), an.probability(), an.shapes(),
            an.dists(), {}, riskConfig);
        const JavaEvaluate::Result result = JavaEvaluate::solve(
            an.state(), an.basicMarks(), an.structure(), an.probability(), an.shapes(),
            an.dists(), risk, {}, evaluateConfig);
        const std::chrono::milliseconds::rep elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - started)
                .count();
        // 数值异常（NaN/Inf）也要显式报错，而不是钳 0。
        if (!std::isfinite(result.weight))
            return json("{\"error\":\"概率引擎数值异常（NaN/Inf），盘面可能无解或数值溢出\"}");
        for (const JavaEvaluate::Candidate& c : result.candidates)
            if (!std::isfinite(c.safety) || !std::isfinite(c.influence) ||
                !std::isfinite(c.expectedClears) || !std::isfinite(c.weight))
                return json("{\"error\":\"概率引擎数值异常（NaN/Inf），盘面可能无解或数值溢出\"}");
        // influence 输出归一化占比（= tally / 方案数），供前端直接显示百分比。
        const long double candidates = an.probability().candidates;
        auto influenceRatio = [&](long double influence) {
            return influence / candidates;
        };
        std::ostringstream os;
        os << std::setprecision(12)
           << "{\"x\":" << result.x << ",\"y\":" << result.y
           << ",\"weight\":" << result.weight
           << ",\"elapsedMs\":" << elapsed << ",\"pseudos\":[";
        for (int i = 0; i < (int)(risk.pseudos.size()); ++i) {
            if (i) os << ',';
            const auto [x, y] = an.state().pos(risk.pseudos[i]);
            os << "[" << x << ',' << y << "]";
        }
        os << "],\"dead\":[";
        for (int i = 0; i < (int)(result.deadCells.size()); ++i) {
            if (i) os << ',';
            const auto [x, y] = an.state().pos(result.deadCells[i]);
            os << "[" << x << ',' << y << "]";
        }
        os << "],\"candidates\":[";
        for (int i = 0; i < (int)(result.candidates.size()); ++i) {
            if (i) os << ',';
            const JavaEvaluate::Candidate& c = result.candidates[i];
            os << "{\"x\":" << c.x << ",\"y\":" << c.y
               << ",\"safety\":" << c.safety
               << ",\"influence\":" << influenceRatio(c.influence)
               << ",\"clears\":" << c.expectedClears
               << ",\"weight\":" << c.weight << '}';
        }
        os << "]}";
        return json(os.str());
}

std::string UiApp::getDetailInfo(int x, int y){
        const GameController::game_info& gi = game_->info();
        std::vector<std::string> lines;
        lines.push_back("格子 (" + std::to_string(x) + ", " + std::to_string(y) + ")");
        lines.push_back("总雷数: " + std::to_string(gi.mines) +
                        "（剩余 " + std::to_string(gi.flagsRemaining) + "）");

        std::string stateLine;
        if (gi.flags[x][y])
            stateLine = "已标旗";
        else if (gi.revealed[x][y])
            stateLine = "已翻开，数字 " + std::to_string(game_->adjacentMines(x, y));
        else
            stateLine = "未翻开";
        lines.push_back("状态: " + stateLine);

        long double p = Interactive::mineProbability(game_->analysis(), x, y);
        std::ostringstream ps;
        ps << std::setprecision(5) << "雷概率: "
           << p << "  (" << p * 100 << "%)";
        lines.push_back(ps.str());

        // 点开结果分布（observe）：爆炸 + 各数字概率（仅未翻开格有意义）。
        // 全部数字保留五位有效数字（有效数字，不是小数点后）。
        if (!gi.revealed[x][y]) {
            const Probability::ObserveResult obr = Interactive::observe(game_->analysis(), x, y);
            auto fmtPct = [](long double v) {
                std::ostringstream os;
                os << std::setprecision(5) << v * 100.0L << '%';
                return os.str();
            };
            lines.push_back("点开: 爆炸 " + fmtPct(obr.explosion));
            for (int k = 0; k <= 8; ++k) {
                lines.push_back("数字 " + std::to_string(k) + ": " + fmtPct(obr.digit[k]));
            }
        }

        std::ostringstream cs;
        cs << "候选方案数: " << formatCount(Interactive::candidates(game_->analysis()));
        lines.push_back(cs.str());
        std::ostringstream tp;
        tp << std::setprecision(5)
           << "非前沿雷概率: " << Interactive::tCellProbability(game_->analysis());
        lines.push_back(tp.str());
        lines.push_back("计算耗时: " + std::to_string(computedMs_) + " ms");

        std::string out;
        for (int i = 0; i < (int)(lines.size()); ++i) {
            if (i) out += "\n";
            out += lines[i];
        }
        return out;
    }

HttpResponse UiApp::jsonAnalyzer(const HttpRequest& req){
        const bool active = req.body.find("true") != std::string::npos;
        ObservedBoard::Result& state = game_->analysis().state();
        if (active) {
            if (!analyzerActive_) editSaved_ = std::make_unique<ObservedBoard::Result>(state);
            analyzerActive_ = true;
        } else {
            if (analyzerActive_ && editSaved_) {
                state = std::move(*editSaved_);
                game_->analysis().initFromState();
            }
            editSaved_.reset();
            analyzerActive_ = false;
        }
        return json("{\"ok\":true}");
    }

HttpResponse UiApp::jsonEdit(const HttpRequest& req){
        if (!analyzerActive_) return json("{\"ok\":true}");
        const int x = bodyInt(req.body, "x");
        const int y = bodyInt(req.body, "y");
        const int v = bodyInt(req.body, "v");
        ObservedBoard::Result& state = game_->analysis().state();
        if (x >= 1 && x <= state.rows && y >= 1 && y <= state.cols && v >= 0 && v <= 9) {
            const Cell old = state.board[x][y];
            const Cell next = (v == 9) ? Cell::Hidden : (Cell)(v);
            if (old != next) {
                state.board[x][y] = next;
                if (!game_->analysis().initFromState()) {
                    state.board[x][y] = old;  // 回滚并重建合法态
                    game_->analysis().initFromState();
                    return json("{\"ok\":true,\"invalid\":true}");
                }
            } else {
                game_->analysis().initFromState();
            }
        }
        return json("{\"ok\":true}");
    }

HttpResponse UiApp::jsonExport(){
        std::ostringstream os;
        if (analyzerActive_) {
            // 分析模式：导出分析视图（数字 / Hidden；F 是前端标记，服务端不存）。
            const ObservedBoard::Result& st = game_->analysis().state();
            os << st.rows << "x" << st.cols << "/" << st.totalMines << "\n";
            for (int x = 1; x <= st.rows; ++x) {
                for (int y = 1; y <= st.cols; ++y) {
                    if (st.board[x][y] == Cell::Hidden) os << 'H';
                    else os << numberValue(st.board[x][y]);
                }
                os << '\n';
            }
        } else {
            // 实战：数字 / 未开 H / 标旗 F；失败后露出的雷按 F 导出（保住已知雷）。
            const GameController::game_info& gi = game_->info();
            os << gi.rows << "x" << gi.cols << "/" << gi.mines << "\n";
            for (int x = 1; x <= gi.rows; ++x) {
                for (int y = 1; y <= gi.cols; ++y) {
                    if (gi.revealed[x][y]) {
                        if (gi.status == GameController::Status::Lost && gi.layout[x][y]) os << 'F';
                        else os << game_->adjacentMines(x, y);
                    } else if (gi.flags[x][y]) {
                        os << 'F';
                    } else {
                        os << 'H';
                    }
                }
                os << '\n';
            }
        }
        return json("{\"text\":" + jsonString(os.str()) + "}");
    }

HttpResponse UiApp::jsonImport(const HttpRequest& req){
        if (!analyzerActive_)
            return {409, "application/json; charset=utf-8",
                    "{\"ok\":false,\"error\":\"导入仅在分析模式下可用\"}"};
        std::string text;
        if (!bodyString(req.body, "text", text)) {
            return json("{\"ok\":false,\"error\":\"缺少 text 字段\"}");
        }
        ImportBoard parsed;
        std::string error;
        if (!parseBoardText(text, parsed, error)) {
            return json("{\"ok\":false,\"error\":" + jsonString(error) + "}");
        }

        ObservedBoard::Result next(parsed.rows, parsed.cols, parsed.mines);
        const int cellCount = parsed.rows * parsed.cols;
        for (int idx = 0; idx < cellCount; ++idx) {
            const int x = idx / parsed.cols + 1;
            const int y = idx % parsed.cols + 1;
            next.board[x][y] = parsed.cells[idx];
        }

        // 覆盖分析视图；不合法（数字矛盾 / 雷数不可行）则回滚。
        ObservedBoard::Result& state = game_->analysis().state();
        ObservedBoard::Result old = state;
        state = std::move(next);
        if (!game_->analysis().initFromState() ||
            game_->analysis().probability().candidates == 0.0L) {
            state = std::move(old);
            game_->analysis().initFromState();
            return json("{\"ok\":false,\"error\":\"盘面与雷数无可行解（数字矛盾或雷数超界）\"}");
        }
        return json("{\"ok\":true}");
    }

void UiApp::openBrowser() const{
        std::string url = "http://127.0.0.1:" + std::to_string(port_) + "/";
        ShellExecuteA(nullptr, "open", url.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
    }

bool UiApp::envFlagSet(const char* name){
        char buf[8] = {};
        return GetEnvironmentVariableA(name, buf, sizeof(buf)) > 0;
    }

void UiApp::writeRunLog(const std::string& msg){
        std::ofstream log("mss_run.log", std::ios::app);
        if (log) log << msg << '\n';
    }

}  // namespace mss
