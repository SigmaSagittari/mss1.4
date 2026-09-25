#pragma once

#include <cstdio>
#include <cstdlib>
#include <source_location>
#include <string_view>

namespace test {

inline int g_checks = 0;

// 失败立刻终止：一旦某个不变量被破坏，后续检查都建立在错误状态上，
// 继续跑只会刷屏并被误导。与 mss::assert_ 同一哲学 —— 违约即死，不静默。
// 终止码 1 表示测试断言失败。
inline void check(bool ok, std::string_view what, std::source_location location = std::source_location::current()) {
    ++g_checks;
    if (ok)
        return;
    std::printf("[FAIL] %.*s\n        at %s:%u\n", static_cast<int>(what.size()), what.data(), location.file_name(),
                static_cast<unsigned>(location.line()));
    std::fflush(stdout);
    std::exit(1);
}

struct Case {
    const char *name;
    void (*run)();
};

} // namespace test