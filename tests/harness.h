#pragma once

#include <cstdio>
#include <source_location>
#include <string_view>

namespace test {

inline int g_checks = 0;
inline int g_failures = 0;

// 失败只记录，不终止：一次跑完所有用例，最后看总数。
// 与 mss::assert_ 不同 —— 后者是"调用契约被破坏，必须立刻死"，测试里不该用它。
inline void check(bool ok, std::string_view what, std::source_location location = std::source_location::current()) {
    ++g_checks;
    if (ok)
        return;
    ++g_failures;
    std::printf("[FAIL] %.*s\n        at %s:%u\n", static_cast<int>(what.size()), what.data(), location.file_name(),
                static_cast<unsigned>(location.line()));
}

struct Case {
    const char *name;
    void (*run)();
};

} // namespace test