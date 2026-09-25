#include <cstdio>
#include <cstring>
#include <iterator>

#include "harness.h"
#include "version.h"

namespace test {
// 用例声明（每个测试文件一个函数）。
void boardTransitions();
} // namespace test

namespace {

constexpr test::Case kCases[] = {
    {"board_transitions", test::boardTransitions},
};

} // namespace

int main(int argc, char **argv) {
    if (argc > 1 && std::strcmp(argv[1], "--list") == 0) {
        for (const test::Case &c : kCases)
            std::printf("%s\n", c.name);
        return 0;
    }

    std::printf("=== mss %s | %d case(s) ===\n", mss::kVersion, static_cast<int>(std::size(kCases)));
    std::fflush(stdout);

    int ran = 0;
    for (const test::Case &c : kCases) {
        if (argc > 1 && std::strcmp(argv[1], c.name) != 0)
            continue;
        std::printf("--- %s\n", c.name);
        std::fflush(stdout);
        c.run(); // 失败会直接 exit(1)，不会继续跑后面的用例
        ++ran;
        std::printf("    ok\n");
        std::fflush(stdout);
    }

    if (argc > 1 && ran == 0) {
        std::printf("no such case: %s (use --list)\n", argv[1]);
        return 2;
    }
    std::printf("%d check(s), all passed\n", test::g_checks);
    return 0;
}