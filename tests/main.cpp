#include <cstdio>
#include <cstring>
#include <iterator>
#include <new>

#include "harness.h"
#include "version.h"

namespace test {
// 用例声明（每个测试文件一个函数）。
void basic();
void boardTransitions();
void structure();
} // namespace test

namespace {

constexpr test::Case kCases[] = {
    {"basic", test::basic},
    {"board_transitions", test::boardTransitions},
    {"structure", test::structure},
};

// 顺序跑用例。失败会直接 exit（见 harness.h），所以这里只管"全部跑完"的路径。
int runCases(int argc, char **argv) {
    std::printf("=== mss %s | %d case(s) ===\n", mss::kVersion, static_cast<int>(std::size(kCases)));
    std::fflush(stdout);

    int ran = 0;
    for (const test::Case &c : kCases) {
        if (argc > 1 && std::strcmp(argv[1], c.name) != 0)
            continue;
        std::printf("--- %s\n", c.name);
        std::fflush(stdout);
        c.run();
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

} // namespace

int main(int argc, char **argv) {
    if (argc > 1 && std::strcmp(argv[1], "--list") == 0) {
        for (const test::Case &c : kCases)
            std::printf("%s\n", c.name);
        return 0;
    }
    // 唯一接住异常的地方：只为在内存耗尽时留下人话，而不是 terminate 无声退出。
    // 项目其余部分不使用异常 —— 这里是刻意的例外。
    try {
        return runCases(argc, argv);
    } catch (const std::bad_alloc &) {
        std::printf("[FATAL] 内存耗尽（bad_alloc）：盘面规模超出可用内存\n");
        return 3;
    }
}