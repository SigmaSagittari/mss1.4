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
    std::printf("=== mss %s | %d case(s) ===\n", mss::kVersion, static_cast<int>(std::size(kCases)));
    int ran = 0;
    for (const test::Case &c : kCases) {
        if (argc > 1 && std::strcmp(argv[1], c.name) != 0)
            continue;
        const int failuresBefore = test::g_failures;
        std::printf("--- %s\n", c.name);
        c.run();
        ++ran;
        std::printf("    %s\n", test::g_failures == failuresBefore ? "ok" : "FAILED");
    }
    if (argc > 1 && ran == 0) {
        std::printf("no such case: %s\n", argv[1]);
        return 2;
    }
    std::printf("%d check(s), %d failure(s), %d case(s) ran\n", test::g_checks, test::g_failures, ran);
    return test::g_failures == 0 ? 0 : 1;
}