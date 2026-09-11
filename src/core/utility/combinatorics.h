#pragma once

#include <cmath>
#include <vector>

#include "core/assert.h"

namespace mss {

// ─────────────────────────────────────────────────────────────
// combinatorics.h — 组合数对数（共享）。
//
// 概率引擎与分布求解共用同一份阶乘对数表：
// thread_local 惰性扩展，数值口径必须全局一致（曾因表分属不同翻译单元/副本
// 导致长尾 ulp 漂移）。调用方保证 0 <= k <= n；越界 = 调用 bug。
// ─────────────────────────────────────────────────────────────

inline std::vector<long double>& logFactorial() {
    static thread_local std::vector<long double> table;
    return table;
}

inline void combiInit(int n) {
    auto& t = logFactorial();
    if (t.empty()) t.push_back(0.0);
    while (static_cast<int>(t.size()) <= n + 1)
        t.push_back(t.back() + std::log(static_cast<long double>(t.size())));
}

inline long double combLog(int n, int k) {
    assert_(k >= 0 && k <= n, "combLog: 参数越界");
    if (k == 0 || k == n) return 1;
    combiInit(n);
    return std::exp(logFactorial()[n] - logFactorial()[k] - logFactorial()[n - k]);
}

}  // namespace mss
