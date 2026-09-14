#pragma once

#include <cmath>
#include <vector>

#include "core/assert.h"

namespace mss {

// ─────────────────────────────────────────────────────────────
// combinatorics.h — 组合数对数（共享）。
//
// 概率引擎与分布求解共用同一份组合数计算入口：
// thread_local 惰性扩展，数值口径必须全局一致（曾因表分属不同翻译单元/副本
// 导致长尾 ulp 漂移）。调用方保证 0 <= k <= n；越界 = 调用 bug。
// ─────────────────────────────────────────────────────────────

inline std::vector<long double>& logFactorial() {
    // 返回当前线程共享的对数阶乘表。
    static thread_local std::vector<long double> table;
    return table;
}

inline void combiInit(int n) {
    // 表按线程独立增长；切换线程不会共享缓存，也不会改变数值口径。
    auto& t = logFactorial();
    if (t.empty()) t.push_back(0.0);
    while (static_cast<int>(t.size()) <= n + 1)
        t.push_back(t.back() + std::log(static_cast<long double>(t.size())));
}

inline long double combLog(int n, int k) {
    // 返回组合数 C(n,k) 的浮点值；全局概率层用它把 Unknown 的选雷方式
    // 乘到组件 ways 上，因此这里的数值口径必须与分布层一致。
    assert_(k >= 0 && k <= n, "combLog: 参数越界");
    if (k == 0 || k == n) return 1;
    combiInit(n);
    return std::exp(logFactorial()[n] - logFactorial()[k] - logFactorial()[n - k]);
}

}  // namespace mss
