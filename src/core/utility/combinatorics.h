#pragma once

#include <algorithm>
#include <vector>

#include "core/assert.h"

namespace mss {

// ─────────────────────────────────────────────────────────────
// combinatorics.h — 组合数（共享）。
//
// 概率引擎与分布求解共用同一份组合数计算入口：
// thread_local 惰性扩展，数值口径必须全局一致（曾因表分属不同翻译单元/副本
// 导致长尾 ulp 漂移）。调用方保证 0 <= k <= n；越界 = 调用 bug。
// ─────────────────────────────────────────────────────────────

struct CombinationCache {
    int n = -1;
    std::vector<long double> values;
};

inline CombinationCache& combinationCache() {
    static thread_local CombinationCache cache;
    return cache;
}

inline long double combLog(int n, int k) {
    // 返回组合数 C(n,k) 的浮点值；全局概率层用它把 Unknown 的选雷方式
    // 乘到组件 ways 上，因此这里的数值口径必须与分布层一致。
    assert_(k >= 0 && k <= n, "combLog: 参数越界");
    k = (std::min)(k, n - k);
    CombinationCache& cache = combinationCache();
    if (cache.n != n) {
        cache.n = n;
        cache.values.assign(1, 1.0L);
    }
    while (static_cast<int>(cache.values.size()) <= k) {
        const int i = static_cast<int>(cache.values.size());
        cache.values.push_back(
            cache.values.back() * static_cast<long double>(n - i + 1) /
            static_cast<long double>(i));
    }
    return cache.values[k];
}

}  // namespace mss
