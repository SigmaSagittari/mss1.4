#pragma once

#include <algorithm>
#include <array>
#include <vector>

#include "core/assert.h"
#include "core/workspace.h"

namespace mss {

// ─────────────────────────────────────────────────────────────
// combinatorics.h — 组合数（共享）。
//
// 组合数缓存按 Pascal 三角形平铺；已经计算过的 n 永久保留，避免反复重算。
// 调用方保证 0 <= k <= n。
// ─────────────────────────────────────────────────────────────

inline long double binomSmall(int n, int k) {
    constexpr int max = 9;
    static constexpr std::array<std::array<long double, max + 1>, max + 1> table = [] {
        std::array<std::array<long double, max + 1>, max + 1> result{};
        for (int i = 0; i <= max; ++i) {
            result[i][0] = 1;
            result[i][i] = 1;
            for (int j = 1; j < i; ++j)
                result[i][j] = result[i - 1][j - 1] + result[i - 1][j];
        }
        return result;
    }();
    return table[n][k];
}

inline long double binom(int n, int k) {
    // 返回组合数 C(n,k) 的浮点值。
    assert_(k >= 0 && k <= n, "binom: 参数越界");
    auto &cache = workspace::Combinatorics::cache;
    if (cache.n < n) {
        const int first = (std::max)(cache.n + 1, 0);
        cache.values.resize((std::size_t)(n + 1) * (n + 2) / 2);
        for (int row = first; row <= n; ++row) {
            const std::size_t offset = (std::size_t)(row) * (row + 1) / 2;
            cache.values[offset] = 1.0L;
            cache.values[offset + row] = 1.0L;
            for (int column = 1; column < row; ++column)
                cache.values[offset + column] = cache.values[offset - row + column - 1] + cache.values[offset - row + column];
        }
        cache.n = n;
    }
    return cache.values[(std::size_t)(n) * (n + 1) / 2 + k];
}

} // namespace mss
