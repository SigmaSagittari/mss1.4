#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace mss {

// 通用 LSD 基数排序：语义命名空间（纯空壳，无状态）。
// 排序键由调用方以"无符号整数字段访问器"给出（主序在前）；排序器按字段
// 字节直接扫，不构造中间关键码。残局搜索用它按"观测向量哈希"分组——
// entry 小、量大，比 std::sort 快。
//
// 稳定性（重要）：两条路径都稳定。小数组分支使用 std::stable_sort，基数路径
// 使用稳定的 LSD 计数排序；等键元素保持输入顺序。
struct radix_sort {
    // 条目数小于该值时用比较排序，省掉基数排序的固定开销
    static constexpr std::uint32_t kThreshold = 256;

private:
    template <typename Key>
    static unsigned char byteAt(const Key& key, std::size_t bi) {
        if constexpr (std::is_unsigned_v<Key>) {
            return static_cast<unsigned char>(key >> (bi * 8));
        } else if constexpr (requires { key.lo; key.hi; }) {
            if (bi < sizeof(key.lo)) return static_cast<unsigned char>(key.lo >> (bi * 8));
            return static_cast<unsigned char>(key.hi >> ((bi - sizeof(key.lo)) * 8));
        } else {
            static_assert(std::is_unsigned_v<Key>, "radix_sort::sortBy 的键必须是无符号整数或具有 lo/hi 的定宽键");
        }
    }

    template <typename Key>
    static constexpr std::size_t keyBytes() {
        if constexpr (std::is_unsigned_v<Key>) return sizeof(Key);
        else if constexpr (requires(const Key& key) { key.lo; key.hi; })
            return sizeof(std::declval<Key>().lo) + sizeof(std::declval<Key>().hi);
        else return 0;
    }

    template <typename Key>
    static bool lessKey(const Key& lhs, const Key& rhs) {
        if constexpr (std::is_unsigned_v<Key>) {
            return lhs < rhs;
        } else if constexpr (requires { lhs.lo; lhs.hi; }) {
            return lhs.hi < rhs.hi || (lhs.hi == rhs.hi && lhs.lo < rhs.lo);
        } else {
            static_assert(std::is_unsigned_v<Key>, "radix_sort::sortBy 的键必须是无符号整数或具有 lo/hi 的定宽键");
        }
    }

public:
    // 按逻辑下标排序。Reader 读取键，Swapper 描述当前存储上的原地交换；
    // 排序器不接触元素的实际布局。
    template <typename Reader, typename Swapper>
    static void sortBy(std::size_t n, Reader read, Swapper swap) {
        using Key = std::remove_cvref_t<decltype(read(std::size_t{}))>;
        static_assert(keyBytes<Key>() != 0, "radix_sort::sortBy 不支持此键类型");
        if (n <= 1) return;

        static thread_local std::vector<std::size_t> target;
        target.resize(n);

        if (n <= kThreshold) {
            std::array<std::size_t, kThreshold> order;
            constexpr std::size_t marker = std::size_t(1) << (sizeof(std::size_t) * 8 - 1);
            constexpr std::size_t indexMask = ~marker;
            for (std::size_t i = 0; i < n; ++i) order[i] = i;
            std::sort(order.begin(), order.begin() + n, [&](std::size_t lhs, std::size_t rhs) {
                const Key lhsKey = read(lhs);
                const Key rhsKey = read(rhs);
                if (lessKey(lhsKey, rhsKey)) return true;
                if (lessKey(rhsKey, lhsKey)) return false;
                return lhs < rhs;
            });
            for (std::size_t i = 0; i < n; ++i) {
                if (order[i] & marker) continue;
                std::size_t source = order[i];
                order[i] |= marker;
                if (source == i) continue;
                swap(i, source);
                while (source != i) {
                    const std::size_t nextSource = order[source] & indexMask;
                    order[source] |= marker;
                    if (nextSource == i) break;
                    swap(source, nextSource);
                    source = nextSource;
                }
            }
            return;
        }

        // 每趟只保存“当前逻辑位置 -> 稳定目标逻辑位置”的置换。
        // 置换应用阶段必须用 swap；对同一份存储直接 write 会破坏非平凡环。
        std::array<std::size_t, 256> next;
        for (std::size_t bi = 0; bi < keyBytes<Key>(); ++bi) {
            std::array<std::size_t, 256> count{};
            for (std::size_t i = 0; i < n; ++i) ++count[byteAt(read(i), bi)];
            std::size_t offset = 0;
            for (std::size_t b = 0; b < count.size(); ++b) {
                next[b] = offset;
                offset += count[b];
            }
            for (std::size_t i = 0; i < n; ++i)
                target[i] = next[byteAt(read(i), bi)]++;
            constexpr std::size_t marker = std::size_t(1) << (sizeof(std::size_t) * 8 - 1);
            constexpr std::size_t indexMask = ~marker;
            for (std::size_t i = 0; i < n; ++i) {
                if (target[i] & marker) continue;
                std::size_t k = target[i];
                target[i] |= marker;
                if (k == i) continue;
                while (k != i) {
                    const std::size_t nextK = target[k] & indexMask;
                    target[k] |= marker;
                    swap(i, k);
                    k = nextK;
                }
            }
        }
    }

    // 按访问器给出的字段升序排序；tmp 为调用方持有、复用的输出缓冲。
    // 小数组走 std::stable_sort，保持与基数路径一致的稳定语义。
    template <typename Entry, typename... Accessor>
    static void sort(std::vector<Entry>& a, std::vector<Entry>& tmp, Accessor... accessor) {
        const std::uint32_t n = static_cast<std::uint32_t>(a.size());
        if (n <= kThreshold) {  // 小数组：比较排序更快
            std::stable_sort(a.begin(), a.end(), [&](const Entry& x, const Entry& y) {
                return std::make_tuple(accessor(x)...) < std::make_tuple(accessor(y)...);
            });
            return;
        }

        tmp.resize(a.size());
        static thread_local std::array<std::uint32_t, 256> bucket;
        Entry* src = a.data();
        Entry* dst = tmp.data();

        // 按某个字段的某个字节做一趟稳定的计数排序：计数 -> 前缀起始下标 -> 正序散射
        auto passByte = [&](auto get, unsigned int bi) {
            for (auto& b : bucket) b = 0;
            for (std::uint32_t i = 0; i < n; ++i)
                ++bucket[static_cast<unsigned char>(get(src[i]) >> (8 * bi))];
            std::uint32_t sum = 0;
            for (auto& b : bucket) {
                const std::uint32_t v = b;
                b = sum;
                sum += v;
            }
            for (std::uint32_t i = 0; i < n; ++i) {
                const unsigned char byte = static_cast<unsigned char>(get(src[i]) >> (8 * bi));
                dst[bucket[byte]++] = src[i];
            }
            std::swap(src, dst);
        };

        // 一个字段的全部字节：低字节先排
        auto passField = [&](auto get) {
            using V = std::remove_reference_t<decltype(get(std::declval<Entry&>()))>;
            static_assert(std::is_unsigned_v<V>, "radix_sort 的排序字段必须是无符号整型");
            for (unsigned int bi = 0; bi < sizeof(V); ++bi) passByte(get, bi);
        };

        // LSD：主序字段最后排 —— 反序遍历访问器（最不重要的先排）
        auto gets = std::tuple(accessor...);
        [&]<std::size_t... I>(std::index_sequence<I...>) {
            constexpr std::size_t N = sizeof...(Accessor);
            (passField(std::get<N - 1 - I>(gets)), ...);
        }(std::make_index_sequence<sizeof...(Accessor)>{});

        // 总趟数奇偶不预设：结果不在 a 里就整体换回
        if (src != a.data()) a.swap(tmp);
    }
};

}  // namespace mss
