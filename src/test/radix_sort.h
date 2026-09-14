#pragma once

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <vector>

#include "core/utility/radix_sort.h"
#include "test/common.h"

namespace test {

// 同时验证小数组比较排序和大数组 LSD 路径的稳定多字段顺序。
inline void radixSort() {
    struct Entry {
        std::uint32_t key;
        std::uint32_t order;
    };
    auto checkSortedStable = [](const std::vector<Entry>& entries) {
        for (std::size_t i = 1; i < entries.size(); ++i)
            check(entries[i - 1].key < entries[i].key ||
                      (entries[i - 1].key == entries[i].key &&
                       entries[i - 1].order <= entries[i].order),
                  "test/radix_sort: unstable or unsorted result");
    };
    auto runSmallTest = [&] {
        std::vector<Entry> entries = {
            {2, 0}, {1, 1}, {2, 2}, {0, 3}, {1, 4}, {2, 5}};
        std::vector<Entry> tmp;
        mss::radix_sort::sort(entries, tmp,
                              [](const Entry& entry) { return entry.key; });
        checkSortedStable(entries);
    };
    auto runLargeTest = [&] {
        std::vector<Entry> entries;
        for (std::uint32_t i = 0; i < 1024; ++i)
            entries.push_back({(i * 37) % 11, i});
        std::vector<Entry> tmp;
        mss::radix_sort::sort(entries, tmp,
                              [](const Entry& entry) { return entry.key; });
        checkSortedStable(entries);
    };
    runSmallTest();
    runLargeTest();
    std::cout << "test/radix_sort: stable small and large paths passed\n";
}

}  // namespace test
