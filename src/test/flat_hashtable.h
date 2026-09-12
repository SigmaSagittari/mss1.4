#pragma once

#include <cstdint>
#include <iostream>
#include <limits>
#include <map>
#include <random>
#include <set>

#include "core/utility/flat_hashtable.h"
#include "test/common.h"

namespace test {

inline void flatHashtable() {
    auto runTableChecks = []<typename Table>(Table& table) {
    std::map<std::uint64_t, std::uint64_t> map;
    std::set<std::uint64_t> set;
    constexpr std::uint64_t kGrowthKeys = 4096;
    for (std::uint64_t key = 0; key < kGrowthKeys; ++key) {
        table.emplace(key, key ^ 0x9e3779b97f4a7c15ULL);
        map.emplace(key, key ^ 0x9e3779b97f4a7c15ULL);
        set.insert(key);
    }
    const std::uint64_t fixedKeys[] = {
        0, 1, (std::numeric_limits<std::uint64_t>::max)(),
        0x8000000000000000ULL, 0xffff00000000ffffULL};
    for (std::uint64_t key : fixedKeys) {
        table[key] = key ^ 0x9e3779b97f4a7c15ULL;
        map[key] = key ^ 0x9e3779b97f4a7c15ULL;
        set.insert(key);
    }
    std::mt19937_64 rng(0xc0ffee12345ULL);
    constexpr int kOperations = 200000;
    for (int i = 0; i < kOperations; ++i) {
        const std::uint64_t key = rng();
        const std::uint64_t value = rng();
        switch (rng() & 3ULL) {
        case 0:
            table[key] = value;
            map[key] = value;
            set.insert(key);
            break;
        case 1:
            table.emplace(key, value);
            map.emplace(key, value);
            set.insert(key);
            break;
        case 2: {
            const auto* got = table.find(key);
            const auto ref = map.find(key);
            check((got != nullptr) == (ref != map.end()), "find presence mismatch");
            if (got != nullptr) check(*got == ref->second, "find value mismatch");
            break;
        }
        case 3:
            check(table.size() == map.size(), "size mismatch");
            check(table.empty() == map.empty(), "empty mismatch");
            check(set.size() == map.size(), "set size mismatch");
            break;
        }
    }
    check(table.size() == map.size(), "final size mismatch");
    check(set.size() == map.size(), "final set size mismatch");
    for (const auto& [key, value] : map) {
        const auto* got = table.find(key);
        check(got != nullptr, "stored key missing");
        check(*got == value, "stored value mismatch");
        check(set.find(key) != set.end(), "set key missing");
    }
    const auto& constTable = table;
    for (const auto& [key, value] : map) {
        const auto* got = constTable.find(key);
        check(got != nullptr && *got == value, "const find mismatch");
    }
    table.clear();
    map.clear();
    set.clear();
    check(table.empty(), "clear did not empty table");
    check(table.size() == map.size(), "clear size mismatch");
    };
    mss::FlatHashTable<std::uint64_t, std::uint64_t> defaultHash;
    runTableChecks(defaultHash);
    mss::FlatHashTable<std::uint64_t, std::uint64_t, mss::SplitMix64Hash> mixedHash;
    runTableChecks(mixedHash);
    std::cout << "test/flat_hashtable: growth, default and explicit SplitMix64Hash passed\n";
}

}  // namespace test
