#pragma once

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <vector>

#include "core/utility/flat_hashtable.h"
#include "third_party/libcuckoo/cuckoohash_map.hh"
#include "test/common.h"

namespace test {

struct HashTableBenchmarkOperation {
    std::uint64_t key;
    std::uint64_t value;
    std::uint64_t kind;
};

struct HashTableBenchmarkResult {
    double milliseconds;
    std::size_t size;
    std::uint64_t checksum;
};

inline std::vector<HashTableBenchmarkOperation>
makeHashTableBenchmarkOperations() {
    std::vector<HashTableBenchmarkOperation> operations;
    constexpr std::uint64_t kGrowthKeys = 4096;
    for (std::uint64_t key = 0; key < kGrowthKeys; ++key)
        operations.push_back({key, key ^ 0x9e3779b97f4a7c15ULL, 1});
    const std::uint64_t fixedKeys[] = {
        0, 1, (std::numeric_limits<std::uint64_t>::max)(),
        0x8000000000000000ULL, 0xffff00000000ffffULL};
    for (std::uint64_t key : fixedKeys)
        operations.push_back({key, key ^ 0x9e3779b97f4a7c15ULL, 0});
    std::mt19937_64 rng(0xc0ffee12345ULL);
    constexpr int kOperations = 200000;
    for (int i = 0; i < kOperations; ++i)
        operations.push_back({rng(), rng(), rng() & 3ULL});
    return operations;
}

template <typename Table, typename Assign, typename Insert, typename Find>
inline HashTableBenchmarkResult runHashTableBenchmark(
    const std::vector<HashTableBenchmarkOperation>& operations, Table& table,
    Assign assign, Insert insert, Find find) {
    std::uint64_t checksum = 0;
    const auto started = std::chrono::steady_clock::now();
    for (const auto& operation : operations) {
        switch (operation.kind) {
        case 0:
            assign(table, operation.key, operation.value);
            checksum ^= operation.value;
            break;
        case 1:
            insert(table, operation.key, operation.value);
            checksum += operation.key;
            break;
        case 2:
        case 3: {
            std::uint64_t value = 0;
            if (find(table, operation.key, value))
                checksum ^= value + operation.key;
            break;
        }
        default:
            std::abort();
        }
    }
    const double milliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    return {milliseconds, table.size(), checksum};
}

inline void hashTablePerformance() {
    const auto operations = makeHashTableBenchmarkOperations();

    mss::FlatHashTable<std::uint64_t, std::uint64_t> flat;
    const auto flatResult = runHashTableBenchmark(
        operations, flat,
        [](auto& table, std::uint64_t key, std::uint64_t value) {
            table[key] = value;
        },
        [](auto& table, std::uint64_t key, std::uint64_t value) {
            table.emplace(key, value);
        },
        [](auto& table, std::uint64_t key, std::uint64_t& value) {
            const auto* found = table.find(key);
            if (found == nullptr) return false;
            value = *found;
            return true;
        });

    libcuckoo::cuckoohash_map<std::uint64_t, std::uint64_t,
                              mss::SplitMix64Hash>
        cuckoo;
    const auto cuckooResult = runHashTableBenchmark(
        operations, cuckoo,
        [](auto& table, std::uint64_t key, std::uint64_t value) {
            table.insert_or_assign(key, value);
        },
        [](auto& table, std::uint64_t key, std::uint64_t value) {
            table.insert(key, value);
        },
        [](auto& table, std::uint64_t key, std::uint64_t& value) {
            return table.find(key, value);
        });

    check(flatResult.size == cuckooResult.size,
          "flat and libcuckoo final sizes differ");
    check(flatResult.checksum == cuckooResult.checksum,
          "flat and libcuckoo benchmark checksums differ");
    std::cout << "test/hash_table_performance: flat_ms=" << std::fixed
              << std::setprecision(3) << flatResult.milliseconds
              << " libcuckoo_ms=" << cuckooResult.milliseconds
              << " ratio=" << cuckooResult.milliseconds / flatResult.milliseconds
              << "x size=" << flatResult.size << '\n';
}

// 覆盖开放寻址表的首次插入、命中、扩容和 clear 后复用。
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
    hashTablePerformance();
}

}  // namespace test
