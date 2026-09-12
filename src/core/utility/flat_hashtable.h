#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>
#include <vector>

namespace mss {

// 适合二次幂容量和低位取桶的 64 位混合哈希。需要稳定的整数性能时显式传入。
struct SplitMix64Hash {
    std::size_t operator()(std::uint64_t x) const noexcept {
        x += 0x9e3779b97f4a7c15ULL;
        x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
        x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
        return x ^ (x >> 31);
    }
};

// 开放寻址、线性探测；只支持插入/查找，不支持删除。
// 键值连续存储，容量为 2 的幂，clear() 保留 capacity。
// Hash 必须提供 const 的 operator()(const Key&)；默认使用 SplitMix64Hash。
template <typename Key, typename Value, typename Hash = SplitMix64Hash>
class FlatHashTable {
public:
    static constexpr double kMaxLoadFactor = 0.5;

    static_assert(std::is_invocable_r_v<std::size_t, const Hash&, const Key&>, "Hash must be callable as const Hash&(const Key&)");

    std::size_t size() const { return size_; }
    bool empty() const { return size_ == 0; }

    void clear() { activeCapacity_ = 0; size_ = 0; }

    // 预留容量，使负载因子不超过 kMaxLoadFactor
    void reserve(std::size_t expected) {
        const auto need = nextPowerOfTwo(expected > 0 ? expected * 2 : 1);
        if (need <= activeCapacity_) return;
        if (activeCapacity_ == 0) {
            activate(need);
            return;
        }
        rehash(need);
    }

    // 查找：命中返回指向值的指针，未命中返回 nullptr
    Value* find(const Key& key) { return valueAt(values_, findIndex(key)); }
    const Value* find(const Key& key) const { return valueAt(values_, findIndex(key)); }

    // 查找，不存在则插入默认值并返回引用（与 std::unordered_map::operator[] 一致）
    Value& operator[](const Key& key) {
        const auto i = insertionIndex(key);
        if (insertKey(i, key)) values_[i] = Value{};
        return values_[i];
    }

    // 仅当键不存在时插入（与 std::unordered_map::emplace 一致）
    void emplace(const Key& key, const Value& value) {
        const auto i = insertionIndex(key);
        if (insertKey(i, key)) values_[i] = value;
    }

private:
    std::size_t capacity() const { return activeCapacity_; }

    void activate(std::size_t newCap) {
        if (newCap > keys_.size()) {
            keys_.resize(newCap);
            values_.resize(newCap);
            used_.resize(newCap, 0);
        }
        for (std::size_t i = 0; i < newCap; ++i) used_[i] = 0;
        activeCapacity_ = newCap;
    }

    template <typename Values>
    auto valueAt(Values& values, std::size_t i) const -> decltype(&values[i]) {
        return i == activeCapacity_ ? nullptr : &values[i];
    }

    bool insertKey(std::size_t i, const Key& key) {
        if (used_[i]) return false;
        used_[i] = 1; keys_[i] = key; ++size_;
        return true;
    }

    // capacity() 是未命中哨兵；实际槽位范围为 [0, capacity())。
    std::size_t findIndex(const Key& key) const {
        if (size_ == 0) return capacity();
        const auto mask = activeCapacity_ - 1;
        auto i = hash_(key) & mask;
        for (;;) {
            if (!used_[i]) return activeCapacity_;
            if (keys_[i] == key) return i;
            i = (i + 1) & mask;
        }
    }

    std::size_t insertionIndex(const Key& key) {
        if (size_ + 1 > activeCapacity_ * kMaxLoadFactor) grow();
        const auto mask = activeCapacity_ - 1;
        auto i = hash_(key) & mask;
        for (;;) {
            if (!used_[i] || keys_[i] == key) return i;
            i = (i + 1) & mask;
        }
    }

    static std::size_t nextPowerOfTwo(std::size_t n) {
        auto p = std::size_t{1};
        while (p < n) p <<= 1;
        return p;
    }

    void grow() {
        if (activeCapacity_ == 0)
            activate(keys_.empty() ? 4 : keys_.size());
        else rehash(activeCapacity_ * 2);
    }

    void rehash(std::size_t newCap) {
        std::vector<Key> newKeys(newCap);
        std::vector<Value> newValues(newCap);
        std::vector<unsigned char> newUsed(newCap, 0);
        const auto mask = newCap - 1;
        for (auto i = std::size_t{}; i < activeCapacity_; ++i) {
            if (!used_[i]) continue;
            auto j = hash_(keys_[i]) & mask;
            while (newUsed[j]) j = (j + 1) & mask;
            newUsed[j] = 1;
            newKeys[j] = keys_[i];
            newValues[j] = values_[i];
        }
        keys_ = std::move(newKeys);
        values_ = std::move(newValues);
        used_ = std::move(newUsed);
        activeCapacity_ = newCap;
    }

    Hash hash_;
    std::vector<Key> keys_;
    std::vector<Value> values_;
    std::vector<unsigned char> used_;
    std::size_t size_ = 0;
    std::size_t activeCapacity_ = 0;
};

}  // namespace mss
