#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace mss {

class DynamicBitset {
public:
    void resize(std::size_t size) {
        size_ = size;
        words_.resize((size + 63) / 64);
    }

    void setAll() {
        for (std::uint64_t& word : words_) word = ~std::uint64_t{};
        if (size_ % 64 != 0) words_.back() &= tailMask();
    }

    void reset(std::size_t index) {
        words_[index / 64] &= ~(std::uint64_t{1} << (index % 64));
    }

    void set(std::size_t index) {
        words_[index / 64] |= std::uint64_t{1} << (index % 64);
    }

    bool test(std::size_t index) const {
        return ((words_[index / 64] >> (index % 64)) & 1) != 0;
    }

    template <typename Callback>
    void for_each(Callback&& callback) const {
        for (std::size_t wordIndex = 0; wordIndex < words_.size(); ++wordIndex) {
            std::uint64_t word = words_[wordIndex];
            while (word != 0) {
                // word 是局部副本：提取和消去低位不改变 bitset 本身。
                const std::uint64_t lowbit = word & -word;
                callback(wordIndex * 64 + std::countr_zero(lowbit));
                word ^= lowbit;
            }
        }
    }

private:
    std::uint64_t tailMask() const {
        const std::size_t tail = size_ % 64;
        return tail == 0 ? ~std::uint64_t{} : (std::uint64_t{1} << tail) - 1;
    }

    std::vector<std::uint64_t> words_;
    std::size_t size_ = 0;
};

}  // namespace mss
