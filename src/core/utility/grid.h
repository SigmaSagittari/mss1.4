#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace mss {

// 0-based 二维网格，无 padding：下标 x ∈ [0, rows)、y ∈ [0, cols)。
// 存储是行连续的，线性下标 = x * cols + y —— 与 ObservedBoard::CellId 同一布局，
// 因此 CellId 可以直接当它的一维下标用。
// 不做任何边界检查（热路径语义）：越界是调用方 bug。
template <typename T> class Grid {
  public:
    Grid() = default;

    Grid(int rows, int cols, const T &value) {
        resize(rows, cols, value);
    }

    // resize 丢弃旧内容，把全部元素填成 value。
    void resize(int rows, int cols, const T &value) {
        rows_ = rows;
        cols_ = cols;
        data_.assign(static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols), value);
    }

    void fill(const T &value) {
        std::fill(data_.begin(), data_.end(), value);
    }

    void swap(Grid &other) noexcept {
        std::swap(rows_, other.rows_);
        std::swap(cols_, other.cols_);
        data_.swap(other.data_);
    }

    int rows() const {
        return rows_;
    }

    int cols() const {
        return cols_;
    }

    bool inBounds(int x, int y) const {
        return x >= 0 && x < rows_ && y >= 0 && y < cols_;
    }

    // grid[x][y]：返回整行指针，便于热循环里做指针算术。
    T *operator[](int x) {
        return data_.data() + static_cast<std::size_t>(x) * static_cast<std::size_t>(cols_);
    }

    const T *operator[](int x) const {
        return data_.data() + static_cast<std::size_t>(x) * static_cast<std::size_t>(cols_);
    }

    T &at(int x, int y) {
        return data_[index(x, y)];
    }

    const T &at(int x, int y) const {
        return data_[index(x, y)];
    }

    T *data() {
        return data_.data();
    }

    const T *data() const {
        return data_.data();
    }

  private:
    std::size_t index(int x, int y) const {
        return static_cast<std::size_t>(x) * static_cast<std::size_t>(cols_) + static_cast<std::size_t>(y);
    }

    int rows_ = 0;
    int cols_ = 0;
    std::vector<T> data_;
};

} // namespace mss