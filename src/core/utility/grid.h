#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace mss {

// 0-based 连续二维数组，不带 padding；适合固定行列的算法表。
template <typename T> class RawGrid {
  public:
    RawGrid() = default;

    RawGrid(int rows, int cols, const T &value) {
        resize(rows, cols, value);
    }

    void resize(int rows, int cols, const T &value) {
        rows_ = rows;
        cols_ = cols;
        data_.assign((std::size_t)rows * cols, value);
    }

    void fill(const T &value) {
        std::fill(data_.begin(), data_.end(), value);
    }

    void swap(RawGrid &other) noexcept {
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

    T *operator[](int row) {
        return data_.data() + (std::size_t)row * cols_;
    }

    const T *operator[](int row) const {
        return data_.data() + (std::size_t)row * cols_;
    }

    T &at(int row, int col) {
        return data_[index(row, col)];
    }

    const T &at(int row, int col) const {
        return data_[index(row, col)];
    }

    T *data() {
        return data_.data();
    }

    const T *data() const {
        return data_.data();
    }

  private:
    std::size_t index(int row, int col) const {
        return (std::size_t)row * cols_ + col;
    }

    int rows_ = 0;
    int cols_ = 0;
    std::vector<T> data_;
};

// 二维网格，1-based 坐标（与盘面约定一致）。
// 存储垫一行一列：(rows+1)×(cols+1)、下标 = x*(cols+1)+y —— 与
// ObservedBoard::id 的 CellId 布局一致，CellId 可直接作 Grid 的下标。
template <typename T> class Grid {
  public:
    struct Row {
        Grid &grid;
        int row;
        T &operator[](int j) {
            return grid.at(row, j);
        }
    };

    struct ConstRow {
        const Grid &grid;
        int row;
        const T &operator[](int j) const {
            return grid.at(row, j);
        }
    };

    // 创建空网格，尺寸为 0×0。
    Grid() = default;

    // rows/cols 是真实盘面尺寸；内部额外分配一行一列 padding。
    Grid(int rows, int cols, const T &value) {
        resize(rows, cols, value);
    }

    // resize 会丢弃旧内容，并把真实区与 padding 全部填成 value。
    void resize(int rows, int cols, const T &value) {
        raw_.resize(rows + 1, cols + 1, value);
    }

    // 用同一值填充真实区域和 padding。
    void fill(const T &value) {
        raw_.fill(value);
    }

    // 返回真实行数。
    int rows() const {
        return raw_.rows() == 0 ? 0 : raw_.rows() - 1;
    }
    // 返回真实列数。
    int cols() const {
        return raw_.cols() == 0 ? 0 : raw_.cols() - 1;
    }

    // 判断坐标是否落在真实 1-based 网格内。
    bool inBounds(int x, int y) const {
        return x >= 1 && x <= rows() && y >= 1 && y <= cols();
    }

    // at/operator[] 只接受真实的 1..rows、1..cols 坐标；padding 只为 CellId
    // 线性布局和邻居访问留出空间，不代表可参与分析的格子。
    T &at(int x, int y) {
        // 返回指定真实坐标的可写元素。
        return raw_.data()[index(x, y)];
    }

    const T &at(int x, int y) const {
        // 返回指定真实坐标的只读元素。
        return raw_.data()[index(x, y)];
    }

    // 支持 grid[i][j]。
    Row operator[](int i) {
        // 返回一行代理，使调用方可以使用 grid[i][j]。
        return Row{*this, i};
    }

    ConstRow operator[](int i) const {
        // 返回只读行代理，使 const 网格仍支持 grid[i][j]。
        return ConstRow{*this, i};
    }

  private:
    std::size_t index(int x, int y) const {
        // 将 1-based 二维坐标映射到带 padding 的一维存储下标。
        return (std::size_t)x * raw_.cols() + y;
    }

    RawGrid<T> raw_;
};

} // namespace mss
