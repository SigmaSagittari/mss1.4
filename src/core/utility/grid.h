#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace mss {

// 二维网格，1-based 坐标（与盘面约定一致）。
// 存储垫一行一列：(rows+1)×(cols+1)、下标 = x*(cols+1)+y —— 与
// ObservedBoard::id 的 CellId 布局一致，CellId 可直接作 Grid 的下标。
template <typename T>
class Grid {
public:
    Grid() = default;

    Grid(int rows, int cols, const T& value) { resize(rows, cols, value); }

    void resize(int rows, int cols, const T& value) {
        rows_ = rows;
        cols_ = cols;
        data_.assign((rows + 1) * (cols + 1), value);
    }

    void fill(const T& value) { std::fill(data_.begin(), data_.end(), value); }

    int rows() const { return rows_; }
    int cols() const { return cols_; }

    bool inBounds(int x, int y) const {
        return x >= 1 && x <= rows_ && y >= 1 && y <= cols_;
    }

    T& at(int x, int y) {
        return data_[index(x, y)];
    }

    const T& at(int x, int y) const {
        return data_[index(x, y)];
    }

    // 支持 grid[i][j]。
    auto operator[](int i) {
        struct Row {
            Grid& grid;
            int row;
            T& operator[](int j) { return grid.at(row, j); }
        };
        return Row{*this, i};
    }

    auto operator[](int i) const {
        struct Row {
            const Grid& grid;
            int row;
            const T& operator[](int j) const { return grid.at(row, j); }
        };
        return Row{*this, i};
    }

private:
    std::size_t index(int x, int y) const {
        return x * (cols_ + 1) + y;
    }

    int rows_ = 0;
    int cols_ = 0;
    std::vector<T> data_;
};

}  // namespace mss
