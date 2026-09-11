#pragma once

#include <cstddef>
#include <span>
#include <vector>

#include "algo/basic.h"
#include "algo/observed_board.h"
#include "algo/shape_solver/shape_solver.h"
#include "algo/structure.h"

namespace mss {

namespace Probability {

    // 将浮点概率尾差收敛到精确的一。
    long double limitProbability(long double probability);

    class Result {
    public:
        struct Component {
            std::span<const long double> boxProbabilities;
        };

        Result() = default;

        Result(const Result&) = delete;
        Result& operator=(const Result&) = delete;
        Result(Result&&) noexcept = default;
        Result& operator=(Result&&) noexcept = default;

        std::span<const Component> components() const { return components_; }
        long double tCellProbability() const { return tCellProbability_; }
        long double candidates() const { return candidates_; }

        long double mineProbability(CellId cell,
                                    const ObservedBoard::Result& board,
                                    const Basic::Result& basic,
                                    const Structure::Result& structure) const;

        template <typename Callback>
        void frontierCells(const ObservedBoard::Result& board,
                           const Structure::Result& structure,
                           Callback&& callback) const;

    private:
        friend void analyze(const ObservedBoard::Result& board,
                            const Basic::Result& basic,
                            const Structure::Result& structure,
                            const Structure::ShapePool& shapes,
                            ShapeSolver::Distribution::Pool& distributions,
                            Result& result);

        void reset(std::span<const std::size_t> componentBoxCounts);

        std::vector<Component> components_;
        std::vector<long double> boxProbabilities_;
        long double tCellProbability_ = 0.0L;
        long double candidates_ = 0.0L;
    };

}  // namespace Probability

}  // namespace mss

template <typename Callback>
inline void mss::Probability::Result::frontierCells(
    const mss::ObservedBoard::Result& board,
    const mss::Structure::Result& structure, Callback&& callback) const {
    for (std::size_t cid = 0; cid < components_.size(); ++cid) {
        const Component& component = components_[cid];
        const Structure::Instance& instance = structure.components[cid];
        for (std::size_t box = 0; box < component.boxProbabilities.size(); ++box) {
            const long double probability = component.boxProbabilities[box];
            for (std::size_t i = instance.boxes.boxOf[box];
                 i < instance.boxes.boxOf[box + 1]; ++i) {
                const auto [x, y] = board.pos(instance.boxes.cells[i]);
                callback(x, y, probability);
            }
        }
    }
}
