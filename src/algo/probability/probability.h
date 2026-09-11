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
    inline long double limitProbability(long double probability) {
        return probability >= 1.0L - 1e-10L ? 1.0L : probability;
    }

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

//==============================================================================
namespace mss::Probability {

inline void Result::reset(std::span<const std::size_t> componentBoxCounts) {
    components_.reserve(componentBoxCounts.size());
    components_.resize(componentBoxCounts.size());
    std::size_t totalBoxCount = 0;
    for (const std::size_t count : componentBoxCounts) totalBoxCount += count;
    boxProbabilities_.resize(totalBoxCount);
    const std::span<const long double> allProbabilities = boxProbabilities_;
    std::size_t offset = 0;
    for (std::size_t i = 0; i < componentBoxCounts.size(); ++i) {
        const std::size_t count = componentBoxCounts[i];
        components_[i].boxProbabilities = allProbabilities.subspan(offset, count);
        offset += count;
    }
}

inline long double Result::mineProbability(
    CellId cell, const ObservedBoard::Result& board, const Basic::Result& basic,
    const Structure::Result& structure) const {
    const auto [x, y] = board.pos(cell);
    const CellLocation loc = structure.cellLoc[cell];
    if (loc.component == -1) {
        if (basic.marks[x][y] == Basic::Mark::Mine) return 1.0L;
        if (basic.marks[x][y] == Basic::Mark::Unknown)
            return limitProbability(tCellProbability_);
        return 0.0L;
    }
    if (loc.box == -1) return 0.0L;
    const long double probability =
        components_[loc.component].boxProbabilities[loc.box];
    return limitProbability(probability);
}

}  // namespace mss::Probability

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
