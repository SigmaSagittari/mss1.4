#include "algo/probability/probability.h"

namespace mss {

namespace Probability {

long double limitProbability(long double probability) {
    return probability >= 1.0L - 1e-10L ? 1.0L : probability;
}

void Result::reset(std::span<const std::size_t> componentBoxCounts) {
    components_.reserve(componentBoxCounts.size());
    components_.resize(componentBoxCounts.size());
    std::size_t totalBoxCount = 0;
    for (const std::size_t count : componentBoxCounts)
        totalBoxCount += count;
    boxProbabilities_.resize(totalBoxCount);
    const std::span<const long double> allProbabilities = boxProbabilities_;
    std::size_t offset = 0;
    for (std::size_t i = 0; i < componentBoxCounts.size(); ++i) {
        const std::size_t count = componentBoxCounts[i];
        components_[i].boxProbabilities =
            allProbabilities.subspan(offset, count);
        offset += count;
    }
}

long double Result::mineProbability(
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

}  // namespace Probability

}  // namespace mss
