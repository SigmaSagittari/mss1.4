#include "ui/interactive.h"

namespace mss::Interactive {

long double mineProbability(GameController::Analysis& an, int x, int y) {
    return an.probability().mineProbability(an.state().id(x, y), an.state(),
                                            an.basicMarks(), an.structure());
}

long double candidates(const GameController::Analysis& an) {
    return an.probability().candidates;
}

long double tCellProbability(const GameController::Analysis& an) {
    return an.probability().tCellProbability;
}

Grid<long double> materializeProbability(GameController::Analysis& an) {
    Grid<long double> grid(an.state().rows, an.state().cols, 0.0L);
    for (int x = 1; x <= an.state().rows; ++x)
        for (int y = 1; y <= an.state().cols; ++y)
            grid[x][y] = mineProbability(an, x, y);
    return grid;
}

Probability::ObserveResult observe(GameController::Analysis& an, int x, int y) {
    const ObservedBoard::Result& state = an.state();
    const auto& basic = an.basicMarks();
    const auto& structure = an.structure();
    return Exact::observe(state, basic, structure, an.probability(), an.dists(),
                          state.id(x, y));
}

}  // namespace mss::Interactive
