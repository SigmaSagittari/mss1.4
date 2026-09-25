#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <span>
#include <utility>
#include <vector>

#include "core/types.h"
#include "core/utility/flat_hashtable.h"
#include "core/utility/grid.h"
#include "core/utility/hash.h"

namespace mss::workspace {

// src/algo/probability_engine/basic.h
struct Basic {
    // Basic::update
    struct Update {
        std::vector<CellId> pending;
        std::vector<unsigned char> queued;
    };

    inline static thread_local Update update;
};

// src/algo/probability_engine/structure.h
struct Structure {
    // Structure::analyze
    struct Analyze {
        Grid<char> visited;
        Grid<U128> cellHash;
        std::vector<CellId> cells;
    };

    // Structure::update
    struct Update {
        Grid<char> visited;
        Grid<U128> cellHash;
        std::vector<CellId> cells;
        Grid<char> dirty;
        std::vector<CellId> dirtyCells;
        std::vector<char> removed;
        std::vector<InstanceId> staged;
    };

    struct Buffers {
        using Analyze = Structure::Analyze;
        using Update = Structure::Update;

        Analyze analyze;
        Update update;

        FlatHashTable<U128, BoxId, U128Hash> hashBox;
        std::vector<BoxId> boxOfCells;
        std::vector<std::array<CellId, 9>> buckets;
        std::vector<std::uint8_t> bucketSize;
        std::vector<char> boxUsed;
        std::vector<BoxId> allBoxIds;
    };

    inline static thread_local Buffers workspace;
};

// src/core/utility/combinatorics.h
struct Combinatorics {
    // binom
    struct Cache {
        int n = -1;
        std::vector<long double> values;
    };

    static thread_local Cache cache;
};
inline thread_local Combinatorics::Cache Combinatorics::cache;

// src/core/utility/radix_sort.h
struct RadixSort {
    // RadixSort::sortBy
    struct SortByBuffers {
        std::vector<std::size_t> target;
    };

    template <typename Reader, typename Swapper>
    inline static thread_local SortByBuffers sortBy;

    // RadixSort::sort
    struct SortBuffers {
        std::array<std::uint32_t, 256> bucket;
    };

    template <typename Entry, typename... Accessor>
    inline static thread_local SortBuffers sort;
};

// src/algo/probability_engine/shape_solver/dfs_solver.h
struct DfsSolver {
    // DfsSolver::forEachAssignment
    struct ForEachAssignment {
        struct Frame {
            int index = 0;
            int nextMine = 0;
            int appliedIndex = -1;
            int appliedMine = 0;
            long double ways = 0;
        };

        std::vector<int> boxHead;
        std::vector<int> constraintNext;
        std::vector<int> constraintIds;
        std::vector<int> constraintSum;
        std::vector<int> constraintMaxAdd;
        std::vector<int> currentSum;
        std::vector<int> assignedSize;
        std::vector<char> assignment;
        std::vector<Frame> frames;
    };

    inline static thread_local ForEachAssignment forEachAssignment;

    // DfsSolver::analyze
    struct Analyze {
        std::vector<long double> ways;
        RawGrid<long double> moments;
    };

    inline static thread_local Analyze analyze;
};

// src/algo/probability_engine/shape_solver/graph_solver/graph_solver_order.h
struct GraphSolverOrder {
    // GraphSolver::orderScore
    struct OrderScore {
        std::vector<int> remaining;
        std::vector<char> selected;
    };

    inline static thread_local OrderScore orderScore;

    // GraphSolver::makeSAOrder
    struct MakeSAOrder {
        std::vector<BoxId> current;
        std::vector<BoxId> candidate;
    };

    inline static thread_local MakeSAOrder makeSAOrder;
};

// src/algo/probability_engine/shape_solver/graph_solver/graph_solver_dp.h
struct GraphSolverDp {
    // GraphSolver::analyze
    struct Layer {
        struct Count {
            int mineCount = 0;
            long double ways = 0;
            std::size_t momentOffset = 0;
            int next = -1;
        };

        struct State {
            std::size_t frontierOffset = 0;
            int firstCount = -1;
            int lastCount = -1;
        };

        struct MomentValue {
            long double value;

            MomentValue() {
            }
        };

        std::vector<State> states;
        std::vector<Count> counts;
        std::vector<BoxId> momentBoxes;
        std::vector<MomentValue> momentValues;
        std::vector<std::uint64_t> frontierWords;
        FlatHashTable<U128, std::size_t, U128Hash> index;

        std::uint64_t frontierValue(const State &state, int slot) const {
            const std::uint64_t word = frontierWords[state.frontierOffset + slot / 16];
            return (word >> ((slot & 15) * 4)) & 0xf;
        }

        void reset() {
            states.clear();
            counts.clear();
            momentBoxes.clear();
            momentValues.clear();
            frontierWords.clear();
            index.clear();
            states.push_back({0, 0, 0});
            counts.push_back({0, 1.0L, 0, -1});
        }

        template <typename Plan> void advance(const Plan &plan, Layer &nextLayer) const;
    };

    inline static thread_local Layer current;
    inline static thread_local Layer next;
};

// src/algo/probability_engine/probability/probability.h
struct Probability {
    // Probability::analyze
    struct Analyze {
        struct Poly {
            int start = 0;
            std::span<const long double> coeffs;

            std::span<const long double> coefficients() const {
                return coeffs;
            }
        };

        struct TreePoly {
            int start = 0;
            std::vector<long double> coeffs;
            std::span<const long double> view;

            std::span<const long double> coefficients() const {
                return view.empty() ? std::span<const long double>(coeffs) : view;
            }

            Poly asPoly() const {
                return {start, coefficients()};
            }

            void setView(Poly poly) {
                start = poly.start;
                coeffs.clear();
                view = poly.coefficients();
            }
        };

        struct Buffers {
            std::array<long double, 1> identity{1.0L};
            std::vector<DistributionId> distributions;
            std::vector<std::size_t> componentBoxCounts;
            std::vector<Poly> factors;
            std::vector<long double> distributionProbabilities;
            std::vector<std::span<const long double>> distributionProbabilityViews;
            std::vector<TreePoly> tree;
            std::vector<TreePoly> outside;
        };
    };

    static thread_local Analyze::Buffers globalWorkspace;
};
inline thread_local Probability::Analyze::Buffers Probability::globalWorkspace;

// src/algo/probability_engine/probability/observe.h
struct ProbabilityObserve {
    // Probability::observe
    struct Transfer {
        int neighborMines = 0;
        int componentMines = 0;
        long double ways = 0.0L;
    };

    struct Poly {
        int start = 0;
        std::vector<long double> coeffs;
    };

    struct Buffers {
        Poly rest;
        Poly all;
        Poly mult;
        std::vector<ComponentId> captured;
        std::vector<char> seen;
        std::vector<int> adjacentBoxCells;
        RawGrid<long double> dp;
        RawGrid<long double> nextDp;
        std::vector<long double> restWays;
        std::vector<Transfer> transfers;
    };

    static thread_local Buffers observeWorkspace;

    // Probability::buildDfsTable
    struct BuildDfsTable {
        std::vector<std::array<long double, 9>> accumulated;
    };

    inline static thread_local BuildDfsTable buildDfsWorkspace;

    // Probability::buildGraphTable
    struct BuildGraphTable {
        struct Layer {
            struct Count {
                int componentMines = 0;
                int neighborMines = 0;
                long double ways = 0.0L;
                int next = -1;
            };

            struct State {
                std::size_t frontierOffset = 0;
                int firstCount = -1;
                int lastCount = -1;
            };

            std::vector<State> states;
            std::vector<Count> counts;
            std::vector<char> frontierValues;
            FlatHashTable<U128, std::size_t, U128Hash> index;

            void reset() {
                states.clear();
                counts.clear();
                frontierValues.clear();
                index.clear();
                states.push_back({0, 0, 0});
                counts.push_back({0, 0, 1.0L, -1});
            }

            Count &findOrAddCount(State &state, int componentMines, int neighborMines) {
                for (int i = state.firstCount; i >= 0; i = counts[i].next)
                    if (counts[i].componentMines == componentMines && counts[i].neighborMines == neighborMines)
                        return counts[i];
                const int index = counts.size();
                counts.push_back({componentMines, neighborMines, 0.0L, -1});
                if (state.lastCount >= 0)
                    counts[state.lastCount].next = index;
                else
                    state.firstCount = index;
                state.lastCount = index;
                return counts.back();
            }

            template <typename Plan>
            void advance(const Plan &plan, Layer &nextLayer, std::span<const int> adjacentBoxCells, int xBox) const;

            void emit(std::vector<Transfer> &out) const {
                for (const State &state : states)
                    for (int index = state.firstCount; index >= 0; index = counts[index].next) {
                        const Count &count = counts[index];
                        if (count.ways == 0.0L)
                            continue;
                        out.push_back({count.neighborMines, count.componentMines, count.ways});
                    }
            }
        };
    };
};
inline thread_local ProbabilityObserve::Buffers ProbabilityObserve::observeWorkspace;

// src/algo/probability_engine/bruteforce/bruteforce_normal.h
struct BruteForceNormal {
    struct Scratch {
        struct Layer {
            std::vector<int> deaths;
            std::vector<int> safeCells;
            std::vector<int> order;
            std::array<std::vector<std::uint32_t>, 9> groups;
            std::vector<std::pair<int, int>> groupList;
            std::vector<U128> safeHashes;
            std::vector<int> safeGroupIds;
            std::vector<int> safeGroupSizes;
            std::vector<int> safeGroupOffsets;
            std::vector<std::uint32_t> safeGroupedConfigs;
            std::vector<std::span<std::uint32_t>> safeGroupList;
        };

        Layer &layer(int depth);
        void reset();

        FlatHashTable<U128, int, U128Hash> safeGroupTable;
        std::deque<Layer> layers;
    };

    inline static thread_local Scratch scratch;
    inline static thread_local FlatHashTable<U128, int, U128Hash> cache;
};

// src/algo/probability_engine/bruteforce/bruteforce_multimask.h
struct BruteForceMultiMask {
    template <typename Mask> struct Layer {
        std::vector<int> deaths;
        std::vector<int> order;
        std::array<std::vector<std::uint32_t>, 9> groups;
        std::vector<std::pair<int, int>> groupList;
        std::vector<int> safeGroupIds;
        std::vector<int> safeGroupSizes;
        std::vector<int> safeGroupOffsets;
        std::vector<std::uint32_t> safeGroupedConfigs;
        std::vector<std::span<std::uint32_t>> safeGroupList;
    };

    template <typename Mask> struct Scratch {
        using LayerType = Layer<Mask>;

        LayerType &layer(int depth) {
            if ((int)(layers.size()) <= depth)
                layers.emplace_back();
            return layers[depth];
        }

        void reset() {
            for (LayerType &layer : layers) {
                layer.deaths.clear();
                layer.order.clear();
                for (std::vector<std::uint32_t> &group : layer.groups)
                    group.clear();
                layer.groupList.clear();
                layer.safeGroupIds.clear();
                layer.safeGroupSizes.clear();
                layer.safeGroupOffsets.clear();
                layer.safeGroupedConfigs.clear();
                layer.safeGroupList.clear();
            }
            safeGroupTable.clear();
        }

        std::deque<LayerType> layers;
        FlatHashTable<U128, int, U128Hash> safeGroupTable;
        std::array<std::array<int, Mask::kBitCount>, 9> javaLiteMineCounts{};
        std::array<int, 9> javaLiteGroupSizes{};
    };

    template <typename Mask> inline static thread_local Scratch<Mask> scratch;
    template <typename Mask> inline static thread_local FlatHashTable<U128, int, U128Hash> cache;
};

// src/algo/winrate_solver/java_transplant/long_term_risk_helper.h
struct LongTermRisk {
    template <typename ProbabilityResult, typename BoardDelta>
    struct ForceWorkspace {
        ProbabilityResult probability;
        BoardDelta boardDelta;
    };

    template <typename ProbabilityResult, typename BoardDelta>
    inline static thread_local ForceWorkspace<ProbabilityResult, BoardDelta> forceWorkspace;

    template <typename Influence, typename Board, typename Basic, typename StructureResult, typename ProbabilityResult, typename ShapePool,
              typename DistributionPool>
    struct InfluenceTallyCache {
        std::array<std::vector<long double>, 6> values;
        std::array<std::vector<unsigned char>, 6> ready;
        const void *owner = nullptr;
        const Board *board = nullptr;
        const Basic *basic = nullptr;
        const StructureResult *structure = nullptr;
        const ProbabilityResult *probability = nullptr;
        const ShapePool *shapes = nullptr;
        const DistributionPool *distributions = nullptr;

        void reset(int rows, int cols) {
            const int size = (rows + 1) * (cols + 1);
            for (int i = 0; i < 6; ++i) {
                values[i].resize(size);
                ready[i].assign(size, 0);
            }
            owner = nullptr;
        }

        bool reusable(const Board &board_, const Basic &basic_, const StructureResult &structure_, const ProbabilityResult &probability_,
                      const ShapePool &shapes_, const DistributionPool &distributions_, const Influence &full) const {
            return owner == full.tiles.data() && board == &board_ && basic == &basic_ && structure == &structure_ &&
                   probability == &probability_ && shapes == &shapes_ && distributions == &distributions_;
        }
    };

    template <typename Influence, typename Board, typename Basic, typename StructureResult, typename ProbabilityResult, typename ShapePool,
              typename DistributionPool>
    inline static thread_local InfluenceTallyCache<Influence, Board, Basic, StructureResult, ProbabilityResult, ShapePool, DistributionPool>
        influenceTallyCache;
};

} // namespace mss::workspace
