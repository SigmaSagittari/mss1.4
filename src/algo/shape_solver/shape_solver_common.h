#pragma once

#include <cstddef>
#include <span>
#include <utility>
#include <vector>

#include "algo/structure.h"
#include "core/types.h"
#include "core/utility/flat_hashtable.h"
#include "core/utility/hash.h"

namespace mss {

struct ShapeSolver {
    struct Distribution {
        class Result {
        public:
            Result(int start, int boxCount, std::vector<long double> ways,
                   std::vector<long double> perBoxExpectations);

            Result(const Result&) = delete;
            Result& operator=(const Result&) = delete;
            Result(Result&&) noexcept = default;
            Result& operator=(Result&&) noexcept = default;

            int start() const { return start_; }
            int boxCount() const { return boxCount_; }
            std::span<const long double> ways() const { return ways_; }
            std::span<const std::span<const long double>> perBoxExpectations() const {
                return perBoxExpectations_;
            }
            std::span<const long double> perBoxExpectation(std::size_t i) const {
                return perBoxExpectations_[i];
            }

        private:
            int start_;
            int boxCount_;
            std::vector<long double> ways_;
            std::vector<std::span<const long double>> perBoxExpectations_;
            std::vector<long double> perBoxExpectationData_;
        };

        struct Pool {
            DistributionId find(U128 hash) const;
            const Result& get(DistributionId id) const { return results_[id]; }
            DistributionId insert(U128 hash, Result result);
            void clear();
            std::size_t size() const { return results_.size(); }

        private:
            std::vector<Result> results_;
            FlatHashTable<U128, DistributionId, U128Hash> index_;
        };
    };

    struct DfsSolver;
    struct GraphSolver;

    static DistributionId analyze(const Structure::Shape& shape,
                                  Distribution::Pool& pool);

    static long double binom(int n, int k);

    inline static constexpr int graphThreshold = 35;
};

}  // namespace mss
