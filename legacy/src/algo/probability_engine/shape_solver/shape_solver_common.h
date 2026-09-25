#pragma once

#include <cstddef>
#include <span>
#include <utility>
#include <vector>

#include "algo/probability_engine/structure.h"
#include "core/types.h"
#include "core/utility/combinatorics.h"
#include "core/utility/flat_hashtable.h"
#include "core/utility/grid.h"
#include "core/utility/hash.h"

namespace mss {

struct ShapeSolver {
    // 图 DP 的消元顺序策略；只影响状态峰值和耗时，不影响最终分布。
    enum class OrderAlgo {
        Adjacent, // 只按相邻关系贪心，开销最低，但顺序质量较差。
        Window3, // 比较局部三步窗口，开销略高，通常能得到更好的顺序。
        SA, // 反复优化候选顺序，开销最高，顺序质量接近最优。
        Auto, // 组合 Adjacent 和 Window3；默认策略，足以应付所有随机盘面。
        AutoSA, // 在 Auto 结果仍不理想时追加 SA，面向人工构造的极端盘面。
    };

    // 一个 Distribution 按组件 Box 数量统计：ways[k] 是恰有 k 雷的具体布局权重，
    // perBoxExpectation(k)[box] 是在该条件下该 Box 的期望雷数；外层 Probability
    // 再用 ways 与其它组件多项式卷积，才能得到全局而非组件内的概率。
    struct Distribution {
        class Result {
          public:
            // 创建指定雷数区间和 Box 期望值的分布结果。
            Result(int start, int boxCount, std::vector<long double> ways, RawGrid<long double> perBoxExpectations);

            // 禁止复制分布结果，避免复制内部 span 视图。
            Result(const Result &) = delete;
            // 禁止复制赋值分布结果。
            Result &operator=(const Result &) = delete;
            // 移动分布结果及其拥有的系数存储。
            Result(Result &&) noexcept = default;
            // 移动赋值分布结果。
            Result &operator=(Result &&) noexcept = default;

            // 返回 ways[0] 所对应的实际最小组件雷数，实际雷数为 start()+索引。
            int start() const {
                return start_;
            }
            // 返回该组件的 Box 数量。
            int boxCount() const {
                return boxCount_;
            }
            // 返回各总雷数对应的布局权重。
            std::span<const long double> ways() const {
                return ways_;
            }
            // 返回按组件总雷数分组的每个 Box 期望雷数视图；索引仍是压缩区间索引。
            std::span<const std::span<const long double>> perBoxExpectations() const {
                return perBoxExpectations_;
            }
            // 返回指定总雷数索引对应的 Box 期望雷数。
            std::span<const long double> perBoxExpectation(std::size_t i) const {
                return perBoxExpectations_[i];
            }

          private:
            int start_;
            int boxCount_;
            std::vector<long double> ways_;
            std::vector<std::span<const long double>> perBoxExpectations_;
            RawGrid<long double> perBoxExpectationData_;
        };

        struct Pool {
            // 按结构哈希查找已缓存的分布，未命中返回 -1。
            DistributionId find(U128 hash) const;
            // 读取指定分布缓存项。
            const Result &get(DistributionId id) const {
                return results_[id];
            }
            // 插入分布或返回相同哈希已有的缓存项。
            DistributionId insert(U128 hash, Result result);
            // 清空分布缓存但保留底层容量。
            void clear();
            // 返回当前缓存中的分布数量。
            std::size_t size() const {
                return results_.size();
            }

          private:
            std::vector<Result> results_;
            FlatHashTable<U128, DistributionId, U128Hash> index_;
        };
    };

    struct DfsSolver;
    struct GraphSolver;

    // 按组件规模选择 DFS 或 Graph 后端并返回缓存句柄；两者必须产出同一分布，
    // 阈值只为控制状态爆炸和运行时间。
    static DistributionId analyze(const Structure::Shape &shape, const Structure::Pool &shapes, Distribution::Pool &pool,
                                  const OrderAlgo &algo = OrderAlgo::Auto);

    // 小组件走 DFS，大组件走图 DP；这是性能阈值，不是正确性分界。
    inline static constexpr int graphThreshold = 35;
};

} // namespace mss
