#pragma once

//==============================================================================
// 多掩码残局后端：普通后端（bruteforce_normal.h）的“位运算加速版”，不是另一套
// 算法。两者必须返回相同的 possibilities、Move::wins 和动作顺序，
// test::bruteforce() 用 Route::Common 对拍；读本文件时请随时对照 normal 后端。
//
// 【结构】
//   位集合本体在 core/utility/bit_mask.h（与 DynamicBitset 平级的通用定宽掩码），
//   本文件只负责把残局搜索映射到它上面：
//   1. 四个工具函数                 把 CommonSession 转成掩码布局、并决定搜索顺序
//        buildSession()             方案表 → 每方案一张雷位 Mask + 字节打包雷表 + 揭示表
//        javaLiteScore()            同 death 候选之间的启发式权重（只影响顺序）
//        orderCandidates()          统计 death → 排序 → 队首同 death 层再做 javaLite 重排
//        groupSafeConfigs<Small>()  按“同时揭示的安全格向量”给方案分桶
//   2. solve<CheckAllMoves, IsRoot>()  递归本体，三种模式共用同一个函数体
//
// 【返回值协议：带符号阈值】
//   value > 0  当前 configs 的【精确】可赢方案数
//   value < 0  达不到本次 need，|value| 是【可赢方案数的上界】
//   负数不是“负的胜局数”。上层只在 value > 0 时把它累加进 wins，
//   所以失败分支自然贡献 0，而它的 |value| 被用来收紧 upper 剪枝。
//
// 【缓存协议】
//   键   = hashConfigs(configs)，无序集合哈希。
//   值>=0  精确值，可回答任意 need：cached >= need 就返回 cached，否则返回 -cached。
//   值<0   当前 need 下证明不了的上界，只有 |cached| < need 时才能直接判失败，
//          否则必须继续搜（见 solve 内的缓存查询注释）。
//   键里【不含】unopened：同一 configs 下、已从 unopened 移除的格子必然对所有方案
//   都安全，而 solve 在继续分支前会统一消掉共同安全格，所以精确结果只由 configs
//   决定。negative 上界则取决于本次搜索停在哪里，本来就不是 configs 的固有结果。
//
// 【掩码不变量（load-bearing）】
//   bitMask 定义在 core/utility/bit_mask.h，其高位（>= kBitCount 的部分，以及
//   unopened 中 >= candidateCount 的部分）必须恒为 0。整个算法只依赖这一条，
//   就能用 ~mask 直接表示“其余候选”，于是“共同安全格 = unopened & ~(并集) /
//   逐方案 &= ~mineMask” 这类写法才成立。一旦高位混入脏位，safeMask 就会凭空
//   多出候选，进而生成越界的分组。
//
// 【剪枝骨架】
//   由 runBranchRun 统一实现，safe 分支与候选分支只是 initialRemaining 不同
//   （safe 从 n 起扣；候选从 n - deaths[candidate] 起扣，因为认为该候选是雷的
//   雷位方案直接出局、不进入任何 reveal 组）：
//       target = max(best + 1, need)
//       loop 按 size 降序的 group:
//           if (wins + size + remaining < target) → 失败，upper 取该和
//           value = recurse(group, max(1, target - wins - remaining))
//           if (value <= 0) → 失败，upper 取 wins - value + remaining
//           wins += value
//
// 【solve 的三种模式】
//   A. CheckAllMoves && IsRoot   根节点逐候选汇总，把每个候选的 wins 写进
//                                result.moves[candidate]；失败分支按 0 计入汇总。
//   B. 一般递归节点              按下方注释依次走：终止 → 缓存 → 共同安全格
//                                → 无安全格时逐候选尝试分裂。
//   C. IsRoot 且 n == 1          写回一格推荐动作（无其他语义）。
//==============================================================================

#include <algorithm>
#include <array>
#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

#include "algo/bruteforce/bruteforce_common.h"
#include "core/config.h"
#include "core/utility/bit_mask.h"

namespace mss {

// 本后端使用的四档定宽掩码；宽度按候选数选择最小够用的一档。
using u64 = bitMask<1>;
using u128 = bitMask<2>;
using u256 = bitMask<4>;
using u512 = bitMask<8>;

// 阈值剪枝主循环的结果：reached 为 true 时 wins 是该节点集合的精确可赢方案数；
// 为 false 时 upper 是可证明的可赢上界（按负数协议由调用方编码成 -upper）。
struct BranchRun {
    bool reached = false;
    int wins = 0;
    int upper = 0;
};

template <typename Mask> struct BruteForce::MultiMaskSolver {
    using ConfigId = std::uint32_t;
    using Common = BruteForce::CommonSession;
    using Session = BruteForce::MultiMaskSession<Mask>;
    using Result = BruteForce::Result;
    // 方案数低于此阈值时 javaLiteScore 的分支统计与打分成本不划算，跳过它；
    // 它只影响搜索顺序，因此提高/降低这个值不会改变最终结果。
    inline static constexpr int kJavaLiteConfigThreshold = 10000;
    // 小残局不启动线程；小子问题保留私有缓存，避免锁的开销占主导。
    inline static constexpr int kParallelConfigThreshold = 4096;
    inline static constexpr int kSharedConfigThreshold = 32;

    using Layer = workspace::BruteForceMultiMask::Layer<Mask>;
    using Scratch = workspace::BruteForceMultiMask::Scratch<Mask>;

    struct SharedCache;
    // 仅在本线程参与一次并行搜索期间指向该次调用拥有的共享缓存。
    inline static thread_local SharedCache *sharedCache = nullptr;
    // 根节点的并行闸门，由 BruteForce::solve 按 config.solver 在每次求解开始时设置：
    // Solver::BitwiseRootParallel / BitwiseMultithread 为 true，Solver::Bitwise 为
    // false。递归层不改动它，于是"哪个后端会并行"只由调用入口一个地方决定。
    inline static thread_local bool rootParallel = false;

    // 把完整方案表转换成多 word 雷掩码和揭示数字表。
    static Session buildSession(const Common &common);
    // 用 Java-lite 启发式给同死亡数候选排序。
    static int javaLiteScore(const Common &common, const Session &session, std::span<const ConfigId> configs, int candidate);
    // 统一生成当前 unopened 候选的搜索顺序；root 也必须使用它来预热共享缓存。
    // 【有副作用】同时把每个候选的 death 数写进 layer(depth).deaths，solve 的候选
    // 分支会读它做上界剪枝；返回值是 layer(depth).order 的引用。
    static std::vector<int> &orderCandidates(const Common &common, const Session &session, std::span<const ConfigId> configs, int depth);
    // 按同时揭示的安全格向量对方案分组。
    template <bool Small>
    static void groupSafeConfigs(const Session &session, std::span<ConfigId> configs, const Mask &safeMask, Layer &buf);
    // 把 configs 按点击 candidate 的揭示数字落进 groups[0..9)；返回非空桶数。
    // reveal == 9 表示该方案认为 candidate 是雷，直接出局，不进任何桶——这是本
    // 后端唯一的“雷位判定”入口，桶大小之和因此恒为 configs.size() - 雷位数。
    // 桶是追加语义，调用方负责先清空；单个非空桶只统计一次。
    static int groupByReveal(const Session &session, std::span<const ConfigId> configs, int candidate,
                             std::array<std::vector<ConfigId>, 9> &groups);

    // 按给定顺序遍历一组互不相交、并集等于本次待判方案的桶，做阈值剪枝的累加。
    // sizeOf(i) 返回第 i 个桶的大小，recurse(i, subNeed) 返回该桶的带符号结果。
    // safe 分支与候选分支共用它：两者的唯一差别是 initialRemaining（前者为 n，
    // 后者为 n - deaths[candidate]，即已出局方案数）。
    template <typename SizeOf, typename Recurse>
    static BranchRun runBranchRun(int target, int initialRemaining, int groupCount, SizeOf &&sizeOf, Recurse &&recurse);
    // 在 IsRoot 的精确终局里挑一格展示用推荐动作：取 config 下第一个非雷候选
    // （下标最小者）。这两处只知道"该状态精确可赢 1 局"，没有任何择格依据，
    // 所以固定挑最小下标以保证与普通后端逐项一致。
    static void writeFirstSafeMove(const Common &common, const Session &s, ConfigId config, Result &result);

    template <bool CheckAllMoves>
    static int solveCandidatesParallel(const Common &common, Session &s, std::span<ConfigId> configs, int need, Result &result);

    template <bool CheckAllMoves, bool IsRoot>
    // 递归搜索当前方案集合，按 need 返回可保证的胜局数或失败上界。
    // 返回值符号语义与缓存语义见文件头。Session 的构建与公开结果翻译由
    // BruteForce::solveWithMask / runSolver 负责，本类只管递归本身。
    static int solve(const Common &common, Session &s, std::span<ConfigId> configs, int need, int depth,
                     FlatHashTable<U128, int, U128Hash> &table, Result &result);
};

//==============================================================================

template <typename Mask>
inline int BruteForce::MultiMaskSolver<Mask>::groupByReveal(const BruteForce::MultiMaskSession<Mask> &session,
                                                            std::span<const ConfigId> configs, int candidate,
                                                            std::array<std::vector<ConfigId>, 9> &groups) {
    // 这是热路径最内层之一（根模式每个候选一次、候选分支每个分裂候选一次），
    // 必须完全内联；拆成函数只为消除三处逐字重复。
    int groupCount = 0;
    for (ConfigId config : configs) {
        const int reveal = session.revealByConfig[config][candidate];
        if (reveal == 9)
            continue;
        if (groups[reveal].empty())
            ++groupCount;
        groups[reveal].push_back(config);
    }
    return groupCount;
}

template <typename Mask>
template <typename SizeOf, typename Recurse>
inline BranchRun BruteForce::MultiMaskSolver<Mask>::runBranchRun(int target, int initialRemaining, int groupCount, SizeOf &&sizeOf,
                                                                Recurse &&recurse) {
    // 这是递归树中每个分裂节点都要走的循环，必须完全内联；抽成函数只为让 safe
    // 分支与候选分支共享同一份剪枝逻辑（原先两份逐字重复，负号技巧散落在两处）。
    // 桶按大小降序给出。remaining 是【尚未展开的桶】里的方案数上界，每轮先扣掉
    // 本桶，于是 wins + size + remaining 是乐观上界：已赢 + 本桶全赢 + 剩余桶全赢。
    BranchRun run;
    int wins = 0;
    int remaining = initialRemaining;
    for (int i = 0; i < groupCount; ++i) {
        const int size = sizeOf(i);
        remaining -= size;
        if (wins + size + remaining < target) {
            // 乐观上界都够不到 target，后面桶不必再看；这正是"精确值"与"上界"
            // 分离的地方：upper 只保证 >= 真实可赢数。
            run.upper = wins + size + remaining;
            return run;
        }
        // 该桶至少要贡献 target - wins - remaining 个胜利才可能达标，这个下界
        // 直接变成子树自己的 need（至少 1，保证不会传 0）。
        const int value = recurse(i, (std::max)(1, target - wins - remaining));
        if (value <= 0) {
            // 本桶失败：-value 是本桶的可赢上界，加上剩余桶的全部方案数得到整组
            // 的乐观上界。value 为负故写成 wins - value。
            run.upper = wins - value + remaining;
            return run;
        }
        wins += value;
    }
    // 全部桶都算完，累加值是精确结果，可缓存并复用给任意 target。
    run.reached = true;
    run.wins = wins;
    return run;
}

template <typename Mask>
inline BruteForce::MultiMaskSession<Mask> BruteForce::MultiMaskSolver<Mask>::buildSession(const BruteForce::CommonSession &common) {
    // 把 CommonSession 的 CSR 雷位列表展开为 Mask、按候选字节打包的雷表，
    // 并预计算每个方案点击每格后的数字；雷位置在 revealByConfig 中记为 9。
    Session session;
    session.mineMaskByConfig.resize(common.possibilityCount);
    session.mineByteWordsByConfig.resize(common.possibilityCount, (common.candidateCount + 7) / 8, 0);
    for (int config = 0; config < common.possibilityCount; ++config) {
        for (int i = 0; i < common.minesPerConfig; ++i) {
            const CandidateId candidate = common.mineCandidateIds[config][i];
            session.mineMaskByConfig[config].set(candidate);
            session.mineByteWordsByConfig[config][candidate / 8] |= std::uint64_t{1} << (8 * (candidate % 8));
        }
    }
    session.revealByConfig.resize(common.possibilityCount, common.candidateCount, 0);
    for (int config = 0; config < common.possibilityCount; ++config)
        for (int candidate = 0; candidate < common.candidateCount; ++candidate) {
            int value = common.candidates[candidate].fixedMines;
            const CommonSession::Candidate &current = common.candidates[candidate];
            for (std::uint32_t i = 0; i < current.linksCount; ++i)
                value += session.mineMaskByConfig[config].test(common.links[current.linksOffset + i]);
            session.revealByConfig[config][candidate] = session.mineMaskByConfig[config].test(candidate) ? 9 : value;
        }
    return session;
}

// 返回 Java-lite 的评分分子；实际分母恒为 5 * configs.size()。
// 只统计点击 candidate 后仍可点击的候选，不包含 50/50、热点和进度项。
// 语义：把 configs 按“点击 candidate 会揭示出的数字”分成最多 9 组（reveal == 9
// 表示该方案认为 candidate 是雷，直接出局）；每组内统计每个下一候选在多少方案中
// 是雷，取雷数最少的两名作为该组最可能安全的两格，按下标权重 4:1 给分。
// 分数只用于同 death 候选之间的排序，不参与任何返回值/剪枝协议。
// 缓冲区借用 workspace 的 scratch<Mask>，函数返回后其内容即失效。
template <typename Mask>
inline int BruteForce::MultiMaskSolver<Mask>::javaLiteScore(const BruteForce::CommonSession &common,
                                                            const BruteForce::MultiMaskSession<Mask> &session,
                                                            std::span<const ConfigId> configs, int candidate) {
    // 在死亡数相同的候选中，按“点击后各数字分支还能暴露多少低风险格”排序；
    // 这只改变搜索顺序，不改变 solve 的返回值协议。
    std::array<std::array<int, Mask::kBitCount>, 9> &mineCounts = workspace::BruteForceMultiMask::scratch<Mask>.javaLiteMineCounts;
    std::array<int, 9> &groupSizes = workspace::BruteForceMultiMask::scratch<Mask>.javaLiteGroupSizes;
    for (std::array<int, Mask::kBitCount> &counts : mineCounts)
        counts.fill(0);
    groupSizes.fill(0);
    Mask remaining = session.unopenedCandidates;
    remaining.reset(candidate);
    for (ConfigId config : configs) {
        const int reveal = session.revealByConfig[config][candidate];
        if (reveal == 9)
            continue;
        const Mask &mines = session.mineMaskByConfig[config];
        ++groupSizes[reveal];
        Mask nextMines = mines;
        nextMines &= remaining;
        nextMines.forEachSetBit([&](int next) {
            ++mineCounts[reveal][next];
        });
    }

    int score = 0;
    for (int reveal = 0; reveal < 9; ++reveal) {
        const int groupSize = groupSizes[reveal];
        int bestMine = groupSize;
        int secondMine = groupSize;
        for (int next = 0; next < common.candidateCount; ++next) {
            if (!remaining.test(next))
                continue;
            const int mines = mineCounts[reveal][next];
            if (mines < bestMine) {
                secondMine = bestMine;
                bestMine = mines;
            } else if (mines < secondMine) {
                secondMine = mines;
            }
        }
        score += 4 * (groupSize - bestMine) + (groupSize - secondMine);
    }
    return score;
}

// 生成当前 unopened 候选的搜索顺序；它是本后端所有剪枝的入口，因为
// “按 death 升序”正是候选分支能提前 break 的依据（见 solve 中 n - deaths 的判断）。
// 【副作用】排序结果写进 layer(depth).order，同时把每个候选的 death 数写进
// layer(depth).deaths —— solve 的候选分支会重新读这两个容器，所以调用方必须
// 用同一个 depth 调 orderCandidates，不能把返回的引用带过递归边界。
// 分两段：先全局按 death 升序；再对【队首同一 death 层】用 javaLiteScore 重排，
// 因为只有第一个被尝试的 death 层才有机会真正进入递归。
template <typename Mask>
inline std::vector<int> &BruteForce::MultiMaskSolver<Mask>::orderCandidates(const BruteForce::CommonSession &common,
                                                                            const BruteForce::MultiMaskSession<Mask> &session,
                                                                            std::span<const ConfigId> configs, int depth) {
    Layer &buf = workspace::BruteForceMultiMask::scratch<Mask>.layer(depth);
    std::vector<int> &deaths = buf.deaths;
    deaths.assign(common.candidateCount, 0);
    {
        // 统计每个候选在多少方案中是雷 —— deaths[candidate] = Σ_config mineMaskByConfig.test(candidate)。
        // 使用预处理与 SIMD 加速代码：
        // for (ConfigId config : configs) {
        //     for (int i = 0; i < common.minesPerConfig; ++i)
        //         ++deaths[common.mineCandidateIds[config][i]];
        // }
        // 这里改走 mineByteWordsByConfig：每 8 个候选打成 8 个字节，一次读入 64 位
        // word 就能把 64 个候选的行值累加进 sums。
        // 每批最多 255 行是防溢出设计：sums[word] 的每个字节是 8 位计数器，
        // 255 行叠加后最大 255，不会进位污染相邻字节。
        const int wordCount = session.mineByteWordsByConfig.cols();
        std::array<std::uint64_t, Mask::kBitCount / 8> sums{};
        for (int begin = 0; begin < (int)configs.size(); begin += 255) {
            sums.fill(0);
            const int end = (std::min)(begin + 255, (int)configs.size());
            for (int i = begin; i < end; ++i) {
                const std::uint64_t *row = session.mineByteWordsByConfig[configs[i]];
                for (int word = 0; word < wordCount; ++word)
                    sums[word] += row[word];
            }
            for (int word = 0; word < wordCount; ++word) {
                std::uint64_t value = sums[word];
                // word 覆盖 8 个候选；endCandidate 把最后一个 word 裁到 candidateCount
                //（因为 wordCount 是 (candidateCount + 7) / 8 向上取整）。
                const int endCandidate = (std::min)(word * 8 + 8, common.candidateCount);
                for (int candidate = word * 8; candidate < endCandidate; ++candidate) {
                    deaths[candidate] += value & 255;
                    value >>= 8;
                }
            }
        }
    }
    std::vector<int> &order = buf.order;
    order.clear();
    session.unopenedCandidates.forEachSetBit([&](int candidate) {
        order.push_back(candidate);
    });
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        if (deaths[a] != deaths[b])
            return deaths[a] < deaths[b];
        return a < b;
    });
    // 首轮竞争只在最低 death 的候选之间使用 Java-lite；其余 death 层仍按
    // death 递增，避免为不会先被尝试的候选支付额外评分成本。
    // tieEnd 是 order 中与 order[0] 同 death 的前缀长度；tieEnd > 1 才值得评分。
    if (configs.size() >= kJavaLiteConfigThreshold && order.size() > 1) {
        std::size_t tieEnd = 1;
        while (tieEnd < order.size() && deaths[order[tieEnd]] == deaths[order[0]])
            ++tieEnd;
        if (tieEnd > 1) {
            std::array<int, Mask::kBitCount> javaLiteScores{};
            for (int i = 0; i < (int)(tieEnd); ++i)
                javaLiteScores[order[i]] = javaLiteScore(common, session, configs, order[i]);
            std::sort(order.begin(), order.begin() + tieEnd, [&](int a, int b) {
                if (javaLiteScores[a] != javaLiteScores[b])
                    return javaLiteScores[a] > javaLiteScores[b];
                return a < b;
            });
        }
    }
    return order;
}

// 把 configs 按“同时揭示的安全格向量”分桶：safeMask 是当前所有方案共同认定的
// 安全格，一次把它们全打开后不必逐格建子问题，只按揭示结果分组即可。
// 分桶键 = (revealByConfig[config][safeCandidates[0..k)]) 这个 9 进制向量。
//【输入输出】分桶结果写进 buf：safeGroupList（每个桶一个 span）、safeGroupedConfigs
// 为底层存储，以及 buf.safeGroupOffsets。调用方在返回后立刻按 size 降序排序，
// 排序 tie-break 用 span.data() 比较，因此桶内/桶间顺序都由这几个缓冲决定。
// 【两种实现】
//   Small=true   safeCount <= 3 时键空间只有 9^safeCount <= 729，直接用 729 项
//                直查表（static thread_local，按 keys 记录用过的槽，末尾逐个清空）。
//   Small=false  任意 safeCount，把揭示值按 4bit 打包做 U128 哈希分桶，占用
//                scratch 的 safeGroupTable（每次调用前 clear）。
//   两条路径产出的桶集合相同，只是一条走直查一条走哈希，选择纯粹是性能取舍。
template <typename Mask>
template <bool Small>
inline void BruteForce::MultiMaskSolver<Mask>::groupSafeConfigs(const BruteForce::MultiMaskSession<Mask> &session,
                                                                std::span<ConfigId> configs, const Mask &safeMask, Layer &buf) {
    std::vector<std::span<ConfigId>> &groupList = buf.safeGroupList;
    groupList.clear();
    std::vector<int> &groupOffsets = buf.safeGroupOffsets;
    std::vector<ConfigId> &groupedConfigs = buf.safeGroupedConfigs;
    std::array<int, Small ? 3 : Mask::kBitCount> safeCandidates;
    int safeCount = 0;
    safeMask.forEachSetBit([&](int candidate) {
        safeCandidates[safeCount++] = candidate;
    });
    if constexpr (Small) {
        // 729 == 9^3；Small 只在 safeCount <= 3 时被实例化（见 solve 中的调用点），
        // safeCount 同时也是 9 进制键的位数，所以 key 一定落在 [0, 729) 内。
        static thread_local std::array<std::vector<ConfigId>, 729> groups;
        static thread_local std::vector<int> keys;
        // safeCandidates 已经按 forEachSetBit 的下标递增顺序排好，因此分桶键与
        // 方案在 configs 中的出现顺序无关，只由揭示向量本身决定。
        // 直查表不预先 clear：靠 keys 记住本轮用过的槽，末尾按 keys 逐个清空。
        // 注意这里刻意不用 groupByReveal：本处键是跨 safeCount 个格子的 9 进制
        // 联合键（直接当直查下标用），而不是单个候选的揭示值，两者不同构。
        for (ConfigId config : configs) {
            int key = 0;
            int factor = 1;
            for (int i = 0; i < safeCount; ++i) {
                key += session.revealByConfig[config][safeCandidates[i]] * factor;
                factor *= 9;
            }
            if (groups[key].empty())
                keys.push_back(key);
            groups[key].push_back(config);
        }
        // 把桶摊平成 offsets + groupedConfigs，使每个桶成为一段连续 span。
        groupOffsets.resize(keys.size() + 1);
        groupOffsets[0] = 0;
        groupedConfigs.resize(configs.size());
        for (int i = 0; i < (int)(keys.size()); ++i) {
            std::vector<ConfigId> &group = groups[keys[i]];
            groupOffsets[i + 1] = groupOffsets[i] + group.size();
            std::copy(group.begin(), group.end(), groupedConfigs.begin() + groupOffsets[i]);
            groupList.emplace_back(groupedConfigs.data() + groupOffsets[i], group.size());
        }
        for (int key : keys)
            groups[key].clear();
        keys.clear();
    } else {
        // 大 safeCount 路径：把每个方案的揭示向量按 4bit/格打包后哈希成一个 U128，
        // 再用 FlatHashTable 做值 → 桶 id 的映射（slot 存 id + 1，0 表示空槽）。
        FlatHashTable<U128, int, U128Hash> &groupTable = workspace::BruteForceMultiMask::scratch<Mask>.safeGroupTable;
        std::vector<int> &groupIds = buf.safeGroupIds;
        std::vector<int> &groupSizes = buf.safeGroupSizes;
        groupTable.clear();
        groupTable.reserve(configs.size());
        groupIds.resize(configs.size());
        groupSizes.clear();
        const int full = safeCount / 16 * 16;
        for (int i = 0; i < (int)(configs.size()); ++i) {
            // 每 16 格打成一个 u64（4bit * 16 = 64bit）后喂给 hasher；不足 16 的
            // 尾巴单独打包一次。哈希碰撞只会让不同揭示向量共用桶 id —— 与 normal
            // 后端一致，是同一套近似分组策略，故此处不做二次校验。
            const auto *row = session.revealByConfig[configs[i]];
            U128Hasher hasher;
            for (int j = 0; j < full; j += 16) {
                std::uint64_t packed = 0;
                for (int k = 0; k < 16; ++k)
                    packed |= (std::uint64_t)row[safeCandidates[j + k]] << (4 * k);
                hasher.mix(packed);
            }
            if (full < safeCount) {
                std::uint64_t packed = 0;
                for (int j = full; j < safeCount; ++j)
                    packed |= (std::uint64_t)row[safeCandidates[j]] << (4 * (j - full));
                hasher.mix(packed);
            }
            int &slot = groupTable[hasher.finalize()];
            if (slot == 0) {
                slot = groupSizes.size() + 1;
                groupSizes.push_back(0);
            }
            groupIds[i] = slot - 1;
            ++groupSizes[groupIds[i]];
        }
        // 计数排序：先由桶大小算前缀偏移，再把 groupSizes 复用成写游标。
        groupOffsets.resize(groupSizes.size() + 1);
        groupOffsets[0] = 0;
        for (int i = 0; i < (int)(groupSizes.size()); ++i)
            groupOffsets[i + 1] = groupOffsets[i] + groupSizes[i];
        groupedConfigs.resize(configs.size());
        for (int i = 0; i < (int)(groupSizes.size()); ++i)
            groupSizes[i] = groupOffsets[i];
        for (int i = 0; i < (int)(configs.size()); ++i)
            groupedConfigs[groupSizes[groupIds[i]]++] = configs[i];
        for (int i = 0; i < (int)(groupSizes.size()); ++i)
            groupList.emplace_back(groupedConfigs.data() + groupOffsets[i], groupOffsets[i + 1] - groupOffsets[i]);
    }
}

template <typename Mask>
inline void BruteForce::MultiMaskSolver<Mask>::writeFirstSafeMove(const BruteForce::CommonSession &common,
                                                                 const BruteForce::MultiMaskSession<Mask> &s, ConfigId config,
                                                                 BruteForce::Result &result) {
    // 冷路径（只在 IsRoot 的精确终局触发一次），但仍保证与原先内联写法逐字等价：
    // revealByConfig == 9 即该方案认为此格是雷，跳过，取第一个非雷候选。
    for (int candidate = 0; candidate < common.candidateCount; ++candidate)
        if (s.revealByConfig[config][candidate] != 9) {
            result.moves[0].x = common.candidates[candidate].x;
            result.moves[0].y = common.candidates[candidate].y;
            return;
        }
}

template <typename Mask>
template <bool CheckAllMoves, bool IsRoot>
inline int BruteForce::MultiMaskSolver<Mask>::solve(const BruteForce::CommonSession &common, BruteForce::MultiMaskSession<Mask> &s,
                                                    std::span<ConfigId> configs, int need, int depth,
                                                    FlatHashTable<U128, int, U128Hash> &table, BruteForce::Result &result) {
    // 与普通后端相同，正数是当前方案集合的精确可赢数，负数是未达到 need
    // 时的可赢上界编码；Mask 只改变雷位/安全集合的表示，不改变这个调用协议。
    // 根节点在 CheckAllMoves 模式下逐候选汇总各数字分支，否则只求一条推荐路径。
    //【缓冲区纪律】下面的 Layer 来自 scratch.layer(depth)：同一 depth 的容器会被
    // 所有兄弟递归共享，所以任何跨递归存活的数据都必须先落到局部变量，或者读完
    // 立刻用掉。orderCandidates 与 groupSafeConfigs 都往这一层写结果。
    if constexpr (CheckAllMoves && IsRoot) {
        // 模式 A：根节点枚举。对每个候选都独立跑一遍“点击它之后”的完整搜索，
        // 因此这里不用 need 做阈值剪枝（need 恒为 1），wins 是各数字分支的累加。
        ++s.nodes;
        const int n = configs.size();
        result.moves.clear();
        // 方案总数不足 need，-n 表示这是本节点可达到的最大上界。
        if (need > n)
            return -n;
        Layer &buf = workspace::BruteForceMultiMask::scratch<Mask>.layer(depth);
        result.moves.resize(common.candidateCount);
        int best = 0;
        std::array<std::vector<ConfigId>, 9> &groups = buf.groups;
        std::vector<int> &order = orderCandidates(common, s, configs, depth);
        if (rootParallel && kMaxBruteforceCores > 1 && n >= kParallelConfigThreshold)
            return solveCandidatesParallel<true>(common, s, configs, need, result);
        for (int candidate : order) {
            // 按揭示数字把 configs 分 9 桶；认为 candidate 是雷的方案直接出局，
            // 不计入任何桶。
            for (std::vector<ConfigId> &group : groups)
                group.clear();
            groupByReveal(s, configs, candidate, groups);
            s.unopenedCandidates.reset(candidate);
            int wins = 0;
            for (int reveal = 0; reveal < 9; ++reveal)
                if (!groups[reveal].empty()) {
                    // 各桶互不相交，wins 是它们的精确值之和；失败分支按 0 计入。
                    const int value = solve<false, false>(common, s, groups[reveal], 1, depth + 1, table, result);
                    if (value > 0)
                        wins += value;
                }
            s.unopenedCandidates.set(candidate);
            // result.moves 与 order 顺序一致（而不是候选下标顺序），这是对拍契约的一部分。
            result.moves[candidate] = {common.candidates[candidate].x, common.candidates[candidate].y, wins};
            best = (std::max)(best, wins);
        }
        return best;
    } else {
        ++s.nodes;
        const int n = configs.size();
        if (n <= 1) {
            // 单方案节点至多贡献一个胜利方案；不足 need 时返回负上界。
            // 注意 need > n 的检查必须在返回 n 之前：调用方可能带着更大的 need 进入。
            if (need > n)
                return -n;
            if constexpr (IsRoot)
                if (n == 1)
                    // 只剩一个方案时可赢数精确为 1；推荐格取该方案下下标最小的非雷候选。
                    writeFirstSafeMove(common, s, configs[0], result);
            return n;
        }
        // 当前方案集合规模不足以满足目标阈值；n 是 configs 的固有性质，
        // 所以这一步可以放在查表之前，也不必写进缓存。
        if (need > n)
            return -n;
        if constexpr (!IsRoot) {
            if (n == 2) {
                // 两方案都能赢，当且仅当某个共同安全格能把它们区分开。
                Mask safe = s.unopenedCandidates;
                safe &= ~s.mineMaskByConfig[configs[0]];
                safe &= ~s.mineMaskByConfig[configs[1]];
                while (safe.any()) {
                    const int candidate = safe.firstSetBit();
                    if (s.revealByConfig[configs[0]][candidate] != s.revealByConfig[configs[1]][candidate])
                        return 2;
                    safe.reset(candidate);
                }
                return need <= 1 ? 1 : -1;
            }
        }
        const U128 key = sharedCache == nullptr ? BruteForce::hashConfigs(configs) : sharedCache->hashConfigs(configs);
        // 正缓存值可直接回答阈值查询；负缓存值只有在绝对值仍低于 need 时
        // 才能直接证明失败，否则该缓存只对更低阈值有效。
        // 例：缓存 -5 表示"最多赢 5 个"；need=3 时 5 >= 3 无法证明失败，
        // 必须继续搜；need=7 时 5 < 7 可直接返回 -5。
        const bool shared = sharedCache != nullptr && n >= kSharedConfigThreshold;
        int cached = 0;
        if (shared)
            cached = sharedCache->find(key);
        else if (const int *value = table.find(key))
            cached = *value;
        if (cached != 0) {
            if (cached > 0)
                return cached >= need ? cached : -cached;
            if (-cached < need)
                return cached;
        }
        Layer &buf = workspace::BruteForceMultiMask::scratch<Mask>.layer(depth);
        std::vector<int> &deaths = buf.deaths;
        // 共同安全格 = unopened（当前候选全集）中所有方案都认为不是雷的格。
        // 逐方案 &= ~mineMask 取交集；任何一步交集为空就提前结束。
        // 这里依赖“mineMask 高位恒为 0”，否则 ~mineMask 会带回脏的高位候选。
        Mask safeMask = s.unopenedCandidates;
        for (ConfigId config : configs) {
            safeMask &= ~s.mineMaskByConfig[config];
            if (!safeMask.any())
                break;
        }
        if (safeMask.any()) {
            // safeMask 是所有 configs 的交集安全格；将其整体消去后按整组揭示向量
            // 聚类，复用“同时打开安全格”的递归子问题。
            if constexpr (IsRoot) {
                // 根节点只记一格代表性安全格；同组安全格全开，等价结果唯一，
                // 记哪一格不影响对拍（normal 后端同样取第一个）。
                const ConfigId candidate = safeMask.firstSetBit();
                result.moves[0].x = common.candidates[candidate].x;
                result.moves[0].y = common.candidates[candidate].y;
            }
            s.unopenedCandidates &= ~safeMask;
            // safeCount <= 3 走 729 项直查表，否则走哈希分桶；两者产出同样的桶集合。
            if (safeMask.popcount() <= 3)
                groupSafeConfigs<true>(s, configs, safeMask, buf);
            else
                groupSafeConfigs<false>(s, configs, safeMask, buf);
            std::vector<std::span<ConfigId>> &groupList = buf.safeGroupList;
            // 按桶大小降序：大桶先算，能让后面小桶的阈值剪枝更容易命中；
            // tie-break 用 data() 比较，保证同规模桶的处理顺序确定。
            std::sort(groupList.begin(), groupList.end(), [](auto a, auto b) {
                if (a.size() != b.size())
                    return a.size() > b.size();
                return a.data() < b.data();
            });
            // 桶已按 size 降序；initialRemaining = n 表示一个桶都还没算。
            const BranchRun run = runBranchRun(
                need, n, (int)(groupList.size()), [&](int i) { return (int)(groupList[i].size()); },
                [&](int i, int subNeed) { return solve<false, false>(common, s, groupList[i], subNeed, depth + 1, table, result); });
            s.unopenedCandidates |= safeMask;
            if (!run.reached) {
                // 仍无法达到 need；upper 包含已累计结果和未展开分支的最大贡献。
                if (shared)
                    sharedCache->store(key, -run.upper);
                else
                    BruteForce::saveFail(key, run.upper, n, table);
                return -run.upper;
            }
            // 所有桶都算完，wins 是精确值，可缓存并复用给任意 need。
            if (shared)
                sharedCache->store(key, run.wins);
            else
                table[key] = run.wins;
            return run.wins;
        }

        // ---- 没有共同安全格，只能逐候选尝试分裂 ----
        // orderCandidates 同时准备好 buf.deaths 与 buf.order：deaths[candidate] 是
        // 认为该候选是雷的方案数，也就是点击它会直接出局的方案数，下面的上界
        // n - deaths[candidate] 由此而来。
        std::vector<int> &order = orderCandidates(common, s, configs, depth);
        if constexpr (IsRoot)
            if (rootParallel && kMaxBruteforceCores > 1 && n >= kParallelConfigThreshold)
                return solveCandidatesParallel<false>(common, s, configs, need, result);
        int best = 0;
        // upper 是"在所有已尝试候选上见过的最好上界"，与 best（最好精确值）互不覆盖；
        // 结尾取 max(best, upper) 作为整节点的上界。
        int upper = 0;
        for (int candidate : order) {
            // 候选按 death 升序，于是目标阈值不降时可以直接 break：
            // 连乐观值 n - deaths 都低于 target，后面的候选只会更差。
            const int target = (std::max)(best + 1, need);
            if (n - deaths[candidate] < target) {
                upper = (std::max)(upper, n - deaths[candidate]);
                break;
            }
            // 按揭示数字分桶：认为候选是雷的方案直接出局，不进任何桶，
            // 故所有桶的规模之和 = n - deaths[candidate]。
            std::array<std::vector<ConfigId>, 9> &groups = buf.groups;
            for (std::vector<ConfigId> &group : groups)
                group.clear();
            const int groupCount = groupByReveal(s, configs, candidate, groups);
            if (groupCount <= 1) {
                // 不分裂候选不会产生新的信息分支；不更新 upper，供末尾识别所有
                // 候选都不分裂的精确终局。
                continue;
            }
            std::vector<std::pair<int, int>> &groupList = buf.groupList;
            groupList.clear();
            for (int reveal = 0; reveal < 9; ++reveal)
                if (!groups[reveal].empty())
                    groupList.push_back({reveal, groups[reveal].size()});
            // 大桶优先，理由与 safe 分支相同。
            std::sort(groupList.begin(), groupList.end(), [](const auto &a, const auto &b) {
                if (a.second != b.second)
                    return a.second > b.second;
                return a.first < b.first;
            });
            s.unopenedCandidates.reset(candidate);
            // 与 safe 分支同一套剪枝，只是 initialRemaining 换成 n - deaths[candidate]：
            // 认为该候选是雷的方案已经出局，本来就不该计入任何桶。
            const BranchRun run = runBranchRun(
                target, n - deaths[candidate], (int)(groupList.size()), [&](int i) { return groupList[i].second; },
                [&](int i, int subNeed) {
                    return solve<false, false>(common, s, groups[groupList[i].first], subNeed, depth + 1, table, result);
                });
            s.unopenedCandidates.set(candidate);
            if (!run.reached)
                upper = (std::max)(upper, run.upper);
            if (run.reached && run.wins > best) {
                best = run.wins;
                if constexpr (IsRoot) {
                    result.moves[0].x = common.candidates[candidate].x;
                    result.moves[0].y = common.candidates[candidate].y;
                }
            }
        }
        if (best >= need) {
            if (shared)
                sharedCache->store(key, best);
            else
                table[key] = best;
            return best;
        }
        // upper == 0 说明没有可分裂候选进入失败路径；所有候选行为等价，
        // 因而该状态的精确可赢数是 1，而不是依赖本次 need 的失败上界。
        if (best == 0 && upper == 0) {
            // 这里是精确结果，可供任意 need 直接复用。
            if (shared)
                sharedCache->store(key, 1);
            else
                table[key] = 1;
            // 所有候选行为等价，没有择格依据，固定取最小下标的非雷候选。
            if constexpr (IsRoot)
                writeFirstSafeMove(common, s, configs[0], result);
            return 1;
        }
        // 未达标：max(best, upper) 是本节点可证明的可赢上界，按负数协议缓存。
        if (shared)
            sharedCache->store(key, -(std::max)(best, upper));
        else
            BruteForce::saveFail(key, (std::max)(best, upper), n, table);
        return -(std::max)(best, upper);
    }
}

} // namespace mss

#include "algo/bruteforce/multimask/bruteforce_multimask_rootparallel.h"
