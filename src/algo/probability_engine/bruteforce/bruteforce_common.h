#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "algo/probability_engine/basic.h"
#include "algo/probability_engine/observed_board.h"
#include "algo/probability_engine/shape_solver/shape_solver.h"
#include "algo/probability_engine/structure.h"
#include "core/utility/bit_mask.h"
#include "core/utility/dynamic_bitset.h"
#include "core/utility/flat_hashtable.h"
#include "core/utility/grid.h"
#include "core/utility/hash.h"
#include "core/workspace.h"

namespace mss {

struct BruteForce {
    // 残局搜索后端。三个成员都必须返回相同的 Move::wins 和动作
    // 顺序，只在耗时与适用规模上不同；测试层用 Solver::Common 做逐项对拍。
    enum class Solver {
        // 普通递归后端（bruteforce_normal.h）。规模不限，总是可用。
        Common,
        // 掩码后端（multimask/bruteforce_multimask.h），单线程执行。
        Bitwise,
        // 掩码后端 + 根节点按候选并行。
        BitwiseRootParallel,
    };

    struct Config {
        // 是否在根节点求出每一个候选格的精确可赢数。
        bool checkAllMoves;
        // 单推荐格模式要求当前动作至少能保证的胜局数。
        int minWins;
        // 指定后端；退化规则见 solve 内的注释。
        Solver solver = Solver::Bitwise;
    };

    struct Result {
        struct Move {
            // 候选格的实际棋盘坐标。
            int x = 0;
            int y = 0;
            // 点击此格后仍能保证继续赢下去的完整布局数，不是概率。
            int wins = 0;
        };

        // 递归 solve 实际访问的节点数。
        long long nodes = 0;
        // 根节点输出的动作；单推荐格模式最多保留一个动作。
        std::vector<Move> moves;
    };

    // BruteForce::Result 只返回搜索结果；完整布局总数和各格概率由 Probability 层提供。
    // Move::wins 是点击该格后仍能保证最终获胜的布局数，不是概率。

    // 在当前观测和约束允许的完整雷位布局上搜索；minWins 只影响单推荐格模式。
    // solver 用于选择后端。所有后端必须返回相同的 wins 和动作
    // 顺序，测试层用 Solver::Common 做逐项对拍。
    static Result solve(const ObservedBoard::Result &board, const Basic::Result &basic, const Structure::Result &structure,
                        const Structure::Pool &shapes, const Config &config);

  private:
    // ConfigId 是完整雷位方案在 CommonSession 中的下标。
    using ConfigId = std::uint32_t;
    // CandidateId 是候选格在 CommonSession::candidates 中的稠密下标。
    using CandidateId = std::uint32_t;

    struct CommonSession;
    struct Session;
    template <typename Mask> struct MultiMaskSession;
    using ScratchBuffers = workspace::BruteForceNormal::Scratch;
    template <typename Mask> struct MultiMaskSolver;

    inline static constexpr int multiMaskCandidateThreshold = 512;
    // 阈值必须落在最宽掩码的位宽内：越过的候选会被 Mask::all 直接断言掉，
    // 所以这两个数字不允许改脱节。
    static_assert(multiMaskCandidateThreshold <= bitMask<8>::kBitCount, "阈值不能超过最宽掩码的位宽");

    // 为当前方案集合生成无序缓存键；递归分组会改变 span 顺序，但同一集合的
    // 子问题结果必须命中同一个缓存项。
    static U128 hashConfigs(std::span<const ConfigId> configs);
    // 将“最多还能赢多少”编码进搜索缓存：正值是已知精确值，负值的绝对值是
    // 在当前 need 下证明不了的上界；调用者据此决定能否提前停止递归。
    static void saveFail(const U128 &key, int upper, int count, FlatHashTable<U128, int, U128Hash> &table);
    // 把盘面约束压缩为候选格、链接关系和完整雷位方案。
    static CommonSession buildCommonSession(const ObservedBoard::Result &board, const Basic::Result &basic,
                                            const Structure::Result &structure, const Structure::Pool &shapes);
    // 预计算普通后端所需的方案雷表和揭示数字表。
    static Session buildSession(const CommonSession &common);
    // 两个多掩码宽度的统一驱动：建 Session 后交给 MultiMaskSolver 求解。
    template <typename Mask> static Result solveWithMask(const CommonSession &common, const Config &config);
    // 两个后端共用的求解驱动：按 checkAllMoves/minWins 分派两个 solve 入口，
    // 并把带符号的递归返回值翻译成公开的 Move::wins。
    template <typename Solver, typename SessionT>
    static Result runSolver(const CommonSession &common, SessionT &session, const Config &config,
                            FlatHashTable<U128, int, U128Hash> &table);
    template <bool CheckAllMoves, bool IsRoot>
    static int solve(const CommonSession &common, Session &s, std::span<ConfigId> configs, int need, int depth,
                     FlatHashTable<U128, int, U128Hash> &table, Result &result);
};

struct BruteForce::CommonSession {
    // 候选格压成稠密 CandidateId 后，每个候选格保存其棋盘位置和揭示数字所需的邻接信息。
    struct Candidate {
        // 候选格的 1-based 棋盘坐标。
        int x = 0;
        int y = 0;
        // links 中本候选格的邻接候选区间起点。
        std::uint32_t linksOffset = 0;
        // links 中本候选格的邻接候选数量。
        std::uint8_t linksCount = 0;
        // 本候选格周围已经确定为雷的格子数量。
        std::uint8_t fixedMines = 0;
    };
    // candidates、mineCandidateIds 都使用这些稠密下标；候选格数量。
    int candidateCount = 0;
    // 完整雷位方案数量；每个方案对应一个 ConfigId。
    int possibilityCount = 0;
    // CandidateId -> 候选格坐标和邻接信息。
    std::vector<Candidate> candidates;
    // 所有候选格的邻接 CandidateId 连续存储区；每个候选格的区间由 linksOffset/linksCount 给出。
    std::vector<int> links;
    // 每个完整方案固定包含的候选雷数量，也就是当前盘面的剩余雷数。
    int minesPerConfig = 0;
    // [ConfigId][第几个雷] 的候选雷 CandidateId 表；构建时按行追加，最终列数恒为 minesPerConfig。
    RawGrid<CandidateId> mineCandidateIds;
};

struct BruteForce::Session {
    // 普通后端的 [ConfigId][CandidateId] 雷标记表；1 表示该候选格在方案中是雷。
    RawGrid<std::uint8_t> mineByConfig;
    // 普通后端的 [ConfigId][CandidateId] 揭示数字表，避免递归中重复计算。
    RawGrid<std::uint8_t> revealByConfig;
    // CandidateId 位图；1 表示该候选格尚未被当前搜索路径打开。
    DynamicBitset unopenedCandidates;
    // 当前 Session 递归访问的节点数。
    long long nodes = 0;
};

template <typename Mask> struct BruteForce::MultiMaskSession {
    // 每个 ConfigId 一张 Mask；第 CandidateId 位为 1 表示该格是雷。
    std::vector<Mask> mineMaskByConfig;
    // [ConfigId][每 8 个 CandidateId 一组]；每个字节最低位表示对应候选为雷。
    RawGrid<std::uint64_t> mineByteWordsByConfig;
    // 与普通后端相同的 [ConfigId][CandidateId] 揭示数字表。
    RawGrid<std::uint8_t> revealByConfig;
    // CandidateId 位图；1 表示该候选格尚未被当前搜索路径打开。
    Mask unopenedCandidates;
    // 当前 Session 递归访问的节点数。
    long long nodes = 0;
};

} // namespace mss
