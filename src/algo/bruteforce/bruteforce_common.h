#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "algo/basic.h"
#include "algo/observed_board.h"
#include "algo/shape_solver/shape_solver.h"
#include "algo/structure.h"
#include "core/utility/dynamic_bitset.h"
#include "core/utility/flat_hashtable.h"
#include "core/utility/hash.h"

namespace mss {

struct BruteForce {
    struct Config {
        enum class Route { Automatic, Common };

        // 调用方必须显式填写这两个字段；它们没有默认值，未初始化会直接改变搜索结果。
        bool checkAllMoves;
        int minWins;
        Route route = Route::Automatic;
    };

    struct Result {
        struct Move {
            int x = 0;
            int y = 0;
            int wins = 0;
        };

        int possibilities = 0;
        long long nodes = 0;
        std::vector<Move> moves;
    };

    // possibilities 是 buildCommonSession 枚举到的完整雷位方案数；moves 在
    // checkAllMoves=true 时保留根节点每个候选的 wins，否则只保留达到 minWins 的一格。
    // Move::wins 是该点击在所有揭示分支上可保证继续赢下去的方案数，不是概率。

    // 在当前盘面可能性上进行残局搜索；minWins 只影响单推荐格模式。
    // route 用于选择自动后端或固定使用 Common 后端。两个后端必须返回相同的
    // possibilities、wins 和动作顺序，测试层用 Route::Common 做逐项对拍。
    static Result solve(const ObservedBoard::Result &board, const Basic::Result &basic, const Structure::Result &structure,
                        const Structure::Pool &shapes, const Config &config);

  private:
    using ConfigId = std::uint32_t;
    using CandidateId = std::uint32_t;

    struct CommonSession;
    struct Session;
    template <typename Mask> struct MultiMaskSession;
    struct ScratchBuffers;
    template <typename Mask> struct MultiMaskSolver;

    inline static constexpr int multiMaskCandidateThreshold = 512;

    static thread_local ScratchBuffers scratch;
    static thread_local FlatHashTable<U128, int, U128Hash> cache;

    // 读取某个方案下点击候选格后显示的数字。
    static int revealAt(const CommonSession &common, const Session &session, ConfigId config, CandidateId candidate);
    // 判断某个候选格在指定雷位方案中是否为雷。
    static bool mineAt(const CommonSession &common, const Session &session, ConfigId config, CandidateId candidate);
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
    template <bool CheckAllMoves, bool IsRoot>
    static int solve(const CommonSession &common, Session &s, std::span<ConfigId> configs, int need, int depth,
                     FlatHashTable<U128, int, U128Hash> &table, Result &result);
};

struct BruteForce::CommonSession {
    // CommonSession 将候选格压成稠密下标；mineCells/mineOffsets 是按方案分段的 CSR 布局。
    struct Candidate {
        int x = 0;
        int y = 0;
        std::uint32_t linksOffset = 0;
        std::uint8_t linksCount = 0;
        std::uint8_t fixedMines = 0;
    };
    int candidateCount = 0;
    int possibilityCount = 0;
    std::vector<Candidate> candidates;
    std::vector<int> links;
    std::vector<std::uint32_t> mineOffsets;
    std::vector<CandidateId> mineCells;
};

struct BruteForce::Session {
    // 普通后端用按“方案 × 候选格”的扁平表，避免递归过程中反复计算揭示数字。
    std::vector<std::uint8_t> mine;
    std::vector<std::uint8_t> reveal;
    DynamicBitset unopened;
    long long nodes = 0;
};

} // namespace mss
