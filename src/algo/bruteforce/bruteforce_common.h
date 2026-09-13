#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "algo/basic.h"
#include "algo/observed_board.h"
#include "algo/structure.h"
#include "algo/shape_solver/shape_solver.h"
#include "core/utility/dynamic_bitset.h"
#include "core/utility/flat_hashtable.h"
#include "core/utility/hash.h"

namespace mss {

struct BruteForce {
    struct Config {
        bool checkAllMoves;
        int minWins;
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

    // 在当前盘面可能性上进行残局搜索。
    // Config 的所有字段必须由调用方显式指定；minWins 只影响非 checkAllMoves 模式。
    static Result solve(const ObservedBoard::Result& board,
                        const Basic::Result& basic,
                        const Structure::Result& structure,
                        const Structure::Pool& shapes,
                        const Config& config);

private:
    using ConfigId = std::uint32_t;
    using CandidateId = std::uint32_t;

    struct CommonSession;
    struct Session;
    struct BitwiseSession;
    struct ScratchBuffers;
    struct BitwiseSolver;

    inline static constexpr int bitwiseCandidateThreshold = 64;

    static thread_local ScratchBuffers scratch;
    static thread_local FlatHashTable<U128, int, U128Hash> cache;

    static int revealAt(const CommonSession& common, const Session& session,
                        ConfigId config,
                        CandidateId candidate);
    static bool mineAt(const CommonSession& common, const Session& session,
                       ConfigId config,
                       CandidateId candidate);
    static U128 hashConfigs(std::span<const ConfigId> configs);
    static void saveFail(const U128& key, int upper, int count,
                         FlatHashTable<U128, int, U128Hash>& table);
    static CommonSession buildCommonSession(
        const ObservedBoard::Result& board, const Basic::Result& basic,
        const Structure::Result& structure, const Structure::Pool& shapes);
    static Session buildSession(const CommonSession& common);
    static BitwiseSession buildBitwiseSession(const CommonSession& common);
    template <bool CheckAllMoves, bool IsRoot>
    static int solve(const CommonSession& common, Session& s,
                     std::span<ConfigId> configs, int need,
                     int depth, FlatHashTable<U128, int, U128Hash>& table,
                     Result& result);
};

struct BruteForce::CommonSession {
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
    std::vector<std::uint8_t> mine;
    std::vector<std::uint8_t> reveal;
    DynamicBitset unopened;
    long long nodes = 0;
};

struct BruteForce::BitwiseSession {
    std::vector<std::uint64_t> mineMasks;
    std::vector<std::uint8_t> reveal;
    std::uint64_t unopened = 0;
    long long nodes = 0;
};

}  // namespace mss
