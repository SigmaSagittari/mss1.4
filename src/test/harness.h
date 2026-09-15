#pragma once

#include "test/basic.h"
#include "test/bruteforce.h"
#include "test/flat_hashtable.h"
#include "test/observed_board.h"
#include "test/performance.h"
#include "test/probability_case.h"
#include "test/real_endgame_performance.h"
#include "test/radix_sort.h"
#include "test/structure.h"

namespace test {

inline void harness() {
    // Basic：局部确定性推理、标记传播和增量更新的功能回归。
    //basic();
    // Bruteforce：逐格/多掩码残局搜索的结果一致性与边界回归。
    //bruteforce();
    // FlatHashTable：平坦哈希表与 libcuckoo 的固定操作序列性能对比。
    //flatHashtable();
    // ObservedBoard：棋盘观测、翻开更新和边界状态的功能回归。
    //observedBoard();
    // ProbabilityCase：人工构造的极端概率盘面压力测试，不代表通用性能。
    probabilityCase();
    // Performance：固定种子随机实战盘面，完整概率分析链路的 20 秒吞吐基线。
    //performance();
    // RadixSort：基数排序的功能与固定数据性能回归。
    //radixSort();
    // Structure：组件拆分、Box 压缩、池化和增量 Delta 回放的功能回归。
    //structure();
    // RealEndgamePerformance：随机实战残局样本的暴力搜索节点/耗时与结果对比。
    //real_endgame_performance(0, 50000, 1200.0, true);
}

}  // namespace test
