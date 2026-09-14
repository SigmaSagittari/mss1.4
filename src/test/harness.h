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
    // 当前测试入口：构建并运行即可执行 probabilityCase。
    //basic();
    probabilityCase();
    //flatHashtable();
    //observedBoard();
    //radixSort();
    //structure();
    //performance();
    //real_endgame_performance(0, 50000, 1200.0, true);
}

}  // namespace test
