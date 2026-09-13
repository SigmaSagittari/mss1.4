#pragma once

#include "test/basic.h"
#include "test/bruteforce.h"
#include "test/flat_hashtable.h"
#include "test/observed_board.h"
#include "test/performance.h"
#include "test/real_endgame_performance.h"
#include "test/radix_sort.h"
#include "test/structure.h"

namespace test {

inline void harness() {
    //basic();
    bruteforce();
    //flatHashtable();
    //observedBoard();
    //radixSort();
    //structure();
    //performance();
    //real_endgame_performance(10000, 200000, 600.0);
}

}  // namespace test
