#pragma once

#include "algo/ref/java_evaluate.h"
#include "test/basic.h"
#include "test/bruteforce.h"
#include "test/flat_hashtable.h"
#include "test/observed_board.h"
#include "test/radix_sort.h"
#include "test/structure.h"

namespace test {

inline void harness() {
    basic();
    bruteforce();
    flatHashtable();
    observedBoard();
    radixSort();
    structure();
}

}  // namespace test
