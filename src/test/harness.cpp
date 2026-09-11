#include "test/harness.h"

#include "test/basic.h"
#include "test/bruteforce.h"
#include "test/flat_hashtable.h"
#include "test/observed_board.h"
#include "test/radix_sort.h"
#include "test/structure.h"

namespace test {

void harness() {
    basic();
    bruteforce();
    flatHashtable();
    observedBoard();
    radixSort();
    structure();
    // check(false, "intentional fail test");
}

}  // namespace test
