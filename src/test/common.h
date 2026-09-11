#pragma once

#include <source_location>

namespace test {

void check(bool condition, const char* errmsg,
           std::source_location location = std::source_location::current());

}  // namespace test
