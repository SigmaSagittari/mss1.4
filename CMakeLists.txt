cmake_minimum_required(VERSION 3.20)
project(mss LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(mss STATIC
    src/board/observed_board.cpp
)
target_include_directories(mss PUBLIC src)

add_executable(mss_tests tests/board_transitions.cpp)
target_link_libraries(mss_tests PRIVATE mss)

# 两条可执行的门禁：/W4 和 -Wold-style-cast（1.5 的新代码不许再有 C 风格转换）
if(MSVC)
    target_compile_options(mss PRIVATE /W4 /utf-8)
    target_compile_options(mss_tests PRIVATE /W4 /utf-8)
else()
    target_compile_options(mss PRIVATE -Wall -Wextra -Wold-style-cast)
    target_compile_options(mss_tests PRIVATE -Wall -Wextra -Wold-style-cast)
endif()