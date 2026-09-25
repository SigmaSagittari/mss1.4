cmake_minimum_required(VERSION 3.25)
project(mss LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 去掉 VS 里的 ZERO_CHECK 工程：构建入口统一走 build.bat / cmake 命令。
# 代价：改了 CMakeLists 之后要重新 configure（build.bat 每次都 configure）。
set(CMAKE_SUPPRESS_REGENERATION ON)

add_library(mss STATIC
    src/board/observed_board.cpp
)
target_include_directories(mss PUBLIC src)

# 测试入口：唯一 exe，跑全部用例；带参数时只跑指定名字的用例。
add_executable(mss_tests
    tests/main.cpp
    tests/board_transitions.cpp
)
target_link_libraries(mss_tests PRIVATE mss)

# 两条可执行的门禁：/W4 和 -Wold-style-cast（1.5 的新代码不许再有 C 风格转换）
if(MSVC)
    target_compile_options(mss PRIVATE /W4 /utf-8)
    target_compile_options(mss_tests PRIVATE /W4 /utf-8)
else()
    target_compile_options(mss PRIVATE -Wall -Wextra -Wold-style-cast)
    target_compile_options(mss_tests PRIVATE -Wall -Wextra -Wold-style-cast)
endif()