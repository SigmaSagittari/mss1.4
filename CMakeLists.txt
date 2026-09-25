cmake_minimum_required(VERSION 3.25)
project(mss LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 去掉 VS 里的 ZERO_CHECK 工程：构建入口统一走 build.bat / cmake 命令。
# 代价：改了 CMakeLists 之后要重新 configure（build.bat 每次都 configure）。
set(CMAKE_SUPPRESS_REGENERATION ON)

# 文件清单显式列出（头文件也要列，否则 VS 工程里看不到它们）
set(MSS_SOURCES
    src/version.h
    src/board/observed_board.h
    src/board/observed_board.cpp
    src/core/assert.h
    src/core/types.h
    src/core/utility/grid.h
)
set(MSS_TEST_SOURCES
    tests/harness.h
    tests/main.cpp
    tests/board_transitions.cpp
)

add_library(mss STATIC ${MSS_SOURCES})
target_include_directories(mss PUBLIC src)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/src FILES ${MSS_SOURCES})

# 测试入口：唯一 exe，跑全部用例；带参数时只跑指定名字的用例。
add_executable(mss_tests ${MSS_TEST_SOURCES})
target_link_libraries(mss_tests PRIVATE mss)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/tests FILES ${MSS_TEST_SOURCES})

# 两条可执行的门禁：/W4 和 -Wold-style-cast（1.5 的新代码不许再有 C 风格转换）
if(MSVC)
    target_compile_options(mss PRIVATE /W4 /utf-8)
    target_compile_options(mss_tests PRIVATE /W4 /utf-8)
else()
    target_compile_options(mss PRIVATE -Wall -Wextra -Wold-style-cast)
    target_compile_options(mss_tests PRIVATE -Wall -Wextra -Wold-style-cast)
endif()

# VS 里 F5 默认启动 mss_tests（否则启动项目是 ALL_BUILD，它没有可执行输出）
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT mss_tests)