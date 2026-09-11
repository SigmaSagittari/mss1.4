@echo off
setlocal

cd /d "%~dp0"
set "CXX=D:\codeenvset\mingw64\bin\g++.exe"
set "BUILD=build-mingw"
set "OPT=-O3 -flto=auto -fuse-linker-plugin -flto-partition=one -fwhole-program -march=native -mtune=native -fomit-frame-pointer"

if not exist "%BUILD%" mkdir "%BUILD%"

"%CXX%" -std=gnu++20 %OPT% -Wall -Wextra -pedantic -Isrc ^
    src\main.cpp src\test\common.cpp src\test\harness.cpp src\test\basic.cpp src\test\bruteforce.cpp src\test\flat_hashtable.cpp src\test\observed_board.cpp src\test\radix_sort.cpp src\test\structure.cpp ^
    src\algo\observed_board.cpp src\algo\basic.cpp src\algo\structure.cpp ^
    src\algo\shape_solver\shape_solver.cpp src\algo\shape_solver\dfs_solver.cpp src\algo\shape_solver\graph_solver.cpp ^
    src\algo\probability\probability.cpp src\algo\probability\global_solver.cpp src\algo\probability\observe.cpp ^
    src\algo\bruteforce\bruteforce.cpp ^
    -ldbghelp -o "%BUILD%\mss1.3-test.exe"
if errorlevel 1 (
    set "MSS_BUILD_STATUS=1"
    goto :finish
)

"%BUILD%\mss1.3-test.exe"
set "MSS_BUILD_STATUS=%errorlevel%"

:finish
pause
exit /b %MSS_BUILD_STATUS%
