@echo off
setlocal

cd /d "%~dp0"
set "CXX=D:\codeenvset\mingw64\bin\g++.exe"
set "BUILD=build-mingw"
set "OPT=-O3 -flto=auto -fuse-linker-plugin -flto-partition=one -fwhole-program -march=native -mtune=native -fomit-frame-pointer"

if not exist "%BUILD%" mkdir "%BUILD%"

"%CXX%" -std=gnu++20 %OPT% -Wall -Wextra -pedantic -Isrc src\main.cpp ^
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
