@echo off
setlocal
rem Single build entry. Build definition lives in CMakeLists.txt / CMakePresets.json.
rem Usage:
rem   build.bat                             -> mingw preset, run all cases
rem   build.bat vs                          -> VS 2026 preset, run all cases
rem   build.bat mingw board_transitions     -> run one case
rem   set MSS_NOPAUSE=1                     -> no pause at the end (for automation)

set "PRESET=%~1"
if "%PRESET%"=="" set "PRESET=mingw"
shift

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" goto :no_vswhere

for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -property installationPath`) do set "VSPATH=%%i"
set "CMAKE=%VSPATH%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
if not exist "%CMAKE%" goto :no_cmake

pushd "%~dp0"

"%CMAKE%" --preset %PRESET%
if errorlevel 1 goto :fail

"%CMAKE%" --build --preset %PRESET%
if errorlevel 1 goto :fail

set "OUT=%~dp0..\mss1.5-build\%PRESET%"
if /i "%PRESET%"=="mingw" set "EXE=%OUT%\mss_tests.exe"
if /i "%PRESET%"=="vs" set "EXE=%OUT%\Release\mss_tests.exe"
if not exist "%EXE%" goto :no_exe

echo.
"%EXE%" %1 %2 %3 %4
set "STATUS=%errorlevel%"
popd
if not defined MSS_NOPAUSE pause
exit /b %STATUS%

:fail
popd
echo [build] configure or build failed
exit /b 1

:no_exe
popd
echo [build] test binary not found: %EXE%
exit /b 1

:no_vswhere
echo [build] vswhere not found: %VSWHERE%
exit /b 1

:no_cmake
echo [build] cmake not found: %CMAKE%
exit /b 1