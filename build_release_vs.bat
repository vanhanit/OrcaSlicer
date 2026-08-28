@REM OrcaSlicer build script for Windows with VS auto-detect
@REM
@REM By default, auto-detects the latest installed Visual Studio (2019/2022/2026)
@REM with the "Desktop development with C++" workload and uses its generator.
@REM Pass vs2019, vs2022, or vs2026 as an argument to force a specific toolchain
@REM instead, e.g.:
@REM   build_release_vs.bat vs2022
@REM   build_release_vs.bat vs2026 arm64 debug
@echo off
set WP=%CD%
set _START_TIME=%TIME%

@REM Default target architecture to the host CPU arch; override by passing
@REM "x64" or "arm64" as an argument. PROCESSOR_ARCHITEW6432 covers a 32-bit
@REM shell running on a 64-bit OS, where PROCESSOR_ARCHITECTURE reads "x86".
set arch=x64
if /I "%PROCESSOR_ARCHITECTURE%"=="ARM64" set arch=ARM64
if /I "%PROCESSOR_ARCHITEW6432%"=="ARM64" set arch=ARM64
if /I "%1"=="arm64" set arch=ARM64
if /I "%2"=="arm64" set arch=ARM64
if /I "%1"=="x64" set arch=x64
if /I "%2"=="x64" set arch=x64

@REM Check for Ninja Multi-Config option (-x)
set USE_NINJA=0
for %%a in (%*) do (
    if "%%a"=="-x" set USE_NINJA=1
)

@REM Optional toolchain override: pass vs2019, vs2022, or vs2026 to force that
@REM generator instead of auto-detecting the latest installed one.
set VS_OVERRIDE=
for %%a in (%*) do (
    if /I "%%a"=="vs2019" set VS_OVERRIDE=16
    if /I "%%a"=="vs2022" set VS_OVERRIDE=17
    if /I "%%a"=="vs2026" set VS_OVERRIDE=18
)

@REM Check for clang-cl option (-l). Combined with -x it also builds the deps with
@REM clang-cl; on the Visual Studio generator it applies to the slicer only, because
@REM the dependency sub-builds have no toolset to inherit and stay on MSVC.
set CLANG_ARG=
set TOOLSET_ARG=
for %%a in (%*) do (
    if "%%a"=="-l" (
        set CLANG_ARG=-DCMAKE_C_COMPILER=clang-cl -DCMAKE_CXX_COMPILER=clang-cl
        set TOOLSET_ARG=-T ClangCL
    )
)

@REM Check for unit-tests option ("tests")
set BUILD_TESTS=OFF
for %%a in (%*) do (
    if /I "%%a"=="tests" set BUILD_TESTS=ON
)

if "%USE_NINJA%"=="1" (
    echo Using Ninja Multi-Config generator
    set CMAKE_GENERATOR="Ninja Multi-Config"
    set VS_VERSION=Ninja
    goto :generator_ready
)

if defined VS_OVERRIDE set VS_MAJOR=%VS_OVERRIDE%
if defined VS_OVERRIDE echo Toolchain override requested: VS_MAJOR=%VS_MAJOR%
if defined VS_OVERRIDE goto :map_generator

@REM Detect the latest installed Visual Studio with the C++ workload via
@REM vswhere, which -latest sorts by version so this always picks the newest
@REM instance regardless of what happens to be first on PATH.
set VS_MAJOR=
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if exist "%VSWHERE%" (
    echo Detecting latest installed Visual Studio via vswhere...
    for /f "usebackq tokens=1 delims=." %%i in (`"%VSWHERE%" -latest -prerelease -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationVersion`) do set VS_MAJOR=%%i
)

if not "%VS_MAJOR%"=="" (
    echo Detected latest installed Visual Studio: major version %VS_MAJOR%
    goto :map_generator
)

@REM Fallback: detect the version via msbuild already on PATH (e.g. when run
@REM from a Developer Command Prompt and vswhere isn't found).
echo vswhere unavailable or found no instance with the C++ workload; falling back to msbuild on PATH...
for /f "tokens=*" %%i in ('msbuild -version 2^>^&1 ^| findstr /r "^[0-9][0-9]*\.[0-9][0-9]*\.[0-9][0-9]*"') do (
    for /f "tokens=1 delims=." %%a in ("%%i") do set VS_MAJOR=%%a
    set MSBUILD_OUTPUT=%%i
    goto :version_found
)

@REM Alternative method for newer MSBuild versions
if "%VS_MAJOR%"=="" (
    for /f "tokens=*" %%i in ('msbuild -version 2^>^&1 ^| findstr /r "[0-9][0-9]*\.[0-9][0-9]*\.[0-9][0-9]*"') do (
        for /f "tokens=1 delims=." %%a in ("%%i") do set VS_MAJOR=%%a
        set MSBUILD_OUTPUT=%%i
        goto :version_found
    )
)

:version_found
if defined MSBUILD_OUTPUT echo MSBuild version detected: %MSBUILD_OUTPUT%

if "%VS_MAJOR%"=="" (
    echo Error: Could not determine an installed Visual Studio version.
    echo Install VS2019/2022/2026 with the "Desktop development with C++" workload,
    echo or pass vs2019/vs2022/vs2026 as an argument to force a specific toolchain.
    exit /b 1
)

:map_generator
if "%VS_MAJOR%"=="16" (
    set VS_VERSION=2019
    set CMAKE_GENERATOR="Visual Studio 16 2019"
) else if "%VS_MAJOR%"=="17" (
    set VS_VERSION=2022
    set CMAKE_GENERATOR="Visual Studio 17 2022"
) else if "%VS_MAJOR%"=="18" (
    set VS_VERSION=2026
    set CMAKE_GENERATOR="Visual Studio 18 2026"
) else (
    echo Error: Unsupported Visual Studio version: %VS_MAJOR%
    echo Supported versions: VS2019 (16.x^), VS2022 (17.x^), VS2026 (18.x^)
    exit /b 1
)

echo Detected Visual Studio %VS_VERSION% (version %VS_MAJOR%)
echo Using CMake generator: %CMAKE_GENERATOR%

:generator_ready

@REM Pack deps
if "%1"=="pack" (
    setlocal ENABLEDELAYEDEXPANSION
    cd %WP%/deps/build
    if "%arch%"=="ARM64" cd %WP%/deps/build-arm64
    for /f "tokens=2-4 delims=/ " %%a in ('date /t') do set build_date=%%c%%b%%a
    echo packing deps: OrcaSlicer_dep_win-!arch!_!build_date!_vs!VS_VERSION!.zip

    %WP%/tools/7z.exe a OrcaSlicer_dep_win-!arch!_!build_date!_vs!VS_VERSION!.zip OrcaSlicer_dep
    goto :done
)

set debug=OFF
set debuginfo=OFF
if "%1"=="debug" set debug=ON
if "%2"=="debug" set debug=ON
if "%1"=="debuginfo" set debuginfo=ON
if "%2"=="debuginfo" set debuginfo=ON
if "%debug%"=="ON" (
    set build_type=Debug
    set build_dir=build-dbg
) else (
    if "%debuginfo%"=="ON" (
        set build_type=RelWithDebInfo
        set build_dir=build-dbginfo
    ) else (
        set build_type=Release
        set build_dir=build
    )
)
if "%arch%"=="ARM64" set build_dir=%build_dir%-arm64
echo build type set to %build_type%, arch=%arch%

setlocal DISABLEDELAYEDEXPANSION
cd deps
mkdir %build_dir%
cd %build_dir%
set "SIG_FLAG="
if defined ORCA_UPDATER_SIG_KEY set "SIG_FLAG=-DORCA_UPDATER_SIG_KEY=%ORCA_UPDATER_SIG_KEY%"

if "%1"=="slicer" (
    GOTO :slicer
)
echo "building deps.."
if defined CLANG_ARG if "%USE_NINJA%"=="0" echo Note: -l needs -x for the dependencies; building them with MSVC.

echo on
REM Set minimum CMake policy to avoid <3.5 errors
set CMAKE_POLICY_VERSION_MINIMUM=3.5
if "%USE_NINJA%"=="1" (
    cmake ../ -G %CMAKE_GENERATOR% %CLANG_ARG% -DCMAKE_BUILD_TYPE=%build_type%
    cmake --build . --config %build_type% --target deps
) else (
    cmake ../ -G %CMAKE_GENERATOR% -A %arch% -DCMAKE_BUILD_TYPE=%build_type%
    cmake --build . --config %build_type% --target deps -- -m
)
@echo off

if "%1"=="deps" goto :done

:slicer
echo "building Orca Slicer..."
cd %WP%
mkdir %build_dir%
cd %build_dir%

echo on
set CMAKE_POLICY_VERSION_MINIMUM=3.5
if "%USE_NINJA%"=="1" (
    cmake .. -G %CMAKE_GENERATOR% %CLANG_ARG% -DORCA_TOOLS=ON %SIG_FLAG% -DBUILD_TESTS=%BUILD_TESTS% -DCMAKE_BUILD_TYPE=%build_type%
    cmake --build . --config %build_type% --target all
) else (
    cmake .. -G %CMAKE_GENERATOR% -A %arch% %TOOLSET_ARG% -DORCA_TOOLS=ON %SIG_FLAG% -DBUILD_TESTS=%BUILD_TESTS% -DCMAKE_BUILD_TYPE=%build_type%
    cmake --build . --config %build_type% --target ALL_BUILD -- -m
)
@echo off
cd ..
call scripts/run_gettext.bat
cd %build_dir%
cmake --build . --target install --config %build_type%

:done
@echo off
for /f "tokens=1-3 delims=:.," %%a in ("%_START_TIME: =0%") do set /a "_start_s=%%a*3600+%%b*60+%%c"
for /f "tokens=1-3 delims=:.," %%a in ("%TIME: =0%") do set /a "_end_s=%%a*3600+%%b*60+%%c"
set /a "_elapsed=_end_s - _start_s"
if %_elapsed% lss 0 set /a "_elapsed+=86400"
set /a "_hours=_elapsed / 3600"
set /a "_remainder=_elapsed - _hours * 3600"
set /a "_mins=_remainder / 60"
set /a "_secs=_remainder - _mins * 60"
echo.
echo Build completed in %_hours%h %_mins%m %_secs%s
