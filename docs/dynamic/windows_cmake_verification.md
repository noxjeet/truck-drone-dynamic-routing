# Verifying the OFF-by-default dynamic CMake option (native Windows)

**Status: NOT YET VERIFIED.** The `TDR_BUILD_DYNAMIC` block (targets `tdr_dynamic`, `tdr_dynamic_experiment`, `test_dyn_core`, `test_dyn_slice`, `test_dyn_dm5`) has never been configured or built with CMake. Everything so far was built on a Linux VM without CMake using `g++ -std=c++20 -O2 -Wall -Wextra -I include`. Do not report CMake as passing until the steps below have been run and the outputs checked.

Requirements: CMake ≥ 3.16, a C++20 compiler (Visual Studio 2022 "Desktop development with C++" or MinGW-w64 g++ ≥ 11). Use an **out-of-tree build directory outside the repo** so the existing `build\` folder and `git status` stay untouched.

## A. Visual Studio 2022 (multi-config generator) — PowerShell
```powershell
cd "C:\Users\jeeta\OneDrive\Desktop\Study Materials\BTP\Code\truck-drone-dynamic-routing"
# 1. default build (dynamic OFF) must still configure and be unchanged
cmake -S . -B "$env:TEMP\tdr-off" -G "Visual Studio 17 2022" -A x64
cmake --build "$env:TEMP\tdr-off" --config Release
# expected: no tdr_dynamic* / test_dyn_* targets exist
cmake --build "$env:TEMP\tdr-off" --config Release --target tdr_dynamic    # must FAIL with "unknown target"

# 2. dynamic ON
cmake -S . -B "$env:TEMP\tdr-dyn" -G "Visual Studio 17 2022" -A x64 -DTDR_BUILD_DYNAMIC=ON
cmake --build "$env:TEMP\tdr-dyn" --config Release
ctest --test-dir "$env:TEMP\tdr-dyn" -C Release --output-on-failure
```
## B. Ninja or MinGW (single-config) — from a "x64 Native Tools" prompt or MSYS2 shell
```powershell
cmake -S . -B "$env:TEMP\tdr-dyn" -G Ninja -DCMAKE_BUILD_TYPE=Release -DTDR_BUILD_DYNAMIC=ON
cmake --build "$env:TEMP\tdr-dyn"
ctest --test-dir "$env:TEMP\tdr-dyn" --output-on-failure
```
## Expected results (to compare against, NOT yet observed under CMake)
* `test_dyn_core` 64 checks, 0 failures; `test_dyn_slice` 166 checks, 0 failures; `test_dyn_dm5` 256 checks, 0 failures (g++ on Linux).
* The static tests (`ctest` also lists them) must give the same results as before the dynamic option existed.
* Golden scenario digest `4f46e73a729dfc3e` (in `test_dyn_slice`) uses `std::log` for Poisson counts, so a different standard library could in principle change it; if that single check fails on MSVC, report it rather than editing the golden value.
* MSVC may emit warnings the g++ `-Wall -Wextra` build did not; they are not errors unless the build fails.

## C. Run the demo from the CMake build
```powershell
$X = "$env:TEMP\tdr-dyn\Release\tdr_dynamic.exe"      # Ninja/MinGW: "$env:TEMP\tdr-dyn\tdr_dynamic.exe"
& $X --n=15 --regime=F --rho=0.8 --w-tilde=1.5 --eps-tilde=2 --scan=g3 --admission=apm --vns-iters=150 --scenarios=1 --omega=900 --sigma=0 --trace
```
Compare with `docs/dynamic/demo/trace_A.txt` (Linux/g++). Plan numbers depend on the static VNS (floating point and library behaviour) — if the Windows trace differs, report the first differing line instead of assuming either is wrong. Bash users can use `docs/dynamic/run_demo.sh <binary> [A|B]`.

## If something fails
Record the exact command, the first error, and the compiler version; do not change static sources. Typical candidates: missing `<cstdint>`/`<cmath>` includes under MSVC, `M_PI`-style macros, or a long-path problem in OneDrive (build in `C:\tmp\...` instead of `%TEMP%`).
