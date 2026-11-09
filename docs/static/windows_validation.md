# Windows validation of the runner, writer and CLI (M8 closure, 9 Nov 2026)

Requested: validate Windows compatibility (filesystem operations, temporary-file rename / overwrite, paths with spaces, CLI execution). The development machine is Windows; the sandbox that runs the build is Linux, so **native Windows validation was not possible from here**. What was done instead, and what remains for you:

## 1. What was validated here (Windows PE binaries under Wine)
* Cross-compiled with Zig 0.17.1 (`zig c++ -target x86_64-windows-gnu -std=c++20`, clang + libc++ + mingw-w64 runtime) the three M8 programs: `tdr_static.exe`, `test_experiment_config.exe`, `test_runner_cli.exe` (source = the repository headers; build in `~/winbuild`; copies are in `Claude outputs\winbuild\` of your repository folder, so you can also run them natively, see section 3).
* Executed them under Wine 9.0 (Windows API emulation on Linux; `Z:\` drive mapping, so paths are real Windows-style paths such as `Z:\tmp\wo\dir with spaces\r1`):
  * `test_experiment_config.exe`: **25 of 25** pass (includes CRLF line ends and a UTF-8 BOM in a configuration file);
  * `test_runner_cli.exe`: **76 of 76** pass, including: paths containing spaces (configuration file and nested output directory), CRLF + BOM configuration file, results replaced through `--overwrite` (rename over an existing file) with no `.tmp` left behind, overwrite refusal and exit code 4, the failing experiment with exit code 1, the schema and the interpretation checks;
  * `tdr_static.exe run --out "Z:\tmp\wo\dir with spaces\r1" ...`: exit 0; a second run without `--overwrite`: exit 4; with `--overwrite`: exit 0; a failing configuration: exit 1.
* **Cross-platform reproducibility:** the same experiment (10 customers, 3 runs, 300 passes, seed 31415, traces on) run by the Windows binary (clang/libc++, `-O1`) and by the Linux binary (g++, `-O3`) produced **byte-identical** `results.json`, `runs.csv`, `traces.csv` and `config.resolved.cfg`:

| file | SHA-256 |
|---|---|
| results.json | `2ad66977cd6b85d1236261d1c4ab72cb1c02b35c49630a5b82f5c30a98f0b388` |
| runs.csv | `68caa2afb3041451967d55468e041faba74d1eac21196727a62346a715f0f50c` |
| traces.csv | `2dbb8e5c9d0ffcca61e5269b467040d7e266bb9330a9168c78b9120dcdddde3a` |
| config.resolved.cfg | `3d61a0868dc1c1d1dff74b4f1bb1c7f78502092ba47168ce7633cc2d6959e783` |

Code-level Windows hygiene verified by reading the writer: files are opened in binary mode (no CRLF translation, so digests are platform independent); the temporary file is renamed over the destination and, if the rename is refused, the destination is removed and the rename retried; configuration files are read in binary mode and tolerate CRLF and a BOM (new in this closure: BOM stripping + test).

## 2. Limitations (recorded, not hidden)
* Wine is not Windows: NTFS semantics (sharing violations when another process, e.g. an antivirus scanner or a spreadsheet, holds `results.json` open; OneDrive-synchronised folders; long paths over 260 characters; case-insensitive names) were **not** exercised. Your repository sits in OneDrive: if a rename fails there, the program reports exit code 4 with the path (it does not retry more than once).
* MSVC was not tried (clang/libc++ and g++/libstdc++ were); the headers use only standard C++20 and `std::filesystem`, but an MSVC build is unverified.
* Non-ASCII characters in paths or names: `std::filesystem::path` built from a narrow string uses the ANSI code page on Windows; not tested.
* The Zig-built binaries are unsigned; Windows SmartScreen may ask for confirmation.

## 3. Exact PowerShell commands for you (native Windows check)
Run in a normal PowerShell (the binaries need nothing installed):

```powershell
cd "C:\Users\jeeta\OneDrive\Desktop\Study Materials\BTP\Code\truck-drone-dynamic-routing"
$b = ".\Claude outputs\winbuild"
Get-FileHash "$b\*.exe" -Algorithm SHA256 | Select-Object Hash, Path     # expected: see the staging record in test_and_validation.md section 15 (tdr_static 5faee7f3..., test_experiment_config 23a3a85d..., test_runner_cli ccea8f1a...)
& "$b\test_experiment_config.exe" | Select-Object -Last 2                # expect: 25 passed, 0 failed / ALL TESTS PASSED
& "$b\test_runner_cli.exe"        | Select-Object -Last 2                # expect: 76 passed, 0 failed / ALL TESTS PASSED
```

Reproduction check against the table above (paths with spaces, overwrite, exit codes):

```powershell
$out = Join-Path $env:TEMP "tdr check\run 1"
Remove-Item -Recurse -Force (Join-Path $env:TEMP "tdr check") -ErrorAction SilentlyContinue
& "$b\tdr_static.exe" run --out $out --set instance.num_customers=10 --set experiment.runs=3 --set vns.iter_max=300 --set seed.base_seed=31415 --set output.record_trace=true --quiet
"exit code: $LASTEXITCODE"                                                # expect 0
Get-FileHash "$out\results.json","$out\runs.csv","$out\traces.csv","$out\config.resolved.cfg" -Algorithm SHA256 | Select-Object Hash, @{n="File";e={Split-Path $_.Path -Leaf}}   # compare with the table (case-insensitive)
& "$b\tdr_static.exe" run --out $out --set instance.num_customers=10 --quiet ; "exit code: $LASTEXITCODE"     # expect 4 (already exists)
& "$b\tdr_static.exe" run --out $out --set instance.num_customers=10 --set experiment.runs=1 --set vns.iter_max=5 --overwrite --quiet ; "exit code: $LASTEXITCODE"   # expect 0
Get-ChildItem $out | Select-Object Name                                   # expect no *.tmp
& "$b\tdr_static.exe" run --out "$out failing" --set instance.num_customers=10 --set vehicle.truck_speed_mps=3e-308 --quiet ; "exit code: $LASTEXITCODE"      # expect 1
# the OneDrive case: write inside the synchronised repository folder
& "$b\tdr_static.exe" run --out ".\results\win check\r1" --set instance.num_customers=10 --set experiment.runs=2 --set vns.iter_max=50 --quiet ; "exit code: $LASTEXITCODE"
& "$b\tdr_static.exe" run --out ".\results\win check\r1" --set instance.num_customers=10 --set experiment.runs=2 --set vns.iter_max=50 --overwrite --quiet ; "exit code: $LASTEXITCODE"   # expect 0 (or 4 with a message naming the locked file)
```

Native build from source (Visual Studio Developer PowerShell, or MinGW-w64 g++ on PATH):

```powershell
cmake -S . -B build-win -DTDR_BUILD_BENCH=OFF
cmake --build build-win --config Release
ctest --test-dir build-win -C Release --output-on-failure                  # expect 23 of 23
```
If this fails to compile under MSVC, please send the first error: it would be an M8 portability defect to fix.
