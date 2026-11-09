# Static reproduction: experiment runner, result writer, configuration and CLI (M8)

Instruction of 9 Nov 2026, 14:34 IST ("runner, result writer, configuration wiring and CLI only"). M8 adds no algorithm: it wires the existing M1--M7 components to a configuration file, runs experiments, writes machine-readable results and exposes a command line. VNS and operator behaviour is unchanged (checked, section 6). Categories as elsewhere: **[PAPER]** stated by Han, Chi and Sun 2026; **[DECISION]** approved by the user; **[ASSUMPTION]** our choice awaiting approval.

## 1. Scope and separation

| Piece | File | Does | Does not |
|---|---|---|---|
| Configuration | `include/config/experiment_config.hpp` | one field table (key, getter, setter) drives parsing, `--set`, the canonical dump and `list-keys`; strict validation | contain algorithm defaults: those are the members' own initialisers (`StaticConfig`, `VnsOptions`, `ShakeOptions`, `N4Options`, `KMeansPolicy`) |
| Runner | `include/runner/static_runner.hpp` | per dataset: instance, initial solution (F1--F3), then `runs` VNS runs through the unchanged `run_vns`; verifies every returned best solution | touch files or the console |
| Writer | `include/runner/result_writer.hpp` | renders `results.json`, `runs.csv`, `traces.csv`, `timing.json`, `manifest.json`, the resolved config; writes via temp file + rename; refuses to overwrite | run the algorithm |
| CLI | `include/runner/cli.hpp`, `apps/tdr_static.cpp` | argument parsing, configuration precedence, exit codes; `cli_main(args, out, err)` is testable in-process | contain logic beyond wiring |

New CMake targets: `tdr_static`, `test_experiment_config`, `test_runner_cli`. Existing sources changed: none (`CMakeLists.txt` only); `vns.hpp` gained the M7-closure reporting fields (`vns.md` section 10).

## 2. Configuration

Format: INI-like text, `key = value`, optional `[section]` headers (`[vns]` + `iter_max = 5` is `vns.iter_max = 5`), `#` or `;` comments. 48 keys (`tdr_static list-keys` prints all with defaults): `experiment.*`, `output.*`, `seed.base_seed`, `instance.*`, `vehicle.*`, `flp.*`, `neighbourhood.*`, `initial.*`, `vns.*`, `shake.*`, `n4.*`. Strict: unknown keys, duplicate keys, malformed or non-finite numbers, negative or fractional integers, bad enum names and bad operator lists are errors, all reported with their line number in one pass; a rejected value leaves the field unchanged. Precedence: built-in defaults < `--config FILE` < `--set key=value` (later `--set` wins). The complete resolved configuration is written to `config.resolved.cfg` (all keys, canonical order, doubles `%.17g`) and is itself a valid `--config` file: `parse(dump(c))` reproduces `c` exactly (tested field by field).

Wiring (single point, `effective_vns_options`): `seed.base_seed` -> VNS base seed; `neighbourhood.n1_repeats / n2_repeats` -> the VNS list; `neighbourhood.adjacency_window` -> `SearchContext`; `output.record_trace` -> `VnsOptions::record_trace`; everything else passes through. Per run the runner sets only `run_index = first_run_index + r`.

Moved into the configuration (M1 follow-up "TSP heuristic seed and k-means constants from config in M8"): `initial.tsp_max_exact_n = 21`, `initial.tsp_heuristic_restarts = 8`, `initial.tsp_heuristic_seed = 42`, `initial.kmeans_max_iterations`, `initial.kmeans_convergence_epsilon` (KMeansPolicy defaults). **The defaults equal the values that were hard-coded before, so every initial solution is unchanged** (tested against `TSPSolverFactory{}`).

## 3. Experiment protocol

`experiment.num_datasets` datasets (indices `instance.dataset_index` ...), each with `experiment.runs` independent VNS runs (run indices `experiment.first_run_index` ...). Default 1 dataset x 10 runs (the 10-run protocol named in the M0 plan; the paper's four datasets per size are a setting, not a default, and the run count is a default of ours, not a paper fact). Every run starts from the same initial solution; its seeds depend only on (base seed, stream, run index). Hence run r of an experiment is bit-identical to the one-run experiment with `first_run_index = r`, and dataset d to the single experiment with `dataset_index = d` (both tested).

## 4. Outputs (schema `tdr.static.results/2`, field reference in `output_schema.md`)

* `config.resolved.cfg` -- see section 2.
* `results.json` -- `experiment` (name, runs, datasets, indices, base seed, `config_digest` = FNV-1a 64 of the resolved config, `deterministic`, `ok`, `errors`); per dataset: instance seed, customers, FLP candidates, `initial` (feasibility, objective, trips, diagnostics, solution), `runs[]`, `summary` (runs_ok, best / mean / worst-of-best objective, best run index, sample std, mean improvement %). Per run: status, stop reason, `verified`, initial / best / final-current objective, `improvement_pct`, iterations, T0 and final T, best updates, improving and SA-accepted moves, **`passes`** (`completed`, `skipped`, `guard_truncated`, `guard_triggered`, `local_steps_total`, `max_local_steps_in_pass` -- the A38 reporting), `shakes` (applied, NotApplicable and infeasible redraws, by kind), `operators[8]` (calls, no anchor, no candidate, improving, SA accepted / rejected), `num_trips`, `best_solution` (truck route and drone trips with node ids; FLP ids refer to the eager pool of the instance). Objectives are in seconds with minutes alongside (`*_s`, `*_min`, D10).
* `runs.csv` -- one row per (dataset, run) with the headline numbers; `traces.csv` (only with `output.record_trace`) -- per pass: temperature, shake kind, pass outcome (`completed` / `skipped` / `guard-truncated`), local steps, moves, current and best objective.
* `timing.json` -- wall-clock seconds, the **only** non-deterministic file. `manifest.json` -- file sizes and FNV-1a 64 digests of the deterministic files (`not-reproducible` for `timing.json`).
* Numbers are printed round-trip exact (`%.17g`; non-finite -> `null`); line ends are `\n`; files are written to `<name>.tmp` then renamed; an existing `results.json` is not replaced without `--overwrite`; a stale `traces.csv` is removed when a rerun records no trace.

## 5. CLI

```
tdr_static run          --out DIR [--config FILE] [--set key=value]... [--overwrite] [--quiet]
tdr_static check        [--config FILE] [--set key=value]...     validate, print the resolved configuration
tdr_static print-config [--config FILE] [--set key=value]...
tdr_static list-keys | help | version
```
Exit codes: 0 success; 1 the experiment ran but reported errors (infeasible initial solution, failed verification); 2 usage or configuration error (nothing written); 4 output could not be written. Reproduce an experiment: `tdr_static run --config <dir>/config.resolved.cfg --out <new dir>`; `results.json`, `runs.csv`, `traces.csv` and `config.resolved.cfg` come out byte-identical (tested end to end).

## 6. Validation (details: `test_and_validation.md` section 14)

`test_experiment_config` 25 assertions; `test_runner_cli` 76 assertions (24 and 63 at the first delivery, extended by the M8 closure); the 21 earlier programs unchanged (23 of 23 in a fresh Release build with 0 warnings and under ASan + UBSan for the new programs); M8 mutation 44 of 44 killed (after six test-strengthening iterations); **behaviour preserved**: the runner's runs are bit-identical (trace, best, current, counters) to direct `run_vns` calls, and the CLI protocol runs reproduce the M7 `bench_vns` default numbers exactly (section 7).

## 7. Measurements (Release -O3, base seed 31415, dataset 0, 10 runs x 2000 passes through `tdr_static`; raw: `m8_cli_protocol_raw.txt`)

| n | mean gain over the initial solution | best / worst-of-best (s) | wall per run | whole experiment | results.json |
|---|---|---|---|---|---|
| 10 | 14.63 % | 57,198.8 / 58,295.4 | 0.21 s | 2.1 s | 36.5 kB |
| 20 | 19.58 % | 145,626.4 / 165,288.3 | 0.28 s | 4.0 s | 37.4 kB |
| 50 | 20.54 % | 595,980.0 / 620,465.9 | 1.18 s | 11.9 s | 41.6 kB |

These equal the M7 `bench_vns` default figures (14.63 % and 19.58 %; at n = 50 `bench_vns 50 2000 10 1 default` re-measured now: 20.54 % mean, 18.70--21.91 % range, 1.13 s per run). The runner adds no measurable overhead (setup and writing are milliseconds against seconds of search). No algorithm default was changed or tuned in M8: `list-keys` prints exactly the M7 defaults.

## 8. Assumptions requiring approval (A39--A44, implementation choices)

* **A39** Configuration is a flat INI-like text format with a strict parser (no JSON/YAML dependency); overrides by `--set`.
* **A40** Experiment protocol: `num_datasets` x `runs`, run indices from `first_run_index`; defaults 1 x 10; each run restarts from the dataset's initial solution.
* **A41** F1/F2 constants are configuration keys with their former hard-coded values as defaults; the heuristic TSP seed stays 42 and is **not** derived from `seed.base_seed` (as before M8), so the base seed changes the instance and the VNS streams but not the TSP tie-breaking.
* **A42** Output set, schema and determinism contract of section 4 (timing separated; digests; atomic write; overwrite protection).
* **A43** At configuration level `shake.max_segment_length` and `shake.max_shift` must be >= 1 (the M6 code treats smaller values as 1; the algorithm is unchanged).
* **A44** Every run's best solution is re-verified (every customer served exactly once; the production evaluator accepts it with exactly the stated objective); a failure makes the experiment report errors and exit code 1.

## 9. Limitations and open points

* (Closed at the M8 sign-off) The exit-code-1 path is now covered by a CLI test (preflight: non-finite travel times); Windows is covered by cross-built binaries under Wine, not natively (`windows_validation.md`).
* No result comparison with the paper's tables, no multi-size sweep driver and no plotting: the paper comparison was done in M9 outside the runner (`validation_vs_paper.md`; the sweep is `scripts/m9_run_all.sh`). `vns.max_seconds > 0` makes a run wall-clock dependent; the results then say `deterministic: false`.
* A39--A44 approved; M8 signed off (9 Nov 2026, 15:35 IST).
