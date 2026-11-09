# Configuration reference (48 keys; frozen at M10)

`tdr_static list-keys` prints the authoritative list with defaults (they are the algorithm structs' own member initialisers, so this page and the code cannot disagree about defaults without a test noticing; `scripts/m10_doc_check.py` compares the key set of this page with `list-keys`). Format, precedence (defaults < `--config` file < `--set key=value`) and validation rules: `runner_cli.md` section 2. Provenance classes: **P** = stated in the paper (Section 6.1 unless noted); **A** = our implementation assumption (the paper is silent), with the decision or assumption number; **O** = operational (no algorithmic content). The M9 protocol changes only `experiment.*`, `seed.base_seed` and `instance.num_customers` (`config/m9/*.cfg`).

| Key | Default | Class | Meaning |
|---|---|---|---|
| `experiment.name` | `static` | O | label stored in the outputs |
| `experiment.num_datasets` | 1 | A (A40) | number of consecutive datasets (instances) from `instance.dataset_index` |
| `experiment.runs` | 10 | P (10 runs per instance) | independent VNS runs per dataset |
| `experiment.first_run_index` | 0 | A (A40) | run index of the first run (selects the VNS and shake seed streams) |
| `output.record_trace` | false | O | write `traces.csv` (one row per pass) |
| `output.write_solutions` | true | O | include solutions in `results.json` |
| `seed.base_seed` | 20261109 | A | the single seed from which instance, VNS and shake streams are derived (`derive_seed(base, stream, index)`) |
| `instance.num_customers` | 10 | P | 10 / 20 / 50 |
| `instance.area_side_m` | 60000 | P | side of the square (60 x 60 km) |
| `instance.depot_placement` | `centre` | A (D9) | `centre`, `corner` or `explicit` |
| `instance.depot_x_m`, `instance.depot_y_m` | 0, 0 | O | used only for `explicit` |
| `instance.dataset_index` | 0 | O | which dataset; selects the instance seed |
| `instance.truck_service_time_s`, `instance.drone_service_time_s` | 30, 30 | P | customer service times |
| `instance.theta` | 1 | P | load coefficient $\theta$ |
| `vehicle.truck_speed_mps`, `vehicle.drone_speed_mps` | 15, 12 | P | constant speeds |
| `vehicle.endurance_s` | 1800 | P | drone endurance (30 min) |
| `vehicle.launch_prep_time_s`, `vehicle.recovery_time_s` | 60, 40 | P | SL, SR (timing rule: SR Option B, user decision of 8 Nov 2026) |
| `vehicle.num_drones` | 2 | P | drones carried by the truck |
| `flp.unit_division_length_m` | 2000 | A (D1/D8) | FLP spacing $d$ of eq. (41); the paper gives 2000 m only for its Section 6.3 sweeps; the count rule is ours; pool sizes differ from the paper's Table 4 |
| `neighbourhood.adjacency_window` | 3 | A (D1) | candidate window ("adjacent") of N1-N8 |
| `neighbourhood.n1_repeats`, `neighbourhood.n2_repeats` | 2, 2 | A (D2) | repetitions of N1 / N2 in the local-search list (paper: "several times") |
| `initial.tsp_max_exact_n` | 21 | A | exact Held-Karp up to this many TSP nodes, heuristic above (substitute for CPLEX) |
| `initial.tsp_heuristic_restarts`, `initial.tsp_heuristic_seed` | 8, 42 | A | heuristic TSP (nearest neighbour + 2-opt/Or-opt); the seed is not derived from the base seed |
| `initial.kmeans_max_iterations`, `initial.kmeans_convergence_epsilon` | 100, 1e-9 | A | k-means details (the paper gives none; $k=\lceil\lvert C\rvert/(\lvert U\rvert+2)\rceil$ is P) |
| `vns.iter_max` | 2000 | A (A33, D7; pilot) | passes per run |
| `vns.t0_fraction`, `vns.t0_absolute` | 0.01, 0 | A (A33, D7; pilot) | $T_0=\max(t_{min}, \mathrm{t0\_fraction}\cdot f(x_0))$ unless `t0_absolute` > 0 |
| `vns.cooling_rate` | 0.995 | A (A31, D7; pilot) | geometric cooling once per pass |
| `vns.t_min` | 1e-9 | A | temperature floor |
| `vns.max_no_improve` | 0 (off) | A (A36) | stop after this many passes without a new best-so-far |
| `vns.max_seconds` | 0 (off) | O | wall-clock limit; makes a run non-reproducible (flagged in the outputs) |
| `vns.max_shake_attempts` | 10 | A (A35) | shake redraws per pass before the pass is skipped |
| `vns.max_local_steps` | 1000 | A (A38) | safety guard per pass; reaching it is always reported (per-pass outcome, counters, `guard_triggered`) |
| `vns.min_improvement` | 0 | A (A37) | improvement iff $f(z) < \mathrm{ref}-\delta$; 0 = strict `<` |
| `vns.reference` | `incumbent` | A (A30) | acceptance reference $f(x)$ (`incumbent` or `shaken`) |
| `vns.neighbourhoods` | `default` | P (order) / A (D2) | local-search order N4, N5, N8, N3, N6, N7 then N1/N2 repeated; or an explicit list |
| `shake.max_segment_length`, `shake.max_shift` | 3, 3 | A (A20-A23) | shake magnitudes (the paper says "several") |
| `n4.mode` | `relocate-and-exchange` | A | N4 candidate set |
| `n4.affected_trips` | `reanchor` | A (M2 final) | policy for trips whose launch FLP becomes stale |
| `n4.max_reanchor_variants` | 100 | A (M2 final) | re-anchoring budget per candidate |

All keys are validated strictly (unknown or duplicate keys, malformed values and out-of-range values are errors with line numbers; nothing is silently defaulted).
