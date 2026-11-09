# DM6 demonstrable dynamic experiment (identity optimiser)

**Status: demo evidence, not a study.** Counts are means per scenario over 10 paired scenarios (identical seeds and scenario digests in every arm; checked by the driver). One instance (n0=20, dataset/seed defaults, regime P, static-VNS base plan with 150 VNS iterations, base objective 157839). The optimiser is the identity (arm B1): **no optimisation improvement is measured or claimed.** W̃, ε̃, ς are unfrozen placeholders (DM0 §7.2 grid-placement rule not yet applied). Conventions C1–C3 are unchanged.

Reproduce: `docs/dynamic/run_dm6_demo.sh <tdr_dynamic_experiment binary> <out dir>` (CMake target `tdr_dynamic_experiment` inside the OFF-by-default `TDR_BUILD_DYNAMIC` block, **still unverified with CMake**; built here with g++ by hand). `experiment.csv`/`summary.csv`/`config.txt` are byte-identical on replay (verified for E1); `runtime.csv` (wall clock) is not deterministic and is kept apart. Per-arm cost proxy that *is* deterministic: evaluator calls.

Arms: **B0** scan none (reject-all, demand baseline); **G0/A0** all-area scan, no promises (baseline, unchanged); **G0/APM**, **G3/APM** (finite W, E; U1 chain); **G3c/APM** = G3 ∩ tube(ς·ℓ), ς=1, **explicitly non-conservative** (a demand restriction, not a filter).

## E1: n0=20, ρ=1, ω=60 s, W̃=2 (W=8000 s), ε̃ ∈ {0.5, 2, 8}
| arm | out-of-scan | expired | stale | rej-no-ins | rej-promise | accepted | acc/generated | acc/considered | mean queue wait (s) | evaluator calls | area frac. (base plan, τ=0) | mean Δobjective per accepted |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| B0 | 21.40 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.000 | 0.000 | 0.0 | 0 | 0.0000 | 0 |
| G0/A0 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 21.40 | 1.000 | 1.000 | 0.9 | 379.8 | 1.0000 | 20848 |
| G0/APM(eps=0.5) | 0.00 | 0.00 | 0.00 | 0.00 | 13.90 | 7.50 | 0.350 | 0.350 | 0.9 | 404.1 | 1.0000 | 16377 |
| G3/APM(eps=0.5) | 0.00 | 0.00 | 0.00 | 0.00 | 13.90 | 7.50 | 0.350 | 0.350 | 0.9 | 404.1 | 1.0000 | 16377 |
| G3c/APM(eps=0.5) | 7.70 | 0.00 | 0.00 | 0.00 | 7.40 | 6.30 | 0.294 | 0.460 | 1.0 | 280.4 | 0.9407 | 15161 |
| G0/APM(eps=2) | 0.00 | 0.00 | 0.00 | 0.00 | 11.70 | 9.70 | 0.453 | 0.453 | 0.9 | 425.2 | 1.0000 | 16946 |
| G3/APM(eps=2) | 0.00 | 0.00 | 0.00 | 0.00 | 11.70 | 9.70 | 0.453 | 0.453 | 0.9 | 425.2 | 1.0000 | 16946 |
| G3c/APM(eps=2) | 7.70 | 0.00 | 0.00 | 0.00 | 5.70 | 8.00 | 0.374 | 0.584 | 1.0 | 281.5 | 0.9407 | 15325 |
| G0/APM(eps=8) | 0.00 | 0.00 | 0.00 | 0.00 | 13.30 | 8.10 | 0.379 | 0.379 | 0.9 | 549.2 | 1.0000 | 23104 |
| G3/APM(eps=8) | 0.00 | 0.00 | 0.00 | 0.00 | 13.30 | 8.10 | 0.379 | 0.379 | 0.9 | 549.2 | 1.0000 | 23104 |
| G3c/APM(eps=8) | 5.10 | 0.00 | 0.00 | 0.00 | 7.80 | 8.50 | 0.397 | 0.521 | 1.1 | 410.7 | 0.9407 | 22628 |

## E2: as E1 but W̃=8 (window effectively slack; isolates the tolerance E)
| arm | out-of-scan | expired | stale | rej-no-ins | rej-promise | accepted | acc/generated | acc/considered | mean queue wait (s) | evaluator calls | area frac. (base plan, τ=0) | mean Δobjective per accepted |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| B0 | 21.40 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.000 | 0.000 | 0.0 | 0 | 0.0000 | 0 |
| G0/A0 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 21.40 | 1.000 | 1.000 | 0.9 | 379.8 | 1.0000 | 20848 |
| G0/APM(eps=0.5) | 0.00 | 0.00 | 0.00 | 0.00 | 0.60 | 20.80 | 0.972 | 0.972 | 0.9 | 476.3 | 1.0000 | 27240 |
| G3/APM(eps=0.5) | 0.00 | 0.00 | 0.00 | 0.00 | 0.60 | 20.80 | 0.972 | 0.972 | 0.9 | 476.3 | 1.0000 | 27240 |
| G3c/APM(eps=0.5) | 3.60 | 0.00 | 0.00 | 0.00 | 0.00 | 17.80 | 0.832 | 1.000 | 1.0 | 348.5 | 0.9407 | 23570 |
| G0/APM(eps=2) | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 21.40 | 1.000 | 1.000 | 0.9 | 429.7 | 1.0000 | 23798 |
| G3/APM(eps=2) | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 21.40 | 1.000 | 1.000 | 0.9 | 429.7 | 1.0000 | 23798 |
| G3c/APM(eps=2) | 4.50 | 0.00 | 0.00 | 0.00 | 0.00 | 16.90 | 0.790 | 1.000 | 1.1 | 300 | 0.9407 | 20099 |
| G0/APM(eps=8) | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 21.40 | 1.000 | 1.000 | 0.9 | 410.4 | 1.0000 | 21741 |
| G3/APM(eps=8) | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 21.40 | 1.000 | 1.000 | 0.9 | 410.4 | 1.0000 | 21741 |
| G3c/APM(eps=8) | 5.00 | 0.00 | 0.00 | 0.00 | 0.00 | 16.40 | 0.766 | 1.000 | 1.0 | 277.3 | 0.9407 | 18178 |

## E3: ρ=2, ω=600 s, W̃=1.5, ε̃=2 (long decisions: queue wait, expiry, stale)
| arm | out-of-scan | expired | stale | rej-no-ins | rej-promise | accepted | acc/generated | acc/considered | mean queue wait (s) | evaluator calls | area frac. (base plan, τ=0) | mean Δobjective per accepted |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| B0 | 42.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.000 | 0.000 | 0.0 | 0 | 0.0000 | 0 |
| G0/A0 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 42.00 | 1.000 | 1.000 | 4032.0 | 846 | 1.0000 | 25997 |
| G0/APM(eps=2) | 0.00 | 5.30 | 8.20 | 0.00 | 23.70 | 4.80 | 0.114 | 0.114 | 2930.5 | 524.7 | 1.0000 | 14422 |
| G3/APM(eps=2) | 2.20 | 4.60 | 6.70 | 0.00 | 23.70 | 4.80 | 0.114 | 0.121 | 2852.8 | 524.7 | 1.0000 | 14422 |
| G3c/APM(eps=2) | 17.40 | 0.00 | 0.00 | 0.00 | 16.80 | 7.80 | 0.186 | 0.317 | 1300.1 | 480.6 | 0.9407 | 16224 |

## Reading the tables (and what not to conclude)
* **Baseline:** G0/A0 accepts everything in E1/E2 (expected: no promise, end insertion always feasible). E3 shows the same with queue waits of ~4000 s, which is only harmless because nothing is promised.
* **Promise rejections are driven mostly by W in E1** (compare E2: with W̃=8 almost nothing is rejected, even at ε̃=0.5). Acceptance is **not monotone in ε̃** (0.35 → 0.45 → 0.38 in E1): decisions are path dependent (each acceptance changes later slack and the cut), so single-instance trends in ε̃ are not evidence of an effect.
* **G3 = G0 in E1/E2 outcome-for-outcome** (as documented: tolerance-only G3 is the whole area while the end gap is editable). In E3 G3 drops ~2 of 42 requests as out-of-scan because no gap was editable at arrival+ω (empty region); G0 classifies the same situation as stale at decision time, so the *totals* are comparable only as in-scan vs decided counts. The area-fraction column is measured on the base plan at τ=0 only and therefore shows 1.0 for G3.
* **G3c** loses 17–36 % of demand to the tube in E1/E2 (41 % in E3) (out-of-scan), which raises acc/considered but lowers acc/generated; it is a demand restriction and, as tested, `G2′ ⊄ G3c`.
* **Objective values are not comparable across arms**: arms accept different request sets, so final objectives differ mainly through *how many* customers are served. The table reports mean Δobjective per accepted request as a descriptive number only.
* E3: A-PM turns a 42-request all-accepted baseline into ~5 accepted (expired 5.3, stale 8.2, rejected-promise 23.7): with ω=600 s and W=6000 s queue waiting consumes the window (C3), which is the intended latency coupling. `rejected_no_insertion` is 0 in all arms (needs a depot-recovered trip; covered by a hand-built test).

## Limitations
One instance and one base plan; 10 scenarios (no confidence intervals); G3 collapses to G0 here; identity optimiser; truck-insertion move class only (G2′ coverage not claimed beyond it); no G3W; Tube term untested; wall-clock includes the identical base-plan build; CMake block unverified.
