# Static reproduction of Han, Chi & Sun (2026): final summary (M10, documentation freeze, 9 Nov 2026)

Prepared for inclusion in the thesis report. Everything stated here is backed by a document or output in this repository (index: `INDEX.md`). **Status: validation and comparison complete; numerical reproduction of the paper's results was NOT achieved** (section 5).

## 1. Scope
A C++20 implementation of the static truck-multi-drone delivery problem with flexible launch points (objective: sum over customers of the service-arrival time) and of the paper's solution method: constructive initial solution, eight local-search neighbourhoods N1-N8, four shaking operators and the VNS driver (Algorithm 1 with simulated-annealing acceptance). A runner, a configuration file format, a command-line tool (`tdr_static`) and a machine-readable result format make every experiment reproducible from one configuration file. The dynamic extension is **not** part of this work (on hold, not started). Nothing is committed or pushed.

## 2. What was built
| Layer | Content | Specification |
|---|---|---|
| Model and evaluator | nodes, solution, flexible launch point (FLP) pool, timing and feasibility engine (endurance (36)/(37), synchronisation, rule (28), $\lvert U\rvert$ limit, SR timing Option B decided by the user on 8 Nov 2026); the evaluator is the single authority for feasibility and is checked by an independent frozen reference and 1,558 differential cases | `math_spec.md`, `DECISIONS.md`, `performance_followup_P1_P2.md` |
| Initial solution | truck-only TSP (exact Held-Karp up to 21 nodes, labelled heuristic above), k-means with $k=\lceil \lvert C\rvert/(\lvert U\rvert+2)\rceil$, FLP pool, greedy drone trips checked by the evaluator | `README.md`, `DECISIONS.md` |
| Neighbourhoods | N4, N5 (M2); N8, N7 (M3); N3, N6 (M4); N1, N2 (M5), each with contract, hand-derived tests, brute-force oracle judged by the frozen evaluator, and mutation testing | `operators_N4_N5.md`, `operators_N7_N8.md`, `operators_N3_N6.md`, `operators_N1_N2.md` |
| Shakes | Random_Reverse, Random_Exchange, Random_Shift, Random_Drone_Remove (M6) | `shakes.md` |
| VNS | Algorithm 1 with corrected iteration counter, separate best-so-far, geometric cooling, explicit per-pass outcome and local-step-guard reporting (M7) | `vns.md` |
| Runner, configuration, CLI, outputs | strict 48-key configuration, deterministic seeds, `results.json` / `runs.csv` / `traces.csv` / `timing.json` / `manifest.json`, exit codes 0/1/2/4 (M8) | `runner_cli.md`, `configuration_reference.md`, `output_schema.md`, `windows_validation.md` |
| Validation vs paper | 360-run protocol, statistics, comparison, feasibility analysis (M9) | `validation_vs_paper.md` |

## 3. Provenance of every statement
| Class | Meaning | Where recorded |
|---|---|---|
| **Paper fact** | stated in the paper and implemented as stated: objective; constraints and generalised flight time; sizes 10/20/50 in 60 x 60 km; speeds 15/12 m/s; endurance 1800 s; launch/recovery/service 60/40/30 s; $\theta=1$; two drones; Algorithm 1 structure; neighbourhood order; four shakes; 10-run protocol; gap formulas (38), (39) | `math_spec.md` tags [PAPER]; `validation_vs_paper.md` section 2 |
| **Approved decision** | decided by the user at a milestone sign-off: D1-D10 (M0), SR Option B, N4 bounded re-anchoring (M2), N6 partner set (A8), N1/N2 FLP-only launches kept as a limitation (M5), A19-A29 (M6), A30-A38 (M7, A38 with a reporting condition), A39-A44 (M8), A45-A47 with qualifications (M9) | `DECISIONS.md` items; `MILESTONES.md` |
| **Implementation assumption** | the paper is silent; our choice, labelled and configurable where it is a number: depot at the centre; FLP spacing 2000 m and count rule; adjacency window 3; N1/N2 twice; $\mathrm{iter\_max}=2000$, $T_0=0.01 f(x_0)$, cooling 0.995 (pilot defaults, not tuned to the paper); shake magnitudes; seeds; k-means and heuristic-TSP details | A1-A44 in the per-component documents; `configuration_reference.md` |
| **Deviation** | knowingly different from the paper: exact Held-Karp/heuristic TSP instead of CPLEX; Algorithm 1's literal defects corrected (D8); N1 does not use customer nodes as launch nodes (the paper's text and figures conflict); no MILP | `DECISIONS.md` items 53-66, `validation_vs_paper.md` |
| **Unavailable data** | coordinates of the 12 benchmark instances, depot position, main-experiment FLP spacing, VNS parameters and stopping rule, MILP solutions (hence $\Delta z$), Dalian instances C1-C4 | `validation_vs_paper.md` section 2 |
| **Unresolved question** | (a) why our objective is 2-3 times smaller than the paper's (section 5); (b) the paper's unit of $z$; (c) main-experiment FLP spacing $d$ and rule; (d) launch locations (customer/depot launches in figures vs the FLP-only formal model); (e) Algorithm 2 details (recovery node, `BestRoute()` scoring); (f) launch-delay slack in (36) | `validation_vs_paper.md` sections 4, 8; `DECISIONS.md` Stage-2 open items |

## 4. Verification evidence
* 24 CTest programs (23 test programs, about 7,695,737 assertions/checks, plus `m9_protocol_reproduces`), all passing in a Release build with `-Wall -Wextra -Wpedantic`; full ASan+UBSan runs at M5-M8 with 0 reports; mutation campaigns at every milestone (counts in `test_and_validation.md`).
* The evaluator is differential-tested against an independent frozen reference; every operator is checked against a brute-force oracle judged by that reference; the objective is confirmed independently by an exact dynamic program (agreement 2.9e-11 s).
* Determinism: one base seed reproduces a whole experiment; the recorded results re-run byte-identically; Linux (g++ -O3) and Windows (clang/libc++ cross-build under Wine) outputs are byte-identical for the one experiment compared. Native Windows was not available; PowerShell commands are in `windows_validation.md`.

## 5. Results of the validation (M9) and the findings that must stay explicit
Protocol: 10/20/50 customers x 4 datasets x 3 independent seed sets x 10 runs = 360 runs; pilot defaults. The reported intervals are 95 % t-intervals **over generated instances**; a seed set changes both the instances and the search streams, so they are not intervals over search runs alone. Objective in minutes (the paper does not confirm its unit; minutes is a provisional comparison unit).

| n | mean $z_{best}$ | mean $z_{avg}$ | 95 % interval of mean $z_{avg}$ over 12 instances | mean improvement over initial | s per run |
|---|---|---|---|---|---|
| 10 | 931 | 938 | 858 .. 1017 | 20.2 % | 0.18 |
| 20 | 2378 | 2482 | 2379 .. 2585 | 16.5 % | 0.28 |
| 50 | 9367 | 9817 | 9491 .. 10144 | 23.1 % | 0.68 |

**Finding 1, objective discrepancy (unresolved).** Our mean $z_{avg}$ is 0.45 / 0.33 / 0.47 of the paper's Table 3 means (paper: 2097 / 7500 / 20782, if its unit is minutes), and the paper's means lie outside the range of all 12 of our instances at every size. It is not explained by the depot position (corner: +6..+8 %), by doubling the endurance (about $-1$ % or less), or by the objective formula (confirmed exactly) or the magnitude (a 20-customer tour in 60 x 60 km at 54 km/h implies a mean arrival of about 2 h, ours 2.1 h). Open explanations: a different objective definition or unit in the paper, different instance geometry, a different speed or SR-timing convention.
**Finding 2, FLP-count discrepancy.** The paper's Table 4 lists 214 / 837 / 3,493 flexible launch nodes; our pools are 1,722 / 6,266 / 37,606 (7.5 to 11 times more). The paper's rule is unspecified.
**Finding 3, no numerical reproduction.** The MILP columns, $\Delta z_{best}$, $\Delta z_{avg}$ and the Dalian sensitivity instances cannot be reproduced; per-instance comparison is impossible without the paper's coordinates. The paper's run-to-run dispersion (3.9 / 10.4 / 7.7 %) is larger than ours (0.7 / 4.5 / 4.9 %), under different and partly unknown budgets.
**Independent reference (strictly for the truck-only sub-problem, not a MILP substitute).** At n = 10 the VNS best equals the exact truck-only optimum in 10 of 12 instances (the others +0.54 % and +0.63 %); it is never below it.
**Behaviour of the search.** The best solutions use no drones in 357 of 360 runs, so the trip-based neighbourhoods N1, N2, N3, N5, N6, N8 have no anchor in 93-99.6 % of the calls; N7 is mostly endurance-infeasible (drone reach about 20 km against customer spacing 19 / 13 / 8.5 km) and its feasible candidates are slightly worse than the base. Endurance, geometry and operators were not altered to change this; labelled sensitivity runs show the same picture with a doubled endurance (20-33 % of the best solutions then use a drone).

## 6. Limitations
No instance coordinates; no MILP; unit of the paper's $z$ unconfirmed; the objective and FLP-count discrepancies unresolved; VNS parameters are pilot defaults (the search is still improving at 2000 passes for n = 50); timings not comparable with the paper (language, hardware, budget); one platform for the 360-run protocol; native Windows untested. Unapproved proposals (not implemented): P1 larger iteration budget for n >= 50, P2 diversification for stalled runs, P3 guard in the heuristic TSP for non-finite travel times.

## 7. Reproduction
```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DTDR_BUILD_BENCH=ON && cmake --build build -j
ctest --test-dir build                                    # 24 programs
scripts/m9_run_all.sh build/tdr_static all                # all recorded experiments (about 7 minutes)
python3 -I scripts/m9_analyse.py results/m9               # tables of validation_vs_paper.md sections 3-5
python3 -I scripts/m9_sensitivity.py results ; python3 -I scripts/m9_exact_truck_only.py results/m9
python3 -I scripts/m10_doc_check.py --build build         # documentation consistency checks
tdr_static run --config <result dir>/config.resolved.cfg --out <new dir>   # reproduce any recorded run
```
