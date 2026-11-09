# M9: validation and comparison with the paper (9 Nov 2026, instruction 15:35 IST)

**Status: signed off (9 Nov 2026, 16:13 IST) as a validation and comparison milestone. This does NOT mean that numerical reproduction of the paper was achieved.** The objective discrepancy (section 4), the FLP-count discrepancy (section 4, item 4) and the unavailable paper data (section 2) are findings and limitations that stay explicit. Qualifications of the approval are in section 8.

Scope: run the implemented static algorithm on several instances and independent seeds under reproducible configurations, compare with Han, Chi & Sun (2026), Table 3, **only where the paper's data allow**, and analyse the neighbourhoods that are rarely feasible **without changing the algorithm, the evaluator, the endurance or the geometry**. No source file under `include/` or `src/` was changed in M9 (the only code added is a read-only diagnostic under `bench/`, scripts under `scripts/`, and a CTest guard). Every number below is reproduced by the commands in section 9; raw outputs are `m9_*_raw.txt` in this folder.

## 1. Protocol (all parameters are those of the paper's Section 6.1 or the recorded pilot defaults)

| Item | Value | Status |
|---|---|---|
| Sizes | 10, 20, 50 customers | paper |
| Instances per size | 4 datasets x 3 independent seed sets = **12 generated instances per size** (paper: 4; these are our generated instances, not the paper's) | paper's 4, tripled by us |
| VNS runs per instance | **10** independent runs (run index 0..9), best and mean reported | paper protocol |
| Seed sets | `seed.base_seed` = 20261109, 31415926, 271828183. A seed set changes the instances **and** the search streams (instance d: `derive_seed(base, instance, d)`; run r: `derive_seed(base, ..., r)`), so the three sets are independent replications of the whole experiment | ours |
| Area, speeds, endurance, launch/recovery/service, theta, drones | 60 x 60 km; 15 / 12 m/s; 1800 s; 60 / 40 / 30 s; 1; 2 | paper |
| Depot, FLP spacing d, window, N1/N2 repeats | centre; 2000 m; 3; 2 | ours (D9, D1, D3, ...) |
| VNS | `iter_max` 2000, T0 = 0.01 f(init), cooling 0.995, reference = incumbent, shakes uniform | ours (D7, pilot defaults, **not tuned to the paper**) |
| Configurations | `config/m9/n{10,20,50}_seed{1,2,3}.cfg` (9 files, 6 keys each: experiment name, 4 datasets, 10 runs, trace off, seed, size); outputs `results/m9/<name>/` | |

All 360 runs finished with status ok, were re-verified by the runner (every customer once, evaluator-feasible, exact objective), triggered the `max_local_steps` guard 0 times and skipped 0 passes. Re-running `results/m9/n10_seed2/config.resolved.cfg` gives byte-identical `results.json` and `runs.csv`; the CTest `m9_protocol_reproduces` re-runs the whole `n10_seed1` experiment and compares three files byte for byte (negative-tested: a one-field change in `runs.csv` or in a configuration makes it fail).

## 2. What is exact reproduction, what is an assumption, what is unavailable

| Class | Items |
|---|---|
| **Exact reproduction** (stated in the paper, implemented as stated) | objective (sum of customer service-arrival times; confirmed independently, section 6); constraints and generalised flight time (evaluator, frozen, validated M0-M6 against the reference and differential tests); sizes 10/20/50 in 60 x 60 km; all Section 6.1 parameters; two drones; Algorithm 1 structure, neighbourhood order N4, N5, N8, N3, N6, N7 then N1/N2; the four shakes; the 10-runs-then-average protocol; the gap formulas (38), (39) |
| **Implementation assumptions** (the paper is silent; recorded in `DECISIONS.md`) | depot at the centre (D9); FLP generation rule and spacing d = 2000 m (D1/D8); adjacency window 3; N1/N2 twice per pass; N3 = consecutive trips of one drone; VNS `iter_max`, T0, cooling, reference (D7); shake selection uniform; k-means and TSP details of the initial solution; N4 re-anchoring budget; objective **unit for comparison** (section 4) |
| **Unavailable paper data** | coordinates of the 12 benchmark instances (so no instance can be re-created, and per-instance comparison is impossible); depot position; FLP spacing d for the main experiments; VNS parameters and stopping rule; MILP solutions (no MILP is part of this repository, so MILP `z`, GAP, `Delta z_best`, `Delta z_avg` of formulas (38)/(39) **cannot be reproduced**, and for n = 50 the paper has none either); the Dalian instances C1-C4 of Section 6.3 (so the sensitivity analyses cannot be reproduced); the paper's per-run counters |

Consequence: the comparison below is **statistical** (distribution of objective values over instances of the stated size and area), not per instance, and it is only meaningful if the unit and the instance distribution agree, which section 4 shows they probably do not.

## 3. Results on our instances (objective in minutes = seconds / 60)

Per size over the 12 generated instances (`m9_analysis_raw.txt` B, F). The 95 % intervals are intervals over **generated instances** (instance-to-instance variation): a seed set changes both the instances and the search streams, so they are not intervals over search runs alone and do not separate the two sources of variation:

| n | mean z_best | mean z_avg | 95 % interval of the mean z_avg over the 12 generated instances (t, 11 d.f.) | range of z_avg | mean (z_avg - z_best)/z_best | mean improvement over initial | mean initial | seconds per run |
|---|---|---|---|---|---|---|---|---|
| 10 | 931 | 938 | 858 .. 1017 | 801 .. 1143 | 0.68 % | 20.2 % | 1196 | 0.18 |
| 20 | 2378 | 2482 | 2379 .. 2585 | 2259 .. 2783 | 4.53 % | 16.5 % | 2977 | 0.28 |
| 50 | 9367 | 9817 | 9491 .. 10144 | 8900 .. 10608 | 4.88 % | 23.1 % | 12763 | 0.68 |

* Seed-set to seed-set spread of the mean z_avg: 9.0 % (n = 10), 9.0 % (n = 20), 2.1 % (n = 50) of the mean, i.e. the instance sample, not the search, dominates for n = 10 and 20.
* Within an instance the search is very stable at n = 10 (6 of 12 instances: all 10 runs identical; mean z_avg within 0.7 % of z_best) and noisier at n = 20 and 50 (up to 9.2 % and 12.5 % between mean and best).
* Convergence: the mean iteration of the last best-update is 251 / 642 / 1554 of 2000 (n = 10 / 20 / 50), so the pilot budget is ample for n = 10, adequate for n = 20 and **binding for n = 50** (section 7: 4x the budget lowers the n = 50 mean by 2 .. 3.4 %).
* Every initial solution is worse than the VNS best (36/36 instances, mean improvement 16 .. 23 %); no run hit the local-step guard.
* Time: 0.18 / 0.28 / 0.68 s per run (C++, g++ -O3, one core of the development VM); the paper reports 1.72 .. 11.93 s per instance (Java, i5-9500H, unknown iteration budget). These are not comparable (language, hardware, budget).

## 4. Comparison with the paper's Table 3 (VNS columns only)

Unit hypothesis (provisional): **the paper does not confirm the unit of `z`**; its range (1,020 .. 21,981) is of the magnitude of **minutes** of summed arrival times (ours: 800 .. 10,600 minutes; in seconds ours would be 48,000 .. 640,000), so the comparison is made in minutes. This is an assumption (not an approval item for the code: reporting in minutes is already in every output as `*_min`).

| n | paper mean z_best (4 instances) | paper mean z_avg | paper range z_avg | paper mean (z_avg - z_best)/z_best | ours mean z_avg | ours / paper |
|---|---|---|---|---|---|---|
| 10 | 2004 | 2097 | 1020 .. 3097 | 3.90 % | 938 | 0.45 |
| 20 | 6783 | 7500 | 6701 .. 9137 | 10.44 % | 2482 | 0.33 |
| 50 | 19312 | 20782 | 19050 .. 21981 | 7.74 % | 9817 | 0.47 |

Findings (none of them adjusted):
1. **The objective values do not agree**: our mean z_avg is 33 .. 47 % of the paper's, and the paper's mean lies outside our 95 % CI **and outside the whole range** of our 12 instances at every size. Per customer: ours 94 / 124 / 196 min, paper 210 / 375 / 416 min.
2. The ratios of the means between sizes differ too: paper 20/10 = 3.58 and 50/20 = 2.77; ours 2.65 and 3.96. With four paper instances per size and a paper spread of 1020 .. 3097 at n = 10 (ours 801 .. 1143) the paper's instance distribution looks more dispersed than uniform-in-the-square at the same size; nothing more can be said without its coordinates.
3. Run-to-run dispersion: the paper's mean (z_avg - z_best)/z_best is larger (3.9 / 10.4 / 7.7 %) than ours (0.7 / 4.5 / 4.9 %): our search is more repeatable under our budget; the paper's budget and stopping rule are unknown.
4. FLP counts: the paper's Table 4 lists 214 / 837 / 3,493 flexible launch nodes; ours are 1,722 / 6,266 / 37,606 (mean pool sizes, all arcs, d = 2000 m). The paper evidently uses a coarser or sparser rule; it is unspecified (the "biggest gap" of the extraction), the pool only enlarges the search space of our operators and does not change the problem.
5. `Delta z_best`, `Delta z_avg` (needs the MILP) and the n = 50 MILP columns: unavailable; no MILP is implemented.

### Why the scale gap is not explained by what we can vary (labelled sensitivity runs, section 7)
* Depot in the corner instead of the centre: ours 959 / 2640 / 10445 (+6 .. +8 % over the default) -> ratio to the paper 0.46 / 0.35 / 0.50: **not** the cause.
* Endurance doubled: the objective does not move (903 / 2434 / 9747): **not** the cause either.
* An independent calculation shows our objective is the stated one (section 6) and our magnitude is the plausible one for a 54 km/h truck: a 20-customer tour in 60 x 60 km has length about 0.7124 sqrt(n A) = 191 km = 3.5 h, so the mean service-arrival time is about 2 h = 120 min (ours 124 min; the paper's 375 min would need a tour of about 12 h).
* Therefore the unexplained factor 2 .. 3 is one of: a different objective definition or unit in the paper, different instance geometry (clustered / larger / non-uniform), or a different speed convention. **Open question for the user**: if the paper's equation (1), the depot and a plotted instance can be checked against the paper PDF (for example a figure with a solved example and its objective), the factor can be pinned down; until then the absolute numbers are not claimed to agree.

## 5. Neighbourhood feasibility (read-only diagnostic; nothing is changed)

Two data sources: operator counters of all 360 runs (`m9_analysis_raw.txt` E) and `bench/m9_feasibility_probe.cpp`, which sweeps every operator over every anchor of (a) the initial solution and (b) the best solution of VNS run 0, tabulating the evaluator's rejection reasons (`m9_feasibility_probe_raw.txt`; seed set 1, 12 instances; the production operators with the production options; N7 additionally with window 1000 to show the window is not the limit).

**Over all 360 runs (per size, 240,000 passes each):**

| Operator | calls with no anchor | anchor but no feasible candidate | with a feasible candidate | improving moves |
|---|---|---|---|---|
| N1, N2, N3, N5, N6, N8 (trip-based) | n = 10: 99.4 .. 99.6 %; n = 20: 97.3 .. 97.9 %; n = 50: 92.8 .. 95.0 % | 0 .. 4.1 % | 0 .. 6.3 % | 0 .. 3,237 per 240,000 |
| N7 (truck customer -> drone) | 0 % | 69.8 % / 48.0 % / 18.4 % | 30.2 % / 52.0 % / 81.6 % | 53 / 129 / 319 |
| N4 (truck relocate) | 0 % | 0.1 .. 2 % | 98 .. 99.9 % | 749 / 1081 / 3292 |

**Why:** (i) *Trip-based operators have no anchor because the search state has no drone trips.* The initial solution has 2.3 / 6.8 / 22.6 trips on average, but the best solutions contain **0 trips in 357 of 360 runs** (n = 10: 120/120; n = 20: 118/120; n = 50: 119/120). Once the first N8 moves, the shakes and the acceptance rule have dissolved the initial trips, N1, N2, N3, N5, N6, N8 are simply not applicable (the "no anchor" share equals the share of passes spent in drone-free states). This is a property of the search trajectory, not an infeasibility of the operators: on the initial solution, where trips exist, the probe shows N8 feasible in 23 .. 86 % of its candidates (and improving at every anchor), N1 7 .. 9 %, N5 2 .. 9 %, N2/N3/N6 0 .. 12 %.
(ii) *Where trips do exist, candidates are rejected for endurance, then for invalid recovery or affected trips.* Rejections on the initial solutions (sums over the 12 instances): N1: invalid-recovery 1,983, evaluator-endurance 2,215 (+72 drone-sequencing), N2: invalid-recovery 341, endurance 279; N3: endurance 208 of 230; N5: endurance 4,984, affected-trips 566; N6: endurance 448 of 474; N7: endurance 1,429, affected-trips 163 (window 3). `eval-flp-arc`, `eval-flp-unused` and the other construction-error reasons are **0**, i.e. no operator builds a structurally wrong candidate.
(iii) *N7 candidates are overwhelmingly endurance-infeasible* (96 .. 98 % of the generated drone insertions on the drone-free best solutions at n = 10, 90 % at n = 20, 72 % at n = 50 with window 3; the rest of the rejection is the same reason under window 1000). The window is not the limit: window 1000 generates 2 .. 10 times more candidates and the feasible count stays within the same range. The share grows with n (feasible 3.8 % -> 9.9 % -> 27.6 % at window 3), because the customers are closer together (mean spacing about sqrt(A/n) = 19 / 13 / 8.5 km) while the drone's total path is limited to at most 21.6 km (1800 s x 12 m/s, before launch, recovery and service times are deducted); with 10 customers in 3,600 km2 most customers are simply out of reach of any launch/recovery pair on the truck route.
(iv) *Feasible drone insertions are not improving.* On the drone-free best solutions N7 finds feasible candidates at nearly every anchor set (e.g. 16, 28, 17, 7 candidates for the four 10-customer instances), but the best of them is **worse than the base by +0.0 .. +2.6 %** (one candidate ties exactly, +0.00 %, and is not accepted under strict `<`); improving anchors: 0 of 40 (n = 10), 0 of 80 (n = 20), 4 of 200 (n = 50, window 1000). With drone speed 12 m/s below truck speed 15 m/s, a 130 s launch/recovery/service overhead per trip and an objective that sums arrival times, serving a customer by drone rarely pays off; this is the evaluator's economics, not a feasibility defect.

Summary: the low activity of N1, N2, N3, N5, N6, N8 is a consequence of drone-free search states; N7's low feasibility is endurance-driven and geometry-driven (reach vs customer spacing). **No change to endurance, geometry or operators was made to alter this**; section 7 shows what happens if endurance is doubled as a clearly separate diagnostic.

## 6. Independent checks of the implementation (reference points that do not need the paper)

Exact optimum of the **truck-only** sub-problem for the 10-customer instances (`scripts/m9_exact_truck_only.py`, dynamic programming over subsets, minimum-latency with service times, `m9_exact_truck_only_raw.txt`):
* The formula reproduces the evaluator's objective of every drone-free VNS best solution to 2.9e-11 s: the objective is the stated one and the evaluator and the DP agree.
* VNS z_best equals the exact truck-only optimum in **10 of 12** instances; in the other two (seed set 1 dataset 1: +0.54 %; seed set 2 dataset 0: +0.63 %) none of the 10 runs reaches it. The mean z_best gap is +0.097 %, the mean z_avg gap +0.77 %; the number of runs (of 10) that find the optimum per instance is 10, 10, 10, 10, 9, 9, 7, 3, 3, 1, 0, 0. VNS z_best is never below the truck-only optimum (0 of 12): with the paper's parameters drones did not help on these instances.
* This is the closest we can get to the paper's "MILP optimum" check at n = 10; the truck-only optimum is an upper bound of the true optimum, not a lower bound (a MILP with drones could still be better; the N7 sweeps in section 5 found no better drone insertion at the best solutions, which supports but does not prove optimality).
* Observation (reported, not acted on): in 2 of 12 instances the search stalls 0.5 .. 0.6 % above the truck-only optimum in all 10 runs. Possible lever (needs approval, not implemented): a larger iteration budget or restart diversification; this is an algorithmic proposal and is listed in section 8.

## 7. Labelled sensitivity diagnostics (seed set 1; NOT the protocol; defaults unchanged; `m9_sensitivity_raw.txt`)

| n | variant | mean z_avg (min) | improvement over initial | drone trips in best (mean) | runs with 0 trips | N7: feasible candidates as % of calls |
|---|---|---|---|---|---|---|
| 10 | default | 905 | 17.9 % | 0.00 | 40/40 | 34.5 |
| 10 | depot in a corner | 960 | 16.6 % | 0.00 | 40/40 | 36.2 |
| 10 | endurance 3600 s | 903 | 25.8 % | 0.40 | 27/40 | 63.8 |
| 20 | default | 2446 | 19.5 % | 0.00 | 40/40 | 49.9 |
| 20 | depot in a corner | 2640 | 17.6 % | 0.00 | 40/40 | 52.0 |
| 20 | endurance 3600 s | 2434 | 23.3 % | 0.50 | 27/40 | 87.8 |
| 50 | default | 9857 | 24.2 % | 0.00 | 40/40 | 79.2 |
| 50 | depot in a corner | 10445 | 20.2 % | 0.00 | 40/40 | 80.0 |
| 50 | endurance 3600 s | 9747 | 25.8 % | 0.28 | 32/40 | 94.6 |

Even with a doubled endurance only 20 .. 33 % of the best solutions use any drone and the mean objective changes by 0.2 .. 1.1 %: endurance is not what makes the drones unattractive. Iteration budget (`vns.iter_max` 8000 against 2000, same instances and run indices; the geometric schedule is unchanged, the run just continues): n = 20 mean z_avg -0.6 .. -2.0 %, n = 50 -2.0 .. -3.4 %, z_best at n = 50 improves by up to 8.2 % (instance 1); the last best-update then happens at iteration 4,185 .. 6,003 of 8,000, i.e. n = 50 is still improving. These runs are diagnostics; the pilot defaults (D7) are unchanged.

## 8. Limitations, and items for the user

Limitations: no instance coordinates (statistical comparison only, 12 vs 4 instances per size); no MILP (no `Delta z`); unit of the paper's `z` unconfirmed; the factor 2 .. 3 objective gap unresolved; FLP rule differs (counts 7.5 .. 11 times the paper's); VNS parameters are the pilot defaults, not the paper's; timings not comparable; seed sets change instances and search together (a paired design for the same instances with different search seeds is available through `experiment.first_run_index`, not used); one platform for the 360-run protocol (cross-platform identity was shown for a smaller experiment in `windows_validation.md`).

Approved with qualifications (9 Nov 2026, 16:13 IST; no code change involved):
* **A45** minutes is a **provisional** comparison unit for the paper's `z`; the paper does not confirm its unit (seconds are reported next to minutes in every output).
* **A46** the protocol is 12 instances per size (4 datasets x 3 seed sets) with 10 runs each, pilot VNS defaults, reported as in section 3 (intervals over generated instances; a seed set changes instances and search streams together); a per-instance table with the paper's per-instance numbers is deliberately not produced because instance identity cannot be matched.
* **A47** the exact truck-only optimum (n = 10) is accepted strictly as an independent reference for the truck-only sub-problem, **not** as a substitute for the unavailable MILP optimum (limits in section 6).

Proposals (**unapproved**; algorithmic or parameter changes are not made; each needs a separate approval):
* **P1** raise the pilot `vns.iter_max` for n >= 50 (and report it as a recorded parameter change), since the n = 50 search is still improving at 2000 passes (section 3, 7).
* **P2** diversification/restart for the two n = 10 instances where all runs stall 0.5 .. 0.6 % above the optimum (section 6).
* **P3** the Stage-1 heuristic TSP crash for non-finite travel times (known since M8; the runner preflight guards it).
* **Q1** (information, not a change): pin down the paper's unit/instance geometry (section 4).

## 9. Reproduction

```
cmake --build <build> --target tdr_static m9_feasibility_probe      # (probe needs -DTDR_BUILD_BENCH=ON)
scripts/m9_run_all.sh <build>/tdr_static main                       # 9 experiments -> results/m9, about 3 minutes
scripts/m9_run_all.sh <build>/tdr_static sens                       # labelled sensitivity runs -> results/m9_sensitivity (plus the iteration-budget runs listed in m9_sensitivity_raw.txt)
python3 -I scripts/m9_analyse.py results/m9                         # tables of sections 3, 4, 5 -> results/m9/summary.json
python3 -I scripts/m9_sensitivity.py results                        # section 7
python3 -I scripts/m9_exact_truck_only.py results/m9                # section 6
<build>/m9_feasibility_probe config/m9/n10_seed1.cfg                # section 5 (also n20, n50)
ctest -R m9_protocol_reproduces                                      # byte-identical re-run of n10_seed1
```
The iteration-budget runs used `--set vns.iter_max=8000` (n = 20: all 4 datasets; n = 50: `experiment.num_datasets=2` with `instance.dataset_index=0` and `2`), output directories `results/m9_sensitivity/iter8000_*`.
