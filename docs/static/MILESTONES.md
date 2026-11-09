# Static reproduction: milestone log

Branch: `stage1-2-evaluator-hardening-sr-timing`, base commit `cc37e49`. **Nothing from M1 onward has been committed or pushed** (no authorisation given).
Dynamic extension: on hold, not started.

**Frozen status (M10): M0--M9 signed off, M10 documentation freeze complete; the authoritative status table is `INDEX.md` section 1.** Counts and figures inside each milestone section are those of that milestone (final suite: 24 CTest programs, 23 test programs with about 7,695,737 assertions); where a later section or the M10 section differs, the later one governs.

## M0 --- decisions (9 Nov 2026, approved by the user)

Implementation sequence M0--M10 approved. D1--D10 approved with defaults (full text and clarifications in `DECISIONS.md`, "Static reproduction"):
adjacency window 3; N1/N2 repeated twice per pass; N3 = consecutive trips of the same drone; N5/N7 re-selection = best valid FLP on the resulting arcs within the window, recovery at the next truck
node; N8 searches all valid insertion positions; shakes dissolve only trips invalidated by the shake, with counts and reasons; VNS parameters configurable and calibrated by pilot runs;
clarified VNS (correct iteration counter, search from the current accepted solution, separate best-so-far, accepted improvements carried forward); depot at the centre; seconds internally
with minutes reported alongside. Move of `assign_drone_ids_by_route_order` approved on condition that behaviour is unchanged and regression-tested. FLP pool stays eager; no dynamic FLP id scheme.

## M1 --- infrastructure and evaluator profile (9 Nov 2026) --- accepted (the user authorised the M1 performance follow-up and M2 together)

**Files modified (2):** `CMakeLists.txt` (+12 lines: two test targets); `include/init/initial_solution_builder.hpp` (function body moved out; `using` forwarding declaration and one `#include` added; 5 lines added, 43 removed).

**Files added (11):**

| File | Lines |
|---|---:|
| `include/util/rng.hpp` | 136 |
| `include/config/static_config.hpp` | 118 |
| `include/instance/static_instance.hpp` | 97 |
| `include/flp/flp_arc_index.hpp` | 84 |
| `include/search/drone_assignment.hpp` | 72 |
| `include/search/route_ops.hpp` | 281 |
| `include/search/candidate_window.hpp` | 107 |
| `tests/test_search_infra.cpp` | 555 |
| `tests/test_fuzz_search_infra.cpp` | 321 |
| `docs/static/*.md` (6 files) | documentation |

**Not modified:** `timing/timing_engine.hpp` and every other existing header and test.

**Verification (all run fresh, outside the repository tree):**

| Check | Result |
|---|---|
| Baseline before changes, six programs | 26 + 43 + 59 + 291 + 29 + 171 = 619 passed, 0 failed |
| After M1, same six programs | identical counts, 619 passed, 0 failed |
| New: `test_search_infra` | 144 passed, 0 failed |
| New: `test_fuzz_search_infra` | 11,262 assertions, 0 failed (794 edited cases) |
| Compiler warnings (`-Wall -Wextra -Wpedantic`, g++ 11.4, -O3 and CMake Release) | 0 |
| CMake configure + build + `ctest` (CMake 4.4.4 installed in a scratch directory) | 8 of 8 tests passed |
| ASan + UBSan (`-fno-sanitize-recover=undefined`, leak detection on), all 8 programs | 0 sanitizer reports, all passed |
| Move of `assign_drone_ids_by_route_order` | function body asserted identical to the original by script; 40,000-case old-vs-new differential output byte-identical; 20,000-case checksum pinned in the repo test |
| Mutation checks | 9 injected bugs, all detected (`test_and_validation.md` §4) |

**Performance finding.** At 50 customers one `evaluate_solution` call takes ~280--335 us, and ~96% of it is one registry scan in the Stage 1 validation; with the registry reduced
to the used nodes the same call takes ~9--12 us (identical objective). See `performance_profile_M1.md`. **No evaluator change was made**; proposals P1 (remove the scan) and P2
(ordered summation) await approval and should be applied before the first neighbourhood benchmark.

**Issues found and resolved during M1 (test expectations, not product defects):**
1. `unused_flps_in_route` vs evaluator rule (6): they differ when a trip is unplaced (the evaluator also counts FLPs whose only trips are unplaced). Verified on data (41 cases, none with all trips placed;
   the helper never over-reports); documented and tested as equality-when-all-placed, subset otherwise.
2. `lowest_free_drone_for_trip` returns nothing only for unplaceable trips (0 placeable cases among 20); the test now states exactly that.
3. The first fuzz harness could not detect an overlap-ignoring mutation (initial solutions never use drone id > 0); strengthened (see `test_and_validation.md` §4).

**Open follow-ups recorded for later milestones (not done in M1):**
* `TSPSolverFactory` has a hard-coded heuristic seed (42) and `KMeansPolicy` constants: to be driven from `StaticConfig`/the InitialSolution stream in M8 ("nothing hardcoded").
* `build_initial_solution` still generates the FLP pool inline; it equals what `FlpArcIndex` consumes, and a test checks consistency, but it is not yet shared code.
* The ordered-summation item (P2) and the evaluator speed-up (P1) need your decision.
* CMake was verified with a scratch CMake 4.4.4; the repository's stated minimum is 3.16 and was not tested.

**Next:** the M1 performance follow-up (below), then M2 (N4, N5).

## M1 performance follow-up: P1 and P2 (9 Nov 2026) --- accepted and applied

Full report: `performance_followup_P1_P2.md`. Summary:

* **P1** (customer-id cache): `SpatialNetwork::customer_ids()`, an immutable ascending vector built in the constructor from the by-value, mutator-free registry; the evaluator reads it instead of scanning the ~39k-node FLP pool.
  Verified **bitwise identical in every output** to the frozen reference (strict differential mode).
* **P2** (ascending-id summation): `customer_records` and the objective are now produced in ascending customer-id order. Objective differs from the old value by at most 6.98e-10 s (relative 9.6e-16, 0.154 x the rigorous bound);
  `customer_records` order and the order of customer-level violation messages changed (contents identical). All 93 changed pairwise comparisons (of 11,922) are mathematically exact ties whose old ordering depended on hash-table insertion order;
  production agrees with exact arithmetic on all pairs. No tolerance introduced. Reverting P2 is a three-hunk change.
* **Speed** (median, Release, 3 instances per size): n = 50 populated solution 345 us -> 10.5 us (33 x), all-truck 331 us -> 5.0 us (66 x); n = 20: 9--15 x; n = 10: 5.6--8 x; P2 adds no measurable cost.
* **Tests:** `test_evaluator_differential` (26,745 assertions, 1,558 differential cases, seed 31415) + 13 mutants killed; 9 of 9 CTest programs pass in Release and under ASan + UBSan; 0 warnings.
* **Files:** changed `include/graph/spatial_network.hpp`, `include/timing/timing_engine.hpp`, `CMakeLists.txt`; created `tests/reference/timing_engine_reference.hpp`, `tests/test_evaluator_differential.cpp`, `bench/bench_evaluator.cpp`,
  `docs/static/performance_followup_P1_P2.md`, `docs/static/perf_followup_bench_raw.txt`.
* **Open item for M7:** strict-improvement acceptance on floating-point objectives (see §4 of the follow-up report); nothing needed for M2.

## M2 --- neighbourhoods N4 and N5 (9 Nov 2026) --- implemented and **signed off** (N4 policy: see the follow-up and finalization below; figures in this section are the original Reject-policy figures)

Operator contracts, interpretations and open questions: `operators_N4_N5.md`. Evidence: `test_and_validation.md` §6. Benchmark raw output: `operator_bench_raw.txt`. M3 and later were **not** started.

**Files created (7):** `include/search/move_types.hpp` (213 lines), `include/search/neighbourhood_n4.hpp` (214), `include/search/neighbourhood_n5.hpp` (215), `tests/test_n4_n5.cpp` (708), `tests/test_n4_n5_oracle.cpp` (438),
`bench/bench_operators.cpp` (158), `tests/mutation/m2_mutate.py` (76); docs `operators_N4_N5.md`, `operator_bench_raw.txt`, `m2_mutation_results_raw.txt`.
**Files changed:** `include/search/route_ops.hpp` (+`flp_insertion_position`), `include/search/candidate_window.hpp` (+`gaps_within_window`) --- both M1 files, additions only; `CMakeLists.txt` (two test targets, `bench_operators` under `TDR_BUILD_BENCH`); `DECISIONS.md`, this file, `test_and_validation.md` (M2 sections appended). Nothing deleted. The evaluator, the frozen reference and all M1 behaviour are untouched.

* **N4** (truck service node relocate): relocate and exchange within the D1 window, duplicates removed, affected trips rejected and named, everything else judged by the evaluator.
* **N5** (T-D service node swap): partner swap, old trip dissolved (its now-unused launch FLP dropped explicitly), new trip $(f', t, y)$ over FLPs of resulting arcs in the window with recovery at the next truck node and the lowest free drone, affected other trips rejected and named.
* **Tests:** `test_n4_n5` 130 assertions (hand-derived values on line fixtures); `test_n4_n5_oracle` 713,256 comparisons against an independent brute-force enumeration judged by the frozen reference evaluator (240 bases from 60 seeds, 3,144 N4 and 880 N5 calls, ~97,900 candidates compared one by one, deterministic across two runs).
  11 of 11 CTest programs pass in Release and under ASan + UBSan; 0 warnings; 35 injected bugs, all detected (5 were missed by the first test versions and led to new tests); 5 repeated runs byte-identical.
* **Performance:** the P1/P2 evaluator gains are intact (n = 50 populated solution 10.7 us, 30 x faster than the reference). N4 sweep over all 27 truck customers at n = 50: 0.77 ms (generation 1.7 us/candidate net of base evaluations); N5 sweep over 23 trips: 6.1 ms, of which evaluation is 83 % (13.5 us per evaluated candidate, ~ one evaluator call).
* **Findings needing your decision (details in `operators_N4_N5.md` §6--7):** (1) with *reject* as the only affected-trip policy, 87--99 % of N4 candidates on initial solutions are rejected because they move a node next to an FLP-launching trip; (2) N4 relocate vs exchange (both searched); (3) N5 launch window centred on the swapped customer's former position.
  **All three were decided in the follow-up below** (conservative re-anchoring; both modes kept; centre approved). The figures in this section are the pre-follow-up (Reject) figures.
* **Bug found and fixed during M2 (regression-tested):** the classifier reported a recovery-before-launch trip as the rule-(6) cascade; it now reports `EvalTripOrder`.

*(Historical; the log continues below.)*

## M2 follow-up --- N4 affected-trip policy and sign-off (9 Nov 2026) --- implemented; cap semantics superseded by the M2 finalization below

Instruction: "M2 follow-up: resolve N4 affected-trip policy and final sign-off" (06:17 IST). Semantics, rejected cases and measurements: `operators_N4_N5.md` §6--7. Evidence: `test_and_validation.md` §7. Raw output: `operator_bench_raw.txt` (before: `operator_bench_raw_reject_policy_before.txt`), `n4_reanchor_cap_sensitivity_raw.txt`, `evaluator_bench_after_m2_followup_raw.txt`, `m2_followup_mutation_results_raw.txt`.
**Not committed, not pushed. M3 and later were not started at that point** (no N8, N7, N3, N6, N1, N2, shakes, VNS, runner, CLI or dynamic code; M3 was authorised later, see below).

**Paper facts / approved decisions / implementation assumptions / deviations**

| Item | Class |
|---|---|
| The paper describes N4 as relocating a truck node (its example is an exchange) and is silent on trips disturbed by the move | **PAPER** |
| Conservative re-anchoring of the launch FLPs of trips *directly* affected by an N4 move; recovery, drone customer and drone id preserved; recovery/ordering failures rejected, never repaired; no dissolving | **DECISION (approved 9 Nov 2026), implemented as ASSUMPTION A6** |
| Replacement FLPs: the resulting arc only (radius 0), ratio order, evaluator-judged joint best, strict `<`, first wins ties (the N5 convention); joint enumeration with a variant cap (first 10,000 with a `TooManyReanchorings` rejection; **superseded by the M2 finalization below: budget 100, no cap rejection**) | **ASSUMPTION A6** (cap value and "arc only" are my choices inside the approved rule) |
| Keep N4 relocate **and** exchange (`N4Mode`); an interpretation, not two paper neighbourhoods | **DECISION / ASSUMPTION A1** |
| N5 launch-window centre = the swapped truck customer's former position | **DECISION / ASSUMPTION A3** (N5 otherwise unchanged) |
| Strict `<`, no epsilon, first-generated tie-break in N4/N5; configurable threshold only to be considered in M7 | **DECISION** |
| Deviations from the paper | none |

**Files changed (code):** `include/search/move_types.hpp` (new reasons `NoReplacementFlp`, `InvalidRecovery`, `TooManyReanchorings`; `AffectedTripPolicy`; `Reanchor`; counters), `include/search/neighbourhood_n4.hpp` (`N4Options{mode, affected_trips, max_reanchor_variants}`, re-anchoring). `neighbourhood_n5.hpp`, the evaluator, `spatial_network.hpp` and the frozen reference are **byte-identical** (sha256 verified).
**Files changed (tests/bench):** `tests/test_n4_n5.cpp` (+73 assertions, the 7 Reject-era N4 tests now pin `AffectedTripPolicy::Reject` explicitly, no assertion weakened), `tests/test_n4_n5_oracle.cpp` (existing N4 oracle pinned to Reject, new independent Reanchor oracle), `tests/mutation/m2_mutate.py` (+21 mutants, 3 patterns updated), `bench/bench_operators.cpp` (both policies + diagnostics).
**Docs:** this file, `DECISIONS.md` (items 15, 21--26), `operators_N4_N5.md` (§3, 4, 6, 7), `test_and_validation.md` (§7), and the project milestone summary. New raw files listed above.

* **Tests:** `test_n4_n5` 203 assertions (was 130); `test_n4_n5_oracle` 905,628 checks (was 713,256), 0 failures; 11 of 11 CTest programs pass in Release and under ASan + UBSan; 0 warnings; 56 of 56 mutants detected (35 M2 + 21 re-anchor); repeated runs byte-identical.
* **Effect and cost (n = 10 / 20 / 50, initial solutions):** feasible N4 candidates 13.3 / 8.2 / 0.8 % $\to$ 17.8 / 24.5 / 2.0 %; re-anchored 4.4 / 16.4 / 1.2 %; still rejected 82.2 / 75.5 / 98.0 %. N4 sweep cost rises from 47 us / 140 us / 0.78 ms to 5.3 / 13.2 / 362 ms (joint enumeration). N5 and the evaluator are unchanged (n = 50 populated evaluation 10.6 us, 31--34 x vs reference).
* **For your decision (not acted on):** (1) the benefit at n = 50 is small because 69 % of the stale candidates also break a recovery relationship, which the approved rule leaves rejected; (2) the exhaustive joint enumeration is costly: `max_reanchor_variants = 100` keeps the same accepted candidates in these bases at ~12 x lower cost at n = 50; I kept the complete default (10,000) pending your choice [**resolved in the M2 finalization: default 100, budget-bounded**]; (3) a re-selection of the recovery node or an independent per-trip choice would be new rules and are not implemented.
* **Found and fixed during the follow-up:** a defensive guard (an FLP off its arc that no trip launches from is judged as it stands) -- found by a mutant that made the enumeration loop forever; unreachable through the public entry points.

**Next (historical):** the M2 finalization below (budget 100).

## M2 finalization --- bounded N4 re-anchoring and sign-off (9 Nov 2026, instruction 08:06 IST) --- implemented and **signed off by the user** (final N4 policy: Reanchor, default budget 100)

Authorised: (1) default N4 joint re-anchoring cap = 100 (configurable), (2) focused validation and documentation, (3) finalizing M2. **Not authorised and not done: M3 or any later milestone.** Semantics: `operators_N4_N5.md` §6; evidence: `test_and_validation.md` §8; decisions: `DECISIONS.md` items 28--32. Raw output: `operator_bench_raw.txt` (final), `operator_bench_raw_cap10000_before.txt`, `operator_bench_raw_reject_policy_before.txt`, `n4_reanchor_cap_sensitivity_raw.txt`, `evaluator_bench_after_m2_followup_raw.txt`, `m2_followup_mutation_results_raw.txt` (64 mutants).
**Not committed, not pushed.** HEAD is still `cc37e49`; `git status` is identical before and after this task.

| Item | Class |
|---|---|
| The paper specifies neither this N4 re-anchoring rule nor a combination cap | **PAPER (silence)** |
| Conservative re-anchoring of affected launch FLPs is permitted; recovery, drone customer, drone id, unaffected trips preserved; invalid recovery, missing replacement and evaluator failures are rejected | **DECISION (approved)** |
| Default joint-combination budget = 100, configurable (`N4Options::max_reanchor_variants`, 0 = unlimited) | **IMPLEMENTATION ASSUMPTION (approved empirical choice, A6b)**, not a paper parameter |
| The capped search is budget-bounded and potentially incomplete: it may miss the best candidate exhaustive enumeration would find; it is never described as exhaustive when the cap is reached | **DEVIATION / LIMITATION** |
| Reject policy kept and selectable (`AffectedTripPolicy::Reject`) | **DECISION** |

**Semantics:** deterministic odometer enumeration (stale trips in index order, last fastest, options in ratio order); generation stops once the budget is spent; the combinations reached are evaluated; the lowest-objective feasible one is kept (strict `<`, first generated wins ties). There is **no cap rejection** any more: `RejectReason::TooManyReanchorings` was removed (12 reasons remain). `cap_reached` is a flag orthogonal to the outcome; every candidate still receives exactly one outcome.
**Diagnostics:** per candidate `combinations_total`, `cap_reached`; per sweep `stale_launch_candidates`, `reanchored_candidates`, `trips_reanchored`, `variants_generated` (combinations generated), `evaluated` (evaluator calls), `cap_reached_candidates`, `rejected[InvalidRecovery]`, `rejected[NoReplacementFlp]`, `rejected[Eval*]`; `accepted + rejected_total() == generated`.

**Files changed (code):** `include/search/move_types.hpp`, `include/search/neighbourhood_n4.hpp`. `neighbourhood_n5.hpp`, `timing_engine.hpp`, `spatial_network.hpp` and `tests/reference/timing_engine_reference.hpp` are **byte-identical** (sha256 verified).
**Files changed (tests/bench):** `tests/test_n4_n5.cpp`, `tests/test_n4_n5_oracle.cpp`, `tests/mutation/m2_mutate.py`, `bench/bench_operators.cpp`.
**Docs:** this file, `DECISIONS.md` (items 21, 23, 27 revised; 28--32 new), `operators_N4_N5.md`, `test_and_validation.md` (§8), the project milestone summary; raw files listed above.

* **Tests:** `test_n4_n5` 233 assertions (was 203); `test_n4_n5_oracle` 1,117,041 checks (was 905,628), 0 failures; total 1,156,044. 11 of 11 CTest programs in Release and under ASan + UBSan; 0 warnings; **64 of 64 mutants detected** (35 M2 + 21 re-anchor + 8 cap); repeated runs byte-identical.
* **Effect and cost (n = 10 / 20 / 50, initial solutions):** feasible N4 candidates 13.3 / 8.2 / 0.8 % (Reject) $\to$ 17.8 / 24.5 / 2.0 % (Reanchor, budget 100 and 10,000 identical); re-anchored 4.4 / 16.4 / 1.2 %. N4 sweep: Reject 40 us / 128 us / 0.64 ms; budget 100: 2.47 / 7.92 / 67.8 ms; budget 10,000: 4.76 / 12.19 / 302 ms. Combinations evaluated per stale candidate (budget 100): 26.9 / 21.1 / 21.7 (10,000: 51.3 / 32.4 / 96.3). Candidates hitting the budget: 5.1 / 7.9 / 14.3 % of stale candidates. On these fixtures budget 100 and 10,000 give the same best candidate for 7/7, 14/14 and 27/27 anchors (an observation, not a guarantee).
* **The cap trades exhaustive search for bounded runtime.**
* **Correction to the approval premise:** the "about 12 x" came from the previous reject-over-cap semantics. Under the approved budget-bounded semantics, over-budget candidates are still evaluated on their first 100 combinations, so the n = 50 saving is **about 4.5 x** (67.8 ms versus 302 ms), n = 20 about 1.5 x, n = 10 about 1.9 x. Implemented as instructed; reported for your information.
* **N5 and evaluator:** unchanged; N5 generation 0.12 / 0.17 / 1.17 us per candidate, N5 sweep 1.16 / 2.69 / 5.16 ms, evaluator 1.5 / 2.7 / 10.4 us; `bench_evaluator` n = 50 populated 7.75--8.03 us, 30.8--32.5 x vs the reference (P1/P2 gains intact).
* **Not claimed:** M2 is not an exact reproduction of every paper detail; the N4 treatment of disturbed trips and the budget are project decisions/assumptions.

**Next (historical):** M3, implemented --- see the M3 section below. M2 is signed off.

## M3 --- neighbourhoods N8 and N7 (9 Nov 2026, instruction 08:45 IST) --- implemented and **signed off** (9 Nov 2026, with the decisions below)

Instruction: "M3 implementation: N8 + N7" (authorises M3 only). Contracts, interpretation, assumption A7, measurements and findings: `operators_N7_N8.md`. Evidence: `test_and_validation.md` section 9. Decisions: `DECISIONS.md` items 33--44. Register: `paper1_decision_register.md`, "M3 addendum". Raw output: `n7_n8_bench_raw.txt`, `n7_n8_bench_endurance_x3_raw.txt`, `n7_n8_oracle_stats_raw.txt`, `m3_mutation_results_raw.txt`, `m2_mutation_results_after_m3_raw.txt`, `operator_bench_after_m3_raw.txt`, `evaluator_bench_after_m3_raw.txt`.
**Not committed, not pushed.** HEAD is still `cc37e49`, branch unchanged. **M4 and later were not started** (no N3, N6, N1, N2, shakes, VNS, runner, CLI or dynamic code).

**Baseline before editing:** HEAD `cc37e49`, 7 tracked files modified and the M1/M2 untracked files present (preserved); the M2 final state was confirmed in the code (`max_reanchor_variants = 100`, no `TooManyReanchorings`) and the M2 counts reproduced in a Release build of the untouched tree (11 of 11 programs pass; `test_n4_n5` 233, oracle 1,117,041, 1,156,044 in total). The ASan + UBSan baseline is the M2 final result (11 of 11, recorded); a separate pre-edit sanitizer build was not made, but the final sanitizer run covers the same eleven unchanged programs plus the two new ones.

| Item | Class |
|---|---|
| N7 = truck node delete / drone node insert, N8 = drone node delete / truck node insert, with the quoted one-sentence descriptions; Figs. 12a/12b only via their captions | **PAPER** |
| The paper is silent on adjacency size, N7 launch/recovery choice, "available", other trips disturbed by the move, and the meaning of "optimal order of truck visits" | **PAPER (silence)** |
| D1 window 3 (skeleton-counted), D4 (best valid FLP on resulting arcs, recovery = next truck node), D5 (N8 all valid positions), D6 (no shakes), item 15 (disturbed trips rejected and named, never repaired), re-anchoring N4-only, strict `<`, first wins | **DECISION (approved earlier)** |
| **A7**: N7's window is measured on the original skeleton (merged arc at coordinate $2s_t$, eligible iff within $2w$; 5 gaps at $w=3$) | **IMPLEMENTATION ASSUMPTION** (open for confirmation) |
| N7 with no FLP on any eligible arc yields one `NoReplacementFlp` candidate; N8 "optimal order" = best position of the one inserted customer; `N8Options::insertion_window` optional restriction (default off); Rng overloads draw once | **IMPLEMENTATION ASSUMPTIONS** |
| Deviations from the paper | **none identified** |

**Files created (6 + docs):** `include/search/neighbourhood_n7.hpp`, `include/search/neighbourhood_n8.hpp`, `tests/test_n7_n8.cpp`, `tests/test_n7_n8_oracle.cpp`, `tests/mutation/m3_mutate.py`, `bench/bench_n7_n8.cpp`; `docs/static/operators_N7_N8.md` and seven raw result files (listed above).
**Files modified:** `include/search/move_types.hpp` (additive: `OperatorId::N7/N8`, `MoveKind::N7TruckToDrone/N8DroneToTruck`, comments), `CMakeLists.txt` (two tests, one bench), this file, `DECISIONS.md` (items 33--44), `docs/paper1_decision_register.md` (M3 addendum), `docs/static/test_and_validation.md` (section 9), the project milestone summary. **Deleted: none.**
**Byte-identical (sha256 checked):** `neighbourhood_n4.hpp`, `neighbourhood_n5.hpp`, `route_ops.hpp`, `candidate_window.hpp`, `drone_assignment.hpp`, `flp_arc_index.hpp`, `timing_engine.hpp`, `spatial_network.hpp`, `tests/reference/timing_engine_reference.hpp`.

**Semantics (one line each).** N7($t$): delete interior truck customer $t$; any other trip left with a launch FLP off its arc or recovering at $t$ makes the move one `AffectedTrips` candidate; otherwise one candidate per (eligible gap, FLP of that arc) with trip $(f, t, \text{arc end})$, FLP reused if on the route, lowest free drone id (`NoFreeDrone` otherwise), judged by the evaluator. N8(trip $i$): dissolve trip $i$ (drop its FLP iff unused), insert its customer at **every** gap (D5); gaps that still hold other trips' launch FLPs are `AffectedTrips`; the rest are judged by the evaluator.

* **Tests:** `test_n7_n8` 253 assertions, `test_n7_n8_oracle` 1,696,098 checks (independent brute force judged by the frozen reference evaluator; 360 bases; N7 128,212 and N8 9,120 candidates), 0 failures. Whole suite: **2,852,395 checks in 13 of 13 CTest programs** (was 1,156,044 in 11); all earlier programs unchanged.
* **Validation:** fresh Release build 0 warnings; 13 of 13 pass in Release and under ASan + UBSan (0 reports); repeated runs byte-identical; M3 mutation 52 of 53 detected (1 equivalent survivor, CW-02; two mutants that first survived exposed test gaps that were closed); M2 mutation suite 64 of 64 on unchanged M2 tests.
* **Benchmarks (n = 10 / 20 / 50, initial solutions):** N7 161 / 174 / 88 candidates, 0 feasible (endurance), sweep 290 / 516 / 882 us; N8 24 / 90 / 644 candidates, 75 / 67 / 21 % feasible, sweep 31 / 188 / 1,817 us; cost per evaluated candidate = the evaluator's own cost. N4/N5/evaluator regression: counts identical, times within machine variation, `bench_evaluator` $n=50$ 30.2--32.6 $\times$ faster than the frozen reference (P1/P2 gain intact), N4 budget-100 behaviour unchanged.
* **Findings reported, not acted on:** N7 is almost never endurance-feasible at paper scale; disturbed trips dominate N7/N8 at $n=50$ (an argument for the open question of re-anchoring in N7/N8, not authorised); at $n=50$ the per-call base evaluation is about 82 % of an N7 generation sweep.
* **Open for your decision:** A7 (N7 window centre); whether N7/N8 should get a bounded re-anchoring alternative; the N8 `insertion_window` default; what "optimal order of truck visits" should cover; the original paper PDF is not in the repository.

**M3 sign-off (with the M4 instruction of 09:42 IST).** (1) **A7 approved**: the N7 window is centred on the original skeleton around the deleted customer, merged-arc coordinate $2s_t$ (still an implementation assumption, not a paper fact). (2) **N8 stays unrestricted by default**: every valid insertion position (D5); the optional `insertion_window` stays off. (3) **N7's zero feasible candidates on the paper-scale benchmark stay documented** as measured; endurance and geometry are not to be changed to force feasibility.


## M4 --- neighbourhoods N3 and N6 (9 Nov 2026, instruction 09:42 IST) --- implemented and **signed off** (9 Nov 2026, 12:30 IST, with the A8 revision below; the figures in this section are the original M4 figures)

Instruction: "M4 implementation: N3 + N6" (authorises M4 only). Contracts, paper facts, assumptions A8--A12, tests, measurements and findings: `operators_N3_N6.md`. Evidence: `test_and_validation.md` section 10. Decisions: `DECISIONS.md` items 45--53. Register: `paper1_decision_register.md` (M4 addendum).
**Not committed, not pushed.** HEAD is still `cc37e49`, branch unchanged. **M5 and later were not started** (no N1, N2, shakes, VNS, runner, CLI or dynamic code).

**Baseline before editing:** HEAD `cc37e49`, 8 tracked files modified and the M1--M3 untracked files present (preserved); the M3 state was reproduced in a Release suite (13 programs, 2,852,395 checks) before any edit. The N4 cap-100 wording was grepped across the docs: the only remaining cap-10,000 / `TooManyReanchorings` text is in passages already marked superseded (this file lines 101--124, `test_and_validation.md` N4 reject-policy table, `operators_N4_N5.md` follow-up note, `DECISIONS.md` items 21 and 23); no unmarked stale text was found, none changed.

| Item | Class |
|---|---|
| N3 = "drone service node relocate": reposition the drone service node of the drone trip, searching the **adjacent trip** of the original trip (Fig. 10c: trips $\{C1,C4,C3\}$, $\{C3,C2,C5\}$ become $\{C1,C2,C3\}$, $\{C3,C4,C5\}$, route unchanged); N6 = "D-D service node swap": randomly select a drone trip and search an adjacent drone service node of **other** drone trips (Fig. 11b: customers C2 and C4 exchanged between a drone-1 and a drone-2 trip); a trip serves exactly one customer; both pick the best candidate by customer waiting time. Figures known only via captions and the register | **PAPER** |
| The paper is silent on the extent of "adjacent trip", direction of the search, what "other drone trips" excludes, what "adjacent" means for N6, and whether launch/recovery nodes move | **PAPER (silence)** |
| D3 (N3: consecutive trips of the same drone), D1 (window 3, skeleton-counted), D4 *not applicable* (nothing is disturbed, so no FLP is selected), item 15 (no invented repair; re-anchoring N4-only), strict `<`, first wins, eager pool | **DECISIONS** |
| **A8** N6 partners are trips of a *different* drone by default (`N6Options::include_same_drone` admits any other trip); **A9** only the customers move (launch FLPs, recovery nodes and drone ids stay with their trips); **A10** N6 adjacency = D1 window on launch-gap half-steps; **A11** N3 searches predecessor, then successor in the drone's sequence (ordered by launch route position, recovery position, index); **A12** the `Rng` overloads draw once with `uniform_index(#trips)` | **IMPLEMENTATION ASSUMPTIONS** (open for confirmation) |
| Deviations from the paper | **none identified** |

**Semantics (one line each).** N3($i$) and N6($i$) swap the `service_customer` of two trips and nothing else; no FLP, recovery node, drone id, route or trip order changes, so no trip is disturbed and feasibility is exactly the evaluator's verdict (reachable reasons: endurance and `EvalOther`; the other reasons are unreachable by construction and were never observed). N3: the anchor's predecessor then successor on its drone (a lone trip: zero candidates, `Ok`). N6: drone customers of other-drone trips whose launch-gap half-step is within $2w$, nearest first, ties by customer id.

**Files created:** `include/search/trip_exchange.hpp`, `include/search/neighbourhood_n3.hpp`, `include/search/neighbourhood_n6.hpp`, `tests/test_n3_n6.cpp`, `tests/test_n3_n6_oracle.cpp`, `tests/mutation/m4_mutate.py`, `bench/bench_n3_n6.cpp`, `docs/static/operators_N3_N6.md`, and raw evidence files `n3_n6_*_raw.txt`, `m4_mutation_results_raw.txt`, `m1_m3_mutation_spotcheck_after_m4_raw.txt`, `operator_bench_after_m4_raw.txt`, `n7_n8_bench_after_m4_raw.txt`, `evaluator_bench_after_m4_raw.txt`.
**Files modified:** `include/search/move_types.hpp` (additive: `OperatorId::{N3,N6}` — the enum is now `N3,N4,N5,N6,N7,N8` and is only compared, never cast —, `MoveKind::{N3Exchange,N6Swap}`, `Move::partner_trip`; `kRejectReasonCount` still 12), `CMakeLists.txt` (two tests, one bench), this file, `DECISIONS.md`, `docs/paper1_decision_register.md`, `docs/static/test_and_validation.md`; Project doc `claude/static-reproduction-milestones.md`. **Nothing deleted.**
**Byte-identical (sha256 checked before/after):** `neighbourhood_n4.hpp`, `neighbourhood_n5.hpp`, `neighbourhood_n7.hpp`, `neighbourhood_n8.hpp`, `route_ops.hpp`, `candidate_window.hpp`, `drone_assignment.hpp`, `flp_arc_index.hpp`, `timing_engine.hpp`, `spatial_network.hpp`, `tests/reference/timing_engine_reference.hpp`.

* **Tests:** `test_n3_n6` 565 assertions (hand-derived fixtures: Fig. 10c, chain, two/three drones, exact ties, endurance boundary, depot recovery, window boundaries, errors, flags, determinism), `test_n3_n6_oracle` 595,093 checks (independent brute force judged by the frozen reference; 640 bases; N3 7,036 and N6 5,806 candidates; 0 failures). Whole suite: **3,448,053 checks in 15 of 15 CTest programs** (was 2,852,395 in 13; M4 adds 595,658). No M1--M3 test was changed.
* **Validation:** fresh Release build 0 warnings (`-O3 -DNDEBUG -Wall -Wextra -Wpedantic`), 15 of 15 pass; fresh ASan + UBSan build 0 warnings, 15 of 15 pass, 0 reports; repeated runs byte-identical (3 runs of each new program); M4 mutation 54 of 58 detected, the 4 survivors analysed (N3-12 and N6-07 equivalent; CW-04 and CW-06 do not affect N3/N6 and are killed by the M2/M3 suites); spot check of 33 older M2/M3 mutants: 33 of 33 detected (the full M2/M3 mutation suites were **not** re-run: their mutated sources and tests are byte-identical to the signed-off runs).
* **Benchmarks (paper parameters, n = 10 / 20 / 50, initial solutions, which fly every trip with drone 0):** N3 4 / 10 / 44 candidates, 0 / 0 / 9.1 % feasible (the rest endurance), sweep 10.8 / 44.0 / 664 us; **N6 (default) 0 candidates at all three sizes** (one drone in use), sweep 5.3 / 19.0 / 278 us (base evaluation and neighbour index only); cost per evaluated candidate = the evaluator's own (1.6 / 2.8 / 9.9 us). Labelled variants (alternate drone ids; endurance $\times3$; `include_same_drone`) are in `operators_N3_N6.md` section 8.
* **Regression:** N4/N5/N7/N8 candidate and outcome counts identical to the M3 runs, times within machine variation; evaluator at $n=50$ 33.3--35.2 $\times$ faster than the frozen reference (P1/P2 intact).
* **Findings reported, not acted on:** N6 under A8 is empty whenever all trips use one drone (including every initial solution); most N3 exchanges are endurance-infeasible at paper scale (same as N7).
* **Open for your decision:** A8 (N6 partners: different-drone trips versus any other trip), A9 (customers only versus re-selecting launch/recovery), A10 (N6 adjacency), A11 (N3 both directions), the missing paper PDF (Figs. 10c/11b only via captions).

**M4 sign-off (9 Nov 2026, 12:30 IST).** The user approved M4 with these decisions: **A8 revised** (N6's default partner set is *any other trip*, same-drone trips included; `N6Options::include_same_drone` now defaults to `true`, `false` restricts to different drones; N3/N6 candidate overlap is acceptable), **A9** (customers-only exchange), **A10** (D1 window on launch-gap half-steps) and **A11** (predecessor, then successor) **approved** as implemented. Only the A8 default and what pinned it was changed (`neighbourhood_n6.hpp`, `tests/test_n3_n6.cpp`, `tests/test_n3_n6_oracle.cpp`, `tests/mutation/m4_mutate.py` N6-13, `bench/bench_n3_n6.cpp`, `operators_N3_N6.md` (note and section 10), raw evidence files, `DECISIONS.md` item 54, the register's M4 addendum, `test_and_validation.md` section 10). Results after the change: `test_n3_n6` **587** assertions and `test_n3_n6_oracle` **613,135** checks, 0 failures; N6 mutants 13 of 14 killed (N6-07 equivalent); N6 default 2 / 8 / 108 candidates (0 / 0 / 4 feasible; 3.70 % at $n=50$) on the paper-scale initial solutions, the restricted variant 0. The superseded A8 text above (the semantics line, "Open for your decision", finding "N6 ... zero candidates") describes the original default. M1--M3 and the rest of M4 were not re-audited.

**Next (historical):** M6; M5 was signed off later, see below.

---

## M5 --- neighbourhoods N1 and N2 (9 Nov 2026, instruction 12:30 IST) --- implemented and **signed off** (9 Nov 2026, 13:19 IST, with A13--A18 approved and the N1 customer-node omission kept as a documented reproduction limitation)

**Baseline before editing:** HEAD `cc37e49`, 8 tracked files modified and the M1--M4 untracked files present (preserved); the M4 state was reproduced (after the A8 revision) before any M5 edit. No repository re-audit; only the files and paper material for N1/N2 were read (`claude/han-chi-sun-2026-extraction.md`, register Issues 7 and 8, `route_ops.hpp`, `candidate_window.hpp`, `trip_exchange.hpp`).

| Item | Class |
|---|---|
| N1 = "launch node relocate": reposition the launch node of a drone trip, "searching adjacent customer nodes and flexible launch nodes of the drone launch node in original drone trip" (Sec 5.3 p20--21, Fig. 10a); N2 = "recover node relocate": reposition the recovery node, "searching adjacent customer nodes of the drone recover node" (p21, Fig. 10b); both pick the best candidate; "performed several times" during the local search (Sec 6.1). Figures known only via captions | **PAPER** |
| The paper is silent on the meaning of "adjacent" (size, direction), the fate of the old launch FLP, the drone identity, and the order of launch and recovery | **PAPER (silence)** |
| Register Issue 7 (launches only at FLPs), D1 (window 3, skeleton-counted), D2 (repetition is VNS, M7), item 15 (no repair), explicit FLP drop, strict `<`, first wins, eager pool | **DECISIONS** |
| **A13** N1 candidates = every FLP on the arcs within $w$ gaps of the old launch (current launch excluded; shared and unrouted FLPs included); **A14** N2 candidates = skeleton nodes (customers and end depot) within $w$ of the old recovery, ascending, never the start depot; **A15** a launch/recovery that breaks "recovery after launch" is `InvalidRecovery` (generated, named, not evaluated, not repaired); **A16** the drone id never changes; **A17** the old launch FLP is dropped iff unused; **A18** the `Rng` overloads draw once with `uniform_index(#trips)` | **IMPLEMENTATION ASSUMPTIONS** (open for confirmation) |
| N1 does not consider customer launch nodes ("adjacent customer nodes"): FLP-only (existing Issue 7 decision) | **DEVIATION (existing)** |

**Semantics (one line each).** N1($i$) replaces the trip's launch FLP by another FLP of the window's arcs (inserted in ratio order if new, old one dropped if unused), keeping drone id, customer, recovery and every other trip; candidates whose launch is not before the recovery are `InvalidRecovery`. N2($i$) replaces the recovery node by a skeleton node of the window, route untouched; candidates not after the launch are `InvalidRecovery`. Feasibility is the evaluator's verdict alone (endurance, (28) sequencing, fleet capacity classified, never repaired or overridden).

**Files created:** `include/search/trip_relocate.hpp`, `include/search/neighbourhood_n1.hpp`, `include/search/neighbourhood_n2.hpp`, `tests/test_n1_n2.cpp`, `tests/test_n1_n2_oracle.cpp`, `tests/mutation/m5_mutate.py`, `bench/bench_n1_n2.cpp`, `docs/static/operators_N1_N2.md`, and raw evidence `n1_n2_*_raw.txt`, `m5_mutation_results_raw.txt`, `m5_release_ctest_raw.txt`, `m5_sanitizer_ctest_raw.txt`, `operator_bench_after_m5_raw.txt`, `n7_n8_bench_after_m5_raw.txt`, `n3_n6_bench_after_m5_raw.txt`, `evaluator_bench_after_m5_raw.txt`.
**Files modified:** `include/search/move_types.hpp` (additive: `OperatorId::{N1,N2}`, `MoveKind::{N1LaunchRelocate,N2RecoveryRelocate}`, `Move::replaced_node`; `kRejectReasonCount` still 12), `CMakeLists.txt` (two tests, one bench), this file, `DECISIONS.md`, `docs/paper1_decision_register.md`, `docs/static/test_and_validation.md`; Project doc `claude/static-reproduction-milestones.md`. **Nothing deleted.**
**Byte-identical (sha256 checked against the M4 end):** `candidate_window.hpp`, `drone_assignment.hpp`, `neighbourhood_n4.hpp`, `neighbourhood_n5.hpp`, `neighbourhood_n7.hpp`, `neighbourhood_n8.hpp`, `route_ops.hpp`, `timing_engine.hpp`, `spatial_network.hpp`, `flp_arc_index.hpp`, `tests/reference/timing_engine_reference.hpp`; `neighbourhood_n3.hpp`, `neighbourhood_n6.hpp`, `trip_exchange.hpp` untouched by M5.

* **Tests:** `test_n1_n2` 419 assertions (hand-derived fixtures: window 0/1/3/9, endurance inclusive boundaries, shared launch FLP, (28) versus fleet capacity, exact ties, depot recovery, errors, flags, Rng), `test_n1_n2_oracle` 3,470,761 checks (independent brute force judged by the frozen reference; 640 bases, windows 0..3; N1 231,964 and N2 25,986 candidates; 0 failures). Whole suite: **6,937,297 checks in 17 of 17 CTest programs** (was 3,466,117 in 15 after the A8 revision). No earlier test was changed or weakened.
* **Validation:** fresh Release build 0 warnings (`-O3 -DNDEBUG -Wall -Wextra -Wpedantic`), 17 of 17 pass; fresh ASan + UBSan build 0 warnings, 17 of 17 pass, 0 reports; repeated runs byte-identical; **M5 mutation 62 of 62 killed**; M1--M4 mutation suites not re-run (their sources are unchanged).
* **Benchmarks (paper parameters, $n$ = 10 / 20 / 50):** N1 156 / 234 / 750 candidates, 7.7 / 7.7 / 9.2 % feasible (rest `InvalidRecovery` ~43--51 %, endurance, a few sequencing), sweep 137 / 402 / 4,694 us; N2 12 / 30 / 129 candidates, 1 feasible each, sweep 17.6 / 59.8 / 874 us; cost per evaluated candidate = the evaluator's own (1.6 / 2.8 / 10.2 us). Variants (endurance $\times3$, alternate drones) in `operators_N1_N2.md` section 8.
* **Regression:** N3/N4/N5/N6/N7/N8 candidate and outcome counts identical to the earlier runs; evaluator still $\times5$--$\times10$ faster than the frozen reference (noisier VM).
* **Findings reported, not acted on:** about half of N1's generated candidates are `InvalidRecovery` (the symmetric window includes FLPs behind the recovery); 80--86 % of evaluated N1 candidates are endurance-infeasible at paper scale; N2 has at most one feasible candidate per sweep at paper scale.
* **Open for your decision:** A13 (N1 candidate set), A14 (N2 candidate set), A15 (`InvalidRecovery` named versus silently skipped), A16 (drone id fixed), A17 (old FLP dropped iff unused), A18 (Rng draw), the N1 FLP-only deviation, the extent of "adjacent", and the missing paper PDF.

**M5 sign-off (9 Nov 2026, 13:19 IST).** The user approved **A13** (all eligible FLPs on the D1 window's arcs, current launch excluded), **A14** (skeleton-node recovery candidates within the window, start depot excluded), **A15** (explicit `InvalidRecovery` accounting), **A16** (drone id unchanged), **A17** (old launch FLP dropped only when unused) and **A18** (one uniform draw over trip indices). **N1's omission of customer launch nodes stays an explicitly documented reproduction limitation**: N1 is not to be silently expanded to customer launch nodes without further paper evidence. No code changed for the sign-off.

**Next:** M6 (shaking operators only), recorded below.




## M6 --- shaking operators (9 Nov 2026, instruction 13:19 IST) --- implemented and **signed off** (9 Nov 2026, 13:56 IST; A19--A29 approved, A26 with a clarification)

**Baseline before editing:** HEAD `cc37e49`, 8 tracked files modified and the M1--M5 untracked files present (preserved). No repository re-audit; only shake-related paper text (`claude/han-chi-sun-2026-extraction.md` Sec 5.4, p23), the register entries and the existing helpers (`route_ops.hpp`, `SearchContext`, `classify_evaluation`, `Rng`) were read. **Contracts written before implementation:** `docs/static/shakes.md` sections 0--5 (paper facts / approved decisions / implementation assumptions A19--A29 kept apart).

* **Implemented:** Random_Reverse, Random_Exchange, Random_Shift, Random_Drone_Remove and the uniform `shake()` in `include/search/shakes.hpp` (explicit-parameter cores + randomised entries with documented draw orders). D6 preserved: only trips whose launch arc vanished (`LaunchArcVanished`) or whose recovery is not strictly after the launch (`RecoveryNotAfterLaunch`) are dissolved, with counts and reasons recorded; everything else is preserved untouched; nothing is repaired or re-anchored; the result is judged by the evaluator and returned as judged. Returned customers go to a uniformly drawn free gap; with no free gap a drawn gap's trips are dissolved (`ArcSplitByReinsertion`).
* **Files:** new `include/search/shakes.hpp`, `tests/test_shakes.cpp`, `tests/test_shakes_oracle.cpp`, `tests/mutation/m6_mutate.py`, `bench/bench_shakes.cpp`, `docs/static/shakes.md`, raw outputs `docs/static/shakes_*_raw.txt`, `m6_*_raw.txt`, `*_after_m6_raw.txt`; edited `CMakeLists.txt` (3 entries), `DECISIONS.md` (items 67--70), `docs/paper1_decision_register.md` (M6 addendum), `docs/static/test_and_validation.md` (section 12), this file. No existing source, test or the evaluator changed (15 guard sha256 identical to the M5 end).
* **Validation:** fresh Release build 0 warnings, **19 of 19** pass; `test_shakes` 675 assertions, `test_shakes_oracle` 728,109 checks; ASan + UBSan 0 reports; repeated runs byte-identical; **M6 mutation 70 of 71 killed, 1 equivalent survivor**; N1--N8 benchmark counts identical to M5, evaluator unchanged.
* **Findings:** shakes cost 4--5 us (n = 10) to 33--38 us (n = 50) per full call; from the initial solution every shake result was evaluator-feasible; dissolutions are almost all `LaunchArcVanished`; the fallback and `RecoveryNotAfterLaunch` are rare (8 and 85 of 42,888 reinsertions / 35,796 judged shakes on random-walk bases); 0.4 % of random-walk results were evaluator-infeasible (endurance).
* **Assumptions:** A19--A29 approved at sign-off (`shakes.md` section 5). **A26 clarification:** only trips whose launch FLPs the reinsertion genuinely invalidates are dissolved, with the affected trips and the exact reason recorded; trips that merely occupy another gap are never dissolved (already the behaviour; no code change).
* **Not committed, not pushed.** HEAD still `cc37e49`. **M7 and later were not started.**

**Next:** M7 (VNS), recorded below.


## M7 --- VNS driver (9 Nov 2026, instruction 13:56 IST) --- implemented and **signed off** (9 Nov 2026, 14:34 IST; A30--A37 approved, A38 approved with a reporting condition)

**Baseline before editing:** HEAD `cc37e49`, 8 tracked files modified and the M1--M6 untracked files present (preserved). No repository re-audit; only the VNS paper text (Algorithm 1, Sec 5/6.1 in `claude/han-chi-sun-2026-extraction.md`), D2/D6/D7/D8, the operator and shake interfaces and the seed-stream API were read. **Contract written before implementation:** `docs/static/vns.md` sections 0--5 (paper facts / approved decisions / assumptions A30--A38 kept apart).

* **Implemented:** `run_vns` in `include/search/vns.hpp`. Pass = shake from the current accepted solution (uniform, M6) -- rejected (`NotApplicable` or evaluator-infeasible) shakes are redrawn, never repaired, up to `max_shake_attempts`, else the pass is skipped (still counted, still cooled) -- then the sequential local search (Sec 6.1 order N4, N5, N8, N3, N6, N7, then N1/N2 alternately twice, D2) with SA acceptance, restart on improvement and accepted candidates carried forward, then one geometric cooling step. Separate best-so-far returned. Termination: `iter_max` (counter fixed), optional no-improvement and wall-clock limits. Two seed streams (Vns, Shake) from base seed + run index: bit-identical reproducibility. Deviations from the printed pseudocode (all D8): section 4 of `vns.md`.
* **Files:** new `include/search/vns.hpp`, `tests/test_vns.cpp`, `tests/test_vns_oracle.cpp`, `tests/mutation/m7_mutate.py`, `bench/bench_vns.cpp`, `docs/static/vns.md`, raw outputs `docs/static/vns_*_raw.txt`, `m7_*_raw.txt`, `*_after_m7_raw.txt`; edited `CMakeLists.txt` (3 entries), `DECISIONS.md` (items 71--76), `docs/paper1_decision_register.md` (M7 addendum), `docs/static/test_and_validation.md` (section 13), this file. No existing source, test, evaluator or frozen reference changed (15 guard sha256 identical).
* **Validation:** fresh Release build 0 warnings, **21 of 21** ctest programs; `test_vns` 85 assertions; `test_vns_oracle` 29,447 checks (independent loop, exact per-pass comparison, frozen-reference re-judgement); ASan + UBSan 0 reports; repeated runs byte-identical; **M7 mutation 51 of 52 killed, 1 equivalent survivor**; N1--N8 and shake benchmark counts identical to the M6 end. Whole suite about 7,695,000 checks.
* **Pilot (D7):** defaults `iter_max = 2000`, $T_0 = 0.01 f(x_{init})$, $\alpha = 0.995$, acceptance reference = the printed $f(x)$ (clearly better than the alternative at n = 20 and 50). Gains over the initial solution at the defaults: 14.6 % / 19.6 % / 20.8 % for n = 10 / 20 / 50 at 0.2 / 0.25 / 1.4 s per run. One instance per size; not compared with the paper.
* **Found while testing:** at a very high temperature the local search can loop indefinitely (SA moves alternating with improvements); guard `max_local_steps` added (A38), never triggered at pilot temperatures.
* **Assumptions:** A30--A37 approved at sign-off; A38 approved conditionally and the condition implemented (guard trigger reported per pass and in counters, never silent; `vns.md` section 10).
* **Not committed, not pushed.** HEAD still `cc37e49`. **M8 and later were not started.**

* **M7 closure (14:34 IST):** `VnsTraceRow::end` (`Completed`/`Skipped`/`GuardTruncated`), `local_steps`, `VnsResult::{passes_completed, local_steps_total, max_local_steps_in_pass, guard_triggered()}`; `test_vns` 108 assertions, oracle extended (29,447 checks), mutation 57/58 (V-10 equivalent); full 21-program ASan + UBSan run: 21/21, 0 reports. Default threshold 0 and strict `<` confirmed.

**Next:** M8, recorded below.


## M8 --- runner, result writer, configuration wiring and CLI (9 Nov 2026, instruction 14:34 IST) --- implemented and **signed off** (9 Nov 2026, 15:35 IST; A39--A44 approved after two validation gaps were closed)

**Baseline before editing:** HEAD `cc37e49`, nothing committed; M1--M7 files preserved. M7 was closed first (guard reporting, full-suite ASan + UBSan; section above), then M8 started. Spec, usage, schema, assumptions and limits: `docs/static/runner_cli.md`.

* **Implemented (separate pieces):** `include/config/experiment_config.hpp` (one 48-key field table drives parsing, `--set`, canonical dump and `list-keys`; defaults are the algorithm structs' own initialisers, nothing duplicated; strict validation); `include/runner/static_runner.hpp` (instance, initial solution, `runs` unchanged-`run_vns` runs per dataset, per-run verification); `include/runner/result_writer.hpp` (`results.json`, `runs.csv`, `traces.csv`, `timing.json`, `manifest.json`, `config.resolved.cfg`; temp-file + rename; overwrite protection); `include/runner/cli.hpp` + `apps/tdr_static.cpp` (`run`, `check`, `print-config`, `list-keys`, `help`, `version`; exit codes 0/1/2/4). F1/F2 constants (TSP exact limit / restarts / seed 42, k-means) moved into the configuration with their former values as defaults (initial solutions unchanged).
* **Preserved:** VNS and operator behaviour (runner runs bit-identical to direct `run_vns` calls); deterministic seeding (derive_seed streams by run index; run r of an experiment equals the one-run experiment `first_run_index = r`); no default tuned (`list-keys` = the M7 defaults).
* **Reproducibility:** every file except `timing.json` is byte-identical across repeated runs; `config.resolved.cfg` is a valid `--config` file that reproduces `results.json`, `runs.csv`, `traces.csv` byte for byte (tested end to end).
* **Files created:** the four headers above, `apps/tdr_static.cpp`, `tests/test_experiment_config.cpp`, `tests/test_runner_cli.cpp`, `tests/mutation/m8_mutate.py`, `docs/static/runner_cli.md`, raw outputs `docs/static/m8_*_raw.txt`, `m7_closure_*_raw.txt`. **Edited:** `CMakeLists.txt` (3 targets), `include/search/vns.hpp` (M7-closure reporting only), `tests/test_vns.cpp`, `tests/test_vns_oracle.cpp`, `tests/mutation/m7_mutate.py`, docs. No operator, shake, evaluator or frozen-reference source changed.
* **Validation:** fresh Release build 0 warnings, **23 of 23** ctest programs (about 7,695,723 checks); `test_experiment_config` 24, `test_runner_cli` 63 assertions at first delivery (25 and 76 after the M8 closure; 23 of 23 programs); ASan + UBSan 0 reports (new programs; the full 21-program suite was run at the M7 closure); M8 mutation **44 of 44 killed** (ten mutants first survived and drove ten test improvements; one comparison was vacuous and was made non-vacuous).
* **Benchmarks:** the 10-run x 2000-pass protocol through the CLI reproduces the M7 figures: n = 10 / 20 / 50 mean gain 14.63 / 19.58 / 20.54 %, 0.21 / 0.28 / 1.18 s per run, 2.1 / 4.0 / 11.9 s per experiment, 37--42 kB of `results.json`.
* **Assumptions A39--A44** (`runner_cli.md` section 8; approved at the M8 sign-off). **Not committed, not pushed.** (M9 followed after this sign-off, see below.)

* **M8 closure (15:35 IST):** (1) CLI-level failure test: a valid configuration whose travel times overflow (`vehicle.truck_speed_mps=3e-308`) now fails cleanly through a runner preflight (exit code 1, error named on stderr, complete parseable record with `ok = false` written); found on the way: the Stage-1 heuristic TSP indexes with -1 when all travel times are infinite (heap-buffer-overflow under ASan, segfault in Release) -- **reported, `src/tsp` not changed** (the preflight is a runner-level input check). (2) Windows: native validation impossible from the Linux sandbox; instead the three M8 programs were cross-compiled for Windows (Zig/clang, mingw runtime) and run under Wine: 25/25 and 76/76 pass incl. paths with spaces, CRLF + BOM configuration, rename-over-existing overwrite, exit codes 0/1/4; the Windows and Linux builds give **byte-identical** results.json / runs.csv / traces.csv / config; BOM + CRLF configuration support added; exact PowerShell commands and the limitations (Wine is not NTFS/OneDrive, MSVC untried) in `windows_validation.md`. (3) Schema: results schema revised to `tdr.static.results/2` (adds `tool`, `units`, per-dataset `nodes` table with coordinates and FLP arcs), documented field by field in `output_schema.md`, key sets pinned by test. `test_runner_cli` 76, `test_experiment_config` 25.


## M9 --- validation and comparison with the paper (9 Nov 2026, instruction 15:35 IST) --- implemented and **signed off** (9 Nov 2026, 16:13 IST; A45--A47 approved with qualifications; a validation and comparison milestone, NOT a numerical reproduction)

Scope kept: validation only. **No file under `include/` or `src/` changed**; evaluator, frozen reference and operators untouched (15 guard hashes OK). Full report: `docs/static/validation_vs_paper.md`; raw outputs `docs/static/m9_*_raw.txt`.

* **Protocol:** 3 sizes (10/20/50) x 4 datasets x 3 independent seed sets x 10 runs = 360 runs, configurations `config/m9/n{10,20,50}_seed{1,2,3}.cfg`, outputs `results/m9/`; all runs ok and re-verified, guard never triggered, 0 skipped passes; re-run byte-identical (CTest `m9_protocol_reproduces`, negative-tested).
* **Results (minutes):** mean z_avg 938 / 2482 / 9817 (95 % CI 858..1017 / 2379..2585 / 9491..10144), mean improvement over the initial solution 20.2 / 16.5 / 23.1 %, 0.18 / 0.28 / 0.68 s per run; n = 50 still improving at 2000 passes (4x budget: -2.0..-3.4 % mean).
* **Paper comparison (Table 3, VNS columns; unit hypothesis: minutes):** exact reproduction / assumptions / unavailable data separated in section 2 of the report. Our mean z_avg is 0.45 / 0.33 / 0.47 of the paper's and the paper's means lie outside our instance ranges; not explained by depot position (corner: +6..+8 %) or endurance (x2: no change); the objective was confirmed independently (exact DP, 2.9e-11 s). MILP columns, Delta z, instance coordinates, Dalian C1--C4: unavailable. Unresolved, reported to the user (Q1).
* **Independent reference:** exact truck-only optimum at n = 10: VNS z_best equals it in 10 of 12 instances (+0.54 % and +0.63 % in the other two, all 10 runs stalled); drones never helped.
* **Feasibility analysis (no change to endurance/geometry/operators):** trip-based operators have no anchor in 93..99.6 % of the calls because the best solutions are drone-free (357 of 360 runs); N7 is mostly endurance-infeasible (reach about 20 km versus customer spacing 19 / 13 / 8.5 km) and its feasible candidates are worse (+0.0..+2.6 %); `bench/m9_feasibility_probe.cpp` (read-only) tabulates rejection reasons; construction-error reasons are 0.
* **Labelled sensitivity diagnostics** (not the protocol, defaults unchanged): depot corner, endurance 3600 s, iter_max 8000.
* **Added:** `config/m9/*.cfg` (9), `scripts/m9_run_all.sh`, `m9_analyse.py`, `m9_sensitivity.py`, `m9_exact_truck_only.py`, `m9_check.cmake`, `bench/m9_feasibility_probe.cpp` (CMake target under TDR_BUILD_BENCH, test `m9_protocol_reproduces`), `results/m9/`, `results/m9_sensitivity/`, report and raw files.
* **Approved with qualifications (16:13 IST):** A45 (minutes as a *provisional* comparison unit; the paper does not confirm its unit), A46 (12 instances per size; the confidence intervals are intervals over generated instances, and a seed set changes both instances and search streams), A47 (exact truck-only optimum strictly as an independent reference for the truck-only sub-problem, not a substitute for the unavailable MILP optimum). The objective discrepancy (ours 0.33--0.47 of the paper's means), the FLP-count discrepancy (our pools 7.5--11 times the paper's Table 4) and the unavailable paper data remain explicit findings and limitations. **P1--P3 remain unapproved proposals (not implemented):** P1 iteration budget for n >= 50, P2 diversification for the two stalled n = 10 instances, P3 heuristic-TSP guard for non-finite travel times. **Not committed, not pushed.** M10 and the dynamic extension not started.

## M10 --- documentation freeze (9 Nov 2026, instruction 16:13 IST) --- complete (awaiting the user's review of the freeze)

Scope kept: documentation only. **No file under `include/`, `src/`, `apps/` or `tests/` changed in M10; no algorithm, evaluator, endurance, geometry or default changed; P1--P3 remain unapproved proposals.** Guard hashes unchanged.

* **Added:** `INDEX.md` (authoritative milestone table, specification-coverage matrix, where each class of statement lives), `configuration_reference.md` (48 keys with default, provenance class and meaning), `static_reproduction_summary.md` (thesis-ready summary; the objective and FLP-count discrepancies are findings 1 and 2), `scripts/m10_doc_check.py` (consistency checks), `m10_checks_raw.txt`.
* **Qualifications of the M9 approval applied** (A45 provisional unit, A46 intervals over generated instances, A47 reference only for the truck-only sub-problem; M9 is not a numerical reproduction) in `validation_vs_paper.md`, `DECISIONS.md` item 89, the register and this log.
* **Stale statements corrected:** README (status, "still to be implemented", layout, build/run instructions); `experiment_methodology.md` (VNS seed formula $1000\cdot$dataset+run replaced by the implemented `derive_seed(base, Vns, run index)`; "planned" items marked implemented or not done; objective summation order; MILP substitute); `runner_cli.md` (test counts 24/63 -> 25/76; "M8 not signed off", "M9 not started"); `shakes.md`, `vns.md`, `operators_N3_N6.md` ("awaiting approval/review", "not signed off", "M7 was not started"); `test_and_validation.md` (plan table V3--V8 statuses; final counts, new section 16); `DECISIONS.md` (items 67, 72, 76, 78, 82, 88 status tags; two OPEN/planned notes closed or marked not done; items 89--91 added); `paper1_decision_register.md` (three addendum headings, M9 note); `MILESTONES.md` (frozen-status note, M1 heading, two "awaiting" statements, M9 status, counts); `CODE_REPORT.md` (scope note); `scripts/m9_run_all.sh` (the iteration-budget diagnostics were missing from the reproduction script). Per-milestone figures inside older sections are intentionally left as they were at that milestone.
* **Checks run:** see `m10_checks_raw.txt` (documentation check, fresh Release build with `-Wall -Wextra -Wpedantic` and bench ON: 0 warnings, 24 of 24 CTest programs including the byte-identical M9 re-run, assertion total 7,695,737 recomputed from the programs, reproduction of two labelled sensitivity experiments from the documented commands, guard hashes).
* **Remaining limitations:** as `static_reproduction_summary.md` section 6: no numerical reproduction (objective 0.33--0.47 of the paper's mean, FLP pools 7.5--11 times the paper's), paper unit unconfirmed, no instance coordinates, no MILP, native Windows untested, pilot VNS parameters, unapproved P1--P3. Nothing committed or pushed. The dynamic extension is not started.
