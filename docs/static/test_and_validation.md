# Tests and staged validation (static reproduction)

## 1. Validation ladder (from the Phase K brief)

| # | Stage | Evidence | Status |
|---|---|---|---|
| V1 | Hand-checkable instances | every expected value derived by hand in the test comments (`test_timing`, `test_init`, `test_search_infra`) | done for evaluator, builder and M1 helpers |
| V2 | Feasibility and objective | evaluator tests incl. Stage 1 input validation and Stage 2 SR rule; independent Python re-derivations | done (Stages 1--2) |
| V3 | Neighbourhoods | per-neighbourhood hand cases, "every returned candidate passes the evaluator", exactly-once, no silent repair, fuzz | done for N1-N8 (M2--M5, §6-§11) |
| V4 | Shakes | valid-or-explicitly-rejected, dissolve counts, seed reproducibility | done (M6, §12) |
| V5 | VNS convergence | best-so-far monotone, counter exact, stopping rules, never worse than the initial solution | done (M7, §13) |
| V6 | Controlled-seed repeats | byte-identical outputs for repeated seeded runs | done (M8, §14; re-run byte-identity also in `m9_protocol_reproduces`, §15) |
| V7 | Benchmarks | 10/20/50-customer protocol, 10 runs, statistics | done (M9, §15; our protocol uses 12 instances per size) |
| V8 | Comparison with the paper's tables | protocol in `experiment_methodology.md` | done (M9, `validation_vs_paper.md`); no numerical agreement, findings recorded |

## 2. Test programs after M1

| Program | Assertions | Notes |
|---|---:|---|
| `test_graph` | 26 | unchanged |
| `test_model` | 43 | unchanged |
| `test_flp` | 59 | unchanged |
| `test_timing` | 291 | unchanged (evaluator untouched) |
| `test_tsp` | 29 | unchanged |
| `test_init` | 171 | unchanged; exercises the moved function through the forwarding name |
| `test_search_infra` (new) | 144 | RNG vectors, config, instance generator, FLP arc index, drone-id assignment (hand cases + pinned 20,000-case checksum), route operations, candidate window |
| `test_fuzz_search_infra` (new) | 11,262 | 30 seeded instances x 30 edits (794 effective edited cases): helpers vs evaluator |
| **Total** | **12,025** | previously 619 |

## 3. What the M1 tests establish

* **RNG:** SplitMix64 seed-0 outputs equal the published reference; xoshiro256\*\* and `derive_seed` values, `uniform_int` sequences and a shuffle equal an independent pure-Python
  implementation; statistical sanity (60,000 draws, six buckets within 5%); determinism and reseeding.
* **Instance generator:** bit-identical regeneration; four coordinates pinned to the Python reference for two datasets; prefix property when $c$ grows; depot placements;
  invalid configuration returns `ok == false` without throwing.
* **FLP arc index:** hand-computed pool ($|L_{01}|=2$, $|L_{02}|=5$, $|L_{12}|=5$, $|L_{10}|=2$, ids 3..16); per-arc counts equal $\lfloor D/\delta\rfloor$ for every ordered pair on a real
  generated instance; ratio ordering; independence from insertion order.
* **Drone-id assignment (moved function):** hand cases (disjoint, overlapping, $\lvert U\rvert=1$ and $0$, vector order vs launch order, unlocatable launch, recovery before launch,
  sequential reuse, old name forwards) **and** a regression pin: the FNV checksum of 20,000 random cases was computed with the *pre-move* implementation at commit `cc37e49`
  (value `0xd32e63262ac4e064`, 7,722 infeasible outcomes) and is asserted against the moved code. A separate 40,000-case differential run (old tree vs new tree, scratch harness) produced
  byte-identical output (md5 `834e09aae0046200135a7a0a2f75dc18`).
* **Route operations and candidate window:** hand-computed skeleton, gaps, foreign-FLP reports (including removal of a customer, foreign owner, missing ownership data, trailing FLP),
  trip lookups, service map, explicit clean-up, lowest free drone, half-step coordinates and windows.
* **Fuzz (helpers vs the existing evaluator):** (i) exactly-once service preserved by structure-preserving edits and visibly broken by a dropped trip; (ii) the foreign-FLP report has the
  same cardinality as the evaluator's arc-ownership violations on every edited route (246 non-empty cases); (iii) unused-FLP count equals the evaluator's rule-(6) count whenever all
  trips are placed, and is never larger otherwise (89 unplaced cases); (iv) `drop_unused_flps` removes exactly the offenders and keeps every used FLP; (v) a successful assignment never
  leaves a (28)/$\lvert U\rvert$ violation, and re-seating any single trip at its lowest free drone keeps it so (1,271 checks, 42 with drone id > 0); (vi) an evaluator-feasible edited
  solution has no foreign/unused FLPs and serves every customer once; (vii) the whole generator + builder pipeline is deterministic. Non-vacuity thresholds are asserted.

## 4. Mutation evidence (do the tests detect bugs?)

Bugs injected into a scratch copy of the headers, then the tests run:

| Injected bug | `test_search_infra` fails | `test_fuzz_search_infra` fails |
|---|---:|---:|
| drone assignment treats touching intervals as free (`<=`) | 2 | -- |
| foreign-FLP check ignores `arc_to` | 3 | 76 |
| adjacency window not doubled | 2 | -- |
| RNG rotation constant 45 $\to$ 44 | 6 | -- |
| instance x/y draw order swapped | 6 | -- |
| arc index ordered by descending ratio | 4 | -- |
| every FLP deemed unused | -- | 698 |
| `lowest_free_drone_for_trip` ignores overlaps | 4 | 43 (after strengthening, see below) |
| drone assignment ignores overlaps | -- | 225 |

`--` = not run against that program.

A first version of the fuzz harness did **not** detect the `lowest_free_drone_for_trip` mutation: the builder's initial solutions never use more than one drone, so the
overlap branch was never reached. The harness was strengthened (an edit kind that stretches a trip's recovery to create overlapping flights; every trip checked instead of one;
explicit thresholds on how many drone-id > 0 cases occur) and the mutation is now caught. The unit test had caught it from the start.

## 5. How to run

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-Wall -Wextra -Wpedantic" && cmake --build build -j && (cd build && ctest --output-on-failure)
# sanitizers
cmake -S . -B build-san -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="-O1 -g -Wall -Wextra -Wpedantic -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined"
cmake --build build-san -j && (cd build-san && ctest --output-on-failure)
```

## 6. M2: N4 and N5 (9 Nov 2026)

### 6.1 Test programs

| Program | Assertions | Notes |
|---|---:|---|
| `test_n4_n5` (new) | 130 | hand-derived values on line fixtures (see below) |
| `test_n4_n5_oracle` (new) | 713,256 | independent brute-force enumeration, judged by the frozen reference evaluator |
| (earlier programs) | 38,770 | graph 26, model 43, flp 59, timing 291, tsp 29, init 171, search_infra 144, fuzz 11,262, differential 26,745 |
| **Total** | **752,156** | |

`test_n4_n5` fixtures: depot at $x=0$, customer $C_i$ at $x=1000i$ m, truck 10 m/s, drone 20 m/s, $SL=SR=0$, service 0, $	heta=0$, $d=600$ m, all-ordered-pair FLP pool (ids from 100); all times are integers, so every expected value was derived on paper and the objectives compare exactly.
Covered: the new helpers (`flp_insertion_position`, `gaps_within_window`, even and odd half-steps); N4 candidate sets, order, objectives and best for windows 0--3; both depot boundaries; two-customer and one-customer routes; modes; affected vs unaffected trips; anchor = a trip's recovery node; invalid anchors; infeasible input; `evaluate_candidates=false`;
N5 candidate counts and objectives per partner (windows 1 and 2), the best candidate's route/trip/dropped FLP, recovery at the next truck node, exactly-once, no foreign FLPs, depot recovery, endurance rejection with a hand-derived pass/fail pattern, $|U|=1$ `NoFreeDrone` vs $|U|=2$ second drone, reuse of an on-route launch FLP, existing trips keeping their ids,
short routes, exact ties (first generated wins) for both operators, `classify_evaluation` on real evaluator results (feasible / (28) / fleet capacity / trip order), the random overloads (determinism for 24 and 10 seeds, spread of anchors, `NoEligibleAnchor`), and the non-mutation of every input.

`test_n4_n5_oracle`: 60 seeded instances (6--11 customers, 1--3 drones, every third with endurance $	imes 0.7$), each base solution followed by a seeded random walk of 3 steps through either operator = 240 bases (208 with trips). For windows 1 and 3, every N4 anchor and every N5 trip is compared **record by record** with a brute-force enumeration written with its own route arithmetic:
same candidate sequence and routes, same move descriptors, same affected-trip lists, same feasibility and objective as the frozen reference evaluator (relative 1e-6), the lowest feasible drone id, `best` = first minimum, exactly-once coverage, no foreign FLP in an accepted candidate. Non-vacuity is asserted:
3,144 N4 calls (12,277 candidates: 4,651 accepted, 7,626 affected), 880 N5 calls (85,584 candidates: 1,412 accepted, 795 affected-swaps, 2,416 `NoFreeDrone`, 80,961 endurance, 821 on-route FLP reuses, 6,122 depot recoveries, 9 second-drone). `EvalFlpArc` and `EvalFlpUnused` never occur (they would indicate a construction bug).
N4 evaluator rejections (other than affected trips) do not occur in the random campaign because trips built by D4 recover at the next truck node; the one reachable case (recovery moved ahead of the launch) is covered by the hand fixture. Determinism: the whole campaign is run twice and a digest of every result must match.

### 6.2 Results

| Check | Result |
|---|---|
| Release build (`-O3 -DNDEBUG -Wall -Wextra -Wpedantic`), clean recompile of the new targets | 0 warnings |
| CTest, Release | 11 of 11 pass |
| ASan + UBSan (`-O1 -fsanitize=address,undefined -fno-sanitize-recover=undefined`) | 11 of 11 pass, 0 reports, 0 warnings |
| Repeated runs (5 each) | `test_n4_n5` and `test_n4_n5_oracle` stdout byte-identical |
| Mutation (`tests/mutation/m2_mutate.py`, 35 mutants) | 35 of 35 detected |
| M1/P1/P2 regression | `bench_evaluator` rerun: n = 50 populated 9.9--11.5 us (sign-off 10.5 us), speed-up 30--33 x, all variants equivalent; `timing_engine.hpp` and `spatial_network.hpp` unchanged by M2 |

### 6.3 Mutation evidence

35 injected bugs, one at a time into a copy of the sources: window $\pm1$ in N4 relocate, N4 exchange and N5 (two windows, centre shifted); incident gaps not excluded; wrong gap mapping; no duplicate removal; affected check skipped (N4 and N5); affected-trip list emptied; trips silently dissolved; trips recovering at $t$ ignored; foreign-FLP trips ignored; old trip not dissolved; unused FLP not dropped; role swap wrong;
wrong recovery node; free-drone search bypassed; on-route FLP reuse disabled; FLP inserted against ratio order or ignoring existing FLPs; landing before the gap's FLPs; partner order reversed; counters dropped; `evaluate_candidates` ignored; tie-break `<=` (both operators); window predicate `<` and `+1` half-step; classifier branches removed.
Run 1 missed five: tie-break `<=` (N4, N5; no tie fixture), window `<` vs `<=` (the operators only pass even half-steps, where they coincide), the fleet-capacity branch of the classifier (operators pre-empt it with `NoFreeDrone`), and one equivalent mutant (replaced). New tests were added for each; the final run detects all 35 (`m2_mutation_results_raw.txt`).
Some mutants are detected only by the hand tests (counters, `evaluate_candidates`, trips recovering at $t$) and one only by the oracle (on-route FLP reuse), so both programs are needed.

### 6.4 Operator benchmark (`bench/bench_operators.cpp`, `-DTDR_BUILD_BENCH=ON`)

Method: instance and initial solution from fixed seeds (base 31415, dataset 0); a sweep calls the operator once per anchor (every truck customer / every trip), window 3, `record_candidates=false`; modes `gen` (`evaluate_candidates=false`) and `full` run round-robin, 15 repetitions each with batches of at least 20 ms, median, min, max and relative spread reported.
Generation cost is reported net of the one base evaluation each call makes; the marginal evaluation cost is (full - gen) per evaluated candidate. g++ 11, 2 cores, shared VM (spread 12--28 % is machine noise, not variance of the code).

| n | trips | N4 candidates / evaluated | N4 gen per candidate | N4 sweep full | N5 candidates / evaluated | N5 gen per candidate | N5 eval per evaluated | N5 sweep full | evaluator alone |
|---:|---:|---|---:|---:|---|---:|---:|---:|---:|
| 10 | 3 | 45 / 6 | 0.50 us | 46 us | 610 / 607 | 0.14 us | 1.92 us | 1.25 ms | 1.66 us |
| 20 | 6 | 110 / 9 | 0.59 us | 153 us | 793 / 781 | 0.20 us | 3.61 us | 3.00 ms | 3.08 us |
| 50 | 23 | 246 / 2 | 1.72 us | 774 us | 494 / 376 | 1.49 us | 13.51 us | 6.10 ms | 12.30 us |

Candidate generation runs at 0.6--7 million candidates/s; evaluation dominates N5 (83 % at n = 50). N4 at n = 50 evaluates only 2 of 246 candidates because 244 are rejected as affected trips (`operators_N4_N5.md` §6).
Comparison with the pre-follow-up evaluator (316--372 us per call at n = 50) shows that without P1 a single N5 sweep at n = 50 (376 evaluations) would have cost about 0.12 s instead of 5 ms.

### 6.5 How to run

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DTDR_BUILD_BENCH=ON && cmake --build build -j && (cd build && ctest)
./build/bench_operators [10|20|50]
TDR_REPO=$PWD TDR_LIB=build/libtdr_core.a python3 tests/mutation/m2_mutate.py        # all mutants; give name prefixes (N5-, CW-, ...) to select
```

## 7. M2 follow-up: N4 conservative re-anchoring (9 Nov 2026)

> **Superseded in part by §8 (M2 finalization):** the variant cap (10,000, `TooManyReanchorings`), the counts, mutant totals and benchmark numbers in this section describe the state of the M2 follow-up. The current semantics are budget-bounded with default 100.

### 7.1 Test programs (changes only; section 6 describes the unchanged remainder)

| Program | Assertions | Notes |
|---|---:|---|
| `test_n4_n5` | 203 (was 130) | 73 new; the 7 N4 hand tests written for the Reject policy now set `AffectedTripPolicy::Reject` explicitly, **no assertion was weakened or removed** |
| `test_n4_n5_oracle` | 905,628 (was 713,256) | existing N4 and N5 oracle unchanged (N4 pinned to Reject); new independent `check_n4_reanchor` |
| (earlier programs) | 38,770 | unchanged |
| **Total** | **944,601** | 11 of 11 CTest programs |

New hand-derived tests (line fixtures with $d=520$ m or $1500$ m so that arcs have known FLP counts; all expected objectives derived on paper, then compared with the frozen reference evaluator), mapped to the instruction's list:
(1) one stale trip re-anchored (gap relocation, two exchanges; the re-anchor record, route, trips); (2) two stale trips re-anchored jointly in trip order (9 = 3 x 3 variants, shared FLP node), determinism across calls, the variant cap (8 rejects, 9 and 0 accept); (3) an unaffected trip byte-for-byte unchanged (trip B and its FLP while trip A is re-anchored; the no-stale candidate carries the trips verbatim);
(4)(5) drone id, customer and recovery preserved in every record of every fixture; (6) no replacement FLP ($d=1500$: arcs of 1000 m have none), never evaluated, trip left intact; (7) replacement exists but every variant violates endurance (hand argument: launch $\le 450$ s, recovery at $1000$ s, $E=300$ s) with the reference evaluator rejecting all 7, while another candidate with a later launch is accepted;
(8) replacement exists but $|U|=1$ makes the in-flight intervals overlap (reference confirms), while the same relocation with $|U|=2$ is accepted; (9)(10) recovery node moved before its launch: `InvalidRecovery`, not evaluated, shown unrepaired (route and trips as the move produced them), reference also infeasible; (11) 7 replacement FLPs against the closed form $2400+f/10+|3000-f|/20$ and the unique minimum chosen;
(12)(17) $v_d=v_t$: four replacement FLPs tie exactly at 2700, the first (lowest ratio) is kept, strict `<`; (13) exactly-once coverage in every record including rejected ones; (14) input unchanged; (15) Reject policy and the original N4 tests unchanged, default policy identical on stale-free fixtures; (16) N5 deterministic and unaffected by the N4 options; diagnostics (counts, no double counting, `evaluate_candidates=false`).

Independent re-anchor oracle (`check_n4_reanchor`, written from the contract with its own arithmetic: stale detection, recovery position, replacement options from the FLP data in ratio order, joint odometer enumeration, variant route building, judgement by the frozen reference evaluator, first-minimum selection, per-reason tallies): record by record against the production operator on the same 240 bases x windows 1 and 3.
Campaign (N4 Reanchor): 12,277 candidates --- 4,904 stale, 1,159 re-anchored (1,211 trips, 959 candidates with several stale trips), 5,774 accepted, 2,728 invalid-recovery, 4 no-replacement, 17 too-many, 3,754 evaluator rejections (Reject on the same bases: 4,615 accepted, 7,632 affected, 30 evaluator rejections). Exact variant ties never occur in random instances (covered by the hand fixture). Note: the oracle's random-walk step uses the default N4 policy, which is now Reanchor, so the 240 bases differ from section 6.1 (hence 4,615 instead of 4,651 accepted Reject candidates); the independent N4 Reject checks, the N5 checks and the non-vacuity thresholds are unchanged.

### 7.2 Results

| Check | Result |
|---|---|
| Release build from scratch (`-O3 -DNDEBUG -Wall -Wextra -Wpedantic`, `TDR_BUILD_BENCH=ON`) | 0 warnings |
| CTest, Release | 11 of 11 pass |
| ASan + UBSan (`-O1 -fsanitize=address,undefined`) | 11 of 11 pass, 0 reports |
| Repeated runs (3 x `test_n4_n5`, 2 x oracle) | stdout byte-identical |
| Mutation (`tests/mutation/m2_mutate.py`, 56 mutants) | 56 of 56 detected (35 M2 + 21 re-anchor); raw: `m2_followup_mutation_results_raw.txt` |
| Frozen files | sha256 of `neighbourhood_n5.hpp`, `timing_engine.hpp`, `spatial_network.hpp`, `tests/reference/timing_engine_reference.hpp` identical to before the follow-up |
| Evaluator regression (`bench_evaluator`) | n = 50 populated 10.6--10.8 us, 31--34 x the reference, all variants equivalent (`evaluator_bench_after_m2_followup_raw.txt`) |

### 7.3 Re-anchor mutants (21)

Recovery check skipped / searched from route start; no-replacement check skipped / reported as `AffectedTrips`; drone id or recovery changed by re-anchoring; tie `<=` among variants; stale trips in reverse order; enumeration skipping the last option; options in reverse ratio order; stale FLP not removed; replacement put into the first gap; cap off by one; invalid-recovery counted twice; unaffected trips treated as stale; trips_reanchored counting candidates; evaluator judging the stale candidate instead of the variant; stale candidates not counted; launch not updated; accepted not counted; `evaluate_candidates` ignored.
One first-version mutant survived (dropping only the `arc_to` comparison of the gap match) and is **equivalent** (a skeleton node is the left end of exactly one gap); it was replaced by a non-equivalent one. Mutant N4-09 (trips cleared) exposed an infinite loop in the enumeration when a foreign FLP has no launching trip, so a defensive guard was added (unreachable through the public entry points, because inputs must be evaluator-feasible). Three old mutant patterns were updated to the new source text; all 35 old mutants are still detected.

### 7.4 Benchmark (same method as 6.4; both policies, `operator_bench_raw.txt`)

| n | N4 Reject sweep | N4 Reanchor sweep | N4 gen / candidate (Reject -> Reanchor) | N5 gen / candidate | N5 eval / evaluated | N5 sweep | evaluator |
|---:|---:|---:|---|---:|---:|---:|---:|
| 10 | 47 us | 5.3 ms | 0.53 -> 19.97 us | 0.14 us (was 0.14) | 2.03 us (1.92) | 1.33 ms | 1.62 us |
| 20 | 140 us | 13.2 ms | 0.64 -> 15.38 us | 0.19 us (0.20) | 3.50 us (3.61) | 2.90 ms | 3.00 us |
| 50 | 776 us | 362 ms | 1.78 -> 129.2 us | 1.45 us (1.49) | 13.82 us (13.51) | 6.19 ms | 11.9 us |

N5 and evaluator numbers agree with section 6.4 within the machine noise (12--31 % spread). The N4 cost is the evaluator cost times the number of variants (n = 50: 23,508 evaluator calls per sweep); the cap sensitivity is in `n4_reanchor_cap_sensitivity_raw.txt`.

## 8. M2 finalization --- bounded N4 re-anchoring (9 Nov 2026)

### 8.1 Counts

| Program | Assertions / checks | Change |
|---|---:|---|
| `test_n4_n5` | 233 (was 203) | old cap tests replaced by budget semantics; new section 3c; nothing weakened |
| `test_n4_n5_oracle` | 1,117,041 (was 905,628) | existing oracle kept; `check_n4_reanchor` updated to the budget semantics and run at the default budget **and** budget 4; new `check_cap_on_bench_fixtures` |
| (earlier programs) | 38,770 | unchanged |
| **Total** | **1,156,044** | 11 of 11 CTest programs |

### 8.2 The 14 required tests

(1) default budget is 100 (`N4Options{}` and a default-versus-explicit-100 identity); (2) a configured budget overrides it (budgets 1, 3, 4, 5, 7, 8, 9 give the hand-derived results); (3) budget 1; (4) budget 100; (5) budget larger than the number of combinations (10,000 and 0 identical to 100 on the ladder fixture; a budget equal to the total is *not* `cap_reached`); (6) budget reached before exhaustive enumeration (`cap_reached`, `combinations_total`, `variants_generated == budget`, `cap_reached_candidates`, with capped candidates still judged once: evaluator-rejected ones report their reason, e.g. `eval-endurance`); (7) deterministic enumeration (repeated calls identical; an enumeration-order fixture whose acceptance flips between budgets 3 and 4 fixes the odometer order, last stale trip fastest); (8) budget 100 versus 10,000 on the n = 10 / 20 / 50 benchmark fixtures: best identical for 7/7, 14/14 and 27/27 anchors, candidates identical 45/45, 110/110, 246/246, budget reached in 2, 8 and 35 candidates --- **asserted as an observation on these fixtures only**, not as a general equivalence; (9) first-generated tie-break stable (four-way exact tie at 2700, first FLP kept for budgets 1, 2, 4, 5, 7, 100; `cap_reached` iff budget < 7); (10) recovery nodes unchanged; (11) drone ids and customers unchanged (both checked in every record of every fixture); (12) Reject policy unchanged (identical for budget 1 and 100, 3 `AffectedTrips`, no variants, budget never reached; the seven Reject-era N4 tests pin `AffectedTripPolicy::Reject`); (13) N5 unchanged (deterministic, unaffected by the N4 options, `neighbourhood_n5.hpp` byte-identical); (14) input solution unchanged. Also: exactly-once outcome (`accepted + rejected == generated`), exactly-once customer coverage.
The independent oracle remains independent of the production re-anchoring helpers (its own stale detection, recovery position, option lists in ratio order, odometer, variant routes, judgement by the frozen `tdr::timing_ref` evaluator) and checks `variants_generated == min(K, budget)`, `cap_reached == (K > budget)`, `combinations_total == K` and `cap_reached_candidates`.

### 8.3 Results

| Check | Result |
|---|---|
| Release build from scratch (`-O3 -DNDEBUG -Wall -Wextra -Wpedantic`, `TDR_BUILD_BENCH=ON`, fresh directory) | 0 warnings |
| CTest, Release | 11 of 11 pass (7.8 s) |
| ASan + UBSan (`-O1 -fsanitize=address,undefined`) | 11 of 11 pass, 0 reports |
| Repeated runs (3 x `test_n4_n5`, 2 x oracle) | stdout byte-identical (md5 equal) |
| Mutation, 64 mutants (35 M2 + 21 re-anchor + 8 cap) | **64 of 64 detected**; raw: `m2_followup_mutation_results_raw.txt` |
| Frozen files | sha256 of `neighbourhood_n5.hpp`, `timing_engine.hpp`, `spatial_network.hpp`, `tests/reference/timing_engine_reference.hpp` identical before and after |
| Git | HEAD `cc37e49`; `git status` identical before and after; nothing committed or pushed |

Cap mutants (8): default budget back to 10,000; budget off by one; cap ignored; `cap_reached_candidates` not counted; budget 0 treated as zero instead of unlimited; first stale trip fastest; `combinations_total` not recorded; budget applied to the Reject path. Re-anchor mutant RA-09 became "`cap_reached` off by one (`>=`)". **CAP-06 first survived** (no fixture distinguished the odometer direction under a budget); the enumeration-order fixture was added and it is now killed.

### 8.4 Benchmarks (same methodology and fixtures as 6.4; single runs, machine noise 10--50 % spread)

| n | N4 Reject sweep | N4 Reanchor, budget 100 | N4 Reanchor, budget 10,000 | N5 gen / candidate (follow-up) | N5 sweep (follow-up) | evaluator alone (follow-up) |
|---:|---:|---:|---:|---:|---:|---:|
| 10 | 40.3 us | 2.47 ms | 4.76 ms | 0.12 us (0.14) | 1.16 ms (1.33) | 1.52 us (1.62) |
| 20 | 128.4 us | 7.92 ms | 12.19 ms | 0.17 us (0.19) | 2.69 ms (2.90) | 2.73 us (3.00) |
| 50 | 637.5 us | 67.8 ms | 302.1 ms | 1.17 us (1.45) | 5.16 ms (6.19) | 10.42 us (11.9) |

Combinations evaluated per stale candidate (budget 100 / 10,000): n = 10: 26.9 / 51.3; n = 20: 21.1 / 32.4; n = 50: 21.7 / 96.3. Candidates reaching the budget (budget 100): 2 (5.1 % of stale), 8 (7.9 %), 35 (14.3 %); budget 10,000: 0. Feasible N4 candidates: Reject 13.3 / 8.2 / 0.8 %; Reanchor 17.8 / 24.5 / 2.0 % (identical at both budgets); re-anchored 4.4 / 16.4 / 1.2 %. N5 and the evaluator show no regression (differences are within noise; N5 code unchanged). `bench_evaluator`, n = 50 populated: 7.75--8.03 us, 30.8--32.5 x the frozen reference, all variants equivalent (P1/P2 gains intact).
**The budget trades exhaustive search for bounded runtime.** The saving of budget 100 over 10,000 is about 1.9 x / 1.5 x / 4.5 x at n = 10 / 20 / 50, smaller than the earlier "about 12 x", which was measured when over-cap candidates were rejected unevaluated. Cap sensitivity (feasible at n = 20 / 50): budget 1: 11 / 2; 10: 21 / 2; 30: 27 / 4; 100 and above: 27 / 5.

## 9. M3 --- neighbourhoods N8 and N7 (9 Nov 2026, instruction 08:45 IST)

Operator contracts, assumption A7, measurements and findings: `operators_N7_N8.md`. Raw output: `n7_n8_bench_raw.txt`, `n7_n8_bench_endurance_x3_raw.txt`, `n7_n8_oracle_stats_raw.txt`, `m3_mutation_results_raw.txt`, `m2_mutation_results_after_m3_raw.txt`, `operator_bench_after_m3_raw.txt`, `evaluator_bench_after_m3_raw.txt`.

### 9.1 Counts (Release, fresh build directory)

| Program | Assertions / checks | Change |
|---|---:|---|
| `test_n7_n8` (new) | 253 | hand-derived fixtures N7-1..N7-10, N8-1..N8-5 |
| `test_n7_n8_oracle` (new) | 1,696,098 | independent brute force + frozen reference evaluator, 60 seeds, 360 bases |
| `test_n4_n5` | 233 | unchanged |
| `test_n4_n5_oracle` | 1,117,041 | unchanged |
| `test_evaluator_differential` | 26,745 | unchanged |
| `test_fuzz_search_infra` | 11,262 | unchanged |
| `test_timing`, `test_init`, `test_search_infra`, `test_flp`, `test_model`, `test_tsp`, `test_graph` | 291, 171, 144, 59, 43, 29, 26 | unchanged |
| **Total** | **2,852,395** (was 1,156,044) | 13 of 13 CTest programs; 0 failures |

The earlier programs' assertion counts are identical to the M2 final ones (38,770 in the nine oldest programs; 233; 1,117,041): no existing test was changed or weakened.

### 9.2 Results

| Check | Result |
|---|---|
| Release build from scratch (`-O3 -DNDEBUG -Wall -Wextra -Wpedantic`, `TDR_BUILD_BENCH=ON`, fresh directory) | 0 warnings in the fresh build (two warnings seen during development, an unused function in the new oracle and a `-Wmismatched-new-delete` in the new bench's allocation counter, were fixed) |
| CTest, Release | 13 of 13 pass (7.8 s) |
| ASan + UBSan (`-O1 -g -fsanitize=address,undefined`, fresh directory) | 13 of 13 pass, 0 reports |
| Repeated runs (2 x every deterministic program incl. both oracles) | stdout byte-identical (md5 equal) |
| Mutation, M3 (53 mutants) | 52 detected, 1 equivalent survivor (CW-02, see `operators_N7_N8.md` section 7); 2 mutants first survived (N7-10, N8-20), the tests were strengthened, both now detected |
| Mutation, M2 suite re-run on the unchanged M2 tests | **64 of 64 detected** |
| Frozen files | sha256 of `neighbourhood_n4.hpp`, `neighbourhood_n5.hpp`, `route_ops.hpp`, `candidate_window.hpp`, `drone_assignment.hpp`, `flp_arc_index.hpp`, `timing_engine.hpp`, `spatial_network.hpp`, `tests/reference/timing_engine_reference.hpp` identical before and after; `move_types.hpp` changed additively |
| Git | HEAD `cc37e49`; branch unchanged; nothing committed or pushed |

### 9.3 What the new tests establish

Exactly-once coverage, FLPs only on their arc, no foreign or idle FLP reaching the evaluator, trips untouched (identity, customer, launch, recovery, drone id), input immutability, deterministic order, strict `<` with first-generated tie-break, the counting identity `accepted + rejected = generated`, rejection reasons (affected-trips / no-replacement-flp / no-free-drone / eval-endurance), the lowest-free-drone rule, FLP reuse, depot boundaries, window boundaries, D5 (N8 positions outside the D1 window), `insertion_window`, and objective and verdict equality with the frozen reference on every evaluated record.

### 9.4 Mutants

N7 (28): window $\pm1$, boundary strict, three coordinate-mapping errors, $t$ not removed, wrong recovery, affected checks skipped (all, recovery-at-$t$, foreign FLP), free-drone search bypassed, tie `<=`, `NoReplacementFlp` / `NoFreeDrone` not counted, FLP reuse disabled, anchor check skipped, `evaluate_candidates` ignored, reversed gap / FLP order, ratio-ignoring insertion, best move without drone id, no-replacement test ignoring the window, `Rng` picks the first, trips lost, wrong customer, affected list not recorded, reason overridden. N8 (21): default restricted to the window (violates D5), first / last gap skipped, affected check skipped, trip not dissolved, FLP not dropped, insertion before the gap's FLPs, tie `<=`, `insertion_window` $\pm1$ and shifted centre, local instead of base indices, unsorted list, `AffectedTrips` not counted, `evaluate_candidates` ignored, `flps_dropped` not recorded, index check skipped, `Rng` picks the first, double-counted rejection, best move without replaced trip, customer inserted twice. Helpers (4): FLP insertion order and position, window boundary.
Several are killed only by the oracle (N7-23, N8-07, N8-13) or only by the hand fixtures (N7-03, N7-04, N7-10, N7-13, N7-17, N7-18, N7-22, N7-24, N8-08, N8-15, N8-17, N8-18, N8-19): the two layers are complementary.

### 9.5 Benchmarks

See `operators_N7_N8.md` section 8 (N7/N8 independently at n = 10, 20, 50; N4/N5/evaluator regression: counts identical, times within machine variation, P1/P2 gain intact at 30.2--32.6 $\times$).

### 9.6 How to run

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -DNDEBUG -Wall -Wextra -Wpedantic" -DTDR_BUILD_BENCH=ON
cmake --build build && ctest --test-dir build
./build/bench_n7_n8 [endurance_multiplier [n]]
TDR_REPO=$PWD TDR_LIB=build/libtdr_core.a TDR_WORK=/tmp/m3mut python3 tests/mutation/m3_mutate.py [name-prefix ...]
```

## 10. M4 --- neighbourhoods N3 and N6 (9 Nov 2026, instruction 09:42 IST)

Operator contracts, assumptions A8--A12, measurements and findings: `operators_N3_N6.md`. Raw output: `n3_n6_unit_tests_raw.txt`, `n3_n6_oracle_stats_raw.txt`, `n3_n6_bench_raw.txt`, `n3_n6_bench_alt_drones_raw.txt`, `n3_n6_bench_endurance_x3_raw.txt`, `n3_n6_bench_endurance_x3_alt_drones_raw.txt`, `m4_mutation_results_raw.txt`, `m1_m3_mutation_spotcheck_after_m4_raw.txt`, `operator_bench_after_m4_raw.txt`, `n7_n8_bench_after_m4_raw.txt`, `evaluator_bench_after_m4_raw.txt`.

### 10.1 Counts (Release, fresh build directory)

| Program | Assertions / checks | Change |
|---|---:|---|
| `test_n3_n6` (new) | 565 (**587 after the A8 revision**) | hand-derived fixtures N3-1..N3-8 (incl. N3-4b), N6-1..N6-7 (incl. N6-2b); after A8: N6-1 restricted, new N6-1b (default), N6-4/N6-6 on both option sets |
| `test_n3_n6_oracle` (new) | 595,093 (**613,135 after the A8 revision**) | independent brute force + frozen reference evaluator, 80 seeds, 640 bases; after A8 the default options are also checked equal to `include_same_drone = true` |
| `test_n7_n8` | 253 | unchanged |
| `test_n7_n8_oracle` | 1,696,098 | unchanged |
| `test_n4_n5` | 233 | unchanged |
| `test_n4_n5_oracle` | 1,117,041 | unchanged |
| `test_evaluator_differential` | 26,745 | unchanged |
| `test_fuzz_search_infra` | 11,262 | unchanged |
| `test_timing`, `test_init`, `test_search_infra`, `test_flp`, `test_model`, `test_tsp`, `test_graph` | 291, 171, 144, 59, 43, 29, 26 | unchanged |
| **Total** | **3,448,053** (was 2,852,395); **3,466,117 after the A8 revision** (M4 = 613,722) | 15 of 15 CTest programs; 0 failures; M4 adds 595,658 (613,722 after A8) |

Every earlier program's count is identical to the M3 run: no existing test was changed or weakened.

**A8 revision (9 Nov 2026, 12:30 IST).** N6's default now admits any other trip (`include_same_drone = true`). Updated only: `test_n3_n6` (N6-1 and the checks that pinned the old default now name the restricted option; new N6-1b; N6-4 and N6-6 test both option sets), `test_n3_n6_oracle` (adds a default-equals-`true` check), `m4_mutate.py` (N6-13 flips `true` to `false`: killed; 13 of 14 N6 mutants killed, N6-07 equivalent), `bench_n3_n6` (the labelled variant is now the *restricted* one: default N6 2 / 8 / 108 candidates, 0 / 0 / 4 feasible at $n=10/20/50$; restricted 0). Raw files regenerated: `n3_n6_unit_tests_raw.txt`, `n3_n6_oracle_stats_raw.txt`, `n3_n6_bench*_raw.txt`, `m4_mutation_results_raw.txt` (A8 re-run appended). The figures in the tables of this section that say 565 / 595,093 are the original M4 figures.

### 10.2 Results

| Check | Result |
|---|---|
| Release build from scratch (`-O3 -DNDEBUG -Wall -Wextra -Wpedantic`, `TDR_BUILD_BENCH=ON`, fresh directory) | **0 warnings** |
| CTest, Release | 15 of 15 pass (16.3 s) |
| ASan + UBSan (`-O1 -g -fsanitize=address,undefined`, fresh directory) | 0 warnings; 15 of 15 pass (94.7 s); 0 reports |
| Repeated runs (3 x `test_n3_n6` and `test_n3_n6_oracle`) | stdout byte-identical (md5 equal) |
| Oracle | 595,093 checks, 0 failures; N3 7,036 and N6 5,806 candidates judged by the frozen reference; 0 non-endurance evaluator reasons; no random candidate tie (ties are covered by the hand fixtures N3-4/N3-4b/N6-2/N6-2b) |
| Mutation, M4 (58 mutants, `tests/mutation/m4_mutate.py`) | **54 detected**, 4 survivors, all analysed: N3-12 and N6-07 equivalent (constraint (28); a defensive guard that cannot fire); CW-04 and CW-06 do not affect N3/N6 (truck-drone distances only) and are killed by the M2/M3 suites (`test_n4_n5`, `test_n4_n5_oracle`, `test_n7_n8`, `test_n7_n8_oracle`). One mutant (TE-24, tie rule `<=` only for later trip indices) first survived; fixtures N3-4b and N6-2b were added and it is now detected |
| Mutation, older suites | spot check of **33** M2/M3 mutants on the unchanged M2/M3 tests: 33 of 33 detected. The **full** M2 (64) and M3 (53) mutation runs were *not* repeated: their sources and tests are byte-identical to the signed-off runs |
| Frozen files | sha256 of `neighbourhood_n4.hpp`, `neighbourhood_n5.hpp`, `neighbourhood_n7.hpp`, `neighbourhood_n8.hpp`, `route_ops.hpp`, `candidate_window.hpp`, `drone_assignment.hpp`, `flp_arc_index.hpp`, `timing_engine.hpp`, `spatial_network.hpp`, `tests/reference/timing_engine_reference.hpp` identical before and after; `move_types.hpp` changed additively |
| Git | HEAD `cc37e49`; branch unchanged; nothing committed or pushed (new untracked: `tests/test_n3_n6.cpp`, `tests/test_n3_n6_oracle.cpp`; the other new files lie in directories that were already untracked) |

### 10.3 What the new tests establish

Exactly-once coverage; only the two trips' customers change (launch FLP, recovery node, drone id and trip order untouched); every FLP on its arc; production objective and verdict equal to the frozen reference for every candidate; one outcome per candidate (`accepted + rejected_total() == generated`); only evaluator reasons occur; input immutability; deterministic order; strict `<` with first-generated tie-break in both index orders; the inclusive endurance boundary ($e=(37)$); window boundaries (N6); same-drone consecutive pairs only (N3, including a pair with another drone's launch in between, a non-vector-adjacent pair, and a lone trip); different-drone partners only by default and `include_same_drone` (N6); truck customers inside the window never partners; depot recovery; error statuses; flag behaviour; the `Rng` overloads.

### 10.4 Mutants

N3 (13): predecessor / successor missing or swapped, sequence ordered by trip index / descending / by recovery position, same-drone filter dropped, successor = last or predecessor = first trip of the drone, window wrongly applied, `Rng` always trip 0 or without guard, operator id. N6 (14): same-drone not excluded / filter inverted / option ignored / default flipped, window $\pm1$, truck customers not skipped, self-pair, partner order (farthest first, by id only), `Rng`, operator id, move kind. Shared exchange (24): tie rule, customers / launch / recovery / drone ids exchanged wrongly, evaluator flag ignored, every counter, reason override, move descriptors, base objective, anchor, recorded solution, infeasible input and range checks, infeasible candidates accepted, best tie. Window arithmetic (7): strict window, $+1$, farthest first, half-step offsets, id tie-break. Several are killed only by the oracle (e.g. CW-07) or only by the hand fixtures (e.g. CW-02, CW-05, TE-01, TE-06, TE-15, TE-16, TE-20, TE-24): the two layers are complementary.

### 10.5 Benchmarks

See `operators_N3_N6.md` section 8 (N3 and N6 independently at n = 10, 20, 50; paper parameters plus labelled variants; N4/N5/N7/N8/evaluator regression: counts identical, times within machine variation, P1/P2 gain intact at 33.3--35.2 $\times$ at $n=50$ against 30.2--32.6 $\times$ in the M3 run).

### 10.6 How to run

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -DNDEBUG -Wall -Wextra -Wpedantic" -DTDR_BUILD_BENCH=ON
cmake --build build && ctest --test-dir build
./build/bench_n3_n6 [endurance_multiplier [n|all [alt]]]
TDR_REPO=$PWD TDR_LIB=build/libtdr_core.a TDR_WORK=/tmp/m4mut python3 tests/mutation/m4_mutate.py [name-prefix ...]
TDR_TESTS=test_n4_n5,test_n7_n8 ... python3 tests/mutation/m4_mutate.py CW-04   # run a mutant against other test programs
```

## 11. M5 --- neighbourhoods N1 and N2 (9 Nov 2026, instruction 12:30 IST)

Operator contracts, assumptions A13--A18, measurements and findings: `operators_N1_N2.md`. Raw output: `n1_n2_unit_tests_raw.txt`, `n1_n2_oracle_stats_raw.txt`, `n1_n2_bench_raw.txt`, `n1_n2_bench_alt_drones_raw.txt`, `n1_n2_bench_endurance_x3_raw.txt`, `n1_n2_bench_endurance_x3_alt_drones_raw.txt`, `m5_mutation_results_raw.txt`, `m5_release_ctest_raw.txt`, `m5_sanitizer_ctest_raw.txt`, and the regression runs `operator_bench_after_m5_raw.txt`, `n7_n8_bench_after_m5_raw.txt`, `n3_n6_bench_after_m5_raw.txt`, `evaluator_bench_after_m5_raw.txt`.

### 11.1 Counts (Release, fresh build directory)

| Program | Assertions / checks | Change |
|---|---:|---|
| `test_n1_n2` (new) | 419 | hand-derived fixtures N1-A..N1-E, N2-A..N2-D, errors and Rng overloads; every record re-checked independently (`check_records`) against the frozen reference |
| `test_n1_n2_oracle` (new) | 3,470,761 | independent brute force (no production helper) + frozen reference evaluator, 80 seeds, 640 bases, windows 0..3, every trip |
| `test_n3_n6` / `test_n3_n6_oracle` | 587 / 613,135 | unchanged since the A8 revision |
| `test_n7_n8` / `test_n7_n8_oracle` | 253 / 1,696,098 | unchanged |
| `test_n4_n5` / `test_n4_n5_oracle` | 233 / 1,117,041 | unchanged |
| `test_evaluator_differential` | 26,745 | unchanged |
| `test_fuzz_search_infra` | 11,262 | unchanged |
| `test_timing`, `test_init`, `test_search_infra`, `test_flp`, `test_model`, `test_tsp`, `test_graph` | 291, 171, 144, 59, 43, 29, 26 | unchanged |
| **Total** | **6,937,297** (was 3,466,117 after A8) | 17 of 17 CTest programs; 0 failures; M5 adds 3,471,180 |

No existing test was changed or weakened in M5.

### 11.2 Results

| Check | Result |
|---|---|
| Release build from scratch (`-O3 -DNDEBUG -Wall -Wextra -Wpedantic`, `TDR_BUILD_BENCH=ON`, fresh directory) | **0 warnings** (the new files also compile warning-free with `-Wshadow -Wconversion`) |
| CTest, Release | 17 of 17 pass (13.2 s) |
| ASan + UBSan (`-O1 -g -fsanitize=address,undefined -Wall -Wextra -Wpedantic`, fresh directory, `detect_leaks=1`, `halt_on_error=1`) | 0 warnings; 17 of 17 pass (87.4 s); 0 reports |
| Repeated runs (3 x `test_n1_n2` and `test_n1_n2_oracle`) | stdout byte-identical (md5 equal) |
| Oracle | 3,470,761 checks, 0 failures; N1 231,964 and N2 25,986 candidates judged by the frozen reference; `EvalFlpArc`, `EvalFlpUnused`, `AffectedTrips`, `NoFreeDrone`, `NoReplacementFlp` never occur; the whole campaign run twice with equal digests |
| Mutation, M5 (62 mutants, `tests/mutation/m5_mutate.py`) | **62 of 62 killed**, none survived, none invalid (N1 26, N2 17, shared step 17, `flp_insertion_position` order, `gaps_within_window` boundary). The two `Rng`-guard mutants are killed only by `test_n1_n2` |
| Mutation, older suites | not repeated: M5 changed no M1--M4 production code except the additive `move_types.hpp` fields; the M2/M3/M4 mutation runs use the sources of their own milestones |
| Frozen files | sha256 of `candidate_window.hpp`, `drone_assignment.hpp`, `neighbourhood_n4.hpp`, `neighbourhood_n5.hpp`, `neighbourhood_n7.hpp`, `neighbourhood_n8.hpp`, `route_ops.hpp`, `timing_engine.hpp`, `spatial_network.hpp`, `flp_arc_index.hpp`, `tests/reference/timing_engine_reference.hpp` identical to the M4 end; `move_types.hpp` changed (additive, intended); `neighbourhood_n3.hpp`, `neighbourhood_n6.hpp` and `trip_exchange.hpp` not touched by M5 |
| Git | HEAD `cc37e49`; branch unchanged; nothing committed or pushed (new untracked: `tests/test_n1_n2.cpp`, `tests/test_n1_n2_oracle.cpp`; the other new files lie in directories that were already untracked) |

### 11.3 What the new tests establish

Exactly one node of exactly one trip changes (the launch for N1, the recovery for N2; drone id, customer, the other node, every other trip, the skeleton (N1) or the whole route (N2) untouched); the route's FLPs are exactly the trips' launches and `flps_dropped` is the old launch iff no trip uses it any more; every FLP stays on its arc in ratio order; exactly-once coverage; `InvalidRecovery` exactly when the recovery is not after the launch (independent position test), such candidates never evaluated and never repaired; production objective and verdict equal to the frozen reference for every evaluated candidate; (28) / fleet-capacity classification and the inclusive endurance bounds (36)/(37); one outcome per candidate; no duplicates; best = first lowest feasible (strict `<`, exact ties go to the first generated); anchor = vector index; Rng overloads draw once; determinism; input immutability.

### 11.4 Mutants

N1 (26): window $\pm1$ / ignored / wrong half-step; current launch not excluded; FLP inserted at gap start / end / even if present / never; launch not changed; old launch always / never dropped; drop not reported; recovery repaired instead of rejected; `InvalidRecovery` off by one either way or never raised; gap order reversed; move kind, replaced node, drone id, operator id wrong; drone id changed; depot-arc gap skipped; Rng always trip 0 / no empty guard. N2 (17): window $\pm1$; start depot allowed; end depot excluded; current recovery not excluded; order descending; recovery not changed; drone id changed; `InvalidRecovery` off by one either way or never raised; move kind, replaced node, operator id wrong; route modified; Rng mutants. Shared step (17): tie-break `<=`; `evaluate_candidates` ignored; counters (generated / evaluated / accepted / rejected, both paths); invalid-recovery reason or trip wrong; evaluator reason overridden; infeasible accepted; best without the candidate solution; dropped FLPs or solution not recorded; launch gap or recovery located wrongly. Helpers (2): FLP ratio order reversed in `flp_insertion_position`; `gaps_within_window` strict.

### 11.5 Benchmarks

See `operators_N1_N2.md` section 8 (N1 and N2 independently at $n=10,20,50$; paper parameters plus labelled variants; N3/N4/N5/N6/N7/N8 candidate and outcome counts identical to the earlier runs; evaluator still $\times5$--$\times10$ faster than the frozen reference on this noisier VM).

### 11.6 How to run

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -DNDEBUG -Wall -Wextra -Wpedantic" -DTDR_BUILD_BENCH=ON
cmake --build build && ctest --test-dir build
./build/bench_n1_n2 [endurance_multiplier [n|all [alt]]]
TDR_REPO=$PWD TDR_LIB=build/libtdr_core.a TDR_WORK=/tmp/m5mut python3 tests/mutation/m5_mutate.py [name-prefix ...]
```


## 12. M6 --- shaking operators (9 Nov 2026, instruction 13:19 IST; signed off 13:56 IST)

Contracts, assumptions A19--A29, measurements: `shakes.md`. Raw output: `shakes_unit_tests_raw.txt`, `shakes_oracle_stats_raw.txt`, `shakes_bench_raw.txt`, `shakes_bench_endurance_x3_raw.txt`, `m6_mutation_results_raw.txt`, `m6_release_ctest_raw.txt`, `m6_sanitizer_raw.txt`, `*_after_m6_raw.txt`.

| Program | Assertions / checks | Change |
|---|---|---|
| `test_shakes` | 675 | new |
| `test_shakes_oracle` | 728,109 | new (independent re-implementation, exact comparison incl. RNG consumption) |
| all other programs | unchanged (e.g. `test_n1_n2` 419, `test_n1_n2_oracle` 3,470,761) | none |

Whole suite: 19 of 19 programs, about 7,666,000 checks (6,937,297 + 675 + 728,109); fresh Release build 0 warnings; ASan + UBSan 0 reports for the M6 programs; M6 mutation 70 of 71 killed (1 equivalent). Commands:

```
cmake --build build && ctest --test-dir build
./build/bench_shakes [endurance_multiplier [n|all]]
TDR_REPO=$PWD TDR_LIB=build/libtdr_core.a TDR_WORK=/tmp/m6mut python3 tests/mutation/m6_mutate.py [name-prefix ...]
```


## 13. M7 --- VNS driver (9 Nov 2026, instruction 13:56 IST; signed off 14:34 IST)

Contract, assumptions A30--A38, pilot and findings: `vns.md`. Raw output: `vns_unit_tests_raw.txt`, `vns_oracle_stats_raw.txt`, `vns_pilot_n{10,20,50}_raw.txt`, `vns_curve_n{10,20,50}_raw.txt`, `vns_default_runs_raw.txt`, `m7_mutation_results_raw.txt`, `m7_release_ctest_raw.txt`, `m7_sanitizer_raw.txt`, `*_after_m7_raw.txt`.

| Program | Assertions / checks | Change |
|---|---|---|
| `test_vns` | 85 | new |
| `test_vns_oracle` | 29,447 | new (independent loop, exact per-pass comparison) |
| all other programs | unchanged | none |

Whole suite: 21 of 21 programs, about 7,695,000 checks (7,666,081 + 85 + 29,447); fresh Release build 0 warnings; ASan + UBSan 0 reports for the M7 programs; M7 mutation 51 of 52 killed (1 equivalent). Commands:

```
cmake --build build && ctest --test-dir build
./build/bench_vns <n> <iter_max> <runs> [endurance_multiplier [t0|cool|all|curve|default]]
TDR_REPO=$PWD TDR_LIB=build/libtdr_core.a TDR_WORK=/tmp/m7mut python3 tests/mutation/m7_mutate.py [name-prefix ...]
```

### 13.1 M7 closure (14:34 IST)
`test_vns` 85 -> **108** assertions (guard reporting); `test_vns_oracle` 29,447 checks (pass outcomes, local steps and totals now compared); mutation **57 of 58** killed (V-44 adapted, V-53..V-58 new, V-10 equivalent); the complete 21-program suite under ASan + UBSan passes with 0 reports (`m7_closure_sanitizer_raw.txt`, `m7_closure_vns_unit_raw.txt`, `m7_closure_vns_oracle_raw.txt`). Whole suite: about 7,695,636 checks (7,695,613 + 23).


## 14. M8 --- runner, result writer, configuration, CLI (9 Nov 2026, instruction 14:34 IST)

Spec and measurements: `runner_cli.md`. Raw output: `m8_config_unit_raw.txt`, `m8_runner_cli_unit_raw.txt`, `m8_release_ctest_raw.txt`, `m8_build_raw.txt`, `m8_sanitizer_raw.txt`, `m8_mutation_results_raw.txt` (chronological log: ten mutants first survived, each followed by its re-run after the test was strengthened), `m8_cli_protocol_raw.txt`, `m8_bench_vns_default_n50_raw.txt`.

| Program | Assertions | Covers |
|---|---|---|
| `test_experiment_config` | 24 | defaults equal the algorithm structs' own (VNS, shake, N4, StaticConfig, KMeansPolicy, former TSP constants); every key accepts its own printed value; round trip with every value kind non-default (field-by-field through the table); sections, comments, line-numbered errors, duplicate / unknown / malformed values, rejected value leaves the field unchanged; validation classes; one wiring point into the VNS options; config files |
| `test_runner_cli` | 63 | runner = hand-built pipeline + direct `run_vns` (bit-identical trace / best / current / counters; frozen-reference re-check); F1/F2 values reach the builder (non-vacuous discriminating TSP seed and k-means settings); VNS, shake, N4, list, window, repeats keys honoured; run-index and dataset independence; byte-identical rendered outputs, no wall clock in deterministic files; independent JSON reader checks results.json / timing.json / manifest.json against the in-memory result (bit-exact objectives, solutions, summary incl. sample std); CSV shape; A38 guard visible end to end; JSON writer; FNV-1a known answers; writer files, manifest digests, overwrite protection, stale traces, unwritable path; verification negative tests; CLI exit codes, precedence, `--quiet`, reproduction from `config.resolved.cfg` byte for byte |

Whole suite at first delivery: **23 of 23** programs, about 7,695,723 checks (7,695,636 + 24 + 63; after the M8 closure 25 + 76, see section 16); fresh Release build (`-O3 -DNDEBUG -Wall -Wextra -Wpedantic`, bench ON) 0 warnings; ASan + UBSan 0 reports for both new programs and `tdr_static`; M8 mutation (`tests/mutation/m8_mutate.py`, 44 mutants over config / runner / writer / CLI) **44 of 44 killed**. Commands:

```
cmake -S . -B build -DTDR_BUILD_BENCH=ON && cmake --build build && ctest --test-dir build
./build/tdr_static run --out results/exp1 --config exp1.cfg --set vns.iter_max=2000
TDR_REPO=$PWD TDR_LIB=build/libtdr_core.a TDR_WORK=/tmp/m8mut python3 tests/mutation/m8_mutate.py [name-prefix ...]
```

### 14.1 M8 closure (15:35 IST)
`test_experiment_config` 25 (BOM + CRLF), `test_runner_cli` 76 (CLI exit code 1 with a complete failure record, runner preflight, paths with spaces, CRLF + BOM config, overwrite, schema key sets, node-table interpretation). Windows (cross-built, Wine): 25/25 and 76/76; Windows and Linux outputs byte-identical (`windows_validation.md`). Cross-built binaries copied to `Claude outputs\winbuild\` (SHA-256: `tdr_static.exe` 5faee7f382f2a35bd100a4f7d1fe190e1eb4f1aba7dd1d7b97451499faa7f7cc, `test_experiment_config.exe` 23a3a85d71fd5adcbbc3203075f15c9f51d4797c55f0bec65266fd7ceac111e3, `test_runner_cli.exe` ccea8f1ac630b71cbbf12bf1a19bba14848921f11b68bf83228fa607afa59b5e; a `_to_delete` subfolder holds compiler by-products, delete it freely).

## 15. M9 --- validation protocol (9 Nov 2026, instruction 15:35 IST; signed off 16:13 IST)

No source change; regression state: 24/24 CTest programs pass in the Release build tree (-O3 -Wall -Wextra -Wpedantic; the 24th is `m9_protocol_reproduces`; raw: `m9_release_ctest_raw.txt`), 15 guard hashes unchanged. `m9_protocol_reproduces` (`scripts/m9_check.cmake`): validates the 9 protocol configurations with `tdr_static check` and re-runs `config/m9/n10_seed1.cfg`, comparing `results.json`, `runs.csv`, `config.resolved.cfg` byte for byte with `results/m9/n10_seed1/` (negative tests: one altered byte in `runs.csv`, one invalid seed in a configuration -> failure). Independent checks: exact truck-only DP agrees with the evaluator objective to 2.9e-11 s on every drone-free best solution (`scripts/m9_exact_truck_only.py`); the feasibility probe reports zero construction-error rejection reasons (`eval-flp-arc`, `eval-flp-unused`). Results, comparison and limitations: `validation_vs_paper.md`.

## 16. Final suite at the M10 freeze (9 Nov 2026)

24 CTest programs: 23 test programs plus `m9_protocol_reproduces`; all pass in the Release build tree. Assertions and checks per program (as printed by the programs; `scripts/m10_doc_check.py --build <dir>` recomputes the total): graph 26, model 43, flp 59, timing 291, tsp 29, init 171, search_infra 144, fuzz_search_infra 11,262, evaluator_differential 26,745 (sub-total 38,770); n4_n5 233 + 1,117,041; n7_n8 253 + 1,696,098; n3_n6 587 + 613,135; n1_n2 419 + 3,470,761; shakes 675 + 728,109; vns 108 + 29,447; experiment_config 25; runner_cli 76. **Total 7,695,737.** Earlier milestone totals in sections 6-14 are the totals of those milestones and are unchanged. Final M9/M10 figures: `m9_release_ctest_raw.txt`, `m10_checks_raw.txt`.
