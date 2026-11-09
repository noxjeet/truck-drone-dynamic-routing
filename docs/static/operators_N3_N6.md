# N3 and N6: drone-trip exchange neighbourhoods (M4)

Status: contracts (sections 1--5) written **before** implementation (9 Nov 2026, M4 instruction of 09:42 IST); implementation notes, tests, measurements and findings (sections 6--9) added after implementation. M4 was signed off on 9 Nov 2026, 12:30 IST.
Related: `operators_N4_N5.md`, `operators_N7_N8.md` (shared contract, D1 window, reject policy), `DECISIONS.md` items 15--44 and 45--53 (M4), `paper1_decision_register.md` Issues 4, 8.

> **A8 revised at the M4 sign-off (9 Nov 2026, 12:30 IST).** Wherever this section says that N6's default excludes trips of the anchor's own drone, or that N6 is empty on one-drone solutions, the statement is **superseded by section 10**; the original wording is kept for the audit trail. A9, A10 and A11 were approved as written.

## 0. Sources and their limits

The original PDF of Han, Chi & Sun (2026) is **not** in the repository or the project files. The paper text used is the page-image-based extraction `claude/han-chi-sun-2026-extraction.md` (quotations near-verbatim) and `docs/paper1_decision_register.md` (Issue 8). Figures 10c and 11b are known only through their captions and the register's reading of the figure (trip compositions, colours).

## 1. Paper facts

* **N3, drone service node relocate** (Sec 5.3, p21, Fig. 10c): "repositioning the drone service node of the drone trip. The operation requires searching **adjacent trip** of the original drone trip, with the objective of ascertaining the most optimal relocation location". Caption: "The drone service node C4 of drone trip {C1,C4,C3} and the drone service node C2 of drone trip {C3,C2,C5} perform relocation operations, reducing the waiting times for customers C2 and C4." Result of the figure: trips {C1,C2,C3} and {C3,C4,C5}; truck route unchanged; both trips drawn in the same colour (drone 1, Fig. 9 legend); the first trip's recovery node C3 is the second trip's launch node.
* **N6, D-D service node swap** (p22, Fig. 11b): "Randomly selecting a drone trip and searching adjacent drone service node of **other** drone trips." Caption: C2 and C4 are exchanged "to reduce the time it took the truck to recover the drone at node C3 and reduce the waiting time for customer C5". The figure shows an orange (drone 1) trip and a green (drone 2) trip.
* Every neighbourhood selects "the most optimal" candidate by the updated customer waiting time (objective unchanged). Execution order (Sec 6.1): N4, N5, N8, **N3**, **N6**, N7 (VNS integration is M7).
* A drone trip serves exactly one customer (the representation is `(drone_id, launch FLP, customer, recovery node)`).
* The paper does **not** say: what "adjacent trip" means (next/previous trip of the same drone, or nearby in route or space); whether both directions are searched; for N6, whether "other drone trips" excludes trips of the same drone, what "adjacent" means, and whether launch and recovery nodes move with the customers.

## 2. Approved decisions that apply

| ID | Content | Used by |
|---|---|---|
| D1 | adjacency window $w=3$, configurable, counted in skeleton (non-FLP) nodes via half-step coordinates; an implementation interpretation | N6 partner pre-filter. **Not used by N3** (its adjacency is the drone sequence, D3) |
| D3 | N3 exchanges **consecutive trips belonging to the same drone** | N3 |
| D4 | FLP re-selection convention (best valid FLP on resulting arcs) | **not applicable**: neither operator changes the truck route or any launch/recovery node, so no FLP is selected |
| item 15 / M3 | disturbed trips are rejected and named, never repaired; re-anchoring is N4-only | not triggered: nothing is disturbed |
| -- | objective = sum of truck and drone customer arrival times, end-depot return excluded; strict `<`, first generated wins ties, no epsilon; FLP pool eager | both |

## 3. N3 contract (consecutive trips of the same drone)

**Transformation (paper + D3).** Two trips of the **same drone** that are **consecutive in that drone's sequence** exchange their served customers: $s_1=(u,i_1,a,k_1)$ and $s_2=(u,i_2,b,k_2)$ become $s_1'=(u,i_1,b,k_1)$, $s_2'=(u,i_2,a,k_2)$. Fig. 10c is exactly this. Because every trip serves exactly one customer, "relocating" a drone service node to the adjacent trip *is* the exchange: the only other customers a trip's drone could take are truck customers (N5) or customers of trips of other drones (N6).

* **Changes:** the `service_customer` of two trips. **Fixed:** truck route, every launch FLP and recovery node, every drone id, trip order in the solution vector, all other trips.
* **Input:** an evaluator-feasible `Solution` and an anchor trip index $i$. Randomised overload: one uniform draw over the trip indices ("randomly selecting a drone trip", the N5/N8 convention). Index out of range $\Rightarrow$ `InvalidAnchor`; infeasible input $\Rightarrow$ `InputInfeasible`; no trips $\Rightarrow$ `NoEligibleAnchor` (Rng overload).
* **"Consecutive".** The trips of drone $u$ are ordered by the route position of their launch FLP (ties by recovery position, then trip index). Two trips are consecutive iff adjacent in that order. (Constraint (28) forbids overlapping trips of one drone, so in a feasible input this order is also the time order.)
* **Enumeration.** For the anchor: at most two candidates, **predecessor first, then successor**. An anchor with no same-drone neighbour (alone on its drone) yields zero candidates and status `Ok`. The D1 window is *not* applied (D3 defines adjacency by the drone sequence); `SearchContext::adjacency_window` has no effect on N3.
* **Affected truck nodes and trips:** none. The route is unchanged, so no FLP can leave its arc and no recovery node changes; `AffectedTrips`, `NoReplacementFlp`, `InvalidRecovery` and `NoFreeDrone` cannot occur.
* **Feasibility:** by the evaluator alone (endurance (36)/(37) of both trips with the new flight legs, SL/SR timing, (28), $|U|$, rule (6)). The exchange can change flight times and hence the timing of the whole route; each verdict is classified (`EvalEndurance`, `EvalFleetCapacity`, `EvalDroneSequencing`, ...), never overridden.
* **Duplicates:** none within a call (distinct partners give distinct solutions); `duplicates_skipped == 0`. The pair $(s_1,s_2)$ reached from either anchor is the same solution, but N3 is invoked per anchor.
* **Order / ties:** predecessor, successor; strict `<`, first generated wins. Deterministic given the anchor.

## 4. N6 contract (customers of trips of different drones) --- partner rule superseded by section 10 (A8)

**Transformation (paper text and caption).** The drone service node of the anchor trip and the drone service node of an *adjacent other* trip are exchanged: $s=(u,i,a,k)$ and $s'=(u',i',b,k')$ become $(u,i,b,k)$ and $(u',i',a,k')$. Launch and recovery nodes and drone ids stay with their trips (**A9**).

* **Changes / fixed:** as N3 (two `service_customer` fields; nothing else).
* **Input:** an evaluator-feasible `Solution` and an anchor trip index. Randomised overload: one uniform draw over the trip indices ("randomly selecting a drone trip"). Out of range $\Rightarrow$ `InvalidAnchor`, etc. as N3.
* **Partners ("adjacent ... of other drone trips", D1).** Let $h$ be the half-step coordinate of a drone customer ($2g+1$, $g$ = gap holding its launch FLP; `PositionIndex`). The partners are the drone customers of *other* trips with $|h_{\text{anchor}}-h_{\text{partner}}|\le 2w$ (the same function that gives N5 its adjacent truck customers, `PositionIndex::neighbours`), ordered nearest first, ties by customer id (that function's order), restricted to **trips of a different drone** (**A8**, below; `N6Options::include_same_drone` = true admits every other trip).
* **Affected truck nodes and trips:** none (as N3).
* **Feasibility, rejections, duplicates, order, ties:** as N3. `duplicates_skipped == 0`.

## 5. What the contracts rest on

| Class | Item |
|---|---|
| **Paper fact** | the quoted descriptions and captions (section 1); Fig. 10c's result (exchange, route unchanged, launch/recovery unchanged, same colour); Fig. 11b shows an exchange between a drone-1 and a drone-2 trip; objective and "most optimal" selection |
| **Paper silence** | the extent of "adjacent trip" / "adjacent drone service node"; both-direction search; whether N6 may pair trips of one drone; whether launch/recovery move in N6 |
| **Approved decision** | D1, D3, item 15 and the M3 rule (no invented repair rules), re-anchoring N4-only, strict `<` / first wins |
| **Implementation assumption** | **A8** N6 partners are trips of a **different** drone by default (the paper says "other drone trips"; the register and the Fig. 11b colours point to different drones, and it makes the N3 and N6 candidate sets disjoint, so the two neighbourhoods do not duplicate each other). Switchable. **A9** N6 exchanges only the customers; launch FLPs, recovery nodes and drone ids stay with their trips (as Fig. 10c shows for N3; Fig. 11b's trip compositions are not available). **A10** N6 adjacency = the D1 window on the drone customers' launch-gap half-steps (same family as A3/A7). **A11** N3 predecessor and successor are both searched (anchor-centred), predecessor first. **A12** the randomised overloads draw once uniformly over the trip indices. |
| **Deviation from the paper** | none identified |
| **Unresolved** | section 9 |

## 6. Implementation notes

| File | Role |
|---|---|
| `include/search/trip_exchange.hpp` (new, 78 lines, `detail_exchange`) | the part N3 and N6 share: `begin` (evaluate the base: `InputInfeasible`; anchor range: `InvalidAnchor`; set `base_objective`, `anchor_customer`) and `judge` (copy the base, exchange the two `service_customer` fields, build the `Move`, evaluate, classify, count, keep the best by strict `<`) |
| `include/search/neighbourhood_n3.hpp` (new, 85 lines) | `n3_drone_service_node_relocate(base, ctx, trip_index, N3Options)` and the `Rng` overload; `detail_n3::drone_sequence` |
| `include/search/neighbourhood_n6.hpp` (new, 67 lines) | `n6_drone_drone_swap(base, ctx, trip_index, N6Options)` and the `Rng` overload; `N6Options::include_same_drone` (default false) |
| `include/search/move_types.hpp` (additive) | `OperatorId::{N3, N6}` (the enum is now `N3, N4, N5, N6, N7, N8`; it is only compared, never cast to an integer), `MoveKind::{N3Exchange, N6Swap}`, `Move::partner_trip`; `kRejectReasonCount` is still 12 |

* **Nothing is disturbed.** An exchange changes two `service_customer` fields and nothing else, so the truck route, every launch FLP, every recovery node, every drone id and the trip order stay as they were. `AffectedTrips`, `NoReplacementFlp`, `InvalidRecovery`, `NoFreeDrone`, `EvalFlpArc` and `EvalFlpUnused` cannot occur; endurance, fleet capacity, (28) and the timing are the evaluator's verdict, classified by the same `classify_evaluation` as every other operator. Consequently **no repair, re-anchoring or dissolution rule is involved** (item 15 and "re-anchoring is N4-only" are untouched), and D4 (FLP re-selection) has nothing to select: it is *not applicable* to either operator.
* **Which evaluator reasons are reachable.** Endurance (36)/(37) is: the exchange changes the flight legs of both trips, and via the truck waiting at a recovery node it changes the timing of everything after it. Drone sequencing (28) compares launch and recovery *route positions* of one drone's trips, which an exchange does not change; a drone is always recovered before its next launch because the truck leaves a recovery node only after the drone has arrived, so the number of drones in flight cannot exceed the number of drone ids in use: `EvalDroneSequencing` and `EvalFleetCapacity` are therefore not reachable by construction. This is an argument, not a proof about the evaluator's code; it is confirmed by the evidence (0 occurrences in 595,093 oracle checks and in the unit tests).
* **Counting.** `generated` counts candidates; with `evaluate_candidates` every candidate is evaluated (`evaluated == generated`) and receives exactly one outcome (`accepted + rejected_total() == generated`); `evaluate_candidates = false` accepts nothing. `duplicates_skipped` is always 0 (distinct partners give distinct solutions).
* **N3 enumeration.** `drone_sequence` orders the anchor drone's trips by (route position of the launch FLP, route position of the recovery node after it, trip index); the candidates are the predecessor, then the successor, whichever exist. A lone trip gives zero candidates with status `Ok`. N3 does not read `SearchContext::adjacency_window`.
* **N6 enumeration.** `PositionIndex::neighbours(anchor customer, w)` (the function N5 uses): customers whose half-step coordinate is within $2w$ of the anchor's, nearest first, ties by ascending customer id; truck customers are skipped (that is N5), then trips of the anchor's drone unless `include_same_drone`. The window is applied to the half-step of the *launch gap* of each drone customer ($2g+1$).
* **Randomised overloads.** One `uniform_index(#trips)` draw (A12), then the deterministic call. `NoEligibleAnchor` if there are no trips.

## 7. Tests

`tests/test_n3_n6.cpp` — **565 assertions**, hand-derived on a line (every expected number is worked out in the comment of its fixture and was *not* read back from the code; all of them agreed on the first run). `tests/test_n3_n6_oracle.cpp` — **595,093 checks**.

| Fixture | What it pins down |
|---|---|
| N3-1 (Fig. 10c: trips $\{C1,C4,C3\},\{C3,C2,C5\}$ of one drone) | the exchange itself: 1550 $\to$ 1400 (strict improvement), both anchors reach the same pair, route/launch/recovery/drone ids unchanged, descriptors |
| N3-2 (three trips of one drone, vector order T3,T1,T2) | predecessor before successor, ordering by *launch position* (not vector index), non-consecutive trip never exchanged, best = second candidate (2875 < 2975), window has no effect on N3 |
| N3-3 (two drones, T2 between T1 and T3 in the route) | **only trips of the same drone, consecutive in that drone's sequence**: T1's partner is T3 (3725, with a truck wait of 175 s propagating to later nodes), the lone trip on drone 1 has zero candidates, no partner ever belongs to another drone |
| N3-4 / N3-4b, N6-2 / N6-2b (drone speed 100 m/s, exact tie 2605 = 2605) | strict `<`, first generated wins, in both index orders (the tied candidate with the larger trip index must *not* win); equality is exact, no tolerance |
| N3-5 (e = 260 / 274.99 / 275 / 3600) | `EvalEndurance` and only that, the best-effort objective (1700) of a rejected candidate, the inclusive boundary $e=(37)=275$ |
| N3-6 / N6-5 (recovery at the end depot) | the depot recovery and all launch nodes survive the exchange; N6 window boundary at $w=1$ (none) vs $w=2$ |
| N3-7 / N6-7, N6-5 | lone trip $\Rightarrow$ zero candidates; `InvalidAnchor`; trip-free solution; omitted customer and duplicate customer $\Rightarrow$ `InputInfeasible` with nothing generated; `NoEligibleAnchor` |
| N3-8 / N6-7 | `evaluate_candidates=false`, `record_candidates=false`, determinism, `Rng` overload (anchor $=$ `uniform_index`, one draw, all trips reachable) |
| N6-1 (chain, T1,T3 on drone 0, T2 on drone 1) | windows $w=0,1,2,3$ at the half-step boundaries ($dh=4$ and $6$), same-drone partners excluded by default and admitted by `include_same_drone`, nearest-first order (T3 at $dh=2$ before T1 at $dh=4$), truck customers inside the window never partners, N3 and N6 partner sets disjoint |
| N6-3 (three drones, partners in the same gap) | $dh=0$ is inside every window ($w=0$), equal distance $\Rightarrow$ ascending *customer id* (not trip index), a tie of a candidate with the base objective (1295) is no improvement |
| N6-4 (e = 260, two drones) | `EvalEndurance`, window 1 vs 2, same-drone option |
| N6-6 | one-drone solution: zero candidates by default; `include_same_drone` gives N3's pair in N6's order |

`check_records` is applied to *every* candidate of the main fixtures: exactly the two trips' customers exchanged and nothing else, exactly-once coverage, no FLP off its arc, production objective and verdict equal to the **frozen reference** evaluator, only evaluator reasons, one outcome per candidate, no duplicate solutions, best = first lowest feasible.

**Oracle** (independent brute force; calls no production helper — not `drone_sequence`, not `PositionIndex`, not `trips_serving`): for 80 seeded instances ($n=8$--$14$, 1--3 drones; four variants of area/endurance so feasible exchanges occur) and 8 random-walk bases each (N3/N4/N5/N6/N7/N8), plus drone-id re-draws of the bases (any assignment the reference accepts is a legal solution; 299 accepted), it recomputes the drone sequences from route positions (and cross-checks that the reference launch times increase along them), the half-step coordinates from its own skeleton arithmetic, the expected candidate lists for N3 (at $w=0$ and $3$) and for N6 ($w=0,1,3$, both drone policies), and judges every candidate with the reference. `n3_n6_oracle_stats_raw.txt`:

| counter (names as printed in `n3_n6_oracle_stats_raw.txt`) | N3 | N6 (default policy; $w=0,1,3$) |
|---|---|---|
| calls | 3,007 | 9,021 |
| candidates / accepted | 7,036 / 3,540 | 5,806 / 3,066 |
| endurance rejections / `other_eval` | 3,496 / 0 | 2,740 / 0 |
| `improving_calls` | 612 | 671 |
| `candidate_ties` | 0 | 0 |
| `both_neighbours` / `lone` (N3) | 919 / 408 | |
| `partner_not_vector_adjacent` (N3: partner not next in the solution vector) | 1,688 | |
| `other_drone_launches_between` (N3) | 954 | |
| `empty_calls` / `calls_excluding_same_drone_partner` (N6) | | 5,538 / 3,777 |
| `same_drone_candidates(+same)` (N6 with `include_same_drone`) | | 6,468 |
| `calls_with_truck_customer_in_window` (N6) | | 6,014 |

Bases: 640 (638 with trips), 401 with $\ge 2$ drones in use, 464 with $\ge 3$ trips on one drone; 299 drone-id re-draws accepted by the reference.

0 failures; FLP-arc, idle-FLP, fleet, sequencing and construction-time rejections: **0**; the run is repeated and the digest of all results is identical. No *random* tie occurred (0 candidate ties) — ties are covered by the hand fixtures and the mutants.

**Mutation** (`tests/mutation/m4_mutate.py`, 58 mutants, `m4_mutation_results_raw.txt`): **54 detected**; 4 survive the M4 programs, all equivalent for N3/N6: N3-12 (sort by recovery position instead of launch position: identical under (28)), N6-07 (the defensive `j == trip_index` guard can never fire), CW-04 and CW-06 (offsets of drone/truck half-steps change only truck-drone distances, which N6 never uses; both are killed by the M2/M3 suites, `TDR_TESTS=...`). Two gaps found by the mutants and closed: the tie-break `<=` restricted to larger trip indices (TE-24) survived until fixtures N3-4b/N6-2b were added; N6-07/N3-12 were analysed as equivalent, not added. A spot check of 33 of the 117 older M2/M3 mutants on the unchanged M2/M3 tests: 33 of 33 detected (`m1_m3_mutation_spotcheck_after_m4_raw.txt`).

## 8. Measurements --- the N6 rows are superseded by section 10 (A8 default changed); the N3 rows are unchanged

`bench/bench_n3_n6.cpp`, the same methodology and fixtures as M2/M3 (base seed 31415, dataset 0, initial solution, window 3; a sweep = one call per drone trip; `gen` = `evaluate_candidates=false`, `full` = with evaluation; 15 reps, median; Release `-O3`; two shared cores, so spreads of 5--20 % are normal). Raw files: `n3_n6_bench_raw.txt` (paper parameters), `n3_n6_bench_alt_drones_raw.txt`, `n3_n6_bench_endurance_x3_raw.txt`, `n3_n6_bench_endurance_x3_alt_drones_raw.txt`.

**Paper parameters (endurance 1800 s, 60 km square), initial solutions.** The initial solution flies *every* trip with drone 0 at all three sizes.

| n | trips | N3 candidates | N3 feasible | N3 evaluator calls | N3 sweep | N3 gen / candidate | N3 eval / evaluated candidate | N6 candidates | N6 sweep |
|---|---|---|---|---|---|---|---|---|---|
| 10 | 3 | 4 | 0 (4 endurance) | 4 | 10.8 us | 0.12 us | 1.61 us (evaluator 1.27) | **0** | 5.3 us |
| 20 | 6 | 10 | 0 (10 endurance) | 10 | 44.0 us | 0.12 us | 2.83 us (evaluator 2.43) | **0** | 19.0 us |
| 50 | 23 | 44 | 4 = 9.1 % (40 endurance) | 44 | 664 us | 0.27 us | 9.88 us (evaluator 9.42) | **0** | 278 us |

* **N6 has no candidate on any initial solution** (one drone in use $\Rightarrow$ no trip of a different drone, A8). The N6 sweep cost is then the per-call base evaluation and the `PositionIndex` build only (about 12 us per call at $n=50$, of which the evaluator is 9.4 us).
* Optional variant `include_same_drone` (labelled, not the default): 2 / 8 / 108 candidates; 0 / 0 / 4 feasible (3.7 %); sweep 8.6 us / 43.7 us / 1.39 ms; generation 0.76 / 0.57 / 0.58 us per candidate; 10.3 us per evaluated candidate at $n=50$.
* **Labelled variants (not paper parameters):** (a) *alternate drones* — the same trips assigned alternately to drones 0,1,0,1,... (kept if the evaluator accepts): N6 then has 2 / 6 / 70 candidates, 0 / 0 / 4 feasible (5.7 %), sweep 9.0 / 39.4 / 1,009 us, 0.79 / 0.91 / 0.99 us per generated candidate, 10.3 us per evaluated candidate at $n=50$; N3 2 / 8 / 42 candidates, 0 feasible. (b) *Endurance $\times 3$* (5400 s, to exercise the accepted path only): N3 8 / 18 / 44 candidates, 25 / 56 / 86 % feasible, sweep 23 / 92 / 657 us; with alternate drones N6 has 12 / 28 / 62 candidates, 17 / 36 / 71 % feasible, sweep 34 / 139 / 885 us.
* **Cost per candidate** is dominated by the evaluator: generation (copy + exchange) is 0.1--0.3 us per N3 candidate and about 1 us per N6 candidate (the neighbour search), against 1.3--10 us for the evaluation, which is the evaluator's own cost.
* **Allocations** (full sweep, `new` counter): N3 392 / 1,292 / 11,387 (98 / 129 / 259 per candidate; 1.2 MB at $n=50$); N6 with candidates (alternate drones) the same order. Each call evaluates the base once (about 170 allocations inside the evaluator) and each candidate copies the solution (3 allocations) and evaluates it; nothing unexpected, nothing optimised.
* **Regression** (`operator_bench_after_m4_raw.txt`, `n7_n8_bench_after_m4_raw.txt`, `evaluator_bench_after_m4_raw.txt`): N4 / N5 / N7 / N8 candidate, evaluator-call, acceptance and rejection counts are identical to the M3 files; times are within machine variation (e.g. N5 sweep 1.04 / 2.42 / 4.55 ms vs 1.07 / 2.47 / 4.68 ms; N4 budget-100 sweep unchanged in behaviour; N7 287 / 524 / 937 us vs 290 / 516 / 882 us; N8 31 / 189 / 1,920 us vs 31 / 188 / 1,817 us); `bench_evaluator` populated $n=50$: production 8.4--8.8 us, 33.3--35.2 $\times$ faster than the frozen reference (M3: 30.2--32.6 $\times$): the P1/P2 gain is intact.

## 9. Findings and unresolved questions --- finding 1 and question 1 are resolved by section 10; questions 2--4 were approved at the sign-off

**Findings (reported, nothing changed).**
1. **N6 (default) is empty on paper-scale initial solutions** and, more generally, whenever all trips fly with one drone. `lowest_free_drone_for_trip` hands out drone 0 whenever it is free, so a second drone appears only when trips overlap; in the oracle's own random walks drone 1 is rarely used until ids are re-drawn. Under A8 N6 then contributes nothing in a VNS run until N5/N7 happen to create overlapping trips. This is a direct consequence of A8, and of A8 being a *reading* of "other drone trips".
2. **At paper scale most exchanges are endurance-infeasible** (N3 feasible 0 / 0 / 9 % at $n=10/20/50$; with the labelled tripled endurance 25--86 %): the exchange moves a customer to a trip whose launch and recovery points were chosen for another customer, and the paper's 30-minute endurance is tight (same finding as N7).
3. **Evaluator cost dominates** (1.3 / 2.4 / 9.4 us); N3/N6 generation is negligible. Each call re-evaluates the base solution (needed for `InputInfeasible` and `base_objective`); inside a VNS iteration that is one extra evaluation per call, not per candidate.

**Unresolved questions (for your decision; none blocks M4).**
1. **A8, N6 partners.** Is "other drone trips" *trips of a different drone* (implemented default) or *any other trip*? Fig. 11b's colours (drone 1 and drone 2) suggest the former, but the figure is known only through its caption. `N6Options::include_same_drone` switches it, but then N6 re-generates N3's consecutive-pair candidates (the same solution, from a different operator); the oracle and tests cover both settings.
2. **A9, what moves in N6.** Only the customers (launch FLPs, recovery nodes and drone ids stay with their trips). An alternative reading would also re-select each trip's launch/recovery for its new customer (D4); the paper does not say, and that would add a disturbed-trip problem N6 does not have today.
3. **A10, N6 adjacency.** The D1 window on launch-gap half-steps. Another reading: adjacency of the *customers'* positions or of the trips' launch/recovery spans. The window is configurable.
4. **Extent of N3's "adjacent trip".** Both the predecessor and the successor are searched (A11). A successor-only or a larger-neighbourhood reading is possible; D3 fixes only "consecutive trips of the same drone".
5. **Source limits.** Figs. 10c and 11b are known only through their captions and the register's reading; the original PDF is not in the repository. Everything in sections 1--2 is quoted from the extraction; nothing was checked against the printed figures.
6. **Overlap with N5 and N7 results.** The exchange operators cannot change the truck route, so they reach only solutions in which the set of drone-served customers is unchanged; mixed moves are N5/N7/N8.

## 10. Sign-off revision (9 Nov 2026, 12:30 IST): A8 default changed, A9--A11 approved

**Decisions of the sign-off.** A8: N6's default admits **any other trip**, trips of the anchor's own drone included; an option restricts the partners to different drones; the N3/N6 candidate overlap is acceptable. A9 (customers-only exchange), A10 (D1 window on launch-gap half-steps) and A11 (predecessor then successor) are approved as written.

**Change (the only code change).** `N6Options::include_same_drone` now defaults to **true** (`include/search/neighbourhood_n6.hpp`, one default value and the header comment). `include_same_drone = false` is the restricted option (only trips of a different drone). Everything else in N6 and all of N3 is unchanged; with the default, N6 for an anchor visits every other trip's customer within the window, nearest first, ties by customer id (section 4, items 1, 3--5).

**Consequences.**
* N6 and N3 now overlap: an anchor's consecutive same-drone trip inside the window is generated by both, as the same solution (accepted by the sign-off; no de-duplication across operators).
* N6 is no longer empty on one-drone solutions: the paper-scale initial solutions (all trips on drone 0) give N6 candidates (below). The restricted option still gives zero there.
* A8 is therefore an *approved decision*, no longer an open implementation assumption.

**Tests after the change.** `test_n3_n6`: **587** assertions (was 565): the five assertions that pinned the old default were rewritten to name the restricted option explicitly, and block N6-1b was added (default is true; default == explicit `include_same_drone = true` for every anchor and $w=0..3$; the T1/T3 same-drone partner appears in the nearest-first order; the overlap with N3 is the same solution; the restricted option drops it; N6-4 and N6-6 test both options). `test_n3_n6_oracle`: **613,135** checks (was 595,093): the oracle still passes the option explicitly (both settings, as before) and now also checks that default-constructed options equal `include_same_drone = true` in every call where it ran the unrestricted brute force. Whole suite: see `test_and_validation.md` section 10.

**Mutation.** `tests/mutation/m4_mutate.py` N6-13 now flips the new default (`true` to `false`); the 14 N6 mutants were re-run: 13 detected, N6-07 (equivalent, a guard that cannot fire) survives as before. The N3 and exchange mutants are unaffected (their sources are unchanged); the earlier 54-of-58 result is therefore 54 of 58 again (`m4_mutation_results_raw.txt`, appended section).

**Measurements (paper parameters, n = 10 / 20 / 50, initial solutions on drone 0; raw: `n3_n6_bench_raw.txt` regenerated).** N3 is unchanged (4 / 10 / 44 candidates, 0 / 0 / 9.1 % feasible). N6 default: **2 / 8 / 108 candidates**, 0 / 0 / 4 feasible (3.70 % at $n=50$), the rest `EvalEndurance`; sweep 8.8 / 43.2 / 1,368 us; generation 0.76 / 0.60 / 0.62 us per generated candidate, 1.71 / 3.01 / 10.07 us per evaluated candidate (evaluator alone 1.28 / 2.39 / 9.27 us); allocations per full sweep 335 / 1,308 / 23,833. The restricted option (labelled variant): 0 / 0 / 0 candidates, sweep 5.4 / 19.0 / 272 us. Labelled variants (regenerated): alternate drone ids, N6 default 2 / 8 / 108 candidates (restricted 2 / 6 / 70, 4 feasible = 5.7 % at $n=50$); endurance $\times3$ with drone 0 only, N6 default 18 / 44 / 104 candidates, 11 / 23 / 62 % feasible; endurance $\times3$ with alternate drones, N6 default 18 / 44 / 104, 11 / 23 / 62 % feasible. (The old N6 rows of section 8 are the restricted option.)

**Findings.** Finding 1 of section 9 (N6 empty on initial solutions) holds only for the restricted option. The cost of an N6 call is the evaluator's own cost per candidate; with the default the $n=50$ sweep is 1.37 ms (108 evaluator calls). Unresolved question 1 of section 9 is closed; questions 2--4 are approved (A9, A10, A11). Still open: the extent of "adjacent trip" beyond the consecutive pair (N3), the missing paper PDF.
