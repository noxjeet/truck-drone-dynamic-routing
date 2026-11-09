# N7 and N8: deletion-reinsertion neighbourhoods (M3)

Status: **implemented and validated (M3, 9 Nov 2026, instruction of 08:45 IST)**. Sections 1--5 are the contracts, written *before* implementation and not changed by it; sections 6--9 (implementation notes, tests, measurements, findings) were added afterwards. Nothing is committed.
Related: `operators_N4_N5.md` (shared contract, D1 window, A3, reject policy), `DECISIONS.md` items 15--32 and the M3 items, `paper1_decision_register.md` Issues 4, 7, 9.

## 0. Sources and their limits

The original PDF of Han, Chi & Sun (2026) is **not** available in the repository or the project files. The paper text used here is the page-image-based extraction `claude/han-chi-sun-2026-extraction.md` (all pages rendered and read visually, quotations near-verbatim), plus `docs/paper1_decision_register.md`. Figures 12a and 12b are known only through their captions in that extraction.

## 1. Paper facts

**[PAPER] numbering.** The paper's eight neighbourhoods (Sec 5.3, p20--23) are, in its order: 1 launch node relocate, 2 recover node relocate, 3 drone service node relocate, 4 truck service node relocate, 5 T-D service node swap, 6 D-D service node swap, **7 truck node delete--drone node insert**, **8 drone node delete--truck node insert**. The project follows that numbering (it is the one the earlier decisions use: N4 = truck relocate, N5 = T-D swap).

* **N7**, p22, Fig. 12a: "randomly selecting a truck service node and then attempting to assign an **available drone** to construct a drone trip." Figure caption: C3 removed from the truck route, trip {C1, C3, C4} added.
* **N8**, p22--23, Fig. 12b: "randomly selecting a drone trip and then attempting to assign it to the truck route. After performing a **feasibility test on the new route**, the cost ... is calculated to determine the optimal order of truck visits." Figure caption: trip {C1, C2, C3} removed, C2 served by the truck.
* Every neighbourhood picks "the most optimal" candidate by the updated customer-waiting-time cost (objective unchanged).
* Execution order (Sec 6.1): N4, N5, **N8**, N3, N6, **N7** (VNS integration is M7, not M3).
* FLP arc rule (p9, (13)--(15)): a launch point of $L_{i,j}$ is usable only while the truck traverses arc $(i,j)$; recovery only at a truck service point or the end depot after the launch (p15--16); every FLP in the route must be used (rule (6)).
* The paper does **not** say: how many truck nodes / positions are "adjacent"; how N7 chooses launch and recovery; what "available" means; what happens to other trips whose FLP arcs the route change destroys; whether "optimal order of truck visits" means the best position of the one inserted customer or a re-optimisation of the whole order.

## 2. Approved decisions that apply

| ID | Content | Used by |
|---|---|---|
| D1 | adjacency window $w=3$, configurable, counted in non-FLP (skeleton) nodes; an implementation interpretation | N7 launch window |
| D4 | new launch/recovery arrangement: best valid FLP on the resulting truck-route arcs within the window; recovery at the next truck node | N7 (new trip) |
| D5 | N8 considers all valid insertion positions (configurable) | N8 |
| A3 | N5's launch window is centred on the former position of the customer that needs a launch | extended to N7 as A7 below |
| item 15 | trips disturbed by a move are rejected and named (`AffectedTrips`), never repaired; only N4 has the approved Reanchor alternative | N7, N8 |
| D6 | dissolution only for shakes | not used: N7 dissolves nothing; N8 dissolves only the trip it is asked to move |
| -- | objective = sum of truck and drone customer arrival times, end-depot return excluded; strict `<`, first generated wins ties | both |

## 3. N7 contract

**Changes:** the truck route loses one truck-served customer $t$; the drone side gains one trip $(f, t, y)$. Nothing else changes.

* **Input:** an evaluator-feasible `Solution`; anchor $t$ = an *interior* truck-served customer (skeleton index $1..S-2$). Randomised overload: one uniform draw over the interior truck customers in route order ("randomly selecting a truck service node", the N4 convention). Not a skeleton customer $\Rightarrow$ `InvalidAnchor`; infeasible input $\Rightarrow$ `InputInfeasible`.
* **Step 1 (delete).** $t$ is removed from the route; the skeleton shrinks, the two arcs $(x,t)$ and $(t,y)$ merge into $(x,y)$.
* **Step 2 (affected trips).** Any other trip whose launch FLP now lies off its own arc (an FLP of $(x,t)$ or $(t,y)$) or whose recovery node is $t$ makes the move one `AffectedTrips` candidate (trips named, not evaluated). Nothing is repaired, no trip is re-anchored or dissolved. **[Deviation-free]**: the paper's own feasibility rule makes such a route infeasible; the project rejects instead of repairing (item 15).
* **Step 3 (new trip, D4).** For each gap $g$ of the *resulting* skeleton whose arc lies within the D1 window of $t$ **measured on the original skeleton** (**A7**) and each FLP $f \in L(x_g, y_g)$ in ratio order: trip $(f, t, y_g)$, recovery $y_g$ = the next truck node after $f$ (the arc's end, the depot when the arc ends at the depot). $f$ is inserted into its gap in ratio order, or reused when it is already on the route (a second launch). Drone id = the lowest id whose other trips do not overlap the new trip's route-position interval (the N5 rule); none $\Rightarrow$ `NoFreeDrone`. The other trips keep their ids.
* **Step 4.** Every candidate is judged by the evaluator (endurance, SL/SR, (28), $|U|$, arcs, exactly-once service are the evaluator's verdict).
* **Invalid candidates:** `AffectedTrips`; `NoReplacementFlp` (no FLP on any eligible arc: one candidate per anchor, nothing else generated); `NoFreeDrone`; `Eval*`.
* **Candidate order / ties:** gaps ascending, FLPs by ratio ascending; strict `<`, first generated wins. Deterministic given the anchor; no duplicates are possible (distinct $(g,f)$ give distinct solutions).
* **[A7, assumption, to be confirmed]** window centre. A3 centres N5's window on the former skeleton position of the customer that needs a launch. For N7 that node no longer exists in the resulting skeleton; the window is therefore evaluated on the **original** skeleton: a resulting gap is eligible iff the half-step coordinate of its arc in the original skeleton is within $2w$ of $2s_t$ (merged arc: coordinate $2s_t$; arcs left of it: $2g+1$; right of it: $2g+3$). For $w=3$ these are 5 gaps (the merged arc and two arcs on each side, all with endpoints within 3 skeleton nodes of $t$). This is the same family as A3, affects only the pre-filter, and is a one-line change if you prefer another centre.

## 4. N8 contract

**Changes:** one drone trip is dissolved; its customer $c$ is inserted into the truck route.

* **Input:** an evaluator-feasible `Solution`; anchor = a trip index $i$. Randomised overload: one uniform draw over trip indices ("randomly selecting a drone trip", the N5 convention). Index out of range $\Rightarrow$ `InvalidAnchor`; no trips $\Rightarrow$ `NoEligibleAnchor`.
* **Step 1 (dissolve).** Trip $i$ is removed (the only dissolution; explicit, the trip is the anchor). Its launch FLP leaves the route iff no other trip launches from it (`flps_dropped`, the N5 convention; rule (6)). Other trips keep their ids, customers and recoveries.
* **Step 2 (insert, D5).** For **every** gap $g$ of the truck skeleton ($0..S-2$, depot-adjacent gaps included; *not* restricted to the D1 window) $c$ is inserted directly before skeleton node $g+1$ (after any FLPs of that gap, the N4 convention). An optional `insertion_window` (D5 "configurable") restricts the gaps to the D1 window around $c$'s half-step coordinate; **default: all gaps**.
* **Step 3 (affected trips).** Inserting $c$ splits the arc of gap $g$. A gap that still holds FLPs of other trips therefore leaves them off their arc: that insertion is one `AffectedTrips` candidate (trips named, not evaluated). Valid insertion positions are exactly the gaps without remaining launch FLPs. No repair, no re-anchoring.
* **Step 4.** Valid insertions are judged by the evaluator. N8 cannot produce `InvalidRecovery` or `NoFreeDrone` (no recovery moves; one trip fewer).
* **Candidate order / ties:** gaps ascending; strict `<`, first generated wins. Deterministic given the trip. Distinct gaps give distinct routes, so no duplicates exist (`duplicates_skipped` stays 0).
* **Interpretation of "optimal order of truck visits":** the best position of the one inserted customer among the existing truck order (D5). The other truck nodes are not reordered (that would be N4).
* **Consequence to be reported, not changed:** the share of valid positions shrinks as trips use more gaps.

## 5. Classification of what the contracts rest on

| Class | Item |
|---|---|
| **Paper fact** | N7 and N8 exist with the quoted descriptions (section 1); the FLP-arc rule; recovery after launch; rule (6); objective and "most optimal" selection |
| **Paper silence** | adjacency size; N7 launch/recovery choice; "available"; fate of other trips whose FLP arcs the move destroys; meaning of "optimal order of truck visits" |
| **Approved decision** | D1 (window 3, skeleton-counted), D4, D5, D6, item 15 (disturbed trips rejected and named, not repaired), strict `<` / first wins, N4-only re-anchoring |
| **Implementation assumption** | **A7** (N7 window centre, section 3); N7 is *not* silently empty when no eligible arc has an FLP (one `NoReplacementFlp` candidate); N8's "optimal order" = best position of the one inserted customer; the Rng overloads draw once uniformly |
| **Deviation from the paper** | none identified. The paper's text is silent where the project rejects (disturbed trips); no paper rule is contradicted |
| **Unresolved** | section 9 |

## 6. Implementation notes

* **Files.** `include/search/neighbourhood_n7.hpp` (`N7Options`, `n7_truck_node_delete_drone_insert`, two overloads: anchor / `Rng`), `include/search/neighbourhood_n8.hpp` (`N8Options{insertion_window}`, `n8_drone_node_delete_truck_insert`, two overloads). `move_types.hpp` changed **additively only**: `OperatorId::N7, N8`, `MoveKind::N7TruckToDrone, N8DroneToTruck`, comments. No reason code was added (`kRejectReasonCount` is still 12); `neighbourhood_n4.hpp`, `neighbourhood_n5.hpp`, `route_ops.hpp`, `candidate_window.hpp`, `drone_assignment.hpp`, `flp_arc_index.hpp`, the evaluator, `spatial_network.hpp` and the frozen reference are byte-identical to the state before M3 (sha256 checked).
* **Reuse.** `make_skeleton`, `remove_from_route`, `insertion_position_for_gap`, `flp_insertion_position`, `flps_off_their_arc`, `trips_launching_from`, `trips_recovering_at`, `drop_unused_flps`, `lowest_free_drone_for_trip`, `FlpArcIndex::flps_on_arc`, `gaps_within_window`, `PositionIndex`, `classify_evaluation`, `detail_n4::truck_customers_in_order`. The only new helper is `detail_n7::gaps_within_window_after_removal` (A7). N7's construction of the new trip duplicates five lines of N5's inline logic on purpose, so that `neighbourhood_n5.hpp` stays byte-identical (the instruction forbids changing N4/N5 to ease N7/N8).
* **Counting conventions** (same as N4/N5): `generated` and `accepted` count candidates, `evaluated` counts evaluator calls, `accepted + rejected_total() == generated` when candidates are evaluated, `duplicates_skipped` is always 0. With `evaluate_candidates = false` nothing is accepted.
* **Diagnostics.** `AffectedTrips` records name the trips (N7: indices of the input; N8: indices of the **input** solution, not of the dissolved one), sorted and unique, and are never evaluated. N8 records carry `flps_dropped`. The route shown in a rejected N7 `AffectedTrips` / `NoReplacementFlp` record is the route **without** $t$ (the intermediate state), which is why the test-side coverage check skips those records.
* **Deliberate difference from N5.** N5 silently generates nothing when no window arc has an FLP; N7 records one `NoReplacementFlp` candidate (a visible, countable outcome). N5 is unchanged.
* **Randomised overloads.** N7: one `uniform_index` draw over the interior truck customers; N8: one over the trip indices. `uniform_index` may use rejection sampling, so "one draw" means one call.

## 7. Tests (`tests/test_n7_n8.cpp`, `tests/test_n7_n8_oracle.cpp`)

**Hand-derived fixtures (253 assertions, `test_n7_n8`).** All expected objectives are derived by hand on a line instance (truck 10 m/s, drone 20 m/s unless stated) and are also compared with the frozen reference evaluator.

| Fixture | Establishes |
|---|---|
| N7-1 | basic generation, 14 candidates in order (gaps ascending, FLPs by ratio), six hand-derived objectives (1900/1575/1475/1500/1600/1725), recovery = arc end, strict-`<` best, input unchanged |
| N7-2 | window boundaries for $w=0,1,2,3$ on both sides of $t$ (A7) |
| N7-3 | depot boundaries (anchors $C_1$ and $C_5$), recovery at the end depot |
| N7-4 | no FLP on any eligible arc $\Rightarrow$ one `NoReplacementFlp`; invalid anchors (depot, FLP, drone customer, unknown); infeasible input; no eligible anchor |
| N7-5 | one trip: `AffectedTrips` (recovery vanishes; FLP on a foreign arc), unaffected anchor $\Rightarrow$ 15 candidates, **FLP reuse** (second launch), lowest free drone id, `|U|=1` $\Rightarrow$ `NoFreeDrone` never evaluated |
| N7-6 | two trips: affected sets named exactly |
| N7-7 | endurance 200 s: 5 accepted, 9 `EvalEndurance` |
| N7-8 | exact tie ($v_d = v_t$): first generated wins |
| N7-9 | `Rng` overload |
| N7-10 | a trip whose **recovery is not the end of its launch arc**: the recovery rule alone produces `AffectedTrips` (added after mutant N7-10 survived) |
| N8-1 | 5 gaps, objectives 2300/1500/2100/2300/2100, **D5: positions outside the D1 window are generated**, `insertion_window` variants, best move descriptor |
| N8-2 | $n=6$, endurance 500: `AffectedTrips` for gaps with FLPs, 2 `EvalEndurance`, best 2025 |
| N8-3 | an FLP shared by two trips: dissolving one keeps the FLP, no valid position remains, objective 950 |
| N8-4 | 2-D exact tie, first generated wins |
| N8-5 | invalid index, no trips, infeasible input, `Rng` overload |

**Independent oracle (1,696,098 checks, `test_n7_n8_oracle`).** Written with its own route / skeleton / arc arithmetic: it reads FLP ownership straight from the FLP data (not `FlpArcIndex`), computes the foreign-FLP and window logic itself, inserts FLPs by its own ratio rule, and judges **every** candidate with the frozen reference evaluator `tdr::timing_ref::evaluate_solution` (lowest reference-feasible drone id for N7). It compares candidate counts, order, routes, trips (identities, customers, launches, recoveries), drone ids, dropped FLPs, `AffectedTrips` lists, reasons, objectives and the best candidate record by record. 60 seeds, $n=6..11$, 1--3 drones, every third seed with shorter endurance; the bases are the initial solutions **and** their random-walk successors under N4/N5/N7/N8, windows 1 and 3, N8 both unrestricted and with `insertion_window = 1`.

| | calls | candidates | accepted | other outcomes |
|---|---|---|---|---|
| N7 | 5,188 | 128,212 | 1,125 | affected 1,314; no-replacement 3; no-free-drone 2,613; endurance 123,157; FLP reuse in 812; depot recovery in 10,919; non-zero drone id in 27; calls with a strict improvement 122 |
| N8 | 1,696 (848 windowed) | 9,120 | 7,946 | affected 1,174; all 1,696 calls drop an FLP; 3,366 candidates lie outside the D1 window (D5); calls with a strict improvement 1,640 |

360 bases (241 with at least one trip). A second full campaign yields an identical digest. Non-vacuity checks are asserted (all the listed categories occur).
The oracle cannot distinguish two N7 window-arithmetic mutants (strict boundary; merged-arc coordinate +2) because half-step distances to the removed node are $0$ or odd, so the boundary is never attained for even $2w$; the hand fixtures N7-2/N7-3 kill them.

**Mutation testing (`tests/mutation/m3_mutate.py`, 53 mutants).** 28 N7 and 21 N8 mutants plus the four shared-helper mutants re-checked against the M3 tests. 52 detected, 1 survivor. Two mutants survived at first (N7-10 *trips recovering at $t$ ignored*, N8-20 *best move lacks the replaced trip*); each exposed a real test gap (no hand-derived case where the recovery rule alone fires; no assertion on the best candidate's move descriptor), the tests were strengthened and both mutants are now detected. The survivor **CW-02** (`window boundary +1 half-step`) is equivalent in M3: N8 centres its optional window on a drone customer's half-step $2g+1$ (odd) against gap coordinates $2g'+1$ (odd), so every difference is even and a $+1$ slack changes nothing; N7 does not use `gaps_within_window`. (The same mutant is detected by the M2 tests, which use truck-customer centres.) The complete M2 suite (64 mutants) was re-run against the unchanged M2 tests: 64 of 64 detected.

## 8. Measurements (`bench/bench_n7_n8.cpp`, `-DTDR_BUILD_BENCH=ON`; raw: `n7_n8_bench_raw.txt`, `n7_n8_bench_endurance_x3_raw.txt`)

Same methodology and fixtures as the M2 operator benchmark (base seed 31415, dataset 0, initial solution, window 3, `record_candidates = false`, 15 repetitions, batch $\ge 20$ ms, median; machine spread 5--25 %, single runs). A sweep = one call per interior truck customer (N7) / per trip (N8; default = all gaps). *gen* = `evaluate_candidates=false`; *full* = true. "Per generated" is net of the one base evaluation every call makes.

| | n = 10 | n = 20 | n = 50 |
|---|---|---|---|
| truck customers / trips | 7 / 3 | 14 / 6 | 27 / 23 |
| evaluator alone | 1.26 us | 2.32 us | 8.71 us |
| **N7** candidates (generated / evaluator calls) | 161 / 157 | 174 / 164 | 88 / 63 |
| N7 feasible | 0 (0 %) | 0 (0 %) | 0 (0 %) |
| N7 rejections | affected 4, endurance 157 | affected 10, endurance 164 | affected 25, endurance 63 |
| N7 sweep, gen / full | 25.5 / 290 us | 58.9 / 516 us | 286 / 882 us |
| N7 per generated candidate (gen, net) | 0.10 us | 0.15 us | 0.58 us |
| N7 per evaluated candidate (full $-$ gen) | 1.68 us | 2.79 us | 9.46 us |
| N7 allocations per full sweep | 10,313 (662 KiB; 64 / candidate) | 15,186 (1.2 MiB; 87) | 16,021 (1.7 MiB; 182) |
| **N8** candidates (generated / evaluator calls) | 24 / 18 | 90 / 60 | 644 / 138 |
| N8 feasible | 18 (75 %) | 60 (66.7 %) | 138 (21.4 %) |
| N8 rejections | affected 6 | affected 30 | affected 506 |
| N8 sweep, gen / full | 8.3 / 30.9 us | 34.6 / 188 us | 480 / 1,817 us |
| N8 per generated candidate (gen, net) | 0.19 us | 0.23 us | 0.43 us |
| N8 per evaluated candidate (full $-$ gen) | 1.25 us | 2.55 us | 9.68 us |
| N8 allocations per full sweep | 1,113 (62 KiB; 46 / candidate) | 5,271 (417 KiB; 59) | 31,768 (3.5 MiB; 49) |

Per evaluated candidate the cost is the evaluator's own cost (within about 10 %); per generated candidate the operators cost 0.1--0.6 us. No bottleneck was found that justifies an optimisation now. Two observations are reported, not acted on: (i) at $n=50$ about 82 % of N7's *gen* sweep (27 base evaluations $\times$ 8.7 us of 286 us) is the precondition check that evaluates the base once per call, which a VNS driver can amortise (M7); (ii) every candidate copies a `Solution` (about 50--180 allocations per candidate including the evaluator's), acceptable at these sizes.

With the endurance tripled (an instance parameter only; `n7_n8_bench_endurance_x3_raw.txt`) the initial solutions hold 5 / 10 / 23 trips and **every** N7 anchor is rejected as `AffectedTrips` (5 of 5, 10 of 10, 27 of 27 candidates, none evaluated); N8 then generates 30 / 110 / 644 candidates with 33 / 18 / 21 % feasible.

N4/N5/evaluator regression, same fixtures: counts identical to the M2 final run; times within the machine's variation (this run was about 10 % faster across the board, evaluator included): N4 Reject sweep 37 / 119 / 567 us (M2: 40 / 128 / 638), N4 budget 100 2.21 / 7.44 / 61.7 ms (2.47 / 7.92 / 67.8), N4 budget 10,000 at $n=50$ 276 ms (302), N5 generation 0.11 / 0.16 / 1.07 us per candidate (0.12 / 0.17 / 1.17), N5 sweep 1.07 / 2.47 / 4.68 ms (1.16 / 2.69 / 5.16), evaluator 1.35 / 2.52 / 9.37 us (1.5 / 2.7 / 10.4); `bench_evaluator` $n=50$ populated 7.86--8.38 us, 30.2--32.6 $\times$ faster than the frozen reference (M2: 7.75--8.03 us, 30.8--32.5 $\times$), all variants equivalent. Raw: `operator_bench_after_m3_raw.txt`, `evaluator_bench_after_m3_raw.txt`.

## 9. Findings and unresolved questions (not decided here)

**Findings (measured, no change made).**

1. **N7 is almost never feasible on paper-scale instances.** In the 60-seed campaign (smaller $n$) 0.9 % of N7 candidates are accepted, 96 % fail endurance; on the benchmark's initial solutions (60 km square, 12 m/s, 1,800 s) none of the 384 evaluated candidates (423 generated) is feasible; every evaluated one is rejected by the endurance rule. Likely explanation (not separately measured): the new drone leg is launch FLP $\to t \to$ next truck node, the drone range is about 21.6 km (1,800 s at 12 m/s) and inter-customer distances in a 60 km square are of that order. What *is* verified: the independent oracle, using the frozen reference evaluator, reproduces every verdict record by record, so this is a property of the instance scale and the evaluator rules, not an implementation limit. It means N7 will rarely contribute at $n\le 20$ unless instances are denser or endurance is larger.
2. **Disturbed trips dominate.** Without re-anchoring (item 15) 25 of 88 N7 candidates at $n=50$ are `AffectedTrips` (and 27 of 27 when the endurance is tripled); for N8 the share of valid insertion positions falls from 75 % ($n=10$) to 21 % ($n=50$) because gaps that hold other trips' launch FLPs are invalid. This is the same effect that motivated the N4 re-anchoring policy.
3. D5 matters: 3,366 of the oracle's 9,120 N8 candidates lie outside the D1 window; a window-restricted N8 would have missed them.

**Unresolved (need a decision, not blocking M3).**

1. **A7**, the N7 window centre (section 3). Alternative centres change only the pre-filter.
2. Whether N7/N8 should get a bounded re-anchoring alternative like N4's (finding 2). Not authorised; the reject policy is the approved default for disturbed trips (item 15), and N4's policy is deliberately not generalised.
3. The N8 `insertion_window` has no paper-given value; the default is unrestricted (D5).
4. Figs. 12a/12b are known only through their captions in the paper extraction; the original paper PDF is not in the repository or the project files.
5. Whether "optimal order of truck visits" (N8) should mean more than the best position of the one inserted customer (it would then overlap N4).
