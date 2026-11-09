# Shaking operators S1--S4 (M6)

**M6 signed off (9 Nov 2026, 13:56 IST).** The user approved A19--A25 and A27--A29 as implemented and A26 with a clarification: *only trips whose launch FLPs are genuinely invalidated by the reinsertion operation are dissolved; the affected trips and the exact reason are recorded; unrelated trips are not dissolved merely because they occupy the selected gap.* The implementation already satisfies this and no code changed: inserting a customer into a gap splits the arc $(u,v)$ of that gap, so exactly the trips whose launch FLP lies on that arc are invalidated (reason `ArcSplitByReinsertion`, recorded with trip index, drone id, customer, launch FLP and recovery); trips launching in any other gap are preserved (asserted by fixture F in `test_shakes`: the trip of the other gap stays in `preserved`, for 40 seeds and both drawn gaps).

Status: contracts (sections 1--5) written **before** implementation (9 Nov 2026, M6 instruction of 13:19 IST). Implementation, tests, measurements and findings are appended after implementation (sections 6--9).
Related: `operators_N1_N2.md` ... `operators_N7_N8.md` (shared conventions), `DECISIONS.md` D6 and the M6 items, `paper1_decision_register.md` Issue 4 (drone identity), Issue 9 (move responsibility for idle FLPs) and the M6 addendum.

## 0. Sources and their limits

The original PDF of Han, Chi & Sun (2026) is not in the project files. The paper text used is `claude/han-chi-sun-2026-extraction.md` (Sec 5.4, p23, quotations near-verbatim) and the register. The extraction has the third shake's text intact ("Random_Shift"); `paper1-understanding.md` records the source as corrupted there, but the extraction's wording is used. Shakes are **not** drawn in a figure that is available here.

## 1. Paper facts

"VNS randomly selects a certain neighborhood for shaking search in each iteration search." Four shake neighbourhoods:

1. **Random_Reverse:** "randomly select two nodes in the truck route, and reverse the truck path segment including two nodes."
2. **Random_Exchange:** "two truck path segments of equal length are selected at random and subsequently exchanged."
3. **Random_Shift:** "randomly move a randomly selected truck path segment forward/backward by several nodes."
4. **Random_Drone_Remove:** "a drone is selected at random, and its whole trips are removed from the current solution. The customers of these trips are served by the truck."

The paper does **not** say: how long the segments are, what "several" is, whether the depots can be selected, how the customers returned by Random_Drone_Remove are placed in the truck route, what becomes of drone trips and flexible launch points when a route segment moves, whether the shake output must be feasible, or the selection probabilities.

## 2. Approved decisions that apply

| ID | Content |
|---|---|
| D6 | Uniform choice among the four. After a shake **only** trips whose launch/recovery relationship became invalid are dissolved; unaffected trips are preserved; the number of dissolved trips and the reason are recorded; invalid trips are **never silently repaired**. |
| Issue 4 | Physical drone id on every trip; Random_Drone_Remove removes all trips with a random id that has at least one trip. |
| Issue 9 / D6 convention | An FLP left with no trip is dropped explicitly and counted. |
| item 15 | No invented repair; re-anchoring of launch FLPs is N4-only (so shakes do **not** re-anchor). |
| D7 | Shake parameters are configurable (calibration is M7). |
| model | Launch at FLPs only; recovery at a truck customer or the end depot, strictly after the launch; FLPs belong to directed arcs (`FlpData::arc_from/arc_to`), in ratio order; rule (6) (no idle FLP); the evaluator is the only feasibility judge and is never overridden; objective and strict `<` unchanged. |

## 3. Common contract

* **Representation.** The truck skeleton is the route's non-FLP nodes $s_0=\text{depot},s_1,\dots,s_m,s_{m+1}=\text{depot}$; $m$ = number of truck-served customers. A "truck path segment" is a run of consecutive interior skeleton nodes $s_a..s_b$, $1\le a\le b\le m$. The depots are never selected, moved or reversed.
* **Input.** An evaluator-feasible `Solution` (otherwise `InputInfeasible`, nothing generated). **Output (`ShakeResult`):** the shaken `Solution`, the kind and the **drawn parameters**, the list of dissolved trips with reasons, the trips removed by the shake itself (Random_Drone_Remove), the preserved trips, the dropped FLPs, the reinsertions, and the evaluator's verdict on the result (`feasible`, `objective`, reason class). Statuses: `Ok`, `InputInfeasible`, `NotApplicable` (the selected shake cannot act: fewer than two truck customers for S1--S3, no drone trip for S4), `InvalidParameters` (explicit-parameter API only).
* **Route shakes (S1--S3), FLPs follow their arcs.** The skeleton is permuted. The new route is rebuilt from the new skeleton: for every consecutive pair $(u,v)$ of the new skeleton, the FLPs of the base route that belong to the directed arc $(u,v)$ are placed in the gap in their old (ratio) order. An arc that survives (a pair consecutive both before and after, e.g. inside a moved or exchanged segment, or at an untouched place) keeps its FLPs and its trips; a **vanished** arc's FLPs leave the route (they are *not* re-anchored).
* **Invalid trips (D6).** In the input trip order, a trip is invalid iff **(R1) `LaunchArcVanished`**: its launch FLP is no longer on the route (its arc vanished), or **(R2) `RecoveryNotAfterLaunch`**: its launch survives but its recovery node is not strictly after the launch on the new route (the end depot always is). Every other trip is preserved untouched (drone id, launch, customer, recovery, relative order). Endurance, SL/SR timing, (28) and fleet capacity are **not** dissolution reasons: they are the evaluator's verdict on the result, reported, never repaired, so a shake output may be evaluator-infeasible (M7's concern).
* **Dissolution.** An invalid trip is removed; its customer is returned to the truck; FLPs left with no trip are dropped explicitly (`flps_dropped`, rule (6)). The number of dissolved trips and each reason are recorded.
* **Returned customers.** In input-trip order (dissolved trips, or the removed drone's trips), each returned customer is inserted into one **free gap** of the current skeleton, drawn uniformly (one `uniform_index(#free)` draw per customer, always): a gap is free iff no FLP entry lies in it (after the idle-FLP drop every FLP entry is some trip's launch, so inserting into a free gap splits no arc and invalidates nothing); depot-adjacent gaps are included. **If no gap is free**, one gap is drawn uniformly among all gaps (`uniform_index(#gaps)`), the trips launching from its FLPs are dissolved with reason **`ArcSplitByReinsertion`** (their FLPs dropped, their customers appended to the queue) and the customer is inserted; the queue strictly shrinks the trip set, so this terminates. Nothing else is moved, nothing is re-selected, no drone id changes.
* **Result.** The evaluator judges the result (`evaluate_result`, default on) and its verdict is classified with `classify_evaluation`; a rejected result is returned as such and **never altered**. Exactly-once coverage, no foreign FLP, no idle FLP hold by construction.
* **Randomised API.** Each shake has (i) a deterministic core taking the parameters explicitly (tests, oracle, replay) and (ii) a randomised entry drawing them from an `Rng` in the documented order; `shake()` first draws `uniform_index(4)` (uniform among the four, D6) and, if the drawn shake is not applicable, returns `NotApplicable` (no redraw, no substitution; M7 decides).

## 4. The four contracts

| Shake | Parameters and draw order | Transformation |
|---|---|---|
| **S1 Random_Reverse** | two distinct interior indices, unordered pair uniform: $x=\texttt{uniform\_index}(m)$, $y=\texttt{uniform\_index}(m-1)$, $y\mathrel{+}=1$ if $y\ge x$, $a=\min$, $b=\max+1$ (1-based); `NotApplicable` if $m<2$ | $s_a..s_b$ reversed in place (both selected nodes included) |
| **S2 Random_Exchange** | $L=\texttt{uniform\_int}(1,\min(L_{\max},\lfloor m/2\rfloor))$; then one `uniform_index(K)` over the $K$ pairs of non-overlapping segments $[a,a+L-1],[b,b+L-1]$, $a+L\le b\le m-L+1$, ranked by $(a,b)$ ascending (adjacent segments allowed); `NotApplicable` if $m<2$ | the two segments exchange places (each keeps its internal order) |
| **S3 Random_Shift** | $L=\texttt{uniform\_int}(1,\min(L_{\max},m-1))$; $a=\texttt{uniform\_int}(1,m-L+1)$; the possible directions (backward if $a>1$, forward if $a+L-1<m$, in this order) with `uniform_index`; distance $d=\texttt{uniform\_int}(1,\min(D_{\max},\text{room}))$; `NotApplicable` if $m<2$ | the segment moves $d$ nodes forward (towards the end depot) or backward, the $d$ jumped nodes closing the gap; always a real change |
| **S4 Random_Drone_Remove** | the drone id: `uniform_index` over the ascending list of ids that fly at least one trip; `NotApplicable` if there is no trip | every trip of that drone is removed (the shake's own effect, recorded as *removed*, not as *dissolved*); its customers go back to the truck as above; the route skeleton is otherwise unchanged; idle FLPs dropped; no other trip is touched (they can only be dissolved by the no-free-gap fallback) |

$L_{\max}$ = `ShakeOptions::max_segment_length` and $D_{\max}$ = `max_shift`, **default 3 each** (assumption A22).

## 5. What the contracts rest on

| Class | Item |
|---|---|
| **Paper fact** | the four shakes' one-line definitions and the uniform "randomly selects" (section 1); the segments are truck-route segments; Random_Drone_Remove returns the customers to the truck |
| **Paper silence** | segment lengths, "several nodes", depot selectability, placement of returned customers, fate of trips and FLPs on moved segments, feasibility of the output, selection probabilities |
| **Approved decision** | D6 (uniform choice; dissolve only invalid trips; count and record reasons; never silently repair), Issue 4 (drone ids), explicit FLP drop, item 15 (no re-anchoring), strict `<`, evaluator is the judge |
| **Implementation assumption** | **A19** segments and "nodes" are the interior truck-served customers; depots fixed; FLPs are not nodes and follow their arcs. **A20** Random_Reverse picks an unordered pair uniformly. **A21** Random_Exchange: $L$ uniform in $1..\min(L_{\max},\lfloor m/2\rfloor)$, a uniformly drawn non-overlapping pair (adjacent allowed). **A22** Random_Shift: $L$, start, feasible direction, distance $1..\min(D_{\max},\text{room})$ uniform; defaults $L_{\max}=D_{\max}=3$ ("several"), configurable (`ShakeOptions`, D7). **A23** Random_Drone_Remove picks uniformly among drone ids with at least one trip. **A24** FLPs follow their arcs: a surviving arc keeps its FLPs and trips, a vanished arc's FLPs leave the route and its trips are dissolved (no re-anchoring). **A25** invalid = R1 or R2 only; endurance/(28)/fleet are the evaluator's verdict and the (possibly infeasible) result is returned unrepaired. **A26** returned customers: uniform random free gap; fallback gap with `ArcSplitByReinsertion` dissolution when none is free. **A27** `shake()`: uniform among the four, `NotApplicable` without redraw. **A28** the draw orders of section 4 (and one `uniform_index` per reinsertion) are part of the contract; the caller owns the `Rng` (M7 derives it from the `Shake` seed stream). **A29** the input must be evaluator-feasible (`InputInfeasible` otherwise). |
| **Deviation from the paper** | none identified (Random_Drone_Remove's customer placement and the handling of invalid trips are paper silences) |
| **Unresolved** | section 9 |

## 6. Implementation

`include/search/shakes.hpp` (new, header-only, additive: no existing file changed for M6 except `CMakeLists.txt` entries). Entry points: `shake_reverse/exchange/shift/drone_remove` (explicit parameters, deterministic cores), `shake_random_*` (draw the parameters in the order of section 4), `shake()` (uniform kind). `ShakeResult` carries the solution, the drawn parameters (`seg_a/seg_b/seg_len/shift/drone`), `dissolved` (trip index in the input, drone id, customer, launch, recovery, **reason**), `removed` (S4's own removals), `preserved` (input indices, ascending), `flps_lost_with_arcs`, `flps_dropped`, `reinsertions` (customer, anchor node, number of free gaps, fallback flag) and the evaluator's judgement (`evaluated/feasible/objective/base_objective/reason`; `dissolved_with(reason)` counts). Common back end `detail_shake::settle`: dissolve (R1/R2) or remove (S4) -> drop idle FLPs -> reinsert returned customers (free gap, else fallback) -> judge. Route shakes use `std::reverse`, `std::swap_ranges`, `std::rotate` on the interior and `rebuild` (FLPs follow arcs). Neither the evaluator nor any operator, helper or the frozen reference was changed (15 guard-file sha256 values identical to the M5 end).

## 7. Validation

| Check | Result |
|---|---|
| `test_shakes` (hand-derived fixtures on a line: Q, P, F; parameter validation; statuses; options; random-parameter replay and coverage; `shake()` distribution) | **675 assertions, 0 failures** (`shakes_unit_tests_raw.txt`) |
| `test_shakes_oracle` (80 seeded instances, 10 bases each incl. random-walk bases, 12 seeds x 4 shakes per base; independent re-implementation, exact comparison of route, trips, records, RNG consumption; frozen-reference verdicts; second full run for the digest) | **728,109 checks, 0 failures**; non-vacuity checks passed (`shakes_oracle_stats_raw.txt`) |
| Mutation (`tests/mutation/m6_mutate.py`, 71 single-bug mutants of `shakes.hpp`) | **70 killed, 1 equivalent survivor** (D6-03: searching for the recovery from the launch position instead of one after it is identical because the launch FLP can never be the recovery node) (`m6_mutation_results_raw.txt`) |
| Fresh Release build, `-O3 -DNDEBUG -Wall -Wextra -Wpedantic` | 0 warnings; ctest **19 of 19** (`m6_release_ctest_raw.txt`) |
| Fresh ASan + UBSan (`-fsanitize=address,undefined`, no-recover) | `test_shakes`, `test_shakes_oracle` 0 reports (`m6_sanitizer_raw.txt`) |
| Determinism | oracle output byte-identical over 3 runs; unit tests and the benchmark's outcome lines byte-identical over 2 runs; digest equal on a full second oracle run |
| Regression | N1--N8 benchmark counts (generated, evaluated, feasible, rejected by reason) identical to the M5-end files (`*_after_m6_raw.txt`); evaluator benchmark equivalence `yes`, speed within noise; all earlier tests unchanged and passing |

Worked fixtures confirm the contracts by hand: Q (reverse(1,2): the surviving arc (4,5) keeps its FLP and trip, objective 2025 -> 3025; reverse(1,5): arc becomes (5,4), trip dissolved `LaunchArcVanished`, FLP lost with its arc), P (an exchange producing both `RecoveryNotAfterLaunch` for a trip whose launch arc survives and `LaunchArcVanished`; S4 removal with explicit idle-FLP drop), F (no free gap: fallback draw, `ArcSplitByReinsertion`, 40 replayed seeds, both gaps drawn).

## 8. Measurements (Release; `bench_shakes`, base seed 31415, initial solution, window 3, 4000 seeded draws per shake; raw: `shakes_bench_raw.txt`, `shakes_bench_endurance_x3_raw.txt`)

* **Cost per call** (construction only / with the result's evaluator verdict; each call also evaluates the input once to confirm it is feasible): n = 10: 4.0 / 5.2--5.4 us (Drone_Remove 3.8 / 4.6); n = 20: 7--7.5 / 10--10.5 us (Drone_Remove 7.5 / 9.3); n = 50: 25--28.5 / 33--38 us. The evaluator share of a full call is 13--33 %; the evaluator costs 1.2 / 2.4 / 9.2 us per call at these sizes. 180--910 allocations per call (rebuilt routes, queues, records), growing with n.
* **Outcomes on the initial solution** (all trips fly on drone 0, so Random_Drone_Remove removes every trip): every applicable draw (12 of 12 shake/size combinations, 4000 draws each) was evaluator-feasible, also at endurance x 3. Mean objective change of the feasible results is strongly positive and none improves the base in more than 0.7 % of draws, i.e. shakes from the initial solution are perturbations, not improvements. Trips dissolved per shake: Reverse 1.5 / 2.4 / 9.3 of 3 / 6 / 23 trips (n = 10 / 20 / 50), Exchange 1.2 / 1.5 / 3.3, Shift 1.0 / 1.1 / 2.5; `RecoveryNotAfterLaunch` and `ArcSplitByReinsertion` do not occur there.
* **Random-walk bases** (oracle campaign, 35,796 applicable-and-judged shakes of 38,400 calls; 2,604 Drone_Remove calls were `NotApplicable`): dissolved by reason `LaunchArcVanished` 25,278, `RecoveryNotAfterLaunch` 85, `ArcSplitByReinsertion` 8; S4 removed 17,517 trips; 42,888 reinsertions, only 8 needed the no-free-gap fallback; 156 results (0.4 %) were evaluator-infeasible, 133 of them endurance. About 40 % of the shakes dissolved or removed nothing.
* **Reading:** a route shake almost always breaks the arcs of the trips launched near the moved nodes, so most dissolutions are `LaunchArcVanished`; keeping trips whose arc survives (D6) is what makes small shakes small. The recovery-order reason and the fallback are rare but are exercised by the tests. Shakes do not use the objective, so the calibration of $L_{\max}$, $D_{\max}$ and the shake share belongs to M7 (D7).

## 9. Open points and assumptions requiring approval

A19--A29 (section 5) are implementation assumptions (approved at the M6 sign-off, status line above). Points the paper does not decide and the code makes visible rather than hides:
1. **Segment sizes and "several nodes"** ($L_{\max}=D_{\max}=3$ default, configurable): calibration is M7.
2. **FLP fate under route shakes** (A24): FLPs follow their arcs and a vanished arc's trips are dissolved; the alternative "re-anchor launches" would be N4-style repair and contradicts item 15.
3. **Placement of returned customers** (A26): uniform free gap; the fallback dissolves further trips (counted, 8 in 42,888 reinsertions on the random-walk bases).
4. **Infeasible outputs** (A25): returned as judged with reasons; whether M7 retries, rejects or accepts them is M7's decision.
5. **Non-applicable shakes** (A27): `NotApplicable`, no redraw; M7 decides the policy (e.g., all-truck solutions and Random_Drone_Remove).
6. **Selection probabilities**: uniform among the four (D6), not otherwise specified by the paper.
7. (Historical, written before the sign-off.) M6 was signed off on 9 Nov 2026, 13:56 IST; the VNS driver (M7) and later milestones are complete.
