# Neighbourhoods N4 and N5 (M2): contracts, interpretations, open questions (updated by the M2 follow-up, 9 Nov 2026)

Code: `include/search/move_types.hpp` (shared types), `neighbourhood_n4.hpp`, `neighbourhood_n5.hpp`; helpers added to `route_ops.hpp` (`flp_insertion_position`) and `candidate_window.hpp` (`gaps_within_window`).
Tests: `tests/test_n4_n5.cpp` (hand-derived), `tests/test_n4_n5_oracle.cpp` (brute-force oracle). Evidence: `test_and_validation.md` §6.

Every statement below is tagged **[PAPER]** (stated by Han, Chi & Sun 2026), **[DECISION]** (approved project decision, `DECISIONS.md` D1--D10 and the paper decision register), **[ASSUMPTION]** (implementation interpretation, not stated by the paper),
**[DEVIATION]** (departs from the paper's text) or **[OPEN]** (needs a user decision). There is no [DEVIATION] in M2. The one policy choice that mattered (what N4 does with the trips it disturbs) was decided in the M2 follow-up: **conservative re-anchoring** (§6, assumption A6); the original *reject* policy remains available (`AffectedTripPolicy::Reject`).

## 1. What the paper says (and does not say)

* **[PAPER]** Sec. 5.3: all eight neighbourhoods pick "the most optimal" candidate by the updated solution cost (customer waiting time). Local-search order (Sec. 6.1): N4 first, N5 second.
* **[PAPER]** N4 "truck service node relocate": repositions a truck service node; "searching adjacent customer node of random truck service node". Fig. 10d is captioned as nodes C2 and C4 *exchanged* in the truck route.
* **[PAPER]** N5 "T-D service node swap": swaps a truck service node with a drone service node; "randomly selecting a drone trip and searching adjacent truck service nodes". Fig. 11a: C2 and C3 exchanged.
* **Not stated:** how many nodes "adjacent" covers; whether N4 relocates, exchanges or both; where the newly drone-served customer of N5 is launched and recovered; what happens to trips whose FLP or recovery node the move disturbs. (The decision register records the last two as "defer until the affected neighbourhood".)

## 2. Approved decisions used

* **[DECISION] D1** adjacency window 3, configurable (`SearchContext::adjacency_window`), counted on non-FLP skeleton nodes via half-steps: a truck customer at skeleton index $s$ has $h = 2s$; a drone customer launched in gap $g$ has $h = 2g+1$; gap $g$ has $h = 2g+1$; two positions are adjacent iff $|\Delta h| \le 2w$.
  This is an *implementation interpretation* of "adjacent", not a paper fact.
* **[DECISION] D4** N5 re-selection = best valid FLP on the resulting route's arcs within the window; recovery at the next truck node.
* **[DECISION] I-7** FLP-only launches (no customer or depot launches). **FLP pool eager**, no new id scheme: `FlpArcIndex` only indexes the existing pool.
* **[DECISION] Phase-G "report, don't repair"**: operators never decide feasibility themselves beyond the structural affected-trip check, never repair silently, never mutate their input (copy-and-modify).

## 3. Shared contract (`OperatorResult`)

| Aspect | Behaviour |
|---|---|
| Input | an **evaluator-feasible** `Solution` (checked first; otherwise `status = InputInfeasible`, nothing generated), a `SearchContext` (full eager network, `CustomerData`, FLP ownership data, `FlpArcIndex`, window) and an anchor |
| Output | `best` = lowest-objective evaluator-feasible candidate (**strict `<`, no epsilon; the first generated wins ties**), counters `generated / evaluated / accepted / duplicates_skipped`, `rejected[reason]`, N4 re-anchoring counters `stale_launch_candidates / reanchored_candidates / trips_reanchored / variants_generated / cap_reached_candidates`, optional per-candidate `log` (each record also carries `combinations_total` and `cap_reached`) |
| Counting | `generated`, `accepted` and every `rejected[reason]` count **candidates** (moves); every generated candidate is accepted or has exactly **one** rejection reason, so `accepted + rejected_total() == generated` (asserted by the oracle). `evaluated` counts **evaluator calls**, which exceeds `generated` when a candidate is judged through several re-anchoring variants. For a candidate whose variants are all rejected by the evaluator the reason of its first variant is reported. |
| Mutation | none: the input is `const`; every candidate is a copy |
| Report | `RejectReason`: `AffectedTrips` (policy Reject, and N5), `NoFreeDrone`, `NoReplacementFlp`, `InvalidRecovery` (N4 Reanchor), `EvalEndurance`, `EvalFleetCapacity`, `EvalDroneSequencing`, `EvalTripOrder`, `EvalFlpArc`, `EvalFlpUnused`, `EvalOther`. The evaluator's verdict is only *classified*, never overridden. `EvalFlpArc` / `EvalFlpUnused` indicate an operator construction bug and must stay 0 (asserted over every candidate of the oracle campaign). |
| Options | `record_candidates` (keep every candidate; off in search), `evaluate_candidates` (false = construction + structural checks only; used to time generation separately). N4 only: `N4Options::mode`, `affected_trips` (`Reanchor` default, `Reject`), `max_reanchor_variants` (**search budget per candidate**, default **100**, configurable; 0 = unlimited, i.e. exhaustive). `N5Options` is unchanged. |
| Randomness | the deterministic overloads take the anchor; the `Rng` overloads draw **one** uniform index (N4: over interior truck customers in route order; N5: over trip indices) and then run the deterministic overload. Same seed, same result (tested) |
| Allocation | no full-registry scan: customer ids come from the P1 cache, FLP candidates from `FlpArcIndex::flps_on_arc` |

## 4. N4 -- truck service node relocate

Anchor: a truck-served interior customer $c$ at skeleton index $s_c$. Candidates, in generation order:

1. **Relocate** [ASSUMPTION A1]: $c$ is re-inserted into every gap $g$ of the *original* skeleton with $|2g+1-2s_c| \le 2w$, ascending, except the two gaps that touch $c$ (that would reproduce the input). It lands after the FLPs of that gap, directly before skeleton node $g+1$.
   Gap mapping after removal: $g \le s_c-2 \to g$, $g \ge s_c+1 \to g-1$.
2. **Exchange** [ASSUMPTION A1]: $c$ is exchanged with every other *truck-served* customer within the window, nearest first, ties by id. Raw route positions are swapped; FLP entries do not move. (Exchanging with a drone customer is N5.)
3. Identical routes are generated once (`duplicates_skipped`). `N4Mode` selects relocate, exchange or both (default both).

Trips: none is created or deleted and no drone id, drone customer or recovery node is ever changed. What happens to a trip whose launch FLP the move leaves on a vanished arc depends on `N4Options::affected_trips`:
**Reanchor** (default, §6) re-anchors only that launch; **Reject** (the original M2 policy) rejects the candidate as `AffectedTrips` without evaluating it. All remaining feasibility (launch-before-recovery order, (28), $|U|$, endurance, synchronisation) is the evaluator's verdict on the complete candidate.
Exactly-once service holds structurally (a permutation of truck-route entries).

**[ASSUMPTION A1 --- approved 9 Nov 2026]** **[PAPER]** the text of N4 describes relocation; its example (Fig. 10d) is an exchange of two truck nodes. Supporting both is an implementation interpretation, not a claim that the paper specifies two independent neighbourhoods. Both are searched (`N4Mode::RelocateAndExchange`, default; `Relocate` / `Exchange` select one); generation order and tie-breaking are unchanged by the follow-up.
**[ASSUMPTION A2]** ties are broken by generation order, with strict `<` on the objective and no tolerance (see §7).

## 5. N5 -- T-D service node swap

Anchor: trip index $i$ with trip $(f, d, k)$ ($d$ = drone customer, $f$ = launch FLP, $k$ = recovery). Partners: truck-served customers $t$ with $|h_d - h_t| \le 2w$, nearest first, ties by id. Per partner $t$:

1. $d$ takes $t$'s slot in the truck route (same raw position, skeleton indices unchanged); $t$ leaves the route.
2. Trip $i$ is **dissolved**; its launch FLP $f$ is removed from the route iff no other trip launches from it. This is explicit and recorded (`flps_dropped`), because rule (6) would otherwise reject the candidate.
3. **Affected trips:** any *other* trip whose launch FLP is now on a foreign arc (the two gaps that touched $t$'s slot changed their endpoint) or that recovered at $t$ makes the whole swap `AffectedTrips` (one rejected candidate, trips named). Nothing is repaired, no trip is silently kept or dissolved.
4. **New trip for $t$** [DECISION D4]: for each gap $g$ of the resulting skeleton with $|2g+1-2s_t| \le 2w$ and each FLP $f'$ of that arc $(x,y)$ in ratio order, the trip is $(f', t, y)$ with recovery at $y$, **the next truck node**. $f'$ is inserted in its gap in ratio order, or reused if it is already on the route (a second launch there).
   The drone is the lowest id free over the trip's route-position interval (`lowest_free_drone_for_trip`); other trips keep their ids; none free gives `NoFreeDrone` (not evaluated).
5. The candidate is evaluated; endurance, SL/SR timing, (28), $|U|$, FLP arcs and exactly-once service are the evaluator's verdict.

**[ASSUMPTION A3 --- approved 9 Nov 2026]** The launch window is centred on $t$'s former skeleton position (the swapped truck customer's former position, the customer that needs a launch). D4 says "within the configured window" without naming the centre; the choice is an implementation assumption, approved, and N5 is otherwise unchanged by the follow-up. A mutation that shifts the centre by one skeleton step is detected by the tests, so switching to another centre is a one-line change.
**[ASSUMPTION A4]** ties are broken by generation order (partner, gap ascending, FLP ratio ascending).
**[ASSUMPTION A5]** N5 does not re-optimise the recovery: it is always the next truck node (D4). Recovery at the depot is allowed when the arc ends at the depot.

## 6. N4 affected trips: conservative re-anchoring (assumption A6) --- RESOLVED 9 Nov 2026

**[PAPER]** defines re-anchoring only for the one new trip of N5/N7 (D4) and dissolution only for shakes (D6); it says nothing about the *other* trips an N4 move disturbs.
**[DECISION, approved in the M2 follow-up, recorded as implementation assumption A6]** N4 re-anchors the trips *directly* affected by its route change, conservatively; it is not general repair. Recovery relationships stay unchanged and invalid ones cause rejection.
**[IMPLEMENTATION ASSUMPTION A6b, approved 9 Nov 2026 as an empirical implementation choice]** the default joint-combination budget is **100**; it is not a paper parameter, and the paper specifies neither this re-anchoring rule nor a combination cap.
**[LIMITATION, not a paper deviation]** a finite budget makes the N4 re-anchoring search budget-bounded and potentially incomplete: it may miss the best candidate that exhaustive enumeration would find. M2 is therefore a faithful implementation of the approved rules, **not** an exact reproduction of every paper detail (N4's treatment of disturbed trips and its search budget are the project's own).

### 6.1 Representation (verified before implementing)

`DroneTrip{drone_id, launch_node, service_customer, recovery_node}`. The **launch** is an FLP id; an FLP is a route entry that owns one arc $(a,b)$ (`FlexibleLaunchPointData`) and is valid only between the non-FLP route nodes $a$ and $b$. The **recovery** is a customer or depot node found at a route position strictly after the launch's position (the evaluator's own search, `find_route_position_from(route, recovery, launch_pos+1)`).
An N4 move permutes truck nodes and never touches FLP entries, so it can invalidate **both** relationships: (i) the FLP now sits between other nodes (stale launch); (ii) the recovery node can move to a position before the launch FLP (invalid recovery). Exploration on the oracle bases showed that (ii) occurs essentially only together with (i), but is independent of it: replacing the launch cannot repair it.

### 6.2 Exact semantics (`AffectedTripPolicy::Reanchor`, the default)

For every generated N4 candidate (route built exactly as before; the trips are copied unchanged):

1. **Invalid recovery first.** If any trip's recovery node does not occur after its launch FLP in the candidate route, the candidate is rejected as `InvalidRecovery` (trips named), **not evaluated, not repaired**; the record shows the route and trips exactly as the move produced them. (Replacing the launch FLP does not make the recovery valid again, so no variant is tried.)
2. **Nothing stale:** the candidate is judged by the evaluator as before (no re-anchoring, trips verbatim).
3. **Stale trips** = trips whose launch FLP lies between non-FLP nodes $(x,y)$ that are not its own arc; ordered by trip index. For each, the **resulting arc** is $(x,y)$ around the stale FLP's current position (radius 0: only that arc; the candidate window is *not* widened, assumption A6). Its **options** are `FlpArcIndex::flps_on_arc(x,y)` in ratio order --- the N5 convention (FLPs of an arc, ratio ascending, evaluator-judged, strict `<`, first wins) --- so no second ranking rule exists.
4. Any stale trip without options $\Rightarrow$ `NoReplacementFlp` (trips named, not evaluated, trips untouched --- a trip is **never** silently dissolved).
5. **Joint enumeration, budget-bounded.** The combinations are the Cartesian product of the stale trips' options (stale trips in index order, the last stale trip varies fastest, options in ratio order): a deterministic odometer. Let $K=\prod_k |L_k|$ (saturating at the maximum `size_t`) and let the **budget** be `max_reanchor_variants` (default **100**, configurable; 0 = unlimited). The enumeration generates combinations in that order and **stops after the budget is spent**; only the combinations reached are evaluated. `cap_reached` $\Leftrightarrow K>$ budget (a budget equal to $K$ is *not* reached; the search is then exhaustive). Nothing is rejected because of the cap: the candidate is still evaluated and receives its one outcome. Each variant: the candidate route without the stale FLPs, plus the chosen FLPs inserted into the gap of their arc in ratio order (an FLP already on the route is reused --- two launches from one FLP, as in N5); the stale trips' `launch_node` set to the choice; **everything else unchanged**. Each variant is judged by the complete evaluator.
6. The candidate's objective is its **best feasible variant among those reached** (strict `<`; the first variant in enumeration order wins ties); if no reached variant is feasible, the candidate is rejected with the evaluator reason of its first variant. Because the choice is made jointly, two stale trips may select the same FLP when that is cheapest. **Limitation:** when `cap_reached` is true the result is the best of a *prefix* of the enumeration; a feasible or cheaper combination after the budget is not seen, so the search is budget-bounded and potentially incomplete. It must not be described as exhaustive. When `cap_reached` is false for every candidate (`cap_reached_candidates == 0`) the sweep was exhaustive.

**Preserved always:** drone customer, drone id, recovery node of every trip; launch of every unaffected trip; the input solution; exactly-once coverage. **Counters:** a candidate is counted once (accepted or exactly one reason); `variants_generated`, `evaluated` count variants / evaluator calls.
After re-anchoring a trip's recovery is no longer necessarily the next truck node (it is preserved, as approved); the evaluator's endurance and synchronisation rules judge the longer flight.

### 6.3 Cases intentionally rejected

| Case | Reason reported | Why not repaired |
|---|---|---|
| a recovery node ends up before (or at) its launch FLP | `InvalidRecovery` | re-selecting a recovery is not an approved rule; replacing the launch does not fix it |
| a stale trip's resulting arc has no FLP (short arc, large $d$) | `NoReplacementFlp` | dissolving the trip or changing its customer is not approved |
| more than `max_reanchor_variants` combinations | **not rejected**: the first `max_reanchor_variants` are evaluated, `cap_reached` is set | the budget bounds the search, it is not a rejection reason (the old `TooManyReanchorings` reason no longer exists) |
| every replacement violates endurance / fleet capacity / (28) / ... | the evaluator's reason of the first variant | feasibility is the evaluator's verdict (e.g. a drone that loiters at a far recovery) |
| input not feasible | `InputInfeasible` (nothing generated) | unchanged from M2 |

### 6.4 Measured effect (production operators, window 3, base = initial solution; `operator_bench_raw.txt` = final, default budget 100; Reject-only baseline `operator_bench_raw_reject_policy_before.txt`; exhaustive-style budget 10,000 run in the same binary)

**The budget trades exhaustive search for bounded runtime.** With the default budget of 100 the N4 search is *budget-bounded and may be incomplete*: a candidate whose joint combination count exceeds 100 is judged on the first 100 combinations only (§6.2). On the fixtures below this lost nothing (identical accepted candidates and identical best per anchor for 100 versus 10,000), but that is an observation on these fixtures, not a guarantee.

| n (trips) | N4 candidates | feasible, Reject | feasible, Reanchor (budget 100 = budget 10,000) | of which re-anchored | still rejected | invalid recovery | no replacement | evaluator | sweep: Reject / budget 100 / budget 10,000 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10 (3) | 45 | 6 (13.3 %) | 8 (17.8 %) | 2 (4.4 %) | 37 (82.2 %) | 11 | 0 | 26 | 40 us / 2.47 ms / 4.76 ms |
| 20 (6) | 110 | 9 (8.2 %) | 27 (24.5 %) | 18 (16.4 %) | 83 (75.5 %) | 38 | 0 | 45 | 128 us / 7.92 ms / 12.19 ms |
| 50 (23) | 246 | 2 (0.8 %) | 5 (2.0 %) | 3 (1.2 %) | 241 (98.0 %) | 169 | 0 | 72 | 638 us / 67.8 ms / 302.1 ms |

Combinations and budget hits (same runs):

| n | stale candidates | combinations generated / evaluator calls, budget 100 | per stale candidate (budget 100) | candidates that hit the budget (budget 100) | combinations generated / evaluator calls, budget 10,000 | per stale candidate (budget 10,000) | hit the budget (budget 10,000) |
|---|---:|---:|---:|---:|---:|---:|---:|
| 10 | 39 | 1,050 / 1,056 | 26.9 | 2 (5.1 % of stale) | 2,002 / 2,008 | 51.3 | 0 |
| 20 | 101 | 2,135 / 2,144 | 21.1 | 8 (7.9 %) | 3,269 / 3,278 | 32.4 | 0 |
| 50 | 244 | 5,303 / 5,305 | 21.7 | 35 (14.3 %) | 23,506 / 23,508 | 96.3 | 0 |

(Combinations are counted per candidate and summed over the sweep; `variants_generated` and the evaluator-call counter differ only by the evaluator calls spent on candidates that are not re-anchored.)
Cap sensitivity (`n4_reanchor_cap_sensitivity_raw.txt`; feasible candidates at n = 20 / n = 50): budget 1: 11 / 2, 10: 21 / 2, 30: 27 / 4, **100 and above: 27 / 5**. n = 50 sweep: budget 1 about 1.6 ms, 10 about 9.8 ms, 30 about 24 ms, 100 about 65 ms, 300 about 137 ms, 10,000 about 280 ms. Budget 100 is the smallest tested value that retains every acceptance on these bases.
Independent oracle campaign (240 bases, n = 6--11, windows 1 and 3): every candidate is checked at the default budget and at budget 4 (so the budget is actually reached; 5,579 budget-reached candidates over both runs) against an independent re-implementation; 0 mismatches.

**Findings:**
(1) The gain is real but modest on trip-rich solutions: at n = 50 the feasible share rises from 0.8 % to 2.0 %, because 69 % of the candidates (169 of 244 stale) also break a recovery relationship, which re-anchoring deliberately does not repair; most of the rest fail the evaluator.
(2) **Measured saving of the budget, corrected.** The approval of the default budget was based on the reject-over-cap semantics of the previous follow-up, where a candidate over the cap was rejected *unevaluated* (n = 50: 28.6 ms versus 347 ms, about 12 x). Under the approved budget-bounded semantics an over-budget candidate is still evaluated on its first 100 combinations, so the saving is smaller: **at n = 50 about 4.5 x (67.8 ms versus 302.1 ms), at n = 20 about 1.5 x, at n = 10 about 1.9 x**, with identical accepted candidates. The cap bounds the per-candidate cost (at most 100 combinations) rather than removing it.
(3) The budget does not change the per-evaluator-call cost (the P1/P2 gains are intact). A cheaper selection (independent per-trip choice, sum instead of product) would change the semantics and is not implemented.
(4) In the oracle campaign evaluator rejections rise sharply under Reanchor: re-anchoring onto a long arc while keeping a far recovery often violates endurance or fleet limits, so most extra candidates that reach the evaluator are rejected there.

## 7. Other items

* **[DECISION, approved 9 Nov 2026] Objective comparison.** Strict `<` on the raw floating-point objective in N4 and N5 (and for the best re-anchoring variant); no epsilon; first-generated wins ties. A configurable improvement threshold may be considered during **M7** (VNS integration); it is *not* authorised now.
* **[FINDING]** N5 sends nearly every constructed candidate to the evaluator (n = 10: 607 of 610) although only ~1 % are feasible, because the launch window is wide and most far FLPs violate endurance. A cheap pre-check would have to reproduce the evaluator's rule exactly, so it was not added; candidate for M7 if profiling says so.
* **[FINDING]** the evaluator reports an out-of-order trip (recovery before launch) as a cascade ("FLP unused (6)", "not reachable in truck-route order", "customer's trip infeasible"). The classifier reports the root cause as `EvalTripOrder` (checked before rule (6)); pinned by a regression test. Under Reanchor the structural `InvalidRecovery` check catches this case before the evaluator; under Reject it still reaches the evaluator.
* **[OPEN, minor]** the defensive branch "an FLP off its arc that no trip launches from is judged as it stands" is unreachable through the public entry points (inputs must be evaluator-feasible, so by rule (6) every FLP launches); it exists so that malformed internal states cannot loop.
