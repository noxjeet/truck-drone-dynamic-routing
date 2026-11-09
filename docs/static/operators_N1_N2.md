# N1 and N2: launch node relocate and recover node relocate (M5)

Status: contracts (sections 1--4) written **before** implementation (9 Nov 2026, M5 instruction of 12:30 IST). Implementation notes, tests, measurements and findings are added after implementation (sections 6--9).
Related: `operators_N3_N6.md`, `operators_N4_N5.md`, `operators_N7_N8.md` (shared contract, D1 window, reject policy), `DECISIONS.md` items 15--53 and the M5 items, `paper1_decision_register.md` Issue 7 (FLP-only launches) and the M5 addendum.

**Sign-off (9 Nov 2026, 13:19 IST):** A13--A18 approved. The omission of customer launch nodes in N1 is an explicitly documented **reproduction limitation**; N1 is not to be expanded to customer launch nodes without further paper evidence.

## 0. Sources and their limits

The original PDF of Han, Chi & Sun (2026) is **not** in the repository or the project files. The paper text used is the page-image-based extraction `claude/han-chi-sun-2026-extraction.md` (quotations near-verbatim) and `docs/paper1_decision_register.md` (Issues 7 and 8). Figs. 10a and 10b are known only through their captions and the register's reading.

## 1. Paper facts

* **N1, launch node relocate** (Sec 5.3, p20--21, Fig. 10a): "repositions the launch node of a drone trip. ... requires searching **adjacent customer nodes and flexible launch nodes** of the drone launch node in original drone trip". Caption: the launch is moved "from truck service node C1 to the flexible node belonging to the C1--Depot segment, reducing the waiting time for customer C2".
* **N2, recover node relocate** (p21, Fig. 10b): "repositions the recover node ... searching **adjacent customer nodes** of the drone recover node in original drone trip". Caption: the recovery moves from truck service node C3 to C4, "avoiding the time consumption of truck and drone merging at node C3".
* Every neighbourhood selects "the most optimal" candidate by the updated customer waiting time (the objective is unchanged).
* Execution (Sec 6.1): "Given the existence of multiple flexible launch nodes in the truck route, launch node relocate and recover node relocate neighborhood operations are performed **several times** during the local search process". N1 and N2 are *not* in the six-step order; their repetition and placement are VNS-integration matters (D2: twice per pass; milestone M7, not implemented here).
* The model (Assumption 4, condition 1, constraints (5)(6)(11)(17)(19)(21), Table 1): drones launch **only from flexible launch points**; recovery is at a truck-served customer or the end depot (`has_valid_node_roles`), strictly after the launch on the truck route.
* The paper does **not** say: what "adjacent" means numerically, whether the search is in both directions, what happens to the old launch FLP, whether the drone identity may change, or how N1's "adjacent customer nodes" is reconciled with Assumption 4.

## 2. Existing decisions that apply

| ID | Content | Used by |
|---|---|---|
| Register Issue 7 (FLP-only; the evaluator and `has_valid_node_roles` enforce it) | launches only at FLPs; the N1 text's "adjacent customer nodes" contradicts Assumption 4 and is **not implemented**: N1 candidates are FLPs only; Fig. 10a's customer launch is read as an FLP adjacent to that customer | N1 |
| D1 | adjacency window $w=3$, configurable (`SearchContext::adjacency_window`), counted in skeleton (non-FLP) nodes via half-step coordinates; an implementation interpretation | N1, N2 |
| D2 | N1 and N2 are repeated twice per pass (VNS integration, M7) | not implemented here |
| item 15, M3 rule | no invented repair; disturbed trips are rejected and named; re-anchoring is N4-only | both |
| N4/N5/N8 convention | an FLP left with no trip is dropped **explicitly** and counted (`flps_dropped`); an FLP already on the route may be reused as a second launch | N1 |
| -- | objective = sum of customer arrival times, end-depot return excluded; strict `<`, first generated wins ties, no epsilon; FLP pool eager; drone identities and the endurance/fleet/sequencing rules are the evaluator's | both |

## 3. N1 contract (launch node relocate)

**Transformation.** One trip's launch FLP is replaced by another FLP of the pool: $(u,f,c,k)\to(u,f',c,k)$. The drone id $u$, the customer $c$, the recovery node $k$, the truck skeleton (all customers and the depot, their order), every other trip and the trip order stay as they are. **Changes:** `launch_node` of one trip, and the route's FLP entries (the new FLP is inserted into its gap, the old one dropped if unused).

* **Input:** an evaluator-feasible `Solution` and an anchor trip index $i$. Randomised overload: one uniform draw over the trip indices (as N3/N6/N8). Index out of range $\Rightarrow$ `InvalidAnchor`; infeasible input $\Rightarrow$ `InputInfeasible`; no trips $\Rightarrow$ `NoEligibleAnchor` (Rng overload).
* **Candidate set (A13).** Let $g_0$ be the skeleton gap that holds the trip's current launch FLP. The eligible gaps are those with $|g-g_0|\le w$ (half-step coordinate $2g+1$ within $2w$ of $2g_0+1$: the same coordinate a drone customer launched there has). The candidates are **every FLP of every eligible arc**, in gap-ascending then arc-position (ratio) order, **excluding the current launch FLP**: the other FLPs of the current arc, FLPs not yet on the route (inserted in ratio order, `flp_insertion_position`), and FLPs already on the route that other trips use (a second launch at that FLP).
* **The old launch FLP.** Dropped from the route iff no other trip launches from it (explicit, counted in `flps_dropped`; rule (6)); kept if another trip uses it.
* **Recovery relationship (A15).** If the new launch does not lie before the trip's recovery node on the route (the recovery's skeleton index is not after the new launch's gap), the candidate is **generated and rejected as `InvalidRecovery`**, not evaluated and not repaired (the same reason and meaning as in N4). The recovery node is never changed.
* **Feasibility.** The evaluator's verdict alone: endurance (36)/(37) with the new flight legs, SL/SR timing, (28) drone sequencing, fleet capacity, rule (6). The drone id is **not** re-chosen (A16): a violated (28) or fleet limit is `EvalDroneSequencing` / `EvalFleetCapacity`, never silently fixed by moving the trip to another drone.
* **No alternative FLP in the window** $\Rightarrow$ zero candidates, status `Ok` (N1 is not forced to produce a move, unlike N7's "must become drone-served").
* **Order / ties / duplicates:** gaps ascending, FLPs by ratio then id; strict `<`, first generated wins; no duplicates are possible (`duplicates_skipped == 0`).
* **Reasons that can occur:** `InvalidRecovery`, `EvalEndurance`, `EvalDroneSequencing`, `EvalFleetCapacity`, `EvalTripOrder`/`EvalOther` (argued unreachable or rare). `AffectedTrips`, `NoFreeDrone`, `NoReplacementFlp`, `EvalFlpArc`, `EvalFlpUnused` cannot occur: no skeleton node moves, so no other FLP leaves its arc; the new FLP is built on its own arc; the old FLP is dropped exactly when unused.

## 4. N2 contract (recover node relocate)

**Transformation.** One trip's recovery node is replaced by another truck node: $(u,f,c,k)\to(u,f,c,k')$. The route, every FLP, the drone id, the customer, the launch FLP, every other trip stay as they are. **Changes:** `recovery_node` of one trip only. Nothing is dropped or inserted.

* **Input, errors, Rng overload:** as N1.
* **Candidate set (A14).** Let $s_k$ be the skeleton index of the trip's current recovery (the end depot has the last index; the start depot, which shares the depot's node id, is never a recovery position). The candidates are the skeleton nodes with $|s-s_k|\le w$ (half-step $2s$ within $2w$ of $2s_k$), $s\ge1$, $s\neq s_k$: truck-served customers and the end depot, **ascending skeleton index**. (The paper's "adjacent customer nodes"; the end depot is a legal recovery node of the model.)
* **Recovery relationship (A15).** A candidate that does not lie after the trip's launch FLP (skeleton index not after the launch's gap) is **generated and rejected as `InvalidRecovery`**, not evaluated and not repaired.
* **Feasibility, drone id, order, ties, duplicates:** as N1 (evaluator alone; drone id unchanged; ascending skeleton index; strict `<`, first wins; `duplicates_skipped == 0`).
* **No node in the window** $\Rightarrow$ zero candidates, status `Ok`.
* **Reasons that can occur:** `InvalidRecovery`, `EvalEndurance`, `EvalDroneSequencing`, `EvalFleetCapacity`, `EvalOther`. The others cannot occur (the route is untouched).

## 5. What the contracts rest on

| Class | Item |
|---|---|
| **Paper fact** | the quoted descriptions and captions (section 1); both operators relocate one node of one drone trip and select the best; N1/N2 are run "several times" |
| **Paper silence** | the meaning of "adjacent" (size, direction); the fate of the old launch FLP; the drone identity; the handling of an invalid launch/recovery order |
| **Existing decision** | Issue 7 FLP-only (so N1's customer candidates are dropped); D1 window; D2 repetition (VNS, later); item 15 / no repair; explicit FLP drop; strict `<`/first wins; eager pool |
| **Implementation assumption** | **A13** N1 candidate set = every FLP on the arcs of the D1 window around the old launch's gap (excluding it, shared and unrouted FLPs included). **A14** N2 candidate set = skeleton nodes (customers and end depot) within $w$ of the current recovery, ascending. **A15** a launch/recovery that breaks the "recovery after launch" relationship is rejected as `InvalidRecovery` (named, unevaluated, unrepaired). **A16** the drone id never changes. **A17** the old launch FLP is dropped iff unused. **A18** the `Rng` overloads draw once with `uniform_index(#trips)`. Zero candidates give status `Ok`. |
| **Deviation from the paper** | N1 does not consider customers as launch nodes ("adjacent customer nodes"): FLP-only, the existing Issue 7 decision. Identified, not new. |
| **Unresolved** | section 9 |

## 6. Implementation (files)

* `include/search/trip_relocate.hpp` (new, `detail_relocate`): the shared step. `locate()` finds, on the **base** route, the skeleton, the launch gap $g_0$ and the recovery's skeleton index; `reject_invalid_recovery()` counts and records an `InvalidRecovery` candidate (named trip, not evaluated, shown unrepaired); `judge()` evaluates, classifies with `classify_evaluation` (the evaluator's verdict is never overridden), counts and keeps the best by strict `<`. It reuses `detail_exchange::begin` (base evaluation, anchor check) from M4.
* `include/search/neighbourhood_n1.hpp` (new): for each gap $g$ of the D1 window around $g_0$ (`gaps_within_window`, coordinate $2g+1$), for each FLP of the arc (`flps_on_arc`, ratio then id) except the current launch: copy the base, insert the FLP in ratio order if it is not on the route (`flp_insertion_position`), set the trip's launch, drop the old FLP iff `trips_launching_from` is empty (`flps_dropped`), then `InvalidRecovery` if the recovery's skeleton index is below $g+1$, else `judge`.
* `include/search/neighbourhood_n2.hpp` (new): for $s=\max(1,s_k-w)\dots\min(\mathrm{last},s_k+w)$, $s\ne s_k$: set `recovery_node = skeleton[s]`; `InvalidRecovery` if $s<g_0+1$, else `judge`. The route is untouched.
* `include/search/move_types.hpp` (additive): `OperatorId::{N1,N2}` (the enum is now N1..N8), `MoveKind::{N1LaunchRelocate,N2RecoveryRelocate}`, `Move::replaced_node` (the old launch for N1, the old recovery for N2). `kRejectReasonCount` is still 12.
* No other production file changed. `CMakeLists.txt` gains `test_n1_n2`, `test_n1_n2_oracle` and (with `TDR_BUILD_BENCH`) `bench_n1_n2`.

Decision traceability: all reasoning about *why* the candidate set is what it is stays in sections 1--5; the code adds no further behaviour.

## 7. Tests

**`tests/test_n1_n2.cpp`: 419 assertions.** Hand-derived fixtures on a line (truck 10 m/s, drone 20 m/s, SL = SR = 0, service times 0; every expected objective and endurance value was derived by hand **before** the first run and then matched exactly, to $10^{-9}$, by both the production evaluator and the frozen reference):

| Block | Fixture | Establishes |
|---|---|---|
| N1-A | n=4, one trip, launch F3000 | window $w=0,1,3,9$: candidates, order (gap ascending, ratio order), objectives 975, 1075, 925, 875; the arc after the recovery gives 6 `InvalidRecovery`; the old launch is dropped and reported; the depot arc is a legal gap; clipping at the route; `evaluate_candidates=false`, `record_candidates=false`, determinism, input unchanged |
| N1-B | same, $e=250 / 249.99 / 200$ | endurance (36) is inclusive ($=250$ accepted, $249.99$ rejected, best-effort objective still carried by a rejected record) |
| N1-C | n=5, two drones, **shared** launch F3000 | an FLP used by another trip is *not* dropped; new FLP inserted in ratio order next to it; both anchors |
| N1-D | n=5 (skeleton 0,1,3,5,0), trips in vector order [T2,T1] | launching from another trip's FLP (shared target); `EvalDroneSequencing` (28) with one drone for both trips, `EvalFleetCapacity` with $|U|=1$; the drone id is **not** re-chosen; anchor = vector index, not route order |
| N1-E | launch arc with a single FLP | $w=0$ gives zero candidates, status `Ok` |
| N2-A | n=5, drone 10 m/s | $w=0$ none; $w=1$: before-launch node `InvalidRecovery`; $w=3$: C1, C2 invalid, C5 and the end depot both 1600 (exact tie, **first generated (C5) wins**), the start depot is never a candidate; route untouched; flags; determinism |
| N2-B | $e=250 / 249.99$ | endurance (37) inclusive: C5 accepted at 250; depot rejected on (36)=650 |
| N2-C | fixture S, anchor T1 | (28) / fleet capacity for N2; objective 1500 equal to the base is accepted but not an improvement; tie to C5 |
| N2-D | trip recovering at the end depot | the only in-window node C4; replaced node = depot |
| Errors | -- | `InvalidAnchor`, `InputInfeasible` (omitted customer, duplicated customer, input with recovery before launch), `NoEligibleAnchor`; Rng overloads: one `uniform_index` draw, anchor = vector index, both trips drawn over 40 seeds |

Every record of every block is also passed through `check_records`, an independent re-check (own skeleton/route arithmetic): exactly one node of exactly one trip changed (the skeleton for N1, the whole route for N2); every route FLP is some trip's launch and `flps_dropped` is exactly the old launch iff unused; no FLP leaves its arc; exactly-once coverage; `InvalidRecovery` $\Leftrightarrow$ the recovery is not after the launch on the route; objective **and verdict equal the frozen reference's** in every evaluated record; only the allowed reasons occur; `accepted + rejected == generated`; `evaluated + InvalidRecovery == generated`; no duplicates; best = first lowest feasible.

**`tests/test_n1_n2_oracle.cpp`: 3,470,761 checks, 0 failures.** An independent brute force with its own skeleton/gap/ratio arithmetic that calls **none** of the production helpers (not `detail_relocate`, `flp_insertion_position`, `remove_from_route`, `trips_launching_from`, `gaps_within_window`, `FlpArcIndex::flps_on_arc`; the FLPs of an arc are re-derived from the `FlpData` map and the route is rebuilt from skeleton + FLP set), judged by the frozen reference. 80 seeded instances (8--14 customers, 1--3 drones, four variants: paper scale, endurance $\times3$, a 15 km area, both), 640 random-walk bases over all eight operators (634 with trips, 380 with two drones in use, 24 with a shared launch FLP; 287 accepted drone-id re-draws), windows $w=0,1,2,3$, every trip. Record-by-record equality of candidate order, solutions, `flps_dropped`, reason and objective; the whole campaign runs twice and the digest must match. Non-vacuity (all asserted): N1 231,964 candidates (59,918 accepted, 83,721 `InvalidRecovery`, 77,799 endurance, 9,145 sequencing, 1,381 fleet, 215,078 new-FLP targets, 16,886 shared-FLP targets, 230,158 dropped and 1,806 kept old launches, 22,534 depot-arc targets); N2 25,986 candidates (5,039 accepted, 12,191 invalid, 5,184 endurance, 2,996 sequencing, 576 fleet, 424 exact ties, 2,410 end-depot targets, 4,626 window-clipped calls); `EvalFlpArc`, `EvalFlpUnused`, `AffectedTrips`, `NoFreeDrone`, `NoReplacementFlp` never occur.

**Mutation (`tests/mutation/m5_mutate.py`): 62 of 62 mutants killed** (N1 26, N2 17, shared step 17, `flp_insertion_position` ratio order, `gaps_within_window` boundary). Two (the Rng no-trip guard) are killed only by the unit test, as the oracle never calls the Rng overload on an empty solution; all others by both programs. None survived, none failed to compile. Raw: `m5_mutation_results_raw.txt`.

## 8. Validation and measurements

* **Fresh Release build** (`-O3 -DNDEBUG`): 0 warnings; the new files also compile warning-free with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion`; **17 of 17 CTest programs pass**. **Fresh ASan + UBSan build** (`-O1 -g -Wall -Wextra -Wpedantic`): 0 warnings, 17 of 17 pass, 0 reports. Three repeated runs of each new program give byte-identical output (md5).
* **Regression:** `test_n3_n6` 587 and `test_n3_n6_oracle` 613,135 (the post-A8 totals), `test_n4_n5` 233, `test_n4_n5_oracle` 1,117,041, `test_n7_n8` 253, `test_n7_n8_oracle` 1,696,098, `test_evaluator_differential` 26,745, `test_fuzz_search_infra` 11,262: unchanged. Candidate and outcome counts of `bench_n3_n6` (paper and endurance $\times3$), `bench_n4_n5` (`bench_operators`) and `bench_n7_n8` are **identical** to the earlier runs (diff of the count lines); the evaluator is still $\times5$--$\times10$ faster than the frozen reference (`evaluator_bench_after_m5_raw.txt`; the VM was noisier than in earlier runs).
* **Benchmarks (`bench_n1_n2`, base seed 31415, window 3, initial solutions, which fly every trip with drone 0; single runs on a 2-core VM):**

| n | trips | N1 candidates | N1 feasible | N1 outcomes | N1 sweep gen / full | N2 candidates | N2 feasible | N2 outcomes | N2 sweep gen / full |
|---|---|---|---|---|---|---|---|---|---|
| 10 | 3 | 156 (76 evaluated) | 12 (7.7 %) | 80 invalid, 64 endurance | 15.4 / 137 us | 12 (8 evaluated) | 1 | 4 invalid, 7 endurance | 4.9 / 17.6 us |
| 20 | 6 | 234 (133) | 18 (7.7 %) | 101 invalid, 115 endurance | 33.3 / 402 us | 30 (16) | 1 | 14 invalid, 15 endurance | 16.5 / 59.8 us |
| 50 | 23 | 750 (431) | 69 (9.2 %) | 319 invalid, 343 endurance, 19 sequencing | 302 / 4,694 us | 129 (66) | 1 | 63 invalid, 55 endurance, 10 sequencing | 223 / 874 us |

Generation costs 0.07--0.12 us per generated candidate (net of the base evaluations); the marginal cost of an evaluated candidate is the evaluator's own (N1 1.60 / 2.77 / 10.19 us against 1.32 / 2.38 / 9.12 us for one `evaluate_solution`; N2 1.58 / 2.71 / 9.86 us). Allocations per full sweep: N1 4,844 / 11,778 / 79,474 (31 / 50 / 106 per generated candidate: the candidate solution copy, dominated by the truck route), N2 619 / 1,780 / 15,199. Labelled variants (not paper parameters): **endurance $\times3$**: N1 feasible 15.6 / 16.3 / 15.2 %, N2 4.8 / 2.0 / 3.9 %, sequencing rejections appear (N1 15 / 64 / 207: all trips fly with drone 0); **alternate drone ids**: N1 feasible 7.7 / 7.7 / 11.7 %, N2 8.3 / 3.3 / 8.5 %; **both**: N1 19.4 / 28.6 / 32.6 %, N2 19.1 / 22.5 / 24.8 %. Raw: `n1_n2_bench*_raw.txt`.

## 9. Findings and open points

* **The evaluator, not the operators, decides feasibility.** At paper scale 80--86 % of the evaluated N1 candidates are endurance-infeasible (64 of 76, 115 of 133, 343 of 431) (same as N7/N3: 1800 s against 60 km trips), so only 8--9 % of generated N1 candidates are feasible; N2 yields at most one feasible candidate per sweep at paper scale.
* **About half of N1's generated candidates are `InvalidRecovery`** (51 / 43 / 43 % at $n=10/20/50$): the symmetric D1 window includes the FLPs behind the recovery node, especially the (long) arc after it. They cost nothing to evaluate (never evaluated) but inflate `generated`. This follows from the choice to name rather than silently skip them (A15, the same convention as N4). If you prefer not to generate them at all, it is a one-line change in each operator; the accepted set would be identical.
* **Both operators do move the objective where they can:** in the oracle campaign (random-walk bases, windows 0--3) 3,831 of 11,232 N1 calls and 2,861 of 11,232 N2 calls found a strict improvement. At paper scale the benchmark base has at most one feasible N2 candidate per sweep and 12--69 feasible N1 candidates per sweep; whether any of them improves was not measured there (the benchmark does not record objectives).
* **Decisions needing your approval (all are implementation assumptions; none is a paper fact):**
  1. **A13** N1 candidate set (every FLP on the arcs within $w$ gaps, current launch excluded, shared and unrouted FLPs included).
  2. **A14** N2 candidate set (skeleton nodes, customers and the end depot, within $w$; never the start depot).
  3. **A15** `InvalidRecovery` for a launch/recovery that breaks "recovery after launch": generated, counted, unevaluated, unrepaired (versus silently skipping).
  4. **A16** the drone id never changes (N1/N2 are not allowed to dodge a sequencing violation by re-assigning the drone).
  5. **A17** the old launch FLP is dropped iff unused (explicit, counted), kept if another trip launches there.
  6. **A18** `Rng` overloads: one `uniform_index(#trips)` draw.
  7. The **N1 deviation**: customers are not launch candidates (existing Issue 7 FLP-only decision), although the paper's text says "adjacent customer nodes and flexible launch nodes".
* **Unresolved from the paper:** the exact meaning of "adjacent" for N1/N2 (size and direction), and Figs. 10a/10b (known only through captions). The original PDF is still not in the project files; a copy would let A13/A14 be checked against the figures.
