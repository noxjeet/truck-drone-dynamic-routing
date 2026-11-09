# Paper 1 decision register (reconciliation pass)

**Scope:** proposed decisions for the ten paper-vs-code issues found in `docs/paper1_full_implementation_audit.md`. This is a read-only pass. Nothing in the implementation, tests, CMake, README, `DECISIONS.md` or the plans was changed. This file is the only output.

**Sources, in priority order:**
1. The original Paper 1 PDF (Han, Chi & Sun 2026, ITOR 33:4072–4109).
2. The audit.
3. The current repository at commit `aebc03f`, whose source and tests are byte-identical to what the audit examined (checksum verified 28 Sep 2026).
4. `DECISIONS.md`.

**Re-inspection for this pass:** the following pages were re-read as images, not from notes.

| PDF page | Content re-read | Resolution |
|---|---|---|
| p10 | Assumptions, Fig. 3 | 220 dpi |
| p13 | Eqs. (9)–(23) | 300 dpi; (18) previously at 400 dpi |
| p14 | Eqs. (24)–(34) and their text | 300 dpi |
| p15 | Timing and endurance text, eqs. (35)–(37) | 300 dpi |
| p16 | Figs. 4–6 and text | 300 dpi |
| p18–19 | Representation, initialisation, Fig. 8 | 300 dpi |
| p20 | Algorithm 2 | 300 dpi |
| p21–23 | Figs. 10–12 and neighbourhood text | 300 dpi |
| p24 | Parameters and local-search order | 300 dpi |
| p33 | Table 6, eq. (41) | 300 dpi |

No disagreement was found between the audit and the paper. One nuance the audit did not state is added here: the launch-delay slack in Issue 1.

**Page convention:** PDF page N = journal page 4071+N. Both are given where useful.

**Classification labels:** MATCH · PAPER-BASED INTERPRETATION · ASSUMPTION · PAPER AMBIGUITY · PAPER CONTRADICTION · DEVIATION · UNKNOWN.

**Notation:**
- t_i, l_i: truck arrival and departure at node i.
- l̂_{i,u}: drone u's departure time from node i.
- t̂_{j,u}: drone u's arrival time at customer j.
- d̂_{ij}: raw drone travel time from i to j.
- n_i: number of drones launching at FLP i.
- R_k: set of drones recovered at node k; r_k = |R_k|.

---

## Decision table

| ID | Issue | Paper evidence | Current code | Classification | Proposed decision | Confidence | Must fix before Phase G? |
|---|---|---|---|---|---|---|---|
| I-1 | Multiple launches and SL | (18), (19), (21), p13; text p15 | Truck leaves FLP at t_i + SL for any n_i; every drone launches at t_i + SL | DEVIATION (truck side only) | Drones launch concurrently at t_i + SL; truck holds n_i·SL | HIGH | YES |
| I-2 | SR placement | (18) sums SR over P_i⁺ (launches), p13; (20) p13; text of (18) p15 says "drone recovery tasks" | Concurrent SR windows max(t_k, landing) + SR at the recovery node; nothing at the launch node | **PAPER AMBIGUITY: SR** (the equation contradicts its own explanation) | Controlled interpretation "B": second sum of (18) read over P_k⁻ (recoveries). l_k ≥ t_k + Se^T_k + r_k·SR, plus (20) for each drone. Literal reading "A" kept as a documented alternative. | MEDIUM | YES |
| I-3 | Endurance | (29)–(30) p14; (35)–(37) p15; text p15 and p30 | outbound·(1+θ) + return ≤ e | DEVIATION | E = (d̂_ij + Se^D_j)(1+θ_j) + max(d̂_jk, t_k − l̂_{j,u}) ≤ e | HIGH | YES |
| I-4 | Physical drone identity | U-indexed variables, (7), (11), (12), (28) p12–14; Figs. 3 and 7; p23 Random_Drone_Remove | `drone_slot` is a per-trip counter; evaluator ignores \|U\| | DEVIATION | Physical `drone_id` ∈ [0, \|U\|) on each trip. Evaluator checks each drone's route order per (28) and the id range. | HIGH | YES |
| I-5 | FLP generation | p8, p9, **p18 "proportional to the truck's driving distance"**, p19; 2 km (p26, p28, p31); eq. (41) ⌊D/d⌋ (p33); Table 4 (p25) | Fixed `points_per_arc` (1, midpoint) on r0 arcs, generated once | DEVIATION (structure) + UNKNOWN (main-experiment d) | Spacing rule per arc: n_ab = ⌊D_ab/d⌋ points at equal ratios, depot arcs included. L_ab is a deterministic function of arc (a,b), so it is effectively static for all arcs. d is configurable with no paper default. | MEDIUM–HIGH (rule) / UNKNOWN (d) | YES (structure); d value NO |
| I-6 | FLP–arc validity in Phase F | **p9 "If the truck does not pass through arc (i,j), it cannot access the launch point in set L_ij"**; (13)–(15) p13; text p14; Figs. 3 and 8d | F4 replaces j by an FLP of L_{prev,j}; the truck then drives prev→FLP→next. Not checked anywhere. | DEVIATION | Invalid per the paper. Add an evaluator check: FLPs between a and b must belong to L_ab, in ratio order. F4 must draw launch FLPs from arcs of the resulting route (smallest correction: L_{prev,next}). | HIGH | YES |
| I-7 | Launch from customers or depot | FLP-only: Assumption 4 (p10), condition 1 (p11), (5)(6)(11)(17)(19)(21), Table 1 (p7), p1, p16. Customer/depot launches: Figs. 7, 10a, 11a, 11b, 12a, 12b; N1 text (p20–21); route figures (p29, p30, p32). | FLP-only | PAPER CONTRADICTION | FLP-only everywhere. Customer launches in neighbourhood figures are treated as schematic; the paper defines no timing for them. | MEDIUM–HIGH | YES (affects N1, N5, N6, N7 semantics) |
| I-8 | Neighbourhood 3 semantics | Text p21 ("adjacent trip"); Fig. 10c (both trips drone 1, orange) | Implemented in M4 (`neighbourhood_n3.hpp`); the backup spec's "Reading A" (single-trip retarget) contradicts the figure | PAPER-BASED INTERPRETATION | N3 = exchange the served customers of a trip and an adjacent (consecutive) trip **of the same drone**; launch and recovery nodes and the truck route stay fixed | MEDIUM | NO (implemented in M4 as proposed; see the M4 addendum) |
| I-9 | Stale / unused FLPs | (6) p12; text p14 ("prevent the truck from arriving without the drone being launched"); (8)–(9); Fig. 8c→8d; p18 | F4 never leaves idle FLPs; the evaluator accepts them silently | DEVIATION (evaluator scope) | Invariant: every FLP on the route has ≥ 1 launch and appears at most once. Moves must remove an FLP when it loses its last launch. | HIGH | YES (check + invariant) |
| I-10 | Algorithm 2 drone limit | Alg. 2 lines 3–4, 13–14 (p20); text p19 | Global time-aware fleet check; no per-cluster cap | DEVIATION (from the literal pseudocode) | Literal reading: per cluster, k = 1..D, at most one new trip per drone k, assigned to drone k; feasibility includes that drone's (28) sequencing. Do it together with I-4/I-6 in Phase F. | MEDIUM | NO (starting-solution quality only); recommended alongside I-4/I-6 |

---

## Issue 1: Multiple drone launches and SL

### What the paper explicitly says
- **(18) p13 / 4084:** t_i + Se^T_i + Σ_{u∈U} Σ_{s∈P_i⁺} y_s^u·SL + Σ_{u∈U} Σ_{s∈P_i⁺} y_s^u·SR ≤ l_i, for i ∈ C₀ ∪ L. The SL term equals n_i·SL.
- **(19) p13:** l̂_{i,u} − (1 − Σ_{s∈P_i⁺} y_s^u)M ≤ l_i. Each drone launched at i departs no later than the truck.
- **(21) p13:** t_i + SL − (1 − Σ_{s∈P_i⁺} y_s^u)M ≤ l̂_{i,u}. Each drone departs at least SL after the **truck's arrival**.
- **Text p15:** (18) "the truck may only leave the node after completing the required customer service, drone launch, and drone recovery tasks". (19) "all drones launched at a certain launch point must depart before the truck leaves the node". (21) "the drone must be launched after the truck arrives at the launch point and completes the launch preparation".
- **p16 and Fig. 6:** multiple drones may launch from the same FLP. **Fig. 3 (p10):** both drones launch from FLP e.

### What the current code does
At an FLP with n_i ≥ 1 launches: `departure = arrival + SL`, and every launching drone gets `launch_time = departure`. The code is `timing_engine.hpp:378–380`.

### What is unambiguous
- Truck departure accounting: **l_i ≥ t_i + n_i·SL**. This holds under both readings of (18), since the SR question in Issue 2 does not touch the SL term.
- Individual drone departures: each drone must satisfy t_i + SL ≤ l̂_{i,u} ≤ l_i. No constraint orders drones against each other, and nothing states a staggered time such as t_i + m·SL.
- So the MILP describes **concurrent individual launches, with the truck paying n_i·SL**. It does not describe sequential launches. This combination is not physically intuitive, but it is what the equations say. Per the reproduction goal, it is reproduced as printed.

### What is ambiguous/contradictory
**Launch-delay slack.** In the MILP, l̂_{i,u} may take any value in [t_i + SL, l_i]. The objective (1) rewards the earliest launch, since t̂ increases with l̂_i. The endurance constraint (36), however, can *favour* a later launch: (t_k − l̂_j) shrinks when the drone leaves later and would otherwise wait for the truck. An optimiser could therefore delay a launch to make an otherwise endurance-infeasible trip feasible. The paper does not discuss this. It is a property of the formulation, not a stated rule.

### Proposed decision
- Each launched drone: **l̂_{i,u} = t_i + SL**, the earliest time allowed by (21).
- Truck: **l_i = t_i + n_i·SL** (+ n_i·SR only if reading A of Issue 2 is chosen).
- Launch-delay slack is **not** exploited. It is recorded as a known difference from the MILP's freedom, and listed in "Do not change yet".

### Consequences
- **Phase E:** one line changes at the FLP departure rule.
- **Phase F:** today every F4 FLP has n_i = 1, so there is no numeric change unless I-6/I-10 produce shared FLPs.
- **Phase G/H:** the rule matters as soon as moves put two trips on one FLP, as in the paper's own Fig. 3 and Fig. 6.
- **Tests:** add a two-launch FLP test (truck departure t + 2·SL; both drones launch at t + SL). The existing single-launch tests are unchanged.
- **Endurance:** the earliest launch maximises hover time under I-3, so a few trips may be rejected that an exact MILP could keep through delay.

### Confidence
HIGH for n_i·SL at the truck and for concurrent drone departures. MEDIUM that ignoring launch-delay slack is acceptable for reproduction.

---

## Issue 2: SR / recovery-time ambiguity — PAPER AMBIGUITY: SR

### What the paper explicitly says
1. **Literal mathematics.** The (18) print, verified at 400 dpi, has **both** double sums over s ∈ P_i⁺. P_i⁺ is defined only for i ∈ L, the launch points (p12 / 4083). (18) quantifies over i ∈ C₀ ∪ L. Therefore:
   - at an FLP: l_i ≥ t_i + Se^T_i + n_i·SL + n_i·SR, where Se^T_i is undefined for an FLP and would be 0;
   - at a customer or the start depot: l_i ≥ t_i + Se^T_i. Both sums are empty.
2. **(20) p13:** l̂_{j,u} + d̂_{jk} + SR − (1 − Σ_{s∈P_j∩P_k⁻} y_s^u)M ≤ l_k. Per drone: the truck leaves recovery node k no earlier than that drone's landing + SR. The windows are independent (concurrent), and there is no reference to t_k.
3. **Operational text p15:** (18) makes the truck leave "after completing the required customer service, drone launch, and **drone recovery** tasks".
4. **p9 / 4080:** "the time consumed by the drone from take-off to return to the truck includes take-off preparation time (SL), recovering time (SR), and service time". **Table 2 (p11):** SR = "Drone recovering time". **Assumption 7 (p10):** the first vehicle to arrive waits for the other.

### What the current code does
- At the recovery node, each drone completes at max(t_k, landing) + SR, and l_k = max(t_k + Se^T_k, max over R_k of completion). Nothing is charged at the launch node.
- This is neither literal A nor B. It adds max(t_k, ·), which appears in no equation.

### What is unambiguous
- SR is a per-drone recovery duration, and some SR constraint applies at the recovery node through (20).
- (20) alone gives concurrent per-drone windows measured from the drone's landing.
- The printed (18) charges SR at launch nodes only.

### What is ambiguous/contradictory
- **The equation contradicts its own explanation.** Literally, (18) charges recovery time at a launch point where no recovery happens, and charges no SR at customers, where recoveries do happen. The text says (18) covers recovery tasks. The literal SR term also double-counts SR per trip: once at the launch FLP and once at recovery through (20).
- **Structural evidence for an index typo.**
  - (18) is quantified over C₀ ∪ L. Customers are included, but under the literal print the only term that applies to them is Se^T.
  - A P_k⁻ sum is defined exactly for recovery nodes (k ∈ C_E).
  - Each term would then apply to the node type where that operation happens: Se^T and SR at customers, SL at FLPs.
  - (5)/(6) pair with (7) in the same launch-set / recovery-set pattern.
- **Figures and algorithms:** Figs. 3, 5, 7 and 16 show recoveries (including several at one customer in Fig. 3) but carry no timing. Algorithm 2 and the neighbourhood text say nothing about recovery time.
- **Does the paper resolve it elsewhere?** No. No page restates (18), gives a worked timing example, or quantifies recovery handling. **Unresolved.**

Numeric illustration (truck arrives at k at 100, Se^T = 30, SR = 40, one drone):

| Drone lands at | Literal A (at k) | Reading B | Current code |
|---|---|---|---|
| 50 | max(130, 90) = **130** | max(170, 90) = **170** | max(130, 140) = **140** |
| 200 | max(130, 240) = **240** | max(170, 240) = **240** | max(130, 240) = **240** |

Under reading A, the launching FLP additionally pays n·SR.

### Proposed decision
**Controlled interpretation B**, labelled "PAPER AMBIGUITY: SR":

- At an FLP: l_i = t_i + n_i·SL (no SR).
- At a truck-served customer k: l_k = max( t_k + Se^T_k + r_k·SR, max over u ∈ R_k of (l̂_{j_u,u} + d̂_{j_u,k} + SR) ). The first term is (18) with P_k⁻; the second is (20).
- At the end depot c+1: (18) does not apply (c+1 ∉ C₀ ∪ L). Only (20) applies there, and it does not affect the objective.
- **Why B and not A:** B changes one set symbol and makes (18) consistent with its own text, the set definitions, the quantifier domain, and (20). A needs no symbol change, but it contradicts the paper's explanation of (18) and charges recovery time at launch points.
- **Keep A implementable** behind a documented policy switch, so the sensitivity of results to this ambiguity can be measured. Do **not** keep the current max(t_k, ·) rule: it matches neither reading.
- **Consequence for "Se1 vs SR concurrency" (DECISIONS UNRESOLVED 5b):** under B, truck service and recovery handling at the same node are **additive**. Under A they are concurrent (max).

**If you disagree:** the least-edit alternative is A. Its formulas are l_i = t_i + n_i·(SL+SR) at an FLP, and l_k = max(t_k + Se^T_k, max over u of landing_u + SR) at a customer.

### Consequences
- **Phase E:** the recovery-node departure formula changes; launch-node SR is added only under A.
- **Timing tests:** Block 9 (currently asserts concurrency) and the hand-computed §12–14 and §8 values must be recomputed.
- **Init tests:** the hand-computed objectives of B6, B8, B9 and B10 must be re-derived. Values will change whenever SR > 0 or Se^T > 0 at a recovery node.
- **Phase F/G:** no structural change, only different numbers.
- **Documentation:** DECISIONS "UNRESOLVED 2" and "5b" should become "PAPER AMBIGUITY: SR → interpretation B (A as switch)".

### Confidence
MEDIUM that B is the authors' intent. HIGH that the literal print (A) and the current code differ from each other and from B.

---

## Issue 3: Drone endurance / battery

### What the paper explicitly says
- **(29) p14:** e + M(1 − y_s) ≥ (t_k − l̂_{j,u}) + (l̂_{j,u} − l̂_{i,u}).
- **(30) p14:** e + M(1 − y_s) ≥ (l̂_{j,u} − l̂_{i,u}) + d̂_{jk}.
- **Text p15:** "(29) establishes that the flight duration of a drone, **while awaiting the truck**, must not exceed the permissible battery capacity. Likewise ... if the truck is waiting, the drone's flight duration must also adhere to the battery limit."
- **(35) p15:** t̄_u = t_u(1+θ_j), θ_j ≥ 0, where θ_j is the "energy consumption coefficient of the drone during the flight to serve customer j" and the first stage is "a loaded flight from the launch point to the customer location".
- **(36) p15:** e + M(1 − y_s) ≥ (t_k − l̂_{j,u}) + (l̂_{j,u} − l̂_{i,u})(1+θ_j). **(37) p15:** e + M(1 − y_s) ≥ (l̂_{j,u} − l̂_{i,u})(1+θ_j) + d̂_{jk}. These replace (29) and (30).
- **Text p30 (Fig. 16):** endurance insufficient "for long-distance flights, **including waiting for the truck at the merging point**".
- Parameter e: Table 2 says "in minutes"; the experiments use 30 min (p24).

### What the current code does
E = d̂_ij·(1+θ_j) + d̂_jk ≤ e. Service time Se^D and waiting are excluded, and θ is applied to the flight only (`timing_engine.hpp:445–446`). `DECISIONS.md` states that waiting and ground time do not count.

### What is unambiguous
- With the earliest schedule (Issue 1): l̂_{i,u} = launch departure, and l̂_{j,u} = l̂_{i,u} + d̂_ij + Se^D_j by (17) and (22).
- Both (36) and (37) apply to every trip. Together they are equivalent to:

  **E_s = (d̂_ij + Se^D_j)·(1+θ_j) + max( d̂_jk , t_k − l̂_{j,u} ) ≤ e**

  where t_k is the **truck's arrival** at recovery node k.

| Component | Counts toward e? | Source |
|---|---|---|
| Outbound flight d̂_ij | **Yes**, × (1+θ_j) | (36), (37) |
| Drone service Se^D_j at customer | **Yes**, × (1+θ_j) | (l̂_j − l̂_i) includes (22) |
| Waiting for the truck at k (truck arrives after drone) | **Yes**, unscaled, up to t_k | (36); text p15, p30 |
| Return flight d̂_jk | **Yes**, unscaled | (37); within (t_k − l̂_j) in (36) |
| Recovery SR | **No** (t_k is arrival; no SR term) | (36), (37) |
| Launch preparation SL | **No** (l̂_i is departure after SL) | (21) |
| Truck service at k | **No** (t_k is arrival) | (36) |
| Truck waiting for the drone | No extra term; (37) binds | (37) |

### What is ambiguous/contradictory
- **θ scope.** The prose calls θ a *flight* coefficient, but (36) and (37) multiply (l̂_j − l̂_i), which includes Se^D. The equations are explicit, so they are followed.
- **Units.** e is given "in minutes" while the results are in seconds. Treat this as a unit conversion, not an ambiguity.
- **Launch delay.** Launch-delay slack (Issue 1) would reduce the waiting term. It is not exploited.

### Proposed decision
The evaluator implements exactly:

```
outbound  = d̂(i,j)
service   = Se^D(j)
ret       = d̂(j,k)
dep_j     = launch_time + outbound + service          // l̂_{j,u}
wait_term = max(ret, t_arrival_truck(k) − dep_j)
E         = (outbound + service)·(1 + θ_j) + wait_term
feasible  ⇔ E ≤ e  (with ε tolerance)
```

### Consequences
- **Phase E:** endurance moves after truck arrival times are known. The forward pass already computes t_k before the check, so no second pass is needed.
- **Feasible set:** fewer trips are feasible. With θ = 1 and Se^D = 30 s, each trip spends at least 60 s of endurance on service, plus any hover time.
- **Tests:** timing §9 and §11 values change; add hover-wait and Se^D·θ cases. init B4, B6, B8–B10 may accept or reject different trips, so re-derive them.
- **Documentation:** DECISIONS' endurance section needs correcting.

### Confidence
HIGH. The formula is a direct transcription of (36)–(37) under the earliest schedule.

---

## Issue 4: Physical drone identity

### What the paper explicitly says
- **Variables:** y_s^u, t̂_{i,u} and l̂_{i,u} are indexed by drone u ∈ U (p11–14). Table 2 lists y_s without u, but every constraint uses y_s^u.
- **(7), (11), (12):** per drone, at most one launch per FLP and at most one recovery per node, and recovery only where the truck visits.
- **(28) p14:** t_k − M(3 − p_{ig} − Σ_{s∈P_i⁺∩P_k⁻} y_s^u − Σ_{s∈P_g⁺} y_s^u) ≤ l̂_{g,u}. If drone u flew i→…→k and launches again at g after i, it cannot depart g before t_k. Text p15: "prevent the relaunching of drones without truck recovery".
- **p9:** "drones are capable of multiple takeoffs and recoveries".
- **Fig. 3 (p10):** each of the 2 drones flies 3 sequential trips.
- **Fig. 7 (p18):** each drone has its own ordered list of blocks (trips).
- **Algorithm 2 (p20):** trip f^(k) is produced per drone index k.
- **p23 Random_Drone_Remove:** "a drone is selected at random, and its whole trips are removed".
- **p22 N7:** "assign an **available** drone".

### What the current code does
`DroneTrip::drone_slot` is a counter (0, 1, 2, …) set in F4. It is not bounded by |U| and has no physical meaning. `evaluate_solution()` never reads `num_drones`. F4 separately checks max time-overlap ≤ |U| (`drone_fleet_capacity.hpp`).

### What is unambiguous
- The paper has a persistent physical drone identity, and every trip is assigned to one drone.
- One physical drone performs multiple trips sequentially.
- Different drones may fly simultaneously, including from the same FLP.
- The fleet is at most |U| drones.
- **Trip assignment matters for feasibility:** (28) constrains each drone's own sequence. It also matters for search: Random_Drone_Remove and "available drone" depend on it.

**Derived fact (INFERENCE from (28) plus route order).** Relaunch at g must satisfy l̂_{g,u} ≥ t_k. If g lies *after* k on the route, this is automatic, since t_g ≥ l_k ≥ landing + SR by (20). If g lies *between* i and k, it is impossible, since l̂_g ≤ l_g < t_k. So (28) is equivalent to a route-order rule:

> **for each drone, trips sorted by launch position satisfy launch_pos(next) > recovery_pos(previous).**

Under that rule, a trip set with max overlap ≤ |U| always admits some valid assignment (interval partitioning). The current count check therefore tests "an assignment exists"; it does not represent the assignment.

### What is ambiguous/contradictory
Whether "available drone" (N7) means "a drone idle over the candidate's route interval" is not defined. The route-order rule above gives it a precise meaning, so it is not a real conflict.

### Proposed decision
The minimum faithful data-model change:
- Give `drone_slot` the semantics **`drone_id` ∈ [0, |U|)**, a physical drone, or add a new field. Every trip in a `Solution` must carry a valid id.
- The trip-to-drone assignment is part of the `Solution`. There is no separate structure: a drone's trips are the trips carrying its id, ordered by launch route position.
- The evaluator checks: (a) id range; (b) per drone, launch_pos(next) > recovery_pos(prev), which is (28); (c) (11)/(12), which follow from (b). A violation is reported with a reason.
- The count sweep in `drone_fleet_capacity.hpp` becomes a derived consistency check, or is retired. It is no longer the source of truth.

### Consequences
- **Fleet capacity:** enforced in the evaluator through ids. This settles DECISIONS "UNRESOLVED 3" (the paper requires it) and "UNRESOLVED 4" (physical identity).
- **Evaluator:** new checks. Timing is unchanged apart from I-1/2/3.
- **Algorithm 2:** trips are assigned to drone k (I-10). Under the patched reading, F4 must assign ids, e.g. the lowest-index drone free over the trip's route interval.
- **Random_Drone_Remove:** becomes well defined: remove all trips with a random id that has at least one trip.
- **Future neighbourhoods:** N3 (Issue 8) needs the same-drone trip order. N6 swaps across drones. N7 needs an idle drone. N8 frees a drone slot.
- **Tests:** add id-range, overlap-per-drone and sequential-reuse tests at evaluator level. The model tests on `drone_slot` change meaning.

### Confidence
HIGH on identity being required and on the (28) equivalence. MEDIUM on reusing the `drone_slot` field versus a new field; that is purely an engineering choice.

---

## Issue 5: FLP generation

### What the paper explicitly says
- **p3 / 4074:** launch points "evenly distributed across the truck's route".
- **p8 / 4079:** launches "at any two customer locations **or between a customer location and the depot** along the truck route"; the method "divides a series of feasible launch points by **equal proportions of the distances** between two locations".
- **p9 / 4080:** "The flexible launch points are distributed evenly along the truck route, L = L_ij ∪ L_kg ∪ … L_ij = set of viable emission points between node i and node j. If the truck does not pass through arc (i,j), it cannot access the launch point in set L_ij." **Table 2 (p11):** L_ij is defined for i ∈ C₀, j ∈ C_E, i.e. for every arc.
- **p18 / 4089:** "the insertion of the flexible launch point along the truck route gives rise to a **linkage effect in the event of a change to the truck service node**" and "**the number of flexible points inserted is proportional to the truck's driving distance**".
- **p18–19:** initialisation step (c) "insert flexible launch points in the route of existing trucks" is "performed on the current truck route according to the predefined flexible launch point partitioning ratio".
- **Figures:** Fig. 2 (p9) shows 4 evenly spaced points on one arc. Fig. 7 (p18) shows 3 per customer–customer arc, schematic. Fig. 8c (p19) shows 0, 1 or 2 points per arc depending on length, depot arcs included. In Fig. 3 (p10), FLPs a and b lie on the depot→C3 arc of the final route.
- **p26, p28, p31 (Sec 6.3.1–6.3.3, Dalian sensitivity):** "intervals between launching points are established at 2 km".
- **p33 (Sec 6.3.4 sensitivity):** "Launch points are equitably divided based on the distances between successive customer points. Equation (41)…" with **n = ⌊D/d⌋**, where n is the number of divided nodes, D the distance between two customer locations, and d the unit division length, d ∈ [1000, 5000] m in 500 m steps.
- **Table 4 (p25):** 214 / 837 / 3493 FLPs for 10 / 20 / 50 customers. Text (p25–26): these counts are why the MILP fails ("3493 … exceeds the solving capability of the MILP solver").
- **p35:** the method "requires pre-determining the number of flexible launch points along the truck route".
- **Sec 6.1 (p24):** the main-experiment parameters list gives speeds, e, SL, SR, service time, θ and drone count. **No FLP spacing or count is given.**

### What the current code does
`FlpGenerationPolicy{points_per_arc = 1}` at ratios k/(n+1) on every arc of the TSP route r0, depot arcs included. Generated once and frozen. Launch candidates for j = FLPs with arc_to == j. `DECISIONS.md` quotes eq. (41) as ⌈D/d⌉.

### What is unambiguous
- FLPs are evenly spaced along individual arcs, and the number per arc depends on **arc length** (p8, p18).
- Depot arcs are included (p8; Figs. 3 and 8c).
- In the MILP an FLP belongs to exactly one arc and is usable only if that arc is traversed (p9).
- Eq. (41) is ⌊D/d⌋ (floor), not ceiling. `DECISIONS.md` is wrong on this point.
- The 2 km spacing and eq. (41) are both stated inside the Sec 6.3 **sensitivity analyses**. Neither is stated as the main-experiment value.

### What is ambiguous/contradictory
- **The main-experiment d is not given anywhere.** UNKNOWN.
- **Placement within an arc for n = ⌊D/d⌋:** either ratios m/(n+1), or multiples of d from the start. Not stated.
- **Depot arcs vs eq. (41):** eq. (41) says "two customer locations", while p8 and Fig. 8c include depot arcs. Treat as wording, not a rule.
- **p19 "partitioning ratio"** could be read as a fixed count per arc. p18's explicit "proportional to driving distance" outweighs it.
- **Table 4 does not confirm any rule.** Arithmetic only (not from the paper): counts per unordered node pair are 214/55 ≈ 3.9, 837/210 ≈ 4.0 and 3493/1275 ≈ 2.7. No single tested rule reproduces all three. Since the text ties Table 4 to MILP size, and the MILP defines L_ij for every arc, Table 4 most likely counts the MILP's all-pair L, not the heuristic's route FLPs (INFERENCE). Table 4 is therefore not a target for the heuristic's FLP count.
- **Static vs regenerated during VNS:** not stated. p18's "linkage effect" implies FLPs follow arc changes.

### Proposed decision
- **Rule:** for every arc (a,b) with length D_ab: n_ab = ⌊D_ab / d⌋ points at ratios m/(n_ab+1), m = 1..n_ab. The ratio placement is an ASSUMPTION matching "equal proportions" (p8) and "equitably divided" (p33). Depot arcs are included.
- **Definition, not pool:** L_ab is a deterministic function of the arc's endpoints and d. It exists conceptually for every arc, as in the MILP's L_ij, and is materialised when an arc appears in a route. There is no "regeneration" step: route changes simply use the L of the new arcs. A trip whose FLP's arc disappears becomes invalid (Issue 6).
- **d:** a required configuration value with **no paper default for the main experiments**. 2 km is the only value the paper uses (sensitivity). Candidate values are listed in "Do not change yet". A fixed-count policy can remain available as a documented alternative, but not as the default.
- **Timing within initialisation:** FLPs are materialised after step (a), on the current route, as in step (c). Under the arc-function definition, "before or after route modification" makes no difference.

### Consequences
- **Phase D:** the policy changes from count-based to spacing-based, and a stable FLP identity keyed by (a, b, m) is needed. Existing FLP tests encode the fixed count and must be revised.
- **Phase F:** candidate launch points change (see I-6).
- **Phase G/H:** every route-changing move (truck relocate, T-D swap, all shakes) needs arc-keyed FLPs. This is a prerequisite for the first local-search move in the order (truck service node relocate).
- **Documentation:** correct ⌈⌉ to ⌊⌋ and the "sensitivity-only" framing in DECISIONS.

### Confidence
MEDIUM–HIGH for the spacing rule per arc including depot arcs. UNKNOWN for d. MEDIUM for the ratio placement.

---

## Issue 6: Initial-solution FLP insertion / route validity

### What the paper explicitly says
- **p9:** "If the truck does not pass through arc (i,j), it cannot access the launch point in set L_ij."
- **(13)–(15) p13 with text p14:** "(13) the truck is not allowed to travel from the launching point to other customer locations; (14) … from the customer location to other launch points; (15) … moving between launch points is prohibited". The printed quantifiers are loose, but the text is explicit.
- **Initialisation (p18–19):** step (c) inserts FLPs on the current truck route; step (d) assigns trips. "The launch and recover node of the drone correspond to the customer point in the cluster or the attached flexible launch point."
- **Fig. 8d:** drone-served customers are removed from the truck route; the used FLPs (red dots) lie on arcs of the resulting route.
- **Fig. 3:** FLPs a and b lie on the arc 0→C3 of the final route, serving C1 and C2, which are not on the route.

### What the current code does
- F4 replaces customer j's route slot with FLP f ∈ L_{prev,j} (generated from r0), giving the route prev → f → next(j). Arc (prev, j) is no longer traversed. f's coordinates lie on segment prev–j, so the truck makes a detour through a point that belongs to a non-existent arc.
- `evaluate_solution()` does not check FLP–arc membership.

### What is unambiguous
The current behaviour is **invalid under the paper**:
- the truck does not traverse (prev, j), so it "cannot access" L_{prev,j} (p9, explicit);
- the arc f → next(j) is prohibited by (13) and its text;
- the figures of final solutions (Figs. 3 and 8d) show launch FLPs on arcs of the final route.

### What is ambiguous/contradictory
- **Which FLPs count as "attached" candidates in Algorithm 2** (p19): not defined.
- **Consecutive FLPs of the same arc:** allowed by Fig. 3 (a→b) and Fig. 6 (left). The (15) text is unqualified, but its equation restricts only moves *between arcs*.

### Proposed decision
- **Evaluator rule (new):** in the truck route, the nodes between two consecutive non-FLP nodes a and b may only be FLPs of L_ab, in increasing ratio order. Anything else is a violation with a reason.
- **Smallest conceptual correction to F4:** when customer j is removed, the new arc is (prev, next). Launch candidates come from arcs of the **resulting** route that lie before the recovery position. Minimal version: the FLPs of the bridging arc L_{prev,next}. Wider version (closer to p19 "attached flexible launch point"): the FLPs of any resulting-route arc incident to a customer of the current cluster. **Proposal: minimal version first, wider version documented as an alternative.** Both are ASSUMPTIONs.
- **FLPs from the current route:** yes, always the current route, never r0. No pool regeneration is needed under the arc-function definition (Issue 5). Every modification simply re-derives L for new arcs, and trips on removed arcs become invalid.
- **Evaluator does not repair.** Repair is the job of the move that caused the change.

### Consequences
- **Phase F:** candidate generation changes (launch FLP on the resulting route). init B6, B8, B9 and B10 geometries and objectives must be recomputed, and B10 ("farther FLP on entering arc wins") must be rewritten.
- **Phase E:** new check plus tests (valid, cross-arc, out-of-order).
- **Phase G/H:** every route-changing move must detach or re-anchor trips whose arcs vanish. That is part of each move's definition.

### Confidence
HIGH that the current behaviour is invalid. MEDIUM on the exact candidate set (ASSUMPTION).

---

## Issue 7: Drones launching from customers / depot

### What the paper explicitly says

**FLP-only:**
- **Assumption 4 (p10 / 4081):** "The drone is only able to take off from the truck at the launch point, not directly from the truck service points."
- **Single-flight condition 1 (p11):** launch node i ∈ L.
- **Constraints:** (5), (6), (11), (17), (19) and (21) define launches only for i ∈ L. No timing is defined for a customer or depot launch.
- **Table 1 (p7):** "This paper" has launch = FP, and the customer-location column is "–".
- **p1:** FLPs are "non-customer stops".
- **p16:** "drones take off from flexible launch points". Fig. 4 prohibits depot→depot flights; Figs. 5–6 draw stars only.

**Customer or depot launches:**
- **Fig. 7 (p18):** block [C1, C8, C3].
- **Fig. 10a and text (p21):** "relocated from **truck service node C1** to the flexible node". N1 definition (p20–21): "searching **adjacent customer nodes** and flexible launch nodes".
- **Fig. 11a:** launches at C2, and at C1 after the move. **Fig. 11b:** two drones launch at C1. **Fig. 12a:** new trip {C1, C3, C4}. **Fig. 12b:** trips from C1 and C3.
- **Route figures (Figs. 15, 16, 19; p29, p30, p32):** dashed drone lines start at the depot or at customer pins. Noted in the audit; not re-inspected in this pass.
- **Counter-evidence against customer launches in the heuristic (INFERENCE):** on the three 10-customer instances where Cplex proved optimality (GAP 0: 1.1, 1.2, 1.4), VNS z_best equals the MILP optimum exactly (Table 3, p25). A heuristic whose feasible set is a superset of the MILP's would not be bounded by it.

### What the current code does
FLP-only (`has_valid_node_roles`: launch must be an FLP).

### What is unambiguous
- The formal model — assumptions, sets, constraints and Table 1 — is FLP-only.
- Several neighbourhood examples and one neighbourhood definition (N1) involve customer launches.

### What is ambiguous/contradictory
Classification: **A — genuine contradiction.** The other options were checked:

| Option | Assessment |
|---|---|
| B: different meanings of "launch point" | Not supported. p21 explicitly distinguishes "truck service node C1" from "the flexible node". |
| C: model vs heuristic distinction | Possible, but never stated. Assumption 4 is a problem assumption, and no timing rule exists for customer launches. |
| D: figure simplification | Plausible for the schematic figures (few nodes, FLPs omitted for clarity). Cannot explain the N1 *text*. |

### Proposed decision
**FLP-only everywhere** (model, evaluator, initialisation and neighbourhoods). Reasons, strongest first:
1. It is the paper's formal problem definition (Assumption 4, condition 1, the constraints, Table 1).
2. The paper defines no timing for customer or depot launches: (18) and (21) exist only for i ∈ L. Allowing them would require inventing timing rules, which contradicts "do not improve the paper".
3. Table 3's exact MILP matches favour a VNS feasible set within the MILP's.

Figures showing customer launches are treated as schematic simplifications of an FLP adjacent to that customer (INFERENCE). The N1 phrase "adjacent customer nodes" is noted as contradicting Assumption 4 and is not implemented.

### Consequences

| Neighbourhood | Consequence |
|---|---|
| N1 launch node relocate | Candidates = FLPs only, adjacent in route position to the current launch (radius undefined, see "Do not change yet"). Customer candidates are dropped. |
| N5 T-D swap | The newly drone-served customer's trip gets an FLP launch on an arc of the resulting route, chosen the same way as I-6. The figure's C1 launch becomes "FLP adjacent to C1". |
| N6 D-D swap | Only served customers are exchanged; launch nodes stay FLPs. The figure's shared C1 launch becomes a shared FLP. |
| N7 truck delete / drone insert | The trip is built with an FLP launch, same candidate rule as F4 (I-6). The figure's {C1, C3, C4} becomes {FLP near C1, C3, C4}. |

### Confidence
MEDIUM–HIGH. The contradiction is genuine, but the formal definition plus the lack of timing rules make FLP-only the least-assumption choice.

---

## Issue 8: Neighbourhood 3 (drone service node relocate)

### What the paper explicitly says
- **Text p21 / 4092:** "repositioning the drone service node of the drone trip. The operation requires searching **adjacent trip** of the original drone trip, with the objective of ascertaining the most optimal relocation location … customer waiting time". Example: "The drone service node C4 of drone trip {C1,C4,C3} and the drone service node C2 of drone trip {C3,C2,C5} perform relocation operations, reducing the waiting times for customers C2 and C4."
- **Fig. 10c:**
  - Before: truck depot→C1→C3→C5→depot; trips {C1→C4→C3} and {C3→C2→C5}.
  - After: {C1→C2→C3} and {C3→C4→C5}.
  - Both trips are orange, which is drone 1 per the Fig. 9 legend (p20).
- **For contrast, N6 (p22):** "searching adjacent drone service node of **other** drone trips". Fig. 11b shows an orange (drone 1) trip and a green (drone 2) trip.

### What the current code does
Not implemented. The backup (non-repo) spec recommended "Reading A", a single-trip retarget, which the figure contradicts.

### What is unambiguous
- The figure performs an **exchange of served customers between two trips**.
- Both trips keep their launch and recovery nodes: C1→·→C3 and C3→·→C5.
- The truck route is unchanged.
- The two trips are **consecutive**: the first's recovery node C3 is the second's launch node.

### What is ambiguous/contradictory
- **Name vs example:** the name says "relocate" but the example is an exchange.
- **"Adjacent trip":** could mean next/previous trip of the same drone (supported by the colours) or a trip nearby in route or space. Not defined.
- **Whether both directions are searched** (predecessor and successor): not stated.
- **Launch at customers C1 and C3 in the figure:** see I-7; treat as FLPs.

### Proposed decision
- **Before:** drone u has consecutive trips s1 = (i1, a, k1) and s2 = (i2, b, k2), with s2 the next trip of the same drone after s1.
- **After:** s1' = (i1, b, k1) and s2' = (i2, a, k2).
- **Moves:** only the two served customers.
- **Fixed:** launch and recovery nodes of both trips, the drone assignment, the truck route, all other trips.
- **Search:** over every pair of consecutive same-drone trips (in both orders); choose the best feasible exchange by objective ("most optimal relocation location").
- **Disallowed:** changing launch or recovery nodes, involving truck-served customers (that is N5), exchanging across drones (that is N6).
- **Requires** physical drone identity (I-4).

### Consequences
- Must be implemented after I-4.
- The backup spec's "Reading A" must not be used.
- If "adjacent" is later widened to route-adjacent trips of any drone, N3 would overlap with N6. The colour evidence argues against that.

### Confidence
MEDIUM. The exchange mechanics are HIGH (the figure is explicit); "same drone" is an INFERENCE from colours.

---

## Issue 9: Stale / unused FLPs

### What the paper explicitly says
- **(6) p12:** Σ_u Σ_{s∈P_i⁺} y_s^u ≥ Σ_h x_{i,h}. If the truck visits FLP i, at least one drone launches there.
- **Text p14:** (5)–(6) "limit the drone's launch point to prevent the truck from arriving without the drone being launched".
- **(8)–(9):** each FLP is visited at most once.
- **p18:** the route contains the FLPs "that have been inserted into the truck route".
- **Fig. 8c → 8d:** after trip assignment, only the FLPs used by trips remain on the route.

### What the current code does
F4 only puts an FLP on the route when a trip uses it, so there are no idle FLPs in initial solutions. `evaluate_solution()` accepts an idle FLP (departure = arrival) and does not flag it. There is no check for an FLP appearing twice.

### What is unambiguous
- An idle FLP on the route is **infeasible** in the paper's model: it violates (6).
- Unused FLPs should not be in the solution's truck route.
- They still exist conceptually in L (Issue 5), so they are neither "regenerated" nor "ignored in place".

### What is ambiguous/contradictory
Nothing. Timing effect: an idle FLP lying on its own arc adds no distance, since it sits on the segment, and no dwell time. Keeping one would not change the objective; it is purely a model-validity matter.

### Proposed decision
- **Invariant:** every FLP node in the truck route has ≥ 1 launch, and appears at most once. The evaluator reports violations.
- **Move responsibility:** any move that removes the last trip launching from an FLP (N1, N7's opposite, N8, Random_Drone_Remove) must also remove that FLP from the route.

### Consequences
- Small evaluator addition and tests.
- Moves need a "prune idle FLP" post-step.
- No effect on current initial solutions.

### Confidence
HIGH.

---

## Issue 10: Algorithm 2 drone limit

### What the paper explicitly says
- **Algorithm 2 (p20 / 4091):**
  - Input: "Cluster List: C, **Number of Drones: D**, Current Solution: x".
  - Line 1: `i ← 1 repeat`. Line 2: node list n ← C^(i). **Line 3: k ← 1. Line 4: while k ≤ D do.**
  - Lines 5–12: build a feasible list F over the node list. Line 13: k ← k+1. Line 14: f^(k) ← BestRoute(F). Line 15: update n. Line 16: x ← f^(k).
  - Line 17: end while. Line 18: until i > C.
- **Text p19:** "check if there are any **unallocated drones** available for assignment (Line 4); if so … design a drone service trip for each node in the current cluster … The optimal drone trip should be selected … the distributable nodes in the current cluster are to be updated, and the drone trip is to be added."
- **k formula (p19):** D is "the number of drones", the same D used in k = ⌈C/(D+2)⌉.

### What the current code does
No per-cluster cap. Rounds continue while any feasible candidate exists. Global time-aware capacity check across all accepted trips. `drone_slot` counter (see I-4).

### What is unambiguous
- D = |U|, the fleet size.
- The loop counter k is reset to 1 for **every cluster** (line 3 is inside the repeat), and runs to D.
- So the literal pseudocode accepts **at most D trips per cluster**, with the counter reset per cluster.
- The trip chosen in round k is labelled f^(k), i.e. associated with drone k.

### What is ambiguous/contradictory
- **Literal defects:** j is never initialised; i is never incremented; line 7 reads n^(k), presumably meaning n^(j); line 14 uses k after it has been incremented (off-by-one); "feasible" and BestRoute are undefined. These are defects in the listing, not semantics.
- **Conflict with U:** none, *if* "feasible" includes the full model. Assigning f^(k) to physical drone k and requiring (28) for that drone's sequence makes "at most one new trip per drone per cluster" consistent with drone reuse across clusters. If "feasible" ignored capacity, the per-cluster reset could exceed |U| airborne drones. The paper does not say which.
- **Is the current patch faithful?** No. It is a defensible repair of capacity, but it permits more than D trips in one cluster, e.g. one drone making two trips within a cluster, which the literal loop does not.

### Proposed decision
Adopt the literal structure once I-4 exists:
- per cluster, k = 1..D;
- in round k, build candidates, pick the best, and assign it to **physical drone k**;
- feasibility = full evaluator (including (28) for drone k's existing trips);
- stop early if F is empty;
- k resets per cluster.

Off-by-one resolution: use the pre-increment k as the drone index (ASSUMPTION, documented).

**Timing:** it is not required before Phase G, since it only affects the starting solution. It is recommended together with the I-4/I-6 Phase F changes, so that F is changed once.

### Consequences
- Initial solutions may contain fewer drone trips than today (at most D per cluster).
- init B8 and B9 expectations will change (B9's cross-cluster reuse is compatible; B8's reasoning changes).
- `drone_fleet_capacity.hpp` becomes a secondary check.

### Confidence
MEDIUM. The literal loop is clear; mapping f^(k) to physical drone k is an INFERENCE.

---

## "Do not change yet" (the paper gives insufficient information)

Do not invent values for these. Keep them configurable and labelled ASSUMPTION or UNKNOWN until a deliberate choice is recorded.

**VNS (Phase I):**
- Initial temperature T_x.
- Cooling rule `update(T)`.
- iter_max and any secondary stopping criteria.
- Best-so-far tracking policy. Algorithm 1 does not track a best solution; adding it is an INFERENCE to be recorded, not changed silently.
- Algorithm 1 literal defects: iter is never incremented; line 7 always searches from x'.

**Shakes (Phase H):**
- Shake-selection distribution ("randomly selects").
- Random_Exchange segment length.
- Random_Shift segment length and shift distance ("several nodes").
- Random_Drone_Remove reinsertion positions for returned customers.
- Handling of trips whose FLP arcs vanish after a shake (repair vs discard).

**Local search (Phase G):**
- How many times launch/recover relocate are repeated ("several times"), and where in the order.
- The "adjacent" radius for N1–N8.
- Candidate enumeration size.
- Launch/recovery re-selection rules in N5 and N7. *(decided in M2/M3: D4 + A3/A7; see the M3 addendum)*
- N8 insertion search range. *(decided: D5, all valid gaps; see the M3 addendum)*

**Experiments (Phase J/K):**
- Random seed and run protocol beyond "10 runs per instance".
- Depot placement in synthetic instances.
- Coordinate distribution beyond "evenly distributed in 60×60 km".

**Initialisation (Phase F):**
- K-means seeding, stopping rule, empty-cluster handling and tie-breaking.
- BestRoute scoring (keep "lowest arrival time of j").
- Cluster and customer processing order.
- Recovery-node choice (next route node vs a cluster customer, p19).
- The wider vs minimal launch-candidate set (I-6).

**FLP generation (Phase D):**
- The main-experiment FLP spacing d. 2 km is the only paper value (sensitivity).
- Point placement within an arc (m/(n+1) vs multiples of d).
- Truck-only TSP method at 50 customers (heuristic substitute for Cplex).

**Parameters:**
- Drone speed contradiction: 12 m/s (p24) vs "Section 6.1 … 15 m/s" (p31).
- Endurance "25 min = 1800 s" (p26, p29) vs the 1500 s implied by the figures. Sensitivity only.
- e units: minutes in Table 2, seconds in the results.
- Table 3 objective units.

**Timing:**
- Launch-delay slack (I-1).
- Reading A vs B for SR. B is proposed; A is kept as a switch. Do not remove either until reviewed.

---

## Required before Phase G

*These are the decisions and corrections needed before implementing the first neighbourhood (truck service node relocate, the first move in the p24 order).*

### A. Must modify code before G
1. **Timing (I-1):** FLP truck departure = t_i + n_i·SL; every drone launches at t_i + SL.
2. **Timing (I-2):** recovery-node departure per interpretation B, with reading A available as a policy switch. Remove the max(t_k, ·) rule.
3. **Endurance (I-3):** E = (d̂_ij + Se^D_j)(1+θ_j) + max(d̂_jk, t_k − l̂_j) ≤ e.
4. **Drone identity (I-4):** physical `drone_id` ∈ [0, |U|) on every trip. Evaluator checks the id range and each drone's route order launch_pos(next) > recovery_pos(prev) per (28).
5. **FLP definition (I-5):** arc-keyed deterministic L_ab with n = ⌊D_ab/d⌋ points; depot arcs included; d configurable.
6. **Route validity (I-6):** evaluator check that FLPs between a and b belong to L_ab, in ratio order.
7. **Idle FLPs (I-9):** evaluator check that every route FLP has ≥ 1 launch and appears at most once.
8. **Phase F:** launch candidates from resulting-route arcs (I-6); assign physical drone ids (I-4). Needed so the initial solution passes the new evaluator.

### B. Must modify tests before G
1. **test_timing:** recompute hand values affected by I-2 and I-3 (§8, §9, §11, §12–14, Block 9).
2. **New timing tests:**
   - two launches at one FLP;
   - two recoveries at one node (reading B, and A if the switch exists);
   - endurance with hover-wait;
   - endurance with Se^D·θ;
   - cross-arc FLP violation;
   - out-of-order same-arc FLPs;
   - idle FLP;
   - duplicate FLP;
   - drone-id out of range;
   - per-drone overlap violation;
   - valid sequential reuse.
3. **test_flp:** replace the fixed-count expectations with spacing-rule expectations (⌊D/d⌋, depot arcs, zero points when D < d).
4. **test_init:** re-derive B4, B6, B8, B9 and B10 under the new timing, candidates and ids. Rewrite B10, whose premise is the entering-arc candidates. Add a test that no initial solution violates FLP–arc validity.
5. **test_model:** update the `drone_slot` semantics tests.

### C. Must update documentation before G
1. **DECISIONS.md:**
   - replace UNRESOLVED 1, 3 and 4 with the decisions above;
   - mark UNRESOLVED 2 and 5b as "PAPER AMBIGUITY: SR → interpretation B (A as switch)";
   - fix the endurance section (Se^D, waiting, θ scope);
   - fix eq. (41) to ⌊D/d⌋ and the sensitivity-only framing of FLP spacing;
   - relabel t = 0 as [PAPER] (p15);
   - correct the Assumption numbering;
   - record the stray-FLP invalidity and its fix;
   - record FLP-only launches as a resolution of a paper contradiction.
2. **CODE_REPORT.md:** update the timing and Phase F descriptions after the code changes.
3. **README.md:** only if the module descriptions become inaccurate (timing, FLP policy).

### D. Can defer until the affected neighbourhood
- N1: adjacency radius; how often launch/recover relocate repeat and where. *(N1/N2 implemented in M5 with assumptions A13--A18, see the M5 addendum; the repetition is VNS integration, M7)*
- N3: same-drone consecutive-trip exchange (I-8). Needs I-4 first. *(implemented in M4, see the M4 addendum)*
- N5 and N7: launch/recovery re-selection details beyond "FLP on a resulting-route arc". *(decided in M2/M3, see the M3 addendum)*
- N6: exact "adjacent" pairing. *(implemented in M4 with assumptions A8--A10; A8 revised and A8--A11 approved at the M4 sign-off, see the M4 addendum)*
- N8: insertion search.
- Idle-FLP pruning inside each move (I-9 move responsibility).

### E. Can defer until final VNS integration
- Algorithm 2 literal per-cluster drone loop (I-10). Recommended together with item A.8, but not blocking.
- T_x, cooling, iter_max, stopping criteria; best-so-far tracking.
- Shake-selection distribution and segment sizes; Random_Drone_Remove reinsertion.
- Launch-delay slack.
- Seed and run protocol; instance generator details; parameter contradictions (drone speed, endurance seconds); Table 3 units.
- Choice of the main-experiment FLP spacing d for reproduction runs.

---

## M3 addendum: N7 and N8 (9 Nov 2026) --- the items above marked "N5 and N7" / "N8 insertion search range" are now decided

This register is the reconciliation record; the decisions below were taken in the static-reproduction milestones (`DECISIONS.md` items 33--44, `docs/static/operators_N7_N8.md`). The register's older entries are unchanged.

| Topic | Paper fact | Approved decision | Implementation assumption / limitation | Open |
|---|---|---|---|---|
| N7 transformation | p22, Fig. 12a: "randomly selecting a truck service node and then attempting to assign an available drone to construct a drone trip" | D4: best valid FLP on the resulting arcs within the window, recovery at the next truck node; item 15: disturbed trips rejected and named; "available drone" = lowest drone id free over the trip's interval (Issue 4) | **A7**: window measured on the original skeleton (merged arc at $2s_t$), 5 gaps at $w=3$; one `NoReplacementFlp` candidate when no eligible arc has an FLP | A7 centre; Fig. 12a only via its caption |
| N8 transformation | p22--23, Fig. 12b: "randomly selecting a drone trip and then attempting to assign it to the truck route ... feasibility test on the new route ... optimal order of truck visits" | D5: all valid insertion positions (default unrestricted); D6: only the anchor trip is dissolved, its FLP dropped iff unused | "Optimal order" read as the best position of the one inserted customer; optional `insertion_window` | whether more than one customer's position is meant |
| Disturbed trips | silent | item 15 (reject, name, never repair); re-anchoring remains N4-only | N8 valid positions are the gaps without remaining launch FLPs | whether N7/N8 should get a bounded re-anchoring alternative (not authorised) |
| Deviations | none identified | | | |

Measured consequences (no change made): N7 candidates are almost always endurance-infeasible at paper scale (60 km square, 12 m/s, 1,800 s), and disturbed trips dominate N7/N8 at $n=50$ (`operators_N7_N8.md` section 9).

---

## M4 addendum: N3 and N6 (9 Nov 2026) --- Issue 8 (N3) is implemented as proposed; N6 is implemented under explicit assumptions

Decisions: `DECISIONS.md` items 45--53; contracts and evidence: `docs/static/operators_N3_N6.md`. The register's older entries are unchanged except for the "implemented" markers in the I-8 row and in section D.

| Topic | Paper fact | Approved decision | Implementation assumption / limitation | Open |
|---|---|---|---|---|
| N3 transformation | p21, Fig. 10c: "repositioning the drone service node of the drone trip ... searching **adjacent trip** of the original drone trip"; trips $\{C1,C4,C3\}$, $\{C3,C2,C5\}$ become $\{C1,C2,C3\}$, $\{C3,C4,C5\}$, route unchanged, both trips drone 1 | D3: consecutive trips of the same drone are exchanged (this Issue 8 proposal, approved in M0) | the exchange of the two served customers *is* the relocation because every trip serves one customer; A11: predecessor then successor of the anchor are both searched; the D1 window is not used by N3; nothing is disturbed, so no repair/re-anchoring applies | whether "adjacent" could mean more than the consecutive pair (the paper is silent); whether only one direction is intended |
| N6 transformation | p22, Fig. 11b: "randomly selecting a drone trip and searching adjacent drone service node of **other** drone trips"; caption: C2 and C4 exchanged (drone-1 and drone-2 trips) | D1 window 3 (skeleton-counted) as the adjacency measure; item 15 and re-anchoring N4-only | A8: partners are trips of a *different* drone by default (switchable `include_same_drone`; the register's I-8 text already assigns cross-drone exchange to N6); A9: only customers move, launch/recovery/drone ids stay (this register's own reading of N6: Issue 7, "Consequences" table, row "N6 D-D swap"); A10: adjacency = D1 window on the launch-gap half-steps | A8, A9, A10 are readings, not paper facts; Fig. 11b is known only through its caption |
| Randomised entry | "randomly selecting a drone trip" | strict `<`, first generated wins | A12: one `uniform_index(#trips)` draw | |
| FLP selection (D4) | | | *not applicable*: no launch or recovery node changes | |
| Deviations | none identified | | | |

Measured consequences (no change made): with A8, N6 has **zero candidates** on every paper-scale initial solution because all trips fly with drone 0; N3 candidates are mostly endurance-infeasible at paper scale (0 / 0 / 9.1 % feasible at $n=10/20/50$) (`operators_N3_N6.md` sections 8--9).

**M4 sign-off and A8 revision (9 Nov 2026, 12:30 IST).** The user approved M4: **A8 is revised**, so N6's default partner set is *any other trip*, same-drone trips included (`N6Options::include_same_drone = true`; `false` restricts to different drones); N3 and N6 candidates may coincide. **A9, A10 and A11 are approved** as implemented. `DECISIONS.md` item 54. The N6 row above and the "zero candidates" sentence below it describe the *original* A8 default and are superseded: with the revised default N6 has 2 / 8 / 108 candidates (0 / 0 / 4 feasible) on the paper-scale initial solutions at $n=10/20/50$; the restricted variant has none.

---

## M5 addendum: N1 and N2 (9 Nov 2026) --- implemented under explicit assumptions

Decisions: `DECISIONS.md` items 55--65; contracts and evidence: `docs/static/operators_N1_N2.md`. The register's older entries are unchanged except for the section D marker.

| Topic | Paper fact | Approved decision | Implementation assumption / limitation | Open |
|---|---|---|---|---|
| N1 transformation | Sec 5.3 p20--21, Fig. 10a: "launch node relocate ... searching adjacent customer nodes and flexible launch nodes of the drone launch node in original drone trip"; caption: the launch moves from truck node C1 to the flexible node of the C1--Depot segment | Issue 7: launches only at FLPs (the "adjacent customer nodes" text is not implemented); D1 window 3 as the adjacency measure; item 15 (no repair); explicit FLP drop (D6 convention) | **A13** candidates = every FLP on the arcs within $w$ gaps of the old launch gap, current launch excluded, shared and unrouted FLPs included; **A15** launch not before the recovery $\Rightarrow$ `InvalidRecovery` (named, unevaluated, unrepaired); **A16** drone id never changes; **A17** old launch FLP dropped iff unused | extent and direction of "adjacent"; Fig. 10a only via its caption |
| N2 transformation | p21, Fig. 10b: "recover node relocate ... searching adjacent customer nodes of the drone recover node"; caption: the recovery moves from C3 to C4 | D1 window 3; the end depot is a legal recovery node of the model; item 15 | **A14** candidates = skeleton nodes (customers and end depot) within $w$ of the old recovery, ascending, never the start depot; **A15**, **A16** as N1 | same |
| Randomised entry | "several times" during the local search (Sec 6.1) | D2: twice per pass (VNS, M7, not implemented) | **A18**: one `uniform_index(#trips)` draw; an empty window gives status `Ok` with zero candidates | |
| FLP selection (D4) | | | *not applicable*: the candidate set is the whole window, no single FLP is selected | |
| Deviations | the paper's customer launch candidates in N1 | existing Issue 7 FLP-only decision | | recorded in `operators_N1_N2.md` sections 1 and 5 |

**M5 sign-off (9 Nov 2026, 13:19 IST):** A13--A18 approved. The N1 customer-launch omission remains a documented reproduction limitation (no expansion without further paper evidence); the "Deviations" row above is therefore a known limitation, not an open question.

Measured consequences (no change made): at paper scale 80--86 % of evaluated N1 candidates are endurance-infeasible and N2 has at most one feasible candidate per sweep; about half of N1's generated candidates are `InvalidRecovery` (`operators_N1_N2.md` sections 8--9).

---

## Files

- **Created:** `docs/paper1_decision_register.md` (this file).
- **Modified:** none.


## M6 addendum (9 Nov 2026): shaking operators (signed off 13:56 IST)

The shake items listed above as open ("Shake-selection distribution", "Random_Exchange segment length", "Random_Shift segment length and shift distance", "Random_Drone_Remove reinsertion positions", "handling of trips whose FLP arcs vanish after a shake") are now **implemented as documented implementation assumptions** A19--A29 (`docs/static/shakes.md` section 5, `DECISIONS.md` items 67--70), not as paper facts: uniform choice among the four (D6), segments of interior truck customers with $L_{\max}=D_{\max}=3$ by default (configurable, calibration in M7), FLPs follow their arcs, trips invalidated by the shake are dissolved with counted reasons and never repaired, returned customers go to a uniform free gap (fallback: dissolve the trips of a drawn gap). Measured consequences are in `shakes.md` section 8. **M6 signed off (9 Nov 2026, 13:56 IST): A19--A25 and A27--A29 approved; A26 approved with the clarification that only trips whose launch FLPs the reinsertion genuinely invalidates are dissolved, with the affected trips and the exact reason recorded.**


## M7 addendum (9 Nov 2026): VNS driver (signed off 14:34 IST)

The VNS items listed above as open ("iter_max and any secondary stopping criteria", "best-so-far tracking policy", "Algorithm 1 literal defects", "$T_x$ and cooling") are now **implemented as documented implementation assumptions** A30--A38 (`docs/static/vns.md`, `DECISIONS.md` items 72--76), not as paper facts: the printed comparison against $f(x)$ with the shaken solution as search start, SA acceptance, geometric cooling, N1/N2 alternately after the Sec 6.1 order, separate best-so-far, redraw of non-applicable / infeasible shakes, `iter_max` plus optional no-improvement and wall-clock limits, two seed streams by run index. Pilot-based defaults (`iter_max = 2000`, $T_0 = 0.01 f(x_{init})$, $\alpha = 0.995$) are labelled ours (D7). Measured consequences are in `vns.md` section 8. Nothing is signed off; M8 not started.

**M7 signed off (9 Nov 2026, 14:34 IST):** A30--A37 approved; A38 conditionally approved and the reporting condition implemented (`vns.md` section 10).


## M8 addendum (9 Nov 2026): runner, writer, configuration, CLI (signed off 15:35 IST)

The earlier M1 follow-up "TSP heuristic seed (42) and k-means constants to come from config in M8" is closed: they are configuration keys with their former values as defaults. The "10-run protocol driver" is the `experiment.runs` / `first_run_index` mechanism (default 10 runs, our default; not a paper fact). The VNS parameters remain labelled assumptions (A30--A38, pilot defaults, D7); M8 changed none of them. New implementation assumptions A39--A44: `docs/static/runner_cli.md` section 8, `DECISIONS.md` items 78--82. Nothing is signed off; M9 not started.

**M8 signed off (9 Nov 2026, 15:35 IST):** A39--A44 approved; closure items recorded in `MILESTONES.md` and `windows_validation.md`.

**M9 validation (9 Nov 2026, 15:35 IST instruction):** results and the separation of exact reproduction / implementation assumptions / unavailable paper data are in `docs/static/validation_vs_paper.md`. New assumptions A45--A47 (comparison unit minutes; 12 instances per size; exact truck-only optimum as reference). Findings that touch paper facts: the paper's Table 3 objective values are 2--3 times ours (unexplained; not caused by depot position or endurance), the paper's FLP counts (Table 4) are 7.5--11 times smaller than our pool, the paper's coordinates and MILP data are unavailable. No decision of M0--M8 was changed. **M9 signed off (9 Nov 2026, 16:13 IST)** as a validation and comparison milestone, not a numerical reproduction; the objective and FLP-count discrepancies remain open findings (`DECISIONS.md` items 89 and 91). M10 froze the documentation (`docs/static/INDEX.md`).
