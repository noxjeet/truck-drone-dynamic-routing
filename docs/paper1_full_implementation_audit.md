# Paper 1 vs current repository: full implementation audit

**Paper:** Han, B., Chi, T., & Sun, Z. (2026). *Collaborative delivery problem of trucks and multiple drones: an en-route operation with flexible launch positions.* Intl. Trans. in Op. Res. 33, 4072–4109. DOI 10.1111/itor.70045. The uploaded file is a 38-page image-based PDF.

**Repository:** `truck-drone-dynamic-routing`, commit `aebc03f`, as it stood on 27 Sep 2026.

**Audit date:** 27 Sep 2026. This is a read-only audit. It changes no implementation files; this report is the only file created.

**How the paper was read:** every page was rendered to an image and read visually. Pages 12–35 were read at 300 dpi. Pages 1–12 were read at 160 dpi and then re-inspected at 220 dpi. Pages 36–38 (references) were read at 150 dpi. Constraint (18) was re-inspected at 400 dpi. No PDF text extraction, OCR, or other copy of the paper was used. Page numbers below are **PDF page numbers**. PDF page N = journal page 4071+N, so PDF p13 is journal p4084.

**Classification tags** (as requested): [PAPER] explicitly stated · [PAPER + INTERPRETATION] · [INFERENCE] · [ASSUMPTION] · [DEVIATION] · [UNKNOWN] · [UNREADABLE]. Where the paper contradicts itself, both sides are cited and the item is marked **[PAPER CONTRADICTION]**. No such contradiction is resolved here.

---

## 0. Headline findings (read this first)

These are the findings that most change the picture relative to `DECISIONS.md` and the current code. Every one is backed by page references in the sections that follow.

1. **SL is counted once per launching drone in the truck-departure constraint.** Constraint (18) (p13) contains the term Σ_u Σ_{s∈P_i⁺} y_s^u · SL. So a truck launching k drones at a flexible launch point (FLP) i cannot leave before t_i + k·SL. Each individual drone may leave at t_i + SL (constraint 21); nothing in the paper staggers the drones' own departures.
   - *Current code:* the truck leaves the FLP at t_i + SL for any k. This is a **[DEVIATION]** whenever k ≥ 2.
   - *Backup patch:* staggered launches at t_i + m·SL. The paper does not say this either.
   - This settles `DECISIONS.md` "UNRESOLVED 1" at the level of truck departure.
2. **SR in constraint (18) is literally summed over *launches*, not recoveries.** At 400 dpi, both double sums in (18) run over s ∈ P_i⁺, the routes *launched* at i. Two readings follow:
   - *Literal reading:* the truck at a launching FLP pays k·(SL+SR). At a customer, (18) contains no SR at all. Recovery then enters only through constraint (20), which gives each drone its own window: truck departure ≥ that drone's arrival + SR. Those windows are concurrent.
   - *Probable-typo reading (second sum over P_i⁻, the recoveries):* at a customer, l_k ≥ t_k + Se^T_k + r_k·SR. That is additive in both service and number of recoveries, and measured from the truck's arrival.
   - **The paper does not settle which reading is intended.** "UNRESOLVED 2" (SR) and "UNRESOLVED 5b" (Se1 vs SR concurrency) remain open. Neither the current code nor the backup patch matches either reading exactly.
3. **Endurance includes drone service time and hovering while waiting for the truck, and θ applies to service time as well.** Constraints (36) and (37) (p15) multiply (l̂_j − l̂_i) by (1+θ_j). That quantity is launch departure → customer departure, i.e. outbound flight plus Se^D. Constraint (36) adds (t_k − l̂_j), which runs to the *truck's* arrival at the recovery node, so it includes any time the drone spends waiting. The text on p15 and p30 says explicitly that drone waiting counts against the battery.
   - *Current code:* outbound·(1+θ) + return, with Se^D excluded and waiting excluded. `DECISIONS.md` says explicitly that waiting does not count. **[DEVIATION], HIGH impact**: in the main experiments θ = 1, Se^D = 30 s and e = 1800 s.
4. **The paper uses persistent drone identities.** Variables y_s^u, t̂_{i,u} and l̂_{i,u}, constraints (7), (11), (12) and (28), the per-drone trip sequences in Figs. 3 and 7, and Random_Drone_Remove ("a drone is selected at random, and its whole trips are removed", p23) all need to know which physical drone flies which trip.
   - *Current code:* `drone_slot` is a unique counter per trip, and `evaluate_solution()` ignores `num_drones`. That settles the "is capacity/identity needed?" part of UNRESOLVED 3 and 4. Where to enforce it remains a design choice.
5. **The paper states that the number of FLPs on an arc is proportional to its length.**
   - p18: "the number of flexible points inserted is proportional to the truck's driving distance".
   - Every sensitivity experiment uses fixed spacing: 2 km (p26, p28, p31), or eq. (41) with n = ⌊D/d⌋ (p33).
   - Fig. 8c shows 0, 1 or 2 FLPs per arc, including on the depot arcs.
   - *Current code:* a fixed count per arc (default 1, at the midpoint). That is a **[DEVIATION]** in structure. The spacing value for the main experiments is **[UNKNOWN]**.
   - `DECISIONS.md` also misquotes eq. (41) as ⌈D/d⌉; the paper prints ⌊D/d⌋.
6. **The initial solution's FLPs break the paper's arc rule.** The paper allows an FLP of L_ij to be used only if the truck actually traverses arc (i,j) (p9 text; constraints 13–15, p13–14).
   - *Current code:* F4 replaces the converted customer j by an FLP taken from arc (prev, j). The truck then drives prev → FLP → next(j). The FLP is therefore used on an arc it does not lie on (the "stray FLP" `DECISIONS.md` acknowledges).
   - `evaluate_solution()` does not check FLP–arc consistency at all.
   - **[DEVIATION]** from the MILP. The paper's own heuristic description is silent on this.
7. **The post-approval Phase F patch moved *away* from the literal Algorithm 2.** Algorithm 2 (p20) loops `k ← 1; while k ≤ D` *inside* the per-cluster loop. Literally, that allows at most D trips per cluster, with the count reset for each cluster (the old, pre-patch behaviour).
   - The current code has no per-cluster cap; it uses a global, time-aware fleet check instead.
   - That check is a defensible reading of "check if there are any unallocated drones available" (p19), but it is a **[DEVIATION]** from the literal pseudocode.
8. **The paper contradicts itself on launching from customers and the depot.**
   - *No:* Assumption 4 (p10), set condition 1 "i ∈ L" (p11), and Table 1 (p7: this paper = flexible position only, customer location "–") all say drones launch only at FLPs.
   - *Yes:* Figs. 7, 10a, 11a, 11b and 12a, and the solution maps in Figs. 15, 16, 18 and 19, show launches from customer nodes and even the depot. Launch-node relocate (p21) searches "adjacent customer nodes and flexible launch nodes".
   - *Current code:* FLP-only.
   - This matters for Phase G: several neighbourhoods as drawn by the paper cannot be expressed in the current model.
9. **The paper gives none of the VNS parameters.** T_x (initial temperature), cooling `update(T)`, iter_max, the shake-selection distribution, the Random_Shift and Random_Exchange sizes, how many times the relocate neighbourhoods repeat, the definition of "adjacent", and the random seed are all absent. The only protocol stated is 10 runs per instance, reporting best and average (p24). All of these are **[ASSUMPTION]** territory.
10. **The paper contradicts itself on parameters.**
    - Drone speed: 12 m/s in Sec 6.1 (p24), but p31 says "In Section 6.1, drone speed is established at 15 m per second".
    - Endurance: "25 minutes, equivalent to 1800 seconds" (p26, p29). The numbers in Figs. 13 and 14 imply 1500 s was actually used [INFERENCE].
    - Recovery location: "recovery exclusively at truck customer locations" (p8) versus Assumption 5 "service point or the ending depot" (p10); figures show depot recovery.

---

## 1. Page-by-page reading summary

A compact table appears in §16; this is the substantive content.

| PDF p. | Journal p. | Implementation-relevant content |
|---|---|---|
| 1 | 4072 | Abstract: flexible launch points = "multiple non-customer stops in the truck's route"; MILP minimises customer waiting time; VNS. |
| 2 | 4073 | Intro. Drone range ≈ 15 km (context only). |
| 3 | 4074 | "Multiple drone launch points **evenly distributed** across the truck's route, permitting the truck to stop and launch **one or more drones** at these points." Objective = time for truck/drone to reach customers. MILP solvable for < 10 customers. |
| 4–6 | 4075–77 | Paper structure; literature review. No implementation detail. |
| 7 | 4078 | Table 1: "This paper" = WT objective, MD, launch = FP only (CL "–"), flexible type PN (predefined node). |
| 8 | 4079 | Fig. 1: FLP "a" on arc C1→C2 launches a drone to C3 (the FLP is *not* on the arc entering the served customer). Text: launches "at any two customer locations or **between a customer location and the depot**"; FLPs divide the segment "by **equal proportions** of the distances between two locations"; "drone recovery **exclusively at truck customer locations**". |
| 9 | 4080 | Fig. 2: 4 evenly spaced FLPs on one arc; two drones launched from two different FLPs on it. Definitions of G, A, C, L, D = {0, c+1} (same physical depot). Drones make multiple takeoffs and recoveries. The truck does not wait stationary during flights. "If the truck does not pass through arc (i,j), it cannot access the launch point in L_ij." Drone time includes SL, SR and Se^D. Objective Σt_i + ΣΣt̂_{i,u}. |
| 10 | 4081 | Fig. 3 worked example (2 drones, 3 trips each, both launching from FLP e, both recovering at C3 and at C7, one recovery at the depot). Assumptions 1–8. |
| 11 | 4082 | Table 2 notation (e "in minutes", SL "launch time", SR "recovering time", Se^T_i, Se^D_i, θ_j, x, y_s, p, z, t, l, t̂, l̂). Single-flight tuple s = ⟨i,j,k⟩ with i ∈ L, j ∈ C. |
| 12 | 4083 | Condition 3: k ∈ C_E, k ≠ j, a truck service point or the ending depot. P_i⁺, P_j, P_k⁻. Objective (1); constraints (2)–(8). |
| 13 | 4084 | Constraints (9)–(23), including the timing block (16)–(22). |
| 14 | 4085 | Constraints (24)–(34), with the text explaining (1)–(16). |
| 15 | 4086 | "Operating time starts at 0"; t̂ = 0 for truck-served customers; explanation of (18)–(30); waiting counts toward endurance; eqs. (35)–(37); start of prohibited operations. |
| 16 | 4087 | Figs. 4–6: prohibited operations (launch and recover at the same FLP; depot → depot); allowed recovery at *any* later truck node or the depot; multiple drones from different FLPs on one arc, or from the same FLP. |
| 17 | 4088 | Algorithm 1 (VNS with SA acceptance) and its description. |
| 18 | 4089 | Fig. 7 representation (block encoding; per-drone trip lists). "Number of flexible points inserted is **proportional to the truck's driving distance**." Initialisation steps (a)–(d); Cplex for (a). |
| 19 | 4090 | Fig. 8 initialisation illustration (variable FLP count per arc, depot arcs included). k = ⌈C/(D+2)⌉. "Predefined flexible launch point partitioning ratio." Launch and recover nodes "correspond to the customer point in the cluster or the attached flexible launch point." Narrative of Algorithm 2. |
| 20 | 4091 | Algorithm 2 pseudocode. Local search: 8 neighbourhoods in 3 types. Start of launch-node relocate. |
| 21 | 4092 | Fig. 10 (relocate-based moves a–d) and their text. |
| 22 | 4093 | Fig. 11 (swap-based moves) and text; deletion–reinsertion text. |
| 23 | 4094 | Fig. 12; the 4 shake operators; Java / Cplex 12.6.3 environment. |
| 24 | 4095 | Instances (60×60 km, 4 datasets × 10/20/50 customers); **all main parameters**; local-search order; 10 VNS runs per instance. |
| 25 | 4096 | Table 3 (Cplex vs VNS), Table 4 (FLP counts 214/837/3493), eqs. (38)–(39). |
| 26 | 4097 | FLP count 3493 exceeds MILP capability. Dalian sensitivity setup (8 customers, 20×20 km, 15 m/s for both, **2 km FLP interval**, "25 min = 1800 s"). Cost eq. (40). |
| 27 | 4098 | Table 5 (cost symbols); Fig. 13 (waiting in **seconds**). |
| 28 | 4099 | Fig. 14 table (endurance 900/1500/2100 s); "launching point is divided into 2 km". |
| 29 | 4100 | Fig. 15 (routes; a drone trip drawn from the depot). The truck waits for the drone within its return range. |
| 30 | 4101 | Fig. 16. "Endurance ... insufficient for Drone 1 ... **including waiting for the truck at the merging point**." |
| 31 | 4102 | Fig. 17 (drone speed 12/15/18); "In Section 6.1, drone speed is established at 15 m/s" (contradicts p24); 2 km intervals. |
| 32 | 4103 | Figs. 18–19 (routes; launches drawn from customers and the depot). |
| 33 | 4104 | Table 6 (unit division length); Sec 6.3.4: 7 customers, 15 m/s, 30 min; eq. (41) n = ⌊D/d⌋, d ∈ [1000, 5000] step 500. |
| 34 | 4105 | Fig. 20 (FL vs TSP-D vs TSP-MD); denser FLPs → lower waiting. Conclusion begins. |
| 35 | 4106 | Conclusion: 5–15 % reduction; "requires **pre-determining the number of flexible launch points** along the truck route"; **one customer per flight**. Acknowledgments; references begin. |
| 36–38 | 4107–09 | References only (Ibroska et al. 2023 = source of the neighbourhood order; Moshref-Javadi et al. 2020 = base MILP; Luo et al. 2021 = representation). |

---

## 2. Repository inventory (verified, not assumed)

| Item | State |
|---|---|
| Branch / HEAD | `main` @ `aebc03f` (= origin/main) |
| Committed implementation | Phases A–F: `include/{model,graph,flp,timing,init,tsp}`, `src/tsp`, 6 test files. |
| Uncommitted | `CODE_REPORT.md` (user edit, 13 Sep); `DECISIONS.md` (documentation-only update, 27 Sep: Phase F sync plus "Pre-Phase-G baseline"). |
| Untracked | `reports/midsem/` |
| Ignored / stale | `build/` (its `test_timing.exe` and `test_init.exe` were built on 26 Sep from since-reverted code), `build-linux-check/`, `Testing/` |
| Empty | `config/`, `instances/`, `results/`, `scripts/` |
| Planning documents in the repo | **None** beyond `README.md` ("8 local-search + 4 shake neighborhoods, the VNS driver, the experiment runner is designed but not yet implemented") and the `DECISIONS.md` Pre-Phase-G section. `paper1-implementation-plan.md` and `paper1-understanding.md` are cited but absent. The 26 Sep spec documents exist only in the backup folder *outside* the repo; they are compared in §10 as external, unverified material. |
| Tests (last fresh build, 27 Sep) | 6 executables, 339 runtime assertions, all passing. |

### 2.1 What the current code does (Phases A–F)

- **A – TSP.** Held-Karp, exact for n ≤ 21 nodes including the depot, so up to 20 customers. Above that, a nearest-neighbour + 2-opt + Or-opt heuristic (8 restarts, seed 42). The factory records exact/heuristic in `TSPResult`.
- **B – Model.** `Node` (Depot, Customer, FLP), `NodeRegistry`, `CustomerData{Se1, Se2, θ}` per customer, `FlexibleLaunchPointData{arc_from, arc_to, ratio}`, `TruckParameters{speed}`, `DroneParameters{speed, endurance_seconds, SL, SR, num_drones}`, `DroneTrip{drone_slot, launch, customer, recovery, unused position fields}`, `TruckRoute` (vector of node ids; the start and end depot share one id), `Solution{route, trips}`.
- **C – Geometry.** Euclidean distance; time = distance / speed (+∞ if speed ≤ 0); `SpatialNetwork` supplies distance and time queries and matrices.
- **D – FLPs.** `points_per_arc` (default 1) points at ratios k/(n+1) on every consecutive pair of the *given* route, including the depot arcs. Generation is pure and deterministic.
- **E – `evaluate_solution()`.**
  - Truck: t₀ = 0; t_p = l_{p−1} + d.
  - At an FLP with ≥ 1 launches: l = t + SL, and every launching drone departs at t + SL.
  - At a customer or depot: l = max(t + Se1, max over recovering drones of [max(t, drone landing) + SR]).
  - Drone: t̂_j = launch + raw outbound; l̂_j = t̂_j + Se2; landing = l̂_j + raw return.
  - Endurance: raw outbound·(1+θ) + raw return ≤ e.
  - Checks: route starts and ends at a depot; launch position strictly before recovery position; each customer served exactly once.
  - Objective: Σ arrival time of the vehicle serving each customer.
  - **Not checked:** num_drones / drone identity, FLP–arc consistency, FLPs visited without a launch, repeated nodes.
- **F – `build_initial_solution()`.**
  - F1: TSP over the truck-time matrix.
  - F2: deterministic K-means with k = ⌈|C|/(|U|+2)⌉.
  - F3: FLPs generated once from the TSP route and frozen. The launch candidates for customer j are *all* FLPs on the arc entering j.
  - F4: clusters in id order. Each round evaluates every unconverted, unlocked customer × every launch candidate. Recovery = next customer or depot on the working route. The converted customer's route slot is replaced by the FLP. A candidate is kept only if it passes the full evaluation and the global fleet check. The lowest customer arrival wins (ties by customer id, then launch id). `drone_slot` = counter. A customer recovery node is locked. The final solution is re-evaluated.

### 2.2 What is not implemented

Phase G (8 local-search neighbourhoods), H (4 shakes), I (VNS with SA), J (instance generator and experiment runner), K (validation against Tables 3/4) and Stage 2 (out of scope) are all absent. There is no program other than the tests, and no instance files.

---

## 3. Mathematical model audit: constraint by constraint

"Code" names the place where the behaviour is implemented, or "—" if nowhere. "Test" names the test file and block.

| Eq. (page) | What it says (as printed) | Meaning | Code | Test | Status / gap |
|---|---|---|---|---|---|
| (1) p12 | min Σ_{i∈C} t_i + Σ_{i∈C} Σ_{u∈U} t̂_{i,u} | Total customer waiting time. t̂ = 0 for truck-served (p15). t_i for a drone-served customer is unconstrained, so it is minimised to 0 [INFERENCE]. | `evaluate_solution` objective | timing §1&6, §12-14; init B6-B10 | [PAPER + INTERPRETATION]. Matches. |
| (2)–(4) p12 | Truck leaves depot 0 once, enters c+1 once, flow conservation over C ∪ L | Single depot-to-depot path | Route vector; start/end depot check | timing §1 | [PAPER]. The code checks endpoints only; flow is implicit in the path representation. |
| (5) p12 | Launches at i only if the truck leaves i | No launch from an unvisited FLP | `on_route` (launch node must be in the route) | init B5; timing | [PAPER]. Matches. |
| (6) p12 | Σ launches at i ≥ Σ_h x_{i,h} | **An FLP visited by the truck must launch at least one drone** (no idle FLP stops) | — (F4 never creates idle FLPs; the evaluator allows them) | none | Evaluator gap. Matters for Phase G moves that remove trips. |
| (7) p12 | For each u: recoveries of drone u at k ≤ x-in(k) | Drone u recovers only where the truck visits | `on_route` (recovery in route) | timing, init B5 | [PAPER]. Matches, but the per-drone index is absent. |
| (8)–(9) p12–13 | Each FLP entered at most once and left at most once | FLP visited at most once | — | none | Not checked. Low. |
| (10) p13 | Σ_{i∈C} x_{ij} + Σ_{h∈L} x_{hj} + Σ_u Σ_{s∈P_j} y_s^u = 1 | Each customer served exactly once (truck or one drone route) | `CustomerServiceRecord` | timing §15; init B6 | [PAPER]. Matches. (The printed index set omits truck arcs from depot 0, x_{0j}. Literal typo; intent clear.) |
| (11) p13 | For each drone u and FLP i: ≤ 1 launch | A drone launches at most once per FLP; **different drones may share an FLP** | — (no drone identity) | none | Several launches per FLP are allowed by the code, per-drone uniqueness is not enforced. |
| (12) p13 | For each drone u and node k: ≤ 1 recovery | A drone is recovered at most once per node; several drones may recover at the same node | — | none | As above. |
| (13)–(15) p13 | x_{k,j} = 0 and x_{i,k} = 0 for k ∈ L \ L_ij; x_{k,g} = 0 for k ∈ L_ij, g ∈ L \ L_ij | **An FLP of L_ij sits strictly between i and j; the truck cannot move between FLPs of different arcs** (text p14: truck may not go FLP → other customers, customer → other FLPs, or FLP → FLP). The printed quantifiers are loose; the text gives the intent. | — | none | **Not enforced.** F4 routes violate it (see D-06). [PAPER]. The text of (15) is unqualified, so whether consecutive FLPs *on the same arc* are allowed is **[UNKNOWN]**; Fig. 3 shows a→b consecutively. |
| (16) p13 | l_j + d_ij − (1−x_ij)M ≤ t_j | Truck arrival = previous departure + travel. **Printed "l_j" is presumably a typo for l_i.** | `arrival[p] = departure[p−1] + truck_time` | timing §1 | [PAPER + INTERPRETATION]. Matches. |
| (17) p13 | l̂_{i,u} + d̂_ij − (1−Σ y) M ≤ t̂_{j,u} | Drone arrival at customer ≥ launch departure + flight | `customer_arrival = launch + raw outbound` | timing §2-5 | [PAPER]. Matches (equality at the optimum). |
| (18) p13 | t_i + Se^T_i + Σ_u Σ_{s∈P_i⁺} y·SL + Σ_u Σ_{s∈P_i⁺} y·SR ≤ l_i, for i ∈ C₀ ∪ L | Truck departure lower bound. **Both sums are over routes launched at i** (verified at 400 dpi). | FLP: l = t + SL; customer: max(Se1, SR windows) | timing §2-5, Block 9 | See §4. SL is per drone (**deviation for k ≥ 2**); the SR placement is literally at the launch node (**unresolved**). |
| (19) p13 | l̂_{i,u} − (1−Σy)M ≤ l_i | Every drone launched at i departs no later than the truck | Implicit (launch = t + SL ≤ l) | timing | [PAPER]. Matches. |
| (20) p13 | l̂_{j,u} + d̂_{jk} + SR − (1−Σy)M ≤ l_k | Truck leaves recovery node k only after **each** drone's arrival + SR. Per drone and independent, so concurrent windows. | `recovery_complete = max(t_k, landing) + SR`; l ≥ that | timing §2-8, Block 9 | [PAPER + INTERPRETATION]. The code adds max(t_k, ·) (Assumption 7 made physical); the MILP has only landing + SR here. |
| (21) p13 | t_i + SL − (1−Σy)M ≤ l̂_{i,u} | **Each** drone departs ≥ truck arrival + SL (one SL per drone, measured from truck arrival, no staggering) | `launch_time = arrival + SL` | timing §2-5 | [PAPER]. Matches for the drones' own departure times. |
| (22) p13 | t̂_{j,u} + Se^D_j − (1−Σy)M ≤ l̂_{j,u} | Drone service time at the customer | `customer_departure = arrival + Se2` | timing §2-5 | [PAPER]. Matches. |
| (23)–(27) p13–14 | MTZ positions z and precedence p_ij | Route ordering; p_ij = 1 iff i before j | Route index order; `find_route_position_from` | timing, init B5 | [PAPER]. Matches through the representation. |
| (28) p14 | t_k − M(3 − p_ig − Σ_{s∈P_i⁺∩P_k⁻} y^u − Σ_{s∈P_g⁺} y^u) ≤ l̂_{g,u} | **Drone reuse:** if drone u flew i→…→k and later launches at FLP g (after i), then its departure at g ≥ **t_k, the truck's arrival** at the recovery node (not + SR). k ∈ C. | F4 only: global interval check with busy interval [launch, max(t_k, landing) + SR] | init B7-B9 | [PAPER]. Evaluator: **absent**. F4: the release time is later than the paper's (t_k), so F4 is more conservative [DEVIATION, LOW]. |
| (29)–(30) p14 | Endurance with θ = 0 | Replaced by (36)–(37) | — | — | See (36)–(37). |
| (31)–(34) p14 | Variable domains; z ≤ c+1+\|L\| | — | Types | — | Fine. |
| (35) p15 | t̄_u = t_u (1+θ_j), θ_j ≥ 0 | Generalised flight time for a loaded flight | `theta_used` | timing §11 | See (36)–(37). |
| (36) p15 | e + M(1−y_s) ≥ (t_k − l̂_{j,u}) + (l̂_{j,u} − l̂_{i,u})(1+θ_j) | Endurance up to the **truck's arrival** at k, so it includes waiting. θ multiplies outbound **+ Se^D**. | outbound·(1+θ) + return | timing §9, §11 | **[DEVIATION]** (D-03). |
| (37) p15 | e + M(1−y_s) ≥ (l̂_{j,u} − l̂_{i,u})(1+θ_j) + d̂_{jk} | Endurance when the truck arrives first | Same as above | timing §9, §11 | **[DEVIATION]**: Se^D is excluded and not multiplied by θ. |

**Also in the model or its text:**
- Drone serves one customer per flight (p11, p35): [PAPER], matches.
- Recovery may not be at an FLP, and launch and recovery may not be at the same FLP (condition 3; Fig. 4): matches via `has_valid_node_roles`.
- No depot→depot drone flight (Fig. 4 right): matches, because launch must be an FLP.
- Recovery at *any* later truck node or the depot (p16, Fig. 5): the evaluator matches; F4 restricts it (see §8).

---

## 4. Timing model: deep audit

### 4.1 Paper behaviour, stated mathematically (no "correction")

Let n_i be the number of drones launching at FLP i, and r_k the number of drones recovering at node k.

| Quantity | Paper | Source |
|---|---|---|
| Start time | Truck operating time starts at 0 | p15 text [PAPER] |
| Truck arrival | t_j ≥ l_i + d_ij | (16) |
| Truck departure at FLP i (launch node) | Literal: l_i ≥ t_i + Se^T_i + n_i·SL + n_i·SR, where Se^T_i is presumably 0 for an FLP. Typo reading: l_i ≥ t_i + n_i·SL. **In both readings SL is multiplied by the number of launching drones.** | (18) |
| Each drone's launch departure | l̂_{i,u} ≥ t_i + SL and l̂_{i,u} ≤ l_i. **No ordering between drones**; all may depart at t_i + SL. | (21), (19) |
| Drone arrival and service at customer | t̂_{j,u} ≥ l̂_{i,u} + d̂_ij; l̂_{j,u} ≥ t̂_{j,u} + Se^D_j | (17), (22) |
| Truck departure at customer / depot 0 | Literal (18): l_k ≥ t_k + Se^T_k (no SR term). Typo reading: l_k ≥ t_k + Se^T_k + r_k·SR. | (18) |
| Recovery coupling | l_k ≥ l̂_{j,u} + d̂_jk + SR **for each** recovering drone u, independently | (20) |
| Synchronisation | "First vehicle to arrive at a service point must wait for the other" | Assumption 7 (p10) |
| Waiting and endurance | Drone waiting for the truck **counts** as flight time | (36); p15 text; p30 text |
| Drone reuse | Relaunch of u at g ≥ t_k (truck arrival at u's previous recovery node) | (28) |
| Objective | Σ customer arrival time of the serving vehicle | (1), p15 |

**Is (18) elapsed operations or departure accounting?** It is a lower bound on the truck's departure variable l_i, i.e. **truck-departure accounting**. The drones' own departure times come separately from (19) and (21). So in the paper's MILP, k drones launched from one FLP can all leave at t_i + SL while the truck is held until t_i + k·SL, or t_i + k·(SL+SR) under the literal reading. The paper does not describe the physical sequence behind this.

### 4.2 Current code vs paper, item by item

| Item | Paper | Current code | Class | Impact |
|---|---|---|---|---|
| Truck arrival | (16) | Same | MATCH | — |
| Truck service | Se^T_i in (18) | `arrival + Se1` | MATCH | — |
| Drone launch preparation | SL per drone from truck arrival (21); truck held n_i·SL (18) | Drones: t+SL ✓; truck held one SL | **DEVIATION** (truck side, n ≥ 2) | MEDIUM: affects every downstream arrival time whenever 2 or more drones share an FLP. The initial solution never does this; Phase G and the paper's own Figs. 3 and 6 do. |
| SR at the launch node | Literal (18): n_i·SR added at the launch FLP | None | **UNKNOWN** (typo or intended?) | MEDIUM |
| Drone flight to customer | (17) | Raw outbound | MATCH | — |
| Drone customer service | (22) | Se2 | MATCH | — |
| Drone return and recovery | (20): l_k ≥ landing + SR per drone | l_k ≥ max(t_k, landing) + SR per drone | INTERPRETATION | LOW–MEDIUM. The code additionally forces l_k ≥ t_k + SR whenever any drone recovers, even if it landed long before. The MILP does not force this. The physical intent of Assumption 7 supports the code. |
| Truck service vs recovery | Literal: max(t+Se^T, landing+SR); typo reading: t+Se^T+r·SR, plus (20) | max(t+Se1, max(t, landing)+SR) | INTERPRETATION (matches the literal reading, not the typo reading) | MEDIUM |
| Truck service vs launch | Launches only at FLPs (Assumption 4), so no interaction | Same | MATCH (but see the figures contradiction, D-09) | — |
| Multiple launches at one node | Allowed (Fig. 3 FLP e; Fig. 6; eq. (11)) | Allowed; one shared SL | DEVIATION (SL count) | MEDIUM |
| Multiple recoveries at one node | Allowed (Fig. 3 C3, C7; eq. (12)); each gets its own (20) window | Allowed; concurrent windows | Literal reading: MATCH. Typo reading: DEVIATION. | MEDIUM |
| Depot timing | 0 start; end-depot arrival counts toward nothing (not in C) | Same | MATCH | — |
| First-arriving vehicle | Assumption 7 | max(t, landing) | MATCH [INFERENCE] | — |
| Waiting time in endurance | Counts ((36), p15, p30) | Does not count | **DEVIATION** | **HIGH** |
| Se^D in endurance | Counts and is multiplied by (1+θ) ((36)–(37)) | Not counted | **DEVIATION** | **HIGH** with θ = 1: +60 s per trip against a 1800 s budget, plus waiting |
| θ scope | (l̂_j − l̂_i)(1+θ) = (outbound + Se^D + slack)·(1+θ); return unscaled | Outbound only | DEVIATION (partial) | HIGH (as above) |
| Endurance reference point | Up to t_k (truck arrival), or l̂_j + d̂_jk if the truck is first | Up to drone landing | DEVIATION | HIGH |
| Fleet size / drone reuse | Built into the model via u ∈ U, (11), (12), (28) | Evaluator: not enforced; F4: separate check | DEVIATION (evaluator scope) | HIGH for Phase G (moves can create over-capacity solutions that the evaluator reports as feasible) |
| Objective | (1) | Sum of serving-vehicle arrival | MATCH | — |
| t = 0 | [PAPER] p15 | Labelled [ASSUMPTION] in code and in DECISIONS | Doc contradiction only | NONE |

### 4.3 Effect on the `DECISIONS.md` "UNRESOLVED" list

| # | Question | What the paper settles |
|---|---|---|
| 1 | Shared SL or k×SL | **Resolved for truck departure: k×SL** ((18), both readings). Drone departures: each drone ≥ t_i + SL, with no staggering implied ((21)). The current code deviates on the truck side. The backup's staggering is *not* paper-supported. |
| 2 | Concurrent SR or k×SR | **Not resolved.** Literal (18) sums SR over launches at the launch node; (20) gives per-drone concurrent windows at the recovery node. Only the "P_i⁻ typo" reading gives k×SR at the recovery node. Mark as [UNREADABLE / UNRESOLVED intent]; the print itself is legible. |
| 3 | Should the evaluator enforce num_drones? | **The paper requires fleet-size feasibility as part of the model** (u ∈ U indexing, (11), (12), (28)). *Where* to enforce it is a design choice. |
| 4 | Drone identity for Random_Drone_Remove | **Resolved: persistent physical drone identity** (per-drone variables, Figs. 3 and 7, p23 wording). The current `drone_slot` counter does not provide it. |
| 5a | θ scope | **Resolved: θ multiplies launch-departure → customer-departure** (outbound + Se^D), not the return leg ((36)–(37)). The current code deviates by excluding Se^D. |
| 5b | Se1/SR concurrency | **Not resolved.** Literal reading gives concurrent (max); the typo reading gives additive. |
| new | Waiting in endurance | **Resolved: waiting counts** ((36), p15, p30). The current code and `DECISIONS.md` say the opposite. |

---

## 5. FLP generation audit

| Question | What the paper says | Class |
|---|---|---|
| Where are FLPs generated? | Along the truck route, "evenly distributed" (p3, p9); "between any two customer locations or between a customer location and the depot along the truck route" (p8). In the MILP, L_ij exists for **every** pair i ∈ C₀, j ∈ C_E (Table 2, p11), and is usable only if arc (i,j) is traversed (p9). | [PAPER] |
| How many? | "Proportional to the truck's driving distance" (p18). Sensitivity: 2 km interval (p26, p28, p31); eq. (41) n = ⌊D/d⌋, D = distance between two customer locations, d = 1000–5000 m (p33). The main-experiment spacing is not given. | [PAPER] for proportionality; **[UNKNOWN]** for the main-experiment value |
| All arcs or only truck-route arcs? | Initialisation: FLPs inserted "in the route of existing trucks" (p18–19, step c). MILP: all arcs. Fig. 8c: only route arcs. | [PAPER] (init = route arcs) |
| Ratios | "Equal proportions of the distances" (p8); "equitably divided" (p33). Whether n points sit at i/(n+1) or at multiples of d from the start is not stated. | [ASSUMPTION] needed |
| Depot arcs included? | Yes (p8 text; Fig. 8c; Fig. 3 FLP "a" on the depot arc). Fig. 7 omits them but is schematic. Eq. (41) wording says "two customer locations". | [PAPER] (included), with minor [PAPER CONTRADICTION] |
| Closing arc included? | Yes: "between a customer location and the depot" (p8) | [PAPER] |
| Dynamic or once? | Initialisation inserts once after the TSP route (p18–19). During VNS the route changes (shakes reverse, exchange and shift segments), and FLPs of new arcs must be usable. The MILP defines L_ij for all arcs, and p35 says the number of FLPs is "pre-determined". **How the VNS handles FLPs on arcs created by moves is not described.** | [UNKNOWN] |
| Multiple per arc expected? | Yes (Fig. 2: 4; Fig. 6; Fig. 7: 3 per arc; Fig. 8c: 0–2) | [PAPER] |
| Table 4 | 214 / 837 / 3493 FLPs for 10 / 20 / 50 customers (p25); the paper does not give the generating rule. My arithmetic (not the paper's): ratios per unordered node pair are ≈ 3.9 (10 customers, 55 pairs), ≈ 4.0 (20 customers, 210 pairs), ≈ 2.7 (50 customers, 1275 pairs). Counts grow roughly quadratically, which suggests generation over many or all node pairs rather than only the TSP route. No single tested rule reproduces all three values. | [UNKNOWN] |

**Current implementation vs the paper:**
- *Fixed count per arc (default 1, midpoint).* **[DEVIATION]** in structure: the paper says the count is proportional to arc length, and every numeric rule it gives is spacing-based. The value is an **[ASSUMPTION]**.
- *Only TSP-route arcs, generated once, frozen.* Matches initialisation step (c). **Insufficient for Phase G/H**, because the route changes.
- *Includes depot arcs.* MATCH.
- *Ratios k/(n+1).* A reasonable reading of "equal proportions" [INFERENCE].
- *Launch candidates for customer j = FLPs on the arc entering j.* **[ASSUMPTION]**, and narrower than the paper. Fig. 1 launches from an FLP on arc C1→C2 to serve C3, which is not that arc's endpoint. Fig. 3 FLP b serves C2 from a different arc. The paper's MILP lets any FLP on the route serve any customer, subject to endurance.

**Contradictions with `DECISIONS.md` (documented, not fixed):**
- (a) `DECISIONS.md` quotes eq. (41) as n = ⌈D/d⌉; the paper prints ⌊D/d⌋ (p33).
- (b) `DECISIONS.md` says the paper gives no rule for the main experiments and treats spacing as sensitivity-only. p18 states the proportional-to-distance principle generally, and 2 km is used in *all* of Sec 6.3.1–6.3.3, not just 6.3.4.
- (c) `DECISIONS.md` describes the frozen-pool "stray FLP" as a harmless inefficiency. Under the paper's MILP ((13)–(15), p9) such a route is infeasible (D-06).

---

## 6. Initial solution audit

| Stage | Paper | Current code | Class |
|---|---|---|---|
| (a) Truck-only route | "Generate an optimal route for truck-only service"; Cplex (p18); Fig. 8a "best TSP truck route" (p19) | Held-Karp exact ≤ 20 customers; heuristic at 50; minimises truck time (= distance/speed) | [PAPER + INTERPRETATION]. The CPLEX substitute is exact for 10/20 customers and not exact for 50 (environment limitation). The paper does not state whether "optimal truck-only" means minimum length or minimum latency; the Fig. 8 caption says TSP. |
| (b) K-means: k formula | k = ⌈C/(D+2)⌉, D = number of drones (p19) | Same | [PAPER] MATCH |
| (b) Initialisation, metric, stopping, empty clusters, ties | Not specified (clusters "according to their distribution locations") | Deterministic id-spaced seeds; squared Euclidean distance; stop on no change / ε / 100 iterations; empty clusters kept; lowest index wins ties | [ASSUMPTION] (Euclidean metric: [INFERENCE]) |
| (c) FLP insertion | "According to the predefined flexible launch point partitioning ratio" (p19); value not given | 1 per arc | [ASSUMPTION] + structural [DEVIATION] (§5) |
| (d) Candidate generation | For each node of the current cluster, "design a drone service trip"; the launch and recover node must "correspond to the customer point in the cluster or the attached flexible launch point" (p19) | Launch: every FLP on the arc entering j. Recovery: next customer/depot on the working route. | Launch: [ASSUMPTION] ("attached" undefined). Recovery: **[DEVIATION risk]**: the recovery node may lie outside the cluster or be the depot, and the paper says the nodes must correspond to cluster customer points. Also, the p19 wording can be read as allowing *customer* launch nodes [UNKNOWN]. |
| (d) Feasibility check | "If f is feasible" (undefined) | Full `evaluate_solution` + global fleet check | [ASSUMPTION] (inherits the E deviations: endurance, SL) |
| (d) BestRoute | Undefined (p20) | Lowest arrival time of j; ties by id, then launch id | [ASSUMPTION]. An alternative, minimum total objective change, is equally plausible. |
| (d) Ordering | Clusters "successively" (p19); pseudocode i = 1..C | Ascending cluster index; customers ascending id | [ASSUMPTION] |
| (d) Global or per cluster | Per cluster (pseudocode lines 1–18) | Per cluster | MATCH |
| (d) Drone availability | `k ← 1; while k ≤ D` **inside** each cluster (lines 3–4, 13): at most D trips per cluster, reset per cluster. Text: "checks if there are any unallocated drones available" (p19). | No per-cluster cap; global time-aware capacity (post-approval patch) | **[DEVIATION]** from the literal pseudocode (D-07). Arguably closer to the physical intent, but not what is printed. |
| (d) Drone reuse | Allowed in general (p9, Fig. 3); for Alg. 2 only implied by the per-cluster reset | Allowed across clusters when intervals do not overlap | [INFERENCE] |
| (d) Simultaneous drones | Allowed in general | Allowed up to \|U\| | MATCH |
| (d) Exhaustive or restricted | One trip "for each node" per round; no search over launch/recovery alternatives stated | All same-arc launch FLPs × one recovery | [ASSUMPTION] |
| (d) Route update | Implied: drone-served customer removed from truck route (Fig. 8d) | Customer slot replaced by FLP | [INFERENCE]; creates D-06 |
| Final evaluation | Not described | Independent re-evaluation | [ASSUMPTION] (harmless) |
| Invented details | — | Recovery locking; frozen pool; tie-breaks; next-node recovery; score | [ASSUMPTION] |

**Literal defects in Algorithm 2 (p20).** These are legible but ill-defined; the paper does not resolve them, so they are marked [UNRESOLVED]:
- `j` is never initialised.
- `i` is never incremented.
- Line 7 `f ← n^(k)` indexes the node list by the drone index, where `n^(j)` is presumably meant.
- Line 9 `F ← f` presumably means "append".
- The empty-F case is unhandled.
- "Feasible" and "BestRoute" are undefined.

---

## 7. Optimisation / VNS audit (paper content)

### 7.1 Algorithm 1 (p17), literal

```
Input: T_x, iter_max, N
1  x ← InitialSolution(); 2 T ← T_x; 3 iter ← 1; repeat
4    x' ← RandomShake(x); 5 k ← 1
6    while k ≤ |N|:
7      x'' ← N^(k)(x')
8      if f(x'') < f(x): x ← x''; k ← 1
11     elif Random(0,1) ≤ exp((f(x) − f(x''))/T): x ← x''; k ← k+1
14     else: k ← k+1
18   T ← update(T)
19 until iter > iter_max
```

Literal issues to record (not to "fix" silently):
- `iter` is never incremented.
- Line 7 always searches from x', the shaken solution, not from the updated x.
- The comparison in line 8 is against the current x.
- No best-so-far solution is tracked or returned, even though SA can accept worse solutions.
- `update(T)` is not defined.

Text (p17): improvement → return to the first neighbourhood; otherwise → next neighbourhood; stop at the iteration limit "or the stopping criteria are met" (other criteria are not defined).

### 7.2 What the paper specifies vs leaves open

| Aspect | Paper | Class |
|---|---|---|
| Initial solution | Two-stage exact + heuristic (Alg. 2) | [PAPER] |
| Shake selection | "Randomly selects a certain neighborhood" each iteration (p23) | Distribution [ASSUMPTION] |
| Local-search order | Truck service node relocate → T-D swap → Drone node delete / Truck node insert → Drone service node relocate → D-D swap → Truck node delete / Drone node insert (p24) | [PAPER] |
| Launch/recover relocate | "Performed several times during the local search process" (p24) | Count and placement [ASSUMPTION] |
| Acceptance | Improvement, or Random ≤ exp((f(x) − f(x''))/T) | [PAPER] |
| T_x, cooling, iter_max | **Not given anywhere in the 38 pages** | [ASSUMPTION] |
| Stopping criteria besides iter_max | Mentioned, not defined | [UNKNOWN] |
| Best-improvement within a neighbourhood | "Ascertaining the most optimal relocation location ... by calculating the updated solution cost" (p20–22) | [PAPER] (best of the searched candidates) |
| Candidate sets | "Adjacent" nodes / trips (undefined radius); several moves start from a **random** trip or node | [PAPER] for random-start moves; radius [ASSUMPTION] |
| Feasibility handling | "After performing a feasibility test on the new route" (p22, move 8); otherwise implied | Discard vs repair [ASSUMPTION] |
| Tabu / visited list | None mentioned | Not part of the paper |
| Randomisation / seed | Not given; 10 runs per instance, best and average reported (p24) | [ASSUMPTION] |
| Objective in search | Customer waiting time (every move) | [PAPER] |

### 7.3 The eight neighbourhoods (what the paper actually says or draws)

| # | Name | Paper definition (text) | Paper example (figure) | Edge cases and notes |
|---|---|---|---|---|
| 1 | Launch node relocate | "Repositioning the launch node of the drone trip ... searching **adjacent customer nodes and flexible launch nodes** of the drone launch node in original drone trip ... most optimal relocation location ... cost (customer waiting time)" (p20–21) | Fig. 10a: trip C1→C2→C3 launched **from customer C1**, moved to an FLP on arc Depot→C1 | Candidate set includes customers (contradicts Assumption 4). "Adjacent" undefined. Runs "several times". |
| 2 | Recover node relocate | "Searching adjacent customer nodes of the drone recover node" (p21) | Fig. 10b: recovery C3 → C4, the next truck customer | Candidates: truck customers (and presumably the depot). Direction not restricted. |
| 3 | Drone service node relocate | "Repositioning the drone service node of the drone trip ... searching **adjacent trip** of the original drone trip" (p21) | Fig. 10c: trips {C1,C4,C3} and {C3,C2,C5} become {C1,C2,C3} and {C3,C4,C5}: the served customers are **exchanged between two consecutive trips** | Behaves as an exchange between adjacent trips (not a single-trip retarget). |
| 4 | Truck service node relocate | "Repositioning the truck service node ... searching adjacent customer node of a **random** truck service node" (p21–22) | Fig. 10d: C4 moved after C2; described as "C2 and C4 are exchanged" | Starts from a random truck node; the drone trip is unchanged. |
| 5 | T-D service node swap | "Randomly selecting a drone trip and searching adjacent truck service nodes ... C2 and C3 are exchanged" (p22) | Fig. 11a: C3 (drone) ↔ C2 (truck); **the trip's launch and recovery also change** (C2→C3→C4 becomes C1→C2→C3) | The launch/recovery re-selection rule is not stated. |
| 6 | D-D service node swap | "Randomly selecting a drone trip and searching adjacent drone service node of **other** drone trips" (p22) | Fig. 11b: two drones launched from the same customer C1 exchange their customers C2 ↔ C4 | Pair = random trip × adjacent other trips. |
| 7 | Truck node delete → drone node insert | "Randomly selecting a truck service node and then attempting to assign an **available drone** to construct a drone trip ... most optimal drone trip" (p22) | Fig. 12a: C3 removed; new trip {C1,C3,C4} (launch from customer C1) by a second drone | Needs drone availability (identity / capacity). Launch/recovery choice rule not stated. |
| 8 | Drone node delete → truck node insert | "Randomly selecting a drone trip and then attempting to assign it to the truck route ... feasibility test ... cost ... to determine the optimal order of truck visits" (p22–23) | Fig. 12b: trip {C1,C2,C3} removed; C2 inserted between C1 and C3 | Best insertion position; the FLP that becomes unused must be removed (constraint 6) [INFERENCE]. |

### 7.4 The four shakes (p23)

1. **Random_Reverse:** "Randomly select two nodes in the truck route, and reverse the truck path segment including two nodes."
2. **Random_Exchange:** "Two truck path segments of equal length are selected at random and subsequently exchanged." Segment length not given.
3. **Random_Shift:** "Randomly move a randomly selected truck path segment forward/backward by **several nodes**." The text is complete and legible; the segment length and "several" are not quantified.
4. **Random_Drone_Remove:** "A drone is selected at random, and its whole trips are removed from the current solution. The customers of these trips are served by the truck." Needs persistent drone identity; the reinsertion position is not given.

How drone trips anchored to FLPs on reversed, exchanged or shifted arcs are handled after a shake is **not described** [UNKNOWN]. Because the arcs are directed, FLPs of L_ij are not those of L_ji.

---

## 8. Test coverage matrix (paper requirement → tests)

| Paper requirement | Implemented? | Tested? | Test (file: block) | Missing test? |
|---|---|---|---|---|
| Objective (1) = Σ serving-vehicle arrival | Yes | Yes | timing §1&6, §12-14, Block 9; init B6, B8-B10 | — |
| Truck timing (16), start at 0 | Yes | Yes | timing §1&6 | — |
| Truck service Se^T | Yes | Yes | timing §1&6, Block 9 | — |
| Drone launch (21) | Yes | Yes | timing §2-5 | — |
| Truck departure at FLP (18), n = 1 | Differently (no SR at launch node) | Partially | timing §2-5 | Literal-reading test |
| Truck departure at FLP (18), n ≥ 2 (k·SL) | **Differently** | **No** | — | **Yes** |
| Recovery (20) + sync (Assumption 7) | Yes (plus max(t,·)) | Yes | timing §2-5, §8 | — |
| Multiple recoveries at one node | Yes (concurrent) | **No** | — | **Yes** |
| Service/recovery interaction | Yes (concurrent) | Yes | timing Block 9 | — |
| Drone service (22) | Yes | Yes | timing §2-5 | — |
| Endurance (36)/(37) incl. Se^D and waiting | **Differently** | Tests encode the current rule | timing §9, §11 | Paper-rule tests |
| θ (35) | Partially (outbound only) | Yes (current rule) | timing §11 | Se^D·θ case |
| Each customer served once (10) | Yes | Yes | timing §15; init B6 | — |
| Recovery after launch (route order) | Yes | Yes | init B5; timing §12-14 (recovery at terminal depot) | — |
| Recovery at depot | Yes | Yes | timing (terminal-depot recovery), §12-14 | — |
| No recovery at FLP / no depot launch (Fig. 4) | Yes (roles) | Yes | model tests; timing §10 | — |
| Unvisited FLP cannot launch (5) | Yes | Indirect | init B5 | — |
| Visited FLP must launch (6) | **No** | No | — | Yes |
| FLP-arc consistency (13)–(15) | **No** | No | — | **Yes** |
| FLP visited at most once (8)–(9) | No | No | — | Low |
| Per-drone launch/recovery uniqueness (11)–(12) | No | No | — | Yes (once identity exists) |
| Fleet size \|U\| / reuse (28) | F4 only | F4 only | init B7-B9 | **Evaluator test** |
| Drone identity (per-drone trip lists) | **No** | No | — | Yes |
| FLPs evenly spaced, count ∝ length | Fixed count | Current rule tested | flp tests | Spacing-rule tests |
| Depot/closing arcs get FLPs | Yes | Yes | flp (closing arc) | — |
| k = ⌈C/(D+2)⌉ | Yes | Yes | init B1 | — |
| TSP optimal (Cplex) | Held-Karp / heuristic | Yes | tsp; init B3 | — |
| Alg. 2 per-cluster D cap | **Differently** | Tests encode the patch | init B8, B9 | Literal-Alg. 2 comparison |
| Alg. 2 recovery in cluster | Not enforced | No | — | Yes |
| VNS / neighbourhoods / shakes | No | No | — | Phase G–I |
| Instances / Tables 3–4 | No | No | — | Phase J–K |

**Implemented and tested:** objective, truck timing, launch (21), drone flight and service, recovery sync, service/recovery concurrency, customer coverage, route order, node roles, k formula, K-means, TSP, F4 fleet checks.

**Implemented but untested:** multiple recoveries at one node; multiple launches at one FLP.

**Implemented differently (tests pass because they encode the current rule):** SL count, endurance (Se^D, waiting, θ scope), Alg. 2 per-cluster cap, FLP count rule.

**Not implemented:** constraints (6), (8)–(9), (11)–(15); drone identity; evaluator fleet check; G–K.

---

## 9. Implementation Assumptions (register)

Confidence = how confident we can be that the assumption is compatible with the paper (HIGH = consistent with the paper's intent; LOW = likely to differ from what the authors did).

| Assumption | Where used | Why needed | Paper support | Confidence | Impact |
|---|---|---|---|---|---|
| FLP count = fixed `points_per_arc` (1) | flp/, init F3 | No main-experiment value | Contradicted in structure (p18 proportional; 2 km in 6.3) | LOW | HIGH |
| FLP placement ratio k/(n+1) | flp/ | Positions within arc unspecified | "Equal proportions" (p8) | MEDIUM | LOW |
| Depot/closing arcs get FLPs | flp/ | — | p8 supports | HIGH | LOW |
| FLPs generated once from r0, frozen | init F3 | Init step (c) once | Matches init; silent for VNS | MEDIUM (init) / LOW (VNS) | HIGH for G/H |
| Launch candidates = FLPs on arc entering j | init F3/F4 | "Attached FLP" undefined | Figs. 1 and 3 show launches from other arcs | LOW | MEDIUM |
| Recovery = next customer/depot on route | init F4 | Alg. 2 silent | p19 "customer point in the cluster"; p16 any later node | LOW–MEDIUM | MEDIUM |
| Recovery locking | init F4 | Protect earlier trips | Not in paper (engineering) | MEDIUM | LOW |
| Converted customer's slot replaced by FLP | init F4 | Route update | Fig. 8d removes customer; arc rule (13)–(15) violated | LOW | MEDIUM |
| No per-cluster cap; global time-aware fleet check | init F4 | Post-approval patch | Literal Alg. 2 caps D per cluster | LOW (vs literal) | MEDIUM |
| Busy interval ends at recovery_complete (= max(t,landing) + SR) | init/drone_fleet_capacity | Release time | (28) releases at t_k | MEDIUM | LOW |
| Touching intervals do not overlap | drone_fleet_capacity | Boundary | Not stated | HIGH | LOW |
| BestRoute = min arrival of j | init F4 | Undefined | Serves the objective | MEDIUM | MEDIUM |
| Tie-breaks by id (customer, launch) | init F4, K-means | Determinism | Not stated | HIGH (harmless) | LOW |
| Cluster/customer processing order ascending | init F4 | Not stated | "Successively" | MEDIUM | LOW |
| K-means seeding (id-spaced), stopping (no change / ε / 100), empty clusters kept, ties to lower index | init F2 | Not stated | Not stated | MEDIUM | LOW–MEDIUM |
| Truck-only route minimises time (= distance) | init F1 | "Optimal truck-only" | Fig. 8 "TSP" | HIGH | LOW |
| Held-Karp ≤ 20 customers, heuristic above | tsp/ | No CPLEX | Paper uses Cplex (exact) | HIGH (10/20) / MEDIUM (50) | LOW–MEDIUM |
| One shared SL per FLP (truck side) | timing | Earlier reading | **Contradicted** by (18) | LOW | MEDIUM |
| No SR term at launch node | timing | Typo assumption | Literal (18) includes it | UNKNOWN | MEDIUM |
| Recovery windows concurrent, with max(t_k, landing) | timing | Assumption 7 | (20) concurrent; max is added | MEDIUM | LOW–MEDIUM |
| Se1 and SR concurrent | timing | (18) + (20) literal | Typo reading says additive | MEDIUM | MEDIUM |
| Endurance excludes Se^D and waiting; θ on outbound only | timing | Earlier reading | **Contradicted** by (36)–(37), p15, p30 | LOW | HIGH |
| Evaluator ignores num_drones | timing | Separation | Contradicted by model structure | LOW | HIGH (G/H) |
| drone_slot = per-trip counter | model, init | Bookkeeping | Paper uses persistent drone ids | LOW | HIGH (H: Random_Drone_Remove) |
| Route-position fields unused | model | — | N/A | HIGH | NONE |
| Start and end depot share one NodeId | model, tsp, timing | Round trip | Paper: two roles, 0 and c+1, same location | HIGH | LOW |
| Speed ≤ 0 → +∞ time | graph | Guard | N/A | HIGH | NONE |
| kTimeEpsilon = 1e-9 tolerance | timing | Floating point | N/A | HIGH | NONE |
| Launches only from FLPs | model roles | Assumption 4 | Text yes; figures no | MEDIUM | HIGH for G |
| Units = seconds | params | Figs. 13–20 in seconds | e given "in minutes" in Table 2 | HIGH | LOW |
| Unspecified VNS parameters (T_x, cooling, iter_max, shake distribution, sizes, repetition, adjacency, seed) | Planned G–I | Not in paper | None | — | HIGH |

---

## 10. Planned phases (G–K) vs paper

**In-repo planning** is limited to the `README.md` one-liner and the `DECISIONS.md` Pre-Phase-G section. Neither defines neighbourhood semantics, so there is nothing in the repo to mark MATCH or DEVIATION.

**External, unverified planning:** the 26 Sep backup `docs/stage1_implementation_spec.md` is outside the repo and is not authoritative. It is compared below because it may be reused.

| Planned item (backup spec) | Paper | Mark |
|---|---|---|
| B.1 VNS with best-so-far tracking, comparing to f(x') | Literal Alg. 1 compares to f(x), searches from x', no best tracked | PARTIAL MATCH (deliberate [INFERENCE] repair; must be labelled as such) |
| B.3 Uniform shake selection | "Randomly" | ASSUMPTION |
| B.4 Local-search order | p24 order | MATCH |
| B.4 Relocate repetition configurable | "Several times" | NOT SPECIFIED BY PAPER |
| B.5.2 k·SL | (18) | MATCH (truck side) |
| B.5.2 Staggered drone launch times | (21) allows all at t_i + SL | DEVIATION (not paper-supported) |
| B.5.2 k·SR at recovery node, staggered | Literal (18)/(20) do not say this | UNKNOWN / possible DEVIATION |
| C.1 Launch relocate over FLPs only | Paper: "adjacent customer nodes and flexible launch nodes" | PARTIAL MATCH (paper self-contradictory) |
| C.2 Recover relocate over later truck nodes | Adjacent customer nodes | MATCH (radius assumption) |
| C.3 "Reading A" single-trip retarget (recommended) | Fig. 10c + "adjacent trip": exchange between adjacent trips | **DEVIATION** (Reading B is closer to the paper) |
| C.4 Truck relocate, candidates = all/radius positions | Random truck node, adjacent positions | PARTIAL MATCH |
| C.5 T-D swap over all pairs within radius | Random trip × adjacent truck nodes; launch/recovery also change | PARTIAL MATCH |
| C.6 D-D swap over pairs within radius | Random trip × adjacent other trips | PARTIAL MATCH |
| C.7 Truck delete → drone insert reusing Alg. 2 logic | Random truck node; "available drone" | PARTIAL MATCH |
| C.8 Drone delete → truck insert with radius-limited insertion | Random trip; optimal insertion order | PARTIAL MATCH |
| D.1 Random_Reverse | p23 | MATCH |
| D.2 Random_Exchange (length range assumed) | p23 (no length) | MATCH + ASSUMPTION |
| D.3 "Segment move ... text cut off, unrecoverable" | **The text is not cut off**: "Random_Shift ... forward/backward by several nodes" (p23) | DEVIATION (factual error in spec); magnitude = ASSUMPTION |
| D.4 Random_Drone_Remove reading (a), per-drone id | Paper: persistent drone | MATCH (but the current `drone_slot` cannot support it) |
| E.5 Endurance invariant: outbound-only θ | (36)–(37) | DEVIATION |
| E.6 Additive SR | Unresolved | UNKNOWN |
| E.7 No launch from customer/depot | Assumption 4 vs figures | PARTIAL MATCH |
| E.4 Capacity in evaluator | Model | MATCH |
| Instance generator (J) | 60×60 km, uniform, 4 × {10,20,50}; depot placement not stated | NOT YET IMPLEMENTED (depot = [ASSUMPTION]) |
| Validation (K) | Table 3 (10 runs, best/avg), Table 4 | NOT YET IMPLEMENTED |

---

## 11. Paper vs Implementation Deviations (register)

| ID | Paper section / eq. | Paper behaviour | Our behaviour | Classification | Impact | Action needed? |
|---|---|---|---|---|---|---|
| D-01 | (18) p13 | Truck held n_i·SL at a launching FLP | Held one SL | DEVIATION | MEDIUM (HIGH once G creates shared FLPs) | Decision before G |
| D-02 | (18) p13 | Literal: n_i·SR also at the launch FLP; SR otherwise only via (20) | Not modelled | UNKNOWN | MEDIUM | Decision before G |
| D-03 | (36)–(37) p15; text p15, p30 | Endurance = (outbound + Se^D)(1+θ) + max(return, time to truck arrival) | outbound(1+θ) + return | DEVIATION | HIGH | Decision before G |
| D-04 | (1)–(12), (28); Figs. 3, 7; p23 | Persistent drone identity; capacity inside the model | `drone_slot` counter; evaluator ignores \|U\| | DEVIATION | HIGH (G/H) | Decision before G |
| D-05 | p18; p26–33 | FLP count ∝ arc length (spacing) | Fixed count per arc | DEVIATION (structure) + ASSUMPTION (value) | HIGH (search space; Table 4 comparability) | Decision before G |
| D-06 | p9; (13)–(15) | FLP of L_ij usable only on the traversed arc (i, FLP, j) | F4 uses the FLP of (prev, j) between prev and next(j); evaluator does not check | DEVIATION | MEDIUM–HIGH | Decision before G |
| D-07 | Alg. 2 p20 | ≤ D trips per cluster, reset per cluster | Unlimited per cluster; global time-aware fleet check | DEVIATION (from literal) | MEDIUM | Decision (can wait) |
| D-08 | p19 | Launch/recover node correspond to cluster customer or attached FLP | Recovery = next route node (any cluster or depot) | INTERPRETATION / possible DEVIATION | LOW–MEDIUM | Can wait |
| D-09 | Assumption 4, p11, Table 1 vs Figs. 7, 10–12, 15–19, p21 | Text: FLP-only launches; figures and move definitions: customer/depot launches | FLP-only | UNKNOWN (paper contradiction) | HIGH for G (moves 1, 5, 6, 7 as drawn) | Decision before G |
| D-10 | (20) + Assumption 7 | l_k ≥ landing + SR per drone | l_k ≥ max(t_k, landing) + SR | INTERPRETATION | LOW | No |
| D-11 | (18) typo reading | Se^T + r·SR additive | Concurrent max | INTERPRETATION (matches literal) | MEDIUM | Decision (with D-02) |
| D-12 | (6) | Visited FLP must launch | Not checked in evaluator | DEVIATION (evaluator scope) | LOW now, MEDIUM in G | Before G |
| D-13 | (28) | Relaunch ≥ t_k | Release at max(t_k, landing) + SR | DEVIATION (conservative) | LOW | Can wait |
| D-14 | (8)–(9), (11)–(12) | FLP once; per-drone launch/recovery uniqueness | Not checked | DEVIATION (evaluator scope) | LOW | With D-04 |
| D-15 | Step (a) p18 | Cplex-optimal TSP | Exact ≤ 20 customers; heuristic at 50 | INTERPRETATION (environment) | LOW | No |
| D-16 | K-means, BestRoute, ordering | Unspecified | Our choices | ASSUMPTION | LOW–MEDIUM | Document only |
| D-17 | p15 | t = 0 stated | Labelled ASSUMPTION | MATCH (documentation mislabel) | NONE | Doc only |
| D-18 | Objective (1), (10) | — | — | MATCH | NONE | No |
| D-19 | (16), (17), (19), (21), (22) | — | — | MATCH | NONE | No |
| D-20 | Recovery rules (condition 3, Figs. 4–5) | Later truck node or end depot; not at FLP | Same (evaluator) | MATCH | NONE | No |
| D-21 | k = ⌈C/(D+2)⌉ | — | — | MATCH | NONE | No |

**Contradictions between `DECISIONS.md` and the paper** (documented; `DECISIONS.md` not edited):
1. Eq. (41) quoted as ⌈D/d⌉; the paper prints ⌊D/d⌋ (p33).
2. "Waiting does not count toward endurance" contradicts (36), p15 and p30.
3. θ described as outbound-only, whereas (36)–(37) multiply outbound + Se^D.
4. The truck starting at t = 0 is labelled ASSUMPTION; the paper states it (p15).
5. Assumption numbering is misattributed: "each customer served once (Assumption 1)" is paper Assumption 3; "launch from FLP / recover at service point or depot (Assumption 3)" is paper Assumptions 4–5; "battery endurance (Assumption 4)" is paper Assumption 6.
6. The frozen-pool "stray FLP" is described as a harmless inefficiency, but it violates (13)–(15) and p9.
7. The FLP rule is described as unspecified by the paper and sensitivity-only; p18 states proportionality generally, and 2 km is used throughout Sec 6.3.

---

## 12. Paper Requirements Not Yet Implemented

**A. Intentionally deferred**
- Stage 2 (dynamic customers): not part of Paper 1 reproduction.
- The MILP itself (Cplex comparison in Table 3). The paper's numbers are used as the reference instead.
- Cost equation (40) and the Sec 6.3 sensitivity studies (Dalian C1–C4, drone count, endurance, speed, unit division).

**B. Planned for G–K**
- The 8 local-search neighbourhoods and the 4 shakes (§7).
- VNS Algorithm 1 with SA acceptance, the local-search order, and relocate repetition.
- Instance generator: 60×60 km, uniform, 4 datasets × {10, 20, 50}; depot position [ASSUMPTION].
- Experiment runner: 10 runs per instance, z_best and z_avg, runtime; Δz per eqs. (38)–(39).
- Parameter configuration: truck 15 m/s, drone 12 m/s (p24; see the p31 contradiction), e = 30 min, SL = 60 s, SR = 40 s, service 30 s, θ = 1, 2 drones.

**C. Accidentally missing** (required by the paper's model, absent from the evaluator or model, and not planned anywhere in the repo)
- Persistent drone identity and per-drone trip sequences (D-04).
- Fleet-size enforcement in solution evaluation (D-04).
- FLP–arc consistency, (13)–(15) (D-06).
- No idle FLP stops, (6) (D-12).
- FLP visited at most once, and per-drone launch/recovery uniqueness, (8)–(9), (11)–(12) (D-14).
- FLPs available on arcs created by route-changing moves (§5, "dynamic or once").
- Endurance including Se^D, θ on Se^D, and waiting (D-03).
- k·SL truck hold (D-01).

**D. Cannot be implemented faithfully without additional interpretation**
- SR placement in (18) (D-02 / D-11).
- Launching from customers/depot, text vs figures (D-09).
- Main-experiment FLP spacing and the Table 4 generating rule (D-05).
- The "attached FLP" and "adjacent" radius definitions (Alg. 2; moves 1–8).
- BestRoute; the Alg. 2 literal defects; the per-cluster drone cap (D-07).
- T_x, cooling, iter_max, shake distribution and sizes, relocate repetition count.
- Launch/recovery re-selection in T-D swap and truck-delete/drone-insert.
- Handling of FLP-anchored trips after shakes.
- Depot placement; objective units in Table 3 (Figs. 13–20 are in seconds).

---

## 13. Paper details that are easy to miss

- **p8:** "Recovery exclusively at truck customer locations" contradicts Assumption 5 and the depot recoveries in Figs. 3, 7 and 16.
- **p9:** "The truck is not obligated to remain stationary awaiting the drone": no waiting at launch.
- **p10, Fig. 3:** two drones launch from the same FLP (e) and both recover at the same customers (C3, C7). The paper's own illustrative solution needs multi-launch/multi-recovery timing.
- **p11:** Table 2 gives e "in minutes"; all results are in seconds.
- **p12:** y_s^u is indexed by drone in the constraints, although Table 2 lists y_s.
- **p13:** (16) prints l_j where l_i is presumably meant. (18) sums both SL and SR over P_i⁺.
- **p14:** (28) releases a drone at the truck's arrival t_k, not at t_k + SR.
- **p15:** "Operating time starts at 0". "For customers who are served by the truck, t̂ is set to 0." Waiting counts toward endurance.
- **p16:** recovery may skip truck nodes (Fig. 5). Multiple drones from one FLP "minimizing truck docking times".
- **p18:** FLP count "proportional to the truck's driving distance". Fig. 7 shows a launch from a customer node.
- **p19:** "Launch and recover node ... correspond to the customer point in the cluster or the attached flexible launch point."
- **p20:** Algorithm 2's `while k ≤ D` inside the cluster loop.
- **p21–23:** moves start from a *random* trip or node, then search "adjacent" candidates. Several examples launch from customers.
- **p23:** the third shake is named **Random_Shift** and is fully legible ("by several nodes").
- **p24:** drone speed 12 m/s (slower than the truck). A single service time of 30 s, with truck vs drone not distinguished. 10 runs per instance.
- **p26 / p29:** "25 minutes, equivalent to 1800 seconds" (inconsistent). Figs. 13 and 14 match 1500 s.
- **p31:** says Sec 6.1 used a drone speed of 15 m/s (contradicts p24). The text says higher speed "correlates with a varied increase" in waiting, while the table shows decreases.
- **p33:** Fig. 19 text attributes behaviour to 12 m/s, but the Fig. 19 caption compares 15 vs 18 m/s.
- **p35:** the FLP count must be "pre-determined". One customer per flight.

---

## 14. Final executive summary

### Current reproduction status

1. **Reproduced closely:**
   - Objective (1).
   - Truck timing (16), with start at t = 0.
   - Drone launch departure (21), drone flight (17), drone service (22).
   - Recovery coupling (20) with Assumption 7 synchronisation.
   - Customer coverage (10); route order and precedence (23)–(27).
   - Recovery rules (condition 3, Figs. 4–5).
   - Cluster count k = ⌈C/(D+2)⌉.
   - Exact truck-only TSP for 10- and 20-customer instances.
   - The overall 4-step initialisation structure.
2. **Approximate interpretations:**
   - Recovery synchronisation (max(t_k, landing) + SR).
   - Service/recovery concurrency (matches only the literal reading of (18)).
   - Heuristic TSP at 50 customers.
   - FLP positions k/(n+1).
   - Customer removal during F4.
3. **Assumptions:**
   - FLP count per arc.
   - All K-means internals.
   - The "attached FLP" definition.
   - Recovery choice in F4.
   - BestRoute scoring.
   - Tie-breaks, orders, recovery locking.
   - All VNS parameters (none are given in the paper).
4. **Deviations from explicit paper content:**
   - One SL instead of k·SL (D-01).
   - Endurance excludes Se^D and waiting, with θ scope too narrow (D-03).
   - No persistent drone identity, and fleet size not enforced by the evaluator (D-04).
   - Fixed FLP count instead of length-proportional (D-05).
   - FLPs used off their arc (D-06).
   - Alg. 2 per-cluster cap removed (D-07).
   - Missing constraint checks (6), (8)–(9), (11)–(15).
5. **Still missing:**
   - Phases G–K entirely.
   - Drone identity, capacity in the evaluator, FLP–arc validity.
   - FLP availability for routes that change during search.
   - Instance generation and the experiment protocol.
6. **Unresolved issues that could materially affect correctness:**
   - SR placement in (18) (D-02 / D-11).
   - Launching from customers/depot (D-09).
   - Main-experiment FLP spacing and the Table 4 rule (D-05).
   - Unspecified VNS parameters.
   - Parameter contradictions: drone speed 12 vs 15 m/s; endurance 1500 vs 1800 s.
7. **Resolve before Phase G:**
   - D-01 (k·SL).
   - D-02 / D-11 (SR reading; this is a decision for you).
   - D-03 (endurance definition).
   - D-04 (drone identity and where capacity is enforced).
   - D-05 / D-06 (FLP generation rule, FLP–arc validity, and how FLPs exist on arcs created by moves).
   - D-09 (customer/depot launches: are neighbourhoods 1, 5, 6 and 7 as drawn in scope?).
   - D-12 (idle FLP removal).
   - Choosing Reading A vs B for neighbourhood 3. The paper's figure indicates an exchange between adjacent trips.
8. **Can safely wait:**
   - D-07 / D-08 (Alg. 2 literalness; affects only the starting solution).
   - D-10, D-13, D-15, D-16, D-17.
   - The documentation contradictions listed at the end of §11.
   - The Sec 6.3 sensitivity studies and cost eq. (40).
   - Table 3 unit confirmation.

No overall percentage is given, since no transparent metric supports one. Counting the §11 register instead: 5 rows are MATCH (D-17 to D-21), 4 INTERPRETATION or partial matches (D-08, D-10, D-11, D-15), 1 ASSUMPTION (D-16), 2 UNKNOWN (D-02, D-09) and 9 DEVIATION (D-01, D-03 to D-07, D-12 to D-14). Five of the deviations (D-01, D-03 to D-06) affect the evaluator or data model that Phase G will depend on.

---

## 15. Files created or modified by this audit

- **Created:** `docs/paper1_full_implementation_audit.md` (this file).
- **Modified:** none. No source, test, CMake, README, DECISIONS, configuration or planning file was changed.

---

## 16. Page coverage table

| Page (PDF) | Journal p. | Main content | Inspected visually? | Relevant implementation information? |
|---|---|---|---|---|
| 1 | 4072 | Title, abstract, intro start | Yes (160 + 220 dpi) | Minor (objective; FLPs = non-customer stops) |
| 2 | 4073 | Introduction | Yes (160 + 220 dpi) | No (context: 15 km range) |
| 3 | 4074 | Intro: approach, contributions | Yes (160 + 220 dpi) | Yes (evenly distributed FLPs; one or more drones per FLP; objective) |
| 4 | 4075 | Structure; related work | Yes (160 + 220 dpi) | No |
| 5 | 4076 | Related work | Yes (160 + 220 dpi) | No |
| 6 | 4077 | Related work | Yes (160 + 220 dpi) | No |
| 7 | 4078 | Table 1 | Yes (160 + 220 dpi) | Yes (FP-only launch classification) |
| 8 | 4079 | Fig. 1; FLP concept; problem definition | Yes (160 + 220 dpi) | Yes (equal proportions; depot arcs; recovery "exclusively" at customers) |
| 9 | 4080 | Fig. 2; definitions | Yes (160 + 220 dpi) | Yes (L_ij usable only if arc traversed; SL/SR/Se^D; objective) |
| 10 | 4081 | Fig. 3; Assumptions 1–8 | Yes (160 + 220 dpi) | Yes (multi-launch/recovery example; assumptions) |
| 11 | 4082 | Table 2; single-flight definition | Yes (160 + 220 dpi) | Yes (notation; i ∈ L) |
| 12 | 4083 | Condition 3; P sets; (1)–(8) | Yes (160 + 220 + 300 dpi) | Yes |
| 13 | 4084 | (9)–(23) | Yes (300 dpi; (18) at 400 dpi) | Yes (timing block) |
| 14 | 4085 | (24)–(34); explanations | Yes (300 dpi) | Yes ((28), (29), (30), text) |
| 15 | 4086 | t = 0; (35)–(37); waiting | Yes (300 dpi) | Yes (endurance, θ) |
| 16 | 4087 | Figs. 4–6; prohibited/allowed operations | Yes (300 dpi) | Yes |
| 17 | 4088 | Algorithm 1 | Yes (300 dpi) | Yes |
| 18 | 4089 | Fig. 7; representation; init steps | Yes (160 + 300 dpi) | Yes (FLP ∝ distance; Cplex) |
| 19 | 4090 | Fig. 8; K-means; Alg. 2 narrative | Yes (160 + 300 dpi zoom) | Yes |
| 20 | 4091 | Algorithm 2; local-search intro | Yes (300 dpi) | Yes |
| 21 | 4092 | Fig. 10; relocate moves | Yes (300 dpi) | Yes |
| 22 | 4093 | Fig. 11; swap and deletion moves | Yes (300 dpi) | Yes |
| 23 | 4094 | Fig. 12; shakes; environment | Yes (300 dpi) | Yes |
| 24 | 4095 | Instances, parameters, local-search order | Yes (300 dpi) | Yes |
| 25 | 4096 | Tables 3–4; eqs. (38)–(39) | Yes (300 dpi) | Yes (validation targets) |
| 26 | 4097 | Sensitivity setup; eq. (40) | Yes (300 dpi) | Yes (2 km spacing; 1800 s inconsistency) |
| 27 | 4098 | Table 5; Fig. 13 | Yes (300 dpi) | Minor (units in seconds) |
| 28 | 4099 | Fig. 14; endurance analysis | Yes (300 dpi) | Minor (2 km; 1500 s) |
| 29 | 4100 | Fig. 15 | Yes (300 dpi) | Minor (depot launch drawn; truck waits) |
| 30 | 4101 | Fig. 16 | Yes (300 dpi) | Yes (waiting counts toward endurance) |
| 31 | 4102 | Fig. 17; speed analysis | Yes (300 dpi) | Minor (speed contradiction; 2 km) |
| 32 | 4103 | Figs. 18–19 | Yes (300 dpi) | Minor (customer/depot launches drawn) |
| 33 | 4104 | Table 6; eq. (41) | Yes (300 dpi) | Yes (n = ⌊D/d⌋) |
| 34 | 4105 | Fig. 20; conclusion start | Yes (300 dpi) | Minor (FLP density effect) |
| 35 | 4106 | Conclusion; acknowledgments; references | Yes (300 dpi) | Minor (pre-determined FLP count; one customer per flight) |
| 36 | 4107 | References | Yes (150 dpi) | No |
| 37 | 4108 | References | Yes (150 dpi) | No |
| 38 | 4109 | References | Yes (150 dpi) | No |
