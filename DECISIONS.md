# Implementation decisions log

Concrete decisions made while implementing the plan in
`paper1-implementation-plan.md`, recorded here as they happen so the
[PAPER]/[INFERENCE]/[ASSUMPTION] classification in that document stays
accurate. Each entry says what was decided, why, and what it means for
reproduction fidelity.

## TSP solver for stage (a) of the initial solution (truck-only route)

**What Paper 1 does.** Section 5.2 states the truck-only route is obtained
by solving the TSP to optimality with CPLEX ("since it is a classic TSP
problem to use only a truck to serve customers, there are currently mature
TSP solvers that can find the optimal solution within an acceptable time
range, so Cplex is used"). This is an explicitly **exact** step in Paper 1's
own methodology, not a heuristic one.

**What we checked.** This machine has an `.ibm_cplex_studio` folder in the
user's profile, suggesting CPLEX Optimization Studio may have been
installed at some point. Whether it is currently installed, licensed, and
has its C++ API (Concert Technology) set up for linking was not confirmed
at the time this decision was made. Per instruction, CPLEX was **not**
silently swapped for a heuristic and called equivalent -- the substitute
below is used instead, and is explicitly flagged as non-equivalent to
Paper 1's method wherever it applies.

**Decision.** Two solvers are implemented behind a common `ITSPSolver`
interface (`include/tsp/tsp_solver.hpp`), selected automatically by
instance size via `TSPSolverFactory`:

- **`HeldKarpSolver`** -- an exact dynamic program (Held-Karp,
  O(n^2 * 2^n) time / O(n * 2^n) memory). When it runs, its result is a
  *genuinely certified global optimum* -- the same guarantee CPLEX gives
  Paper 1 for this step, just via a different exact method. It refuses to
  run (rather than silently producing a slow or memory-exhausting solve)
  if the instance exceeds a configured `max_exact_n` (default 21) or an
  estimated-memory safety limit (default 4 GiB).
- **`HeuristicTSPSolver`** -- nearest-neighbor construction with
  multi-restart, improved by 2-opt and Or-opt local search. This is
  **not exact** and is only used as a fallback when Held-Karp refuses.
  Every `TSPResult` it returns carries `exact = false` and an explicit
  `note` string saying so -- nothing downstream can mistake its output
  for a certified optimum.

**Sizing against Paper 1's own instances.** The truck-only TSP is solved
over customers *plus the depot*, so:

| Paper 1 instance | customers | TSP nodes (incl. depot) | solver used (default `max_exact_n=21`) |
|---|---|---|---|
| small  | 10 | 11 | HeldKarpSolver -- **exact**, matches Paper 1's CPLEX exactness |
| medium | 20 | 21 | HeldKarpSolver -- **exact**, matches Paper 1's CPLEX exactness |
| large  | 50 | 51 | HeuristicTSPSolver -- **not exact**, documented substitute |

**Reproduction-fidelity implication.** Per the plan's three-tier
classification (`paper1-implementation-plan.md`, Sec. 21): stage (a) of
the initial solution is **EXACTLY REPRODUCIBLE** for the small and medium
synthetic instances (both solvers guarantee the same global optimum
CPLEX would find, modulo floating-point/tie-breaking differences in which
optimal tour is returned when several exist), and **ASSUMPTION-DEPENDENT**
for the large (50-customer) instances, where our truck-only route is not
guaranteed optimal the way Paper 1's was. This can affect the initial
solution VNS starts from for large instances -- VNS's own search should
still explore away from a suboptimal start, but any comparison of our
large-instance results against Paper 1's Table 3 should account for this
difference explicitly, not treat a discrepancy there as evidence of a bug.

**If CPLEX turns out to be available and licensed on this machine:** a
`CplexTSPSolver` implementing the same `ITSPSolver` interface can be added
and substituted in `TSPSolverFactory` without touching any other module,
which would upgrade the 50-customer case from ASSUMPTION-DEPENDENT to
EXACTLY REPRODUCIBLE as well. This is intentionally left as a drop-in,
not built now, since CPLEX availability was not confirmed.

**Tests.** `tests/test_tsp.cpp` verifies Held-Karp against independent
brute-force enumeration on small instances, a hand-checkable known-geometry
case (unit square, optimal tour = perimeter = 4), the threshold/memory
refusal behavior, and that the heuristic solver always returns a valid,
correctly-costed tour that is never worse than plain nearest-neighbor.

## Static data model (Phase B)

`include/model/` (header-only, namespace `tdr::model`) implements the core
static domain types from `paper1-implementation-plan.md` Sec. 5, and
nothing else -- no timing, no feasibility, no generation logic, no
algorithms. It exists so that Phase C onward has concrete `Node`,
`Customer`, `FlexibleLaunchPoint`, `DroneTrip`, `TruckRoute`, and
`Solution` types to build against, without those later phases each
inventing their own shape for "a node" or "a solution."

A few modeling choices had to be made where the paper is either silent or
only implies a structure. Each is recorded here so it stays visible
alongside the plan's [PAPER]/[INFERENCE]/[ASSUMPTION] classification,
rather than being buried as an unremarked-on implementation detail.

**Depot represented as one `NodeType`, not two.** Paper 1's own vertex set
has D = {0, c+1}: a start depot and an end depot at the same physical
location, because its formulation treats the truck route as a path, not a
cycle (Sec 2.1). The Phase-B model does *not* introduce a fourth
`NodeType` for "start depot" vs. "end depot" -- both are represented as
ordinary `Node` entries with `NodeType::Depot`, distinguished only by
having different `NodeId`s (and, in practice, identical coordinates).
**[ASSUMPTION: representation only.]** Which id plays which role (i.e.
"this is id 0, the route's start" vs. "this is id c+1, the route's end")
is left to whichever module actually builds a concrete instance (Phase
C/D) -- Phase B only needed to make sure two depot-typed nodes with
different ids *can* coexist, which `NodeRegistry` already supports.

**`theta_j` kept per-customer, not global.** Table 2 defines the
generalized-flight-time coefficient as `theta_j`, indexed by customer,
even though Paper 1's main experiments (Sec 6.1) set it to a single
constant (theta = 1) for every customer. `CustomerData` keeps a
`generalized_flight_time_theta` field per customer rather than putting a
single global constant in `DroneParameters`, to match the paper's own
indexing. **[PAPER]** for the per-customer indexing itself;
**[ASSUMPTION]** only in the sense that nothing here enforces the
main-experiment convention of setting every customer's value equal --
that is a property of how a given experiment's data is populated (Phase
D), not of the data structure.

> **SUPERSEDED (29 Sep 2026):** `drone_slot` was replaced by a physical
> drone id `DroneTrip::drone_id` (u in U, 0 <= u < |U|), enforced by the
> evaluator via (28) and |U|. The paragraph below is kept for history;
> see "Paper-conformance corrections (29 Sep 2026)", C4.

**Nominal per-trip drone identity (`DroneTrip::drone_slot`).** The MILP
formulation only needs the drone *count* |U| as a concurrency cap --
individual drones are otherwise interchangeable, so no drone-identity
variable exists in the constraints. But the `Random_Drone_Remove` shake
neighborhood (Sec 5.4) is phrased as "select a drone at random," which
only makes sense if trips carry some notion of which drone flew them.
`DroneTrip` therefore carries an optional `drone_slot` (default
"unassigned"). **[ASSUMPTION]**, already flagged in
`paper1-implementation-plan.md` Sec. 9/15; Phase B just gives it a
concrete field rather than leaving it as a future problem.

**Flexible-launch-point *placement representation* vs. *generation
rule*.** `FlexibleLaunchPointData` records where an already-generated
point sits on its arc (`arc_from`, `arc_to`, a `position_ratio` in
[0, 1]). It intentionally does **not** decide how many points get
generated per arc or at what ratios -- that is the main open
reproduction gap identified in `paper1-implementation-plan.md` Sec. 8,
and is explicitly Phase E's responsibility, not Phase B's. Building the
generation rule now, even provisionally, would mean guessing at exactly
the thing the plan says must not be silently guessed at.

**No auxiliary indices or timing on `Solution`.** `Solution` is just a
`TruckRoute` plus a `vector<DroneTrip>`. It does not cache timing,
feasibility, or lookup indices (e.g. "which trip launches from node X").
Per the plan's dependency graph (Sec. 24) and implementation order (Sec.
25), auxiliary indices belong to Phase H (built after the initial-solution
and neighborhood code exists, once real access patterns are known) and
timing/feasibility belongs entirely to Phase F's engine. Adding either
now would mean designing them without the modules that actually consume
them.

**Structural validation included; feasibility validation deliberately
not.** Two small free functions, `has_distinct_endpoints` and
`has_valid_node_roles`, reject a `DroneTrip` whose launch/service/recovery
nodes are not pairwise distinct (**[PAPER]**, Sec 2.2 states k != i, j for
every single-flight route), or whose nodes don't have the expected
`NodeType` (launch must be a flexible launch point, service must be a
customer, recovery must be a customer or the depot). Neither function
touches time, distance, or battery endurance -- a trip that is
structurally fine but physically unreachable (e.g. its flight would
exceed `endurance_seconds`) is *not* rejected by anything in Phase B;
that check belongs entirely to Phase E's timing/feasibility engine, per
the instruction not to let representation-layer validation quietly do
feasibility's job.

**Tests.** `tests/test_model.cpp` covers: all three node types and their
storage/retrieval via `NodeRegistry` (including rejection of a duplicate
node id); `CustomerData` field storage; a single well-formed `DroneTrip`
and its node-role validation, including recovery at the depot; multiple
drone trips held together; a `TruckRoute` as an ordered id sequence; a
complete `Solution` combining both; `TruckParameters`/`DroneParameters`
construction; and the explicit rejection cases (endpoint collisions, an
unset trip, a trip launching from a customer-typed node instead of a
flex point, and a trip referencing an unregistered node id).

## Geometry and travel time (Phase C)

`include/graph/` (header-only, namespace `tdr::graph`) implements the
static 2D distance and travel-time infrastructure from
`paper1-implementation-plan.md` Sec. 6, and nothing else -- no instance
generation, no flexible-launch-point placement, no timing/feasibility
evaluation. It builds only on `model/` (not `tsp/`), so that `tsp/`
could later depend on `graph/` if useful, without a circular dependency.

**Distance model.** `euclidean_distance()` implements
`d(i,j) = sqrt((x_i-x_j)^2 + (y_i-y_j)^2)` directly. **[PAPER]** Sec 2.1:
Paper 1's synthetic instances are points in a planar (60x60 km) square,
and nothing in the paper's methodology (main experiments or the VNS
itself) uses anything other than straight-line distance -- there is no
road network anywhere in this reproduction, by design (matches the plan's
Scope Lock). The function itself is unit-agnostic; the plan's 60x60 km
scale is an instance-generation detail that belongs to Phase D, not to
this formula.

**Travel time model.** `truck_travel_time()` and `drone_travel_time()`
both compute `distance / speed`. **[PAPER, Assumption 2]**: "truck and
each drone travel at a constant own speed." Table 2 names the resulting
quantities `d_ij` (truck) and `d'_ij` (drone) -- the function names here
match that directly.

**Raw drone travel time is deliberately NOT the generalized flight
time.** Table 2 lists `d'_ij` (drone travel time) and `theta_j`
(load-dependent generalized-flight-time coefficient) as two separate
parameters, and eq. 35 combines them: `t_j = f_j * (1 + theta_j)`, where
`f_j` is the actual/raw flight time. `drone_travel_time()` in this phase
computes `f_j`/`d'_ij` only. Applying `theta_j` to get the generalized
flight time `t_j` -- the quantity actually used for endurance checks
(constraints 36/37) and downstream timing -- is left entirely to the
timing/feasibility engine (Phase E/F), which already has each customer's
`theta` available via `CustomerData::generalized_flight_time_theta`
(Phase B). **[PAPER]** for the existence and separateness of `f_j` vs.
`t_j`; this phase intentionally implements only the former, per the
implementation-order boundary between Phase C (geometry/travel time) and
Phase E/F (timing/feasibility).

**Non-positive speed returns +infinity, not a silent number.**
`travel_time_for_distance()` guards against a misconfigured (e.g.
still-default, `speed = 0.0`) `TruckParameters`/`DroneParameters` by
returning `+infinity` rather than dividing by zero or returning a
finite-looking wrong answer. **[ASSUMPTION: arithmetic safety, not a
paper rule or a feasibility judgement]** -- it says nothing about
whether a route or trip is feasible; it only prevents a configuration
bug from silently producing a plausible-looking but meaningless finite
time.

**`SpatialNetwork` stores its `NodeRegistry` by value.** A reference
member would avoid a copy but risks dangling if a `SpatialNetwork`
outlives the registry it was built from. **[ASSUMPTION: engineering
choice]** -- for instance sizes here (at most a few thousand flexible
launch points, per Table 4) the copy is cheap enough that the safety is
worth it. This mirrors the value-type preference already used throughout
`model/` (Phase B).

**`build_distance_matrix()` / `build_truck_time_matrix()` /
`build_drone_time_matrix()` take an explicit, caller-supplied list of
node ids** rather than building one all-encompassing matrix over every
registered node. This keeps `graph/` from having to guess which nodes
"belong together" for a given later computation (e.g. Phase G's
truck-only TSP stage needs exactly {depot, customers}, not flexible
launch points) -- that policy decision is left to the caller. The
returned matrix type (`tdr::graph::DistanceMatrix`) is kept as an
independent alias rather than reusing `tdr::tsp::DistanceMatrix`
directly, so that `graph/` does not depend on the `tsp/` module; both
resolve to the same underlying `vector<vector<double>>` type, so a
matrix built here can be passed to `tsp/` code without any conversion.

**Tests.** `tests/test_graph.cpp` covers: `euclidean_distance()` on a
3-4-5 triangle (exact, no floating-point tolerance issues), symmetry,
zero self-distance; `truck_travel_time()`/`drone_travel_time()` on known
distance/speed pairs, and the zero-speed infinity guard;
`SpatialNetwork` construction and accessors, distance/time queries over
a 5-node fixture (depot + 3 customers + 1 flexible launch point) with
hand-checked expected values, and symmetry through the class API;
`build_distance_matrix()` and the truck/drone time matrix builders
against the same fixture, checked both for internal consistency
(diagonal zero, symmetry) and agreement with the equivalent direct
`distance()`/`truck_time()`/`drone_time()` calls. The 5-node fixture
itself lives in `tests/fixtures/small_graph_fixture.hpp` (test-only, not
part of the shipped library) specifically so Phase D/E/F tests can reuse
the same known-distance layout instead of each inventing their own.

## FLP baseline generation policy (Phase D)

`include/flp/flexible_launch_point_generator.hpp` (header-only, namespace
`tdr::flp`) implements flexible-launch-point (FLP) generation: given a
truck route and the node registry, it produces `FlexibleLaunchPointData`
+ `Node` pairs on the arcs the route actually traverses. It does not
touch timing, feasibility, drone assignment, or the initial-solution
construction -- those remain out of scope for this phase.

This is the single most paper-ambiguous component built so far, so the
[PAPER] vs. [ASSUMPTION] line is drawn explicitly and separately below,
per the review instructions for this phase.

**[PAPER] -- directly supported by the source:**
- Flexible launch points exist along the truck route, partitioned per
  arc as `L_ij` (Sec 2.1/4.1).
- A launch point is usable only if the truck's route actually traverses
  the arc it belongs to (constraints 5-6, 13-15) -- untraversed arcs
  contribute no usable launch points.
- Qualitatively, launch points divide each arc "by equal proportions of
  the distances between two locations" (Sec 2.3) -- i.e. an even,
  proportional spacing along the arc, not an arbitrary or random one.

> **SUPERSEDED IN PART (29 Sep 2026):** the fixed count per arc is no
> longer the default. Generation now follows the structure of eq. (41),
> n = floor(D/d) per arc (the printed (41) uses floor; the "ceil" below
> was a transcription error), with d configurable and NO default, because
> the paper does not give the main-experiment d. Launch points are now
> generated for every arc of the depot/customer graph, not only the arcs
> of one route. The fixed count survives only as an opt-in rule that is
> not paper-supported. See "Paper-conformance corrections (29 Sep 2026)", C7.

**[ASSUMPTION] -- our choice, because the source does not specify this
for the main synthetic experiments:**
- The exact **count** of points placed per traversed arc
  (`FlpGenerationPolicy::points_per_arc`, default 1 -- a single midpoint
  per arc) and therefore the exact **ratios** used (`k/(n+1)` for
  `k = 1..n`). Table 4 reports the *resulting* total counts for the main
  experiments (214 launch points for the 10-customer instances, 837 for
  20-customer, 3493 for 50-customer) but gives no formula connecting
  those totals to a per-arc rule, an arc count, or a spacing distance --
  they cannot be uniquely reverse-engineered into a generation rule from
  the information available. Reproducing those exact counts is
  therefore not something this baseline claims to do, and is logged in
  `paper1-implementation-plan.md` Sec. 21 as an ASSUMPTION-DEPENDENT gap,
  not something Phase D resolves.
- The `min_arc_length` zero-length-arc guard (default `1e-9`): the paper
  never discusses a truck route revisiting the same physical location
  twice in a row, so there is no stated rule for it; skipping point
  generation there is this implementation's own defensive choice, not a
  paper rule.

**Equation (41) was deliberately NOT used here.** Sec 6.3.4 gives one
fully numeric FLP rule -- a fixed unit-division length `d`, point count
`n = ceil(D/d)`, `d` swept over `[1000, 5000]` m in 500 m steps (eq. 41)
-- but it is explicitly scoped to a *separate, dedicated* 7-customer/
10-node sensitivity-analysis instance, not the main Table 3/4
experiments. Using eq. 41 as if it were the main-experiment rule would
misrepresent a sensitivity-analysis-only equation as a general
methodology decision the paper never made. `FlpGenerationPolicy` does
not implement eq. 41 at all right now; if a dedicated reproduction of
the Sec 6.3.4 sensitivity analysis is undertaken later, it should get
its own explicitly-labeled policy (e.g. a `UnitDivisionLengthPolicy`)
rather than folding eq. 41 into this baseline.

**This does not resolve the paper's FLP-generation ambiguity.** It
records one defensible, documented, swappable choice so that later
phases (initial solution, neighborhoods, timing) have something concrete
to run against. Per `paper1-implementation-plan.md` Sec. 21, any
comparison of this repository's results against Paper 1's Table 3/4 must
treat FLP placement/count as ASSUMPTION-DEPENDENT -- a mismatch there is
expected, not evidence of a bug, and is not evidence the rule has been
"found" either.

**Isolation for future replacement.** All of the above lives entirely
behind `FlpGenerationPolicy` and `generate_flexible_launch_points()`.
Nothing downstream is expected to depend on *how* a point's coordinates
were chosen -- only on the resulting `Node`/`FlexibleLaunchPointData`
shape already defined in Phase B. If the exact paper rule is ever
recovered (e.g. from the publisher's version of the PDF or from the
authors), only this one file needs to change.

**Tests.** `tests/test_flp.cpp` covers: a single arc producing a point
that lies on it; multiple consecutive arcs each producing their own
point, with an explicit check that the untraversed "shortcut" edge (0,2)
in a {0,1,2} route never appears; unique ids and correct `NodeType`
across a multi-arc, multi-point-per-arc route; an independent linear
-interpolation cross-check of generated coordinates against their
recorded ratio; a full depot-customers-depot loop confirming the
closing (last-node-to-depot) arc is treated like any other; determinism
across repeated calls with identical inputs; two distinct zero-length
cases (a repeated node id, and two different ids that happen to share
coordinates) both producing zero points without crashing; and the
count/ratio behavior of `points_per_arc` at 3, 0, and a negative value.
Reuses the Phase C `small_graph_fixture.hpp` rather than inventing new
coordinates.

## Timing and feasibility engine (Phase E)

`include/timing/timing_engine.hpp` (header-only, namespace `tdr::timing`)
implements the single centralized evaluation of a complete static
`Solution`: truck arrival/departure at every route position, every drone
trip's launch/flight/service/recovery timing, synchronization at every
recovery point, per-customer service resolution, battery-endurance
feasibility, overall solution feasibility with violation reasons, and the
Paper 1 objective (sum of each customer's own service arrival time). It
builds on `model/` (Phase B) and `graph/` (Phase C) only -- not `tsp/` or
`flp/` -- and contains no optimization, search, or dynamic-event logic of
any kind; it is a pure function of one `Solution` plus the static
instance data.

This is the highest correctness-risk module built so far, since Paper 1
states the timing relationships piecemeal across its assumptions,
constraints, and eq. 35, rather than as one worked-out example. Every
convention below is classified explicitly rather than implied by the
code.

**[PAPER] -- directly supported by the source, not a choice made here:**
- Every customer is served exactly once, by exactly one vehicle (Sec 2.2,
  Assumption 1) -- enforced by `CustomerServiceRecord` rejecting both
  "served by neither" and "served by more than one" as violations.
- Drones only launch from flexible launch points and only recover at the
  service point or the ending depot (Assumption 3) -- already enforced
  structurally in Phase B's `has_valid_node_roles`; the timing engine
  additionally refuses to compute any timing for a trip that fails this
  check (`structurally_valid = false`), rather than guessing at times for
  an invalid trip.
- Flight duration is constrained by a finite battery endurance
  (Assumption 4, constraints 36/37).
- Truck and each drone move at their own constant speed (Assumption 2) --
  reuses Phase C's raw `truck_time()`/`drone_time()` unchanged; the
  timing engine never re-derives a speed or distance itself.
- Launch preparation (`SL`), recovery (`SR`), truck service (`Se1`), and
  drone service (`Se2`) durations are all accounted for as real elapsed
  time (Table 2 parameter definitions).
- At a recovery rendezvous, whichever of the truck/drone arrives first
  waits for the other (Assumption 7) -- implemented via `max(truck_arrival,
  drone_arrival) + SR`. Note: this is a recovery-side rule only. The
  launch side is not a rendezvous between two independently-arriving
  vehicles at all -- the drone is physically carried by the truck until
  it launches, so `launch_time = arrival[p] + SL` is a direct consequence
  of constraint 21 ("drone can't launch before truck arrives + SL"), not
  an application of Assumption 7. An earlier draft of this document
  described the launch side as "the drone waiting for the truck" under
  Assumption 7 as well; that framing has been corrected here during the
  Phase E audit -- the behavior was already correct, only the citation
  was wrong.
- The objective is customer **arrival** time at the vehicle that serves
  them, not that vehicle's own recovery/completion time (Sec 2.4's
  customer-satisfaction framing, and the plan's own repeated emphasis
  that recovery time is a routing detail, not a customer-facing
  quantity). `objective_value` sums exactly
  `CustomerServiceRecord::service_arrival_time` -- for a truck-served
  customer this is the truck's `arrival_time` at that route position; for
  a drone-served customer this is `DroneTripTiming::customer_arrival_time`
  (the moment the drone reaches the customer), **never**
  `recovery_arrival_time` or `recovery_complete_time`.
- > **SUPERSEDED (29 Sep 2026):** endurance now follows (36)/(37)
  > literally (loaded portion l^_j - l^_i includes Se_j^D and is scaled by
  > (1 + theta_j); (36) includes the drone waiting for a late truck). The
  > bullets below on the outbound-only theta and on waits not counting
  > are superseded. See "Paper-conformance corrections (29 Sep 2026)", C3.
- Eq. 35's generalized flight time `t_j = f_j * (1 + theta_j)` is
  implemented literally as `DroneTripTiming::generalized_flight_time`,
  using the raw flight times already produced by Phase C and each
  customer's `theta` from Phase B.

**[INFERENCE] -- a necessary reading required to turn the paper's
piecemeal statements into one consistent timing procedure, not a free
choice:**
- **Generalized flight time is used only for endurance checking, never
  for real event timing.** Table 2 and eq. 35 introduce `t_j` purely in
  the context of the endurance constraints (36/37); nothing in the paper
  ever uses `t_j` to advance a clock or compute an arrival time -- every
  other timing-relevant statement (Assumption 2's constant speeds, the
  synchronization assumption) is phrased in terms of actual travel.
  Treating `t_j` as a *feasibility-only* multiplier, while all arrival/
  departure/launch/recovery times use the raw `f_j` throughout, is the
  only reading that keeps both eq. 35 and the constant-speed assumption
  true simultaneously. `DroneTripTiming::customer_arrival_time` and every
  other real timestamp are therefore computed from
  `outbound_flight_time_raw`/`return_flight_time_raw` only;
  `generalized_flight_time` feeds `endurance_feasible` and nothing else.
  Test Block 6 (theta=0 vs theta=0.5 on identical geometry) exists
  specifically to prove this in code, not just assert it in a comment.
- **Theta applies only to the outbound (loaded) leg, not the return
  (empty) leg.** The paper frames `theta_j` as compensating for the
  weight of the package the drone is carrying to customer `j` -- a
  property of the outbound trip, not the empty return flight. Eq. 35's
  own notation (`t_j`, indexed by the customer being served, applied once
  per single-flight-route) is consistent with this reading but does not
  literally spell out "outbound only" vs. "outbound plus return," so this
  is recorded as an inference already flagged in
  `paper1-implementation-plan.md`, not re-derived independently here.
  Implemented as `generalized_flight_time = outbound_flight_time_raw *
  (1 + theta_used) + return_flight_time_raw`.
- **A recovery position must occur strictly after its trip's launch
  position in route order.** Not a numbered paper assumption, but a
  physical necessity: the truck traverses its route once, forward, in
  increasing time, so a drone cannot recover at a route position it
  hasn't reached yet. This also drives an implementation fix worth
  recording explicitly: recovery search must start from the launch
  position and scan forward, not restart a first-occurrence search over
  the whole route. Both depots share the same `NodeId` in a round-trip
  route, so a naive "find first occurrence of this node id" search for a
  trip recovering at the ending depot would incorrectly resolve to the
  *starting* depot's route position. `find_route_position_from()` starts
  its search at `launch_pos + 1`, which is both the physically correct
  search and the fix for that ambiguity. Test Block 7 (a route ending
  with a drone recovering at the terminal depot) exercises exactly this
  case.
- **`SL`, `SR`, and `Se2` are ground time and never count toward battery
  endurance; only the two flight legs do.** Assumption 4 constrains
  *flight* duration specifically; a drone sitting on the truck being
  prepared for launch, being recovered, or servicing a customer on the
  ground is not consuming flight endurance under any reading of "flight
  duration." A synchronization wait at recovery (the drone having
  already landed and waiting for the truck, or vice versa) likewise does
  not count -- a landed drone is not flying regardless of why it is
  waiting.
- **The single forward pass is valid** (one linear scan over route
  positions computes both all truck timing and all drone trip timing,
  with no second pass or fixed point needed) because of the previous
  point: by the time the loop reaches any trip's recovery position, that
  trip's launch position has already been visited and its `launch_time`
  already stashed.
- **Truck service (`Se1`) and a drone recovery (`SR`) at the same node run
  concurrently, not sequentially: `departure = max(arrival + Se1,
  recovery_complete)`, never `arrival + Se1 + SR`.** Added/reclassified
  during the Phase E audit (see "Phase E audit" below). Constraint 18 is
  known only through an OCR-reconstructed prose paraphrase ("truck can't
  depart a node until service + drone launch/recovery ops there finish"),
  not a verified algebraic form, so this cannot be asserted as [PAPER].
  The reading implemented -- both operations are independent
  prerequisites for departure, combined via `max()` -- is the standard
  MILP-scheduling convention for "cannot proceed until X and Y both
  finish" (two independent lower-bound constraints on one departure
  variable, forced to their maximum by a minimization objective that
  wants departure as early as feasible). The alternative reading
  (service must fully finish before recovery can even start, i.e. an
  additive `Se1 + SR`) is not ruled out by anything we can verify in the
  source text. Test Block 9 in `test_timing.cpp` specifically exercises
  this: a drone recovers at a truck-served customer with nonzero `Se1`,
  with the drone's own arrival placed before, during, and after the
  truck's service window, and documents in comments what the additive
  alternative would have produced at each point for contrast.

> **SUPERSEDED IN PART (29 Sep 2026):** the time-0 start is [PAPER]
> (p15: "this paper assumes that operating time starts at 0"); the shared
> SL is replaced by (21) + (18) (each drone launches at t_i + SL, the truck
> leaves at t_i + n_i SL). The SR handling is unchanged and is now marked
> PAPER AMBIGUITY -- PROFESSOR DECISION PENDING. See "Paper-conformance corrections (29 Sep 2026)", C1, C2.

**[ASSUMPTION] -- a choice made because the paper does not specify this at
all, and a different implementation could reasonably choose otherwise:**
- **Base case: the truck departs the starting depot at time 0.** Paper 1
  never states an absolute time origin (it only ever discusses relative
  durations and arrival times). Anchoring the whole schedule at
  `arrival[0] = 0` is the simplest defensible choice and does not affect
  any relative timing, endurance check, or the objective (which only ever
  sums differences/absolute-but-relative arrival times against a
  consistent origin).
- **Simultaneous drone launches from the same node share one `SL`, and
  simultaneous drone recoveries at the same node/time share one `SR`.**
  The parallel-launch reading was already an established assumption in
  `paper1-implementation-plan.md`; this phase extends the same reasoning
  to recovery, treating "N drones and/or the truck all doing recovery
  handling at the same node at the same moment" as one shared recovery
  operation gated by the *latest* arrival among them, rather than N
  independent sequential `SR` charges. Nothing in the paper describes
  what happens when multiple recoveries coincide, so this is a genuine
  choice, made for consistency with the already-accepted launch-side
  assumption rather than picked independently.
- **A structurally invalid or off-route drone trip is excluded from
  timing entirely (marked infeasible with a specific reason) rather than
  assigned a default/zero timing that participates in the objective.**
  This keeps `objective_value` from ever silently including a fabricated
  zero time for a trip that was never actually flyable, at the cost of
  the overall solution being marked infeasible whenever this happens
  (which it should be, per Assumption 1: every customer must actually be
  served by someone).

**Do not claim exact reproduction.** Every timing rule above that is
tagged [INFERENCE] or [ASSUMPTION] is exactly that -- a necessary or
chosen reading, not a directly-quotable paper rule. In particular, the
theta-only-affects-endurance rule, the outbound-only theta application,
and the shared-SL/shared-SR simultaneity handling are all places where a
different, equally defensible implementation could produce different
numbers for the same instance. Any comparison of this repository's
computed objective values against Paper 1's own reported results must
treat this module's output as ASSUMPTION-DEPENDENT wherever these
specific conventions matter, per the same three-tier classification used
in `paper1-implementation-plan.md` Sec. 21.

**Phase E audit (post-checkpoint correctness review).** Before accepting
the Phase E checkpoint, six specific timing conventions were re-audited
against `paper1-understanding.md` Sec 2.4's constraint-by-constraint
summary (itself an OCR-reconstructed paraphrase, not verbatim algebra --
see that document's own caveat). Findings:

1. **Recovery concurrency with truck service (constraints 18-22).**
   Reclassified from an implicit [PAPER] framing to explicit [INFERENCE]
   -- see the new bullet above. The implemented behavior (`max()`, i.e.
   concurrent) was not changed, because it is the standard reading of a
   "both must finish" departure gate, but the citation was overstated
   before this audit; it is not verified as a literal equation. A hand
   worked regression (Block 9) was added specifically to pin this down
   and to make the ambiguity visible rather than silently assumed.
2. **Theta / generalized flight time (eq. 35, constraints 36-37).**
   Re-verified, not changed. Eq. 35 explicitly names `t_j` a
   *generalized* quantity derived from the *actual* flight time `f_j`,
   and `paper1-understanding.md` states plainly that (36)-(37) are (29)
   -(30) "with generalized time substituted in" for endurance purposes --
   i.e. eq. 35's `t_j` is introduced and used *only* in the endurance
   constraints, never in the arrival-time constraint (17). This is
   about as strong a textual basis as is available without the verified
   original equations, so the existing [INFERENCE] tag is kept (not
   upgraded to [PAPER], since the exact algebra of (17)/(36)/(37) is
   still an unverified reconstruction) but the confidence and its basis
   are now spelled out here rather than left implicit. No code change.
3. **Endurance excludes ground/wait time.** Re-verified, not changed. Eq.
   35 is a pure function of flight times (`f_j`, and by extension outbound/
   return raw flight) with no `SL`/`SR`/`Se2`/wait-time term anywhere in
   it, so nothing in the endurance formula could accidentally include
   ground time; `generalized_flight_time` in code only ever combines
   `outbound_flight_time_raw`, `theta`, and `return_flight_time_raw`. No
   code change.
4. **Launch timing (constraints 19, 21).** Re-verified, not changed.
   `launch_time = arrival[p] + SL` matches constraint 21 ("drone can't
   launch before truck arrives + SL") directly, and sharing one
   `departure[p] = launch_time` across every drone launching there
   satisfies constraint 19 ("all drones launched there must depart
   before the truck leaves") since they all depart at exactly that
   instant. Whether multiple simultaneous launches share one `SL` or each
   pay their own is confirmed genuinely unspecified by the paper (already
   flagged in `paper1-implementation-plan.md` Sec. 11) -- kept as
   [ASSUMPTION], not silently upgraded. Also corrected during this audit:
   the launch side was previously (mis)cited as an application of
   Assumption 7 ("first to arrive waits"); it is not a rendezvous between
   two independently-arriving vehicles at all, since the drone is carried
   by the truck until launch. See the corrected [PAPER] bullet above.
   Documentation-only fix; no behavior change.
5. **Customer waiting-time objective (eq. 1, constraints 16-17).**
   Re-verified, not changed. `paper1-understanding.md` flags eq. 1's own
   algebraic reconstruction as OCR-garbled but records the paper's prose
   gloss verbatim: "waiting time is measured by the time required for the
   truck or drone to arrive at the customer location." The implementation
   already matches this prose statement exactly (each customer's own
   arrival time, by whichever vehicle serves them, summed) and does not
   depend on resolving the reconstructed formula's indexing. No code
   change.
6. **Depot handling.** Re-verified, not changed. The existing
   `find_route_position_from()` fix (searching for a trip's recovery
   position starting just after its launch position, rather than from
   the start of the route) already ensures a drone recovering at the
   ending depot resolves to the route's *last* occurrence of that
   `NodeId`, not the first (starting-depot) occurrence, without
   introducing a fourth `NodeType`. Test Block 7 already exercises this.
   No code change.

Net result of the audit: one documentation reclassification ([PAPER] ->
[INFERENCE] for the Se1/SR concurrency rule, plus a citation correction
for the launch side), one new regression test (Block 9) proving the
concurrency behavior against hand-computed alternatives, and no changes
to any other computed value. Items 2-6 confirm the existing
implementation was already correct as designed; item 1 confirms the
*behavior* was already correct but the *documentation* overstated its
paper-fidelity.

**Tests.** `tests/test_timing.cpp` (8 test blocks, 76 assertions) covers
all 15 required categories, with every expected value hand-computed in
code comments *before* being checked -- never obtained by calling the
timing function under test. Geometries are deliberately chosen as clean
Pythagorean triples (3-4-5, 6-8-10, 8-15-17) or colinear layouts so every
expected number is exact and calculator-free: a truck-only route with
full arrival/departure propagation and a hand-summed objective; a single
drone trip where the truck reaches the recovery point first and waits;
a single drone trip where the drone lands first and waits (a colinear,
no-launch-service-time layout so the wait time is an exact integer); an
identical trip geometry checked against a tight endurance budget
(infeasible) and a generous one (feasible); a trip with a customer-typed
node in the launch-node role, rejected by the structural-validity check
inherited from Phase B; identical drone-trip geometry evaluated at
theta=0 and theta=0.5 to prove theta changes endurance feasibility but
never changes the actual customer arrival time; a mixed truck+drone
solution with two drone trips (the second trip recovering at the ending
depot, which is what exposed and pinned down the depot-id search-order
inference above) with a fully hand-computed timing array and objective
(30.5); and a solution with one orphaned, never-referenced customer,
checked for both the specific per-customer violation reason and the
top-level solution infeasibility. All prior Phase A-D tests (126
assertions) continue to pass unchanged.

## Initial-solution construction (Phase F)

> **UPDATED (29 Sep 2026):** F3 and F4 were changed to respect launch-point
> arc membership, and F4 now assigns physical drone ids and no longer uses
> `drone_fleet_capacity.hpp` (fleet limits are enforced by the evaluator).
> Passages below that describe r0-only FLP generation, entering-arc launch
> candidates, the time-sweep fleet check inside F4, `drone_slot`, and the
> old Block 4/6/8/9/10 numbers are superseded; see "Paper-conformance corrections (29 Sep 2026)", C4, C5, C7.

`include/init/` (header-only, namespace `tdr::init`) implements Paper 1's
4-step initial-solution procedure (`paper1-understanding.md` Sec 5,
`paper1-implementation-plan.md` Sec 10-11): (a) the optimal truck-only
route, (b) K-means customer clustering, (c) flexible-launch-point
insertion, (d) feasible drone-trip construction. Four files: `truck_only_route.hpp`
(F1), `customer_clustering.hpp` (F2), `drone_fleet_capacity.hpp` (the
global drone-fleet-capacity check used by F4, added by the post-approval
correctness patch described below), and `initial_solution_builder.hpp`
(F3 + F4 + top-level orchestration, `build_initial_solution()`). This phase
adds **no** dynamic-customer machinery, **no** objective changes, and
**no** VNS/local-search/shake code of any kind -- it only produces one
concrete `Solution` for the VNS driver (a later, not-yet-built phase) to
start from.

**Architecture rule carried over from Phase E, restated because it
governs every design choice below:** this module never computes or
duplicates timing, endurance, or objective logic. F4's candidate
construction only ever builds a candidate `Solution` and asks
`tdr::timing::evaluate_solution` (unchanged) whether it is feasible and
what its numbers are -- never reimplements any part of that engine.

### F1: truck-only route, and the CPLEX limitation

**[PAPER]** Sec 5.2: "the truck-only route is obtained by solving the TSP
to optimality with CPLEX" -- an explicitly EXACT step in Paper 1's own
methodology.

**CPLEX limitation -- an implementation-environment fact, not a Paper 1
rule.** CPLEX is not available in this environment. This is the exact
same gap already documented in this file's very first entry ("TSP solver
for stage (a) of the initial solution"), from Phase A -- Phase F does not
rediscover or re-litigate it, it simply reuses the resolution Phase A
already built: `tsp::TSPSolverFactory`, which runs `HeldKarpSolver` (a
genuinely exact dynamic program, a certified global optimum whenever it
runs) when the instance is small enough, and falls back to
`HeuristicTSPSolver` (explicitly `exact = false`, with a `note` string
saying so) only when Held-Karp refuses. `init::build_truck_only_route()`
(`truck_only_route.hpp`) is a thin wrapper around this existing factory:
it builds the distance matrix (over `{depot} + customers`, depot fixed at
matrix index 0 per the factory's own contract), calls
`TSPSolverFactory::solve()`, and maps the returned tour (matrix indices)
back to actual `NodeId`s. It introduces no new solving logic and never
overrides or discards the `exact` flag -- a heuristic result is passed
through labeled exactly as such. For the instance sizes Paper 1 targets
(10/20/50 customers, i.e. 11/21/51 TSP nodes with the depot), the default
`max_exact_n = 21` means the 10- and 20-customer instances get a
certified-exact truck-only route and only the 50-customer instances fall
back to the heuristic -- exactly the same EXACTLY-REPRODUCIBLE /
ASSUMPTION-DEPENDENT split already recorded in this file's first entry.

**[ASSUMPTION: engineering choice, not a fidelity concern]** The TSP is
solved over truck TRAVEL TIME (`SpatialNetwork::build_truck_time_matrix`),
not raw Euclidean distance. Under Assumption 2 (constant truck speed)
these are related by one fixed positive scale factor, so minimizing one
always minimizes the other and the returned tour is identical either way.

### F2: K-means clustering

**[PAPER]** The use of K-means itself, and the explicit cluster-count
formula `k = ceil(|C| / (|U| + 2))` (`paper1-understanding.md` Sec 5;
`compute_cluster_count()` implements this literally).

**[ASSUMPTION]** Everything about *how* K-means is actually run is
unspecified by the paper (`paper1-implementation-plan.md` Sec 10:
"[ASSUMPTION] seeding/convergence details"), so `run_kmeans()`
(`customer_clustering.hpp`) makes one simple, fully deterministic,
no-randomness choice for each, rather than a more sophisticated variant
(k-means++, multi-restart) the paper gives no basis for:
- **Seeding:** customers sorted by `NodeId` ascending, then k
  evenly-index-spaced customers from that list (index `floor(m*n/k)` for
  `m = 0..k-1`) become the initial centroids. A pure function of the
  input, no RNG.
- **Convergence:** standard Lloyd's algorithm, stopped when no customer's
  assignment changes OR the largest centroid movement in one iteration is
  below `KMeansPolicy::convergence_epsilon` (default `1e-9`), OR
  `max_iterations` (default 100) is reached.
- **Tie-breaking:** a customer exactly equidistant from two or more
  centroids is assigned to the LOWEST-indexed centroid.
- **Empty clusters:** left in place (not reseeded); an empty cluster is a
  legitimate, harmless outcome -- F4 simply has nothing to do for it.
- k is clamped to at most the number of customers as a defensive
  measure for direct callers (e.g. tests); the paper's own formula can
  never actually produce k > |C|, so this clamp is not expected to
  trigger in practice.

### F3: flexible-launch-point insertion

> **SUPERSEDED (29 Sep 2026):** F3 now generates L_(a,b) for every ordered
> pair of truck nodes (depot + customers) with n = floor(D/d), and F4's
> launch candidates for customer j are the points of the arc (a, b) the
> route traverses once j is removed. The text below describes the earlier
> r0-only / entering-arc design, which produced launch points outside
> their own arc. See "Paper-conformance corrections (29 Sep 2026)", C5.

**Phase D's generator is reused completely unchanged** -- no new
placement rule, no eq. 41, nothing. `build_initial_solution()` calls
`tdr::flp::generate_flexible_launch_points()` exactly once, passing it
`r0` (F1's full truck-only route, before any customer is converted to
drone service) and the caller-supplied `FlpGenerationPolicy` (default:
Phase D's own default of one point per arc, at the midpoint).

**[ASSUMPTION] The candidate FLP set is generated once, from r0, and then
FROZEN for the rest of construction** -- it is never regenerated,
extended, or invalidated as F4 removes customers from the working route.
The paper's own (a)-(d) step list does not say whether FLP generation
should be redone after each drone-trip acceptance; freezing it once is
the narrowest reading available, since anything else would require
arc-invalidation/regeneration machinery the source never describes, and
it keeps F3 and F4 fully decoupled (matching Phase D's own stated
isolation design, `DECISIONS.md`'s "FLP baseline generation policy"
entry). A visible consequence: once a customer is converted, the truck's
new route can contain a "stray" FLP whose coordinates were only ever
meant to be a fractional waypoint toward the now-removed customer,
producing a locally odd path shape. This is a real, acknowledged
limitation, not an oversight -- it is exactly the kind of local
inefficiency the (out-of-scope) VNS local search exists to improve later,
and inventing route-repair logic here would exceed this phase's mandate
("do not invent a more sophisticated algorithm").

**[INFERENCE/ASSUMPTION] Each customer's launch candidates are looked up
directly** as the FLPs generated on the arc that entered it in `r0` (the
generated points whose `data.arc_to == customer_id`) -- a map lookup
(`launch_candidates_for`), never a route search. This is the narrowest
defensible reading of Algorithm 2's OCR-corrupted "adjacent nodes"
wording (see F4 below for the parallel reasoning on the recovery side);
it does not search earlier arcs or a geometric radius.

**ACTUAL CURRENT IMPLEMENTATION (post-approval correctness patch, Issue
2): ALL same-arc FLPs are launch candidates.** When `points_per_arc > 1`,
several FLPs share the same arc (hence the same `arc_to`), and every one
of them is kept as a separate candidate; F4 builds and evaluates one
candidate trip per launch FLP (see F4). *Superseded original behavior,
kept here for history only:* the first Phase F version kept only the
LAST FLP written per arc, i.e. only the FLP closest to the customer.
That was an unintended narrowing: the closest point is not always the
best one (e.g. when the drone is faster than the truck, launching
earlier from a farther point can give an earlier customer arrival).
`tests/test_init.cpp` Block 10 is the regression for this.

### F4: feasible drone-trip construction

**[PAPER]** Sec 5.2 step (d) / Algorithm 2's recoverable structure: an
outer loop over clusters, an inner loop that builds candidate drone
trips and repeats "while unassigned drones remain," collecting feasible
candidates and picking the best via a `BestRoute()` selection, until the
cluster is exhausted (`paper1-understanding.md` Sec 5;
`paper1-implementation-plan.md` Sec 10-11). The OCR corruption in the
source pseudocode (e.g. "sible drone_route i", "while j = n") means only
this outer-loop/inner-loop/collect/select/repeat shape is treated as
reliable; nothing about the pseudocode's line-level detail is assumed.

**[ASSUMPTION] Clusters are processed in ascending cluster-index order.**
Not specified by the paper; a simple, deterministic, arbitrary-but-fixed
choice.

**ACTUAL CURRENT IMPLEMENTATION (post-approval correctness patch, Issue
1): drone-fleet capacity is checked GLOBALLY across all accepted trips,
and drones can be reused sequentially across clusters.** There is no
per-cluster drone budget. `fleet_size` (= `|U|`, i.e.
`DroneParameters::num_drones`) and the trip counter are single running
values shared by the whole construction; nothing is reset when a new
cluster starts. A candidate trip is accepted only if the set of ALL
trips accepted so far (in every cluster processed up to that point) plus
the candidate never needs more than `|U|` drones in the air at the same
instant. `tdr::init::check_drone_fleet_capacity()`
(`drone_fleet_capacity.hpp`) decides this.
- **What the check does:** each trip occupies one drone over the interval
  [`launch_time`, `recovery_complete_time`]. Both values are read
  unchanged from Phase E's `DroneTripTiming`; no timing is recomputed.
  A sweep over these intervals finds the maximum number that overlap at
  any instant, and the check passes iff that maximum is at most `|U|`.
- **[INFERENCE]** Max overlap ≤ `|U|` is necessary and sufficient for the
  trips to be assignable to `|U|` physical drones with no drone flying
  two trips at once. This is the standard interval-scheduling result
  applied to [PAPER] constraint 28 (a drone cannot be relaunched before
  it is recovered) and the fixed fleet size `|U|`.
- **[ASSUMPTION]** Touching intervals do not overlap: a drone whose
  recovery completes at time t can launch again at exactly t. At tied
  timestamps, releases are processed before acquisitions (with
  `kTimeEpsilon` tolerance).
- **Consequence -- sequential reuse:** because the check is time-aware
  rather than a count of trips, one physical drone can fly a trip in one
  cluster and then another trip in a later cluster, as long as the two
  intervals do not overlap (`tests/test_init.cpp` Block 9). A candidate
  that would need a second simultaneous drone in a 1-drone fleet is
  rejected even if it comes from a different cluster (Block 8).
- **Scope:** this check exists only inside F4.
  `tdr::timing::evaluate_solution()` does NOT enforce `num_drones` (see
  "Pre-Phase-G baseline" at the end of this file).

*Superseded original behavior, kept here for history only:* the first
Phase F version reset an `available_drones = |U|` budget at the start of
EACH cluster. That capped the NUMBER of trips per cluster but did not
check simultaneous flights across the whole timeline, so it could accept
more than `|U|` drones airborne at once across cluster boundaries, and
it could also block legitimate sequential reuse. The post-approval patch
replaced it with the global, time-aware check above.

**[INFERENCE/ASSUMPTION] Recovery candidate selection** mirrors the
launch side's "narrowest adjacent reading": for a candidate customer `j`
at its current route position, the recovery candidate is the nearest
FOLLOWING node of type Customer or Depot still on the working route
(skipping over any already-inserted FLP entries). Exactly one recovery
node `k` is used per customer per round. One candidate trip `(i, j, k)`
is built for EACH launch candidate `i` of `j` (every same-arc FLP, see
F3), so a customer yields as many candidate trips per round as its
entering arc has FLPs. No search is made over alternative recovery
nodes, which avoids an invented, paper-unsupported combinatorial search
on the recovery side.

**[ASSUMPTION] `BestRoute()` scoring: the candidate with the LOWEST
resulting drone customer-arrival time wins**, ties broken by lowest
customer `NodeId`, then by lowest launch-FLP `NodeId` (needed now that
one customer can have several candidates). Exactly one candidate is
committed per round. The committed trip gets `drone_slot =` a running
per-trip counter (0, 1, 2, ...); this is a trip identifier, not a
physical drone assignment. The paper names a `BestRoute()` selection step but
never gives its scoring formula (`paper1-implementation-plan.md` Sec 10:
"exact `BestRoute` scoring formula not stated for this step"). Minimizing
the customer's own arrival time directly serves Paper 1's own stated
objective (total customer waiting time), which is why it was chosen over
an arbitrary tie-break.

**[ASSUMPTION, engineering: correctness-preserving, not paper-derived]
Recovery-locking.** Once a node is accepted as some trip's recovery
target, it is added to a `recovery_locked` set and is never afterward
considered as a candidate for its OWN conversion to drone service, even
in a different, later-processed cluster. Without this, a later round
could convert a node that an earlier-accepted trip already depends on as
its truck-served recovery point, silently invalidating that earlier
trip. This does not restrict recovery-node REUSE across multiple
different trips (Fig 6 explicitly permits multiple drones converging on
shared infrastructure; Phase E's timing model supports simultaneous
recovery at one node, though no current test exercises 2+ recoveries at
the same node) -- it only prevents converting an in-use recovery target
itself. The depot is never recovery-locked, so in the current
construction simultaneous recoveries can occur only at the end depot.

**Every feasibility decision is made by Phase E, not reimplemented
here.** For each candidate, F4 builds a trial `Solution` (working route
with the candidate customer's slot replaced by its launch FLP, trip
appended to the already-accepted trips) and calls
`tdr::timing::evaluate_solution()`. A candidate is kept only if: the new
trip's own `DroneTripTiming::feasible` is true, the candidate customer's
own `CustomerServiceRecord` shows `served_by_drone && feasible`, the
whole trial solution's `feasible` is true, AND
`check_drone_fleet_capacity()` over all of the trial's trip timings
passes (see above). This directly satisfies every
numbered feasibility requirement in the phase brief (valid FLP launch,
recovery later on the route, endurance, synchronization) without
duplicating any of Phase E's logic -- "recovery occurs later on the
route" in particular is enforced entirely by Phase E's own `on_route`
check (recovery position strictly after launch position), not by any
separate check in this file.

**Duplicate-service prevention and full coverage** follow directly from
the mechanics above: a converted customer is removed from the working
route and added to `converted` (never reconsidered); every
not-yet-converted customer remains in the working route (still
truck-served) by construction. There is no code path that leaves a
customer in neither state, and Phase E's own customer-record resolution
(already tested in Phase E) independently confirms this for every built
solution.

**Final independent evaluation.** After construction ends, the finished
`Solution` (final working route + all accepted trips) is evaluated once
more from scratch by `tdr::timing::evaluate_solution()`. Its result is
stored in `InitialSolutionResult::evaluation`, and `feasible` /
`objective_value` are copied from it. If it is infeasible, every
violation reason is added to `diagnostics` with an `F4:` prefix. The
Block 6/8/9/10 tests also cross-check this by rebuilding a
`SpatialNetwork` from the result's own `node_registry` and re-running
`evaluate_solution()`.

**Tests.** `tests/test_init.cpp` currently has **10 blocks and 107
runtime assertions** (Blocks 1-6: 64 assertions from the original Phase F
version; Blocks 7-10: 43 assertions added by the post-approval
correctness patch). Blocks 1-6 cover all 10 originally required
categories. Block 1 checks `compute_cluster_count()` against
the paper's formula on seven `(|C|, |U|)` pairs, including the `|C|=0`
and `|U|=0` edges. Block 2 hand-traces `run_kmeans()` on a 4-point
colinear layout (customers at x=10/20/30/40, k=2) to its exact expected
partition `{1,2}`/`{3,4}` and centroids `(15,0)`/`(35,0)` after exactly 2
Lloyd iterations, then separately checks determinism (two calls on
identical input produce identical assignments/centroids), a
well-separated "obvious clusters" geometry, and the empty-input/`k>n`/
`k=0` edge cases. Block 3 exercises F1 (`build_truck_only_route`) on the
same 4-point line, asserting only direction-independent invariants --
`exact==true`, cost `==80.0` (`2 x` the maximum pairwise distance, true
for any tour direction on this instance), and route
membership/structure -- deliberately not a specific visiting order,
since a symmetric-metric optimum is never unique (see the note at the
top of the file on how `HeldKarpSolver`'s specific, but fully
deterministic, tie-break was established once by observation rather than
assumed). Block 4 is the endurance-infeasible-candidate-rejection test:
a single customer placed far enough from its only feasible launch point
that no drone trip can satisfy the endurance budget, asserting the
resulting solution has zero drone trips, the customer stays
truck-served, and the (truck-only) objective is exactly the hand-summed
round-trip cost. Block 5 is the recovery-before-launch-rejection test:
a hand-built `Solution` whose one `DroneTrip` names a recovery node that
sits earlier in the route than its own launch node, calling
`evaluate_solution()` directly and asserting `on_route==false` for that
trip, `feasible==false` for both the trip and the overall solution.
Block 6 is the full end-to-end integration test -- 4 colinear customers,
1 drone -- built entirely by hand from the actual (observed-once,
deterministic) TSP tour, FLP placement, and per-candidate drone-arrival
arithmetic: it asserts the F1 tour's exactness and cost, the F2 cluster
partition, the F3 candidate-FLP count and exact arc-midpoint geometry,
the F4 drone-trip count and exact customer/recovery pairing for both
constructed trips, overall feasibility, the hand-computed objective
(157.5), that no customer is ever marked served by both vehicles, that
every customer has exactly one service record (no duplicates, no
omissions), that the truck/drone service split is exactly 2/2, and --
as an independent cross-check -- that rebuilding a `SpatialNetwork` from
the result's own `node_registry` and re-running `evaluate_solution()`
from scratch agrees with the result's stored feasibility and objective
value. Blocks 7-10 cover the post-approval correctness patch:
Block 7 unit-tests `check_drone_fleet_capacity()` directly (zero trips,
touching intervals, an overlapping pair with 1 vs 2 drones, a staggered
triple needing at most 2). Block 8 is a full pipeline run where a
different cluster's candidate is rejected because accepting it would
need 2 simultaneous drones from a 1-drone fleet (objective
172.360679774998). Block 9 is a full pipeline run where two different
clusters' trips do not overlap in time and share one drone sequentially
(objective 2732.5). Block 10 uses 2 FLPs per arc and a drone faster than
the truck to prove that every same-arc candidate is tried: the FARTHER
FLP wins (objective 60 instead of 75). Blocks 8-10 each also re-run
`evaluate_solution()` independently.

> **SUPERSEDED (29 Sep 2026):** current totals are 480 assertions; see
> "Paper-conformance corrections (29 Sep 2026)", "Tests".

**Current suite totals:** Phase A-E tests 29 + 43 + 26 + 35 + 99 = 232
assertions (the last figure includes the Phase E audit's Block 9), plus
107 in `test_init`: **339 assertions across 6 executables, 100% passing**
(verified 27 Sep 2026 on a fresh build of commit aebc03f). *Superseded
figure, for history only:* the original Phase F entry reported 64
`test_init` assertions and a 296-assertion suite, from before the
post-approval patch.

## Paper-conformance corrections (29 Sep 2026)

This pass implements ONLY the corrections confirmed by the forensic paper
audit (`docs/paper1_full_implementation_audit.md`,
`docs/paper1_decision_register.md`), where the paper's behaviour is
unambiguous. It does not start Phase G, does not implement any paper
ambiguity or professor-pending decision, and does not choose any missing
paper parameter. Paper page numbers are PDF pages (journal page = 4071 +
PDF page) of Han, Chi & Sun (2026), ITOR 33:4072-4109.

Classification used below: **[PAPER]** = stated by an equation or the
text; **[INFERENCE]** = forced by combining paper statements;
**[IMPLEMENTATION ASSUMPTION]** = our choice where the paper is silent;
**PAPER AMBIGUITY / PROFESSOR DECISION PENDING** = deliberately not
decided.

### C1. Multiple launches at one launch point: (18) and (21)

- **Paper evidence.** (21): t_i + SL <= l^_{i,u} for every drone u
  launched at i. (19): l^_{i,u} <= l_i. (18): the truck's departure
  l_i >= t_i + Se_i^T + sum_u sum_{s in P_i^+} y_s^u * SL (+ an SR term,
  see C2). With n_i drones launched at i, the SL sum is n_i * SL.
  Neither (19) nor (21) orders the drones against each other.
- **Old behaviour.** One shared SL: the truck left at t_i + SL and every
  drone launched at that same instant, however many launched.
- **New behaviour.** Each launched drone departs at t_i + SL (no
  staggering); the truck departs at t_i + n_i * SL. Se_i^T is 0 at a
  launch point (Se^T is defined only for truck service nodes).
- **Files / functions.** `include/timing/timing_engine.hpp`,
  `evaluate_solution()` forward pass (launch-point branch) and header.
- **Tests.** `test_timing` A: SL = 5, two drones at one FLP reached at
  t = 10 both launch at 15; the truck leaves at 20; objective 68.
- **[IMPLEMENTATION ASSUMPTION] launch-delay slack.** (19)/(21) allow
  any l^_{i,u} in [t_i + SL, l_i], and the MILP may also hold the truck
  longer. The evaluator uses the earliest schedule (earliest launch, truck
  leaving as soon as (18) allows). Delaying a launch never makes a
  customer arrive earlier, but it can shorten the drone's wait in (36);
  the evaluator therefore may reject a trip that a delayed launch would
  make endurance-feasible. Recorded, not resolved.

### C2. SR -- PAPER AMBIGUITY, PROFESSOR DECISION PENDING (unchanged)

- **Paper evidence.** (18) prints its SR sum over P_i^+ (routes LAUNCHED
  at i). The explanatory text (p15) says (18) accounts for drone launch
  and recovery tasks at a node, and (20) separately imposes
  l_k >= l^_{j,u} + d^_{jk} + SR for each drone u recovered at k.
- **Reading (A), literal.** n_i * SR is charged at the launch point i as
  printed; recovery at k is governed only by (20):
  l_k = max(t_k + Se_k^T, max_u(drone arrival_u + SR)).
- **Reading (B), index slip (P_k^- intended).** At a recovery node k,
  l_k = max(t_k + Se_k^T + r_k * SR, max_u(drone arrival_u + SR)), with
  r_k the number of drones recovered at k.
- **Current code (NOT changed).** recovery_complete = max(t_k, drone
  arrival) + SR per trip and l_k = max(t_k + Se_k^T, all recovery
  completions). It matches neither reading exactly. It stays as the
  baseline until the professor decides; no new rule was implemented.
- **Files.** `timing_engine.hpp` header ("PAPER AMBIGUITY: SR") and the
  recovery branch comment. The existing Se1/SR audit test (test_timing
  audit Block 9) is unchanged and now notes that its value is pending.

### C3. Battery endurance exactly as (36)/(37)

- **Paper evidence.** (36): e >= (t_k - l^_{j,u}) + (l^_{j,u} -
  l^_{i,u})(1 + theta_j). (37): e >= (l^_{j,u} - l^_{i,u})(1 + theta_j)
  + d^_{jk}. With (17) t^_{j,u} = l^_{i,u} + d^_{ij} and (22)
  l^_{j,u} = t^_{j,u} + Se_j^D. Text p15: "the flight duration of a
  drone, while awaiting the truck, must not exceed the permissible
  battery capacity"; p30: "including waiting for the truck at the merging
  point".
- **Old behaviour.** generalized = outbound * (1 + theta) + return. It
  dropped Se_j^D (and its theta scaling) from the loaded part and ignored
  the drone's wait for a late truck.
- **New behaviour.** Both expressions are evaluated literally with the
  schedule (l^_i = launch time, l^_j = departure from the customer, t_k =
  truck arrival at the recovery node). Feasible iff both <= e (+1e-9).
  `DroneTripTiming` exposes `endurance_value_36`, `endurance_value_37`
  and `truck_arrival_at_recovery`; `generalized_flight_time` = the larger
  of the two. Consequences of the equations (not added rules): theta
  scales outbound flight AND Se_j^D; the return leg is unscaled; waiting
  for a late truck counts via (36); SL and SR are in neither expression.
- **Files / functions.** `timing_engine.hpp`: `DroneTripTiming`,
  per-trip feasibility assembly, header.
- **Tests.** `test_timing` B (Se2 = 2, theta = 1: (36) = 15, (37) = 19;
  e = 17 infeasible although the old formula gave 15, e = 19 feasible) and
  C (the drone lands at 12, the truck arrives at 18: (36) = 13 > (37) = 7;
  e = 10 infeasible, e = 13 feasible). Updated expectations: Block 2
  (generalized 9 -> 12: (36) = 1.2, (37) = 12), Block 8 ((36) = 12,
  (37) = 6), Block 9 boundary comment, Block 11 (theta sweep re-derived:
  e = 13; theta = 0 gives 12, feasible; theta = 0.5 gives 15.5,
  infeasible). All re-derived by hand.
- **Supersedes** the Phase E [INFERENCE] bullets "theta applies only to
  the outbound leg" and "a synchronization wait does not count", and
  resolves Pre-Phase-G UNRESOLVED item 5 (theta part).

### C4. Physical drone identity u in U, (28) and |U|

- **Paper evidence.** Drone-indexed variables y_s^u, t^_{i,u}, l^_{i,u}
  for u in U (Table 2); (28) and its text (p15): it prevents "the
  relaunching of drones without truck recovery"; |U| is the fleet size;
  p9: drones are capable of multiple takeoffs and recoveries (Fig. 3).
- **Old behaviour.** `DroneTrip::drone_slot` was a per-trip counter, not
  a drone. The evaluator never read `num_drones`; only F4 checked fleet
  capacity, with a time-interval sweep.
- **New behaviour.** `DroneTrip::drone_id` (default `kUnassignedDroneId
  = -1`) names the physical drone. The evaluator rejects: an id outside
  {0, ..., |U|-1}; a drone relaunched before its previous trip is
  recovered ((28)); more than |U| trips in flight at once
  (`SolutionEvaluationResult::max_simultaneous_drones`). Sequential reuse
  is valid: the number of trips is not bounded by |U|.
- **[INFERENCE] route-order form of (28).** Because the truck visits
  route positions in increasing time and every drone is launched and
  recovered by the truck, "relaunch before recovery" is equivalent to:
  for each drone, trips taken in launch-position order, the next launch
  position is strictly after the previous recovery position. Assumes
  positive travel times between distinct positions.
- **[IMPLEMENTATION ASSUMPTION] id assignment in Phase F.**
  `detail::assign_drone_ids_by_route_order`: trips in launch-position
  order take the lowest-numbered drone already recovered. For intervals
  processed by start this uses the minimum number of drones, so it fails
  only when more than |U| trips genuinely overlap. Which free drone is
  chosen is not specified by the paper (Algorithm 2's per-drone loop is
  professor-pending).
- **Files / functions.** `include/model/drone_trip.hpp` (`drone_id`,
  `kUnassignedDroneId`); `timing_engine.hpp` (`drone_id_valid`, (28)
  and |U| checks); `include/init/initial_solution_builder.hpp`
  (`assign_drone_ids_by_route_order`, used in F4);
  `include/init/drone_fleet_capacity.hpp` is no longer used by F4 and is
  retained only as a standalone utility (status note in its header).
- **Tests.** `test_timing` D/D2 (valid sequential reuse: one drone, three
  trips, |U| = 1), E (INVALID overlapping reuse: same id relaunched before
  recovery -> (28); the same trips on two ids are feasible), F (id 1 with
  |U| = 1 and id -1 rejected), G (three trips airborne at once with
  |U| = 2 -> "3 drones are simultaneously in flight but |U| = 2");
  `test_model` renamed field check; `test_init` Blocks 6, 8, 9 assert the
  ids (Block 9: both trips on drone 0).

### C5. Launch-point arc membership (evaluator AND construction)

- **Paper evidence.** p9: "If the truck does not pass through arc (i,j),
  it cannot access the launch point in set L_ij"; L_ij is defined for
  every i in C0, j in C_E (Table 2); (13)-(15) and their text (p14).
- **Old behaviour.** The evaluator never checked which arc a launch point
  belonged to. F3 generated points only on the arcs of r0, and F4 put a
  point of L_(a,j) (the arc entering customer j in r0) in j's place when
  converting j -- after which the truck drives a -> point -> b, an arc
  (a, b) the point does not belong to. Phase F could therefore build
  solutions the paper forbids.
- **New behaviour (evaluator).** Every launch point on the route must
  have arc-ownership data, and its owning arc (i, j) must equal the
  truck (non-launch-point) nodes immediately before and after it; several
  points of the same arc may sit next to each other. Order among points
  of one arc is not constrained by the paper and not checked.
  `evaluate_solution` takes a 4th argument `FlpDataMap flp_data`.
- **New behaviour (construction).** F3 generates L_(a,b) once for every
  ordered pair of truck nodes (depot + customers). In F4, for customer j
  with truck neighbours a and b, the candidates are the points of
  L_(a,b) -- the arc the route traverses once j is removed -- placed in
  j's slot, recovery at b. `InitialSolutionResult::flp_data` returns the
  ownership map. The evaluator was not weakened; construction now
  satisfies it.
- **[IMPLEMENTATION ASSUMPTION]** generating L for all arcs up front is
  equivalent to generating it lazily (a pure function of the arc and the
  policy). Using the bridging arc (a, b) is the narrowest reading of step
  (c) "insert launch points on the current truck route" (p19) that obeys
  the membership rule; recovery at b is the unchanged narrow recovery
  reading.
- **Consequence (disclosed, not a new rule).** Once a point of L_(a,b) is
  on the route, F4 cannot later convert a (the point would end up between
  a's predecessor and b), and b is recovery-locked. With recovery at b,
  F4-built trips therefore never overlap in route positions, so F4 never
  uses more than one drone at a time even when |U| > 1. Richer
  multi-drone construction depends on the professor-pending Algorithm 2
  recovery/per-drone details.
- **Files / functions.** `timing_engine.hpp` (membership check);
  `initial_solution_builder.hpp` (F3 `all_arcs`, `flps_on_arc`, F4
  candidate loop).
- **Tests.** `test_timing` H (a point of L_(1,2) used while the truck
  drives 1 -> 3; control: on its own arc it is accepted) and I (a point of
  L_(1,2) between C3 and C4; a point without ownership data). `test_init`
  Block 11 = test L and `check_phase_f_invariants` (applied to Blocks 6,
  8, 9, 10, 11).

### C6. Launch-point visit rules (6), (8), (9)

- **Paper evidence.** (6): a launch point the truck visits launches at
  least one drone. (8)/(9): a launch point is entered at most once and
  left at most once.
- **Old behaviour.** Not checked: an idle or repeated launch point was
  accepted.
- **New behaviour.** The evaluator rejects a launch point with no launch
  ("... is visited but no drone is launched there (constraint (6))") and
  a launch point appearing more than once in the route ("... appears more
  than once ... (constraints (8)-(9) ...)").
- **Files.** `timing_engine.hpp`.
- **Tests.** `test_timing` J (idle point) and K (repeated point; control:
  two different points of the same arc pass the membership check, only
  the idle one fails (6)). Existing fixtures were adjusted so no test
  relies on an idle point.

### C7. Launch-point generation: distance-dependent, d unresolved

- **Paper evidence.** Sec 2.3: points divide an arc "by equal proportions
  of the distances"; p18: "the number of flexible points inserted is
  proportional to the truck's driving distance"; eq. (41), n = floor(D/d),
  given in the launch-point-division sensitivity analysis (Sec 6.3.4,
  p33).
- **Old behaviour.** A fixed number of points per arc (default 1, the
  midpoint), independent of arc length.
- **New behaviour.** `FlpCountRule::UnitDivisionLength` (default):
  n = floor(D/d) points at ratios m/(n+1) (a 1e-9 guard keeps exact
  multiples exact). `unit_division_length` (d) has **no default**: it is
  0, and generation fails with an explanatory error (`ok = false`) until
  the caller sets it; the builder surfaces it as an `F3:` diagnostic.
  `FlpCountRule::FixedCountPerArc` is kept as an opt-in rule, labelled not
  paper-supported. New pure `generate_flexible_launch_points_on_arcs()`
  plus `flp_count_for_arc()`; the route-based overload remains.
- **UNRESOLVED: the main-experiment d.** The paper does not state it. The
  sensitivity-analysis d values are NOT treated as the main value, and
  Table 4's 214 / 837 / 3493 counts are NOT used as a target. Tests use
  explicitly labelled test-fixture values of d only.
- **Files.** `include/flp/flexible_launch_point_generator.hpp`,
  `initial_solution_builder.hpp` (F3).
- **Tests.** `test_flp` blocks 10-15 (unset or negative d refused;
  floor(5/2) = 2 at ratios 1/3 and 2/3; exact multiple 10/2.5 = 4; d
  longer than the arc gives 0; 7.99 -> 3 vs 8 -> 4 at d = 2;
  proportionality; depot and closing arcs; arc-list ownership and ids);
  blocks 1-9 now use the opt-in fixed-count rule explicitly. `test_init`
  Block 12 (default policy -> F3 error, no solution).

### C8. Launch / recovery location model (unchanged)

Launch only at a flexible launch point; recovery only at a truck-served
customer or the ending depot, as in the formal model. Some figures and
neighbourhood descriptions show launches from customers or the depot:
**PAPER CONTRADICTION -- PROFESSOR DISCUSSION**. No customer launches
were introduced.

### C9. Literal printing errors: coherent interpretation implemented

- **(16)** prints "l_j + d_ij - (1 - x_ij)M <= t_j"; relating node j's
  departure to its own arrival makes (16) meaningless, so l_i is
  implemented (arrival = previous departure + travel time). Documented in
  the `timing_engine.hpp` header.
- **(13)-(15)**: read literally, their quantifiers would forbid every arc
  into or out of any launch point; the text's meaning (a point of L_ij is
  accessible only while the truck drives (i, j)) is implemented (C5).
- **Time origin**: reclassified from [ASSUMPTION] to **[PAPER]** (p15:
  "this paper assumes that operating time starts at 0").

### Tests

Every expected value was derived by hand. The Phase F values were also
cross-checked against an independent Python re-implementation of F3, F4
and the corrected evaluator (written from the equations, not from the
C++); both agree on every block. New or rewritten:

- `test_timing`: regression blocks A-K (above), plus updated Blocks 2,
  8, 9, 11-14 and the audit block (drone ids, arc-ownership data,
  (36)/(37) values).
- `test_flp`: blocks 10-15 (d-based count and errors).
- `test_model`: `drone_id` default and per-trip id.
- `test_init`: Blocks 4, 6, 8, 9, 10 re-derived under the corrected F3/F4
  with test-fixture d (objectives 250, 142.0833, 170.2453052010712,
  2575.6667, 468; the old 100 / 157.5 / 172.36 / 2732.5 / 60 came from
  the superseded design); Block 5 now passes ids and ownership data;
  Block 7 retained (utility no longer used by F4); new Block 11 = test L
  (7 customers, |U| = 2, non-zero SL/SR/Se/theta, objective
  820.1434959367223, every invariant checked directly); new Block 12
  (unset d).

Test-to-requirement map: A two launches at one FLP; B endurance with
Se^D and theta; C drone waiting for the truck; D valid sequential reuse;
E invalid overlapping reuse (relaunch before recovery); F invalid id;
G too many simultaneous drones; H launch point from an untraversed arc;
I launch point between unrelated nodes; J idle launch point; K repeated
launch point; L Phase F solution with valid launch-point arc membership
(`test_init` Block 11 plus `check_phase_f_invariants`).

**Suite result (29 Sep 2026).** Fresh `-Wall -Wextra -Wpedantic` build:
no warnings; `ctest` 6/6. Per executable: test_tsp 29, test_model 43,
test_graph 26, test_flp 59, test_timing 152, test_init 171 = **480
assertions, 0 failures**. The same 480 pass under
AddressSanitizer + UndefinedBehaviorSanitizer with no findings. (Before
this pass: 339.)

## Pre-Phase-G baseline (updated 29 Sep 2026)

This entry records what the repository actually contains before any
optimization work starts. First recorded 27 Sep 2026 at commit `aebc03f`;
updated 29 Sep 2026 for the paper-conformance corrections above
(uncommitted working-tree changes on top of commit `3f1130e`).

**Frozen scope.**
- Phases A-F (TSP, data model, geometry, FLP generation, timing engine,
  initial solution) form the baseline for optimization work. Any later
  change to them gets its own `DECISIONS.md` entry.
- Phases G (8 local-search neighborhoods), H (4 shake mechanisms), I
  (VNS driver), J (instance generation / experiment runner) and K
  (validation against Paper 1) are **not implemented**. `config/`,
  `instances/`, `results/` and `scripts/` are empty, and the only
  executables are the 6 test programs.

**Current behaviour of `tdr::timing::evaluate_solution(solution,
network, customers, flp_data)`:**
- Start at time 0 [PAPER]; arrival = previous departure + travel time
  ((16), l_i reading).
- Launch point: each drone launches at t_i + SL; the truck leaves at
  t_i + n_i * SL ((18), (21)).
- Recovery: max(t_k, drone arrival) + SR per trip, concurrent with truck
  service (PAPER AMBIGUITY: SR -- PROFESSOR DECISION PENDING).
- Endurance: (36) and (37) literally.
- Physical drones: id range, (28) route-order form, and |U| enforced.
- Launch points: arc membership, (6), (8), (9) enforced.
- Customer service and objective unchanged.

**Phase F.** F3 builds L_(a,b) for every arc with n = floor(D/d) (d must
be supplied); F4 uses bridging-arc candidates, assigns drone ids, and
delegates every feasibility decision to the evaluator.

**Data-model fields.** `DroneTrip::drone_id` is the physical drone.
`DroneTrip::launch_position_in_route` / `recovery_position_in_route`
remain unused.

**Open items -- deliberately NOT decided:**
1. **PROFESSOR DECISION PENDING -- SR** (C2): readings (A) literal and
   (B) index slip; the current rule is neither. This also covers several
   recoveries at one node and Se^T/SR concurrency.
2. **UNRESOLVED -- main-experiment d** (C7). No default; Table 4 counts
   not targeted.
3. **PAPER CONTRADICTION -- launch locations** (C8): figures showing
   customer/depot launches vs the formal FLP-only model.
4. **PROFESSOR DECISION PENDING -- Algorithm 2 details**: the per-drone
   loop, recovery-node choice (currently the next truck node) and
   `BestRoute()` scoring. With the narrow recovery reading F4 never flies
   two drones at once (C5 consequence).
5. **IMPLEMENTATION ASSUMPTION -- launch-delay slack** (C1): earliest
   launch used; may be conservative for (36).
6. **Phase H**: Random_Drone_Remove can now use `drone_id`; its exact
   semantics are decided when Phase H is designed.

Items resolved by this pass: shared vs per-drone SL (now (18)/(21));
whether the evaluator enforces |U| (it does); what drone identity means
(u in U); theta scope (now (36)/(37)).

---

## Stage 1 (8 Nov 2026) -- evaluator input validation

**Scope.** `tdr::timing::evaluate_solution` now validates its inputs and
reports violations instead of accepting malformed solutions or throwing.
No timing, objective, endurance, SR, launch-scheduling, FLP, initializer
or VNS rule was changed. Files: `include/timing/timing_engine.hpp`,
`tests/test_timing.cpp` (tests S1-a ... S1-l, +41 assertions).

**Rules enforced (each produces a violation string; the call never throws).**
1. Every node id in the truck route exists in the network. Unknown ids
   return an infeasible result immediately (no timing is attempted).
2. One depot identity: the route starts and ends at the same depot node,
   and the depot appears nowhere in between (constraints (2)-(3)).
3. No customer is visited twice by the truck (constraint (10)).
4. Every network Customer node has a CustomerData entry.
5. Every CustomerData key is a Customer node of the network and its
   `node_id` equals its key. Entries keyed by an unknown node, a depot or
   an FLP are reported and excluded (their service times are not applied).
6. Each customer is served exactly once across truck and drone trips
   (pre-existing branches, now covered by tests).

**Known, deliberately unchanged.** The objective sums over the unordered
`customers` map, so floating-point summation order is not deterministic
across platforms. Not altered under the Stage 1 "no objective changes"
rule; flagged for a later decision.

---

## Stage 2 (8 Nov 2026) -- SR timing, Option B (decision D1)

**Status of the earlier open item.** "PROFESSOR DECISION PENDING -- SR"
(C2, open item 1 above) is now DECIDED by the user as Option B. The text
above that describes the previous rule (`recovery_complete = max(t_k,
drone arrival) + SR`, concurrent with truck service) is history.

**Implemented rule** (`tdr::timing::evaluate_solution`, forward pass):

    l_k = max( t_k + Se^T_k + r_k * SR ,  max_{u in R_k} (a_u + SR) )

- `t_k` truck arrival, `Se^T_k` truck service (0 at the depot), `r_k` the
  number of drones recovered at k, `a_u` drone u's own arrival at k.
- Recovery service is additive to truck service; each recovered drone
  costs SR of truck time; each drone also bounds l_k from below by its own
  arrival plus SR (constraint (20)).
- SR is never charged at a launch FLP: `l_i = t_i + n_i*SL`, every drone
  launches at `t_i + SL` (no staggering). Unchanged.
- With SR = 0 the new and old rules coincide, so every SR = 0 fixture is
  unaffected.

**Interpretation, not a paper value.** The paper prints (18) with its SR
sum over P_i^+ (routes launched at i). Option B reads this as an index
slip for P_k^- (routes recovered at k), consistent with the text on p15
and with (20). The literal alternative (Option A) was not adopted.

**Design point -- `DroneTripTiming::recovery_complete_time` is NOT changed.**
It remains `max(t_k, a_u) + SR`: the earliest time the drone is physically
free (it cannot be recovered before the truck is present). It is read by
`init/drone_fleet_capacity.hpp`, so changing it would alter initializer
behaviour, which Stage 2 forbids. Under Option B it is always <= l_k
(l_k >= t_k+SR and l_k >= a_u+SR), asserted in the tests. It is not the
truck-departure rule.

**Effect on existing expectations (all hand re-derived).**
- test_timing Block 9 sub-cases A and B: departure at the recovery
  customer 16 -> 21 and 17 -> 21 (t=10, Se1=6, SR=5, r=1: 10+6+5 = 21);
  sub-case C stays 27. Block 1 and the two-trip block are unchanged
  (the drone term already bound).
- test_init Block 11: route and trips unchanged, objective 820.1434959367223
  -> 828.143495936722. Both values reproduced by an independent Python
  evaluator (no shared code).
- Mutation check: the new Stage 2 tests were run against the Stage 1
  engine and the cases with differing old/new values fail there (17
  assertions); the ones where the rules agree pass.

**Known, not addressed here.** The objective sums over an unordered map
(floating-point summation order is not fixed) -- recorded as a Stage 3
reproducibility task.

## Static reproduction: decisions D1-D10 and the M1 infrastructure (9 Nov 2026)

Context. The dynamic extension is on hold; the immediate goal is to finish the **static** reproduction (Phases G-K). The user approved the
sequence M0-M10 and decisions D1-D10 (all as proposed, with the clarifications below). Every default is configurable and **[ASSUMPTION]**
unless stated otherwise. Mathematical statements are in `docs/static/math_spec.md`; the milestone log is `docs/static/MILESTONES.md`.

**A correction to an earlier statement.** The Phase G brief said the VNS interpretation was "already selected in DECISIONS.md". It was not: this file had
no VNS section (only `docs/paper1_decision_register.md` listed VNS parameters under "Do not change yet"). D7/D8 below are therefore the first
recorded VNS decisions.

| ID | Decision (approved) | Notes |
|---|---|---|
| D1 | Shared "adjacent" rule for N1-N8, window $w=3$ initially, configurable (`NeighbourhoodConfig::adjacency_window`) | Exact definition chosen in M1: skeleton half-steps, math_spec §7. The user wrote "3 route positions"; counting **skeleton (non-FLP) nodes** rather than raw positions is an implementation choice made so that the neighbourhood does not shrink when a solution uses many FLPs. A drone-served customer sits midway between the two truck nodes of its launch gap. Please tell me if you want raw positions instead. |
| D2 | N1 and N2 are each repeated twice per local-search pass | paper: "several times" |
| D3 | N3 = exchange between consecutive trips of the same drone | already decision I-8 |
| D4 | Re-selection in N5/N7: best valid FLP on the resulting route's arcs within the window; recovery at the next truck node | "best" = lowest objective via the evaluator |
| D5 | N8 searches all valid insertion positions | configurable |
| D6 | Shakes: uniform choice among the four. After a shake, **only** trips whose launch/recovery relationship became invalid are dissolved; unaffected trips are preserved; the number of dissolved trips and the reason are recorded; invalid trips are never silently repaired | implemented in M6 |
| D7 | VNS parameters ($T_0$, cooling, `iter_max`, no-improvement limit) configurable and calibrated in pilot runs; **not** tuned to match the paper's numbers | M7 |
| D8 | Clarified VNS: correct iteration counter, search from the current accepted solution, best-so-far kept separately, accepted improvements carried forward; differences from the printed pseudocode (iter never incremented; line 7 always searches from $x'$; no best-so-far) documented | M7 |
| D9 | Depot at the centre of the square, configurable (`DepotPlacement`) | paper silent |
| D10 | Seconds internally; reports show minutes alongside | |

### M1 decisions and assumptions

1. **RNG [ASSUMPTION].** xoshiro256** seeded through SplitMix64, with the integer/real/permutation transformations implemented in `util/rng.hpp`, because `<random>` distributions are
   implementation-defined and would break cross-platform reproducibility. The paper gives no generator or seed policy. Streams (Instance, InitialSolution, Vns, Shake) are derived from one
   base seed (default 20261109) by `derive_seed(base, stream, index)`.
2. **Instance distribution [ASSUMPTION].** i.i.d. uniform on the square; depot at the centre; customers 1..c, depot 0; draw order x then y per customer, so growing $c$ never reshuffles
   earlier customers. The paper's instances are unpublished, so digit-level reproduction is impossible. Service time 30 s for both vehicles, $\theta=1$, speeds, endurance, SL, SR and
   $\lvert U\rvert=2$ are the Sec 6.1 values **[PAPER]**.
3. **FLP spacing [ASSUMPTION].** `FlpConfig::unit_division_length_m = 2000`: the spacing used in the paper's Sec 6.3 sensitivity analyses; the main experiments state none. It is an explicit config value, not a
   generator default (the generator still refuses an unset $d$). It yields ~39,400 FLPs at 50 customers, about 11 x Table 4's 3,493; not tuned to match.
4. **Eager pool, no lazy cache [user instruction].** `flp/flp_arc_index.hpp` only indexes the existing pool (arc -> FLPs by position; FLP -> arc). The capacity-based FLP id scheme proposed for the dynamic work is not introduced.
5. **`assign_drone_ids_by_route_order` moved** to `search/drone_assignment.hpp` (`tdr::search`); body unchanged; `tdr::init::detail::assign_drone_ids_by_route_order` kept as a forwarding `using`. Behaviour proven unchanged (script check of the body,
   40,000-case differential, 20,000-case pinned checksum in the repo tests).
6. **Report, don't repair [Phase G rule].** `search/route_ops.hpp` never decides feasibility and never repairs silently. `flps_off_their_arc` reports the FLPs an edit has left on a foreign arc (the evaluator's p9 / (13)-(15) condition); `drop_unused_flps` is the only
   clean-up, is explicit, and returns what it removed. Fuzz tests assert that helper and evaluator agree.
7. **Definition of "unused FLP" [IMPLEMENTATION CHOICE].** `unused_flps_in_route` = FLPs that no trip names as its launch node. The evaluator's rule (6) also counts FLPs whose only trips are unplaced (e.g. recovery now before launch). The helper deliberately does not remove an
   FLP that a broken trip still names, so a broken trip is never hidden; the two coincide whenever every trip is placed (tested; the helper never reports more).
8. **Drone identity.** After a single-trip change, `lowest_free_drone_for_trip` picks an id without touching other trips; whole-solution reassignment (`assign_drone_ids_by_route_order`) is the fallback. Either only chooses an id; feasibility remains the evaluator's.
9. **Evaluator throughput (finding; resolved by the follow-up below).** ~96% of a 50-customer evaluation was a registry scan in the Stage 1 validation (`docs/static/performance_profile_M1.md`). P1 (cache the customer-id list) and P2 (ordered summation) were applied in the M1 performance follow-up, differential-tested against a frozen reference copy.
10. **Stale text.** The section "Pre-Phase-G baseline (updated 29 Sep 2026)" earlier in this file still lists the pre-Stage-2 recovery rule under "Current behaviour"; the Stage 2 section above is authoritative (Option B). History was not rewritten.
11. **Held-Karp vs CPLEX.** `TSPSolverFactory(max_exact_n = 21)` applies to the distance-matrix size (depot + customers): exact for 10 and 20 customers, heuristic for 50. Quantifying the effect was not done (listed as a limitation, `docs/static/experiment_methodology.md` §7).

### M1 performance follow-up: P1 and P2 (9 Nov 2026)

12. **P1 [DECISION].** `SpatialNetwork` caches the ascending customer-id list at construction (`customer_ids()`); the evaluator's Stage-1 "every network customer has CustomerData" check reads it. Safe because the registry is held by value, copied in, and has no mutator; no invalidation logic exists or is needed.
    Observable behaviour: identical (strict bitwise differential against the frozen reference).
13. **P2 [DECISION].** The evaluator resolves customers, and sums the objective, in ascending customer-id order instead of `unordered_map` iteration order. The objective's definition is unchanged. Consequences, all documented and tested: `customer_records` and customer-level violation messages are now in ascending-id order;
    `objective_value` can differ in the last bits (max 6.98e-10 s absolute, 9.6e-16 relative in 1,558 cases, within the rigorous bound 2 n eps sum|t|); every changed pairwise comparison (93 of 11,922) was a mathematical tie whose old outcome depended on hash-table insertion history.
    No tolerance is used anywhere in the evaluator or planned for the neighbourhoods. [Resolved at M7: strict `<` with threshold 0, A37, approved 14:34 IST.]
14. **Frozen reference [DECISION].** `tests/reference/timing_engine_reference.hpp` is the evaluator at `cc37e49` in namespace `tdr::timing_ref`; it is test-only and must not be edited. Later changes to `timing_engine.hpp` should be differential-tested against it (strict mode if behaviour must not change).

### M2: neighbourhoods N4 and N5 (9 Nov 2026)

15. **Affected trips: reject and report [DECISION, narrowest choice; SUPERSEDED for N4 by item 21 on 9 Nov 2026 --- `AffectedTripPolicy::Reject` remains selectable].** The paper and D1--D10 define neither re-anchoring nor dissolving for trips an N4 move (or the other trips of an N5 swap) disturbs. M2 therefore *rejects* such a candidate (`RejectReason::AffectedTrips`, trips named, never evaluated, never repaired).
    Measured cost: 87--99 % of N4 candidates on initial solutions (`docs/static/operators_N4_N5.md` §6). A re-anchor or dissolve policy needs your approval first.
16. **N4 = relocate and exchange [ASSUMPTION A1].** Paper text says relocate, its figure shows an exchange; both are searched in the D1 window (`N4Mode` selects one). Exchange partners are truck-served customers only.
17. **N5 launch window centre [ASSUMPTION A3].** The new trip's launch FLPs come from arcs whose gap is within the D1 window of the swapped truck customer's former skeleton position; recovery is the next truck node (D4); drone = lowest free id; other trips keep their ids.
18. **Dissolved trip's launch FLP [DECISION within "report, don't repair"].** Dropped from the route only if no other trip launches from it, and recorded in `flps_dropped`.
19. **Ties [ASSUMPTION A2/A4].** Strict `<` on the raw objective; the first generated candidate wins. No tolerance (see items 13 and the open M7 question).
20. **Classification of evaluator verdicts [IMPLEMENTATION].** Endurance from the structured per-trip fields, then fleet capacity (`max_simultaneous_drones > |U|`), then message text for the rest. `EvalTripOrder` is checked before rule (6) because it is the root cause of that cascade.

### M2 follow-up: N4 affected-trip policy and sign-off (9 Nov 2026, instruction 06:17 IST)

21. **N4 affected trips: conservative re-anchoring [DECISION approved 9 Nov 2026; implementation ASSUMPTION A6].** The paper is silent on trips an N4 move disturbs (it defines re-anchoring only for N5/N7's new trip, D4). Default policy `AffectedTripPolicy::Reanchor`: only the trips whose launch FLP the route change leaves on a vanished arc are re-anchored, onto the FLPs of the arc the stale FLP now sits on (radius 0, ratio order); the best feasible joint choice is kept (evaluator-judged, strict `<`, first wins ties --- the N5 convention);
    drone id, drone customer and recovery node never change; unaffected trips and the input are untouched; trips are never dissolved. No FLP on the arc $\Rightarrow$ `NoReplacementFlp`; the joint combinations are enumerated under a search budget `max_reanchor_variants` (**revised by item 28: default 100, budget-bounded, no cap rejection**; the first follow-up version had default 10,000 and rejected over-cap candidates as `TooManyReanchorings`, which no longer exists). Full semantics: `docs/static/operators_N4_N5.md` §6.
22. **Recovery relationships are not repaired [DECISION, as instructed].** A route change can place a trip's recovery node before its launch FLP (verified in the representation). Replacing the launch cannot fix that, so the candidate is rejected as `InvalidRecovery` (trips named), not evaluated and shown unrepaired. Checked for every trip, before any re-anchoring.
23. **Diagnostics [IMPLEMENTATION].** `OperatorResult` gains `stale_launch_candidates`, `reanchored_candidates`, `trips_reanchored`, `variants_generated`; reasons `NoReplacementFlp`, `InvalidRecovery` (`TooManyReanchorings` was added here and removed again by item 28); item 30 adds the cap diagnostics. Candidates and rejections are counted once (`accepted + rejected_total() == generated`); `evaluated` counts evaluator calls (variants). Plain integer counters on the existing result, no extra work in the hot path.
24. **N4 relocate and exchange both kept [DECISION / ASSUMPTION A1].** The paper's text describes relocation, its example (Fig. 10d) is an exchange; supporting both (`N4Mode`) is an implementation interpretation, not a claim that the paper specifies two independent neighbourhoods. Generation order and tie-breaking unchanged.
25. **N5 launch-window centre [DECISION / ASSUMPTION A3].** Approved: the window is centred on the swapped truck customer's former position (D4 does not name the centre). N5 behaviour is otherwise unchanged (`neighbourhood_n5.hpp` byte-identical).
26. **Objective comparison [DECISION].** Strict `<` on the raw objective in N4 and N5 (including the choice among re-anchoring variants); no epsilon; first-generated candidate wins ties. A configurable improvement threshold may be considered during M7 VNS integration; it is **not** authorised now.
27. **Measured trade-off, reported not resolved [FINDING].** Re-anchoring raises the feasible share of N4 candidates on initial solutions from 13.3 / 8.2 / 0.8 % to 17.8 / 24.5 / 2.0 % (n = 10 / 20 / 50) but multiplies the N4 sweep cost (0.78 ms $\to$ 362 ms at n = 50) because of the joint enumeration; 69 % of the stale candidates at n = 50 also have an invalid recovery. `max_reanchor_variants = 100` kept the same accepted candidates at ~12 x lower cost in the measured bases (under the then reject-over-cap semantics; see item 31 for the figure under the final budget-bounded semantics). **Resolved by items 28--32** for the cap; a per-trip independent selection or a recovery rule remain unimplemented and need your decision.

### M2 finalization: bounded N4 re-anchoring (9 Nov 2026, instruction 08:06 IST)

28. **Default joint-combination budget 100, budget-bounded search [DECISION approved 9 Nov 2026; IMPLEMENTATION ASSUMPTION A6b].** `N4Options::max_reanchor_variants` defaults to **100** (configurable; 0 = unlimited). Combinations are enumerated deterministically (stale trips in index order, last fastest, options in ratio order); generation stops once the budget is spent; the combinations reached are evaluated; the lowest-objective feasible one is kept (strict `<`, first generated wins ties). A candidate is **never rejected because of the cap** (`RejectReason::TooManyReanchorings` removed, 12 reasons remain). Drone id, drone customer, recovery node, unaffected trips and the input are preserved, trips are never dissolved, recovery nodes are never changed; invalid recovery, missing replacement FLP and evaluator failures are rejected. The Reject policy is kept and selectable.
29. **Paper fact / assumption / limitation [RECORD].** PAPER: the paper specifies neither this N4 re-anchoring rule nor a combination cap. DECISION: conservative re-anchoring of affected launch FLPs is permitted. ASSUMPTION: the default budget is 100, an empirical implementation choice approved on the benchmark, not a paper parameter. LIMITATION: with a finite budget the search is budget-bounded and potentially incomplete, so it may miss the best candidate exhaustive enumeration would find; it must never be called exhaustive when the budget is reached (`cap_reached`). M2 is **not** an exact reproduction of every paper detail.
30. **Cap diagnostics [IMPLEMENTATION].** `CandidateRecord::combinations_total` (saturating product) and `cap_reached` (`combinations_total > budget`; a budget equal to the total is not reached); `OperatorResult::cap_reached_candidates` (0 means the sweep was exhaustive). `cap_reached` is a flag, not an outcome: each candidate still gets exactly one outcome (`accepted + rejected_total() == generated`); `variants_generated` counts combinations generated, `evaluated` counts evaluator calls.
31. **Measured effect of the budget, with a correction to the approval premise [FINDING].** The approval cited "same accepted candidates at ~12 x lower runtime"; that figure was measured with the earlier reject-over-cap semantics. Under the approved budget-bounded semantics (over-budget candidates are evaluated on their first 100 combinations) the n = 50 N4 sweep is 67.8 ms versus 302 ms at budget 10,000, about **4.5 x** (n = 20: 7.9 versus 12.2 ms; n = 10: 2.5 versus 4.8 ms), with identical accepted candidates and identical best per anchor on the benchmark fixtures (7/7, 14/14, 27/27 anchors). Budget 100 is the smallest tested value that keeps every acceptance (budget 30 loses one at n = 50). Cap reached in 5.1 / 7.9 / 14.3 % of stale candidates at n = 10 / 20 / 50. Implemented as instructed; a still cheaper budget or an independent per-trip choice is **your decision**.
32. **Not asserted [DECISION on wording].** No test or document asserts that budget 100 always equals the exhaustive search; the 100-versus-10,000 comparison is asserted only on the reported benchmark fixtures and labelled as an observation there.

### M3: neighbourhoods N8 and N7 (9 Nov 2026, instruction 08:45 IST)

Contracts, tests and measurements: `docs/static/operators_N7_N8.md`; evidence: `docs/static/test_and_validation.md` section 9.

33. **Numbering and transformation [PAPER, interpreted].** The paper's neighbourhoods 7 and 8 (Sec 5.3, p22--23, Figs. 12a/12b) are *truck node delete--drone node insert* (N7) and *drone node delete--truck node insert* (N8). N7: one truck-served customer $t$ leaves the truck route and becomes a drone trip $(f, t, y)$; N8: one drone trip is dissolved and its customer is inserted into the truck route. Figs. 12a/12b are known only through their captions in the paper extraction; the original PDF is not in the repository or the project files.
34. **N7 launch and recovery [DECISION D4, applied].** New trips are generated for every FLP of every resulting-skeleton arc inside the D1 window (in gap, then ratio order); recovery = the next truck node after the FLP (the arc end, the depot included). An FLP already on the route is reused (second launch); otherwise it is inserted in ratio order. Drone id = the lowest free id over the trip's route interval (the N5 rule); none $\Rightarrow$ `NoFreeDrone`. Other trips keep their ids.
35. **N7 window centre [IMPLEMENTATION ASSUMPTION A7].** The node $t$ no longer exists in the resulting skeleton, so the D1 window is measured on the *original* skeleton (merged arc at coordinate $2s_t$; eligible iff within $2w$): 5 gaps at $w=3$. Same family as A3 (N5). Affects only the pre-filter; open for confirmation.
36. **Disturbed trips: reject and name, never repair [DECISION, item 15 extended to N7/N8].** A move that leaves another trip's launch FLP off its arc, or (N7) deletes a trip's recovery node, is one `AffectedTrips` candidate (trips named, not evaluated). No re-anchoring, dissolution or recovery change. N4's Reanchor policy stays N4-only; whether to extend it is an open question (finding in `operators_N7_N8.md` section 9).
37. **N7 with no eligible FLP [IMPLEMENTATION].** One `NoReplacementFlp` candidate (counted, visible). N5 silently generates nothing; N5 is unchanged (byte-identical).
38. **N8 insertion positions [DECISION D5, applied].** Every gap of the truck skeleton, depot-adjacent gaps included, *not* limited to the D1 window; customer inserted directly before the gap's end node (after the gap's FLPs). `N8Options::insertion_window` (default off) optionally restricts to the D1 window. Valid positions are exactly the gaps without remaining launch FLPs of other trips. "Optimal order of truck visits" is read as the best position of the one inserted customer (the other truck nodes are not reordered; that would be N4).
39. **N8 dissolution [DECISION D6 respected].** Only the anchor trip is dissolved, explicitly; its FLP leaves the route iff no other trip launches from it (`flps_dropped`, rule (6)). No shake behaviour.
40. **Randomised entry points [IMPLEMENTATION].** N7 draws once uniformly over the interior truck customers (route order); N8 once uniformly over the trip indices. Deterministic operators take the anchor explicitly.
41. **Objective comparison [DECISION, unchanged].** Strict `<`, no epsilon, first-generated candidate wins ties; enumeration is deterministic (N7: gaps ascending, FLPs by ratio; N8: gaps ascending).
42. **Infrastructure change [IMPLEMENTATION].** `move_types.hpp` gained `OperatorId::{N7,N8}` and `MoveKind::{N7TruckToDrone,N8DroneToTruck}` (additive; `kRejectReasonCount` still 12). N7's new-trip construction duplicates a few lines of N5's inline logic deliberately, to keep `neighbourhood_n5.hpp` byte-identical.
43. **Findings [FINDING, nothing changed].** (i) On paper-scale instances N7 candidates are almost always endurance-infeasible (0 of 384 evaluated on the benchmark bases; 0.9 % of 128,212 in the oracle campaign). (ii) Without re-anchoring, disturbed trips dominate: N7 25 of 88 candidates at $n=50$ (27 of 27 with tripled endurance), N8 valid positions 75 % $\to$ 21 % from $n=10$ to $n=50$. (iii) At $n=50$ about 82 % of an N7 generation sweep is the per-call base evaluation. See `operators_N7_N8.md` section 9; none of it is acted on.
44. **Stale M2 wording corrected [DOC].** The M2 summaries in `docs/static/MILESTONES.md` that still said "10,000" / `TooManyReanchorings` were already marked superseded; the M2 follow-up and finalization sections now say so at the point of use and record M2 as signed off.

### M4: neighbourhoods N3 and N6 (9 Nov 2026, instruction 09:42 IST)

Contracts, tests and measurements: `docs/static/operators_N3_N6.md`; evidence: `docs/static/test_and_validation.md` section 10.

45. **Transformation [PAPER, interpreted].** N3 (drone service node relocate, Sec 5.3 p21, Fig. 10c) and N6 (D-D service node swap, p22, Fig. 11b) both reduce, for trips that serve exactly one customer, to *exchanging the served customers of two trips*; the truck route, every launch FLP, every recovery node, every drone id and the trip order stay as they are. Figs. 10c/11b are known only through their captions and the register; the paper PDF is not in the repository.
46. **N3 partners [DECISION D3, applied; A11].** The partner is a trip of the **same drone**, consecutive with the anchor in that drone's sequence (ordered by launch route position, then recovery position, then index); both neighbours are searched, predecessor first. A lone trip yields zero candidates with status `Ok`. The D1 window is not used by N3.
47. **N6 partners [IMPLEMENTATION ASSUMPTIONS A8, A10].** The partners are the drone customers of *other trips* whose launch-gap half-step is within $2w$ of the anchor's ($w=3$, D1; same `PositionIndex::neighbours` as N5, nearest first, ties by customer id), restricted by default to trips of a **different drone** (A8: the paper says "other drone trips"; Fig. 11b shows a drone-1 and a drone-2 trip). `N6Options::include_same_drone = true` admits every other trip. Open for confirmation. **[SUPERSEDED for A8 on 9 Nov 2026 12:30 IST: the default is now *any other trip*, same-drone trips included (`N6Options::include_same_drone = true`); `false` restricts partners to different drones. See item 54. A10 approved.]**
48. **What moves in N6 [IMPLEMENTATION ASSUMPTION A9].** Only the two customers; launch FLPs, recovery nodes and drone ids stay with their trips (as Fig. 10c shows for N3; Fig. 11b's trip compositions are unavailable). Re-selecting launch/recovery for the new customer is not done; D4 is *not applicable* to either operator because no FLP is selected.
49. **No disturbed trips; no repair [DECISION, item 15 respected].** Nothing is disturbed, so no re-anchoring, dissolution or repair rule is needed or introduced (re-anchoring stays N4-only). Feasibility is the evaluator's verdict alone (reachable reasons: endurance and `EvalOther`; fleet capacity, sequencing, `AffectedTrips`, `NoReplacementFlp`, `InvalidRecovery`, `NoFreeDrone`, FLP-arc reasons are unreachable by construction, argued in the spec, and were observed 0 times).
50. **Objective comparison and counting [DECISION, unchanged].** Strict `<` on the raw objective, first-generated candidate wins ties, no epsilon; `accepted + rejected_total() == generated`; `duplicates_skipped` is always 0; `evaluate_candidates=false` accepts nothing. Enumeration is deterministic.
51. **Randomised entry points [IMPLEMENTATION, A12].** Both `Rng` overloads draw once with `uniform_index(#trips)` ("randomly selecting a drone trip"), then run the deterministic call; no trips gives `NoEligibleAnchor`.
52. **Infrastructure change [IMPLEMENTATION].** `move_types.hpp` gained `OperatorId::{N3,N6}` (the enum is now `N3,N4,N5,N6,N7,N8`; it is only compared, never cast, so the renumbering is harmless), `MoveKind::{N3Exchange,N6Swap}` and `Move::partner_trip` (additive; `kRejectReasonCount` still 12). The shared bookkeeping of the two operators is in `include/search/trip_exchange.hpp`. N4/N5/N7/N8, the route helpers, the evaluator and the frozen reference are byte-identical.
53. **Findings [FINDING, nothing changed].** (i) N6 with the default A8 has **zero candidates** on every paper-scale initial solution (all trips fly with drone 0), so N6 contributes only once a second drone is in use; (ii) N3 candidates are mostly endurance-infeasible at paper scale (feasible 0 / 0 / 9.1 % at $n=10/20/50$); (iii) cost per candidate is the evaluator's own cost, generation is negligible; (iv) the oracle (595,093 checks) found no discrepancy against the frozen reference. Open for your decision: A8, A9, A10, A11 (section 9 of the spec). **[Finding (i) superseded by item 54: with the revised A8 default N6 has candidates on the paper-scale initial solutions (2 / 8 / 108 at n = 10 / 20 / 50); the restricted variant still has none.]**

### M4 sign-off and the A8 revision (9 Nov 2026, instruction 12:30 IST)

54. **M4 signed off; A8 revised [USER DECISION].** The user approved M4 and decided: **A8** N6's default partner set is *any other trip*, trips of the same drone included; the restriction to different drones is kept as an option (`N6Options::include_same_drone`, now defaulting to `true`; `false` = different drones only; the option name was kept, only the default changed). N3 and N6 candidates may coincide (the same exchange of two same-drone trips): accepted. **A9** (only the customers move; launch FLPs, recovery nodes and drone ids stay with their trips), **A10** (D1 window on launch-gap half-steps) and **A11** (N3 searches predecessor, then successor) are approved as implemented. Only the default and the tests, oracle, mutation record, benchmarks and docs that pinned it were changed; N3, the shared exchange step and the M1--M3 code are untouched. Effect: N6 2 / 8 / 108 candidates (0 / 0 / 4 feasible; 3.70 % at $n=50$) on the paper-scale initial solutions at $n=10/20/50$, the restricted variant 0. Tests after the change: `test_n3_n6` 587, `test_n3_n6_oracle` 613,135 checks, 0 failures; N6-13 now flips `true` to `false` and is killed (13 of 14 N6 mutants killed, N6-07 equivalent).

### M5: neighbourhoods N1 and N2 (9 Nov 2026, instruction 12:30 IST)

Contracts, tests and measurements: `docs/static/operators_N1_N2.md`; evidence: `docs/static/test_and_validation.md` section 11.

55. **Transformation [PAPER, interpreted].** N1 (launch node relocate, Sec 5.3 p20--21, Fig. 10a) repositions the launch node of one drone trip, "searching adjacent customer nodes and flexible launch nodes" of its launch node; N2 (recover node relocate, p21, Fig. 10b) repositions the recovery node, "searching adjacent customer nodes" of its recovery node. Each changes exactly one node of exactly one trip; the best candidate wins. The paper does not define "adjacent", the old FLP's fate, the drone identity, or the order of launch and recovery. Their repetition ("several times") is VNS integration (D2, M7), not implemented here.
56. **N1 candidates [IMPLEMENTATION ASSUMPTION A13].** Every FLP on the arcs of the D1 window ($|g-g_0|\le w$, half-step coordinate $2g+1$) around the trip's launch gap, ascending gaps, ratio order within an arc, the current launch excluded; includes FLPs not yet on the route (inserted in ratio order) and FLPs other trips already launch from (a shared launch).
57. **N2 candidates [IMPLEMENTATION ASSUMPTION A14].** The skeleton nodes (customers and the end depot) with $|s-s_k|\le w$, $s\ge1$, $s\ne s_k$, ascending; the start depot is never a candidate.
58. **Launch/recovery order [IMPLEMENTATION ASSUMPTION A15; item 15 respected].** A candidate whose recovery is not strictly after its launch on the route is generated, counted and rejected as `InvalidRecovery` (the trip is named, it is not evaluated, nothing is repaired; the other node of the trip is never changed). Same reason and convention as N4.
59. **Drone identity and feasibility [IMPLEMENTATION ASSUMPTION A16; DECISION unchanged].** The drone id never changes; endurance (36)/(37), SL/SR timing, (28) drone sequencing and fleet capacity are the evaluator's verdict (`EvalEndurance`, `EvalDroneSequencing`, `EvalFleetCapacity`, ...), classified and never overridden or repaired.
60. **The old launch FLP [IMPLEMENTATION ASSUMPTION A17; DECISION D6 convention].** Dropped from the route iff no other trip launches from it, explicitly and counted (`flps_dropped`); kept if shared. No idle FLP can result (rule (6)).
61. **FLP-only launches [DECISION, register Issue 7, respected; deviation identified].** The paper's "adjacent customer nodes" as launch candidates contradicts Assumption 4 and the model (launch at flexible launch points only); not implemented. Fig. 10a's customer launch is read as an FLP next to that customer.
62. **Randomised entry points and empty windows [IMPLEMENTATION A18].** Both `Rng` overloads draw once with `uniform_index(#trips)`; no trips gives `NoEligibleAnchor`. A window with no candidate gives status `Ok` with zero candidates (N1/N2 are not forced to move, unlike N7).
63. **Objective comparison and counting [DECISION, unchanged].** Strict `<`, first generated wins ties, no epsilon; `accepted + rejected_total() == generated`; `evaluated + InvalidRecovery == generated` (when evaluating); `duplicates_skipped == 0`; `evaluate_candidates=false` accepts nothing but still counts `InvalidRecovery`.
64. **Infrastructure change [IMPLEMENTATION].** `move_types.hpp` gained `OperatorId::{N1,N2}` (the enum is now `N1..N8`; compared, never cast), `MoveKind::{N1LaunchRelocate,N2RecoveryRelocate}` and `Move::replaced_node`; `kRejectReasonCount` is still 12. New: `trip_relocate.hpp`, `neighbourhood_n1.hpp`, `neighbourhood_n2.hpp`. Evaluator, frozen reference, `route_ops.hpp`, `candidate_window.hpp` and N3--N8 are byte-identical (sha256) apart from the approved A8 default.
65. **Findings [FINDING, nothing changed].** (i) At paper scale 80--86 % of evaluated N1 candidates are endurance-infeasible and N2 has at most one feasible candidate per sweep; (ii) about half of N1's generated candidates are `InvalidRecovery` (the symmetric window includes FLPs behind the recovery); (iii) with all trips on drone 0 the evaluator rejects some candidates on (28) at $n=50$ (19 for N1, 10 for N2). The open points are the A13--A18 confirmations, the N1 FLP-only deviation and the missing paper PDF.
66. **M5 signed off [USER DECISION, 9 Nov 2026, 13:19 IST].** A13--A18 approved as implemented (items 56--62). **Reproduction limitation, kept documented:** N1 does not use customer launch nodes (the paper text says "adjacent customer nodes and flexible launch nodes"; the model and register Issue 7 allow launches at FLPs only). It is not to be silently expanded without further paper evidence.

67. **M6 shaking operators [IMPLEMENTATION; signed off, see item 71].** Random_Reverse / Random_Exchange / Random_Shift / Random_Drone_Remove in `include/search/shakes.hpp`, contracts in `docs/static/shakes.md`. D6 preserved: uniform choice among the four; only invalid trips are dissolved (`LaunchArcVanished`, `RecoveryNotAfterLaunch`, and in the no-free-gap fallback `ArcSplitByReinsertion`), counted with reasons, never repaired; the result is evaluator-judged and returned unaltered. Not applicable shakes return `NotApplicable` (no redraw).
68. **Route shakes act on the truck skeleton; FLPs follow their arcs [IMPLEMENTATION ASSUMPTIONS A19, A24].** Segments are runs of interior truck customers, depots fixed. A surviving directed arc keeps its FLPs and trips; a vanished arc's FLPs leave the route and its trips are dissolved. No re-anchoring (item 15 respected).
69. **Parameters and draw orders [IMPLEMENTATION ASSUMPTIONS A20--A23, A27, A28].** Reverse: unordered pair of interior nodes; Exchange: $L\in[1,\min(L_{\max},\lfloor m/2\rfloor)]$ and a uniform non-overlapping pair (adjacent allowed); Shift: $L$, start, direction, distance in $[1,\min(D_{\max},\text{room})]$; Drone_Remove: uniform among drone ids with a trip. Defaults $L_{\max}=D_{\max}=3$, configurable (D7; calibration is M7). The draw orders are part of the contract.
70. **Returned customers and infeasible outputs [IMPLEMENTATION ASSUMPTIONS A25, A26, A29].** Returned customers go to a uniformly drawn free gap, else a drawn gap whose trips are dissolved (`ArcSplitByReinsertion`). Endurance, SL/SR timing, (28) and fleet capacity are not dissolution reasons: the (possibly infeasible) result is returned with the evaluator's reason. The input must be evaluator-feasible. **A19--A29 approved at the M6 sign-off (item 71).**

71. **M6 signed off [USER DECISION, 9 Nov 2026, 13:56 IST].** A19--A25 and A27--A29 approved as implemented; A26 approved with the clarification that only trips whose launch FLPs the reinsertion genuinely invalidates are dissolved, the affected trips and the exact reason recorded, and unrelated trips never dissolved merely because they occupy the selected gap (already the behaviour: `ArcSplitByReinsertion` applies exactly to trips launching from the FLPs of the split arc).
72. **M7 VNS driver [IMPLEMENTATION; signed off, see item 77].** `include/search/vns.hpp`, contract `docs/static/vns.md`. D7/D8 preserved: configurable parameters (`VnsOptions`), pilot-based defaults, correct iteration counter, search from the accepted solution, separate best-so-far, accepted improvements carried forward. Deviations from the printed Algorithm 1 are listed in `vns.md` section 4 (all D8: counter, search point after acceptance, best-so-far, shake redraw, extra termination criteria).
73. **Acceptance [IMPLEMENTATION ASSUMPTIONS A30, A37].** The printed comparison against $f(x)$ (current accepted solution) with the shaken solution as the initial search point; the SA test $u\le\exp((f(x)-f(z))/T)$ with one draw per non-improving candidate, ties accepted; strict `<` for improvement (threshold $\delta=0$, option kept). `AcceptanceReference::ShakenState` is selectable; the pilot favours the printed reference (clearly at n = 20, 50).
74. **Shakes in the VNS [IMPLEMENTATION ASSUMPTION A35].** `NotApplicable` and evaluator-infeasible shake results are rejected (never repaired) and redrawn from the same $x$, up to `max_shake_attempts = 10`; if none is feasible the pass is skipped (counted, cooled). The distribution is uniform among the shakes that act and give a feasible result.
75. **Schedule, list and seeds [IMPLEMENTATION ASSUMPTIONS A31--A34, A36, A38].** Geometric cooling with floor, $T_0$ as a fraction of $f(x_{init})$; list N4, N5, N8, N3, N6, N7, then N1/N2 alternately (D2); an operator call without feasible candidate is a non-move; termination `iter_max` / optional no-improvement / optional wall clock; guard `max_local_steps = 1000`; streams Vns and Shake by run index (reproducible; only `max_seconds` is not).
76. **Pilot defaults [D7].** `iter_max = 2000`, $T_0=0.01 f(x_{init})$, $\alpha=0.995$, from one instance per size (n = 10, 20, 50); labelled ours, not tuned to the paper's numbers. (Approved at the M7 sign-off, item 77.)
77. **M7 signed off [USER DECISION, 9 Nov 2026, 14:34 IST].** A30--A37 approved; A38 approved on condition that a guard trigger is reported (per-pass outcome `Completed`/`Skipped`/`GuardTruncated`, `passes_truncated`, `passes_completed`, `local_steps_total`, `guard_triggered()`), never silent; implemented, tested and documented (`vns.md` section 10). The default improvement threshold remains 0 and strict `<` is preserved.
78. **M8 runner, writer, configuration and CLI [IMPLEMENTATION; signed off, see item 83].** `docs/static/runner_cli.md`. Separate headers (configuration, runner, writer, CLI); a single field table drives parsing, overrides, the canonical dump and `list-keys`; algorithm defaults are not duplicated (members' own initialisers). VNS and operator behaviour preserved (runs bit-identical to direct `run_vns`); no default changed; deterministic seeding preserved.
79. **Configuration format [ASSUMPTION A39].** Flat INI-like text, strict parser (unknown / duplicate keys, malformed values reported with line numbers), precedence defaults < `--config` < `--set`; the resolved configuration is written to every result directory and reproduces the experiment.
80. **Experiment protocol and F1/F2 constants [ASSUMPTIONS A40, A41, A43].** `num_datasets` x `runs`, run indices from `first_run_index`, every run from the dataset's initial solution (defaults 1 x 10). F1/F2 constants (TSP exact limit 21, restarts 8, heuristic seed 42, KMeansPolicy) became configuration keys with their former values as defaults, so initial solutions are unchanged; the TSP seed is not derived from the base seed (as before). Config-level shake parameters must be >= 1.
81. **Outputs [ASSUMPTIONS A42, A44].** `results.json`, `runs.csv`, `traces.csv`, `timing.json`, `manifest.json`, `config.resolved.cfg`; only `timing.json` is non-deterministic; FNV-1a digests; atomic write; overwrite protection; every run's best solution re-verified (exactly-once + evaluator), failure -> exit code 1. Guard reporting (A38 closure) is part of the per-run output.
82. **(Superseded by item 83.)** M8 not signed off; M9 not started; open: approval of A39--A44.
83. **M8 signed off [USER DECISION, 9 Nov 2026, 15:35 IST].** A39--A44 approved. Closure: CLI exit-code-1 integration test (runner preflight for non-finite travel times; the Stage-1 heuristic-TSP crash on infinite travel times is reported, not changed); Windows validated by cross-compiled binaries under Wine (native Windows not available here; PowerShell commands and limitations in `windows_validation.md`); output schema revised to `/2` and documented (`output_schema.md`).
84. **M9 protocol [ASSUMPTION A46].** 12 instances per size (4 datasets x 3 independent base seeds: 20261109, 31415926, 271828183), 10 runs each, pilot VNS defaults (D7, not tuned to the paper); statistics over instances (mean, 95 % t-interval, range, seed-set spread). Configurations `config/m9/*.cfg`.
85. **Comparison unit [ASSUMPTION A45].** The paper's `z` is compared in minutes (its magnitude matches minutes of summed arrival times). Absolute agreement is NOT achieved (ours 33..47 % of the paper's mean z_avg) and is not claimed; depot position and endurance were ruled out as the cause by labelled sensitivity runs; the cause is open (question Q1 to the user: objective definition/unit, instance geometry, speed convention).
86. **Reference point instead of the MILP [ASSUMPTION A47].** Exact truck-only optimum by dynamic programming for the 10-customer instances; an upper bound of the true optimum, not a MILP replacement.
87. **No algorithmic change in M9.** Observations reported, proposals P1 (n >= 50 iteration budget), P2 (diversification for stalled runs), P3 (heuristic-TSP guard) await approval. Low activity of N1/N2/N3/N5/N6/N8 is a consequence of drone-free search states; N7's low feasibility is endurance/geometry driven (diagnostic `bench/m9_feasibility_probe.cpp`); neither endurance nor geometry was altered.
88. **(Superseded by item 89.)** M9 not signed off; open: approval of A45--A47.
89. **M9 signed off [USER DECISION, 9 Nov 2026, 16:13 IST].** A45--A47 approved with qualifications: A45 minutes only as a provisional comparison unit (the paper does not confirm its unit); A46 12 instances per size, intervals described as intervals over generated instances (a seed set changes instances and search streams); A47 the exact truck-only optimum strictly as an independent reference for the truck-only sub-problem, not a substitute for the MILP optimum. M9 is a validation and comparison milestone, **not** a numerical reproduction; the objective discrepancy (ours 0.33--0.47 of the paper's mean $z_{avg}$), the FLP-count discrepancy (our pools 1,722 / 6,266 / 37,606 vs the paper's 214 / 837 / 3,493) and the unavailable paper data stay explicit findings. P1--P3 remain unapproved proposals; algorithm, evaluator, endurance, geometry and experimental defaults unchanged.
90. **M10 documentation freeze [DOC, 9 Nov 2026].** No code change. Added `docs/static/INDEX.md` (milestone status, specification coverage), `docs/static/configuration_reference.md` (48 keys with provenance), `docs/static/static_reproduction_summary.md` (thesis summary), `scripts/m10_doc_check.py` (consistency checks). Stale statements corrected (list in `MILESTONES.md`, section M10): `experiment_methodology.md` seed formula and planned/implemented statuses, README status and layout, "awaiting review"/"not signed off" statements in shakes, VNS, runner, register and decision items, test counts of the M8 section.
91. **Unresolved questions carried out of the static work (not decided).** (a) the cause of the objective-scale discrepancy (objective definition or unit, instance geometry, speed or SR-timing convention; SR Option B was decided on 8 Nov 2026 and is not revisited); (b) the paper's unit of $z$; (c) main-experiment FLP spacing and rule; (d) launch locations (customer/depot launches in figures vs the FLP-only formal model; N1 keeps FLP-only launches); (e) Algorithm 2 details; (f) launch-delay slack in (36). Unapproved proposals: P1 (iteration budget for n >= 50), P2 (diversification for stalled runs), P3 (heuristic-TSP guard for non-finite travel times).
