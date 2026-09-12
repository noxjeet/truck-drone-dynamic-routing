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

`include/init/` (header-only, namespace `tdr::init`) implements Paper 1's
4-step initial-solution procedure (`paper1-understanding.md` Sec 5,
`paper1-implementation-plan.md` Sec 10-11): (a) the optimal truck-only
route, (b) K-means customer clustering, (c) flexible-launch-point
insertion, (d) feasible drone-trip construction. Three files: `truck_only_route.hpp`
(F1), `customer_clustering.hpp` (F2), `initial_solution_builder.hpp` (F3
+ F4 + top-level orchestration, `build_initial_solution()`). This phase
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

**[INFERENCE/ASSUMPTION] Each customer's launch candidate is looked up
directly** as the FLP generated on the arc that entered it in `r0` (the
generated point whose `data.arc_to == customer_id`) -- an O(1) lookup,
never a route search. This is the narrowest defensible reading of
Algorithm 2's OCR-corrupted "adjacent nodes" wording (see F4 below for
the parallel reasoning on the recovery side). **When
`points_per_arc > 1`,** several FLPs share the same arc (hence the same
`arc_to`); because Phase D emits them in increasing ratio order per arc
and this lookup keeps the LAST one written, the launch candidate that
results is deliberately the FLP CLOSEST to the customer (highest ratio)
-- a defensible, deliberate choice (it minimizes the drone's necessary
outbound leg among the same-arc candidates), called out explicitly here
rather than left as an unremarked side effect of iteration order.

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

**[ASSUMPTION] `available_drones` resets to `|U|` at the start of EACH
cluster's processing**, rather than being a single budget shared across
the whole construction. This is the narrowest reading of Algorithm 2's
own "while unassigned drones remain" loop, which is nested inside the
per-cluster outer loop with no stated cross-cluster drone bookkeeping.
**Acknowledged limitation:** this caps the *count* of trips built per
cluster at `|U|`; it is NOT a true concurrent-flight-count check across
the whole route's timeline (i.e. it does not verify that at every real
instant, no more than `|U|` drones are simultaneously airborne across
ALL clusters combined). Modeling genuine cross-cluster concurrent-flight
scheduling is a materially harder problem the source's own recoverable
Algorithm 2 pseudocode gives no basis for solving here, and building it
would violate the explicit instruction not to invent a more
sophisticated algorithm than the source supports. Every trip actually
constructed is still fully temporally and endurance feasible (verified
by Phase E on every candidate) -- what is not verified is a global
simultaneous-flight-count bound across cluster boundaries.

**[INFERENCE/ASSUMPTION] Recovery candidate selection** mirrors the
launch side's "narrowest adjacent reading": for a candidate customer `j`
at its current route position, the recovery candidate is the nearest
FOLLOWING node of type Customer or Depot still on the working route
(skipping over any already-inserted FLP entries). Exactly one candidate
trip `(i, j, k)` is built per customer per round -- no search over
multiple launch/recovery combinations -- avoiding an invented, paper-
unsupported combinatorial search.

**[ASSUMPTION] `BestRoute()` scoring: the candidate with the LOWEST
resulting drone customer-arrival time wins**, ties broken by lowest
customer `NodeId`. The paper names a `BestRoute()` selection step but
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
shared infrastructure; Phase E already supports and tests shared-SR
simultaneous recovery) -- it only prevents converting an in-use recovery
target itself.

**Every feasibility decision is made by Phase E, not reimplemented
here.** For each candidate, F4 builds a trial `Solution` (working route
with the candidate customer's slot replaced by its launch FLP, trip
appended to the already-accepted trips) and calls
`tdr::timing::evaluate_solution()`. A candidate is kept only if: the new
trip's own `DroneTripTiming::feasible` is true, the candidate customer's
own `CustomerServiceRecord` shows `served_by_drone && feasible`, AND the
whole trial solution's `feasible` is true. This directly satisfies every
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

**Tests.** `tests/test_init.cpp` (6 blocks, 64 assertions) covers all 10
required categories. Block 1 checks `compute_cluster_count()` against
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
value. All prior Phase A-E tests (29 + 43 + 26 + 35 + 99 = 232
assertions, the last figure reflecting the Phase E audit's added Block 9)
continue to pass unchanged; the full suite is 296 assertions across 6
executables, 100% passing.
