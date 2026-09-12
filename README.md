# truck-drone-dynamic-routing

C++17/20 reproduction of the computational methodology in:

Han, B., Chi, T., & Sun, Z. (2026). *Collaborative delivery problem of trucks
and multiple drones: an en-route operation with flexible launch positions.*
International Transactions in Operational Research, 33, 4072-4109.

## Scope

This repository currently implements **Stage 1 only**: the static
truck-multi-drone routing problem with flexible launch positions, its
constructive initial solution, and the VNS metaheuristic, targeting the
synthetic instances Paper 1 describes (10/20/50 customers). The dynamic
customer-request extension (Stage 2) is a separate, later phase of the
BTP project and is not part of this codebase yet.

The full planning documents (Paper 1 analysis, the dynamic-stage research
understanding, and the detailed implementation blueprint with its
[PAPER]/[INFERENCE]/[ASSUMPTION] contract table) are maintained alongside
this repository; `DECISIONS.md` in this repo tracks concrete implementation
decisions as they are made, with the reproduction-fidelity implications of
each spelled out.

## Status

Implemented so far:

- `tsp/` — the truck-only TSP solver used in stage (a) of the initial-solution
  procedure (Paper 1, Sec. 5.2). See `DECISIONS.md` for why this exists as
  two solvers (an exact one and a documented heuristic substitute) rather
  than a single method.
- `model/` — the static core data model: `Node`/`NodeType`/`NodeRegistry`,
  `CustomerData`, `FlexibleLaunchPointData`, `TruckParameters`/
  `DroneParameters`, `DroneTrip` (launch -> customer -> recovery), the
  `TruckRoute` sequence, and the top-level `Solution` (truck route + drone
  trips). Header-only, independent of `tsp/`, and deliberately free of any
  timing/feasibility/optimization logic -- see `DECISIONS.md` for the
  modeling choices made while building it.
- `graph/` — the static 2D geometry and travel-time infrastructure:
  Euclidean distance, truck/drone travel time at a constant speed, and
  `SpatialNetwork`, which bundles a `NodeRegistry` with truck/drone
  parameters to answer distance/time queries and build dense matrices
  over a chosen set of node ids. Header-only, builds only on `model/`
  (not `tsp/`). Deliberately stops at raw travel time -- it does not
  apply Paper 1's load-dependent generalized-flight-time adjustment
  (eq. 35); see `DECISIONS.md`.
- `flp/` — the flexible-launch-point (FLP) generator: given a truck route
  and the node registry, places launch points on the arcs the route
  actually traverses, evenly spaced per a configurable
  `FlpGenerationPolicy`. Header-only, builds only on `model/`. Paper 1
  does not specify a numeric placement rule for the main synthetic
  experiments, so the placement policy here is an explicit, isolated,
  swappable **[ASSUMPTION]** baseline -- see `DECISIONS.md` for exactly
  what is and is not paper-supported here.
- `timing/` — the centralized timing and feasibility engine: given a
  complete `Solution`, computes truck arrival/departure at every route
  position, every drone trip's launch/flight/service/recovery timing,
  synchronization waits, battery-endurance feasibility, per-customer
  service resolution, overall solution feasibility with violation
  reasons, and the Paper 1 objective (sum of each customer's own service
  arrival time -- never a vehicle's recovery/completion time). Header-only,
  builds on `model/` and `graph/` only (not `tsp/` or `flp/`), and
  contains no optimization/search logic. Several timing conventions here
  (e.g. how eq. 35's generalized flight time is used only for endurance,
  never real-event timing; how simultaneous launches/recoveries are
  handled) are necessary inferences or explicit assumptions rather than
  directly-stated paper rules -- see `DECISIONS.md` for the full
  [PAPER]/[INFERENCE]/[ASSUMPTION] breakdown.
- `init/` — Paper 1's initial-solution construction procedure: an
  optimal (or, above Held-Karp's practical size, a clearly-labeled
  heuristic) truck-only TSP route, K-means customer clustering with
  `k = ceil(|C| / (|U| + 2))`, a frozen pool of candidate flexible
  launch points generated once from that route (reusing `flp/`
  unchanged), and a greedy construction of feasible drone trips, cluster
  by cluster, modeled on the paper's Algorithm 2. Builds on `model/`,
  `graph/`, `flp/`, `timing/`, and `tsp/`; deliberately duplicates none
  of `timing/`'s logic -- every candidate trip's feasibility, including
  whether accepting it would ever require more drones simultaneously in
  the air than the fleet has (`init/drone_fleet_capacity.hpp`, checked
  globally across the whole construction, not reset per cluster), is
  decided by calling `tdr::timing::evaluate_solution()` on a trial
  solution and reading its own computed timings back. Paper 1 specifies
  CPLEX for the truck-only route's exact solve, which is unavailable
  here; this is documented as an implementation limitation, not a Paper
  1 rule, and the exact/heuristic distinction is preserved and surfaced
  rather than silently substituted. Several procedural details Algorithm
  2's OCR'd prose leaves unspecified (the frozen-vs-regenerated FLP pool,
  `BestRoute()`'s scoring rule, and the exact scope of "adjacent" launch
  candidates) are explicit, isolated **[ASSUMPTION]**s -- see
  `DECISIONS.md` for the full breakdown.

Everything else described in the implementation plan (the 8 local-search +
4 shake neighborhoods, the VNS driver, the experiment runner) is designed
but not yet implemented.

## Building

Requires CMake >= 3.16 and a C++20 compiler.

```
mkdir build && cd build
cmake ..
cmake --build .
ctest --output-on-failure
```

## Layout

```
include/   public headers, mirrors src/ module by module
src/       implementations
config/    experiment configuration (data, not code) -- not populated yet
instances/ generated/saved problem instances -- not populated yet
results/   experiment output -- not populated yet
tests/     test executables (plain assertions + PASS/FAIL output, no
           external test framework dependency yet)
scripts/   optional post-processing/plotting helpers (Python), not part
           of the C++ build
```
