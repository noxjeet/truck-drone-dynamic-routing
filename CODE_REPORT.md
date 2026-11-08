# Code Report — `truck-drone-dynamic-routing`

This is a full walkthrough of every file in the repository as it stands today, written so you can genuinely own this code — not just point at it. The codebase has grown a lot since the last version of this report (it was TSP-only; it now has six modules), so this is a full rewrite, not a patch.

You told me you know STL-heavy competitive-programming C++ (vectors, algorithms, bitmask DP) but not the "software engineering" side of C++ — classes, headers, namespaces, inheritance. So **Section 1 is a syntax primer** covering every C++ feature that shows up anywhere in this codebase but wouldn't show up in a CP solution. Read that once, then use it as a reference while reading Sections 2–8, which go module by module through every file. Section 9 is a CMake tutorial. Section 10 is the complexity/"how do I make this cheaper" discussion from before, kept and lightly updated.

**Table of contents**
1. C++ syntax primer (everything here that CP doesn't teach you)
2. `model/` — the data types
3. `graph/` — distance and travel time
4. `flp/` — flexible launch point generator
5. `timing/` — the timing/feasibility engine (the biggest, most important file)
6. `init/` — initial-solution construction
7. `tsp/` — the TSP solvers (recap)
8. Tests — all six test executables
9. CMake — a tutorial through this project's `CMakeLists.txt`
10. Complexity and how to reduce it

---

## 1. C++ syntax primer

Everything below is a feature you'll hit somewhere in the next seven sections. Skim it now; come back to any one entry when you hit the thing it explains.

**`#pragma once`** — at the top of every header. Headers get `#include`d into multiple `.cpp` files, and if the compiler processed the same header's contents twice while building one file (e.g. A includes B and C, and both B and C include D), you'd get "redefinition" errors. `#pragma once` tells the compiler "if you've already pulled this exact file in once during this compile, skip it the second time." It's a header guard, done the easy modern way instead of the older `#ifndef FOO_H / #define FOO_H / ... / #endif` pattern.

**Namespaces** — `namespace tdr::model { ... }`. A namespace is just a named box you put names in, so two unrelated `Node` classes in different parts of a large project don't collide. `tdr::model::Node` and `tdr::graph::SpatialNetwork` can never be confused even though both live under the umbrella `tdr`. `tdr::model` (with the `::`) is C++17's shorthand for nesting one namespace inside another (`namespace tdr { namespace model { ... } }`, but in one line). Inside a `.cpp` file you'll see `using namespace tdr::model;` at the top of test files — that means "let me write `Node` instead of `tdr::model::Node` for the rest of this file." Doing that in a header would be bad practice (it would leak into every file that includes that header), but it's fine and common in a small, self-contained test `.cpp`.

**`struct` vs `class`** — in C++ these are almost the same thing; the *only* difference is the default visibility of members. In a `struct`, members are `public` by default (anyone can read/write them directly). In a `class`, members are `private` by default (only the class's own methods can touch them, unless marked `public`). This codebase uses `struct` for plain data bags with no invariants to protect (`Node`, `TruckParameters`, `TSPResult`...) and `class` for things that guard their own internal state (`NodeRegistry`, `SpatialNetwork`, the TSP solvers) — that choice of keyword is itself documentation of intent, not just a style preference.

**Default member initializers** — `double cost = 0.0;` inside a struct/class body. This means "if nobody sets this field explicitly, it starts at 0.0" rather than starting as garbage memory (which is what plain C, and old-style C++, would leave you with). You'll see this on almost every field in every struct in `model/` — it's what makes `TSPResult result;` (no arguments) immediately safe to read from before you've filled anything in.

**Constructors and member-initializer lists** — 
```cpp
SpatialNetwork(NodeRegistry nodes, TruckParameters truck, DroneParameters drone)
    : nodes_(std::move(nodes)), truck_(truck), drone_(drone) {}
```
The part after the `:` and before the `{}` is the *member initializer list* — it sets each member variable directly as the object is being constructed, before the (here, empty) constructor body runs. This is the idiomatic way to initialize members in C++, preferred over assigning them inside the `{}` body, partly for efficiency and partly because some kinds of members (references, `const` members) can *only* be set this way.

**`const` correctness** — you'll see `const` in three different roles constantly:
- `const DistanceMatrix& distances` as a parameter: "give me a reference to your data, don't copy it, and I promise not to modify it." 
- `double distance(NodeId a, NodeId b) const { ... }` — the `const` *after* the parameter list means "this method promises not to modify the object it's called on." Calling a non-`const` method on a `const SpatialNetwork&` would be a compile error; this is the compiler enforcing your own promise.
- `inline constexpr NodeId kInvalidNodeId = -1;` — a compile-time constant (more below).

**References (`&`)** — `const DistanceMatrix& distances` means "an alias for the caller's actual object, not a copy." This is the C++ equivalent of the CP habit of passing `vector<int>&` instead of `vector<int>` to avoid an O(n) copy on every call — except in C++ you also almost always pair it with `const` when you're not modifying it, which is what lets the compiler and other readers trust the function won't mutate the caller's data.

**Inheritance, `virtual`, `override`, pure-virtual (`= 0`)** — this is the biggest genuinely new concept versus CP. 
```cpp
class ITSPSolver {
public:
    virtual ~ITSPSolver() = default;
    virtual TSPResult solve(const DistanceMatrix& distances) const = 0;
};

class HeldKarpSolver : public ITSPSolver {
public:
    TSPResult solve(const DistanceMatrix& distances) const override;
};
```
`class HeldKarpSolver : public ITSPSolver` means "`HeldKarpSolver` *is a kind of* `ITSPSolver`." The `= 0` after `solve(...)` in `ITSPSolver` makes it a *pure virtual function*: `ITSPSolver` has no implementation for `solve` at all, and can never be instantiated directly — it exists purely as a contract ("any concrete solver must provide a `solve`"). `override` on `HeldKarpSolver::solve` is the derived class saying "yes, this is deliberately filling in that contract" — the compiler cross-checks the signature matches exactly and errors out if it doesn't, which catches a whole class of "I meant to override this but made a typo" bugs. `virtual ~ITSPSolver() = default;` is a virtual destructor — necessary so that if you ever delete an object through a base-class pointer, the *derived* class's cleanup still runs. This whole pattern (`ITSPSolver` interface, several concrete implementations) is what lets `TSPSolverFactory` return "some solver" without the caller needing to know or care which concrete class it actually is.

**`static` and `constexpr`** — `static constexpr int kDefaultMaxExactN = 21;` inside a class means this value belongs to the *class itself*, not to any individual object (`HeldKarpSolver::kDefaultMaxExactN`, no instance needed), and `constexpr` means the compiler can (and does) bake the value in at compile time rather than reading it from memory at runtime. `inline constexpr NodeId kInvalidNodeId = -1;` at namespace scope uses `inline` for a related but different reason: a `constexpr` variable defined in a header and included in multiple `.cpp` files would otherwise violate C++'s "one definition per program" rule; `inline` tells the linker "it's fine, all these definitions are identical, just merge them."

**`explicit`** — `explicit HeldKarpSolver(int max_exact_n = kDefaultMaxExactN, ...)`. Without `explicit`, a single-argument constructor would let the compiler silently convert, e.g., an `int` into a `HeldKarpSolver` wherever one was expected — a common source of confusing bugs. `explicit` disables that silent conversion; you must write `HeldKarpSolver(21)` on purpose, it can never happen by accident.

**Enums (`enum class`)** — `enum class NodeType { Depot, Customer, FlexibleLaunchPoint };`. This is a type-safe enumeration: `NodeType::Customer` cannot be silently compared to or mixed up with an unrelated `int` or a different enum, unlike old C-style `enum`. You always write the full `NodeType::Customer`, never bare `Customer`.

**Templates + lambdas as "generic callable"** — in `spatial_network.hpp`:
```cpp
template <typename Fn>
DistanceMatrix build_matrix(const std::vector<NodeId>& ids, Fn value_for) const { ... }
```
```cpp
DistanceMatrix build_distance_matrix(const std::vector<NodeId>& ids) const {
    return build_matrix(ids, [this](NodeId a, NodeId b) { return distance(a, b); });
}
```
`template <typename Fn>` means "this function works for *any* type `Fn`, decided when it's actually called" — here, `Fn` ends up being the compiler-generated type of a lambda. `[this](NodeId a, NodeId b) { return distance(a, b); }` is a **lambda** — an anonymous, inline function value. The `[this]` part is the *capture list*: it says "this lambda needs access to the enclosing object's `this` pointer" (so it can call `distance()`, a member function, from inside the lambda). This pattern — a template parameter that gets filled in with a lambda — is how this codebase gets the effect of "pass a function as a parameter" (like a CP comparator you pass to `sort`) without the runtime overhead of a virtual-function-style indirection; the compiler generates a specialized version of `build_matrix` for each different lambda it's called with.

**`std::optional<T>`** — used throughout `timing_engine.hpp`, e.g. `std::optional<std::size_t> find_route_position(...)`. This represents "either a value of type `T`, or nothing at all" — the modern, type-safe replacement for the C habit of returning `-1` to mean "not found." You check `if (result.has_value())` (or just `if (result)`) and dereference with `*result` to get the value out. It forces you to handle the "not found" case explicitly rather than accidentally treating a sentinel `-1` as a real index.

**`std::unordered_map` and structured bindings** — you already know `unordered_map` from CP (it's the C++ hash map). The new syntax is *structured bindings*:
```cpp
for (const auto& [id, node] : network.nodes().all()) { ... }
```
`all()` returns an `unordered_map<NodeId, Node>`; each element of that map is a `pair<const NodeId, Node>`. `const auto& [id, node]` unpacks that pair into two named variables `id` and `node` in one line, instead of the older `it->first` / `it->second` style.

**Anonymous namespaces vs. `namespace detail`** — you'll see both patterns:
```cpp
namespace {
double tour_cost(const DistanceMatrix& d, const std::vector<int>& tour) { ... }
}
```
An *unnamed* (anonymous) namespace makes every name inside it private to that one `.cpp` file — nothing outside can ever call `tour_cost` from another file, even by accident. This is used in the `.cpp` implementation files for pure helper functions nobody else should ever need. Separately, several headers use `namespace detail { ... }` (a namespace that *does* have a name, just one that signals "internal, not part of the public API, use at your own risk") — the difference is that a named `detail` namespace's contents can still be shared between multiple functions *within the same header*, where a fully anonymous one is scoped to a single file.

**`inline` on free functions in headers** — e.g. `inline double euclidean_distance(...)` in `graph/distance.hpp`. Every `.cpp` file that includes this header gets its own copy of this function's code. Without `inline`, the linker would see multiple definitions of the same function across different `.cpp` files and refuse to link (violating the "one definition per program" rule again). `inline` here doesn't really mean "paste this code at the call site" (that's a compiler optimization decision, made independently); its actual, load-bearing meaning in modern C++ is "it's fine for this definition to appear in multiple translation units, please merge them." This is exactly why so much of this codebase can be **header-only** — a whole module (`model/`, `graph/`, `flp/`, `timing/`) can consist purely of headers with `inline` functions and never need a matching `.cpp` file at all, because there's no "the one true copy lives here" issue to solve.

**`std::move`** — `nodes_(std::move(nodes))` in a constructor. `nodes` here is a parameter passed *by value* (a full copy already made when the caller called the constructor). `std::move(nodes)` doesn't itself copy or move anything — it's a cast that says "I'm done with this local variable, you're allowed to cannibalize its internals instead of copying them again." For something like a `NodeRegistry` wrapping a big hash map, this turns a second expensive deep-copy into a cheap pointer-swap.

**Sentinel values instead of exceptions** — this codebase deliberately never throws C++ exceptions for expected failure cases. Instead you'll see patterns like `bool succeeded = false;` + `std::string note;` (on `TSPResult`), or `kInvalidNodeId = -1` as an "unset" marker, or `std::optional` (above). This is a design choice, not a limitation of C++ — it makes every failure path visible in the function's return type and forces callers to check it, rather than relying on a `try`/`catch` somewhere else in the program.

**`std::size_t`** — the unsigned integer type used for sizes and container indices (`std::vector::size()` returns this). You'll see a lot of `static_cast<std::size_t>(...)` and `static_cast<int>(...)` conversions between this and plain `int` — necessary because mixing signed and unsigned integers in comparisons is a classic C++ footgun (e.g. `int i = -1; if (i < someVector.size())` is true when you'd expect false, because `-1` gets silently reinterpreted as a huge unsigned number). The codebase is careful and explicit about these casts everywhere rather than letting the compiler pick a conversion for you.

**One documentation convention that isn't C++ at all**: you'll see `[PAPER]`, `[INFERENCE]`, and `[ASSUMPTION]` tags all over the comments. This is this project's own convention (not a language feature) for tracking reproduction fidelity against the source paper: `[PAPER]` = a directly-stated rule from the paper; `[INFERENCE]` = a necessary logical consequence of paper facts, not itself stated verbatim; `[ASSUMPTION]` = a choice the paper leaves open that the code had to make on its own. It's worth reading these tags as you go, since they tell you which numbers in the code are "the paper said so" versus "we had to pick something."

---

## 2. `model/` — the data types

Seven headers, all header-only, all pure data with (almost) zero logic — namespace `tdr::model`. This module's whole job is to give every other module a shared, agreed-upon shape for "what is a node," "what is a customer," "what is a route." Nothing here computes a distance, a time, or a feasibility verdict.

**`node.hpp`** — the foundation. `NodeId` is just `using NodeId = int;` (a type alias — a nickname for `int`, purely for readability; the compiler treats `NodeId` and `int` as 100% interchangeable). `NodeType` is an `enum class` with three values: `Depot`, `Customer`, `FlexibleLaunchPoint`. `Point2D` is `{double x; double y;}`. `Node` bundles an id, a type, and coordinates — the same shape for every kind of node; type-specific extra data (a customer's service time, a launch point's arc position) deliberately lives in *separate* structs that reference a `Node` by id, rather than being crammed into `Node` itself, so `Node` stays uniform everywhere. `NodeRegistry` is a small class wrapping `std::unordered_map<NodeId, Node>`: `add()` (returns `false` without changing anything if the id is already taken — using `unordered_map::emplace`'s return value, whose `.second` is `true` only on an actual insertion), `contains()`, `at()` (throws if missing — `unordered_map::at` is the "safe," bounds-checked lookup, versus `operator[]` which would silently insert a default-constructed entry), `size()`, `all()`.

**`customer.hpp`** — `CustomerData`: per-customer service times (`truck_service_time`, `drone_service_time`) and the load-dependent flight-time coefficient `theta`, keyed back to a `Node` via `node_id`. The comment explains *why* these are per-customer fields rather than single global constants — because the paper's own notation indexes them by customer, even though this project's actual experiments happen to set the same value for everyone.

**`parameters.hpp`** — `TruckParameters` (just `speed`) and `DroneParameters` (`speed`, `endurance_seconds`, `launch_prep_time_sl`, `recovery_time_sr`, `num_drones`). Global, not per-vehicle — the paper treats all drones as identical.

**`flexible_launch_point.hpp`** — `FlexibleLaunchPointData`: which arc `(arc_from, arc_to)` a launch point sits on, and `position_ratio` (0 = at `arc_from`, 1 = at `arc_to`). Note this struct only *records* where a point sits — deciding *how many* points to generate and at what ratios is `flp/`'s job (Section 4), not this struct's.

**`drone_trip.hpp`** — `DroneTrip`: `launch_node`, `service_customer`, `recovery_node` (the paper's tuple `(i,j,k)`), plus `drone_id` — the physical drone u ∈ U that flies the trip (0 ≤ u < |U|, default `kUnassignedDroneId = -1`; one drone may fly several trips one after another, so the number of trips is not bounded by |U|) — and two unused cached route-position fields. Two free (non-member) `inline` functions do *structural* sanity checks only — `has_distinct_endpoints` (are the three node ids actually three different ids, and none of them unset) and `has_valid_node_roles` (does `launch_node` really point at a `FlexibleLaunchPoint`-typed node in the given registry, is `service_customer` really a `Customer`, is `recovery_node` really a `Customer` or `Depot`). Neither function knows or cares about time or feasibility — that's `timing/`'s job entirely; this is purely "is this triple of ids even a legal shape."

**`truck_route.hpp`** — one line: `using TruckRoute = std::vector<NodeId>;`. A route is just an ordered list of node ids, start and end at the depot. No wrapper class, because there's no invariant to protect at this layer.

**`solution.hpp`** — `Solution { TruckRoute truck_route; std::vector<DroneTrip> drone_trips; };` — a complete candidate answer: one truck route plus a bag of drone trips, connected only by shared node ids (not embedded into the route sequence itself). Deliberately minimal — no cached timing, no feasibility flag — because *computing* those things is exclusively `timing/`'s responsibility (Section 5); a `Solution` only needs to be *capable of holding* one, not evaluate one.

---

## 3. `graph/` — distance and travel time

Three headers, namespace `tdr::graph`, building only on `model/`.

**`distance.hpp`** — `euclidean_distance(Point2D, Point2D)` computes `sqrt(dx² + dy²)`, the straight-line distance formula (a `Node`-overload just forwards to the `Point2D` one via `a.coordinates, b.coordinates`). This is the *only* distance model anywhere in the repo — no road network, no map API.

**`travel_time.hpp`** — `travel_time_for_distance(distance, speed)` does `distance / speed`, with one safety detail: if `speed <= 0.0` it returns `std::numeric_limits<double>::infinity()` instead of dividing by zero. This matters because a default-constructed `TruckParameters`/`DroneParameters` has `speed = 0.0` (remember Section 2's default member initializers) — if someone forgets to configure speed, they get an unmistakable "infinite travel time" rather than a silent `NaN` or a crash. `truck_travel_time` and `drone_travel_time` are thin wrappers over this shared function. A load-bearing comment here: this file computes the *raw physical* flight time only — the paper's separate "generalized flight time" (which inflates the outbound leg by a load-dependent factor for battery-endurance purposes) is deliberately left to `timing/`, so this low-level module never has to know about that adjustment at all.

**`spatial_network.hpp`** — `SpatialNetwork` is the class that actually gets used by everything downstream: it bundles a `NodeRegistry` (stored *by value*, i.e. a full copy — the comment explains this is a deliberate safety trade against a dangling-reference risk, and that a copy is cheap enough at this project's scale) with `TruckParameters` and `DroneParameters`, and answers `distance(a, b)`, `truck_time(a, b)`, `drone_time(a, b)` between any two node ids. It also has three `build_*_matrix(ids)` methods that build a dense `n×n` matrix over a caller-chosen ordered list of ids — this is exactly what `tsp/`'s `DistanceMatrix` needs, and `graph::DistanceMatrix` is deliberately defined as a second, independent alias for the same underlying type (`vector<vector<double>>`) rather than including `tsp/tsp_types.hpp` directly, keeping `graph/` from depending on the solver module even though the two types are structurally identical and interchangeable. All three `build_*_matrix` methods are one-line callers of a shared private `build_matrix` template — this is the template+lambda pattern from Section 1: `build_distance_matrix` passes a lambda wrapping `distance()`, `build_truck_time_matrix` passes one wrapping `truck_time()`, and so on, so the actual double-loop-over-a-grid logic exists exactly once.

---

## 4. `flp/` — the flexible-launch-point generator

One header, `flexible_launch_point_generator.hpp`, namespace `tdr::flp`, building only on `model/`.

The problem: the paper's method lets a drone launch not just from a customer node, but from a point partway along an arc the truck drives through. This file's whole job is: given a truck route and a policy, generate those points.

`FlpGenerationPolicy` holds `count_rule` (default `UnitDivisionLength`: n = floor(D/d) points on an arc of length D, the structure of the paper's eq. (41), so the number of launch points is proportional to driving distance), `unit_division_length` (d — **no default**: it is 0, and the generator refuses to run with an error until the caller sets it, because the paper never states the main-experiment d; the d values in the sensitivity analysis are not the main-experiment value), `points_per_arc` (used only by the opt-in `FixedCountPerArc` rule, which is kept for experiments but is not paper-supported), and `min_arc_length` (a near-zero-length arc gets no points). `flp_count_for_arc(D, policy)` returns the count (or -1 for an invalid policy).

`generate_flexible_launch_points_on_arcs(arcs, nodes, policy, next_id)` is a pure function — no side effects, no randomness, same inputs always give the same outputs. For each given arc (a, b), in order, it computes the arc's length, asks `flp_count_for_arc` for n, and places n points evenly along it at ratios `1/(n+1), ..., n/(n+1)`. Each point gets a fresh sequential id starting from `next_id` and is returned with a `FlexibleLaunchPointData` (its owning arc — i.e. which set L_(a,b) it belongs to — and its ratio) and the index of the arc in the input list. The result carries `ok`/`error`, so an unset d is reported rather than silently defaulted. `generate_flexible_launch_points(truck_route, nodes, policy, next_id)` is a convenience overload that uses every consecutive pair of the route as the arc list.

---

## 5. `timing/` — the timing and feasibility engine

One header, `timing_engine.hpp` — by far the largest and most important file in the codebase (roughly 530 lines). Namespace `tdr::timing`, building on `model/` and `graph/` only. The comment block at the top is explicit: **this is the only place in the entire repository allowed to compute time, feasibility, or the objective value** — no other module is supposed to duplicate any of this logic, ever. Everything downstream (`init/`, and later the not-yet-built local-search/VNS code) is required to call into this file and read the answer back rather than recomputing it.

### What it's actually simulating

Given one complete `Solution` (a truck route + a set of drone trips), figure out, second by second: when does the truck arrive at and leave every stop; when does each drone launch, fly out, deliver, fly back, and get picked back up; does every trip fit inside its drone's battery endurance; is every customer served exactly once by exactly one vehicle; and what is the paper's actual objective (the sum, over every customer, of *whichever vehicle serves them*'s arrival time at that customer — never a vehicle's own total route time).

### The types

`DroneTripTiming` — every intermediate number for one trip (`launch_time`, `outbound_flight_time_raw`, `customer_arrival_time`, `customer_departure_time`, `return_flight_time_raw`, `recovery_arrival_time`, `recovery_complete_time`, plus the endurance-adjusted `generalized_flight_time`) and its own `feasible`/`violation_reason`. `TruckTimingResult` — two parallel arrays, `arrival_time[p]` and `departure_time[p]`, indexed by *route position* (not by node id — deliberately, since the same node id, the depot, legitimately appears twice in a round-trip route, at position 0 and again at the last position, and indexing by id would make those ambiguous). `CustomerServiceRecord` — per customer, which vehicle served them and their objective-feeding arrival time. `SynchronizationRecord` — for each recovery rendezvous, who arrived first and how long the other one waited. `SolutionEvaluationResult` — the whole bundle, plus overall `feasible` and `objective_value`.

### The algorithm: one forward pass

`evaluate_solution(solution, network, customers, flp_data)` is the single function (`flp_data` maps every launch point id to its `FlexibleLaunchPointData`, i.e. its owning arc) that does everything, and it's worth understanding as one continuous story rather than a pile of separate checks:

1. **Structural preconditions.** The route must have at least 2 entries and should start/end at depot-typed nodes (violations are recorded but don't early-return, except a too-short route, which genuinely can't be processed at all). Then, for every launch point on the route: it may appear only once (constraints (8)–(9)); it must have arc-ownership data; and its owning arc (i, j) must be exactly the pair of truck nodes (non-launch-point nodes) immediately before and after it on the route — the paper's rule that a launch point of L_ij is reachable only while the truck drives arc (i, j) (p9, constraints (13)–(15)). After the per-trip lookup (step 2), every launch point on the route must launch at least one drone (constraint (6)).

2. **Per-trip precomputation, before the main pass.** For every drone trip, it works out: is it structurally valid (Section 2's `has_distinct_endpoints`/`has_valid_node_roles`)? What route position is its launch at, and what position is its recovery at? Route position lookup uses two helper functions — `find_route_position` (first occurrence anywhere) and `find_route_position_from(route, id, from_index)` (first occurrence *at or after* a given index). The second one exists specifically to solve the "start depot and end depot share one id" ambiguity mentioned above: since a drone's recovery must physically happen *after* its launch in time, searching for the recovery node starting just past the launch position is both the physically correct search and the fix for that ambiguity in one move. Two lookup tables, `trips_launching_at[p]` and `trips_recovering_at[p]` (each a `vector<vector<size_t>>`, one inner list per route position), are built so the main pass can answer "what launches/recovers here?" by direct index instead of re-scanning every trip at every position.

3. **The single forward pass over route positions**, `for (p = 0; p < n; ++p)`. This is the heart of the file:
   - Position 0: the truck starts at time 0 (`[PAPER]`, p15: operating time starts at 0).
   - Every later position: `arrival[p] = departure[p-1] + truck_time(route[p-1], route[p])` — pure propagation, each stop's arrival depends only on the previous stop's departure plus travel time.
   - If the current node is a `FlexibleLaunchPoint`: every launching trip's `launch_time` is `arrival[p] + SL` (constraint (21) — drones are not staggered), and the truck leaves at `arrival[p] + n·SL`, where n is the number of drones launched there (constraint (18)).
   - If the current node is a `Customer` or `Depot`: the departure time is the max of two independent lower bounds — the truck's own service completion (`arrival[p] + Se1`, 0 if this node has no `CustomerData`, i.e. it's the depot) and the SR terms for every trip recovering here. Since Stage 2 (8 Nov 2026, decision D1, SR Option B) the departure is `l_p = max( t_p + Se_p^T + r_p * SR , max_u (a_u + SR) )`, with `r_p` the number of drones recovered at p and `a_u` each drone's own physical landing time: recovery work is additive to the truck's own service, and each drone's own completion is a separate lower bound. SR is never charged at a launch FLP. (Option B is an interpretation of an ambiguous printed formula; see DECISIONS.md, Stage 2. The older text in this paragraph described the pre-Stage-2 rule `max(arrival[p], landing) + SR`.) Taking the `max()` across all of these encodes "the truck can't leave until *everything* happening at this stop — its own service, and every drone recovery — has finished." Whichever of those actually turns out largest determines the real departure time. Each recovering trip's full timing chain is computed right here, inline: outbound flight time = `drone_time(launch, customer)`, customer arrival = `launch_time + outbound`, customer departure = `arrival + Se2`, return flight time = `drone_time(customer, recovery)`, physical landing = `departure + return flight`.

4. **Endurance, computed separately from real time, exactly as constraints (36)/(37).** With l̂_i = drone departure from the launch point, l̂_j = drone departure from the customer (so l̂_j − l̂_i covers the loaded outbound flight *and* Se^D_j), t_k = truck arrival at the recovery node and d̂_jk = return flight: `endurance_value_36 = (t_k − l̂_j) + (l̂_j − l̂_i)(1 + θ_j)` (this one includes the drone waiting for a late truck) and `endurance_value_37 = (l̂_j − l̂_i)(1 + θ_j) + d̂_jk`. The trip is endurance-feasible iff both are `<= endurance + kTimeEpsilon`; `generalized_flight_time` reports the larger. SL and SR are not part of either expression. As before, these values feed only the battery check — every real timestamp uses raw flight times. The trip is also rejected if its `drone_id` is not in {0, …, |U|−1}.

   **Physical drones.** For each drone u, its trips ordered by launch position must not overlap: a relaunch must come strictly after the previous trip's recovery position (route-order form of constraint (28)). Separately, the largest number of trips in flight at one route position (`max_simultaneous_drones`) must not exceed |U|.

5. **Customer service resolution.** For every customer in the `customers` map, it checks: is this id present in the truck route at all, and how many drone trips list it as `service_customer`? Exactly one of (truck-served, one feasible drone trip) should be true; anything else (served by both, served by neither, served by more than one drone trip, or served by exactly one drone trip that is itself infeasible) is recorded as an infeasible, explained record.

6. **Synchronization records** — for every feasible trip, who arrived at the recovery point first and by how much, purely for inspection/debugging.

7. **Objective and overall feasibility** — sum every customer's `service_arrival_time` for the objective; the whole solution is feasible iff the `violation_reasons` list ended up empty.

The header comment is unusually candid about where a judgment call was made versus where the paper states something directly — worth reading in the file itself if you want the full [PAPER]/[INFERENCE]/[ASSUMPTION] breakdown for every individual decision (there's a particularly detailed one about whether truck service and a drone recovery at the *same* node run concurrently or sequentially, which the code implements as concurrent — `max()`, not `+` — and documents a hand-worked test specifically to pin that behavior down).

---

## 6. `init/` — initial-solution construction

Four headers, namespace `tdr::init`, orchestrating everything above into Paper 1's actual initial-solution procedure (their "Algorithm 2").

**`truck_only_route.hpp` (step F1)** — `build_truck_only_route(depot_id, customer_ids, network, solver_factory)` builds the ids list `[depot, customer_1, ..., customer_k]`, asks `network.build_truck_time_matrix(ids)` for the matrix, and hands it to a `TSPSolverFactory` (Section 7) — reusing the existing TSP module completely unchanged. If there are no customers at all, it short-circuits to a trivial "the route is just the depot" answer rather than calling the solver on a degenerate 1-node instance. It reconstructs the actual route (a list of real node ids, not the TSP solver's internal 0..n-1 indices) by mapping each returned tour index back through the `ids` list, and appends the depot again at the end to close the loop.

**`customer_clustering.hpp` (step F2)** — `compute_cluster_count(num_customers, num_drones)` implements the paper's formula `k = ceil(|C| / (|U| + 2))` directly (using integer-arithmetic "ceiling division" — `(numerator + divisor - 1) / divisor` — a standard CP trick for computing `ceil(a/b)` with integers only). `run_kmeans(customer_coordinates, k, policy)` implements deterministic Lloyd's-algorithm K-means: seed `k` initial centroids by taking evenly-spaced customers from a sorted-by-id list (a fully deterministic seeding rule with no randomness at all, chosen specifically because the paper doesn't specify a seeding rule and a hidden RNG would make results non-reproducible), then alternate "assign every point to its nearest centroid" and "recompute each centroid as the mean of its assigned points" until nothing changes or a movement threshold isn't crossed, capped at `max_iterations`. Every ambiguous detail (tie-breaking, empty clusters, the iteration cap) is spelled out in comments as an explicit `[ASSUMPTION]`.

**`drone_fleet_capacity.hpp`** — a small, self-contained time-interval sweep-line check (turn every trip into a `+1` at launch and a `-1` at recovery-complete, sort, track the running maximum, compare it with `num_drones`). **Since the 29 Sep 2026 corrections F4 no longer uses it**: fleet limits are now enforced through explicit drone ids and the evaluator's (28)/|U| checks. It is kept as a standalone utility and unit-tested in `test_init.cpp` Block 7.

**`initial_solution_builder.hpp` (F1–F4 orchestration)** — the biggest and most intricate file in `init/`. `build_initial_solution(depot_id, customer_ids, customers, network, policy, solver_factory)` runs, in order: F1 (the truck-only route, above), F2 (K-means clustering on customer coordinates, independent of the route), F3 (generate, once, the launch points L_(a,b) for *every* ordered pair of truck nodes — depot and customers — via `flp/`; if d is not configured it stops with an `F3:` diagnostic), then F4, a greedy, cluster-by-cluster construction loop. For a customer j on the current route, let a and b be the truck nodes just before and after it (launch points are skipped). Converting j to drone service removes it, so the truck then drives the arc (a, b); j's launch candidates are therefore exactly the points of L_(a,b), each placed where j was, with recovery at b. Each candidate is tested by building a trial `Solution`, assigning physical drone ids (`detail::assign_drone_ids_by_route_order`: trips in route order take the lowest-numbered drone already recovered; it fails only if more than |U| trips are in flight at once) and calling `tdr::timing::evaluate_solution` with the arc-ownership data — so arc membership, the launch-point visit rules, drone reuse, |U| and endurance are all checked by the evaluator, never re-implemented. The best-scoring feasible candidate (earliest customer arrival) is committed each round, until no cluster customer has a feasible candidate left. `InitialSolutionResult` also returns `flp_data` so callers can re-evaluate the solution. `test_init.cpp` Block 10 shows that every launch point of the bridging arc is tried: the chosen one is the second of four.

---

## 7. `tsp/` — the TSP solvers (recap)

This module is unchanged since the previous version of this report, so here's the condensed version; ask if you want the full line-by-line walkthrough again.

- **`tsp_types.hpp`**: `DistanceMatrix` (`vector<vector<double>>`) and `TSPResult` (tour, cost, `exact`, `succeeded`, method name, timing, a caveat note).
- **`tsp_solver.hpp`**: `ITSPSolver`, the abstract interface (Section 1's inheritance pattern) every solver implements.
- **`held_karp_solver.hpp`/`.cpp`**: the exact Held-Karp bitmask-DP solver — `dp[mask][j]` = cheapest cost to have visited exactly the customer set `mask`, currently standing at `j`. Refuses to run past a configurable size (`max_exact_n`, default 21) or an estimated-memory limit (default 4 GiB) rather than attempting an impractical solve.
- **`heuristic_tsp_solver.hpp`/`.cpp`**: the non-exact fallback — multi-restart nearest-neighbor construction, improved by 2-opt and Or-opt local search, always reporting `exact = false`.
- **`tsp_solver_factory.hpp`/`.cpp`**: the single decision point — try Held-Karp if the instance is small enough, fall back to the heuristic only on an explicit refusal.

---

## 8. Tests — all six executables

Every test file follows the same house style: a local `check(condition, description)` that prints `[PASS]`/`[FAIL]` and bumps a global failure counter, no external test framework, `main()` returns a non-zero exit code if anything failed (which is what lets `ctest` detect success/failure). Several also use `check_near(a, b, tol, description)` for floating-point comparisons. A shared fixture, `tests/fixtures/small_graph_fixture.hpp`, defines one small hand-checkable 5-node layout (built from 3-4-5-triangle distances so nothing needs `sqrt()` to verify by hand) that `test_graph.cpp` and `test_flp.cpp` both reuse.

**`test_tsp.cpp`** (unchanged) — Held-Karp checked against independent brute-force enumeration, a hand-verifiable unit-square case, the size and memory refusal guards, the heuristic's validity/never-worse-than-nearest-neighbor guarantee, and the factory's routing logic. (Full detail in the previous version of this report.)

**`test_model.cpp`** — pure data-shape tests: nodes carry the right type; `NodeRegistry` stores, retrieves, and correctly *rejects* a duplicate id (rather than silently overwriting); `CustomerData`/parameters store their fields; `DroneTrip`'s two structural-validity helpers correctly accept well-formed trips and reject every kind of ill-formed one (repeated endpoints, an unset trip, a trip "launching" from a customer node instead of a flex point, a trip referencing a node id that was never registered at all).

**`test_graph.cpp`** — Euclidean distance on a known 3-4-5 triangle, symmetry (`d(a,b) == d(b,a)`), zero self-distance, `distance / speed` travel-time arithmetic, the zero-speed-returns-infinity safety guard, and `SpatialNetwork`'s distance/time queries and matrix-building methods checked against the shared fixture's known distances.

**`test_flp.cpp`** — generated points actually lie on the traversed arc (checked via an *independently recomputed* linear interpolation, not by trusting the generator's own formula); no point is ever generated on an untraversed edge; ids are unique; generated coordinates match arc geometry; a full depot-loop route including the "return to depot" arc is handled the same as any other arc; the d-based count n = floor(D/d) (exact multiples, d longer than the arc, proportionality to distance) and the refusal to run without a configured d; repeated calls with identical inputs are perfectly reproducible (no hidden randomness); a zero-length arc (two nodes at the same coordinates, whether via a literally-repeated route entry or two distinct ids that happen to coincide) correctly produces zero points instead of a NaN or a crash; the opt-in fixed-count rule with `points_per_arc` of 3, 0, and a negative number behaves as documented.

**`test_timing.cpp`** (the second-largest file, ~700 lines) — this is worth understanding as a methodology, not just a list: essentially every expected number in this file is worked out by hand in a comment *before* the test code runs, using clean Pythagorean-triple geometry (3-4-5, 6-8-10, 8-15-17) chosen specifically so every distance is an exact, calculator-checkable number, and the test never calls `evaluate_solution()` (or any of its own internals) to produce its own "expected" side of a comparison — that would risk the test just checking the code agrees with itself. The blocks cover: simple multi-stop truck-only propagation; a full single-drone-trip timing chain including the case where the truck arrives at recovery before the drone (truck waits) and the reverse case (drone waits); the endurance feasibility boundary (the identical trip judged feasible under a generous budget and infeasible under a tight one); a trip with an invalid launch-node role being correctly rejected; theta's effect on endurance while leaving the *real* customer arrival time completely untouched; a mixed truck+drone two-trip solution with a fully hand-computed objective; a solution where one customer is served by nobody at all (correctly flagged infeasible with a specific reason); regression blocks A–K for the 29 Sep 2026 corrections (two launches at one launch point, endurance with Se^D and θ, a drone waiting for the truck, sequential reuse of one drone, relaunch before recovery, invalid drone ids, too many simultaneous drones, launch points used off their own arc or between unrelated nodes, an idle launch point, a repeated launch point); and a dedicated three-sub-case audit of what happens when a drone's own recovery arrival falls *before*, *during*, or *after* the truck's own service window at the same node — specifically pinning down that the implemented behavior is "both must finish, whichever is later" (`max()`) rather than "recovery can't even start until service is fully done" (which would give a different, larger number in the "during" sub-case).

**`test_init.cpp`** (the largest file) — the blocks below (Blocks 4, 6 and 8–10 were re-derived for the 29 Sep 2026 corrections, with test-fixture values of d, and Block 11 = test L checks FLP–arc membership and the other Phase F invariants directly; Block 12 checks that an unset d is refused), each with a fully hand-derived expected outcome: the `k = ceil(...)` formula on several inputs; K-means traced by hand for exactly two iterations on a simple 4-point line, plus an "obviously separated" sanity case and edge cases (empty input, `k` larger than the number of points, `k = 0`); the truck-only TSP route's known-optimal cost on a colinear layout; a customer too far from any launch point under a tight endurance budget correctly staying truck-served rather than getting an infeasible drone trip; confirmation that the timing engine's own on-route/feasibility check would correctly reject a recovery-before-launch trip if F4 ever proposed one (it structurally never does, but this checks the safety net anyway); a full end-to-end F1–F4 run on a small colinear instance with every downstream number — cluster assignments, generated FLP coordinates, which customers end up drone- vs. truck-served, and the final objective value — worked out by hand and matched; direct unit tests of the sweep-line drone-fleet-capacity check (touching-but-not-overlapping intervals, genuinely overlapping pairs, a staggered triple that never needs more than 2 drones at once even though there are 3 intervals); a full pipeline run where a *different* cluster's candidate is correctly rejected because accepting it would require 2 simultaneous drones from a 1-drone fleet; a full pipeline run showing the *opposite* — two different clusters' trips that don't overlap in time correctly share one physical drone sequentially; and a case specifically designed (by making the drone faster than the truck) to prove that *every* candidate launch point on a customer's arc is genuinely tried, not just the nearest one, because here the nearest one is provably the worse choice.

---

## 9. CMake — a tutorial through `CMakeLists.txt`

CMake doesn't compile anything itself — it reads a description of what you want built and *generates* the actual instructions for a real build tool (Make, Ninja, MSBuild...) to follow. Here's every line, in order, now that there are six test executables instead of one:

```cmake
cmake_minimum_required(VERSION 3.16)
project(truck_drone_dynamic_routing CXX)
```
Refuses to proceed on an older CMake than 3.16 (protects against relying on a feature that doesn't exist yet in an older install); names the project and declares the language as C++ (`CXX`).

```cmake
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
```
Compile as C++20; make that non-optional (fail loudly rather than silently downgrading); stick to the official standard rather than a compiler's own nonstandard extensions, for portability.

```cmake
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
```
`CMAKE_BUILD_TYPE` controls optimization: `Debug` is unoptimized and easy to step through in a debugger, `Release` is optimized. If nobody picked one, default to `Release` — sensible here given how numerically heavy the TSP/timing code is.

```cmake
add_library(tdr_core
    src/tsp/held_karp_solver.cpp
    src/tsp/heuristic_tsp_solver.cpp
    src/tsp/tsp_solver_factory.cpp
)
target_include_directories(tdr_core PUBLIC ${CMAKE_SOURCE_DIR}/include)
```
Declares a library target `tdr_core` from the only three `.cpp` files that currently exist (everything in `model/`, `graph/`, `flp/`, `timing/` is header-only, remember, so it needs no `.cpp` here at all — those modules exist purely as headers anyone can `#include`). `target_include_directories(... PUBLIC ...)` tells the compiler where to find headers, and `PUBLIC` means "and propagate this same include path to anything that links against `tdr_core`" — this is why every test `.cpp` below can write `#include "tsp/held_karp_solver.hpp"` without repeating the setup itself.

```cmake
enable_testing()
```
Turns on CTest, CMake's built-in test runner. Without this, none of the `add_test(...)` calls below would do anything.

Then six near-identical blocks, one per test executable — this is the actual new content since the last version of this report, and it's worth reading as a map of the project's own dependency graph:

```cmake
add_executable(test_tsp tests/test_tsp.cpp)
target_link_libraries(test_tsp PRIVATE tdr_core)
add_test(NAME test_tsp COMMAND test_tsp)
```
`test_tsp` needs the compiled `tsp/` code, so it links `tdr_core`.

```cmake
add_executable(test_model tests/test_model.cpp)
target_include_directories(test_model PRIVATE ${CMAKE_SOURCE_DIR}/include)
add_test(NAME test_model COMMAND test_model)
```
`model/` is header-only and depends on nothing else, so `test_model` only needs the include path — no library to link at all, since there's no compiled code to link against.

```cmake
add_executable(test_graph tests/test_graph.cpp)
target_include_directories(test_graph PRIVATE ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/tests)
add_test(NAME test_graph COMMAND test_graph)
```
Same idea, but `graph/`'s test also needs `tests/` itself on the include path, because it `#include`s the shared `fixtures/small_graph_fixture.hpp`.

```cmake
add_executable(test_flp tests/test_flp.cpp)
target_include_directories(test_flp PRIVATE ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/tests)
add_test(NAME test_flp COMMAND test_flp)
```
Same pattern again — `flp/` is header-only, builds only on `model/`, and its test also uses the shared fixture.

```cmake
add_executable(test_timing tests/test_timing.cpp)
target_include_directories(test_timing PRIVATE ${CMAKE_SOURCE_DIR}/include)
add_test(NAME test_timing COMMAND test_timing)
```
`timing/` is header-only too; its test builds its own tiny node layouts inline rather than reusing the shared fixture, so it doesn't need `tests/` on the include path.

```cmake
add_executable(test_init tests/test_init.cpp)
target_link_libraries(test_init PRIVATE tdr_core)
add_test(NAME test_init COMMAND test_init)
```
`init/` is the one module besides `tsp/` itself that actually calls into `tsp/` (it needs `TSPSolverFactory` for the F1 truck-only route), so `test_init` goes back to linking `tdr_core`, the same as `test_tsp` does.

`target_link_libraries(... PRIVATE ...)` versus `target_include_directories(... PRIVATE ...)`: `PRIVATE` here just means "this dependency is only needed to build *this* executable itself" — irrelevant in practice since nothing ever links against a test executable, but it's the conventional, correct choice for an executable target either way (as opposed to `PUBLIC`, which matters for libraries other things will link against, like `tdr_core` above).

**Building and running it**, unchanged from before:
```
mkdir build && cd build
cmake ..
cmake --build .
ctest --output-on-failure
```
`mkdir build && cd build` — CMake strongly prefers *out-of-source* builds, keeping every generated file in one throwaway folder instead of scattering them through your source tree. `cmake ..` reads `CMakeLists.txt` and generates the actual build files (it doesn't compile anything yet). `cmake --build .` invokes whatever underlying tool was generated to actually compile everything — six executables and one library, now. `ctest --output-on-failure` runs all six registered tests and only prints output for ones that fail, so a clean run stays quiet instead of drowning you in `[PASS]` lines.

Two things worth learning next, now that the project has real internal structure: `add_subdirectory` (once `src/`/`tests/` grow enough that one flat `CMakeLists.txt` gets unwieldy, you can split it into one per directory) and `target_compile_options` (to turn on stricter compiler warnings like `-Wall -Wextra`, which is generally a good idea for a project this size).

---

## 10. Complexity, and how to reduce it

This section carries forward from the previous report (the concern was specifically about `tsp/`'s exact solver), with brief notes on the new modules' own costs added at the end.

### Why Held-Karp is expensive

Held-Karp tracks `dp[mask][j]` — cheapest cost to have visited exactly the customer set `mask`, ending at `j` — over all `2ⁿ` possible subsets and all `n` possible current positions, trying all `n` possible next moves from each: **O(n² · 2ⁿ) time**, **O(n · 2ⁿ) memory** (two tables of that shape: the cost table and the parent/breadcrumb table for reconstructing the route). At `n = 21` (the current default cutoff) that's about 2.1 million subsets × 21 positions × 12 bytes/entry ≈ 504 MB, which is why the 4 GiB safety guard trips right around there; by `n = 30` the memory requirement is in the hundreds of gigabytes. This isn't sloppiness — TSP is NP-hard and no algorithm of this general shape is known to do fundamentally better in the worst case — but "worst case" leaves real room to do better in practice, which is what the options below exploit.

### Ranked options

| # | Change | Effort | What it buys |
|---|---|---|---|
| 1 | Drop the `parent` table; reconstruct the route by re-checking which predecessor produced each `dp[mask][j]`, rather than storing it upfront | Low | Cuts peak memory by roughly a third to a half |
| 2 | Use `float` instead of `double` for the cost table | Low | Halves that table's memory footprint |
| 3 | Add a lower bound (e.g. a minimum-spanning-tree/1-tree bound) and switch to **branch-and-bound** instead of exhaustive DP | Medium | Still exponential worst-case, but routinely solves 40–80 node instances in well under a second in practice, since most branches get pruned early; memory drops from `O(n·2ⁿ)` to essentially `O(n)` |
| 4 | Swap in a free ILP/MIP solver (Google OR-Tools' CP-SAT, or GLPK/CBC) with subtour-elimination constraints | Medium | This is what the paper's own CPLEX actually does internally — branch-and-cut over an integer-programming formulation, not a bitmask DP. A free solver routinely proves optimality for 50-city TSP in well under a second, which would let all three of the paper's instance sizes (10/20/50 customers) be solved exactly, closing the "ASSUMPTION-DEPENDENT" gap `DECISIONS.md` currently documents for the 50-customer case, without needing a CPLEX license. This is the single highest-leverage fix and slots directly into the existing `ITSPSolver` interface, exactly the way `DECISIONS.md` already anticipates for a future `CplexTSPSolver`. |
| 5 | Christofides algorithm as a stronger fallback than nearest-neighbor + 2-opt/Or-opt | Medium-High | A polynomial-time (`O(n³)`), *provably* ≤1.5×-optimal guarantee for metric distances, versus the current heuristic's "usually good, no guarantee" |
| 6 | Meet-in-the-middle DP (split nodes into two halves, DP each, combine) | High | Cuts memory to roughly `O(2^(n/2))`, but is genuinely fiddly to implement correctly — worth knowing exists, not a near-term recommendation |

**What I'd actually do:** #1 and #2 immediately (cheap, safe, free memory headroom); then #4, since it directly fixes both your stated concern (complexity) and the documented fidelity gap at the same time, without needing to hand-roll more search logic yourself. Keep the heuristic around regardless, as a fast fallback for pathological cases even an exact/ILP solver might choke on.

### The newer modules, briefly

None of `model/`, `graph/`, or `flp/` has any concerning complexity — they're all linear (or small-constant) in the number of nodes/arcs. `customer_clustering.hpp`'s K-means is `O(iterations × k × n)`, trivial at this project's scale (≤50 customers). `drone_fleet_capacity.hpp`'s sweep is `O(trips log trips)` for the sort, also trivial. The one place worth watching as instance sizes grow is `initial_solution_builder.hpp`'s F4 loop: for every candidate trip it considers, it builds a full trial `Solution` and calls `evaluate_solution()` (an `O(route length + trip count)` pass) on it from scratch — so one full construction run costs roughly *(number of accepted rounds) × (candidates considered per round) × (cost of one full evaluation)*. At 50 customers this is still fast in absolute terms, but if you ever push to substantially larger instances, incrementally updating a trial's timing instead of fully recomputing it from scratch on every single candidate would be the natural next optimization — not urgent now, but worth knowing about before it becomes the bottleneck.
