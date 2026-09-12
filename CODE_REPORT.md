# Code Report — `truck-drone-dynamic-routing` (TSP module)

This report walks through every file in the repository, explains what it does in plain language, and — since you asked — gives a serious answer on how to bring down the time and memory cost of the exact solver. Read it top to bottom once, then keep it as a reference while you re-read the code yourself; that's the fastest way to actually own it rather than just having it explained to you once.

## The big picture first

The whole module solves one problem: **given a set of delivery points (customers) plus a depot, find the shortest route that starts at the depot, visits every customer exactly once, and returns to the depot.** This is the classic Traveling Salesman Problem (TSP). It's used here as step (a) of a larger truck+drone routing method from a research paper — the paper solves this exact sub-problem with a commercial solver (CPLEX); this repo reproduces that step without CPLEX.

There are 9 code files. They split cleanly into three jobs:

| File | Job |
|---|---|
| `tsp_types.hpp` | Defines the shared "vocabulary" — what a distance matrix is, what an answer looks like. |
| `tsp_solver.hpp` | Defines the common shape ("interface") every solver must follow. |
| `held_karp_solver.hpp` / `.cpp` | The **exact** solver — guaranteed correct, but expensive. |
| `heuristic_tsp_solver.hpp` / `.cpp` | The **fast-but-not-guaranteed** solver — used when exact is too expensive. |
| `tsp_solver_factory.hpp` / `.cpp` | The "receptionist" — picks which of the above two to actually call. |
| `test_tsp.cpp` | Automated checks that all of the above actually work correctly. |
| `CMakeLists.txt` | The build recipe — tells the compiler how to turn all this into a program. |

Everything is written in C++ inside the namespace `tdr::tsp` (short for "truck-drone-routing :: tsp"), which is just a way of avoiding name clashes with other parts of the project later.

---

## `include/tsp/tsp_types.hpp` — the shared vocabulary

This file defines no logic at all — only two data shapes ("structs"). Think of it as the glossary everyone else in the module agrees to use.

```cpp
using DistanceMatrix = std::vector<std::vector<double>>;
```

This just gives a shorter name to "a grid of numbers." `DistanceMatrix[i][j]` is the distance from point `i` to point `j`. It's an n×n table where n = number of points (customers + depot). Point `0` is always the depot by convention — every solver in this repo assumes that.

```cpp
struct TSPResult {
    std::vector<int> tour;
    double cost = 0.0;
    bool exact = false;
    bool succeeded = false;
    std::string method_name;
    double solve_time_ms = 0.0;
    std::string note;
};
```

This is the answer packet every solver hands back. In plain terms:

- `tour` — the order to visit points in, e.g. `[0, 3, 1, 2]` means "start at depot, go to 3, then 1, then 2, then back to depot."
- `cost` — total distance of that route.
- `exact` — **true only if this is mathematically guaranteed to be the best possible route**, not just a good one.
- `succeeded` — false if the solver couldn't produce anything at all (e.g. it refused to run).
- `method_name` — which algorithm actually produced this, as a readable string.
- `solve_time_ms` — how long it took, for benchmarking.
- `note` — a plain-English caveat, e.g. "this is not exact, here's why."

Why bother with `exact` and `note` instead of just returning a route? Because later stages of the bigger project need to know whether they can trust this route as optimal or not. Baking that into the answer itself means no one downstream can forget to check.

---

## `include/tsp/tsp_solver.hpp` — the common shape

```cpp
class ITSPSolver {
public:
    virtual ~ITSPSolver() = default;
    virtual TSPResult solve(const DistanceMatrix& distances) const = 0;
    virtual std::string name() const = 0;
};
```

This is an **abstract interface** (the `I` prefix is a common convention for "interface"). It doesn't do any work itself — it's a contract that says "any TSP solver must have a `solve()` method that takes a distance matrix and returns a `TSPResult`, and a `name()` method that returns its own name." The `= 0` after each method means "there is no default implementation, every solver must supply its own."

Why is this useful? Because the rest of the code (and the tests) can write "give me *some* solver and call `solve()` on it" without caring whether it's the exact one or the fast one underneath. That's what lets `TSPSolverFactory` swap between them invisibly, and it's what would let a future `CplexTSPSolver` be dropped in later without changing anything else — it would just be a third class that follows the same shape.

---

## `HeldKarpSolver` — the exact solver

**Files:** `include/tsp/held_karp_solver.hpp` (declares it), `src/tsp/held_karp_solver.cpp` (implements it).

### What it's trying to do

Find the *provably* shortest route. The brute-force way to do this is to try every possible visiting order and keep the cheapest — but the number of orders explodes factorially ((n-1)! for n points), so even 15 customers would mean checking about 87 billion routes. That's not usable.

### The trick: dynamic programming (Held-Karp)

The Held-Karp algorithm is smarter. The core idea, in plain words:

> Instead of asking "what's the best full route?", repeatedly ask a smaller question: "what's the cheapest way to have visited *this specific group* of customers, ending at *this specific customer*?" Once you know the answer for small groups, you can build up the answer for bigger groups by reusing what you already computed — you never redo the same work twice.

To make "this specific group of customers" easy to store and look up, the code represents a group as a **bitmask**: a number where each bit says whether one customer is in the group. E.g. with 5 points, the number `0b00101` (binary) means "points 0 and 2 are in this group" (bit 0 = 1, bit 2 = 1, all others 0). This is a very common trick for small sets — it turns "is point x in this group?" into a single fast bit check, and there's a natural way to number every possible group from `0` to `2^n - 1`.

The header:

```cpp
class HeldKarpSolver : public ITSPSolver {
public:
    static constexpr int kDefaultMaxExactN = 21;
    static constexpr std::size_t kDefaultSafetyMemoryLimitBytes = 4ULL * 1024 * 1024 * 1024;

    explicit HeldKarpSolver(int max_exact_n = kDefaultMaxExactN,
                             std::size_t safety_memory_limit_bytes = kDefaultSafetyMemoryLimitBytes);
    TSPResult solve(const DistanceMatrix& distances) const override;
    std::string name() const override { return "HeldKarpExact"; }
private:
    int max_exact_n_;
    std::size_t safety_memory_limit_bytes_;
};
```

`: public ITSPSolver` means this class *is a kind of* `ITSPSolver` — it must (and does) implement `solve()` and `name()`. The two constructor arguments are safety limits, explained below. `max_exact_n_` and `safety_memory_limit_bytes_` are just where the constructor stores those two numbers for later use.

### Walking through `solve()` step by step

1. **Trivial cases first.** If there are 0 points, fail with a message. If there's exactly 1 point, the "route" is just staying at the depot — cost 0, done.

2. **Size guard.**
   ```cpp
   if (n > max_exact_n_) { ...refuse, succeeded = false... }
   ```
   If the instance is bigger than `max_exact_n_` (default 21), the solver **deliberately refuses to run** rather than silently taking hours or crashing. This is a good habit: failing loudly and explaining why beats hanging or running out of memory.

3. **Memory guard.**
   ```cpp
   const std::size_t estimated_bytes = full_mask_count * n * (sizeof(double) + sizeof(int));
   if (estimated_bytes > safety_memory_limit_bytes_) { ...refuse... }
   ```
   Before allocating anything, it calculates roughly how much RAM the tables below would need, and refuses if that's more than the configured limit (default 4 GiB). This is exactly why complexity matters here — see the dedicated section further down.

4. **Building the DP tables.**
   ```cpp
   std::vector<std::vector<double>> dp(full_mask_count, std::vector<double>(n, INF));
   std::vector<std::vector<int>> parent(full_mask_count, std::vector<int>(n, -1));
   dp[1][0] = 0.0;
   ```
   `dp[mask][j]` means: "the cheapest way to start at the depot, visit exactly the customers marked in `mask`, and end up currently standing at customer `j`." It's initialized to infinity everywhere except the starting state: mask = `1` (binary `...0001`, meaning only the depot is visited) and `j = 0` (currently at the depot) costs 0 — that's the starting point of every route. `parent[mask][j]` will remember *which customer we came from* to reach that best cost, so the actual route can be reconstructed afterward (like breadcrumbs).

5. **The DP transition — the heart of the algorithm.**
   ```cpp
   for (mask...) {
     if depot not in mask: skip
     for (j in mask with a known dp value) {
       for (k not yet in mask) {
         cand = dp[mask][j] + distances[j][k];
         if (cand < dp[mask | (1<<k)][k]) update it, remember j as parent
       }
     }
   }
   ```
   In words: for every group of visited customers, and every customer `j` we could currently be standing on, try moving to every unvisited customer `k`. If arriving at `k` this way is cheaper than any way found so far, remember it. This systematically builds every possible "group + current position" answer from smaller groups upward, and — critically — each `(mask, j)` combination is computed **once**, not once per route that happens to pass through it. That reuse is what turns "check every order" (factorial) into "check every group × every position twice" (still big, but far smaller — see below).

6. **Closing the loop back to the depot.**
   ```cpp
   for (j = 1..n-1) candidate = dp[full_mask][j] + distances[j][0];
   ```
   Once every customer has been visited (`full_mask` = all bits set), the algorithm checks which final customer `j`, when you add the trip back to the depot, gives the lowest total. That's the optimal tour's cost.

7. **Reconstructing the actual route.** It walks the `parent` breadcrumbs backward from that best final customer all the way back to the depot, collecting nodes, then reverses the list to get forward visiting order.

8. **Timing and packaging the result** — `std::chrono::steady_clock` measures elapsed time, and everything is written into the `TSPResult` (`exact = true`, since this route is mathematically the best possible).

### Why this is called "exact"

Because it doesn't guess — it exhaustively considers every reachable (group, current-position) state and always keeps the best. There is no approximation anywhere in the logic. The only thing that can stop it from running is the size guard, not a compromise in correctness.

---

## `HeuristicTSPSolver` — the fast, non-guaranteed solver

**Files:** `include/tsp/heuristic_tsp_solver.hpp`, `src/tsp/heuristic_tsp_solver.cpp`.

This exists purely as a fallback for when Held-Karp refuses (too many points). It trades the *guarantee* of optimality for *speed* — it will always give you *a* good route quickly, but never promises it's the shortest possible one. It always reports `exact = false`.

### The building blocks (all private helper functions in the `.cpp` file)

- **`tour_cost(d, tour)`** — adds up the distance of consecutive stops in a route, including the trip back to the start. A basic utility used everywhere else in the file.

- **`nearest_neighbor_tour(d, start)`** — the simplest possible route-building strategy: *"stand somewhere, always walk to the closest place you haven't visited yet, repeat until done."* It's fast (roughly n² work) but often leaves an obviously bad long "last stretch" back to the start, because it never looks ahead.

- **`rotate_to_start_at_zero(tour)`** — a bookkeeping helper. Since the depot must always be listed first by this project's convention, this just cyclically shifts the list of stops so node 0 is at the front again (it doesn't change the route, only where the list "starts" printing from).

- **`two_opt_pass(d, tour)`** — a well-known improvement trick called **2-opt**. Picture the route drawn on a map: if two of its path segments cross each other, you can almost always shorten the total route by "uncrossing" them — which turns out to just mean reversing the order of the stops between them. This function checks every pair of segments, and whenever swapping them (reversing that middle stretch) shortens the total distance, it does so. It repeats this until no such improvement can be found anymore.

- **`or_opt_pass(d, tour)`** — a second improvement trick called **Or-opt**. Instead of reversing a stretch, it tries picking up a small chunk of 1, 2, or 3 *consecutive* stops and re-inserting that chunk somewhere else in the route, if doing so is cheaper. Think of it as "maybe these two stops are better served right after a different stop instead of where they currently sit."

### `HeuristicTSPSolver::solve()`

1. Handles the same trivial 0/1-point cases as Held-Karp.
2. Runs several independent attempts ("restarts", default 8): the first attempt always starts building the route from the depot; the rest start from a random customer (using a seeded random number generator, so results are reproducible given the same seed).
3. For each attempt: build a nearest-neighbor route, then alternate applying `two_opt_pass` and `or_opt_pass` until neither one can improve the route any further, then re-check the cost.
4. Keep whichever attempt produced the cheapest route overall, and return that.

Multiple restarts exist because nearest-neighbor's quality depends a lot on where you start — trying several starting points and keeping the best hedges against a single unlucky choice.

---

## `TSPSolverFactory` — the decision-maker

**Files:** `include/tsp/tsp_solver_factory.hpp`, `src/tsp/tsp_solver_factory.cpp`.

This class doesn't solve anything itself — it just decides *which* of the two solvers above should run, in one single place, so that decision logic doesn't get duplicated or drift out of sync anywhere else in the codebase.

```cpp
TSPResult TSPSolverFactory::solve(const DistanceMatrix& distances) const {
    if (n <= max_exact_n_) {
        HeldKarpSolver exact(max_exact_n_);
        TSPResult r = exact.solve(distances);
        if (r.succeeded) return r;
    }
    HeuristicTSPSolver heuristic(heuristic_restarts_, heuristic_seed_);
    return heuristic.solve(distances);
}
```

In plain terms: "if the instance is small enough, try the exact solver; if that actually worked, use its answer. Otherwise (too big, or the exact solver refused for its own internal reasons), fall back to the heuristic." The comment in the source is careful to say this fallback only happens on an *explicit* refusal from Held-Karp, never on some silently swallowed error — so you can trust that if you get a heuristic result, it's because the exact solver deliberately declined, not because something broke quietly.

This is the "factory" design pattern: a class whose whole job is to construct and hand back the right object (or, here, the right *result*) based on some input condition, so calling code doesn't need to know the decision rule itself.

---

## `tests/test_tsp.cpp` — proving the above actually works

This is a small, dependency-free test program (no external testing library — just a `main()` that prints `[PASS]`/`[FAIL]` for each check and exits with a non-zero code if anything failed, which is what lets `ctest` detect success/failure).

Helper functions at the top:

- **`check(condition, description)`** — prints PASS or FAIL for one assertion and bumps a global failure counter on FAIL.
- **`check_near(a, b, tol, description)`** — same, but for comparing two floating-point numbers within a small tolerance (`tol`), which matters because floating-point arithmetic is never perfectly exact.
- **`random_symmetric_matrix(n, rng)`** — builds a random n×n distance table where distance(i,j) == distance(j,i), using a seeded random generator for reproducibility.
- **`brute_force_optimal(d)`** — an *independent*, deliberately dumb, definitely-correct TSP solver: it tries literally every ordering of customers (via `std::next_permutation`) and keeps the cheapest. This exists purely as ground truth to check Held-Karp against — if two independently-written implementations agree, that's strong evidence both are correct.
- **`is_valid_permutation_tour`** — checks a returned route actually visits every point exactly once (no duplicates, no missing points, no out-of-range indices).

What each test block checks:

1. **Held-Karp vs. brute force**, for n = 4 through 8: confirms Held-Karp's answer matches the dumb-but-trustworthy brute force exactly, that the returned route is a valid permutation, and that the reported cost matches what you get from actually re-adding up the route's distances.
2. **A known-by-hand answer**: 4 points placed at the corners of a unit square. Geometrically, the shortest route around a square is obviously its perimeter — cost exactly 4. This is a sanity check using an answer you can verify with pen and paper, independent of any code.
3. **The size guard actually refuses** when asked to solve something bigger than its configured limit.
4. **The memory guard actually refuses** when given an artificially tiny memory budget (1024 bytes), proving the guard triggers rather than just existing in comments.
5. **The heuristic solver returns something valid** — a real permutation, correctly reports itself as non-exact, and its reported cost matches recomputation.
6. **The heuristic is never worse than plain nearest-neighbor alone** — since the heuristic adds 2-opt/Or-opt improvement on top of nearest-neighbor, this checks that the "improvement" step never accidentally makes things worse.
7. **The factory routes correctly**: with a small threshold configured, it checks a small instance gets routed to Held-Karp (and is marked exact) and a larger one gets routed to the heuristic (and is marked not exact).

This is a genuinely solid test suite for what it covers — it checks correctness against an independent method, checks a hand-verifiable case, and checks the safety guards actually fire, not just that happy-path code runs.

---

## `CMakeLists.txt` — the build recipe, explained line by line

CMake doesn't compile your code itself — it *generates* the actual build instructions (for `make`, Ninja, Visual Studio, etc.) from a description of what you want built. Here's what each line in this file means.

```cmake
cmake_minimum_required(VERSION 3.16)
```
Refuses to proceed if whoever is building this has an older CMake than 3.16 — protects you from using a feature that doesn't exist in an older version without realizing it.

```cmake
project(truck_drone_dynamic_routing CXX)
```
Names the overall project, and tells CMake the language in use is C++ (`CXX`). This sets up some project-wide variables CMake uses internally.

```cmake
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
```
Three settings that together say: "compile as C++20" (the language standard version, which enables newer syntax/features), "and this is not optional — fail loudly if the compiler can't do C++20" rather than silently falling back to an older standard, "and don't use compiler-specific extensions" (stick to the official standard, for portability across compilers).

```cmake
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
```
`CMAKE_BUILD_TYPE` controls optimization level. `Debug` builds are unoptimized and easy to step through in a debugger; `Release` builds are optimized (faster, but harder to debug). This says "if the person building didn't pick one, default to Release" — sensible for a numerically-heavy project like this one where an unoptimized Held-Karp DP would be painfully slow.

```cmake
add_library(tdr_core
    src/tsp/held_karp_solver.cpp
    src/tsp/heuristic_tsp_solver.cpp
    src/tsp/tsp_solver_factory.cpp
)
```
Declares a **library target** named `tdr_core`, built from those three `.cpp` files. A "library" here just means "a reusable bundle of compiled code" (as opposed to `add_executable`, which produces a runnable program). Since no `STATIC`/`SHARED` keyword is given, CMake defaults to a static library — the compiled code gets baked directly into whatever links against it, rather than being a separate `.dll`/`.so` loaded at runtime.

```cmake
target_include_directories(tdr_core PUBLIC ${CMAKE_SOURCE_DIR}/include)
```
Tells the compiler "when compiling `tdr_core`, look in the `include/` folder for headers" (that's why the `.cpp` files can write `#include "tsp/held_karp_solver.hpp"` without a longer path). `PUBLIC` is important: it means "not just `tdr_core` itself needs this include path — anything that links against `tdr_core` inherits it too." That's exactly why `test_tsp.cpp` (below) can `#include "tsp/held_karp_solver.hpp"` without repeating this include-path setup itself.

```cmake
enable_testing()
```
Turns on CMake's built-in test-running system (CTest) for this project. Without this line, `add_test` below would be silently ignored.

```cmake
add_executable(test_tsp tests/test_tsp.cpp)
target_link_libraries(test_tsp PRIVATE tdr_core)
add_test(NAME test_tsp COMMAND test_tsp)
```
`add_executable` declares an actual runnable program named `test_tsp`, built from `tests/test_tsp.cpp`. `target_link_libraries` says "this program needs the compiled code from `tdr_core`" — `PRIVATE` means "this dependency is only for building `test_tsp` itself, it wouldn't propagate further" (irrelevant here since nothing links against `test_tsp`, but it's the correct/conventional choice for an executable). `add_test` registers this executable with CTest under the name `test_tsp`, so running `ctest` will execute it and interpret its exit code (0 = pass, non-zero = fail — which is exactly why `test_tsp.cpp`'s `main()` returns `g_failures == 0 ? 0 : 1`).

### What actually happens when you build

```
mkdir build && cd build
cmake ..
cmake --build .
ctest --output-on-failure
```

- `mkdir build && cd build` — CMake strongly prefers **out-of-source builds**: all generated files (object files, the final binaries, CMake's own cache) go into a separate `build/` folder, keeping your source tree clean. This is a convention worth keeping.
- `cmake ..` — reads `CMakeLists.txt` from the parent folder and *generates* the actual build files (e.g. Makefiles, or a Ninja build file) inside `build/`. This step does not compile anything yet — it just works out the plan.
- `cmake --build .` — invokes whichever underlying build tool was generated (make/ninja/MSBuild) to actually compile everything. This is a CMake wrapper so the same command works regardless of what's installed underneath.
- `ctest --output-on-failure` — runs every test registered with `add_test` (here, just `test_tsp`) and, importantly, only prints a failing test's output (normally CTest is quiet about passing tests) — handy so you're not drowning in `[PASS]` lines when everything's fine.

If you want to learn more CMake from here, the next useful things to look up are `add_subdirectory` (for splitting a growing project into per-folder `CMakeLists.txt` files), `find_package` (for pulling in external libraries, which you'll need the moment you add something like an ILP solver — see below), and `target_compile_options` (for turning on stricter warnings, e.g. `-Wall -Wextra`).

---

## Complexity: where the cost is actually coming from, and how to cut it

You're right to be worried about this — it's worth being precise about *why* it's expensive before picking a fix.

### Why Held-Karp is expensive

- **Time:** for every one of the `2^n` possible groups of visited customers, for every possible "currently standing at customer `j`" (up to `n` choices), it tries extending to every unvisited customer `k` (up to `n` choices). That's `2^n × n × n` = **O(n² · 2ⁿ)**.
- **Memory:** it needs to remember the best cost *and* the parent breadcrumb for every (group, current-customer) pair — two tables, each sized `2^n × n`. That's **O(n · 2ⁿ)**.

Concretely, with `n = 21` (this repo's default cutoff): `2²¹ ≈ 2.1 million` groups, times 21 customers, times (8 bytes for the cost + 4 bytes for the parent index) ≈ **504 MB** — which is exactly why the 4 GiB safety guard is set where it is, and why it would refuse well before `n = 30` (`2³⁰ ≈ 1.07 billion` groups — this would need roughly **270 GB**, not remotely practical on a normal machine).

This isn't a bug or a sloppy implementation — Held-Karp genuinely is the best known *general-purpose exact* algorithm of this simple shape, and TSP is NP-hard, so no algorithm is known (or, under a widely-believed complexity-theory conjecture called the Exponential Time Hypothesis, is even *possible*) that solves it exactly in polynomial time in the worst case. But "worst case" is the key phrase — real-world instances are usually far more forgiving than the worst case, and that gap is exactly what the fixes below exploit.

### Options, from easiest to most impactful

| # | Change | Effort | What it buys you |
|---|---|---|---|
| 1 | Drop the `parent` table; reconstruct the route afterward by re-checking which predecessor produced each `dp[mask][j]` value, instead of storing it upfront | Low | Cuts peak memory by roughly a third to a half (removes the whole `int` table, keeps only the `double` cost table) |
| 2 | Use `float` instead of `double` for the cost table | Low | Halves the cost table's memory (4 bytes vs 8 bytes per entry); usually fine for routing distances, but watch for accumulated rounding error on very long routes |
| 3 | Add a **1-tree / minimum-spanning-tree lower bound** and turn this into **branch-and-bound** instead of a full DP table | Medium | This is the big one. Instead of computing *every* (group, position) state, you explore partial routes and immediately abandon (prune) any partial route whose lower-bound cost already exceeds the best complete route found so far. In the worst case it's still exponential, but in practice it routinely solves 40–80 city instances in well under a second, because most branches get pruned early. Memory drops from `O(n·2ⁿ)` to essentially `O(n)` (just the recursion stack), since you're no longer storing every possible state, only the current path. |
| 4 | Swap in a real **ILP / MIP solver** (e.g. Google OR-Tools' CP-SAT, or open-source GLPK/CBC) formulated with subtour-elimination constraints | Medium (mostly a learning curve, not much code) | This is actually the closest match to what the paper's own method does — CPLEX doesn't run Held-Karp internally either; it solves an integer-programming formulation with branch-and-cut. A free solver like OR-Tools routinely proves optimality for 50-city (or much larger) TSP instances in well under a second. This would let you solve **all three of the paper's instance sizes (10/20/50 customers) exactly**, closing the "ASSUMPTION-DEPENDENT" gap that `DECISIONS.md` currently flags for the 50-customer case — without needing a CPLEX license at all. Given `DECISIONS.md` already anticipates dropping in a `CplexTSPSolver` behind the same `ITSPSolver` interface later, an OR-Tools-based solver is a drop-in replacement for that exact slot. |
| 5 | **Christofides algorithm** as a stronger fallback than the current nearest-neighbor + 2-opt/Or-opt heuristic | Medium-High (needs a minimum-weight perfect matching routine, which is the fiddly part) | Unlike the current heuristic, Christofides comes with a mathematical guarantee: for metric distances (like straight-line/Euclidean distances, which routing usually is), its answer is never more than 1.5× the true optimum. Runs in polynomial time, `O(n³)`. Worth doing only if you want a *provable* quality bound rather than just "seems to work well in practice." |
| 6 | Meet-in-the-middle DP (split customers into two halves, DP each half, combine) | High | Brings memory down to roughly `O(2^(n/2))` instead of `O(2ⁿ)`, letting exact solves reach noticeably larger n. This is a real, published technique, but it's genuinely fiddly to implement correctly and is probably more effort than this project needs right now — mentioning it for completeness, not as a near-term recommendation. |

### What I'd actually do, in order

1. **Do #1 and #2 immediately** — they're cheap, low-risk, and roughly halve memory use for free, buying you a couple more usable values of `n` with zero algorithmic change.
2. **Then do #4** — bring in an open-source ILP solver (OR-Tools is the easiest to get going with and has a permissive license). This is the option that actually solves your stated problem (reducing complexity) *and* improves fidelity to the paper at the same time, since it would make the 50-customer case exact instead of a documented approximation. It slots into the existing `ITSPSolver` interface exactly the way `DECISIONS.md` already planned for a future CPLEX solver — you'd write an `OrToolsTSPSolver` class, register it in `TSPSolverFactory`, and nothing else in the codebase needs to change.
3. **Keep the current `HeuristicTSPSolver` around regardless** — every exact approach (Held-Karp, branch-and-bound, or an ILP solver) can still, in principle, take a long time on a pathological instance, so having a fast fallback with a time budget is good practice, not wasted work.
4. Treat #3 (branch-and-bound) and #5 (Christofides) as things to reach for only if you specifically want to avoid taking on an external solver dependency and are willing to write more of the search logic yourself — they're valid, well-understood techniques, just more implementation work for a smaller win than #4.

---

## Quick self-check on whether you actually understand this now

A good test of whether this has actually sunk in: try explaining, without looking at the code, why `dp[mask][j]` needs *both* a group of visited customers *and* a current position — why isn't the group alone enough? (Answer: because the cost to continue depends on where you currently are, not just who you've already visited — two different "last customer" choices for the same group of visited customers can lead to very different costs going forward, so both have to be tracked together.) If you can answer that in your own words, you understand the core trick of the whole exact solver.
