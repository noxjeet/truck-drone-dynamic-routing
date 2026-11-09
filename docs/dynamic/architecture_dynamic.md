# Dynamic extension — architecture specification (DM2)

Branch `dynamic/dm0-design` (base: tag `static-baseline-m10`, `a0112ce`). Inputs: `claude/dm0-design-specification.md` v3.1, `docs/dynamic/math_spec_dynamic.md` (DM1, accepted). **This is a specification. No source, build configuration or static file is changed by DM2; nothing described here is implemented yet.**

## 0. Status conventions

* **[APPROVED]** a DM0 decision D1–D15 (user-supplied decisions of the DM0 task).
* **[PROPOSED]** a default carried from DM0/DM1 that the user has *not* individually confirmed (U1–U8, Q2, Q6, Q10, Q11, Q12, E7, Q-F). The architecture is built so that each of these is a **swappable policy or a config value**, never a hard dependency.
* **[ARCH]** an architectural choice made in this document (new in DM2; open to review).
* **[MEASURE]** an assumption that DM3 must measure before relying on it.
* Facts about the static code are read from the headers named in each row; they are not re-audited.

## 1. Architectural principles

1. **Static code is frozen and only consumed.** The dynamic layer `#include`s static headers and never edits them. Static code never includes `dynamic/`. A test (T-dep, §9) greps for violations. [ARCH]
2. **One feasibility/objective authority:** `tdr::timing::evaluate_solution`. The dynamic layer has no second evaluator, timer or feasibility check; its own predicates (cut, Φ, scan region) are *necessary conditions or bookkeeping* around it. [APPROVED via DM0 §9.1; math_spec §2–§3]
3. **Plans are immutable values.** An installed plan is `Solution` + its cached evaluator result + the world it lives in. Decisions build candidate plans by copy and install atomically. [ARCH]
4. **Worlds are immutable snapshots; a tentative world is a copy.** Commit = adopt the copy; reject = drop it; no counter or global state to roll back. [APPROVED D8 / DM0 §10.4]
5. **Determinism by construction:** scenario draws are policy-independent; policy-side seeds are keyed by request index; ids are content-addressed. [APPROVED D8]
6. **Policies are strategy objects behind narrow interfaces** (scan, admission, region, local optimiser, base plan), so each experimental arm is a configuration, and each [PROPOSED] default is replaceable without touching the simulator. [ARCH]
7. **Header-only, `namespace tdr::dynamic`, same style as the static tree** (the static build is header-only apart from the TSP sources). [ARCH, follows repo convention]

## 2. Layers, modules and dependencies

```
L0  STATIC (frozen, read-only)    model/*  graph/*  flp/*  timing/*  init/*  search/*  instance/*  config/*  runner/result_writer.hpp (JsonWriter, fnv1a64_hex)  util/rng.hpp
        ▲
L1  CORE        ids  ·  world  ·  plan  ·  cursor  ·  invariants
        ▲
L2  DOMAIN      scenario  ·  scan_region  ·  promise  ·  inserter  ·  region_builder
        ▲
L3  OPTIMISER   scope_mask  ·  local_optimizer  ·  certificate        (needs DM8 hook or fork)
        ▲
L4  ORCHESTRATION  admission  ·  simulator  ·  config  ·  writer  ·  runner  ·  apps/tdr_dynamic.cpp
```
Rule: a module includes only its own layer's earlier modules and lower layers. No cycles; the simulator is the only module that knows the event loop.

| Module (`include/dynamic/`) | Responsibility (one sentence) | Depends on (static / dynamic) | Test file | Milestone |
|---|---|---|---|---|
| `ids.hpp` | encode/decode content-addressed customer and FLP ids; assert bounds | `model/node` / — | `test_dyn_ids.cpp` (T9) | DM3 |
| `world.hpp` | immutable world snapshot (nodes, customers, FLP data, arc index, network) and tentative extension | `graph`, `flp`, `model`, `instance` / `ids` | `test_dyn_world.cpp` | DM3 |
| `plan.hpp` | `InstalledPlan`: `Solution` + cached `SolutionEvaluationResult` + world handle + version | `timing`, `model` / `world` | `test_dyn_plan.cpp` | DM3 |
| `cursor.hpp` | clean-cut computation, committed sets, skeleton/gap queries on a plan | `search/route_ops` (`make_skeleton`, `gap_of_route_position`) / `plan` | `test_dyn_cursor.cpp` (T7, T1) | DM3 |
| `invariants.hpp` | I1–I7 checker returning a structured verdict | `timing` / `plan`, `cursor` | with `test_dyn_cursor.cpp` | DM3 |
| `scenario.hpp` | generate the exogenous request stream (uniform/Poisson first) with request-index ids | `util/rng` (new `SeedStream` values, see §6) / `ids` | `test_dyn_scenario.cpp` (T8) | DM4 |
| `scan_region.hpp` | `ScanRegion` interface; G0 (all), later G3/G3c/G1 and G2′ ground truth | `model` / `plan`, `cursor` | `test_dyn_scan.cpp` (T14) | DM5 (G0 in DM6) |
| `promise.hpp` | `PromiseLedger` (P_i, class, E, W), predicate Φ, Φ-chain helper | `timing` records / `plan` | `test_dyn_promise.cpp` (T11, T12, T15) | DM5 |
| `inserter.hpp` | enumerate gaps ≥ cut, build candidate plans, evaluator-verify, cheapest (B1) | `search/route_ops` / `plan`, `cursor`, `world` | `test_dyn_inserter.cpp` (T3, T4) | DM6 |
| `region_builder.hpp` | R0–R5 regions, closure, `ScopeMask` data | `search/route_ops` / `plan`, `cursor` | `test_dyn_region.cpp` | DM7 |
| `scope_mask.hpp` | scope-mask data type + (hook) the static-change gate artefacts | `search` / — | `test_dyn_scope.cpp` (G0–G6) | DM8 |
| `local_optimizer.hpp` | run static `run_vns` on the composite plan under a scope; budget; RIC-C loop | `search/vns` / `region_builder`, `scope_mask`, `world` | `test_dyn_opt.cpp` (T13) | DM9 |
| `admission.hpp` | `AdmissionPolicy`: A0, A-PM; Φ chain (U1) | / `promise`, `inserter` | with `test_dyn_promise.cpp` | DM5/DM6 |
| `simulator.hpp` | event loop, FIFO queue, tentative/commit, outcome partition | everything above | `test_dyn_sim.cpp` (T11) | DM6 |
| `config.hpp` | `DynamicConfig` (+ validation) | `config/experiment_config` pattern / — | `test_dyn_config.cpp` | DM6 (flags) → DM10 (file) |
| `writer.hpp` | `tdr.dynamic.results/1` writer, manifest | `runner/result_writer.hpp` (`JsonWriter`, `fnv1a64_hex`) / `simulator` | `test_dyn_writer.cpp` | DM6 (min) → DM10 |
| `runner.hpp`, `apps/tdr_dynamic.cpp` | build base plan, loop over scenarios, write outputs, CLI | `instance`, `init`, `search/vns`, `tsp` / all | `test_dyn_runner.cpp` | DM6 (min) → DM10 |

New `SeedStream` enumerators (E5) are the only edit that touches a static header (`util/rng.hpp`); it is additive (existing values 1–5 unchanged) and is performed **only at DM4, with the static regression suite re-run** [APPROVED E5]. Until DM4 no static file is touched.

## 3. Core data structures

Sketches, not final code. All new types live in `tdr::dynamic`.

```cpp
// ids.hpp  [APPROVED D8; math & bounds in DM0 §10.2]
inline constexpr NodeId kC0 = 1u << 24;            // dynamic customer id = kC0 + request_index
inline constexpr NodeId kF0 = 1u << 27;            // dynamic-arc FLP band
struct DenseIndex { int n0; int M_max; int K() const; };         // depot 0, base i->i, request r->n0+r
NodeId customer_id(std::uint32_t r);               std::optional<std::uint32_t> request_index(NodeId id);
NodeId flp_id(int iota_a, int iota_b, int m, int K);              // asserts m in 1..64 and id < 2^31

// scenario.hpp  [APPROVED D1, D2, D8; spatial/arrival defaults PROPOSED (Q11)]
struct Request { std::uint32_t index; double time; Point2D location; double se1, se2, theta; /*mark u: reserved, not drawn*/ };
struct Scenario { std::uint64_t seed; int sigma; double horizon_TA; std::vector<Request> requests; /*sorted by (time,index)*/ std::string digest; };

// world.hpp
struct World {                                     // immutable once built; shared_ptr<const World>
    NodeRegistry nodes; std::unordered_map<NodeId,CustomerData> customers; FlpDataMap flp;   // evaluator inputs
    FlpArcIndex flp_index; SpatialNetwork network; TruckParameters truck; DroneParameters drone;
    SearchContext context(int adjacency_window) const;   // references members: valid only while the World lives
    std::shared_ptr<const World> with_customer(const Request&) const;        // tentative copy: +1 customer node (+CustomerData)
    std::shared_ptr<const World> with_pool_extension(const Request&, const FlpGenerationPolicy&) const; // adds FLPs for arcs with the new node (DM7+, not in the vertical slice)
};

// plan.hpp
struct InstalledPlan {
    Solution solution; SolutionEvaluationResult eval;      // evaluated once at construction by the unchanged evaluator
    std::shared_ptr<const World> world; double installed_at; std::uint32_t version;
};                                                          // invariant: eval.feasible

// cursor.hpp  [math_spec §2: clean cut; P1/P2]
struct Cut { std::size_t route_pos; std::size_t skeleton_index; double node_time; };   // B and t_B
std::optional<Cut> clean_cut(const InstalledPlan&, double theta /*= s + omega*/);
//   B = smallest skeleton position with arrival >= theta, then advanced while some trip has launch_pos <= B < recovery_pos.
//   nullopt  <=>  no editable gap (B would be the end depot).   Committed set = customers at positions <= B and customers of trips launched <= B.
struct Gap { std::size_t a_pos, b_pos; };                  // consecutive skeleton nodes, a_pos >= cut
std::vector<Gap> editable_gaps(const InstalledPlan&, const Cut&);
bool is_straddled(const Solution&, const Gap&);            // math_spec §2

// promise.hpp  [APPROVED D4, D5; numeric W/E grid PROPOSED/calibrated in DM11]
enum class Klass : std::uint8_t { Base, New };
struct Promise { NodeId customer; Klass klass; double first_promised_eta; /*P_i*/ double tau_arrival; };
class PromiseLedger { /* no ratchet by default (D5); counts banked-slack uses */ };
bool phi(const InstalledPlan& candidate, const PromiseLedger&, const PromiseParams&, NodeId new_customer, std::string* why);

// outcome partition  [APPROVED D3, D9; DM0 §6.3(6)]
enum class Outcome : std::uint8_t { OutOfScan, Expired, Stale, RejectedNoInsertion, RejectedPromise, Accepted };
struct DecisionRecord { std::uint32_t request; Outcome outcome; double arrival, start, install; std::optional<Cut> cut;
                        double delta_F; int evaluator_calls; /* later: region size, certificate outcome, cap counters */ };
```

Notes that follow from the code, not from the design:
* `SearchContext` stores **references** to network/customers/flp data/index (move_types.hpp:38–43), and `SpatialNetwork` owns a **copy** of the `NodeRegistry` (spatial_network.hpp:42). Hence `World` owns all four and hands out a `SearchContext` view; a World must never be moved while a context exists (it is held by `shared_ptr`). [read from source]
* The base world is the static eager pool: depot + customers `1..n0` + all FLPs (ids from `max customer id + 1`; ~37 606 points at n=50). Static ids are far below `kC0`, so the bands are disjoint. [STATIC, DM0 §10.2]
* A tentative world copies an `unordered_map` of ≈39 k nodes and rebuilds `FlpArcIndex` and the `SpatialNetwork` customer cache. Its cost at n=50 is **[MEASURE]** in DM3 (DM0 assumed ms-scale). If it is too slow, the documented fallback is to share the base pool by pointer and overlay dynamic nodes (an internal change to `World`; no interface change).

## 4. Interfaces (narrow, each with an independent test)

```cpp
struct ScanRegion      { virtual bool contains(Point2D, const InstalledPlan&, const Cut&) const = 0; };  // G0: always true. G3 etc. DM5
struct AdmissionPolicy { virtual Decision decide(DecisionInput&) = 0; };            // A0 (feasible ⇒ accept) / A-PM (Φ chain, U1 [PROPOSED])
struct BasePlanBuilder { virtual InstalledPlan build(const StaticConfig&, int dataset) = 0; };   // StaticVns (main, D10) | Constructive (ablation)
struct RequestInserter { virtual std::optional<Candidate> insert(const InstalledPlan&, const Cut&, const Request&, const World& tentative) = 0; }; // B1 first
struct RegionBuilder   { virtual Region build(const Candidate&, const Cut&) = 0; };                // DM7
struct LocalOptimizer  { virtual Candidate optimise(const Candidate&, const Region&, Budget, Seed) = 0; };       // DM9: run_vns under a ScopeMask; RIC-C
```
Decision pipeline (inside the simulator): `ScanRegion` at arrival → queue → `clean_cut` → `RequestInserter` → (`RegionBuilder` → `LocalOptimizer`) → `AdmissionPolicy` → install. Every arm of the experimental design (DM0 §12) is one choice per interface; the simulator contains no arm-specific code.

## 5. Control flow

The simulator implements the event loop of DM0 §11 unchanged. Architecture-level points:

1. **Three times** per request: arrival τ, decision start s ≥ τ, install s+ω; FIFO sequential decisions; a decision uses the plan installed when the previous decision finished. [APPROVED D9]
2. **Scan eligibility is evaluated once, at arrival, against the last installed plan; out-of-scan ⇒ lost, never queued.** [APPROVED D3, D9]
3. At start: `clean_cut(plan, s+ω)`; if `nullopt` ⇒ `RejectedNoInsertion` (logged `Stale` if the request's core gaps are behind the cut). `Expired` if the PM1 window cannot be met (no optimiser time consumed).
4. Tentative world `W' = plan.world->with_customer(r)`; candidate plans are evaluated by the unchanged evaluator against `W'`.
5. Invariants I1–I6 are checked on every candidate before install (I7 = Φ is a policy outcome, not a bug). A violation of I1–I6 aborts the decision, rolls back, and flags the run. [APPROVED DM0 §4.7]
6. Install = replace `InstalledPlan` (new version) atomically; the new cut ≥ the old cut, so installed history never changes. Final plan is re-evaluated at the end of the run.
7. No wall-clock quantity influences a decision in reproducible mode; `ω` is simulated time and the optimiser pass budget is a deterministic function of ω (Q6 [PROPOSED]).

## 6. Determinism, ids, seeds

* Customer id of request r = `kC0 + r`; dynamic-arc FLP ids per DM0 §10.2 (`n_ab ≤ 64` as a config-validator assertion, Q-F [PROPOSED]). Base-pool FLPs keep static ids.
* Seeds: `derive_seed(derive_seed(base, DynVns, σ), DynVns, r)` with r the **request index**; likewise `DynShake`, `DynPolicy`; scenario streams `Arrivals`, `Locations`, `Clusters`; `Marks` reserved and **not drawn in v1**. New enumerators are appended to `SeedStream` (E5; existing values 1–5 untouched).
* Golden digest of each scenario (FNV-1a via the existing `fnv1a64_hex`) pins replay across platforms (T8).

## 7. Extension points (nothing below is built before its milestone)

| Point | Mechanism | Gate |
|---|---|---|
| scope-restricted search | `ScopeMask` in `SearchContext` via an optional pointer (E1–E3) | only if **G0–G6 all pass** (D12); otherwise a `dynamic::` fork of operators and shakes (≈1 750 lines) behind the same `LocalOptimizer` interface. The architecture does **not** depend on which is chosen. |
| N4 re-anchoring in the dynamic module | re-implemented from public `route_ops` primitives, differential-tested against N4 (E7 [PROPOSED]) | DM7/DM9 |
| certificate cap counters | `N4Options::max_reanchor_variants`, `CandidateRecord::cap_reached`, `OperatorResult::cap_reached_candidates`; **any non-zero cap count ⇒ `budget-stop`** (D7; math_spec §8) | DM9 |
| proximity-attracted demand | dominated thinning with marks from the reserved stream (D2) | deferred beyond DM12 |
| `FrozenSpanning` cut | alternative `clean_cut` implementation | v2, deferred |

## 8. Configuration and result schema

* **Config:** new `DynamicConfig` with a `dyn.` key prefix and strict validation, in the same `key = value` / single field-table style as `tdr::config::ExperimentConfig` (Q10 [PROPOSED]: separate `tdr_dynamic`, config and schema). The static 48-key table and `tdr.static.results/2` are **not** extended. A dynamic file embeds the static keys by routing `dyn.*` lines to the dynamic parser and the rest to the static parser (the static parser rejects unknown keys, so splitting is required; DM10 detail). Until DM10 the vertical slice takes `--key=value` flags.
* **Key groups:** scenario (ρ, ζ, spatial model, arrival profile, M_max), regime (`P`|`F`; F doubles endurance only, math_spec §7), latency ω, scan policy, admission (`A0`|`A-PM`, W̃, ε̃, class split, ratchet), base plan (`static-vns`|`constructive`), local strategy/region arm (default `none` in the slice), budgets, seed(s).
* **Schema `tdr.dynamic.results/1` (draft):** `scenario` {parameters, digest}; `base_plan` {objective, makespan, reference T0}; `requests[]` {index, time, x, y, outcome, decision start/install, cut, ΔF, promised ETA, final ETA, class}; `decisions[]` {request, cut, evaluator calls, later: region size, certificate outcome, cap counters, out_of_scope_skipped}; `summary` {accepted/generated, accepted/considered, response-time and queue statistics, final ΣETA, makespan, base-customer ETA drift}; `provenance` {tool version, tag `static-baseline-m10`, config echo, file digests}. Outcome partition exhaustive and exclusive (T11).

## 9. Build and test layout (changes require approval; none made now)

* **New CMake option `TDR_BUILD_DYNAMIC`, default OFF**, adding `tdr_dynamic` and `test_dyn_*` targets. With the option OFF the static build graph is byte-identical to the baseline. **This edit to `CMakeLists.txt` is not made in DM2 and needs explicit approval at DM3** (see blocker B1).
* Tests (all new files): the T-numbers in §2; plus **T-dep** (no static file includes `dynamic/`; no dynamic file edits static headers) and **T-reg** (static suites unchanged and rerun only at DM4's `SeedStream` edit and at DM8's gates).
* The DM1 probes (`bench/dm1_probes.cpp`) stay as probes; their cases become regression tests (T1–T6) in `test_dyn_*` at DM3–DM6 with their negative controls.
* O1 (undiagnosed T2 negative controls) is closed inside DM3's T1/T7 work (math_spec §11).

## 10. Minimal vertical slice (the end-to-end demo)

**Goal:** one command runs a static-VNS base plan, a Poisson/uniform request stream, FIFO decisions with latency ω, truck-only cheapest insertion verified by the unchanged evaluator, A0 admission, installation under the clean cut, and writes results — with I1–I6 checked at every installation and the outcome partition exhaustive.

**In the slice:** `ids` (encoder + T9), `world` (`with_customer` only, no pool extension), `plan`, `cursor`, `invariants`, `scenario` (uniform + homogeneous Poisson, fixed-count), `scan_region` G0, `inserter` B1, `admission` A0 (feasibility only), `simulator`, a minimal `writer`/`runner`/CLI (flags), `BasePlanBuilder::StaticVns` and `::Constructive`.
**Not in the slice (and not needed for it):** G3/G3c/G1/G2′, Φ and PM1/PM2 (A-PM), FLP pool extension, region builder, scope hook (DM8), local optimiser and certificate (DM9), clustered/corridor demand, regime F experiments, config file. These come later without changing any interface above.

Why this suffices: B1 evaluates each candidate gap with the evaluator, so stale launch FLPs on a split arc (CE-2) simply yield infeasible candidates and are skipped, and spanning-trip infeasibility (CE-1) is likewise caught; the slice therefore needs no extra mathematical machinery beyond the clean cut (P1). Under paper parameters 357/360 static solutions are drone-free [STATIC], so most slice runs exercise the truck-only path; regime F runs are possible in the slice but will often yield no feasible insertion in gaps carrying trips.

**Acceptance (slice):** (a) installed prefix identical across installs (I2) in all runs; (b) I1, I3–I6 pass; (c) every request has exactly one outcome; (d) replay of the same (config, σ) is bit-identical (digest); (e) changing the policy (G0↔reject-all B0) leaves the scenario digest unchanged (T8); (f) the final plan re-evaluates feasible with the stated ΣETA; (g) DM1 probe cases T1/T2 pass as regression tests in the new harness.

## 11. Minimum implementation sequence to the demo

| Step | Content | Exit test | Notes |
|---|---|---|---|
| S0 (approval) | approve `TDR_BUILD_DYNAMIC` CMake option and the `SeedStream` append | user confirmation | blocker B1; nothing else waits on it except building |
| S1 = DM3 | `ids`, `world`, `plan`, `cursor`, `invariants`; measure tentative-world cost; close O1 | T9, T7, T1(+negatives), world copy equality; cost report | no simulator yet |
| S2 = DM4 (slice part) | `scenario` uniform/Poisson/fixed-count; `SeedStream` append; digest | T8; static suites re-run once | Only edit to a static header |
| S3 = DM6 (slice part) | `scan_region` G0, `inserter` B1, `admission` A0, `simulator`, base-plan builder, minimal writer/runner/CLI | T3/T4 regressions, T11 partition, acceptance (a)–(g) | **end-to-end demo** |
| S4 = DM5 | G3, G2′ ground truth, Φ and chain | T14, T11/T12/T15 | needs U1, U8 decisions confirmed |
| S5 = DM7 | region builder, E7 re-anchoring | region/closure tests | |
| S6 = DM8 | scope hook or fork | G0–G6, G5 protocol | needs U4 (gate machine/subset) |
| S7 = DM9 | local optimiser, RIC-C, baselines B2–B5 | T13 | needs U7 confirmed |
| S8 = DM10–DM13 | full runner/config/schema; pilots; experiments | as in DM0 §14 | |

## 12. Architecture decisions

| Id | Decision | Basis |
|---|---|---|
| AD1 | header-only `include/dynamic/`, `namespace tdr::dynamic`; static never depends on dynamic | [ARCH], repo convention |
| AD2 | layered module graph L0–L4 with the table in §2; simulator is the only event-loop owner | [ARCH] |
| AD3 | `InstalledPlan` = `Solution` + cached evaluator result + `shared_ptr<const World>`; `eval.feasible` is an invariant | [ARCH]; DM0 §9.1 |
| AD4 | immutable `World`; tentative = copy; commit = adopt; no rollback state | [APPROVED] D8, DM0 §10.4 |
| AD5 | strategy interfaces for scan, admission, base plan, inserter, region, optimiser; arms = configuration | [ARCH] |
| AD6 | clean cut computed from the cached evaluation (`clean_cut`), never from the truck's literal position | [APPROVED] D9, math_spec P1 |
| AD7 | slice = B1 + A0 + G0 on a static-VNS base plan; Φ, G3, regions, optimiser deferred | [ARCH] |
| AD8 | CMake option default OFF; first static-header edit (SeedStream append) at DM4 with regression rerun | [ARCH], E5 [APPROVED] |
| AD9 | separate dynamic config and schema; static config/schema untouched | [PROPOSED] Q10 |
| AD10 | scope-hook-or-fork hidden behind `LocalOptimizer`; N4 re-anchoring re-implemented in dynamic module | [APPROVED] D12 / [PROPOSED] E7 |
| AD11 | outcome partition `{OutOfScan, Expired, Stale, RejectedNoInsertion, RejectedPromise, Accepted}` exhaustive and exclusive | [APPROVED] DM0 §6.3 |
| AD12 | pool extension for new arcs (`with_pool_extension`) built only at DM7, using content-addressed FLP ids | [ARCH], D8 |

## 13. Blockers and open items (none is promoted to an approved decision here)

**Blocking implementation start:**
* **B1 — build-configuration approval.** DM3 needs new targets in `CMakeLists.txt` (option default OFF) and DM4 needs the additive `SeedStream` edit. Both are outside the "no build-config change" scope of the current instruction, so they need explicit approval. A manual `g++` build (as for the DM1 probes) is possible meanwhile, but not for CTest integration.

**Not blocking the slice, but to be confirmed before the milestone that needs them:**
* U1 (Φ chain) before DM5/DM6-A-PM; U8 (+∞ bound on straddled gaps) before DM5; U4 (G5 machine/subset) before DM8 (deferred to DM8 by instruction); U7 (certificate re-anchoring budget) before DM9; U2, U3, U5, U6, Q2, Q6, Q10, Q11, Q12, E7, Q-F remain [PROPOSED] and are all implemented as configurable defaults, not hard-wired.

**Risks to measure, not decisions:**
* **R1 tentative-world cost** (map copy + index rebuild at n=50 and n=100/200) — DM3 [MEASURE].
* **R2 evaluation cost of B1** is O(gaps) evaluator calls per request at ≈10 µs each (n=50) [STATIC]; negligible, but scenario sizes beyond the paper (n=100/200) are measured at DM6.
* **R3 slice under paper parameters is nearly drone-free**, so drone-aware behaviour is only exercised in regime F; this is a property of the study, not of the architecture.
* Carried math open items O1–O5 (math_spec §11).

---

## DM5 addendum: G3, promise predicate Φ, U1 admission chain

**CMake caveat (still open).** The `TDR_BUILD_DYNAMIC` option block (now also `test_dyn_dm5`) has **never been configured or built with CMake**: the development VM has no `cmake`. All DM3–DM5 builds and test runs used a manual `g++ -std=c++20 -O2 -Wall -Wextra -I include` script. The block is additive and OFF by default; verify it with `cmake -DTDR_BUILD_DYNAMIC=ON` on the Windows machine before relying on it.

**Conventions (DM3, now explicit).**
* **C1** – every decision that is actually made, *including a rejected one*, occupies the optimiser for ω simulated seconds (install time = start + ω).
* **C2** – if the clean cut leaves **no editable gap** the request is classified `Stale` (it still consumes ω). `Stale` is a decision-time outcome; it is never used for a request that had an editable gap.
* **C3** (DM5, PM1 only) – a queued request whose window can no longer be met (`s + ω > τ_r + W`, since every insertion lies behind the cut and so has ETA > s + ω) is `Expired`: dropped at dequeue, **consumes no ω**. With PM0 `Expired` never occurs.

**New / changed files** (all under `include/dynamic/` unless noted): `promise.hpp` (ledger, PM1+PM2 params, `slack_of`, `phi`), `outcome.hpp` (shared `Outcome`), `scan_region.hpp` (`ScanContext`; `ScanAll`=G0, `ScanNone`=B0, `ScanG3`, `ScanG3c`, tail set, `delta_hat_max`, ellipse/tube helpers, `G3Variant` negative controls), `admission.hpp` (`AdmitA0` unchanged behaviour; `AdmitAPM` = U1 chain; `Optimizer` hook = identity in DM5), `inserter.hpp` (trials keep their candidate plan), `simulator.hpp` (ledger, expiry, stage, drift metrics, I7 re-check), `config/runner/writer.hpp`, `apps/tdr_dynamic.cpp` (`--scan=g3|g3c`, `--admission=apm`, `--w-tilde`, `--eps-tilde`, `--eps-base-tilde`, `--varsigma`), `tests/test_dyn_dm5.cpp`.

**U1 chain (approved).** (1) S⁺ = cheapest evaluator-feasible truck insertion; (2) S* = optimiser(S⁺) (identity in DM5 = arm B1); (3) install S* if Φ holds, else the cheapest Φ-feasible insertion (`stage = fallback`); (4) otherwise `rejected-promise`. Φ = evaluator-feasible ∧ ETA_r ≤ τ_r + W (τ_r = **arrival** time, so queue waiting consumes W) ∧ ∀ promised i: ETA'_i ≤ P_i + E_class(i). `P_i` = first promise (base customers: base-plan ETA at t = 0; new: ETA at acceptance), no ratchet. The simulator re-checks Φ on every installed plan (I7) in addition to I1–I6. `rejected-no-insertion` (no evaluator-feasible insertion) is distinct from `rejected-promise` (feasible insertions exist but none satisfies Φ).

**G3.** `G3 = ⋃_g E_g(δ̂_max(g)) ∪ Tube_line`; δ̂_max(g) = min slack over the tail set T(g) if g is not straddled, **+∞ if straddled** (U8, approved). It is evaluated once at arrival against the last installed plan and ledger, with the earliest possible cut (τ+ω). **G2′ coverage is limited to the tested class**: single truck insertions into editable gaps on the same plan, no new drone trip, no re-anchoring (math_spec O2 stays open). The Tube term is necessary only for a new drone trip and is **not exercised by any test**. G3c = G3 ∩ tube(ς·ℓ) is **non-conservative by construction** (`G2′ ⊄ G3c`, tested); ς is a placeholder. G0 stays the all-area reference and G0/A0/B1 is unchanged (n=20 ρ=0.5: 149387→338975; n=10 ρ=1: 50766.2→200716).

**Finding: with tolerance-only slack, G3 = G0 whenever the end gap is editable.** The last gap (last customer → end depot) has an empty tail set (or is straddled → +∞), so δ̂_max = +∞ and its ellipse is the whole plane; inserting at the end delays no promised customer. G3 is therefore smaller than G0 only when no gap is editable (empty region), and the per-gap ellipses are informative only as a per-gap necessary condition (tested: no false exclusion; tight to ±5 m on the line world). Making G3 shrink in the PM1+PM2 model would need a W-aware term (an ETA lower bound per gap); this is **not implemented** (open decision for the user).

**Why all 162 demo decisions were accepted (G0/A0/B1).** Expected, not a missing path: (i) A0 has no promise; (ii) in these plans the end insertion is always evaluator-feasible (a depot-recovered trip is needed to break it); (iii) the cut always exists while arrivals ≤ T0_ref. The paths exist and are tested: `Stale` appears at large ω (n=20, ρ=2, ζ=3, ω=2000: 16 of 42 stale in one scenario); `rejected-no-insertion` is exercised by a hand-built trip recovered at the end depot (far request, (36) breaks); `rejected-promise` and `fallback` appear under A-PM (n=20, ρ=1, ω=60, W̃=2: 6–24 of 19–30 requests rejected-promise depending on ε̃); `expired` was **not** observed in any demo run and is covered by hand-built unit tests only.

## DM6 addendum: experiment driver
`apps/tdr_dynamic_experiment.cpp` (CMake target `tdr_dynamic_experiment`, same OFF-by-default block, unverified with CMake) runs paired arms (B0, G0/A0, G0/G3/G3c × A-PM at a list of ε̃) on identical scenarios and writes `experiment.csv`, `summary.csv`, `config.txt` (deterministic) and `runtime.csv` (wall clock, non-deterministic). Results and caveats: `docs/dynamic/dm6_experiment.md`; script: `docs/dynamic/run_dm6_demo.sh`. W-aware G3 (G3W) is **deferred** by decision and untouched; G3's bound is unchanged.

## DM7 addendum: region builder (`include/dynamic/region_builder.hpp`)

**Implemented.** Skeleton/trip layout (`layout_of`), `Region` (contiguous skeleton interval `[lo,hi]`, fixed boundary nodes, free customers/trips, frozen trips, pinned nodes), builders R0 (empty), R3/RIC core (insertion gaps + margin `w` + closure; `ring` widens by one skeleton node per ring), R4 (fixed interval), R5 (full editable tail), R1 (kNN comparator), the closure fixpoint (extend, else freeze with recovery pinned; monotone, terminates, idempotent), `ImpactSet`/`impact_of` (suffix behind r, shifted vs straddling trips), `describe` (impact vs optimisation overlap), `validate_region` (V1–V4 + free-set consistency from raw route positions), `check_region_preserved` (protected prefix/suffix/trips/coverage, no repair). Test: `tests/test_dyn_region.cpp` (89,799 checks incl. 600 random plans, P2 tail shift through the evaluator).

**Two sets, never mixed.** Impact = everything potentially affected by inserting r (a suffix; trips launched before and recovered at/after the old gap's right end are reported separately as *straddling*: their ETA is not shifted but their feasibility may change). Optimisation region = what the optimiser may change (a design choice).

**Decisions (first increment; items 1–2 superseded by addendum 2 below).**
3. Freezing only fires for a non-clean cut (a clean cut is never spanned by a trip); tested with a deliberately non-clean cut and reported via `frozen_trips`/`boundary_spanned_by_frozen`.
4. A region `[lo,hi]` with `hi` = end depot is allowed (tail shift is vacuous).

**Deferred.** R2/R2b (DBSCAN), R3t (route-time margin), adaptive-ring *policy* and certificate loop (DM9), ScopeMask type and operator hooks (DM8), any local optimisation (DM9).

## DM7 addendum 2: RIC core (P6 reach + P3 cost ranking), margin reconciliation

**Margin `w` (reconciled, not reinterpreted).** `candidate_window.hpp` places a truck customer at skeleton `s` at half-step `h=2s`, a drone customer launched in gap `g` and gap `g` itself at `h=2g+1`, and calls two entities adjacent iff `|h1-h2| <= 2w` with `w = adjacency_window = 3`. So the static `w=3` is a half-step *distance* of 6 (three truck nodes). The spec's "w = 3 half-steps" is read as this parameter, because it cites that file as its authority; the literal `|dh| <= 3` (1.5 nodes) would be a different, smaller window and is NOT used. The region margin is computed by calling `tdr::search::gaps_within_window` on the half-step of the extreme core gaps (`detail_region::window_hull`); the truck-node count `|g'-g| <= w` is a consequence. Tests pin it to the static function and check V5 against the static `PositionIndex::neighbours` (every adjacent customer lies inside the region). R4 keeps its own definition (fixed skeleton interval `h`), unchanged.

**RIC core (`RegionR3::ric(w)`, `CoreMode::Ric`).** Inputs: candidate plan `S+` (= base `S` plus r at skeleton `k`), `RicInputs{base S, candidate world}`.
1. Per editable base gap `g` (>= cut), P6 reach: `rho_g` = distance from x_r to the segment of `g`; `nu_g` = distance to the nearest skeleton node in `g+1..end depot`; `lhs = (1+theta)(rho_g/v_D + Se2) + nu_g/v_D`; reach-feasible iff `lhs <= e (1+1e-9)` and the fleet is non-empty. theta, Se2 are r's own data; `v_D`, `e` come from the world (regime P/F are just parameters). `drone_reach_radius = v_D (e-(1+theta)Se2)/(2+theta)` (6960 m for P, 14160 m for e=3600) is exposed and tested.
2. Per editable base gap, P3 cost `ETA_r + m_g * delta_g` with `ETA_r = departure_a + tau_ar`, `delta_g = tau_ar + Se1 + tau_rb - tau_ab`, `m_g` = truck customers at skeleton `>= b` plus drone customers launched behind `b`; ranked ascending, ties to the lower gap. A per-gap flag `p3_exact` (no FLP entry on the arc, no trip launched on/after the gap, no trip straddling it) marks where the value equals the evaluator; elsewhere it is only a ranking heuristic (CE-1/CE-2/CE-4 gaps are flagged, not hidden).
3. Core (candidate coordinates) = the two insertion gaps `{k-1,k}` U top `n_cost` cost-ranked gaps (default 1 = "best gap") U reach-feasible gaps (default all, ordered by `rho_g`; `n_reach>0` keeps the nearest `n_reach`). Base gap `g` maps to `g` (g<k-1), `{k-1,k}` (g=k-1), `g+1` (g>k-1).
4. Hull of the core, margin `w` (+`ring`) via the static window, clipped to `[cut, end depot]`, then the unchanged closure (extend, else freeze) and unchanged validation V1–V4.
`Region::ric` carries the full diagnostics (cost table, reach table, rank, core gaps). The candidate is checked to be `S` plus r (skeleton mismatch -> invalid, never guessed). The default `CoreMode::InsertionGaps` preserves the first increment bit for bit; R0/R1/R4/R5 untouched. Impact set (`impact_of`) is independent of the region and of `n_cost/n_reach/w` (tested).

**Evaluator-based validation** (`tests/test_dyn_region.cpp`): reach boundary at `r_D* -/+ 1e-3 m` agrees with the evaluator on a launch=recovery trip (P: 6960 m, F: 14160 m); the same point 10 km away is excluded in P and included in F; 1,000-trial fuzz: every evaluator-feasible trip lies in a reach-feasible gap (231 feasible, 468 gaps excluded, so the test has power both ways); P3 cost equals the evaluator delta on 200 random truck-only plans and the ranking matches; CE-4 world flags the FLP-on-arc and straddled gaps (evaluator rejects the former); region-level boundary toggling (`nu = 14880 -/+ 1e-3` adds/removes the far gap from the core and changes `hi`).

**Ambiguities A1–A5 and their exact interpretations (A1 approved 9 Nov 2026, 23:26 IST; A2–A5 are implementation readings, recorded for review).**
- **A1 (approved).** "Best drone-feasible launch gaps" = **all** reach-feasible editable gaps, listed by increasing `rho_g` (ties to the lower gap), **no truncation**. `RegionR3::n_reach` default is 0 = unlimited. Any finite `n_reach` (and `n_cost > 1`) is a **restricted experimental variant**, never the default RIC policy: such regions carry `restricted_variant = true` and the builder reports the name `R3-ric-core-RESTRICTED-variant`. Consequence: under paper parameters most gaps are reach-feasible at n >= 10, so the default RIC region is wide (the spec's own remark about the tube).
- **A2.** The spec states the sharper reach inequality for the whole editable route (`rho_line`, nearest node `nu`). It is applied **per gap**: `rho_g` = distance to the segment of gap g, `nu_g` = distance to the nearest skeleton node in `g+1..end depot`. Same inequality per arc; a necessary condition only.
- **A3.** "Best gap" = the single P3-cost minimiser (`n_cost = 1`); ties to the lower gap.
- **A4.** P3 cost is used as the ranking the spec names. It equals the evaluator's insertion delta **only where `p3_exact`** (no FLP entry on the arc, no trip launched on or after the gap, no trip straddling the gap). For flagged gaps (`p3_exact = false`: launch FLP on the split arc, CE-2; straddled gap, CE-1/CE-4) the value is a **non-exact estimate** used for ranking only, and the evaluator may reject the insertion outright; no replacement ranking is invented. `Region::ric.costs[i].p3_exact` exposes the flag per gap.
- **A5.** The candidate's actual insertion gaps are always part of the core (r must be a free customer); if the P3-best gap differs from them the core is the union.

**Exact vs non-exact P3 (what the tests establish).** Exact (`p3_exact = true`): equality with the evaluator's `F(S+) - F(S)` to 1e-9 relative on 200 random truck-only plans (all gaps, 1,000+ comparisons) and on the non-straddled, FLP-free gaps of the CE-4 world; the induced ranking equals the evaluator ranking. Non-exact (`p3_exact = false`): only flagged, never asserted equal; the CE-4 world shows the FLP-on-arc gap evaluator-infeasible and the straddled gap flagged.

**Half-step interpretation preserved.** `w = 3` is the static parameter with adjacency `|h1 - h2| <= 2w` (6 half-step units = three truck nodes); the margin is computed by `tdr::search::gaps_within_window`, and tests compare it to that function and to `PositionIndex::neighbours` (V5). The literal "3 half-steps" is not used.

**Move-class limitations of the P6 reach validation (what was and was not tested).** Tested against the evaluator: a **single** drone trip serving r (r as drone customer), launch FLP at a point **on the arc** of the gap (continuous position, one FLP), recovery at a **later skeleton node** (incl. end depot), on **truck-only base plans** (no pre-existing trips), 3–5 customers, one drone-endurance value per trial (e = 1800 and 3600), plus boundary constructions at r_D* -/+ 1e-3 m (launch = recovery node, and a distant-recovery case). Not tested: re-sequencing moves (CE-3: a request reachable only after the truck is re-ordered may be excluded by per-gap reach; reach is a necessary condition for **insertion into the current route**, not for re-optimisation); plans with existing trips or several concurrent drones (fleet-size and rule (28) are ignored by the predicate); discrete FLP spacing (the segment is continuous, so the predicate is looser than any discrete FLP set); (36), which is not used (it depends on truck timing). Reach never claims sufficiency, and the evaluator remains the only feasibility authority.

**DM7 status (closeout).** Complete RIC construction: layout -> core (insertion gaps U best-cost gap U all reach-feasible gaps, A1) -> hull -> static-window margin (+ rings) -> clipped to `[cut, end depot]` -> closure (extend, else freeze; idempotent) -> `validate_region` (independent, from raw route positions: V1–V4, free-set consistency) -> `check_region_preserved` (protected prefix, suffix, trips, coverage; never repairs). Defaults: `w = 3`, `ring = 0`, `n_cost = 1`, `n_reach = 0` (unlimited), `CoreMode::Ric` via `RegionR3::ric()`; the plain `RegionR3` keeps `CoreMode::InsertionGaps` (first-increment behaviour). Not part of DM7: scope mask/hooks (DM8), local optimiser and certificate (DM9), R2/R2b/R3t comparators, adaptive-ring policy.

## DM8 addendum: dynamic local-search scope hook (`include/search/scope_hook.hpp`, `include/dynamic/scope_mask.hpp`)

**Status.** Implemented, gated (below), **not committed**. DM9 (regional optimiser) is not started. Approved decisions of 9 Nov 2026 (23:45 / 23:52 IST): operator-side hooks (no post-evaluation wrapper); scope rejection BEFORE `evaluate_solution`; identity-based `ScopeMask`; DM7 validator/checker unchanged and independent; pinned-node semantics below; scoped shakes refused and deferred; G0–G4, G6 on this VM, G5 informational (gate machine undesignated, open item U4).

**Architecture (smallest additive interface).** `ScopeHook::check(const Solution& candidate, const Move&) -> ScopeViolation` is a pure predicate on ONE fully constructed candidate, reached through the new trailing member `SearchContext::scope` (default `nullptr` = unscoped = the static code path). It sees no base solution: the dynamic mask is an identity snapshot taken from the plan the region was built on, so it stays valid while an iterated search replaces its base. Operators call it immediately before `evaluate_solution`, after every structural rejection they already make. The pattern at every site is
`sv = opt.evaluate_candidates ? violation(ctx, cand, m) : None; if (sv != None) { count; record } else if (opt.evaluate_candidates) { <unchanged evaluator branch> }`.
With `scope == nullptr` the added test is one pointer comparison; no operator reads or writes anything else. Candidate order, RNG consumption (the Rng overloads still draw the anchor over ALL customers/trips), strict-`<` first-wins tie-break, all pre-existing counters and the N4 cap are untouched (the 8 modified pre-existing lines are the `if (opt.evaluate_candidates) {` heads that became `else if`, plus one `first` -> `reason_taken` in N4, see below).

**Operator inventory and hook sites** (all N1–N8; evaluator-call sites verified by code reading):

| Op | candidate = | evaluator call site | hook site | what a scope can stop |
|---|---|---|---|---|
| N1 launch relocate | one trip's launch FLP replaced; route FLP entries edited | `detail_relocate::judge` | before its evaluator call (header `trip_relocate.hpp`) | non-free trip as anchor; a changed protected trip |
| N2 recovery relocate | one trip's recovery node replaced | `detail_relocate::judge` | same | same; recovery moved onto/over protected nodes only if the evaluator and DM7 allow it |
| N3 / N6 | two trips swap drone customers | `detail_exchange::judge` | before its evaluator call (`trip_exchange.hpp`) | either customer not free |
| N4 | truck customer relocated / exchanged; optional re-anchoring of stale launch FLPs | `run_evaluator` (3 call sites) | Reject path, `judge_as_is`, and **each re-anchor variant** | anchor/partner not free; a re-anchor that changes a PROTECTED trip's launch FLP (seen: `protected-drone-customer-changed`) |
| N5 | truck customer <-> drone customer, new trip | inline, after `lowest_free_drone_for_trip` | before the evaluator call | either customer not free; trip changes |
| N7 | truck customer -> new drone trip | inline, after `lowest_free_drone_for_trip` | same | anchor not free |
| N8 | drone trip dissolved, customer into a gap | inline, after the AffectedTrips branch | same | trip customer not free |
| shakes S1–S4 | multi-node perturbation | `detail_shake::begin` | **refused**: scoped call returns `ShakeStatus::InvalidParameters` before any evaluation | everything (no scoped shake exists, see Deferred) |

**Correction (review of 10 Nov 2026, 00:39 IST).** The first DM8 version had no positional boundary rule and left boundary crossings to DM7's post-acceptance check, i.e. after an evaluator call. Fixed: two rules are now kept apart. *Rule 1:* free customers may cross INTERIOR pinned nodes (frozen recoveries); pinned identity, trip assignment and relative order are preserved. *Rule 2:* the boundary is protected before evaluation: the plan's route prefix (through lo) and suffix (from hi) are stored as node-id sequences and must be the candidate route's prefix/suffix verbatim (`ProtectedPrefixChanged`, `ProtectedSuffixChanged`); a trip serving a free customer must launch strictly between prefix and suffix and be recovered after its launch and no later than the first suffix node (`FreeTripOutsideRegion`); protected-trip dependencies were already enforced. No skeleton index or position is stored. R0/invalid region: the whole route is the protected prefix. Also: a scoped call with `evaluate_candidates == false` is now refused by a precondition in every operator (`OperatorStatus::ScopeRequiresEvaluation`, no generation, no hook call, zero evaluator calls), so the hook cannot be bypassed; unscoped `evaluate_candidates=false` is unchanged. Statements below that say the mask has no positional window, or that `evaluate_candidates=false` bypasses the hook, are superseded by this correction.

**Scope rules (`ScopeMask::check`, first violated rule is reported; none repairs anything).** (1) `MoveAnchorNotFree`: `move.customer`/`move.partner` not free. (2) `ProtectedTruckCustomerChanged`: a protected truck customer missing, duplicated, or drone-served. (3) `ProtectedDroneCustomerChanged`: a protected drone customer put on the truck, its trip missing/duplicated, or its drone id / launch / recovery changed. (4) `ProtectedTripDependencyBroken`: a protected trip's launch absent or its recovery no longer after it. (5) `PinnedNodeMissing`, (6) `PinnedOrderChanged`: pinned nodes (Region `pinned_nodes`: boundary lo/hi, frozen recoveries; depots included) absent / no longer in their original relative order. (7) `ProtectedPrefixChanged`, (8) `ProtectedSuffixChanged`, (9) `FreeTripOutsideRegion` (correction above). Order: anchor, protected customers/trips, dependency, pinned, prefix, suffix, free-trip window. Free = `Region::free_customers` minus the pinned nodes (a pinned recovery customer is never free; DM7 itself is unchanged and still lists interior pinned customers as free, A-note of the DM7 closeout). **Decision 3 (with the correction):** pinned nodes keep identity, trip assignment and relative order; free customers may cross interior pinned nodes, but not the lo/hi boundary. `R0` / invalid region -> empty mask: no free customers, no anchors (`free_truck_customers`, `free_trip_indices` empty), every anchored move `MoveAnchorNotFree`. `ScopeMask::full()` is a no-op hook.

**Counter accounting (verified, supersedes DM0 §9.4(4) "skipped not counted as generated").** Unscoped, `evaluate_candidates = true`: `accepted + rejected_total == generated` for N1–N8; N4's `duplicates_skipped` are separate (never in `generated`), N4's `evaluated` counts variants and can exceed `generated`; variants beyond the N4 cap are never generated, so a cap-limited variant is neither evaluated nor a scope event (`cap_reached_candidates` is orthogonal). Scoped (new fields, all 0 without a hook): `scope_rejected` (candidates stopped before the evaluator, counted in NO `rejected[]` slot), `scope_by_violation[]`, `scope_variants_skipped`, `evaluator_calls_avoided`. Identities (asserted for every run in `test_dyn_scope`): `accepted + rejected_total + scope_rejected == generated`; `evaluated + evaluator_calls_avoided == evaluated of the unscoped run`; `evaluator calls == 1 (the base) + evaluated`; `generated`, `duplicates_skipped`, structural rejects, `variants_generated`, `cap_reached_candidates` identical to the unscoped run. N4 re-anchoring: each variant is checked separately; a skipped variant is counted in `scope_variants_skipped` and `evaluator_calls_avoided`; a candidate whose generated variants were ALL skipped is `scope_rejected` (log: `scope_violation` = first skipped variant's rule, `variants_evaluated == 0`); a candidate with some variants evaluated is judged by the evaluator on the rest. **Consequence for partially skipped N4 candidates:** the reported reject reason is that of the first *evaluated* variant (the unscoped code used the first variant; identical when nothing is skipped), so evaluator-reject classes can shift between slots for such candidates (only in that case). With `ScopeMask` partial skipping cannot occur (all variants of one candidate change the same protected identities), the path is exercised by a variant-sensitive probe hook.

**Tests (`tests/test_dyn_scope.cpp`, 2,118,633 checks, 0 failures).** (A) mask unit tests for each rule, R0/invalid/full mask, pinned depots, pinned-recovery exclusion, anchor helpers. (B) bit-identity against the **frozen pre-hook copy** of all 16 `include/search` headers (`tests/reference/search_pre_hook`, verbatim from `static-baseline-m10`, namespace `tdr::search_ref`, regenerated and inverse-verified by `generate.sh`, registered as CTest `dm8_search_pre_hook_verbatim`) for null scope, full mask, and a pass-all hook: complete results incl. logs (every field), best solution, N4 diagnostics, **evaluator calls via a spy**, and RNG state after the Rng overloads; also `evaluate_candidates=false` (hook not consulted). Shakes and `run_vns` unscoped likewise (trajectory, evaluator calls). (C) a reject-all hook yields exactly ONE evaluator call (the base) for every operator/anchor/config, `evaluator_calls_avoided == unscoped evaluated`. (D) scoped brute-force oracle (G6): for every region mask (R0, R1, R3, R4, R5) and variant-sensitive probe hooks, candidate order, structural rejects, per-candidate verdicts, strict-`<` best and all counts equal the oracle derived from the unscoped log (one-sided bound only for partially skipped N4 candidates). (E) every mask-accepted best is checked with DM7 `check_region_preserved`: 0 of 6,495 flagged (before the correction 523 of 6,675 were flagged for boundary reasons). (E2) per-candidate proof with a spy hook wrapper: the evaluator counter never changes between a rejecting `check()` and the next candidate, over all operators/anchors/configs/masks (0 violations); boundary rejections before evaluation per operator over the fixtures: N1 2,847, N2 234, N3 0, N4 4,334, N5 1,623, N6 0, N7 4,750, N8 333 (N3/N6 only exchange customers of existing trips, so they cannot cross a boundary). Regression tests: unit cases for prefix/suffix/trip-window and for interior-pinned crossing (rule 1 retained), N4/N8 boundary counts asserted > 0, precondition tests for all 8 operators (scoped + `evaluate_candidates=false` refused with zero evaluator calls; unscoped unchanged). (F) determinism: each scoped run repeated. (G) mask vs an independent naive re-implementation on 144,000 random edits (all nine violations reached), shakes refused with zero evaluator calls. Fixtures: 40 plans (regime F, n=9–17, 2–3 drones, static-VNS and constructive bases; 18 with drone trips, 95 trips), 360 masks. Observed operator-driven violations: `move-anchor-not-free` and `protected-drone-customer-changed` (N4 re-anchoring of a protected trip); the other rules are protected by unit tests, the naive-oracle fuzz and mutation. One bug was found by these tests during development and fixed: `kScopeViolationCount` was 6 for 7 enumerators (now with a `static_assert`).

**Gates (this VM; no CMake available, manual `g++ -O2 -Wall -Wextra`).**
- **G0** diff vs `static-baseline-m10` in `include/search`, `include/timing`, `src`: only `include/search` changed, 12 files, +100/-8; the 8 removed lines are the 7 `if (opt.evaluate_candidates) {` heads turned into `} else if (...)` chains and N4's `else if (first)` -> `else if (!reason_taken)`; the precondition adds one line per operator; `include/timing`, `src`, evaluator and static defaults untouched. New files only otherwise.
- **G1** all 23 static test programs (re-run after the correction) (`tests/test_*.cpp` except `test_dyn_*`, incl. all oracle tests, `test_runner_cli`, `test_vns*`, `test_shakes*`) rebuilt with the modified headers and pass.
- **G2** M9: the 9 protocol configurations pass `tdr_static check`; `config/m9/n10_seed1.cfg` re-run reproduces `results.json`, `runs.csv`, `config.resolved.cfg` byte-identically (the content of CTest `m9_protocol_reproduces`, run by hand because CMake is unavailable).
- **G3** bit-identity vs the frozen reference: in `test_dyn_scope` (B), 3,355 anchor/config runs x {null, full, pass-all} + Rng overloads + shakes + VNS: 0 differences, evaluator calls and RNG state equal.
- **G4** mutation (`tests/mutation/dm8_mutate.py`, 39 mutants of the guard lines, counters, shake refusal, the eight operator preconditions and all mask rules incl. prefix/suffix/trip-window): 39 killed, 0 survived (the full-mask shortcut mutant, equivalent in the first version, is killed now that the free-trip window rule exists). In the first version one mutant survived the first test version (`scope_variants_skipped` not incremented); the test was strengthened (exact sums) and the mutant is now killed.
- **G6** scoped oracle: part (D) above, 0 failures.
- **G5 (informational, not a gate: VM, no designated machine; re-measured after the correction: ratio 1.018, 831 vs 4,999 evaluator calls)** `tests/bench_dyn_scope.cpp` (n = 30, 14 drone trips, all anchors, N1–N8): unscoped path with null scope vs reference ratio 0.93–1.03 over runs (noise level), identical evaluator calls (4,999) and candidates (1,411); a region mask (6 free customers of 30) cut the evaluator calls of the same sweep from 4,999 to 890 and the wall time from ~0.036 s to ~0.016 s (all anchors still enumerated, so the saving is an upper bound on what an anchor-restricted DM9 loop would not even attempt).
- Dynamic suites re-run unchanged: `test_dyn_core` 64, `test_dyn_slice` 166, `test_dyn_dm5` 256, `test_dyn_region` 130,494 checks, 0 failures. DM6 experiments E1–E3 re-run from `run_dm6_demo.sh`: `experiment.csv`, `summary.csv`, `config.txt` byte-identical to `docs/dynamic/dm6_results` (`runtime.csv` is wall clock and excluded).

**Implemented vs deferred.** Implemented: hook interface, guards in N1–N8, shake refusal, counters, `ScopeMask` (+ anchor helpers, `ScopeStats`), tests, reference copy, mutation script, CMake targets (unverified: no CMake on the VM). Deferred: **scoped shakes** (D); any DM9 element (optimiser loop, certificate, adaptive rings). Limitations and **dependencies for DM9**:
1. **DM9 must not call unscoped shakes or `run_vns` on a region** (shakes would leave the region): a scoped context refuses shakes (`InvalidParameters`, zero evaluator calls) and `run_vns` with a scoped context applies no shake. A scoped shake or a region-local perturbation must be designed first.
2. DM9 should use the explicit-anchor overloads with `ScopeMask::free_truck_customers/free_trip_indices`. The Rng overloads draw the anchor over ALL customers/trips (RNG consumption is deliberately unchanged), so under a scope a protected anchor yields only scope rejections (no evaluator call except the base evaluation).
3. DM7's `validate_region`/`check_region_preserved` remain independent post-acceptance checks that DM9 must still run on every accepted result; the hook now rejects boundary crossings itself.
4. `evaluate_candidates = false` with a scope is refused (`ScopeRequiresEvaluation`).
5. The hook is per candidate/variant; there is no scope-aware pruning of the enumeration itself (candidates are still constructed before being rejected).
6. CMake wiring untested here; G5 numbers are VM numbers; G3W (W-aware G3) remains deferred; one GCC `-Wmaybe-uninitialized` false positive (`std::optional<Candidate>` in `move_types.hpp`) appears only in the informational benchmark TU, not in the tests.

## DM9 addendum: regional local optimiser, arms/baselines and the RIC-C certificate (`include/dynamic/regional_optimiser.hpp`)

**Status.** Implemented and gated on this VM (below), **not committed**. Additive: one new header, one new test, one bench, one mutation script, CMake targets (unverified: no CMake on the VM), a 12-line change to `AdmitA0` in `admission.hpp` (identity-preserving, see 4) and this addendum. Static code, evaluator, DM7 and DM8 are untouched; DM10 (runner/config/CLI/writer) is not started, so `DynamicConfig`, `runner.hpp`, `writer.hpp` and the DM6 result files are unchanged.

**1. What runs.** `optimise_plan(cfg, base, cand, cut, request)` takes S+ (the DM6 cheapest-insertion plan bound to its tentative world) and returns S*. Region by arm (DM7 builders, unchanged): RIC core `RegionR3::ric(w)` for FR/RIC-C/B2, `RegionR5` for FT/B4; `ScopeMask::from_region`. Every operator call is `n?_*(plan, ctx, explicit_anchor)` with `ctx.scope = &mask`, `evaluate_candidates` at its default `true` (never set in the file; DM8 refuses otherwise), anchors taken from `ScopeMask::free_truck_customers / free_trip_indices` in fixed order (N4, N7: truck customers in route order; others: trip indices). **No Rng overload, no `shake`, no `run_vns`** appears in the header (a test reads the source and asserts it; DM8 still refuses scoped shakes). Every candidate is a complete plan judged by the unchanged evaluator; the final S* is re-evaluated once more and falls back to S+ if it were infeasible or worse (defence in depth; a documented equivalent mutant).

**2. Documented defaults (where the spec was silent; none blocked).**
- D1 *search rule*: deterministic first-improvement descent over `default_neighbourhood_list()` (N4,N5,N8,N3,N6,N7,N1,N2,N1,N2). For each operator the free anchors are tried in order; the first anchor whose best candidate improves F by more than `tau_imp` is accepted and the list restarts (VNS k = 0). No simulated annealing (without shakes it would only add randomness); the DM0 §9.2 `t0_absolute` is therefore unused. DM0 §9.2 says `run_vns(S+, ctx_with_scope, ...)`; the user's DM9 instruction (no shakes under scope) overrides it and this loop replaces it.
- D2 *budget*: at most `P = min(P_max, ceil(c_P (|free customers| + 1)))` operator calls, `c_P = 40`, `P_max = 4000`; unit = one operator at one anchor (DM0's "passes" are not reproduced). Deterministic; wall clock is recorded in `OptResult::seconds`, never used.
- D3 *tolerance* (corrected after the DM9 audit): `tau_imp = 0` by default, i.e. strict `F' < F`, exactly the static VNS default (`VnsOptions::min_improvement = 0`) and the DM0 §8.3 M5 requirement of one declared tolerance; the earlier 1e-9 differed from VNS and is withdrawn. `tau_imp` stays configurable (>= 0, finite; anything else is a configuration error handled as an exception, see C2), drives BOTH the search and the sweep, and is logged in the manifest (`tau_imp`, `tau_search`). Rationale for 0: floating-point ties are exact for equal plans (identical evaluations), so a positive tolerance only hides genuine tiny improvements the static search would take.
- D4 *expansion*: when the sweep finds an improving single move, that move is adopted (its plan is evaluator-feasible and inside the tail), its footprint (anchor, partner, replaced/new nodes, the trips it names with launch/recovery, destination gap endpoints) is mapped to skeleton indices of the improved plan, hulled with the current region (boundaries tracked by node identity), re-closed with the DM7 closure (`detail_region::finish`), a new mask is taken from the improved plan, and the scoped search continues. Bounded by `max_expansions = 50`.
- D5 *certificate cost*: the sweep is counted separately (`sweep_operator_calls`, `sweep_evaluator_calls`), not against the search budget (DM0 leaves the accounting to DM11). The sweep may use a larger N4 re-anchoring budget than the search: `sweep_n4_cap = 0` (unlimited), search cap 100 (static default) [U7 default adopted; the downgrade rule holds for any value].
- D6 *Phi*: the optimiser is Phi-unaware and the sweep is about F only. The U1 chain stays where DM5 put it (`AdmitAPM`): S* is installed iff Phi holds, else the cheapest Phi-feasible insertion (B1), else `rejected-promise`. An optimiser failure (nullopt, S+, exception caught inside `RegionalOptimizer`) can therefore never cost a Phi-feasible B1 insertion.

**3. Arms and baselines (`OptArm`).** `R0` identity (B1). `FR` fixed RIC region, no expansion, no certificate. `RIC-C` FR + certificate sweep over the whole tail + expansion until `certified` or `budget-stop`. `FT` full editable tail (R5), same loop, no expansion. `B2` N4 only, restricted to the RIC region. `B4` full tail whose operator-call budget is DERIVED inside `optimise_plan` from the matching RIC region of the same decision (same builder, same `w`; `OptResult::budget_source == "ric-region"`, `ric_free_customers`); `calls_override` remains only as a test override (`budget_source == "override"`). Scan policy (G0/G3/G3c/none, existing), admission (A0/A-PM, existing) and optimiser (`SimOptions::optimizer = &RegionalOptimizer(cfg)`) are three independent objects. The optimisation region (a DM7 `Region`) is never the downstream impact set (`impact_of`, untouched). **Deferred:** RIC-S (the stagnation rule was withdrawn in DM7, C9), B3 (RIC-C + static VNS needs shakes, forbidden under a scope), B5 (exact truck-only Held-Karp reference), B6 (offline hindsight); B4 is the scoped no-shake full-tail loop, not a static VNS with shakes.

**4. A0 hook.** `AdmitA0` now calls `opt.optimise(in, S+)` and installs S* (no Phi): A0 and A-PM share the insertion/optimisation pipeline and differ only in Phi. With the identity optimiser (`nullptr` in `SimOptions`, or `R0`) the path is the previous one (same plan, same trial index), proved by the DM6 reproduction below and by a test that compares the full decision signature of `R0` and the no-optimiser pipeline. An installed optimiser plan has no single-insertion trial index, so I1-I6 (defined for single insertions) are skipped for it, exactly as DM5 already did for A-PM; the evaluator, the cached-evaluation consistency check and (A-PM) I7/Phi still run.

**5. Certificate semantics (RIC-C).** After the regional search, `certificate_sweep` runs on a snapshot of exactly the plan to be returned, with the cut-only mask (full tail), explicit anchors only, all of N1..N8 over the complete anchor domain. **`certified` means: M1 for every operator the anchors swept equal the domain `|A_o|`, computed independently from the plan layout (not from the operator or the mask); M2 every anchor returned without exception, status `Ok`, and `accepted + rejected + scope_rejected = generated`; M3 no N4 re-anchoring cap / limit counter positive; M4 the sweep ran on the returned plan; M5 no single move improves F by more than the logged `tau_imp`.** Any failure of M1-M4, an exception, a sweep budget (`sweep_call_budget`) hit before completion, or the search budget exhausted with an improving move still available gives `budget-stop` (the plan is returned, never `certified`). Moves rejected by the scope in the tail sweep are counted `out_of_scope_skipped`. **Certified is local optimality with respect to the single N1-N8 moves as implemented (window w, N4 re-anchoring enumeration, FLP-only launch candidates, evaluator feasibility); it is not global optimality and says nothing about Phi.** Guarantees on every returned plan: evaluator-feasible, same served set, committed prefix/clean cut and region contract preserved (independent DM7 `check_region_preserved`), `F(S*) <= F(S+)`, termination (budget + `max_expansions`).

**6. Tests (`tests/test_dyn_opt.cpp`).** 30 seeds (chunks of 1-10 seeds via `DM9_SEED0`/`DM9_SEEDS`; seeds 15-17 are slow and were run as 15 / 16 / 17), 108 cases built from consecutive cheapest insertions into mostly constructive base plans (regime F, n0 = 10-18, 2-3 drones, clean cut at random times) plus 96 paired simulations, 0 failures (checks per chunk, e.g. 16,306 for seeds 0-9, 19,501 for seeds 20-29). Covered: R0 identity; every arm evaluator-feasible, `F(S*) <= F(S+)`, same served set, prefix verbatim, DM7 tail/region preservation by an independently built region, a negative control for the DM7 checker; determinism; evaluator spy (observed calls == counted calls); certificate (every `certified` plan has independent domain counts equal to swept counts, `stop != Budget`, M1-M5 true, and no improving single move in an independent brute-force scan); T13 faults (i)-(v) each reported by its own condition, **(vi) exact** (see C5), (vi-b) sweep call budget, **(M5)** tolerance mismatch; the exception, B4, U1 and partner tests of C2-C4, C7 below; simulations with [P15] on identical inputs; source guard.

**7. Other gates (this VM, manual `g++`; no CMake).** Dynamic suites unchanged: core 64, slice 166, dm5 256, region 130,494, scope 2,118,633 checks, 0 failures; all 23 static tests pass; the 9 M9 protocol configurations pass `tdr_static check` and `n10_seed1` reproduces `results.json`, `runs.csv`, `config.resolved.cfg` byte-identically; `generate.sh --check` verbatim; DM6 E1-E3 regenerated from `run_dm6_demo.sh`: `experiment.csv`, `summary.csv`, `config.txt` byte-identical to `docs/dynamic/dm6_results` (`runtime.csv` untracked and untouched).

**8. Benchmark evidence (informational, VM, relative only; `tests/bench_dyn_opt.cpp 10 16`: 10 paired scenarios, n0 = 16, rho = 3, omega = 10 s, G0, 436 decisions per arm).** Reported in two SEPARATE sections: A0 = optimisation quality (served sets driven by feasibility), A-PM = admission performance (served sets diverge). *A0:* served sets are identical to R0 in 10 of 10 scenarios; FR 67 plans improved, RIC-C and FT 68, B2 63; mean final objective vs R0 on identical served sets: FR, RIC-C, FT -1.870 %, B2 -1.267 %. **FR, RIC-C and FT end with identical final objective and served set in 10 of 10 scenarios (9 of them non-trivial; scenario 1 accepts nothing), so the -1.87 % aggregate does not distinguish FR, RIC-C and FT: it separates the regional arms from R0, and B2 from the rest, and nothing more.** Differences among FR/RIC-C/FT (RIC-C improves 68 plans vs FR's 67, FT spends 484 vs 296 evaluator calls per decision) are visible only in decision-level counters, not in final objectives. RIC-C certified 436 of 436 decisions (sweep 434 evaluator calls per decision on top of the search's 295). *A-PM:* accepted 212 (R0), 241 (FR), 235 (RIC-C, FT), 234 (B2); the optimiser changes later feasibility, so only 3 (4 for B2) of 10 runs share R0's served set and objective is compared only inside those (0.000 %); Jaccard overlap with R0's accepted set is printed per scenario (as low as 0.50). Use A-PM for admission, not for objective. The per-scenario table, comparable-run counts and overlap are printed by the bench. DM11 should: use A0 for optimiser quality and A-PM for admission as separate endpoints; compare ETA/objective on the COMMON served set only; replay decisions on identical inputs (as the [P15] checker does) instead of whole-run objectives; run many more scenarios with a paired bootstrap. 10 scenarios at one parameter point are sanity evidence, not a performance study.

**9. Limitations.** (a) Region-restricted search sees only single moves; certified is single-move local optimality. (b) The default sweep leaves N4 re-anchoring unlimited (U7 default); with the static cap of 100 some cases could not be certified. (c) Search budget unit is operator calls, not DM0 "passes"; c_P and P_max are heuristic placeholders for DM11 tuning. (d) `evaluator_calls` inside `OptResult` excludes the evaluations done by `InstalledPlan::make` and the admission Phi checks (those are counted by the simulator). (e) I1-I6 are not defined for optimised plans. (f) The brute-force certificate check hides N4 variants (it keeps the best variant per candidate, so it can only under-count improving moves, never invent one). (g) CMake targets untested; GCC `-Wall -Wextra` is clean for the new files (a pre-existing `-Wmaybe-uninitialized` in `move_types.hpp` appears only in some translation units). (h) The simulator records no optimiser fields (`DecisionRecord` unchanged to keep DM6 outputs byte-identical); they live in `RegionalOptimizer::records`.

**10. Remaining for DM10.** Config keys and CLI for scan/admission/optimiser (`DynamicConfig` has none), writer columns for the optimiser records (objective before/after, accepted moves, evaluator calls, scope rejections, runtime, certificate status, protected-route changes), serialisation of the manifest, certificate-cost accounting policy (DM11), objective comparison by served-request set in the runner (as done by hand in the bench), RIC-S/B3/B5/B6, tuning of `c_P`, `P_max`, `w`, `sweep_n4_cap`.

### DM9 corrections (post-audit)

**C1 Tolerance / M5.** `tau_imp` default 0 (see D3). The manifest carries `m5` and a short failed-condition code (`failed` = M1..M5 / BUDGET / SWEEP-BUDGET / EXPANSIONS / EXCEPTION) and the search's tolerance `tau_search`; `complete` now means M1..M5 hold. Fault `sweep_tolerance_mismatch` (sweep tau != search tau) must give `budget-stop` with `failed == "M5"`; an invalid tau (negative / NaN) is a configuration error (C2).

**C2 Exceptions.** `optimise_plan` wraps the implementation in `try` with `catch (const std::exception&)` and `catch (...)`. Any exception during region building, search, sweep or expansion returns the input plan S+ (`failed`, `failure_reason`, `stop = Error`, `f_after = f_before`, protected/commitment counters 0, counters collected so far kept). RIC-C keeps its arm and reports `budget-stop` with `manifest.failed = "EXCEPTION"`; the other arms report `NotRun`. `RegionalOptimizer::optimise` additionally catches `(...)` and records. `AdmitA0` and `AdmitAPM` call the optimiser through `guarded_optimise` (exception -> same as returning nothing, `Decision::optimiser_failed`), so an exception can never bypass the U1 chain or escape into the simulator; with the identity optimiser behaviour and notes are unchanged (DM6 byte-identical). Tests: std and non-std throws at region / search / expansion sites for FR, RIC-C, FT, B2, B4; throwing optimisers through A-PM and A0 simulations.

**C3 B4 budget.** B4 derives its budget from the RIC region of the same decision inside `optimise_plan`. Simulator-level test: per decision FR, RIC-C and B4 budgets are equal (72 of 72), `budget_source == "ric-region"`, FT's own budget would differ in 34, and B4's evaluator calls differ from FR's in 34 decisions (36,972 vs 28,134 calls in total). **Equal operator-call budgets do not imply equal evaluator cost**: B4 has a larger free region, hence more candidates and more evaluations per call; the budget unit is operator calls only.

**C4 U1 evidence.** `U1Witness` recomputes, test-side and independently, the cheapest Phi-feasible insertion for every A-PM decision and checks that the chain installs the optimised plan when it satisfies Phi (Primary), else exactly that insertion (Fallback), else `rejected-promise`. Run with worst-first (Phi-violating), nothing-returning, std-throwing, non-std-throwing, RIC-C, FT and fault-injected optimisers over 126 simulations (174 decisions each): fallbacks asserted > 0 for every optimiser (36-99), violating-with-feasible-insertion cases > 0, the throwers' simulations are bit-identical to the identity pipeline, and `fallback > 0` is also asserted in `test_simulation`.

**C5 Fault (vi), exact DM0 case.** Region = full tail Lambda0 (`w` spanning the tail, `region_full_tail` asserted) with the search budget exhausted before the sweep: result must be `budget-stop`, never `certified`. Implementation rule: if the regional search stopped because of its budget (`stop == Budget`), the final status is `budget-stop` (`failed = "BUDGET"`) even when the sweep that follows is complete and clean; **`certified` therefore implies the regional search converged**. This is stricter than the first DM9 version (which could certify after a budget cut if the final sweep was clean). The sweep-call-budget variant is kept as (vi-b). Test: 32 of 32 cases (seeds 0-9) have Lambda0 = full tail, all 32 budget-cut runs are budget-stop; the same region with the default budget certifies 32 of 32 (the test has power).

**C6 Benchmark reporting.** See item 8: separate A0 / A-PM sections, per-scenario table with served-set overlap, comparable-run counts, and the explicit statement that the aggregate does not distinguish FR/RIC-C/FT.

**C7 M1 partner enumeration.** The manifest counts ANCHORS (M1 compares swept anchors with an independently computed anchor domain). Partners (N3 predecessor/successor, N5 partners, N6 window, N4 exchange) are enumerated INSIDE the operators. Evidence recorded: (a) scope-invariance for all of N1-N8 on every anchor of every certified plan (804, 950 and 992 operator-anchor pairs in the three seed chunks): the sweep's per-operator `generated` equals an unscoped call of the same operator/anchor/cap, so the scope hook never shortens the partner enumeration (a consistency check against the operator itself, not an independent count); (b) INDEPENDENT counts, computed from plan layout and the documented contracts, match the operators' `generated` exactly for N3 (predecessor + successor in the same drone order; 15 + 37 + 42 anchors in the three chunks) and N6 (one candidate per other trip within 2w half-steps; same anchors). **Not independently measured:** N4-exchange, N5, and the candidate sets of N1, N2, N7, N8. The claim is therefore "complete partner domains verified for N3 and N6, scope-invariant for all"; there is no independent exhaustive partner count for N4-exchange and N5. Drone-trip anchors are rare in the fixtures (about 10 % of swept anchors), which is a stated limitation of this evidence.

