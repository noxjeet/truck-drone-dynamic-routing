# M1 performance follow-up: P1 (customer-id cache) and P2 (ordered objective summation)

Date: 9 Nov 2026. Scope: the two proposals of `performance_profile_M1.md`, authorised by the "M1 performance follow-up, then M2" instruction.
**Status: P1 accepted and applied; P2 accepted and applied (with the behavioural consequences listed in §4). Nothing committed or pushed.**

Legend used below: **[FACT]** measured or verified here; **[DECISION]** a choice made in this follow-up; **[ASSUMPTION]** an interpretation not fixed by the paper.

## 1. Inspection (before any edit)

* **Source of truth for customer ids [FACT].** `SpatialNetwork` holds its `NodeRegistry` **by value** (`nodes_`), exposes it only through `const NodeRegistry& nodes() const`, and has no mutator.
  `NodeRegistry` is a plain `unordered_map<NodeId, Node>` whose only mutator is `add()`. Consequently the set of nodes of a `SpatialNetwork` is fixed at construction, and the registry passed
  to the constructor is **copied**, so mutating the caller's registry afterwards cannot reach the network.
* **The scan [FACT].** In `evaluate_solution`, the Stage-1 check "every network customer has CustomerData" iterated `registry.all()` (the whole eager FLP pool, ~39,400 nodes at 50 customers) on every call,
  then sorted the result. Objective: `for (kv : customers)` over an `unordered_map`, i.e. the summation order followed the hash table.
* **Nothing was partially implemented** before this work (the evaluator was byte-identical to HEAD `cc37e49`; sha256 `f99ef35aef77d96f34e9cc892060bb4dd4cf6401b5cd0f9521171485873930b8`).

## 2. P1 --- immutable construction-time cache [DECISION]

* `SpatialNetwork` now builds `customer_ids_` (ascending vector of the ids whose type is `Customer`) in its constructor and exposes it as `const std::vector<NodeId>& customer_ids() const`.
  Files: `include/graph/spatial_network.hpp` (+2 includes, +11 lines, +1 member).
* `evaluate_solution` iterates `network.customer_ids()` instead of scanning and sorting the registry (`include/timing/timing_engine.hpp`, one hunk, -6/+4 lines). Same loop body, same order, same messages.
* **Why it cannot go stale.** (i) The registry is a by-value member with no mutator; (ii) the cache is computed from that member in the constructor and never written again; (iii) copy/move construction and assignment copy
  or move both members together (compiler-generated, so they cannot diverge); (iv) a network with different nodes (e.g. the builder's augmented network) is a *new object* with its own cache. No static or global state is introduced.
  No invalidation logic is therefore needed, and none exists to get wrong.
* Behaviour is preserved: same validation rules, same violation texts, same order (verified bitwise, §5).

## 3. P2 --- ascending-id summation [DECISION]

* The `CustomerData` keys are sorted once already (for the deterministic validation report). The sorted vector is now hoisted out of the validation block and the customer-resolution loop iterates it, so `customer_records` is
  produced in ascending customer-id order and the objective is the left-to-right sum of those records (`include/timing/timing_engine.hpp`, three small hunks).
* The mathematical objective is unchanged: sum over customers of the serving vehicle's arrival time; the end-depot arrival is not included (mutation M9 below confirms the tests detect adding it).
* **Observable consequences [FACT]**, all confined to ordering and to rounding:
  1. `customer_records` is now in ascending customer-id order (before: hash-table order). No production code and no existing test depends on the order (the builder looks records up by id).
  2. The *order* of customer-level entries in `violation_reasons` follows the same rule (before: hash-table order). The multiset of messages is identical.
  3. `objective_value` may differ in the last bits from the old value (§5).
* **Why it is worth having [FACT].** With the old code the objective of the *same* solution and the *same* data depended on the insertion history/bucket count of the `CustomerData` map:
  in 374 of 1,061 test cases the reference objective changed bitwise when the identical map was rebuilt in another insertion order (production after P2: 0 of 1,061). That is a reproducibility defect across standard
  libraries and, for the later dynamic work (customers appended over time), across runs.

## 4. Does P2 change candidate comparisons? (the question the instruction asks)

Method: for every pair of solutions evaluated on the same instance with content-identical `CustomerData` (11,922 pairs, 1,904 of them feasible--feasible), compare the sign of (objective_i - objective_j) under (a) the reference
evaluated under 6 different map insertion orders, (b) production, and (c) **exact real-number arithmetic** (Shewchuk partial sums on the per-customer times; sign is exact).

| Quantity | Result |
|---|---:|
| Pairs whose *reference* ordering depends on the map insertion order | 1,122 of 11,922 |
| Sign flips reference(natural order) -> production | **93** (45 feasible--feasible) |
| ... of which the reference ordering was unstable under map order | 93 (100%); reference-stable flips: **0** |
| ... of which the two solutions are exactly equal as real numbers | 93 (100%); production reports a tie (gap 0) |
| Pairs where the reference sign disagrees with the exact sign | 93 |
| Pairs where the production sign disagrees with the exact sign | **0** |
| Largest absolute objective difference production vs reference | 6.98e-10 s (relative 9.64e-16) |
| Objectives that differ in at least one bit | 594 of 1,558 cases |
| Rigorous bound used by the test: 2 n eps sum\|t\| | max observed \|diff\| = 0.154 x bound |

**Reading.** Every changed comparison is a pair that is *mathematically tied*; the reference declared a spurious winner (by ~1e-10 s) that depended on hash-table state, production declares the tie that exact arithmetic gives. No pair that the
reference ordered reproducibly changed. No tolerance was added anywhere. So P2 changes no *well-defined* comparison; it removes an arbitrary one. [FACT, on this sample; it is a sample, not a proof, and the proof of the bound is in the test.]

**Caveat carried to M7 [open item, no action taken].** A strict-improvement test `f(new) < f(best)` on floating-point objectives will still treat sub-nanosecond differences between *different but equal-valued* solutions as real differences
(production now gives the exactly-rounded answer in the sampled data, but equal-valued solutions that sum different multisets in a different order could still differ in the last bit). The VNS acceptance rule is to be fixed in M7 and
the question "strict improvement vs improvement by more than a stated threshold" should be decided there explicitly; M2 does not need it.

**Reverting P2 is a three-hunk change** if you prefer not to accept it; the P1 gain does not depend on it (benchmark C vs D, §6).

## 5. Differential validation

* **Frozen reference** `tests/reference/timing_engine_reference.hpp` (711 lines): the evaluator at HEAD `cc37e49`, namespace renamed to `tdr::timing_ref`, otherwise verbatim (script-checked: `sed` of the namespace
  back reproduces `include/timing/timing_engine.hpp@cc37e49` exactly; reference sha256 `38b9fef7ba34b2c79066471582d95f0071c79342ed03c957f11b8722a288a03a`). Test-only; never included by production code.
* **Test** `tests/test_evaluator_differential.cpp` (CTest name `test_evaluator_differential`, **26,745 assertions**, ~4 s Release). Fixed base seed **31415**; instance `i` of the small set uses the `Instance` stream index `i`, edits use the `Fuzz` stream
  (`derive_seed(31415, Fuzz, index)`).
  * 40 small instances (3--14 customers, 1--3 drones) x 40 edits, plus 3 seeds each at n = 10, 20, 50 x 25 edits, **1,558 solutions** (431 feasible, 1,127 infeasible).
  * Edit kinds: relocate, drone-to-truck, slice reversal, stretched recovery (overlapping flights), dropped trip, **simultaneous recoveries** (210 cases), **all-truck** (299 trip-free cases), duplicated trip, corrupted drone id, swapped launch/recovery, unedited builder solution.
    951 cases with >= 2 trips; 96 with a drone id > 0 after `assign_drone_ids_by_route_order`.
  * `CustomerData` tampering (38% of cases): missing entry (92 cases hit the P1 path "network customer has no CustomerData"), entry for an unknown id, entry keyed by an FLP id, `node_id != key`, same content in another insertion order.
  * Hand-built edge cases: depot-only network with routes `{}`, `{0}`, `{0,0}`; customers without any CustomerData; empty route with full data; all-truck solution.
  * Compared field by field: feasibility, `max_simultaneous_drones`, every truck arrival/departure (bitwise), every drone-trip timing field (bitwise, incl. reason strings), synchronisation records, customer records (as an id-keyed set, bitwise), violation reasons (as a multiset),
    objective (within the rigorous bound, plus bitwise equality with the left-to-right sum of the records).
  * P1 unit tests: cache == sorted scan; FLP pool excluded; ascending; stable address/content over repeated access and evaluation; the source registry mutated after construction does not reach the network; copy, move and assignment keep cache and registry consistent; read-only exposure (`static_assert`); ascending, once-only "no CustomerData" reports identical to the reference.
* **Strict mode** (`--strict`) asserts *bitwise identity of everything*, including order and objective. It passes (0 differences, 23,495 assertions) against the **P1-only** evaluator (saved before P2 was applied;
  sha256 `e41db6035e634ec6257537b91634c901fd4857bca290a3b1f3ace8c86fdf52ee`), which proves P1 alone changes nothing observable. The default mode is the P1+P2 contract described above.
* **Mutation checks of the new tests** (injected into a scratch copy, never into the repo): 13 mutants, **all killed** --- cache drops the last customer; cache descending; cache includes FLPs; cache includes the depot; cache built from an empty registry; P2 reverted to map iteration;
  objective skips drone-served customers; objective summed in descending order; objective plus depot return; objective accumulated in long double; P1 path skips the missing-data report; missing-data reports in descending order; records in descending order.
  (One mutant, "objective summed in descending order", *survived* the first version of the test --- it is within the bound and deterministic --- which exposed that the P2 summation-order contract itself was not asserted; the assertion "objective == left-to-right sum of the records, bitwise" was added and the mutant is now killed.)

## 6. Benchmark

* Program `bench/bench_evaluator.cpp`, built only with `-DTDR_BUILD_BENCH=ON` (not in CTest, not in the default build). Variant C needs `-DTDR_BENCH_P1_HEADER=<saved P1-only header>`.
* **Variants:** A = frozen reference on the full eager-pool network (the pre-follow-up behaviour); B = the same frozen reference on a network whose registry holds only the depot, the customers and the FLPs the solution touches ("used-node registry");
  C = P1 only; D = P1 + P2 (production). B is a valid equivalent comparison (bit-identical objective, checked before timing).
* **Identical inputs:** per size 3 instances (base seed 31415, dataset index 5000 + 10 n + k, 2 drones), two solutions each (the builder's initial solution with drone trips, and the all-truck solution); all four variants evaluate the same objects.
  Equivalence is checked before timing: A, B, C bitwise equal objective and verdict; D within the rigorous bound ("ALL VARIANTS EQUIVALENT").
* **Method:** g++ 11.4, CMake Release `-O3 -DNDEBUG -Wall -Wextra -Wpedantic`, 2-core VM, single thread. 50 warm-up calls, batch size calibrated to ~15 ms per variant, 15 repetitions, variants interleaved round-robin inside each repetition (drift hits all alike).
  Reported per instance: median over repetitions, min..max, relative spread (max-min)/median. The table gives the median of the three per-instance medians. Raw output: `perf_followup_bench_raw.txt`.
* **Noise.** Within-instance spread is 15--50%; instance-to-instance medians agree within ~10%. Differences below ~10% (notably C vs D) are **not significant**.

| n | solution | A full registry | B used-node registry | C P1 only | D P1+P2 | A/D |
|---:|---|---:|---:|---:|---:|---:|
| 10 | initial (1--3 trips) | 8.04 us | 1.58 us | 1.46 us | 1.44 us | 5.6 x |
| 10 | all-truck | 7.14 us | 0.94 us | 0.88 us | 0.89 us | 8.0 x |
| 20 | initial (7--9 trips) | 29.07 us | 3.50 us | 3.36 us | 3.24 us | 9.0 x |
| 20 | all-truck | 29.47 us | 2.13 us | 1.80 us | 1.90 us | 15.5 x |
| 50 | initial (21--23 trips) | 345.5 us | 11.21 us | 10.52 us | 10.46 us | 33 x |
| 50 | all-truck | 330.8 us | 5.69 us | 5.16 us | 4.98 us | 66 x |

Conclusions [FACT]: P1 removes the registry-size term and reaches (slightly beats) the used-node-registry figures without restricting the registry, so no caller has to build a reduced registry; **P2 costs nothing measurable** (C vs D within noise, D is not slower).
At n = 50 the evaluator now sustains roughly 95,000 evaluations/s on a populated solution versus ~2,900 before.

## 7. Sign-off gate

| Gate item | Status |
|---|---|
| All existing tests pass unchanged | yes: 26 + 43 + 59 + 291 + 29 + 171 + 144 + 11,262 (no assertion edited) |
| New differential tests pass | yes (26,745 assertions; strict mode vs P1-only 23,495) |
| ASan + UBSan | yes, 9 of 9 programs, leak detection on |
| Compiler warnings | 0 (`-Wall -Wextra -Wpedantic`) for every target including the bench |
| Numerical differences and effect on candidate selection documented | §4 |
| Performance recorded | §6 and `perf_followup_bench_raw.txt` |
| `MILESTONES.md`, decision/report documents updated | yes |

## 8. Files touched by the follow-up

Changed: `include/graph/spatial_network.hpp`, `include/timing/timing_engine.hpp`, `CMakeLists.txt` (+1 test target, +optional bench target). Created: `tests/reference/timing_engine_reference.hpp`, `tests/test_evaluator_differential.cpp`, `bench/bench_evaluator.cpp`,
`docs/static/performance_followup_P1_P2.md`, `docs/static/perf_followup_bench_raw.txt`.
