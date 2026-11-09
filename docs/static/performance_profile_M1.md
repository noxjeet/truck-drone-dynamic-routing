# `evaluate_solution` profile (M1, 9 Nov 2026)

**Purpose.** The Phase G neighbourhoods will call the evaluator very many times (one call per candidate move). Before building them,
the evaluator was profiled to find where its time goes. **No change was made to `timing/timing_engine.hpp` in M1** (P1/P2 were applied afterwards, see the follow-up document). Any optimisation is a
proposal below and needs approval; it must be differential-tested against the current evaluator.

## Method

* Workload: the initial solution produced by the existing builder for a seeded uniform instance (base seed 7, 60 x 60 km, depot at the centre,
  $\delta=2000$ m, 2 drones), evaluated 3000 times in a loop. Release-style build (`-O2`), single thread, one VM; run-to-run noise is
  roughly $\pm10\%$ (two runs of the same $c=50$ case gave 281 and 335 us), so figures are rounded.
* `perf` is unavailable in this VM (kernel `perf_event_paranoid`), so two independent methods were used:
  1. **Ablation:** evaluate the identical solution against a registry holding only the depot, the customers and the FLPs the solution uses
     (13 / 30 / 73 nodes) instead of the whole eager pool (1,901 / 6,873 / 39,451 nodes). The objective was bit-identical in all three cases.
  2. **Section timers:** a scratch copy of the evaluator (outside the repository) with a `steady_clock` read between its 12 internal sections.

## Results

| Customers $c$ | Trips | Route length | Registry nodes (full pool) | Full registry | Registry reduced to used nodes | Ratio |
|---:|---:|---:|---:|---:|---:|---:|
| 10 | 2 | 12 | 1,901 | 8.1 us | 1.4 us | 5.8 x |
| 20 | 9 | 22 | 6,873 | 29 us | 5.4 us | 5.4 x |
| 50 | 22 | 52 | 39,451 | 281--335 us (~3,000--3,500 evals/s) | 9.0--11.9 us | 28--31 x (paired runs) |

Section breakdown (scratch timers, same workload):

| Section of `evaluate_solution` | $c=10$ | $c=50$ |
|---|---:|---:|
| **Instance-data validation: scan of every registry node to list the network's customers (Stage 1 check "every network customer has CustomerData")** | **6.8 us (84%)** | **306 us (96%)** |
| route id existence | 0.06 us | 0.4 us |
| depot / duplicate-customer checks | 0.2 us | 1.6 us |
| FLP arc membership, (8)/(9) | 0.1 us | 1.5 us |
| per-trip position lookup (linear `find_route_position`) | 0.2 us | 2.7 us |
| rule (6) | 0.05 us | 0.2 us |
| forward pass (truck and drone timing) | 0.2 us | 1.2 us |
| per-trip feasibility assembly | 0.05 us | 0.4 us |
| (28) and $\lvert U\rvert$ (quadratic in trips) | 0.2 us | 1.5 us |
| customer service resolution (customers x trips) | 0.15 us | 3.1 us |
| synchronisation records, objective | 0.06 us | 0.4 us |
| total of the above | 8.0 us | 319 us |

## Findings

1. **One loop dominates.** In the Stage 1 input validation, the check that every customer *node of the network* has a `CustomerData`
   entry iterates over the entire registry (`for (kv : registry.all()) if (type == Customer)`). The registry contains the whole eager FLP pool
   in an `unordered_map`, so each call touches $\approx 4\times10^4$ cache-unfriendly hash nodes to find 50 customers. Its cost grows with the
   pool, not with the solution. Everything else together costs about 13 us at $c=50$.
2. This is a consequence of two correct decisions made earlier (Stage 1 validation; the eager pool). It is not a bug, and results are unaffected.
3. After the scan is removed the next costs are small and linear/quadratic in the (small) number of trips and customers: ~3 us for the customer
   resolution loop and ~3 us for position lookups at $c=50$. They matter only after item 1 is fixed.
4. The pool is large because $\delta=2000$ m on a 60 km square yields about 11 x the FLP count of the paper's Table 4. A larger $\delta$ would shrink the
   pool and the cost, but would change the search space; $\delta$ is not to be tuned for speed.
5. Separate from speed: the objective is accumulated over an `unordered_map` iteration, so the floating-point summation order is not
   specified (already recorded as a known item after Stage 2). The effect is at the $10^{-12}$ relative level, but it makes bit-for-bit
   reproducibility across standard-library versions, and tie-breaking of near-equal moves, depend on hash-table internals.

## Proposal (status: P1 and P2 APPLIED in the follow-up --- see `performance_followup_P1_P2.md`; P3 not applied)

* **P1 (speed, behaviour-preserving).** Build the sorted list of customer ids once, where the registry or the network is built, and have the validation read it
  instead of scanning. Expected cost at $c=50$: from ~300 us to roughly the "reduced registry" figure, 10--15 us, i.e. an upper bound of about 28--31 x.
* **P2 (reproducibility).** Sum the objective in ascending customer-id order (use the already-sorted id list).
* **P3 (later, only if profiling after P1 justifies it).** Replace the linear route-position searches and the customer x trip loop by single-pass indices.
* **Safety protocol.** Keep a frozen copy of the current evaluator as a reference oracle (test-only), run both on the fuzzed solutions of
  `tests/test_fuzz_search_infra.cpp` plus the existing fixtures, and require identical `feasible`, `violation_reasons`, per-trip timings and
  objective (bitwise for P1; for P2 the objective may differ in the last bits and the comparison becomes exact against the same ordered sum).
  All 619 existing assertions must still pass unchanged.
* **Why not now.** The instruction for M1 was to profile without prematurely rewriting. The Phase G neighbourhoods are correct regardless of speed; P1 should
  be applied before the first neighbourhood benchmark (M2), not after.

## Reproduction

Scratch programs are kept outside the repository (`~/build_m1/prof/prof_eval.cpp`, `prof_sec.cpp`); they construct the workload with the
repository headers only, so they can be re-created from the description above.
