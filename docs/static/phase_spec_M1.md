# Phase specification: M1 (infrastructure for Phases G--J)

Scope: the minimum shared infrastructure that the eight neighbourhoods, four shakes, the VNS driver and the experiment runner all need.
No neighbourhood, shake or VNS logic is included. **No feasibility logic is added anywhere:** timing, endurance, SL/SR, (28), $\lvert U\rvert$ and the
objective remain exclusively in `tdr::timing::evaluate_solution`.

## Module contracts

| Header | Namespace | Responsibility | Key guarantees |
|---|---|---|---|
| `util/rng.hpp` | `tdr::util` | portable xoshiro256\*\*/SplitMix64, `derive_seed`, unbiased `uniform_int`, `shuffle` | bit-identical on every platform; no `<random>` distributions |
| `include/config/static_config.hpp` | `tdr::config` | `StaticConfig` (instance, vehicle, FLP $\delta$, neighbourhood window/repeats, seed) and `validate()` | every default tagged [PAPER]/[ASSUMPTION]; `validate` never throws |
| `instance/static_instance.hpp` | `tdr::instance` | `generate_static_instance(config)` $\to$ `StaticInstance` (registry, customers, `CustomerData`, vehicle parameters) | deterministic; ids: depot 0, customers $1..c$; fixed draw order (x then y); growing $c$ never reshuffles earlier customers |
| `flp/flp_arc_index.hpp` | `tdr::flp` | `FlpArcIndex`: $L_{ab}$ in position order, `arc_of(f)` | built from the same ownership map the evaluator uses; order independent of hash iteration |
| `search/drone_assignment.hpp` | `tdr::search` | `assign_drone_ids_by_route_order` (moved from `init`) | body unchanged; old name forwards |
| `search/route_ops.hpp` | `tdr::search` | skeleton and gaps, `flps_in_gap`, `flps_off_their_arc`, trip lookups, `classify_service`, `remove_from_route`, `insertion_position_for_gap`, `erase_trip`, `unused_flps_in_route`/`drop_unused_flps`, `lowest_free_drone_for_trip` | report-not-repair; the only clean-up (`drop_unused_flps`) is explicit and returns what it removed; other trips' drone ids are never touched |
| `search/candidate_window.hpp` | `tdr::search` | `PositionIndex`: half-step coordinates, `adjacent`, `neighbours`, `gaps_within` | implements math_spec §7; deterministic ordering (distance, then id) |

## Design rules carried into Phases G--J

1. A neighbourhood move = (mechanical edit via `route_ops`) + (explicit handling of every FLP/trip the edit invalidated) + (`assign`/`lowest_free` for drone ids) +
   (one call to `evaluate_solution`). A candidate is accepted only if the evaluator reports it feasible.
2. An edit that leaves a launch FLP on a foreign arc is not repaired silently. The caller either dissolves the affected trips (recorded and counted, decision D6),
   re-anchors them by a documented rule, or rejects the candidate.
3. `unused_flps_in_route` is defined by *naming* (no trip names the FLP). The evaluator's rule (6) additionally counts FLPs whose only trips are unplaced; the two
   coincide whenever all trips are placed, and the helper never reports more (tested). A broken trip is therefore never hidden by clean-up.
4. The FLP pool stays eager (decision: no lazy cache, no capacity-based id formula while the dynamic work is on hold).

## Not in M1

Neighbourhoods N1--N8, shakes, VNS, result writer, CLI/file configuration parsing, evaluator optimisation (proposal only), fixed-launch comparison mode.
