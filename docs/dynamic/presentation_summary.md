# Dynamic truck–drone routing prototype — technical summary for the presentation

**Labels to state on the first slide.** The prototype handles requests by **truck-only insertion** into the editable part of the current plan. The **optimiser is the identity** (no re-optimisation): no optimisation improvement exists or is claimed. Results are one instance, relative, and are demonstrations, not a study. The static baseline (M0–M10, tag `static-baseline-m10`) is frozen and untouched; M9 is validation, not a numerical reproduction of the paper.

## 1. Problem and pipeline
A static truck–drone plan (static VNS) is executed while random delivery requests arrive. Each request is scanned once at arrival, queued FIFO, decided one at a time (each decision takes ω simulated seconds), and either installed into the plan or rejected. Every installed plan is verified by the unchanged static evaluator, which is the only authority on feasibility and objective (Σ service-arrival times).

## 2. Architecture (`include/dynamic/`, header-only, additive)
* **World / InstalledPlan:** immutable world (nodes, customers, FLP map) shared between plans; a plan is an immutable {solution, cached evaluation} bound to its exact world (consistency checked).
* **Scenario:** policy-independent Poisson/uniform requests from dedicated seed streams, so every policy sees the *same* scenario (paired comparison).
* **Cursor / inserter:** clean cut + editable gaps; cheapest evaluator-verified truck insertion over all editable gaps.
* **Scan region** (G0, G3, G3c, none) → **admission** (A0, A-PM) → **simulator** (event loop, ledger, invariants) → **writer / trace / experiment driver**.
* Tests: `test_dyn_core` 64, `test_dyn_slice` 166, `test_dyn_dm5` 256 checks (g++, Linux); CMake option unverified (`windows_cmake_verification.md`).

## 3. Mathematical decisions
* **Clean cut:** the first skeleton position at or after θ = start + ω that no drone trip spans; only gaps behind it are editable. **No editable gap ⇒ `stale`.**
* **Conventions:** C1 every decision (even a rejection) occupies ω; C2 stale still consumes ω; C3 a request whose window is already unmeetable at dequeue is `expired`, dropped, consuming no ω.
* **Promises:** PM1 response window ETA_r ≤ τ_r + W (τ_r = arrival, so queue waiting eats the window); PM2 tolerance ETA'_i ≤ P_i + E for every promised customer (P_i = first promise, base-plan ETA for base customers; no ratchet). W = W̃·L/v_T, E = ε̃·(L/√n0)/v_T. W̃, ε̃, ς are **unfrozen placeholders**.
* **Φ and the U1 chain (approved):** (1) cheapest evaluator-feasible insertion; (2) optimiser (identity); (3) install if Φ holds, else cheapest Φ-feasible insertion (`fallback`); (4) else `rejected-promise`. `rejected-no-insertion` = no evaluator-feasible insertion at all.
* **G3 (approved, unchanged):** union over editable gaps of detour ellipses with bound δ̂_max(g) = min slack over the tail set (+∞ for straddled gaps, U8), plus a tube term. **G2′ ⊆ G3 is tested only for single truck insertions**; new drone trips and re-anchoring are not covered. G3 is not claimed conservative against re-optimisation. **G3c** (tube of ς·ℓ) is explicitly non-conservative. **G3W** (W-aware) is deferred.

## 4. Experiments (details: `dm6_experiment.md`; paired seeds, 10 scenarios, n0=20, regime P)
* **E1 (ω=60 s, W̃=2):** baseline G0/A0 accepts all 21.4 requests; A-PM accepts 7.5–9.7 and rejects 11.7–13.9 on promise. Acceptance is not monotone in ε̃ (path dependence).
* **E2 (W̃=8):** nearly all accepted ⇒ in E1 the response window, not the tolerance, drove most rejections.
* **E3 (ω=600 s, ρ=2):** baseline accepts 42/42 after ~4000 s mean queue wait (no promise to break); A-PM ends with ~5 accepted, 5.3 expired, 8.2 stale, 23.7 rejected-promise: latency eats the window.
* **G3 = G0** outcome-for-outcome in E1/E2 (end gap always editable and delays nobody); differs only when nothing is editable (E3). **G3c** loses 17–41 % of demand to its tube.
* Objectives are not comparable across arms (different accepted sets). Evaluator calls are the deterministic cost proxy; wall-clock is separate.

## 5. Live demo (deterministic)
`docs/dynamic/run_demo.sh <tdr_dynamic> A` — n0=15, regime F (the initial plan contains a drone trip, launch f828 → c7 → recover c4), 13 requests, ω=900 s, W=6000 s, E=2066 s. The trace shows initial plan, arrivals, FIFO waits (up to 2457 s), 2 accepted (r3, r4), 8 rejected-promise (window W violated), 2 stale (truck too far along), 1 out-of-scan, final plan and objective 98669 → 124988 (more customers served, hence higher sum of ETAs). Variant `B` (ω=1200 s, σ=5) adds 3 expired requests. Committed copies: `docs/dynamic/demo/trace_A.txt`, `trace_B.txt`. Wall-clock never appears in traces.

## 6. Findings
1. Without promises (A0) everything is accepted — rejection needs promises, stale (large ω), or a depot-recovered trip.
2. With promises, optimiser latency ω is a first-order driver (queue wait consumes W): rejected/expired/stale counts grow sharply with ω.
3. Tolerance-only G3 is vacuous while the end gap is editable; informative filtering would need a W-aware bound (future G3W).

## 7. Limitations (state them)
Identity optimiser; truck-only insertion (no new drone trips, no re-anchoring); one instance/base plan; 10 scenarios, no confidence intervals; placeholder W̃/ε̃/ς; G2′ coverage limited to the tested move class, Tube term untested; no G3W, DM7–DM9 or scope hook; CMake block not yet verified; scenario digest golden value uses `std::log` (cross-platform caveat).
