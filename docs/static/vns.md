# VNS driver (M7)

Status: contract (sections 1--5) written **before** implementation (9 Nov 2026, M7 instruction of 13:56 IST). Implementation, validation, pilot calibration and findings are appended after implementation (sections 6--9).
Related: `shakes.md` (M6, signed off), `operators_N*.md`, `DECISIONS.md` D2, D6, D7, D8 and the M7 items, `paper1_decision_register.md`.

## 0. Sources

Paper text: `claude/han-chi-sun-2026-extraction.md` (Algorithm 1 transcribed from a 250 dpi crop; Sec 5 text; Sec 6.1 execution order). The original PDF is not in the project. Existing interfaces used unchanged: `SearchContext`, the eight randomised operator entry points `n1..n8_*(base, ctx, rng, options)` (each returns an `OperatorResult` whose `best` is the lowest-objective evaluator-feasible candidate, strict `<`, first wins ties), `shake()` (M6), `classify_evaluation`, `util::Rng`, `derive_seed`, `SeedStream::{Vns, Shake}`, `NeighbourhoodConfig::{adjacency_window, n1_repeats, n2_repeats}`.

## 1. Paper facts

Algorithm 1: $x\leftarrow$InitialSolution; $T\leftarrow T_x$; `iter` $\leftarrow 1$; repeat: $x'\leftarrow$RandomShake$(x)$; $k\leftarrow1$; **while** $k\le|N|$: $x''\leftarrow N^{(k)}(x')$; **if** $f(x'')<f(x)$ then $x\leftarrow x''$, $k\leftarrow1$; **else if** Random$(0,1)\le\exp((f(x)-f(x''))/T)$ then $x\leftarrow x''$, $k\leftarrow k+1$; **else** $k\leftarrow k+1$; $T\leftarrow$update$(T)$; **until** `iter` $>$ `iter_max`. Text: the shake "updates by randomly selecting a shake neighborhood" (one of four); the local search is "a sequential search based on a predefined list of neighborhoods"; the new solution "is retained ... only if ... superior to the current optimal solution or satisfies the SA criteria". Sec 6.1 execution order: truck service node relocate (N4), T-D swap (N5), drone-node-delete/truck-node-insert (N8), drone service node relocate (N3), D-D swap (N6), truck-node-delete/drone-node-insert (N7); launch-node relocate (N1) and recover-node-relocate (N2) are run "several times". Each neighbourhood is a best-improvement search over its candidate set.

**Not given by the paper:** $T_x$, the definition of update$(T)$, `iter_max`, any further stopping criterion, where N1/N2 sit in the order and how often, the random-number generator and seeds, what happens when a shake cannot act or yields an infeasible solution, any best-so-far bookkeeping.

**Printed-pseudocode defects (D8):** (i) `iter` is never incremented (the loop would not terminate); (ii) line 7 always searches from $x'$, so an accepted $x''$ is never the next search point; (iii) no best-so-far is kept although SA acceptance lets the current solution worsen.

## 2. Approved decisions that apply

| ID | Content |
|---|---|
| D2 | N1 and N2 are each repeated twice per pass (`NeighbourhoodConfig::n1_repeats/n2_repeats`, default 2) |
| D6 / M6 (A19--A29 approved) | uniform shake choice among four; shakes dissolve only invalidated trips; the (possibly infeasible) result is returned as judged; `NotApplicable` without redraw inside `shake()` -- **M7 decides the policy (section 3)** |
| D7 | $T_0$, cooling, `iter_max`, no-improvement limit are configurable and calibrated in pilot runs, **not** tuned to the paper's numbers |
| D8 | correct iteration counter; search from the current accepted solution; best-so-far kept separately; accepted improvements carried forward; deviations documented |
| strict `<` | the evaluator's raw objective, no epsilon, first generated wins ties (operators); a threshold may be *considered* in M7 |
| rules | the evaluator is the only feasibility judge; no silent repair; all M1--M6 operator and evaluator semantics unchanged |

## 3. Contract (what the driver does)

**State.** $x$ = current accepted solution with $f(x)$; $w$ = search point of the local search ($w:=x'$ right after a shake); `best` = best-so-far solution (separate copy, $f_{best}$). Every stored solution is evaluator-feasible with the evaluator's objective (the operators and shakes return judged results; the driver never evaluates anything itself except the initial solution).

**Iteration (one outer pass; `iter` counts 1..`iter_max`, incremented exactly once per pass, skipped passes included):**
1. **Shake** from $x$ with the Shake stream: `shake()` (uniform among the four, M6). *NotApplicable* or *evaluator-infeasible* results are **rejected, never repaired**, and the shake is **redrawn from the same $x$** (a fresh uniform draw, i.e. uniform among the shakes that act and give a feasible result), up to `max_shake_attempts` (default 10) per iteration, all attempts counted by kind. If no attempt gives a feasible shake the iteration is **skipped**: $x$ is unchanged, the iteration still counts, the temperature still cools, and `iterations_skipped` is incremented.
2. **Local search** over the neighbourhood list $N$ (index $k=1..|N|$) with $w=x'$ at the start. Each $N^{(k)}$ is one call of the operator's randomised entry point on $w$ (the operator draws its anchor from the Vns stream, as implemented in M2--M5; its `best` is the best evaluator-feasible candidate). A call without candidate (no eligible anchor, or no feasible candidate) is a non-move: $k\leftarrow k+1$, no acceptance draw. Otherwise with $z$ = `best->solution`, reference value $r$ (below):
   * **improvement** $f(z)<r-\delta$ ($\delta$ = `min_improvement`, default 0 = strict `<`): $w\leftarrow z$, $k\leftarrow1$ (accepted improvements are carried forward: the next neighbourhood searches from $z$);
   * else draw $u$ = `uniform01()` (always one draw per non-improving candidate, so RNG consumption does not depend on $T$); if $u\le\exp((r-f(z))/T)$ then $w\leftarrow z$, $k\leftarrow k+1$ (an exact tie has probability 1 and is accepted as a lateral move); else $k\leftarrow k+1$.
   * Whenever $w$ changes (and for the shaken solution itself) the best-so-far is updated if $f(w)<f_{best}$ (strict).
3. **Reference $r$ and the incumbent (assumption A30).** `AcceptanceReference::Incumbent` (**default, the printed Algorithm 1**): $r=f(x)$ and an accepted $z$ also becomes $x$ immediately, so $x$ and $w$ coincide after the first acceptance; if nothing is accepted the shake is discarded and $x$ is unchanged. `ShakenState` (alternative, selectable, pilot-compared): $r=f(w)$ and $x\leftarrow w$ at the end of the iteration (the shaken-and-searched solution is the next incumbent).
4. **Cooling.** $T\leftarrow\max(T_{min},\,\alpha T)$ once per iteration (A31: update$(T)$ is geometric, $\alpha$ = `cooling_rate`).
5. **Termination** is checked after the cooling: `IterMax` (always), `NoImprovement` (iterations since the best-so-far last improved $\ge$ `max_no_improve`; 0 = off), `TimeLimit` (wall-clock `max_seconds`; 0 = off, the default; the only non-reproducible criterion, reported in the result).

**Neighbourhood list (A32).** Default: N4, N5, N8, N3, N6, N7 (Sec 6.1 order), then N1 and N2 alternately `n1_repeats`/`n2_repeats` times (default N1, N2, N1, N2; $|N|=10$). Configurable (`VnsOptions::neighbourhoods`). Improvement restarts at the first entry, which repeats N1/N2 more often in practice ("several times").

**Temperature (A33).** $T_0=\max(T_{min},\,t_0\!\_fraction\cdot f(x_{init}))$ so that the scale follows the objective, or the absolute `t0_absolute` if $>0$. Defaults for $T_0$, $\alpha$, `iter_max` come from the pilot (section 8), are labelled as ours and are not tuned to the paper's numbers.

**Seeds and reproducibility (A34).** Two independent streams derived from one base seed and the run index: Vns $=$ `derive_seed(base, SeedStream::Vns, run)` (acceptance draws and every operator's anchor draw), Shake $=$ `derive_seed(base, SeedStream::Shake, run)` (everything inside `shake()` incl. redraws). No other randomness, no hash-order dependence, no wall-clock input unless `max_seconds`$>0$: same instance, initial solution, options, base seed and run index give a bit-identical result and trace on any platform. `run_index` 0..9 realises the paper's ten runs.

**Initial solution.** Must be evaluator-feasible (`InitialInfeasible` otherwise, nothing run). The driver never repairs.

**Output (`VnsResult`).** best solution/objective, final current solution/objective, $T_0$, final $T$, iterations completed, stop reason, shakes (feasible applied by kind, not applicable, infeasible), skipped iterations, per-operator statistics (calls, no anchor, no candidate, improving, SA-accepted, SA-rejected), improvements of the best and the iteration of the last one, wall-clock seconds, optional per-iteration trace.

## 4. Deviations from the printed pseudocode (all D8)

1. `iter` increments once per pass (the printed loop never terminates). 2. After an acceptance the next neighbourhood searches from the accepted solution (printed: always from $x'$). 3. A separate best-so-far is returned (printed: none; $x$ may be worse than the best seen). 4. Rejected (not applicable / infeasible) shakes are redrawn and a pass without a feasible shake is skipped (the paper is silent). 5. Added termination criteria beyond `iter_max` (the text hints at "stopping criteria"). The comparison against $f(x)$ and the SA test are otherwise as printed.

## 5. What the contract rests on

| Class | Item |
|---|---|
| **Paper fact** | Algorithm 1 structure, sequential neighbourhood list, restart on improvement, SA acceptance with $\exp((f(x)-f(x''))/T)$, one temperature update per outer pass, uniform-by-wording random shake, Sec 6.1 order, N1/N2 "several times" |
| **Paper silence** | $T_x$, update$(T)$, `iter_max`, other stopping criteria, N1/N2 placement, seeds, not-applicable/infeasible shakes, best-so-far |
| **Approved decision** | D2, D6 (+ M6 A19--A29), D7, D8, strict `<`, evaluator-only feasibility, no repair, eager FLP pool |
| **Implementation assumption (approved at the M7 sign-off)** | **A30** acceptance reference $f(x)$ (printed) with the shaken solution as the initial search point; `ShakenState` offered as alternative. **A31** geometric cooling with floor. **A32** neighbourhood list = Sec 6.1 order + N1/N2 alternately repeated. **A33** $T_0$ as a fraction of $f(x_{init})$ (or absolute); defaults from the pilot. **A34** two seed streams (Vns, Shake) by run index; ties accepted by the SA test; one acceptance draw per non-improving candidate. **A35** rejected shakes redrawn up to `max_shake_attempts`, else the pass is skipped. **A36** an operator call without feasible candidate is a non-move (no draw); termination criteria `iter_max`, no-improvement, optional wall clock. **A37** improvement threshold $\delta$ exists but defaults to 0 (strict `<`). |
| **Deviation from the paper** | section 4 (all D8, documented) |
| **Unresolved** | section 9 |

## 6. Implementation

`include/search/vns.hpp` (new, header-only; no existing source file changed). `run_vns(initial, ctx, VnsOptions)` returns a `VnsResult`; `default_neighbourhood_list(n1_repeats, n2_repeats)` builds the list. The driver evaluates only the initial solution; every other stored solution is the evaluator-judged result of `shake()` or of an operator's `best`. Two `Rng` streams: Vns (operator anchors + acceptance draws) and Shake (inside `shake()`, redraws included). **One guard added beyond section 3 (assumption A38):** `max_local_steps` (default 1000 operator calls per pass, 0 = unlimited; truncations counted in `passes_truncated`): at a very high temperature non-improving moves can alternate with improvements indefinitely, which the printed pseudocode would also do; the unit tests found this (a test with $T=10^{30}$ did not terminate) and with $T_0\le 10^{-1}f(x_{init})$ the guard never triggered in the pilots. Defaults `iter_max = 2000`, `t0_fraction = 0.01`, `cooling_rate = 0.995` come from the pilot (section 8) and are labelled ours (D7). Config-file wiring of `VnsOptions`, the runner and the CLI are M8 (section 10 gives the A38 reporting added at the M7 sign-off).

## 7. Validation

| Check | Result |
|---|---|
| `test_vns` (options, list construction, statuses, iteration counting, temperature schedule, skipped passes when no shake can act, determinism and run index, pure descent, hot SA, threshold, `ShakenState`, custom list, stopping, statistics consistency, frozen-reference re-judgement of best and current) | **85 assertions, 0 failures** (`vns_unit_tests_raw.txt`) |
| `test_vns_oracle` (100 seeded instances, 309 VNS runs, 4,032 passes; independent re-implementation of the loop with its own temperature, acceptance, best-so-far arithmetic and seed streams; exact comparison of every pass (T, f(x), f(best), shake kind, moves), final solutions and every counter; every accepted state re-judged by the frozen reference; second full run for the digest) | **29,447 checks, 0 failures**, non-vacuity passed (all four shake kinds, NotApplicable redraws, infeasible redraws, skipped passes, improving and SA-accepted moves, step-guard and no-improvement stops, `ShakenState`) (`vns_oracle_stats_raw.txt`) |
| Mutation (`tests/mutation/m7_mutate.py`, 52 single-bug mutants of `vns.hpp`) | **51 killed, 1 equivalent survivor** (V-10: `u < p` instead of `u <= p` differs only when the uniform draw equals $p$ exactly, a measure-zero event since `uniform01` is in $[0,1)$ and ties give $p=1$). V-39 (N4 options ignored) first survived because every test used the defaults; an oracle variant with a non-default `N4Options` was added and kills it (`m7_mutation_results_raw.txt`) |
| Fresh Release build, `-O3 -DNDEBUG -Wall -Wextra -Wpedantic` | 0 warnings; ctest **21 of 21** (`m7_release_ctest_raw.txt`) |
| ASan + UBSan | `test_vns`, `test_vns_oracle` 0 reports (`m7_sanitizer_raw.txt`) |
| Determinism | oracle output byte-identical over 3 runs, unit tests over 2 runs, digest equal on a full second oracle run |
| Regression | N1--N8 and shake benchmark counts identical to the M6 end (`*_after_m7_raw.txt`); all 15 guard-file sha256 identical (evaluator, frozen reference, all operators, helpers, `static_config.hpp`, `move_types.hpp` untouched); all earlier tests unchanged and passing |

Notes: the oracle takes the initial objective from the production evaluator, because the production and the frozen-reference objectives differ in the last bit by the documented P2 ordering effect (the oracle also asserts they agree within the bound); the driver's own comparisons use the production evaluator only.

## 8. Pilot calibration and measurements (D7; `bench_vns`; base seed 31415, dataset 0, initial solution of the pipeline, window 3; one instance per size)

Raw: `vns_pilot_n{10,20,50}_raw.txt` (grids), `vns_curve_n{10,20,50}_raw.txt` (best-so-far at checkpoints), `vns_default_runs_raw.txt` (final defaults). *Gain* = relative decrease of the best-so-far objective from the initial solution. These are pilot observations on one instance per size, **not** a comparison with the paper's Table 3 and not tuned to it.

* **Acceptance reference (A30).** The printed reference $f(x)$ (`Incumbent`) is far better at the larger sizes: gain at 300 passes (n = 20): 10.5--14.6 % vs 1.3--4.9 % for `ShakenState`; at 1000 passes (n = 50): 19.6--19.7 % vs 0.1--2.7 %. At n = 10 both reach 11--14 % and `ShakenState` is slightly ahead at 300 passes. Default: `Incumbent`.
* **$T_0$ (as a fraction of $f(x_{init})$).** $10^{-4}$..$10^{-2}$ are equivalent at n = 50 (19.6--19.7 %); $10^{-1}$ is clearly worse (7.2 % at n = 50, 10.5 % at n = 20): too hot. Default $10^{-2}$ (keeps some SA activity at small sizes: n = 10 shows 110 SA-accepted moves per run at the defaults). At n = 50 the annealing is nearly inactive for $T_0\le10^{-2}$ (35 SA-accepted moves per 2000-pass run, 53 improving ones).
* **Cooling $\alpha$.** 0.90, 0.95, 0.98 identical at n = 50; at n = 20 (10 runs, 5000 passes) 0.995 ends higher (21.2 % vs 20.7 %), at n = 10 both reach 14.8 %. Default 0.995 (so $T$ is still $>10^{-3}T_0$ after 1300 passes).
* **`iter_max`.** n = 10 reaches its plateau (14.79 %) between 1000 and 2000 passes; n = 20 still gains slowly (19.4 % at 2000, 20.7--21.2 % at 5000); n = 50 is at 19.7 % after 1000 passes. Default 2000.
* **Defaults (2000 passes, `Incumbent`, $T_0=0.01$, $\alpha=0.995$):** n = 10: gain 14.63 % (min 13.16, max 14.79), 0.20 s per run; n = 20: 19.58 % (13.32--23.63), 0.25 s; n = 50: 20.75 % (18.72--21.91), 1.4 s per run. (The paper reports about 10 s at 50 customers in Java; no comparison is attempted.)
* **Shake handling in practice.** About one shake draw in three is `NotApplicable` (Drone_Remove without trips, or route shakes after the truck has few customers) and is redrawn; evaluator-infeasible shake results occur only at n = 50 (33.6 per 2000 passes, redrawn); no pass was ever skipped in the pilots (`iterations_skipped` is 0 at the defaults; 50 passes were skipped in the oracle campaign, on tiny instances with `max_shake_attempts = 1`).
* **Reading:** the printed reference is the one that works, and the search is dominated by improving moves; annealing at these temperatures is a small perturbation. A larger $T_0$ or `ShakenState` is selectable but is worse on these pilots.

## 9. Open points and assumptions requiring approval

A30--A38 (section 5, plus A38 `max_local_steps`) were implementation assumptions, approved at the M7 sign-off (section 10); D7 pilot values are defaults, not paper values. Points the user may want to decide:
1. **A30** acceptance reference: printed $f(x)$ (default) vs `ShakenState`; the pilot favours the printed one.
2. **A31/A33** geometric cooling and the pilot defaults (`iter_max = 2000`, $T_0 = 0.01 f(x_{init})$, $\alpha = 0.995$): single instance per size, to be re-calibrated on more datasets in M9 if wanted.
3. **A35** shake redraw policy (`max_shake_attempts = 10`; rejected, never repaired) and the skipped-pass rule.
4. **A37** the improvement threshold stays 0 (strict `<`); the option exists.
5. **A32** N1/N2 placement (after the six, alternating); the restart on improvement repeats them more often in practice.
6. The paper's possible secondary stopping criteria are unknown; `max_no_improve` and `max_seconds` exist, off by default.
7. Not done (M8): config-file wiring, runner, result writer, CLI, the 10-run protocol driver. (Historical.) M7 was signed off on 9 Nov 2026, 14:34 IST; M8 and M9 are complete.

## 10. M7 sign-off and the A38 reporting condition (9 Nov 2026, 14:34 IST)

**Approved:** A30--A37 as implemented. **A38 approved conditionally:** `max_local_steps = 1000` stays a configurable safety guard, but reaching it must be reported (termination reason or diagnostic counter), never a silent truncation of the local search; a focused test and a documented effect were required. M7 is signed off after the following changes (no behaviour of the search changed; only reporting and tests).

**What the guard does.** The guard bounds the number of operator calls *within one pass's* local search (default 1000, `0` = unlimited, configurable in `VnsOptions`). When the next call would exceed the bound, that pass's local search stops immediately; the accepted solution, best-so-far and temperature schedule are kept as they are, the pass counts as an iteration, T cools as usual and the next pass starts with a new shake. The default list has 10 entries; a pass that restarts after each improvement needs more calls than 1000 only if improvements keep occurring or non-improving candidates keep being accepted, which at $T_0\le10^{-2}f(x_{init})$ never happened in any pilot (`truncated 0`); it was found at $T=10^{30}$. Effect: the search of the truncated pass is shorter and weaker, never invalid (every stored solution is evaluator-judged as before).

**How it is reported (new).** A guard trigger ends one pass, not the run, so `VnsResult::stop` stays `IterMax` (or the other stop reason); the trigger is reported by:
* per pass (`VnsTraceRow::end`): `Completed` (list exhausted normally), `Skipped` (no feasible shake, no local search) or `GuardTruncated`; plus `local_steps` (operator calls in that pass);
* totals: `passes_completed`, `iterations_skipped`, `passes_truncated`, with the invariant `iterations == passes_completed + iterations_skipped + passes_truncated`; `local_steps_total`, `max_local_steps_in_pass`; the convenience `guard_triggered()` (= `passes_truncated > 0`); names via `vns_pass_end_name`.

**Tests.** `test_vns` 85 -> 108 assertions: guard fixture with `max_local_steps = 1` (every applied pass reported `GuardTruncated`, exactly one call each, run stop reason unchanged, fewer calls than the unguarded run), normal run (no trigger, every applied pass `Completed`), single-operator list (non-improving call ends normally, an improving one is truncated), unlimited guard (never reports), hot run under a modest guard, skipped passes distinguishable. `test_vns_oracle`: the independent loop now also derives each pass's outcome, `local_steps` and the totals, compared exactly (same 29,447 checks, now stronger). Mutation: V-44 repaired for the new code and six new mutants (V-53..V-58: truncated pass reported Completed, completed passes not counted, skipped pass reported Completed, step total not accumulated, truncated flag never set, busiest-pass steps not tracked) **all killed**; the suite is now 58 mutants, 57 killed, V-10 the equivalent survivor.

**Confirmations.** (1) The default improvement threshold is `min_improvement = 0.0` and the test is strict `fz < ref - delta`, i.e. strict `<` at the default (default pinned by `test_vns`; V-12 (`<=`) and V-13 (threshold ignored) killed). (2) The complete 21-program suite was built and run under ASan + UBSan (`-O1 -g -fsanitize=address,undefined -fno-sanitize-recover=undefined`): 21 of 21 pass, 0 sanitizer reports (`m7_closure_sanitizer_raw.txt`). (3) The result records distinguish normal completion, skipped passes and guard-triggered truncation as described above. (4) Only M7 documentation and validation records were updated.
