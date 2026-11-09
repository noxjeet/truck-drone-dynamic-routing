# Static experiment methodology

Status: **protocol document, written at M1 and brought up to date at the M10 freeze.** Everything marked *implemented* exists and was validated; the
realised protocol and its results are in `validation_vs_paper.md`. Nothing in this file is a result.

## 1. Objective of the experiments

Reproduce, as far as the published information allows, the computational study of Han, Chi & Sun (2026) for the **static** problem, and report
honestly what can and cannot be matched. Success criterion: a faithful, documented, deterministic implementation whose outputs are compared to the
paper's tables with explicit explanations of every difference. It is **not** to hit the paper's numbers.

## 2. Instance families

| Family | Customers | Datasets | Area | Source |
|---|---|---|---|---|
| Small / Medium / Large | 10 / 20 / 50 | 4 each (dataset index 0--3) | $60\times60$ km | [PAPER Sec 6.1] sizes; uniform positions and central depot are [ASSUMPTION] |
| Sensitivity analogues | 8 | several | $20\times20$ km | synthetic stand-ins for the unpublished Dalian instances C1--C4; **never** reported as reproducing them |

*Implemented:* generator with dataset index and base seed (`instance/static_instance.hpp`). Defaults and provenance in `include/config/static_config.hpp`.

## 3. Seeds and reproducibility

* One **base seed** (default `20261109`) plus named streams: Instance, InitialSolution, Vns, Shake (`util/rng.hpp`, math_spec §8). *Implemented.*
* Instance seed $=\operatorname{derive}(b,\text{Instance},\text{dataset})$. *Implemented.*
* Replication seed $=\operatorname{derive}(b,\text{Vns},\ r)$ with $r$ = `first_run_index` + run number (the dataset does not enter the VNS seed; the instance does), shakes likewise with the Shake stream (`vns.hpp`). *Implemented (M7/M8); this replaces the M1 plan $1000\cdot\text{dataset}+\text{run}$.*
* The generator and every transformation (integers, reals, permutations) are defined in the repository, so the same configuration gives the same instance
  and the same search trajectory on any compiler/OS. *Implemented for generation and search; verified byte-identical across re-runs and across Linux/Windows builds (`windows_validation.md`).*
* Floating-point summation of the objective was made order-independent (ordered summation, M1 follow-up P2, `performance_followup_P1_P2.md`).

## 4. Algorithms under test

1. Initial solution (existing, frozen): truck-only TSP, $k$-means with $k=\lceil c/(\lvert U\rvert+2)\rceil$, FLP pool, greedy drone trips.
2. Local search: neighbourhoods N1--N8 in the paper's execution order (N4, N5, N8, N3, N6, N7, with N1/N2 repeated twice). *Implemented (M2--M5).*
3. Shaking: four mechanisms, selected uniformly [ASSUMPTION D6]. *Implemented (M6).*
4. VNS: simulated-annealing acceptance, best-so-far tracked separately, iteration counter incremented correctly (D8). *Implemented (M7).*
   $T_0$, cooling rate, `iter_max`, no-improvement limit are configuration values calibrated in pilot runs; **not tuned to match the paper's numbers** (D7).

## 5. Protocol

* **Per instance:** 10 independent VNS runs [PAPER Sec 6.1]; record each run's final objective and wall time; $z_{best}=\min$, $z_{avg}=\text{mean}$ over the runs.
* **Per configuration:** all 4 datasets of each size; report per-dataset values and the mean over datasets.
* **Gaps** [PAPER (38),(39)]: $\Delta z_{best}=\dfrac{z-z_{best}}{z}\times100\%$ and $\Delta z_{avg}=\dfrac{z-z_{avg}}{z}\times100\%$, where $z$ is the reference solution value
  (CPLEX in the paper; see §7).
* **Statistics:** mean, sample standard deviation ($n-1$), minimum and maximum over replications; paired comparisons use the same instances.
* **Units:** seconds internally; reports show seconds and minutes.
* **Validity gate:** a run is reported only if the final solution is accepted by `evaluate_solution`; the count of infeasible/rejected candidates is reported separately.

## 6. Required content of every comparison table

For each quantity: the **paper's published value**, **our value**, **number of replications**, **parameters used** (including $\delta$, $T_0$, cooling, `iter_max`, seed),
**mean and standard deviation**, **absolute and relative difference**, and a **one-line explanation** classified by fidelity tier:

* *Exactly reproducible* (instance sizes, parameters of Sec 6.1, objective definition, cluster-count formula, 10-run protocol): a mismatch needs investigation.
* *Approximately reproducible* (initial-solution details, neighbourhood candidate sets, TSP substitute): mismatches are expected and explained.
* *Assumption-dependent* (FLP rule, depot, seeds, VNS parameters, shake magnitudes, instance coordinates): mismatches are expected and are **not** evidence of a defect.

No result is described as "reproduced" unless it is within the stated tolerance for its tier; no table entry is ever back-filled.

## 7. The CPLEX dependency

The paper obtains the truck-only tour and the reference values $z$ with CPLEX 12.6.3 (Table 3: 1,400--1,800 s even at 10 customers). No MILP solver is available here.

* **Truck-only TSP** (initial-solution step): `TSPSolverFactory` uses Held-Karp (exact) for up to 21 TSP nodes (depot included), i.e. for 10 and 20 customers, which gives the
  same optimal *length* as CPLEX (the tour may differ only by ties or reversal). For 50 customers it falls back to a labelled heuristic (nearest neighbour + 2-opt/Or-opt).
  The effect is confined to the starting solution; heuristic-vs-exact TSP quantification was *not done* (listed as a limitation).
* **Reference value $z$ in Table 3**: not obtainable. Substitute (implemented in M9, A47): the exact truck-only optimum by dynamic programming for the 10-customer instances, used strictly as an independent reference for the truck-only sub-problem and **not** as a substitute for the MILP optimum (`validation_vs_paper.md` section 6); Table 3's CPLEX columns are not reproduced.

## 8. Out of scope

Dynamic requests, admission regions, event-driven replanning, drone-state recovery, and local-VNS re-optimisation (project on hold). The fixed-launch comparison of Sec 6.3.4 (TSP-D, TSP-MD)
requires a separate fixed-launch mode and is a stretch goal pending approval.

## 9. Realised protocol (M9)

12 instances per size (4 datasets x 3 independent seed sets), 10 runs each, pilot VNS defaults, 95 % intervals over generated instances (a seed set changes both the instances and the search streams); minutes are a provisional comparison unit because the paper does not confirm its unit. Configurations `config/m9/*.cfg`, driver `scripts/m9_run_all.sh`, analysis `scripts/m9_analyse.py`; details and results in `validation_vs_paper.md`. The sections above that mention the 4-dataset layout describe the paper's design; our protocol triples it.
