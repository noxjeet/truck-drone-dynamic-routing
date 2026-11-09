# Dynamic extension — formal mathematical specification (DM1)

**Review status.** DM1 was **accepted by the user on 9 Nov 2026 (20:40 IST)** after review of the completion report, subject to the closeout in this revision (documentation only). Acceptance covers the *evidence as labelled below*: derived algebra [ALG], hand arguments [HAND], and evaluator observations [OBS] keep their separate labels; acceptance does **not** promote any [HAND]+[OBS] statement to proven, and the proposed defaults in §9 remain **proposed** (not approved) unless the user says otherwise. The open items are in §11.

Branch `dynamic/dm0-design` (from tag `static-baseline-m10`, commit `a0112ce`). Companion probe: `bench/dm1_probes.cpp` (read-only; not in the default build or CTest). Derived from `claude/dm0-design-specification.md` v3.1. **DM1 is mathematical validation, not dynamic implementation.** Nothing here changes the evaluator, static operators, defaults, tests or result schema.

## 0. Status labels (every statement carries exactly one)

| Label | Meaning |
|---|---|
| **[ALG]** | proven algebra, valid for the stated model with no reference to the evaluator |
| **[HAND]** | hand-derived argument about the evaluator's recurrence; not mechanically proven |
| **[OBS]** | evaluator observation on generated cases (counts below). **An observation is never a guarantee**; it can only refute |
| **[UNV]** | unverified claim or design choice |
| **[STATIC]** | rule of the frozen static model/evaluator, used as given |

A statement shown **[HAND]+[OBS]** means the hand argument exists and the probe found no counterexample in the stated sample; it is **not** promoted to proven.

## 1. Model facts used (from `include/timing/timing_engine.hpp`, read only) [STATIC]

Objective $F=\sum_c \mathrm{ETA}_c$ (service-arrival times). Truck arrival $t_p$ and departure $l_p$ at route position $p$ come from a forward recurrence. Launch $\hat l_i=t_i+S_L$; truck leaves a node with $n_i$ launches at $t_i+n_iS_L$ (+ service). Node completion at a recovery node is $l_p=\max\bigl(t_p+Se_1+r_pS_R,\ \max_u(a_u+S_R)\bigr)$ with $a_u$ the drone's arrival ("Option B"). Endurance (36) uses the **truck** arrival $t_k$: $(t_k-\hat l_j)+(\hat l_j-\hat l_i)(1+\theta)\le e$; (37) $=(\hat l_j-\hat l_i)(1+\theta)+d_{jk}/v_D\le e$. Launch FLPs must lie on the arc they belong to ($p9$, rules (13)–(15)); rule (28) and $|U|$ limits apply. Infeasible plans are still timed best-effort, so `objective_value` is meaningful only when `feasible`. The evaluator's `SynchronizationRecord::wait_time` is $|t_k-a_u|$, an **arrival-to-arrival** gap (timing_engine.hpp:689), **not** the absorbed completion-time delay $w$ used in §4.4 (see CE-4).

## 2. Definitions

**Skeleton.** Route nodes that are depot or customer; FLPs are not skeleton nodes. A *gap* $g=(a,b)$ is a pair of consecutive skeleton nodes, at route positions $P(a)<P(b)$; FLPs of arc $(a,b)$ sit between them.

**Clean cut at skeleton position $B$.** A trip $(i,j,k)$ *spans* $B$ iff $P(i)\le B<P(k)$. The cut is **clean** iff no trip spans it. [ALG as a definition.] Rationale: not timing, but *feasibility of airborne trips*: P1 holds with spanning trips; the clean cut is chosen because a tail edit changes $t_k$ and hence (36) of a spanning trip (CE-1). [HAND]+[OBS] (CE-1).

**Straddled gap.** $g=(a,b)$ is straddled iff some trip has $P(\text{launch})<P(b)\le P(\text{recovery})$. This includes a trip launched at an FLP of arc $(a,b)$ (then $P(\text{launch})\in(P(a),P(b))$ and $P(\text{recovery})\ge P(b)$). [ALG as a definition.]

**Tail set $\mathcal T(g)$.** Truck customers at positions $\ge P(b)$ **plus** drone customers of trips launched at positions $\ge P(b)$. It excludes customers of trips launched before $P(b)$ (their timing is fixed by prefix or governed by the straddle case) and customers of trips recovered at or before $a$. [ALG as a definition; its adequacy is T4/T14.]

**Insertion detour.** For a truck insertion of $r$ into $(a,b)$: $\tau_{ar}=\|a-r\|/v_T$, $\delta_g(r)=\tau_{ar}+Se_1(r)+\tau_{rb}-\tau_{ab}$ (Euclidean, no waiting). [ALG]

**Φ (promise predicate).** $\Phi(S^\ast;r)=$ evaluator-feasible $\wedge$ commitments (I2) $\wedge\ \mathrm{ETA}_r\le\tau_r+W\ \wedge\ \forall i$ pending: $\mathrm{ETA}'_i\le P_i+E_{c(i)}$ (spec §7.3). Slack of $i$ is $D_i-\mathrm{ETA}_i$ with $D_i=P_i+E_{c(i)}$. [UNV: a design definition; evaluated only through the evaluator].

**No separate symbol Ψ (notation resolved).** Spec v3.1 has no Ψ, and none is needed: the slack predicate is already expressed by the existing ellipse notation $E_g(\cdot)$ of §6.2 together with $\hat\delta_{\max}(g)$. For a gap $g$ and request $x$:
$$x\in E_g\bigl(\hat\delta_{\max}(g)\bigr)\iff \delta_g(x)\le\hat\delta_{\max}(g),\qquad \hat\delta_{\max}(g)=\begin{cases}\min_{c\in\mathcal T(g)}\mathrm{slack}_c &\text{if $g$ is not straddled}\\ +\infty &\text{if $g$ is straddled,}\end{cases}$$
with $\min\emptyset=+\infty$ and the comparison inclusive within $10^{-9}$ relative. The equivalence is P5 [ALG]; the use as a filter rests on P4 ([HAND]+[OBS]). Wherever the task text says "Ψ", read "membership in $E_g(\hat\delta_{\max}(g))$". (The DM1 first draft had introduced Ψ as a shorthand; it is withdrawn so that no new symbol enters the specification.)

**Move class $\mathcal M_{\rm ins}$ and G2′.** $\mathcal M_{\rm ins}$: (a) insert $x$ as a truck customer in one gap with N4-style re-anchoring of affected trips, or (b) make $x$ the customer of one new drone trip $(i,x,k)$, $i$ an FLP on an editable arc, $k$ an editable skeleton node. G2′ $=\{x:\exists\,m\in\mathcal M_{\rm ins}$ giving an evaluator-feasible, Φ-satisfying plan$\}$ on the same plan $S_{\rm inst}(\tau_r^-)$ on which Σ is built. [UNV: a definition used as ground truth; no DM1 probe enumerates re-anchoring, see §6 limitations]

**G3 (hybrid filter).** $G3=\bigcup_g E_g(\hat\delta_{\max}(g))\ \cup\ \mathrm{Tube}_{\rm line}$ with **no finite cap**. Claim to be tested at DM5/T14: $G2'\subseteq G3$ (no false exclusion relative to $\mathcal M_{\rm ins}$). **Not claimed** conservative relative to re-optimisation (CE-3). G3c (finite-radius cap) and G1 are demand restrictions, not filters. Status of $G2'\subseteq G3$: **[HAND]**; the truck-insertion part is probed in T3/T4 below ([OBS], 0 false exclusions); move (b) and re-anchoring are **not** probed in DM1 [UNV].

**Reach bounds.** For a drone customer $j$ with launch FLP $i$ and recovery node $k$, (37) with $\hat l_j-\hat l_i=d_{ij}/v_D+Se_2$ gives $(1+\theta)(d_{ij}/v_D+Se_2)+d_{jk}/v_D\le e$. All three quantities below follow from this one inequality [ALG]:

| Quantity | Formula | Paper params (P) | Regime F ($e=3600$) |
|---|---|---|---|
| tight necessary radius $r_D^\ast$ ($d_{ij}=d_{jk}=\rho$) | $v_D[e-(1+\theta)Se_2]/(2+\theta)$ | 6960 m | 14 160 m |
| one-way bound, $d_{jk}\to0$ (probe: $d_{jk}=1$ m) | $v_D[e-(1+\theta)Se_2-d_{jk}/v_D]/(1+\theta)$ | 10 439.5 m ("10.44 km") | 21 239.5 m |
| return-leg-only bound, $d_{ij}\to0$ (probe: $d_{ij}=1$ m) | $v_D[e-(1+\theta)(Se_2+d_{ij}/v_D)]$ | 20 878 m ("20.878 km"; total path 20 879 m) | 42 478 m |
| flight-path bound ignoring $\theta$ and service | $v_De$ | 21.6 km (M9's "≈20 km"; not a radius) | 43.2 km |

$r_D^\ast$ is a **necessary** radius for $\rho=\mathrm{dist}(x,\text{hull of editable skeleton nodes})$ because both flight legs end on the route; the 10.44 and 20.878 km values are loose extremes and must not be used as radii. Code must compute these from the parameters (the probe does; nothing is hard-coded). Sufficiency is false (endurance also depends on timing, rule (28), $|U|$, FLP availability).

## 3. Statements P1–P6

### P1 — prefix invariance
**Statement.** Let two plans agree on instance data (H1), on route$[0..B]$ including the start depot (H2), and on the multiset of trips launched at positions $\le B$ with their drone ids (H3), with $B$ a skeleton position (H4). Then $t_p,l_p$ for $p\le B$, and the ETAs of customers served at positions $\le B$ or by trips launched at $\le B$, are identical (bitwise for the same evaluator build).
**Limitations.** Timing only: the feasibility of a trip launched $\le B$ and recovered $>B$ **can** change (CE-1). It fails if anything at positions $\le B$ changes.
**Status:** [HAND] (forward causal recurrence) + [OBS] T1.

### P2 — tail shift
**Statement.** Let region $[a,b]$ be trip-closed, tail route and tail trips unchanged, $b$ a skeleton node, and **no trip spans $b$**. Then every tail time shifts by the departure shift $\Delta$ at $b$ (to rounding), tail (36)/(37) values are unchanged, and
$F'=F_{\rm prefix}+\sum_{(a,b]}\mathrm{ETA}'+m_{\rm tail}\Delta+\sum_{\rm tail}\mathrm{ETA}$.
**Limitations.** Fails with a trip spanning $b$ (CE-1) and when the tail is also modified (negative control). Equality is not bitwise. $\Delta$ is the **departure** shift at $b$, not $b$'s ETA shift. The identity is vacuous when $b$ is the end depot.
**Status:** [HAND] + [OBS] T2.

### P3 — truck-only insertion cost
**Statement.** Inserting a truck customer $r$ into non-straddled gap $g$ gives $F(S\oplus_g r)-F(S)=\mathrm{ETA}_r+|\mathcal T(g)|\,\delta_g$ with $\mathrm{ETA}_r=\delta_a+\tau_{ar}$ ($\delta_a$ the departure at $a$). With drone recoveries only in the prefix and drone launches only in the tail this is an equality.
**Limitations.** If a trip straddles the gap and no launch FLP lies on the split arc, the evaluator-observed shift is $\le\delta_g$ in every feasible case probed, but this inequality is **only observed**, not proven. If a launch FLP lies on the split arc the plan is infeasible with the stale FLP (CE-2), so no cost statement applies. Feasibility of spanning trips is not covered (CE-1).
**Status:** equality [HAND]+[OBS] T3; straddled inequality [OBS] only; FLP-on-arc infeasibility [STATIC rule]+[OBS].

### P4 — slack test
**Statement.** If $g$ is non-straddled, then for per-customer bounds $D_i$: all existing customers keep their bounds after inserting $r$ iff $\delta_g\le\min_{\mathcal T(g)}\mathrm{slack}$ (necessary and sufficient for the bound check, since all of $\mathcal T(g)$ shift by exactly $\delta_g$ and nothing else changes; feasibility is separate and holds in the probe).
If $g$ is straddled, the bound is **only sufficient when the min is over the straddling trips' customers too, and the plain $\min_{i\ge b}$ is falsely exclusive** (CE-4): this is why $\hat\delta_{\max}=+\infty$ there (decision U8).
**Limitations.** Slack min over a set that omits tail drone customers is not safe (observed false acceptance); a set that includes customers of trips recovered before $a$ is over-conservative (observed false exclusion).
**Status:** non-straddled iff [HAND]+[OBS] T4a; straddled falsifications [OBS] T4b and CE-4 (the CE-4 *numbers* are now [OBS]).

### P5 — detour ellipse
**Statement.** $x$ is insertable into arc $(a,b)$ with detour time $\le\delta_{\max}$ iff $\|a-x\|+\|x-b\|\le\|a-b\|+v_T(\delta_{\max}-Se_1)$, when $\delta_{\max}\ge Se_1$ (else empty).
**Limitations.** Algebra only; says nothing about feasibility or FLPs.
**Status:** [ALG]; the evaluator-agreement check is [OBS] T5.

### P6 — drone reach
**Statement.** Any evaluator-feasible drone trip serving $j$ has $\mathrm{dist}(j,\text{hull of skeleton nodes})\le r_D^\ast$ (table above).
**Limitations.** Necessary only. The 10.44/20.878 km values are not radii.
**Status:** [ALG] for the bound; boundary behaviour [OBS] T6.

## 4. Counterexamples (deterministic, all reproduced with the unchanged evaluator)

All coordinates in metres; regime P; $Se_1=Se_2=30$ s, $\theta=1$.

* **CE-1 [OBS].** Depot $(-3000,0)$, $f=(0,0)$ on arc (depot,$c_1$), $c_1=(6000,0)$, $c_2=(12000,0)$, $X=(6000,2000)$; trip $(f,X,c_2)$. Base feasible: (36)=1387.05 s, (37)=1641.14 s, $F=2537.05$. Insert $r=(9000,9000)$ between $c_1,c_2$: $t_{c_2}$ 1090.00→1984.91, drone ETA 787.05 unchanged, (36)=2281.96>1800 ⇒ infeasible. Prefix arrays $[0..c_1]$ bit-identical.
* **CE-2 [OBS].** Depot $(0,0)$, $f=(6000,0)$, $A=(12000,0)$, $X=(6000,3000)$, trip $(f,X,A)$, $F=1570.00$. Insert $r=(6000,-20000)$: 20 FLPs generated ($d=2000$ m) on arcs (depot,$r$),($r$,$A$): 0 evaluator-feasible; the stale-FLP plan is infeasible.
* **CE-3 [OBS, grid-limited].** Square depot $(0,0)$, $(20000,0)$, $(20000,20000)$, $(0,20000)$; request $X=(10000,10000)$. On the current route, 0 of 1791 (FLP on a 199-point-per-arc grid, later skeleton recovery node) combinations is evaluator-feasible; polyline distance to $X$ is 10 000 m and nearest node 14 142 m, so a tube filter on the current polyline excludes $X$. Re-sequencing to depot–$(20000,20000)$–$(20000,0)$–$(0,20000)$–depot with an FLP at the centre of the diagonal arc: feasible, (37)=1238.5, (36)=972.8. The 0/1791 is a finite grid, not a proof of impossibility.
* **CE-4 [OBS]; hand construction confirmed, one quantity re-defined.** CE-1 layout plus $c_3=(12000,-4000)$ after $c_2$: route depot–$f$–$c_1$–$c_2$–$c_3$–depot, trip $(f,X,c_2)$ (gap $(c_1,c_2)$ straddled); insert $r=(9000,4758)$.
  | Quantity | Hand-derived | Evaluator |
  |---|---|---|
  | $\delta_g$ | ≈380.0 s | 379.976 s |
  | $t_{c_2}$ | 1090.0→1470.0 | 1090.000→1469.976 |
  | drone arrival at $c_2$ | 1344.1 | 1344.093 (unchanged by the insertion) |
  | absorbed waiting $w$ = drone-side minus truck-side completion at $c_2$ (before insertion) | ≈224.1 (≥184 stated) | **224.093** |
  | departure shift at $c_2$ / ETA shift of $c_3$ | ≤196 stated | **155.883** (= $\delta_g-w$) |
  | (36) after | 1767.05 | 1767.022 ≤ 1800; (37) 1641.139 unchanged; trip feasible |
  | evaluator `wait_time` at $c_2$ | not defined in hand text | 254.093 s before ($=a_u-t_k$), 125.883 s after ($=t_k-a_u$, drone waits) |

  **Definition note (wait_time).** In the evaluator, `wait_time` $=|t_k-a_u|$: before the insertion $a_u-t_k=1344.093-1090.000=254.093$ s (truck waited for the drone), after it $t_k-a_u=1469.976-1344.093=125.883$ s (drone waited for the truck). The quantity that bounds how much of $\delta_g$ the straddling trip absorbs is the **completion-side** delay $w=(a_u+S_R)-(t_k+Se_1+r_kS_R)=224.093$ s (with $r_k=1$ recovery), i.e. $w=\text{wait\_time}_{\rm before}-Se_1$. Always compute $w$ from the evaluator's completion rule, never read it off `wait_time`.

**Discrepancy explained:** the evaluator's `wait_time` is arrival-to-arrival (254.093 before the insertion), not the absorbed delay; $w=254.093-Se_1=224.093$ s. The hand claim is confirmed (tolerance 0.1 s on $\delta_g$, 0.2 s on (36)). **Negative control:** with PM2 slacks $\mathrm{slack}_{c_3}=250$ s, $\mathrm{slack}_{c_2}=\infty$, the naive bound $\min_{i\ge b}\mathrm{slack}_i=250<\delta_g$ rejects $r$ (ellipse sum 11 249.6 m > limit 9300.0 m), although the insertion is evaluator-feasible and the shift at $c_3$ is $155.9\le250$: false exclusion, as predicted. Test power confirmed. (The slack values are synthetic, chosen for this test.)

## 5. Probe results (build: g++ -std=c++20 -O2; run `dm1_probes 5`; RNG seed 20261109; 15 s on the VM)

Random plans: 6–12 customers in a 20 km square, 0–3 trips, regimes P and F alternating; every generated plan was evaluator-feasible (no skips in T1). Tolerances: timing identity $10^{-9}$ relative; objective identity $10^{-6}$ s absolute; F-identity $10^{-6}$ s.

| Test | Result |
|---|---|
| **T1** prefix invariance | 20 000 plans. Tail edit, clean cut 18 664: prefix arrays **bit-identical 18 664**; spanning cut 1 336: **bit-identical 1 336**; prefix-ETA mismatches 0. (S′ evaluator-infeasible after the edit: 2355 clean, 317 spanning; timing still identical.) **Negative control** (edit before the cut): arrays differ in 20 000/20 000. |
| **T2** tail shift | 48 961 identity cases, identity held 48 961 (trip in region 7 665; trips in tail 11 742; infeasible S′ not counted 4 581; 15 046 have the end depot as $b$, where the identity is vacuous); max deviation 4.1e-12 s; (36)/(37) tail mismatches 0; F-decomposition violations 0. **Negative controls:** edit with a trip spanning $b$: broke in 6 351/6 351; second tail edit: uniform shift broken in 18 963/20 000 (1 037 unbroken; **not diagnosed** — a coincident zero shift or absorption is plausible but unverified). |
| **T3** truck-only insertion | non-straddled gaps: 159 085 feasible cases (107 141 in plans with trips), max $|\Delta F-\text{pred}|=5.1\times10^{-11}$, violations 0, evaluator-infeasible 0. Straddled, no launch FLP on the gap: 6 253 feasible (inequality $\Delta F\le$ pred violated 0), 5 127 infeasible. Launch FLP on the gap: 14 777 cases, **all** evaluator-infeasible (CE-2 behaviour; stale FLP). |
| **T4a** slack test, non-straddled | pass&feasible 31 566; pass&infeasible 0; fail&feasible 0; fail&infeasible 127 519 ⇒ agrees with the evaluator+bounds in all 159 085 cases. $\hat\delta_{\max}$ with $+\infty$ on straddled gaps: false exclusions 0/34 223 truth-feasible insertions. |
| **T4b** controls (straddled gaps; informative, not pass criteria) | min over $\mathcal T(g)$ without the $+\infty$ relaxation: false exclusion (CE-4 type) 264, false acceptance (CE-1 type) 5 271. Naive set including straddling-trip customers: false exclusions 1 286, false acceptances 2 402. $\mathcal T$ including customers of trips recovered $\le a$: false exclusions 8 715. Truck-only set (omits tail drone customers): false acceptances 25 125. Each wrong definition produces failures, so the test has power. Slack bounds are random synthetic values (10 % zeros), not PM2 values. |
| **T4c** CE-1/2/3/4 | all four deterministic cases as in §4: PASS. |
| **T5** ellipse vs detour | 5 000 000 points (2 525 855 inside); 2 434 inside the $10^{-9}$-relative boundary band; disagreements outside the band 0, inside the band 0. |
| **T6** reach | boundary: R1 bisected 10 439.500 vs formula 10 439.500; R2 symmetric 6960.000 (feasible at $r^\ast-10^{-3}$, infeasible at $r^\ast+10^{-3}$); R3 $D=20879.000$ vs 20879.000 (return leg 20 878). Regime F: 21 239.500, 14 160.000 (±$10^{-3}$ m behaviour as above), 42 479.000 (return leg 42 478). Fuzz: 1 500 000 random trips and parameter sets (regimes P, F and random $e,\theta,Se_2,v_D$), evaluator-feasible 769 616, violations of $\rho_{\rm hull}\le r_D^\ast$ 0, geometric-lemma failures 0, max $\rho/r^\ast$ among feasible 0.991. |

**Probe bugs found and fixed during DM1 (probe code, not model):** depot recovery was located at the *first* depot index instead of the end depot (made 48 spurious T3 failures and 157 spurious T2 F-identity failures); one negative control always chose the end gap; a 1 m label slip in the 20.878 km line. None involved an evaluator change.

## 6. Limitations of the DM1 evidence [UNV where stated]

* All probes are random fuzz on 6–12 customers, ≤3 trips, a 20 km square, $|U|=2$: **observations, not proofs**.
* G2′ ⊆ G3 was checked only for truck insertions (move (a) without re-anchoring); move (b) and N4-style re-anchoring are not probed in DM1 [UNV].
* CE-3's "impossible on the current route" is a grid result.
* PM2 slack values in T4 are synthetic; Φ's response-window condition is not exercised.
* One-depot, Euclidean, fixed-speed model only; no per-drone constraints beyond $|U|$ in these random plans beyond what the evaluator enforces.

## 7. Regime F (frozen as a sensitivity regime) [UNV: decision U5, proposed default]

Regime F = regime P with endurance doubled **only**: $e=3600$ s (P: 1800); $v_T=15$ m/s, $v_D=12$ m/s, $S_L=60$ s, $S_R=40$ s, $Se_1=Se_2=30$ s, $\theta=1$, $|U|=2$, all unchanged. Reach from the formula: $r_D^\ast=12\,(3600-2\cdot30)/3=14\,160$ m (P: 6960 m). Labelled drone-favourable and sensitivity-only; the static defaults are not changed.

## 8. N4 re-anchoring caps/limit counters (read from source, not documentation)

| Name | Location | Meaning for the certificate sweep |
|---|---|---|
| `N4Options::max_reanchor_variants` | `include/search/neighbourhood_n4.hpp:79` (default 100; 0 = unlimited); applied at :298–301, loop break at :342 | the re-anchoring budget; U7 permits a larger value in the sweep |
| `CandidateRecord::cap_reached` | `include/search/move_types.hpp:147` | true for a candidate whose variant enumeration hit the cap |
| `CandidateRecord::combinations_total` | `move_types.hpp:146` | total re-anchoring combinations for the candidate |
| `CandidateRecord::variants_generated` | `move_types.hpp:144` | variants actually generated (< `combinations_total` iff capped) |
| `OperatorResult::cap_reached_candidates` | `move_types.hpp:198` | operator-level count of capped candidates (**any value > 0 ⇒ `budget-stop`**) |
| `OperatorResult::variants_generated` | `move_types.hpp:197` | operator total |
| `adjacency_window` | `move_types.hpp:43` (default 3) | a neighbourhood window, **not** a cap counter, but limits the sweep; must be recorded and, for certification, set to full or counted as a limitation |
| `N8Options::insertion_window` | `include/search/neighbourhood_n8.hpp:43` | same remark for N8 |
| `duplicates_skipped` | `move_types.hpp:190` | dedupe counter only; not a cap |

**Rule (unchanged, D7/U7):** during the certificate sweep, if `CandidateRecord::cap_reached` is true for any candidate, equivalently `OperatorResult::cap_reached_candidates > 0` for any operator call, the outcome is **`budget-stop`**; a larger `max_reanchor_variants` (U7, proposed) only makes `certified` reachable, it never relaxes the rule. A `variants_generated < combinations_total` candidate with `cap_reached == false` must be treated as a counter inconsistency and also downgrades to `budget-stop` (DM9 test).

## 9. Decision log (all **proposed defaults, marked for confirmation**; none user-confirmed unless stated)

| Id | Proposed default | Status |
|---|---|---|
| U1 | four-step Φ chain: cheapest evaluator-feasible insertion → optimise → cheapest Φ-feasible insertion → else `rejected-promise` | proposed, confirm |
| U2 | report **G0** (reference) and **G3** in every comparison; neither is "the" result | proposed, confirm |
| U3 | A1/A2 policies deferred | proposed, confirm |
| U4 | **deferred until DM8** | open |
| U5 | regime F doubles endurance only (§7) | proposed, confirm |
| U6 | scale and corridor always from the static-VNS reference base plan | proposed, confirm |
| U7 | larger N4 `max_reanchor_variants` allowed in the certificate sweep; any `cap_reached` (or `cap_reached_candidates > 0`) ⇒ `budget-stop` | proposed, confirm |
| U8 | keep $\hat\delta_{\max}=+\infty$ on straddled gaps; sharper bound deferred. DM1 evidence supports the need for the relaxation (T4b, CE-4) but proves nothing about any sharper bound | proposed, confirm |
| Q2, Q6, Q10, Q11, Q12, E7, Q-F | adopted as recommended defaults (Q2 objective $F$ with separate admission; Q6 deterministic pass budget tied to $\omega$; Q10 separate `tdr_dynamic`; Q11 demand defaults; Q12 additive seed streams; E7 re-implement N4 re-anchoring in the dynamic module; Q-F $n_{ab}\le64$ validator) | proposed defaults, **not independently user-confirmed** |

## 10. DM1 exit status (accepted)

Required probes T1–T6 and CE-1..CE-4 exist and pass on the stated samples; no mathematical statement of P1–P6 was falsified. Corrections: CE-4's "wait" is clarified (arrival-based `wait_time` vs absorbed delay $w$); the hand numbers are otherwise confirmed. Status of evidence by label: P5, P6 bounds and all definitions are [ALG]; P1–P4 are [HAND]+[OBS]; CE-1..CE-4, T1–T6 results are [OBS]; G2′ ⊆ G3 is [HAND] with a partial [OBS] check.

## 11. Open items carried forward (explicitly retained after acceptance)

| # | Item | Why it stays open | Planned closure |
|---|---|---|---|
| O1 | **1 037 of 20 000 T2 negative controls (second tail edit) kept the uniform shift** | undiagnosed; absorption by synchronization waiting or a coincident zero shift is *plausible, not verified* | classify the 1 037 cases (wait absorption vs zero-delta) in a probe revision during DM3; until then T2's negative control is a ≥ 90 % criterion, not a clean 100 % |
| O2 | **G2′ ⊆ G3 covers truck insertions only** (move (a) without re-anchoring); move (b) (new drone trip) and N4-style re-anchoring are not probed | the G3 conservativeness claim for $\mathcal M_{\rm ins}$ is therefore only partly observed | DM5 / T14 (G2′ ground truth) |
| O3 | **CE-3 "impossible on the current route" is a grid result** (0 of 1 791 combinations on a 199-point-per-arc grid) | a finite grid is not a proof of infeasibility | stays [OBS]; do not cite as a proof |
| O4 | slack bounds in T4 were synthetic; Φ's response-window term is not exercised | PM2/PM1 real values arrive with DM5 | T14 at DM5 |
| O5 | all decisions in §9 are proposed defaults | none user-approved except as noted | confirm in DM0 §15 review |

