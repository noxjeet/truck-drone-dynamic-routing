# Mathematical specification of the static model (code-independent)

Status: living document, synchronised with the implementation at milestone **M1** (9 Nov 2026).
Each statement is tagged **[PAPER]** (stated by Han, Chi & Sun 2026), **[INTERPRETATION]**
(our reading of an ambiguous or defective printed formula) or **[ASSUMPTION]** (the paper is silent).
Equation numbers refer to the paper. Nothing in this file depends on the C++ code; the code is checked
against this file, not the reverse. Sections for the neighbourhoods, shakes and VNS are appended by the
milestone that implements them (M2--M7).

## 1. Sets, parameters and units

All quantities are in SI units: metres, seconds, metres per second. Reports additionally show minutes (decision D10).

| Symbol | Meaning | Baseline value |
|---|---|---|
| $C=\{1,\dots,c\}$ | customers | $c\in\{10,20,50\}$ [PAPER Sec 6.1] |
| $0$ | the depot (start and end of the truck path share this identity) | centre of the square [ASSUMPTION D9] |
| $L=\bigcup_{(a,b)}L_{ab}$ | flexible launch points (FLPs); $L_{ab}$ lies on the directed arc $(a,b)$ | pool of §3 |
| $U$, $\lvert U\rvert$ | drone fleet | $2$ [PAPER Sec 6.1] |
| $v_T,\ v_D$ | truck / drone speed | $15,\ 12\ \mathrm{m/s}$ [PAPER Sec 6.1] |
| $e$ | drone endurance | $1800\ \mathrm{s}$ [PAPER Sec 6.1] |
| $S_L,\ S_R$ | launch / recovery handling time | $60,\ 40\ \mathrm{s}$ [PAPER Sec 6.1] |
| $s^T_j,\ s^D_j$ | service time by truck / by drone | $30\ \mathrm{s}$ each [PAPER Sec 6.1] |
| $\theta_j$ | load coefficient of customer $j$ | $1$ [PAPER Sec 6.1] |
| $W$ | side of the square | $60\,000\ \mathrm{m}$ [PAPER Sec 6.1] |

## 2. Instance model

Let $p_a\in[0,W]^2$ be the position of node $a$. Distances are Euclidean, $d_{ab}=\lVert p_a-p_b\rVert_2$ [PAPER Sec 2.1],
and travel times are distance over a constant speed [PAPER Assumption 2]:
$$\tau^T_{ab}=\frac{d_{ab}}{v_T},\qquad \tau^D_{ab}=\frac{d_{ab}}{v_D}.$$

**Customer positions** [ASSUMPTION]. The paper states only that customers are "evenly distributed" in the square and publishes no
coordinates. We take
$$p_j\ \overset{\text{i.i.d.}}{\sim}\ \mathcal U\big([0,W]^2\big),\quad j\in C,\qquad p_0=\big(\tfrac W2,\tfrac W2\big).$$
Exact numerical reproduction of the paper's instances is therefore impossible by construction.

## 3. The flexible-launch-point pool

For every ordered pair $(a,b)$ of distinct nodes of $\{0\}\cup C$, with unit division length $\delta>0$:
$$n_{ab}=\Big\lfloor \frac{d_{ab}}{\delta}\Big\rfloor \quad\text{[PAPER eq. (41), structure]},\qquad
q_{ab,m}=p_a+\frac{m}{n_{ab}+1}\,(p_b-p_a),\ \ m=1,\dots,n_{ab}\quad\text{[ASSUMPTION: equal-ratio placement]}.$$
$L_{ab}=\{q_{ab,1},\dots,q_{ab,n_{ab}}\}$; arcs $(a,b)$ and $(b,a)$ carry different, geometrically coincident, FLP sets.
The value of $\delta$ for the main experiments is not stated; we use $\delta=2000\,\mathrm{m}$, the spacing of the paper's Sec 6.3
sensitivity analyses [ASSUMPTION]. The pool is generated once ("eager") and never regenerated. The size of the pool is
$\lvert L\rvert=\sum_{a\neq b}n_{ab}$; for $c=50$ and $\delta=2000$ it is $\approx 3.9\times10^4$, about $11\times$ the paper's Table 4
count (3493), whose generating rule is not recoverable.

## 4. Solutions

A solution is a pair $(\pi,\mathcal S)$:

* a **truck route** $\pi=(\pi_0,\dots,\pi_N)$ with $\pi_0=\pi_N=0$, no other occurrence of the depot, each customer at most once,
  each FLP at most once [PAPER (2),(3),(8),(9),(10)];
* a set of **drone trips** $s=(i_s,j_s,k_s,u_s)$: launch FLP $i_s\in L$, served customer $j_s\in C$, recovery node
  $k_s\in C\cup\{0\}$, physical drone $u_s\in U$, with $i_s,j_s,k_s$ pairwise distinct [PAPER Sec 2.2].

Let $\operatorname{pos}(x)$ denote a node's position in $\pi$. A trip is *placed* iff $i_s,k_s\in\pi$ and $\operatorname{pos}(i_s)<\operatorname{pos}(k_s)$.

**Exactly-once service** [PAPER (1),(10)]. Every $j\in C$ either appears in $\pi$ and is the served customer of no trip, or
is not in $\pi$ and is the served customer of exactly one trip.

**Launch only from FLPs** [PAPER Sec 2.2 / decision I-7]: $i_s\in L$ for every trip.

**FLP arc rule** [PAPER p9, (13)--(15), text meaning]. Let $\beta^-(p)$ and $\beta^+(p)$ be the nearest *non-FLP* nodes of $\pi$ before and
after position $p$. An FLP $f\in L_{ab}$ at position $p$ is admissible iff $(\beta^-(p),\beta^+(p))=(a,b)$. Consequently, removing a node
from, or inserting a node into, the gap between two consecutive non-FLP nodes makes **every** FLP in that gap inadmissible.

**Visited FLPs must launch** [PAPER (6)]: every FLP in $\pi$ is the launch node of at least one placed trip.

## 5. Timing

All times are computed in a single forward pass; the truck leaves the depot at $0$ [PAPER p15]. Write $t_p,\ l_p$ for the truck's
arrival and departure at position $p$.
$$t_0=0,\qquad t_p=l_{p-1}+\tau^T_{\pi_{p-1}\pi_p}\quad(p\ge1)\qquad\text{[PAPER (16), read with } l_i\text{]}.$$

**Launch at an FLP** at position $p$ with $n_p$ placed trips launching there [PAPER (18),(21)]:
$$\hat l_s=t_p+S_L\ \ (\text{every trip } s \text{ launching at } p),\qquad l_p=t_p+n_pS_L.$$
Drones launched together all depart at $t_p+S_L$ (no staggering) [ASSUMPTION: earliest admissible value].

**Trip times** [PAPER (17),(22)]:
$$\hat t_{j_s}=\hat l_s+\tau^D_{i_sj_s},\qquad \hat l_{j_s}=\hat t_{j_s}+s^D_{j_s},\qquad a_s=\hat l_{j_s}+\tau^D_{j_sk_s}\ \ \text{(drone arrival at } k_s).$$

**Departure from a customer or depot** $q$ at position $p$, with $R_q$ the set of placed trips recovered at $q$ and $r_q=\lvert R_q\rvert$
[INTERPRETATION, decision D1 "Option B", 8 Nov 2026; the paper prints (18)'s $S_R$ term over the routes *launched* at the node, read here as an
index slip for the routes *recovered* at it]:
$$l_p=\max\Big(\,t_p+s^T_q+r_qS_R,\ \ \max_{s\in R_q}\,(a_s+S_R)\Big),\qquad s^T_0=0.$$
With $R_q=\emptyset$ this is $l_p=t_p+s^T_q$.

**Endurance** [PAPER (36),(37)], with $\hat\ell_s=\hat l_{j_s}-\hat l_s$ the loaded segment (outbound flight plus drone service):
$$E^{(36)}_s=(t_{k_s}-\hat l_{j_s})+\hat\ell_s\,(1+\theta_{j_s})\le e,\qquad
E^{(37)}_s=\hat\ell_s\,(1+\theta_{j_s})+\tau^D_{j_sk_s}\le e.$$
Neither $S_L$ nor $S_R$ enters. Since $E^{(36)}_s-E^{(37)}_s=t_{k_s}-a_s$, we have the identity
$$\max\big(E^{(36)}_s,E^{(37)}_s\big)=E^{(37)}_s+\max\big(0,\ t_{k_s}-a_s\big),$$
i.e. the truck's lateness at the rendezvous is charged one-for-one against the endurance budget.

**Drone identity** [PAPER (28), route-position form; decision I-4]. For each $u\in U$, order its placed trips by launch position
$s^{(1)},s^{(2)},\dots$; then $\operatorname{pos}(i_{s^{(m+1)}})>\operatorname{pos}(k_{s^{(m)}})$. The number of placed trips whose
closed interval $[\operatorname{pos}(i_s),\operatorname{pos}(k_s)]$ contains any given position never exceeds $\lvert U\rvert$.

## 6. Objective

$$z(\pi,\mathcal S)=\sum_{j\in C}\hat T_j,\qquad
\hat T_j=\begin{cases}t_{\operatorname{pos}(j)} & j\in\pi,\\ \hat t_j & j \text{ served by a drone trip}\end{cases}\qquad\text{[PAPER (1),(10)]}.$$
It is the sum of the arrival times of the *serving vehicle* at each customer; the return to the depot is not included.
A solution is feasible iff all rules of §§4--5 hold (no violation is "paid for" by a penalty).

## 7. Neighbourhood adjacency (decision D1) [ASSUMPTION]

The paper uses "adjacent" in every neighbourhood definition (Sec 5.3) without defining it. Let $\sigma=(\sigma_0,\dots,\sigma_{S-1})$
be the *skeleton*, the non-FLP entries of $\pi$ ($\sigma_0=\sigma_{S-1}=0$), and call the arc $(\sigma_g,\sigma_{g+1})$ *gap* $g$,
$g=0,\dots,S-2$. Define the half-step coordinate
$$h(x)=\begin{cases}2g & x\in C\text{ is truck-served and }x=\sigma_g,\\ 2g+1 & x\in C\text{ is drone-served and its launch FLP lies in gap }g,\end{cases}\qquad h(\text{gap }g)=2g+1.$$
Two entities $x,y$ are **adjacent within window $w$** iff $\lvert h(x)-h(y)\rvert\le 2w$. The default is $w=3$ (a configuration value).
This is a candidate pre-filter only; feasibility is decided exclusively by §§4--5. Counting skeleton nodes rather than raw route
positions makes the neighbourhood independent of how many FLPs the current solution happens to use.

## 8. Randomness and seeds [ASSUMPTION]

The paper specifies no generator or seed policy. All randomness is produced by xoshiro256\*\* (Blackman & Vigna 2018) seeded
through SplitMix64 (Steele, Lea & Flood 2014), with the following fixed transformations (so that results are identical on every platform):
$$U=2^{-53}\Big\lfloor \tfrac{X}{2^{11}}\Big\rfloor\in[0,1),\qquad x\sim\mathcal U[\text{lo},\text{hi})\ \text{as}\ \text{lo}+(\text{hi}-\text{lo})\,U,$$
unbiased integers by rejection sampling on $X$, and Fisher--Yates for permutations. An independent stream is derived for each purpose and index:
$$\operatorname{derive}(b,\kappa,n)=\operatorname{SM}\big(\operatorname{SM}\big(\operatorname{SM}(b)\oplus\kappa\,\gamma_1\big)\oplus n\,\gamma_2\big),\qquad
\gamma_1=\texttt{0xd1b54a32d192ed03},\ \gamma_2=\texttt{0x8cb92ba72f3d8dd7},$$
where $\operatorname{SM}(s)$ denotes the first SplitMix64 output from state $s$, $b$ the base seed, $\kappa$ the stream
(instance, initial solution, VNS, shake) and $n$ a dataset or replication index.
