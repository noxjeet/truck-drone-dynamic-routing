#!/usr/bin/env python3
"""M9 analysis of the tdr_static validation runs (stdlib only, deterministic).

Reads results/m9/n{10,20,50}_seed{1,2,3}/results.json (+ timing.json) and writes
  results/m9/summary.json   machine-readable numbers
  stdout                    the tables quoted in docs/static/validation_vs_paper.md

Nothing here changes the algorithm; it only reads outputs. Objective unit: the runs report seconds; the paper's unit is not
stated, so all comparison columns are in MINUTES (= seconds / 60), see the unit hypothesis in the document (section 3).
"""
import json, math, os, statistics as st, sys

ROOT = sys.argv[1] if len(sys.argv) > 1 else "results/m9"
SIZES = (10, 20, 50)
SEEDS = (1, 2, 3)

# Han, Chi & Sun 2026, Table 3 (VNS columns and MILP z), extraction doc claude/han-chi-sun-2026-extraction.md.
PAPER = {
    10: [("1.1", 1020, 1020, 1020, 1.72), ("1.2", 2098, 2098, 2098, 2.07), ("1.3", 3192, 2911, 3097, 2.83), ("1.4", 1989, 1989, 2172, 2.19)],
    20: [("2.1", 11385, 6239, 6890, 4.92), ("2.2", 14270, 8011, 9137, 5.81), ("2.3", 10284, 6042, 6701, 4.07), ("2.4", 13292, 6839, 7273, 4.46)],
    50: [("3.1", None, 19473, 20724, 10.23), ("3.2", None, 17820, 19050, 11.65), ("3.3", None, 20916, 21372, 11.93), ("3.4", None, 19041, 21981, 11.04)],
}
PAPER_FLP = {10: 214, 20: 837, 50: 3493}
T975 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228, 11: 2.201, 39: 2.023}

def mean(x): return sum(x) / len(x)
def sd(x): return st.stdev(x) if len(x) > 1 else 0.0
def ci95(x):
    n = len(x)
    if n < 2: return (float("nan"), float("nan"))
    h = T975[n - 1] * sd(x) / math.sqrt(n)
    return (mean(x) - h, mean(x) + h)
def f(x, p=1): return f"{x:,.{p}f}"

inst = []   # one record per (size, seed set, dataset)
ops_tot = {}
for n in SIZES:
    for s in SEEDS:
        d = json.load(open(f"{ROOT}/n{n}_seed{s}/results.json"))
        tm = json.load(open(f"{ROOT}/n{n}_seed{s}/timing.json"))
        assert d["experiment"]["ok"] and d["experiment"]["deterministic"]
        for ds in d["datasets"]:
            runs = ds["runs"]
            assert all(r["status"] == "ok" and r["verified"] for r in runs)
            best = [r["best_objective_min"] for r in runs]
            secs = [x["seconds"] for x in tm["runs"] if x["dataset_index"] == ds["dataset_index"]]
            rec = dict(n=n, seed_set=s, dataset=ds["dataset_index"], instance_seed=ds["instance_seed"], flp=ds["num_flp_candidates"],
                       init=ds["initial"]["objective_min"], init_trips=ds["initial"]["num_trips"],
                       z_best=min(best), z_avg=mean(best), z_worst=max(best), z_sd=sd(best),
                       impr=[r["improvement_pct"] for r in runs], trips=[r["num_trips"] for r in runs],
                       sec=mean(secs), guard=any(r["passes"]["guard_triggered"] for r in runs),
                       skipped=sum(r["passes"]["skipped"] for r in runs), iters=sum(r["iterations"] for r in runs),
                       best_updates=mean([r["best_updates"] for r in runs]), last_best=mean([r["last_best_iteration"] for r in runs]))
            rec["gap_avg_pct"] = 100.0 * (rec["z_avg"] - rec["z_best"]) / rec["z_best"]
            inst.append(rec)
            for r in runs:
                for o in r["operators"]:
                    a = ops_tot.setdefault((n, o["op"]), dict(calls=0, no_anchor=0, no_candidate=0, improving=0, sa_accepted=0, sa_rejected=0))
                    for k in a: a[k] += o[k]

out = {"instances": inst, "by_size": {}, "operators": {}}

print("== A. per instance (objective in minutes = seconds/60; 10 runs each) ==")
print(f"{'n':>3} {'set':>3} {'ds':>2} {'FLPs':>6} {'initial':>9} {'init trips':>10} {'z_best':>9} {'z_avg':>9} {'z_worst':>9} {'sd':>7} {'(avg-best)/best%':>16} {'impr%(mean)':>11} {'drone trips in best (min..max)':>30} {'s/run':>6}")
for r in inst:
    print(f"{r['n']:>3} {r['seed_set']:>3} {r['dataset']:>2} {r['flp']:>6} {f(r['init']):>9} {r['init_trips']:>10} {f(r['z_best']):>9} {f(r['z_avg']):>9} {f(r['z_worst']):>9} {f(r['z_sd']):>7} {r['gap_avg_pct']:>16.2f} {mean(r['impr']):>11.2f} {min(r['trips']):>14}..{max(r['trips']):<14} {r['sec']:>6.2f}")

print("\n== B. per size, over the 12 instances (4 datasets x 3 independent seed sets) ==")
print(f"{'n':>3} | {'mean z_best':>11} {'mean z_avg':>11} {'95% CI of mean z_avg':>24} {'min..max z_avg':>20} | {'mean (avg-best)/best %':>22} | {'mean impr %':>11} {'sd across inst.':>15} | {'mean initial':>12} | {'s/run':>6}")
for n in SIZES:
    R = [r for r in inst if r["n"] == n]
    za = [r["z_avg"] for r in R]; zb = [r["z_best"] for r in R]
    imp = [mean(r["impr"]) for r in R]
    lo, hi = ci95(za)
    by_set = [mean([r["z_avg"] for r in R if r["seed_set"] == s]) for s in SEEDS]
    out["by_size"][n] = dict(mean_z_best=mean(zb), mean_z_avg=mean(za), ci95_z_avg=[lo, hi], min_z_avg=min(za), max_z_avg=max(za),
                             mean_gap_avg_pct=mean([r["gap_avg_pct"] for r in R]), mean_impr_pct=mean(imp), sd_impr_pct=sd(imp),
                             mean_init=mean([r["init"] for r in R]), sec_per_run=mean([r["sec"] for r in R]), seed_set_means_z_avg=by_set,
                             flp_mean=mean([r["flp"] for r in R]),
                             zero_trip_runs=sum(1 for r in R for t in r["trips"] if t == 0), runs=sum(len(r["trips"]) for r in R),
                             mean_trips_in_best=mean([t for r in R for t in r["trips"]]), mean_init_trips=mean([r["init_trips"] for r in R]),
                             init_worse_than_best_pct=100.0 * sum(1 for r in R if r["init"] > r["z_best"]) / len(R))
    print(f"{n:>3} | {f(mean(zb)):>11} {f(mean(za)):>11} {f(lo)+'..'+f(hi):>24} {f(min(za))+'..'+f(max(za)):>20} | {mean([r['gap_avg_pct'] for r in R]):>22.2f} | {mean(imp):>11.2f} {sd(imp):>15.2f} | {f(mean([r['init'] for r in R])):>12} | {mean([r['sec'] for r in R]):>6.2f}")
    print(f"      seed-set means of z_avg: {', '.join(f(x) for x in by_set)}  (spread max-min = {f(max(by_set)-min(by_set))}, {100*(max(by_set)-min(by_set))/mean(by_set):.1f} % of the mean)")

print("\n== C. drone use in the returned best solutions ==")
for n in SIZES:
    b = out["by_size"][n]
    print(f"n={n}: runs with 0 drone trips in the best solution: {b['zero_trip_runs']}/{b['runs']}; mean trips in best {b['mean_trips_in_best']:.2f}; mean trips in initial {b['mean_init_trips']:.2f}; instances whose initial objective exceeds the best found: {b['init_worse_than_best_pct']:.0f} %")

print("\n== D. paper Table 3 (VNS columns) vs this implementation; objective of ours in MINUTES ==")
print(f"{'n':>3} | {'paper mean z_best':>17} {'paper mean z_avg':>16} {'paper range z_avg':>18} {'paper (avg-best)/best %':>23} | {'ours mean z_best':>16} {'ours mean z_avg':>15} {'ours range z_avg (12 inst.)':>28} {'ours (avg-best)/best %':>22} | {'ours/paper z_avg':>16}")
for n in SIZES:
    P = PAPER[n]
    pb = [x[2] for x in P]; pa = [x[3] for x in P]
    pg = mean([100.0 * (x[3] - x[2]) / x[2] for x in P])
    b = out["by_size"][n]
    ratio = b["mean_z_avg"] / mean(pa)
    out["by_size"][n].update(paper_mean_z_best=mean(pb), paper_mean_z_avg=mean(pa), paper_range_z_avg=[min(pa), max(pa)], paper_gap_avg_pct=pg, ratio_ours_to_paper_z_avg=ratio)
    print(f"{n:>3} | {f(mean(pb),0):>17} {f(mean(pa),0):>16} {f(min(pa),0)+'..'+f(max(pa),0):>18} {pg:>23.2f} | {f(b['mean_z_best'],0):>16} {f(b['mean_z_avg'],0):>15} {f(b['min_z_avg'],0)+'..'+f(b['max_z_avg'],0):>28} {b['mean_gap_avg_pct']:>22.2f} | {ratio:>16.2f}")
print("scale ratios (mean z_avg): paper 20/10 = %.2f, 50/20 = %.2f; ours 20/10 = %.2f, 50/20 = %.2f" % (
    mean([x[3] for x in PAPER[20]]) / mean([x[3] for x in PAPER[10]]), mean([x[3] for x in PAPER[50]]) / mean([x[3] for x in PAPER[20]]),
    out["by_size"][20]["mean_z_avg"] / out["by_size"][10]["mean_z_avg"], out["by_size"][50]["mean_z_avg"] / out["by_size"][20]["mean_z_avg"]))
# is the paper's mean z_avg inside our 95% CI / range per size?
for n in SIZES:
    b = out["by_size"][n]
    pm = b["paper_mean_z_avg"]
    print(f"n={n}: paper mean z_avg {pm:,.0f} {'inside' if b['ci95_z_avg'][0] <= pm <= b['ci95_z_avg'][1] else 'OUTSIDE'} our 95% CI of the mean {f(b['ci95_z_avg'][0],0)}..{f(b['ci95_z_avg'][1],0)}; {'within' if b['min_z_avg'] <= pm <= b['max_z_avg'] else 'outside'} our instance range")
print("FLP candidates: paper Table 4 = %s; ours (mean over 12 instances) = %s" % (PAPER_FLP, {n: round(out["by_size"][n]["flp_mean"]) for n in SIZES}))
print("per-instance VNS time: paper 1.72-11.93 s (Java 18, i5-9500H); ours (C++, 2000 passes, this VM) = %s s per run" % {n: round(out["by_size"][n]["sec_per_run"], 2) for n in SIZES})

print("\n== E. operator activity over all runs (3 seed sets x 4 datasets x 10 runs per size) ==")
print(f"{'n':>3} {'op':>3} {'calls':>9} {'no anchor %':>11} {'anchor, no feasible cand. %':>27} {'feasible cand. %':>16} {'improving':>9} {'SA-accepted':>11} {'SA-rejected':>11}")
for n in SIZES:
    for op in ("N1", "N2", "N3", "N4", "N5", "N6", "N7", "N8"):
        a = ops_tot[(n, op)]; c = a["calls"]
        feas = a["improving"] + a["sa_accepted"] + a["sa_rejected"]
        out["operators"][f"{n}/{op}"] = dict(a, anchor_fraction=(c - a["no_anchor"]) / c, feasible_given_anchor=feas / max(c - a["no_anchor"], 1))
        print(f"{n:>3} {op:>3} {c:>9} {100*a['no_anchor']/c:>11.1f} {100*a['no_candidate']/c:>27.1f} {100*feas/c:>16.1f} {a['improving']:>9} {a['sa_accepted']:>11} {a['sa_rejected']:>11}")

print("\n== F. search diagnostics ==")
for n in SIZES:
    R = [r for r in inst if r["n"] == n]
    print(f"n={n}: guard triggered in {sum(r['guard'] for r in R)} instances; skipped passes {sum(r['skipped'] for r in R)} of {sum(r['iters'] for r in R)}; mean best-updates/run {mean([r['best_updates'] for r in R]):.1f}; mean last-best iteration {mean([r['last_best'] for r in R]):.0f} of 2000")

json.dump(out, open(f"{ROOT}/summary.json", "w"), indent=1, sort_keys=True, default=float)
