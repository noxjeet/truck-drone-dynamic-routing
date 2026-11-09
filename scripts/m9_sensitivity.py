#!/usr/bin/env python3
"""M9 labelled sensitivity diagnostics (reads results/m9 seed set 1 and results/m9_sensitivity; stdlib only).
Each variant changes ONE instance/vehicle parameter via `--set`; nothing in the code or in the defaults changes."""
import json, sys

ROOT = sys.argv[1] if len(sys.argv) > 1 else "results"
PAPER_Z_AVG = {10: 2097.0, 20: 7500.0, 50: 20782.0}   # mean of Table 3 z_avg over the four instances of that size

def load(path):
    d = json.load(open(path + "/results.json"))
    runs = [r for ds in d["datasets"] for r in ds["runs"]]
    best = [r["best_objective_min"] for r in runs]
    per_ds = [sum(r["best_objective_min"] for r in ds["runs"]) / len(ds["runs"]) for ds in d["datasets"]]
    n7 = {k: sum(o[k] for r in runs for o in r["operators"] if o["op"] == "N7") for k in ("calls", "no_candidate", "improving", "sa_accepted", "sa_rejected")}
    return dict(z_avg=sum(per_ds) / len(per_ds), trips=sum(r["num_trips"] for r in runs) / len(runs),
                zero=sum(1 for r in runs if r["num_trips"] == 0), runs=len(runs), impr=sum(r["improvement_pct"] for r in runs) / len(runs),
                init=sum(ds["initial"]["objective_min"] for ds in d["datasets"]) / len(d["datasets"]),
                init_trips=sum(ds["initial"]["num_trips"] for ds in d["datasets"]) / len(d["datasets"]),
                n7_feasible_pct=100.0 * (n7["improving"] + n7["sa_accepted"] + n7["sa_rejected"]) / n7["calls"], n7_improving=n7["improving"],
                n7_cand=n7["improving"] + n7["sa_accepted"] + n7["sa_rejected"],
                flp=sum(ds["num_flp_candidates"] for ds in d["datasets"]) / len(d["datasets"]))

print("Sensitivity diagnostics, seed set 1 (4 datasets x 10 runs per row); objective in minutes. Rows other than 'default' are NOT the protocol.")
print(f"{'n':>3} {'variant':>22} | {'mean initial':>12} {'init trips':>10} | {'mean z_avg':>10} {'impr %':>7} | {'drone trips in best (mean)':>26} {'runs with 0 trips':>17} | {'N7 feasible cand. % of calls':>28} {'N7 improving':>12} | {'paper mean z_avg':>16} {'ours/paper':>10}")
for n in (10, 20, 50):
    for name, path in (("default (centre, 1800 s)", f"{ROOT}/m9/n{n}_seed1"), ("depot corner", f"{ROOT}/m9_sensitivity/corner_n{n}"),
                       ("endurance 3600 s", f"{ROOT}/m9_sensitivity/endurance3600_n{n}")):
        v = load(path)
        print(f"{n:>3} {name:>22} | {v['init']:>12,.1f} {v['init_trips']:>10.1f} | {v['z_avg']:>10,.1f} {v['impr']:>7.2f} | {v['trips']:>26.2f} {str(v['zero'])+'/'+str(v['runs']):>17} | {v['n7_feasible_pct']:>28.1f} {v['n7_improving']:>12} | {PAPER_Z_AVG[n]:>16,.0f} {v['z_avg']/PAPER_Z_AVG[n]:>10.2f}")

# ---- iteration-budget sensitivity: vns.iter_max 2000 (pilot default, protocol) vs 8000 on the same instances / run indices (seed set 1) ----
def per_ds(path):
    d = json.load(open(path + "/results.json")); out = []
    for ds in d["datasets"]:
        b = [r["best_objective_min"] for r in ds["runs"]]
        out.append((ds["dataset_index"], min(b), sum(b) / len(b), sum(r["last_best_iteration"] for r in ds["runs"]) / len(b), sum(r["num_trips"] for r in ds["runs"]) / len(b)))
    return out
print("\nIteration-budget sensitivity (seed set 1; objective in minutes; same instances and run indices; the geometric schedule T(k) = T0 * 0.995^k is identical, the run simply continues past k = 2000, so this is a budget change, not a re-tuning)")
print(f"{'n':>3} {'ds':>2} | {'2000: z_best':>12} {'z_avg':>9} | {'8000: z_best':>12} {'z_avg':>9} {'mean last-best iter':>19} {'mean trips':>10} | {'z_avg change':>12}")
for n, dirs in ((20, ["iter8000_n20"]), (50, ["iter8000_n50_ds01", "iter8000_n50_ds23"])):
    new = [x for p in dirs for x in per_ds(f"{ROOT}/m9_sensitivity/{p}")]
    for b, x in zip(per_ds(f"{ROOT}/m9/n{n}_seed1"), new):
        print(f"{n:>3} {b[0]:>2} | {b[1]:>12,.1f} {b[2]:>9,.1f} | {x[1]:>12,.1f} {x[2]:>9,.1f} {x[3]:>19.0f} {x[4]:>10.2f} | {100*(x[2]-b[2])/b[2]:>+11.1f}%")
