#!/usr/bin/env python3
"""M9 reference point (stdlib only): the EXACT optimum of the truck-only problem for the 10-customer instances.

With no drone trip the objective is the sum over customers of the truck's service-arrival time, i.e. a minimum-latency (travelling-repairman)
problem with a service time at every customer: solved exactly by dynamic programming over (visited set, last customer). The instance data
(depot, customer coordinates) come from the node table in results.json; the formula is first checked against the evaluator's objective
of every drone-free best solution found by VNS (must agree to rounding). Compare with the VNS best objective of the same instance:
 * truck-only optimum <= VNS best, always, when the VNS best is drone-free;
 * gap = (VNS z_best - truck-only optimum) / optimum tells how close VNS gets to the optimum of the drone-free sub-problem.
The truck-only optimum is an UPPER bound on the true optimum (drone trips might still help), not a lower bound and not a MILP replacement.

usage: m9_exact_truck_only.py RESULTS_ROOT
"""
import json, math, sys

ROOT = sys.argv[1] if len(sys.argv) > 1 else "results/m9"
V_TRUCK = 15.0
SERVICE = 30.0

def latency_opt(pts):
    """pts[0] = depot, pts[1..n] customers. Returns the minimum sum of service-arrival times (seconds)."""
    n = len(pts) - 1
    d = [[math.hypot(a[0] - b[0], a[1] - b[1]) / V_TRUCK for b in pts] for a in pts]
    INF = float("inf")
    # state (mask, last): minimum over orders of sum of arrival times so far; ARRIVAL is the arrival instant, a customer is left SERVICE seconds later.
    # Because every later customer's arrival depends on the elapsed time, carry (cost, time) pairs lexicographically is NOT valid; use the standard
    # trick: cost-to-go form with the number of remaining customers: appending a leg of length L delays all (n - |mask|) remaining customers by L.
    full = (1 << n) - 1
    dp = [[INF] * n for _ in range(1 << n)]      # dp[mask][j]: total delay contribution accumulated so far (legs weighted by remaining customers)
    for j in range(n):
        dp[1 << j][j] = d[0][j + 1] * n          # leg depot -> j delays all n customers
    for mask in range(1, 1 << n):
        k = bin(mask).count("1")
        for j in range(n):
            c = dp[mask][j]
            if c == INF: continue
            rem = n - k
            for t in range(n):
                if mask >> t & 1: continue
                # leaving j: service SERVICE at j (delays the rem remaining customers) plus the leg j -> t
                v = c + (SERVICE + d[j + 1][t + 1]) * rem
                nm = mask | (1 << t)
                if v < dp[nm][t]: dp[nm][t] = v
    # service times of the LAST customer do not delay anyone (rem = 0): objective = arrival-time sum
    return min(dp[full][j] for j in range(n))

def route_objective(route, pts_by_id, depot_id=0):
    t = 0.0; total = 0.0
    seq = route[:]
    for a, b in zip(seq, seq[1:]):
        t += math.hypot(pts_by_id[a][0] - pts_by_id[b][0], pts_by_id[a][1] - pts_by_id[b][1]) / V_TRUCK
        if b != depot_id:
            total += t
            t += SERVICE
    return total

rows = []
check_max = 0.0
for s in (1, 2, 3):
    d = json.load(open(f"{ROOT}/n10_seed{s}/results.json"))
    for ds in d["datasets"]:
        nodes = {nd["id"]: (nd["x_m"], nd["y_m"]) for nd in ds["nodes"]}
        cust = sorted(nd["id"] for nd in ds["nodes"] if nd["type"] == "customer")
        pts = [nodes[0]] + [nodes[c] for c in cust]
        opt = latency_opt(pts)
        # formula check against every drone-free best solution
        for r in ds["runs"]:
            if r["num_trips"] == 0:
                ro = route_objective(r["best_solution"]["truck_route"], nodes)
                check_max = max(check_max, abs(ro - r["best_objective_s"]))
        best = min(r["best_objective_s"] for r in ds["runs"])
        avg = sum(r["best_objective_s"] for r in ds["runs"]) / len(ds["runs"])
        hit = sum(1 for r in ds["runs"] if abs(r["best_objective_s"] - opt) < 1e-6 and r["num_trips"] == 0)
        rows.append((s, ds["dataset_index"], opt / 60, best / 60, avg / 60, 100 * (best - opt) / opt, 100 * (avg - opt) / opt, hit))

print(f"formula check: max |route objective - evaluator objective| over all drone-free best solutions = {check_max:.3e} s")
print("exact truck-only optimum (minutes) vs VNS, 10 customers (12 instances, 10 runs each)")
print(f"{'set':>3} {'ds':>2} | {'exact truck-only':>16} {'VNS z_best':>10} {'VNS z_avg':>10} | {'best gap %':>10} {'avg gap %':>9} | {'runs hitting the optimum':>24}")
for r in rows:
    print(f"{r[0]:>3} {r[1]:>2} | {r[2]:>16.3f} {r[3]:>10.3f} {r[4]:>10.3f} | {r[5]:>+10.4f} {r[6]:>+9.4f} | {r[7]:>21}/10")
print(f"mean best gap {sum(r[5] for r in rows)/len(rows):+.4f} %, mean avg gap {sum(r[6] for r in rows)/len(rows):+.4f} %, instances where VNS z_best equals the truck-only optimum: {sum(1 for r in rows if abs(r[5])<1e-9)}/{len(rows)}; "
      f"instances where VNS z_best is BELOW it (drones helped): {sum(1 for r in rows if r[5] < -1e-9)}/{len(rows)}")
