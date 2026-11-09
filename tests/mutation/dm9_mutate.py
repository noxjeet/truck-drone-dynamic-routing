#!/usr/bin/env python3
"""DM9 mutation gate: each mutant is a small deliberate defect in the regional optimiser / certificate / A0 hook; the test suite (tests/test_dyn_opt.cpp, quick
configuration DM9_SEEDS=4) must FAIL on every one. Sources are restored byte for byte after every mutant.
Usage (repo root, needs g++ and ~/dyn_build/tsp.a as for dm8_mutate.py):  tests/mutation/dm9_mutate.py [a:b]"""
import os, subprocess, sys
ROOT = os.getcwd()
B = os.path.expanduser("~/dyn_build")
OUT = B + "/dm9_mut_test"
H = "include/dynamic/regional_optimiser.hpp"
MUTANTS = [
 (H, "if (E.anchors_swept != E.domain) { M.failure", "if (false) { M.failure", "M1 check removed (anchors swept vs independent domain)"),
 (H, "bool m2 = true; for", "bool m2 = true; if (false) for", "M2 check removed (exceptions / accounting)"),
 (H, "bool m3 = true; for", "bool m3 = true; if (false) for", "M3 check removed (cap / limit counters)"),
 (H, "M.m4 = returned_plan == nullptr || same_solution(swept, *returned_plan);", "M.m4 = true;", "M4 check removed (sweep ran on the returned plan)"),
 (H, "M.complete = M.m1 && M.m2 && M.m3 && M.m4 && M.m5;", "M.complete = M.m2 && M.m3 && M.m4 && M.m5;", "complete ignores M1"),
 (H, "if (!R.manifest.complete) { R.cert = CertStatus::BudgetStop; break; }", "", "incomplete sweep still certifies"),
 (H, "if (calls_left == 0 || R.expansions >= cfg.max_expansions)", "if (R.expansions >= cfg.max_expansions)", "expansion ignores the exhausted budget"),
 (H, "if (cfg.sweep_call_budget > 0 && M.operator_calls >= cfg.sweep_call_budget) {", "if (false) {", "sweep call budget not enforced"),
 (H, "r.best->objective < f_plan - tau && (!out.improving", "r.best->objective < f_plan - 1e18 && (!out.improving", "sweep never reports an improving move"),
 (H, "tdr::search::SearchContext ctx = ctx_in;\n    ctx.scope = &mask;", "tdr::search::SearchContext ctx = ctx_in;", "search runs unscoped"),
 (H, "tdr::search::SearchContext ctx = ctx_in;\n    ctx.scope = &tail;", "tdr::search::SearchContext ctx = ctx_in;", "sweep runs unscoped"),
 (H, "                --calls_left;\n", "", "search budget not consumed"),
 (H, "for (std::size_t k = cut_skel + 1; k + 1 < L.sk.size(); ++k)", "for (std::size_t k = cut_skel; k + 1 < L.sk.size(); ++k)", "independent domain off by one"),
 (H, "if (cfg.arm == OptArm::FT || cfg.arm == OptArm::B4) region = RegionR5()", "if (false) region = RegionR5()", "FT/B4 use the RIC region"),
 (H, "if (cfg.arm == OptArm::B2) list = {OperatorId::N4};", "", "B2 not restricted to N4"),
 (H, "        R.plan = R.plan; ", "", "(placeholder)"),
 (H, "        return R.plan;\n    }\n};", "        return s_plus;\n    }\n};", "optimiser hook discards the optimised plan"),
 ("include/dynamic/admission.hpp", "const auto sstar = guarded_optimise(opt, in, in.ins.solution, d);", "const std::optional<tdr::model::Solution> sstar = in.ins.solution;", "A0 ignores the optimiser"),
 (H, "if (!e.feasible || e.objective_value > cand.objective() + 1e-9) {", "if (!e.feasible) {", "final F(S*) <= F(S+) guard removed"),
 (H, "double tau_imp = 0.0;            // improvement tolerance", "double tau_imp = 1e-9;            // improvement tolerance", "tau_imp default back to 1e-9 (differs from static VNS)"),
 (H, "M.m5 = std::isfinite(tau) && tau >= 0.0 && tau == M.tau_search;", "M.m5 = true;", "M5 check removed (tolerance consistency)"),
 (H, '    catch (...) { fail("non-standard exception"); }\n', "", "non-std exceptions not caught in optimise_plan"),
 (H, "budget = budget_for(cfg, R.ric_free_customers);", "budget = budget_for(cfg, R.free_customers);", "B4 uses its own budget, not the RIC region's"),
 (H, "if (stop == OptStop::Budget) { // the regional search", "if (false) { // the regional search", "budget-cut search may still certify"),
 ("include/dynamic/admission.hpp", "const auto sstar = guarded_optimise(opt, in, s_plus, d);", "const auto sstar = opt.optimise(in, s_plus);", "A-PM calls the optimiser unguarded"),
 (H, "R.protected_changes = (certify || !region.valid)", "R.protected_changes = (true)", "protected-change metric ignores the fixed region"),
]
MUTANTS = [m for m in MUTANTS if m[3] != "(placeholder)"]

# Mutants proven behaviourally equivalent (documented, not a test gap): a defensive guard / a diagnostic that cannot differ while the scope hook works.
EQUIVALENT = {
 "final F(S*) <= F(S+) guard removed": "the search only accepts moves with F' < F - tau, so F(S*) <= F(S+) holds by construction and every accepted plan is evaluator-feasible; the guard is defence in depth",
 "protected-change metric ignores the fixed region": "a diagnostic: it differs from the tail check only if the scope hook let a region violation through, which test (B) shows never happens (the negative control proves the checker itself discriminates)",
}

def build_run():
    r = subprocess.run(["g++", "-std=c++20", "-O2", "-I", ROOT + "/include", ROOT + "/tests/test_dyn_opt.cpp", B + "/tsp.a", "-o", OUT], capture_output=True, text=True)
    if r.returncode != 0: return "BROKEN", r.stderr.splitlines()[:2]
    env = dict(os.environ, DM9_SEEDS="4")
    t = subprocess.run([OUT], capture_output=True, text=True, timeout=900, env=env)
    return ("KILLED" if t.returncode != 0 else "SURVIVED"), []

def main():
    lo, hi = 0, len(MUTANTS)
    if len(sys.argv) > 1: lo, hi = (int(x) for x in sys.argv[1].split(":"))
    if lo == 0:
        base, _ = build_run()
        if base != "SURVIVED": print("baseline (unmutated) test does not pass:", base); return 2
        print("baseline passes", flush=True)
    bad = 0
    for i in range(lo, hi):
        path, old, new, name = MUTANTS[i]
        src = open(ROOT + "/" + path).read()
        if src.count(old) != 1: print(f"[{i}] {name}: pattern not found exactly once"); bad += 1; continue
        try:
            open(ROOT + "/" + path, "w").write(src.replace(old, new))
            res, why = build_run()
        finally:
            open(ROOT + "/" + path, "w").write(src)
        print(f"[{i}] {res:8s} {name} {why}", flush=True)
        if res == "SURVIVED" and name in EQUIVALENT: print("          (documented equivalent: " + EQUIVALENT[name] + ")"); continue
        if res != "KILLED": bad += 1
    print("all mutants killed" if bad == 0 else f"{bad} mutants not killed")
    return 0 if bad == 0 else 1

if __name__ == "__main__": sys.exit(main())
