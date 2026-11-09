#!/usr/bin/env python3
"""[DM8] Mutation gate G4: every guard line added by DM8 is mutated in turn; tests/test_dyn_scope.cpp must FAIL (compile error counts as 'broken', not 'killed')
for each one. The sources are restored byte for byte after every mutant.  Usage: tests/mutation/dm8_mutate.py   (run from the repo root; needs g++)"""
import os, shutil, subprocess, sys, tempfile

ROOT = subprocess.check_output(["git", "rev-parse", "--show-toplevel"], text=True).strip()
B = os.path.expanduser("~/dyn_build")
OUT = os.path.join(tempfile.gettempdir(), "dm8_mutant")
V = "detail_scope::violation(ctx, cand, m)"
MUTANTS = [
    ("include/search/trip_relocate.hpp", V, "ScopeViolation::None", "N1/N2 judge: hook not consulted"),
    ("include/search/trip_exchange.hpp", V, "ScopeViolation::None", "N3/N6 judge: hook not consulted"),
    ("include/search/neighbourhood_n5.hpp", V, "ScopeViolation::None", "N5: hook not consulted"),
    ("include/search/neighbourhood_n7.hpp", V, "ScopeViolation::None", "N7: hook not consulted"),
    ("include/search/neighbourhood_n8.hpp", V, "ScopeViolation::None", "N8: hook not consulted"),
    ("include/search/neighbourhood_n4.hpp", "detail_scope::violation(ctx, cand, move) : ScopeViolation::None; sv != ScopeViolation::None) {\n                detail_scope::count_candidate(res, sv); // [DM8] stopped before the evaluator",
     "ScopeViolation::None : ScopeViolation::None; sv != ScopeViolation::None) {\n                detail_scope::count_candidate(res, sv); // [DM8] stopped before the evaluator", "N4 Reject path: hook not consulted"),
    ("include/search/neighbourhood_n4.hpp", "const ScopeViolation sv = opt.evaluate_candidates ? detail_scope::violation(ctx, cand, move) : ScopeViolation::None; // [DM8]\n",
     "const ScopeViolation sv = ScopeViolation::None; // [DM8]\n", "N4 judge_as_is: hook not consulted"),
    ("include/search/neighbourhood_n4.hpp", "detail_scope::violation(ctx, var, move) : ScopeViolation::None; // [DM8] per variant", "ScopeViolation::None : ScopeViolation::None; // [DM8] per variant", "N4 variants: hook not consulted"),
    ("include/search/neighbourhood_n4.hpp", "rec.variants_evaluated == 0 && rec.variants_scope_skipped > 0", "false", "N4: all-variants-skipped candidate not scope-rejected"),
    ("include/search/neighbourhood_n4.hpp", "                ++res.scope_variants_skipped;\n", "", "N4: skipped variants not counted"),
    ("include/search/neighbourhood_n4.hpp", "                ++res.evaluator_calls_avoided;\n                if (first_scope", "                if (first_scope", "N4: skipped variant calls not counted as avoided"),
    ("include/search/neighbourhood_n4.hpp", "} else if (!reason_taken) {", "} else if (first) {", "N4: reason taken from the first VARIANT (not the first evaluated)"),
    ("include/search/neighbourhood_n4.hpp", "                reason_taken = true;\n", "", "N4: reason_taken never set"),
    ("include/search/shakes.hpp", "if (ctx.scope != nullptr) { res.status = ShakeStatus::InvalidParameters; return false; }", "", "shakes: scoped call not refused"),
    ("include/search/move_types.hpp", "    ++res.evaluator_calls_avoided;\n}", "}", "count_candidate: avoided calls not counted"),
    ("include/search/move_types.hpp", "    ++res.scope_by_violation[static_cast<std::size_t>(v)];\n", "", "count_candidate: violation class not counted"),
    ("include/search/move_types.hpp", "ctx.scope != nullptr ? ctx.scope->check(cand, m) : ScopeViolation::None", "ScopeViolation::None", "violation(): scope never consulted"),
    ("include/dynamic/scope_mask.hpp", "if (mv.partner != tdr::model::kInvalidNodeId && !is_free(mv.partner)) return V::MoveAnchorNotFree;", "", "mask: partner not checked"),
    ("include/dynamic/scope_mask.hpp", "if (mv.customer != tdr::model::kInvalidNodeId && !is_free(mv.customer)) return V::MoveAnchorNotFree;", "", "mask: anchor not checked"),
    ("include/dynamic/scope_mask.hpp", "if (hit != protected_truck_) return V::ProtectedTruckCustomerChanged;", "", "mask: protected truck customers not enforced"),
    ("include/dynamic/scope_mask.hpp", "if (n != 1 || found->drone_id != k.drone || found->launch_node != k.launch || found->recovery_node != k.recovery) return V::ProtectedDroneCustomerChanged;", "", "mask: protected trips not enforced"),
    ("include/dynamic/scope_mask.hpp", "if (lp == route.end() || std::find(lp + 1, route.end(), k.recovery) == route.end()) return V::ProtectedTripDependencyBroken;", "", "mask: trip dependency not enforced"),
    ("include/dynamic/scope_mask.hpp", "return a != b ? V::PinnedNodeMissing : V::PinnedOrderChanged;", "return V::None;", "mask: pinned nodes not enforced"),
    ("include/dynamic/scope_mask.hpp", "for (NodeId c : R.free_customers) if (pinned.count(c) == 0) m.free_.push_back(c);", "for (NodeId c : R.free_customers) m.free_.push_back(c);", "mask: pinned recovery customers left in the free set"),
    ("include/dynamic/scope_mask.hpp", "if (route.size() < prefix_.size() + suffix_.size() || !std::equal(prefix_.begin(), prefix_.end(), route.begin())) return V::ProtectedPrefixChanged;", "", "mask: protected prefix not enforced"),
    ("include/dynamic/scope_mask.hpp", "if (!std::equal(suffix_.begin(), suffix_.end(), route.end() - static_cast<std::ptrdiff_t>(suffix_.size()))) return V::ProtectedSuffixChanged;", "", "mask: protected suffix not enforced"),
    ("include/dynamic/scope_mask.hpp", "if (lp == route.end() || li < P || li >= end) return V::FreeTripOutsideRegion;", "", "mask: free-trip launch window not enforced"),
    ("include/dynamic/scope_mask.hpp", "if (rp == route.end() || static_cast<std::size_t>(rp - route.begin()) > end) return V::FreeTripOutsideRegion;", "", "mask: free-trip recovery window not enforced"),
    ("include/dynamic/scope_mask.hpp", "m.prefix_ = plan.truck_route;", "", "mask: R0 route not protected"),
    ("include/search/move_types.hpp", "if (ctx.scope == nullptr || opt.evaluate_candidates) return false;", "return false;", "refuse(): scoped call with evaluate_candidates=false not refused"),
    ("include/search/neighbourhood_n1.hpp", "    if (detail_scope::refuse(ctx, opt, res, OperatorId::N1)) return res; // [DM8] a scope requires evaluate_candidates\n", "", "N1: precondition guard removed"),
    ("include/search/neighbourhood_n2.hpp", "    if (detail_scope::refuse(ctx, opt, res, OperatorId::N2)) return res; // [DM8] a scope requires evaluate_candidates\n", "", "N2: precondition guard removed"),
    ("include/search/neighbourhood_n3.hpp", "    if (detail_scope::refuse(ctx, opt, res, OperatorId::N3)) return res; // [DM8] a scope requires evaluate_candidates\n", "", "N3: precondition guard removed"),
    ("include/search/neighbourhood_n4.hpp", "    if (detail_scope::refuse(ctx, opt, res, OperatorId::N4)) return res; // [DM8] a scope requires evaluate_candidates\n", "", "N4: precondition guard removed"),
    ("include/search/neighbourhood_n5.hpp", "    if (detail_scope::refuse(ctx, opt, res, OperatorId::N5)) return res; // [DM8] a scope requires evaluate_candidates\n", "", "N5: precondition guard removed"),
    ("include/search/neighbourhood_n6.hpp", "    if (detail_scope::refuse(ctx, opt, res, OperatorId::N6)) return res; // [DM8] a scope requires evaluate_candidates\n", "", "N6: precondition guard removed"),
    ("include/search/neighbourhood_n7.hpp", "    if (detail_scope::refuse(ctx, opt, res, OperatorId::N7)) return res; // [DM8] a scope requires evaluate_candidates\n", "", "N7: precondition guard removed"),
    ("include/search/neighbourhood_n8.hpp", "    if (detail_scope::refuse(ctx, opt, res, OperatorId::N8)) return res; // [DM8] a scope requires evaluate_candidates\n", "", "N8: precondition guard removed"),
    ("include/dynamic/scope_mask.hpp", "if (unrestricted_) return V::None;", "", "mask: full mask not unrestricted"),
]

# Mutants proven behaviourally equivalent (documented, not a test gap).
EQUIVALENT = {"mask: full mask not unrestricted": "the full mask has no protected or pinned identities and is_free() is true via unrestricted_, so the early return is only a shortcut"}

def build_run():
    r = subprocess.run(["g++", "-std=c++20", "-O1", "-I", ROOT + "/include", "-I", ROOT + "/tests/reference", ROOT + "/tests/test_dyn_scope.cpp", B + "/tsp.a", "-o", OUT], capture_output=True, text=True)
    if r.returncode != 0: return "BROKEN", r.stderr.splitlines()[:2]
    t = subprocess.run([OUT], capture_output=True, text=True, timeout=900)
    return ("KILLED" if t.returncode != 0 else "SURVIVED"), []

def main():
    # optional slice "a:b" of the mutant list (chunked runs); the baseline is only checked when the slice starts at 0
    lo, hi = 0, len(MUTANTS)
    if len(sys.argv) > 1: lo, hi = (int(x) for x in sys.argv[1].split(":"))
    if lo == 0:
        base, _ = build_run()
        if base != "SURVIVED": print("baseline (unmutated) test does not pass:", base); return 2
        print("baseline passes", flush=True)
    bad = 0
    for path, old, new, name in MUTANTS[lo:hi]:
        p = os.path.join(ROOT, path); orig = open(p).read()
        if orig.count(old) != 1: print("PATTERN-ERROR", name, orig.count(old)); bad += 1; continue
        try:
            open(p, "w").write(orig.replace(old, new))
            verdict, info = build_run()
        finally:
            open(p, "w").write(orig)
        if verdict == "SURVIVED" and name in EQUIVALENT: verdict = "EQUIVALENT"; info = [EQUIVALENT[name]]
        print("%-10s %s %s" % (verdict, name, info), flush=True)
        if verdict not in ("KILLED", "EQUIVALENT"): bad += 1
    print("mutants %d..%d of %d, not killed: %d" % (lo, hi, len(MUTANTS), bad))
    return 0 if bad == 0 else 1

if __name__ == "__main__": sys.exit(main())
