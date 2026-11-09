#pragma once

// Dynamic extension (DM9): regional local optimiser, optimiser arms/baselines and the RIC-C certificate sweep (DM0 §8.3-8.5, §9, §12.2).
//
// WHAT THIS IS. A deterministic, SHAKELESS, scoped local search over the DM8 identity mask, started from the DM6 cheapest-insertion plan S+, plus a
// certificate sweep. Every candidate is a COMPLETE plan judged by the unchanged evaluator (tdr::timing::evaluate_solution through World::evaluate /
// the operators); nothing here decides feasibility or computes an objective.
//   * scope      : every operator call goes through the DM8 hook (SearchContext::scope = &mask) with evaluate_candidates = true (DM8 refuses otherwise) and an
//                  EXPLICIT anchor taken from the mask's free anchors in a fixed order. The Rng overloads (which draw over ALL customers) are never used, and
//                  no shake is ever invoked (DM8: a shake under any scope is InvalidParameters), so the static VNS is untouched.
//   * search     : first-improvement descent over the neighbourhood list N4,N5,N8,N3,N6,N7,N1,N2,N1,N2 (default_neighbourhood_list). For each operator the free anchors
//                  are tried in ascending order; the first anchor whose best candidate improves F by more than tau_imp is accepted and the list restarts (VNS k=0).
//                  A cycle with no improvement ends the search ("local-optimum" w.r.t. the SCOPED single moves). No simulated annealing: without shakes there is
//                  nothing to escape to, and a random acceptance would only add non-determinism. [documented default, DM9-D1]
//   * budget     : at most P = min(P_max, c_P (|free customers| + 1)) operator calls (one call = one operator at one anchor). Deterministic; wall-clock is recorded,
//                  never used. [DM9-D2; DM0 §9.2 heuristic c_P = 40]
//   * certificate: arm RIC-C only. After the regional search, a sweep over the WHOLE editable tail (cut-only mask, explicit anchors, all of N1..N8 over the complete
//                  anchor domain computed INDEPENDENTLY of the operators) either finds an improving single move (-> its footprint is hulled into the region, the region
//                  re-closed, the search continues) or certifies. `certified` = M1..M5 all hold and no move improves F by more than tau_imp; ANY failed condition,
//                  exception, N4 re-anchoring cap hit or exhausted budget gives `budget-stop`. certified is local optimality w.r.t. the single moves N1..N8 (window w,
//                  N4 re-anchor enumeration, FLP-only launches); it is NEVER a global-optimality claim, and the sweep is about F only (not Phi-aware).
// Phi chain (DM5 U1) is NOT here: the optimiser only returns S*; AdmitAPM installs S* iff Phi holds, else falls back to the cheapest Phi-feasible insertion (B1).

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

#include "dynamic/admission.hpp"
#include "dynamic/region_builder.hpp"
#include "dynamic/scope_mask.hpp"
#include "search/neighbourhood_n1.hpp"
#include "search/neighbourhood_n2.hpp"
#include "search/neighbourhood_n3.hpp"
#include "search/neighbourhood_n4.hpp"
#include "search/neighbourhood_n5.hpp"
#include "search/neighbourhood_n6.hpp"
#include "search/neighbourhood_n7.hpp"
#include "search/neighbourhood_n8.hpp"
#include "search/vns.hpp"

namespace tdr::dynamic {

using tdr::search::OperatorId;
using tdr::search::OperatorResult;
using tdr::search::OperatorStatus;

// ---- arms ----------------------------------------------------------------------------------------------------------------------------------
enum class OptArm {
    R0,    // no optimisation: S* = S+ (arm B1; identity)
    FR,    // fixed RIC: scoped search on the RIC region Lambda0 only, no expansion, no certificate
    RIC_C, // FR + certificate sweep over the whole tail + footprint expansion until certified or budget-stop
    FT,    // full editable tail (R5), same scoped search, no expansion
    B2,    // baseline B2: cheapest insertion + N4 only, restricted to the RIC region
    B4,    // baseline B4: full-tail scoped search with the SAME call budget as the RIC region would get (set by budget_from_ric)
};
inline const char* opt_arm_name(OptArm a) {
    switch (a) {
        case OptArm::R0: return "r0"; case OptArm::FR: return "fr"; case OptArm::RIC_C: return "ric-c";
        case OptArm::FT: return "ft"; case OptArm::B2: return "b2"; case OptArm::B4: return "b4";
    }
    return "?";
}
inline std::optional<OptArm> opt_arm_from_name(const std::string& s) {
    for (OptArm a : {OptArm::R0, OptArm::FR, OptArm::RIC_C, OptArm::FT, OptArm::B2, OptArm::B4}) if (s == opt_arm_name(a)) return a;
    return std::nullopt;
}

enum class CertStatus { NotRun, Certified, BudgetStop };
inline const char* cert_status_name(CertStatus c) { return c == CertStatus::Certified ? "certified" : (c == CertStatus::BudgetStop ? "budget-stop" : "not-run"); }

enum class OptStop { None, LocalOptimum, Budget, NoRegion, Identity, Error };
inline const char* opt_stop_name(OptStop s) {
    switch (s) { case OptStop::LocalOptimum: return "local-optimum"; case OptStop::Budget: return "budget"; case OptStop::NoRegion: return "no-region"; case OptStop::Identity: return "identity"; case OptStop::Error: return "error"; default: return "none"; }
}

// Fault injection for the certificate tests (T13). Honest runs leave every field at its default. Each fault must be CAUGHT by M1..M5 (never silently certified).
struct CertFaults {
    int drop_operator = -1;          // (i)   skip this operator index (0 = N1 .. 7 = N8) entirely
    int skip_last_anchor_of = -1;    // (ii)  skip the last anchor of this operator
    int force_cap_counter_of = -1;   // (iii) pretend this operator reported a positive N4 cap / limit counter
    int throw_in_anchor_of = -1;     // (iv)  raise an exception inside the first anchor of this operator
    bool sweep_other_plan = false;   // (v)   sweep a plan other than the returned one (the empty-move neighbour: S+ instead of S)
    bool sweep_tolerance_mismatch = false; // (M5) the sweep uses a tolerance different from the search's
    // exceptions outside the per-anchor sweep guard (0 none, 1 std::runtime_error, 2 a non-std exception (int)); must end in budget-stop (RIC-C) / S+ (other arms), never escape
    int throw_in_region = 0;         // while building the region
    int throw_in_search = 0;         // at the first operator call of the scoped search
    int throw_in_expansion = 0;      // right after a footprint expansion adopted the sweep's improving move
};

struct OptConfig {
    OptArm arm = OptArm::FR;
    int w = 3;                       // RIC margin window (static adjacency window unit)
    double c_P = 40.0;               // calls per (free customer + 1)                         [DM0 §9.2 heuristic]
    std::size_t p_max = 4000;        // hard ceiling on operator calls per optimisation       [DM9-D2]
    std::size_t calls_override = 0;  // >0: exactly this many calls (B4 uses the RIC budget; tests use tiny budgets)
    double tau_imp = 0.0;            // improvement tolerance: improving iff F' < F - tau_imp. Default 0 = strict '<', exactly the static VNS default (VnsOptions::min_improvement = 0, DM0 §8.3 M5);
                                     // configurable (>= 0), the SAME value drives search and sweep, and it is logged in the manifest (tau_imp, tau_search) and checked by M5
    std::size_t search_n4_cap = 100; // N4 re-anchoring budget in the search passes (static default)
    std::size_t sweep_n4_cap = 0;    // N4 re-anchoring budget in the certificate sweep; 0 = unlimited so `certified` is reachable [U7 proposed default]
    std::size_t max_expansions = 50; // RIC-C footprint expansions
    std::size_t sweep_call_budget = 0; // >0: a sweep needing more operator calls aborts incomplete (budget-stop)
    bool audit_sweep = false;        // FR/FT/B2/B4: run the sweep AFTER the search for diagnostics only (the plan is not changed, status stays NotRun)
    std::vector<OperatorId> ops;     // empty -> default_neighbourhood_list(); B2 forces {N4}
    CertFaults faults;               // tests only
};

// ---- instrumentation ----------------------------------------------------------------------------------------------------------------------
struct OpTally { std::size_t calls = 0, no_candidate = 0, improving = 0, accepted = 0; };

struct SweepOpEntry {
    OperatorId op = OperatorId::N1;
    std::size_t domain = 0;          // |A_o|, computed from the plan layout, NOT from the operator or the mask
    std::size_t anchors_swept = 0;   // anchors on which the operator was actually called
    std::size_t generated = 0, evaluated = 0, accepted = 0, rejected = 0, scope_rejected = 0, cap_reached = 0;
    std::size_t non_ok = 0, exceptions = 0, accounting_violations = 0;
};
struct SweepManifest {
    bool ran = false;
    std::array<SweepOpEntry, 8> ops;
    bool m1 = false, m2 = false, m3 = false, m4 = false, m5 = false;
    bool complete = false;           // the sweep itself was complete: M1..M5 all hold (it says nothing about the budget of the SEARCH that preceded it)
    std::string failure;             // text of the first failed condition ("" if none)
    std::string failed;              // short code of the first failed condition: M1..M5, BUDGET (search cut off before the sweep), SWEEP-BUDGET, EXCEPTION; "" if none
    double tau_imp = 0.0, tau_search = 0.0, f_plan = 0.0;   // tolerance used by the sweep / by the search (M5: equal, finite, >= 0)
    bool improving = false;          // some single move improves F by more than tau_imp
    double best_move_objective = 0.0;
    std::size_t operator_calls = 0, evaluator_calls = 0, out_of_scope_skipped = 0;
    std::size_t domain_total() const { std::size_t n = 0; for (const auto& o : ops) n += o.domain; return n; }
    std::size_t swept_total() const { std::size_t n = 0; for (const auto& o : ops) n += o.anchors_swept; return n; }
};

struct OptResult {
    tdr::model::Solution plan;       // S* (== S+ when nothing improved)
    OptArm arm = OptArm::R0;
    OptStop stop = OptStop::None;
    bool region_valid = false;
    std::string note;
    bool failed = false;             // an exception was caught: S+ is returned (RIC-C: cert = budget-stop)
    std::string failure_reason;
    double f_before = 0.0, f_after = 0.0;
    bool region_full_tail = false;             // the region (initial) spans [cut, end depot]
    std::string budget_source;                 // "own" | "override" | "ric-region" (B4: the budget of the corresponding RIC region)
    std::size_t ric_free_customers = 0;        // B4: free customers of the matching RIC region (the budget is derived from it)
    std::size_t free_customers = 0, budget_calls = 0, search_calls = 0, accepted_moves = 0, expansions = 0, sweeps = 0;
    std::size_t evaluator_calls = 0;           // search + final verification (certificate sweep counted separately)
    std::size_t sweep_evaluator_calls = 0, sweep_operator_calls = 0;
    std::array<OpTally, 8> by_op;
    ScopeStats scope;                          // scope rejections / avoided evaluator calls (search + sweeps)
    CertStatus cert = CertStatus::NotRun;
    SweepManifest manifest;                    // the LAST sweep
    std::size_t protected_changes = 0;         // check_region_preserved violations vs the arm's region (RIC-C: vs the whole tail); must be 0
    std::size_t commitment_violations = 0;     // changes behind/at the cut (vs R5); must be 0
    double seconds = 0.0;                      // wall clock, recorded only
    std::uint32_t request = 0;                 // filled by RegionalOptimizer
};

namespace detail_opt {
inline void throw_fault(int kind) { if (kind == 1) throw std::runtime_error("injected std::exception"); if (kind == 2) throw 42; }
inline std::size_t budget_for(const OptConfig& cfg, std::size_t free_customers) {
    return std::min(cfg.p_max, static_cast<std::size_t>(std::ceil(cfg.c_P * static_cast<double>(free_customers + 1))));
}
} // namespace detail_opt

// ---- helpers --------------------------------------------------------------------------------------------------------------------------------
namespace detail_opt {

inline OperatorResult apply(OperatorId op, const tdr::model::Solution& s, const tdr::search::SearchContext& ctx, std::size_t anchor, std::size_t n4_cap) {
    switch (op) {
        case OperatorId::N1: return tdr::search::n1_launch_node_relocate(s, ctx, anchor);
        case OperatorId::N2: return tdr::search::n2_recover_node_relocate(s, ctx, anchor);
        case OperatorId::N3: return tdr::search::n3_drone_service_node_relocate(s, ctx, anchor);
        case OperatorId::N4: { tdr::search::N4Options o; o.max_reanchor_variants = n4_cap; return tdr::search::n4_truck_service_node_relocate(s, ctx, static_cast<tdr::model::NodeId>(anchor), o); }
        case OperatorId::N5: return tdr::search::n5_td_service_node_swap(s, ctx, anchor);
        case OperatorId::N6: return tdr::search::n6_drone_drone_swap(s, ctx, anchor);
        case OperatorId::N7: return tdr::search::n7_truck_node_delete_drone_insert(s, ctx, static_cast<tdr::model::NodeId>(anchor));
        default: return tdr::search::n8_drone_node_delete_truck_insert(s, ctx, anchor);
    }
}
inline bool customer_anchored(OperatorId op) { return op == OperatorId::N4 || op == OperatorId::N7; }

// Free anchors of an operator, ascending/route order: truck customers (N4, N7) or trip indices (the rest).
inline std::vector<std::size_t> anchors_of(OperatorId op, const tdr::model::Solution& s, const ScopeMask& m, const tdr::model::NodeRegistry& nodes) {
    std::vector<std::size_t> a;
    if (customer_anchored(op)) { for (auto c : m.free_truck_customers(s, nodes)) a.push_back(static_cast<std::size_t>(c)); }
    else a = m.free_trip_indices(s);
    return a;
}

// Skeleton index of the skeleton node at or immediately before route position p.
inline std::size_t skel_at_or_before(const tdr::search::Skeleton& sk, std::size_t p) {
    std::size_t s = 0;
    for (std::size_t i = 0; i < sk.size(); ++i) if (sk.route_position[i] <= p) s = i;
    return s;
}

// Independent anchor domain of the tail [cut, end]: computed from the plan layout alone (no mask, no operator).
inline std::size_t independent_domain(OperatorId op, const tdr::model::Solution& s, const tdr::model::NodeRegistry& nodes, std::size_t cut_skel) {
    const PlanLayout L = layout_of(s, nodes);
    if (!L.ok) return 0;
    std::size_t n = 0;
    if (customer_anchored(op)) { for (std::size_t k = cut_skel + 1; k + 1 < L.sk.size(); ++k) if (nodes.contains(L.sk.node[k]) && nodes.at(L.sk.node[k]).type == tdr::model::NodeType::Customer) ++n; }
    else { for (const auto& t : L.trips) if (t.launch_gap >= cut_skel) ++n; }
    return n;
}

inline Region full_tail_region(const tdr::model::Solution& s, const tdr::model::NodeRegistry& nodes, std::size_t cut_skel) {
    const PlanLayout L = layout_of(s, nodes);
    if (!L.ok || cut_skel + 1 >= L.sk.size()) return detail_region::invalid(RegionKind::FullTail, L.ok ? "the cut leaves no editable gap" : L.why);
    return detail_region::finish(L, nodes, RegionKind::FullTail, cut_skel, cut_skel, L.last(), cut_skel, L.last());
}

// Nodes a move touched (anchor, partner, replaced node, new launch/recovery, the trips it names, its destination gap), mapped into skeleton indices of `plan`.
inline void footprint_skeletons(const tdr::search::Move& mv, const tdr::model::Solution& before, const tdr::model::Solution& after, const tdr::model::NodeRegistry& nodes,
                                std::size_t& lo, std::size_t& hi) {
    using tdr::model::NodeId;
    const auto skA = tdr::search::make_skeleton(after.truck_route, nodes);
    const auto skB = tdr::search::make_skeleton(before.truck_route, nodes);
    std::vector<NodeId> ids = {mv.customer, mv.partner, mv.replaced_node, mv.launch, mv.recovery};
    for (std::size_t ti : {mv.replaced_trip, mv.partner_trip})
        if (ti != tdr::search::kNoIndex && ti < before.drone_trips.size()) { const auto& t = before.drone_trips[ti]; ids.push_back(t.launch_node); ids.push_back(t.service_customer); ids.push_back(t.recovery_node); }
    if (mv.gap != tdr::search::kNoIndex) {
        if (mv.gap + 1 < skB.size()) { ids.push_back(skB.node[mv.gap]); ids.push_back(skB.node[mv.gap + 1]); }
        if (mv.gap + 1 < skA.size()) { ids.push_back(skA.node[mv.gap]); ids.push_back(skA.node[mv.gap + 1]); }
    }
    auto touch = [&](NodeId id) {
        if (id == tdr::model::kInvalidNodeId) return;
        for (std::size_t p = 0; p < after.truck_route.size(); ++p) if (after.truck_route[p] == id) { const std::size_t s = skel_at_or_before(skA, p); lo = std::min(lo, s); hi = std::max(hi, std::min(s + 1, skA.size() - 1)); return; }
    };
    for (NodeId id : ids) {
        touch(id);
        for (const auto& t : after.drone_trips) if (t.service_customer == id) { touch(t.launch_node); touch(t.recovery_node); }
    }
}

} // namespace detail_opt

// ---- scoped first-improvement search -----------------------------------------------------------------------------------------------------
struct SearchOutcome { OptStop stop = OptStop::LocalOptimum; };

inline SearchOutcome scoped_search(tdr::model::Solution& cur, double& f, const tdr::search::SearchContext& ctx_in, const ScopeMask& mask, const OptConfig& cfg,
                                   const std::vector<OperatorId>& list, std::size_t& calls_left, OptResult& R, std::vector<tdr::search::Move>* applied = nullptr,
                                   std::vector<tdr::model::Solution>* applied_before = nullptr) {
    tdr::search::SearchContext ctx = ctx_in;
    ctx.scope = &mask;
    const auto& nodes = ctx.nodes();
    bool improved = true;
    while (improved) {
        improved = false;
        for (OperatorId op : list) {
            const auto anchors = detail_opt::anchors_of(op, cur, mask, nodes);
            for (std::size_t a : anchors) {
                if (calls_left == 0) return {OptStop::Budget};
                --calls_left;
                ++R.search_calls;
                if (R.search_calls == 1) detail_opt::throw_fault(cfg.faults.throw_in_search);
                OpTally& T = R.by_op[static_cast<std::size_t>(op)];
                ++T.calls;
                OperatorResult r = detail_opt::apply(op, cur, ctx, a, cfg.search_n4_cap);
                R.evaluator_calls += 1 + r.evaluated;
                R.scope.add(r);
                if (r.status != OperatorStatus::Ok || !r.best) { ++T.no_candidate; continue; }
                if (r.best->objective < f - cfg.tau_imp) {
                    ++T.improving; ++T.accepted; ++R.accepted_moves;
                    if (applied) applied->push_back(r.best->move);
                    if (applied_before) applied_before->push_back(cur);
                    cur = std::move(r.best->solution);
                    f = r.best->objective;
                    improved = true;
                    break;
                }
            }
            if (improved) break; // restart the list (VNS k = 0)
        }
    }
    return {OptStop::LocalOptimum};
}

// ---- the certificate sweep ------------------------------------------------------------------------------------------------------------------
struct SweepFound { bool improving = false; tdr::model::Solution plan; double objective = 0.0; tdr::search::Move move; };

// Sweeps `plan` (M4: exactly the plan that will be returned) over the whole editable tail. Never uses an Rng overload, never shakes.
inline SweepFound certificate_sweep(const tdr::model::Solution& plan, double f_plan, const tdr::search::SearchContext& ctx_in, const ScopeMask& tail, std::size_t cut_skel,
                                    const OptConfig& cfg, SweepManifest& M, const tdr::model::Solution* returned_plan) {
    SweepFound out;
    M = SweepManifest{};
    M.ran = true; M.f_plan = f_plan;
    const double tau = cfg.faults.sweep_tolerance_mismatch ? cfg.tau_imp + 1.0 : cfg.tau_imp;   // the fault makes the sweep disagree with the search
    M.tau_imp = tau; M.tau_search = cfg.tau_imp;
    tdr::search::SearchContext ctx = ctx_in;
    ctx.scope = &tail;
    const auto& nodes = ctx.nodes();
    static const OperatorId order[8] = {OperatorId::N1, OperatorId::N2, OperatorId::N3, OperatorId::N4, OperatorId::N5, OperatorId::N6, OperatorId::N7, OperatorId::N8};
    const tdr::model::Solution& swept = plan;
    for (int oi = 0; oi < 8; ++oi) {
        const OperatorId op = order[oi];
        SweepOpEntry& E = M.ops[static_cast<std::size_t>(oi)];
        E.op = op;
        E.domain = detail_opt::independent_domain(op, swept, nodes, cut_skel);
        if (cfg.faults.drop_operator == oi) continue;
        auto anchors = detail_opt::anchors_of(op, swept, tail, nodes);
        if (cfg.faults.skip_last_anchor_of == oi && !anchors.empty()) anchors.pop_back();
        bool first = true;
        for (std::size_t a : anchors) {
            if (cfg.sweep_call_budget > 0 && M.operator_calls >= cfg.sweep_call_budget) { M.failure = "sweep call budget exhausted before the sweep was complete"; M.failed = "SWEEP-BUDGET"; M.complete = false; return out; }
            ++M.operator_calls;
            ++E.anchors_swept;   // counted when ATTEMPTED: an exception is then M2's to catch, not M1's
            try {
                if (cfg.faults.throw_in_anchor_of == oi && first) throw std::runtime_error("injected fault");
                first = false;
                OperatorResult r = detail_opt::apply(op, swept, ctx, a, cfg.sweep_n4_cap);
                M.evaluator_calls += 1 + r.evaluated;
                if (r.status != OperatorStatus::Ok) ++E.non_ok;
                E.generated += r.generated; E.evaluated += r.evaluated; E.accepted += r.accepted; E.rejected += r.rejected_total(); E.scope_rejected += r.scope_rejected;
                E.cap_reached += r.cap_reached_candidates;
                if (r.accepted + r.rejected_total() + r.scope_rejected != r.generated) ++E.accounting_violations;
                M.out_of_scope_skipped += r.scope_rejected;
                if (r.best && r.best->objective < f_plan - tau && (!out.improving || r.best->objective < out.objective)) {
                    out.improving = true; out.plan = r.best->solution; out.objective = r.best->objective; out.move = r.best->move;
                }
            } catch (...) { ++E.exceptions; first = false; }
        }
        if (cfg.faults.force_cap_counter_of == oi) ++E.cap_reached;
    }
    // M1..M4
    for (const auto& E : M.ops) if (E.anchors_swept != E.domain) { M.failure = "M1: anchors swept != anchor domain for an operator"; M.failed = "M1"; break; }
    M.m1 = M.failure.empty();
    bool m2 = true; for (const auto& E : M.ops) if (E.exceptions || E.non_ok || E.accounting_violations) m2 = false;
    M.m2 = m2; if (!m2 && M.failure.empty()) { M.failure = "M2: an anchor raised, returned a non-ok status, or broke accepted+rejected(+scope)=generated"; M.failed = "M2"; }
    bool m3 = true; for (const auto& E : M.ops) if (E.cap_reached) m3 = false;
    M.m3 = m3; if (!m3 && M.failure.empty()) { M.failure = "M3: a budget/cap counter is positive (N4 re-anchoring cap or limit)"; M.failed = "M3"; }
    M.m4 = returned_plan == nullptr || same_solution(swept, *returned_plan);
    if (!M.m4 && M.failure.empty()) { M.failure = "M4: the sweep did not run on the returned plan"; M.failed = "M4"; }
    M.m5 = std::isfinite(tau) && tau >= 0.0 && tau == M.tau_search;   // M5: one declared, logged tolerance, identical in search and sweep
    if (!M.m5 && M.failure.empty()) { M.failure = "M5: the improvement tolerance of the sweep differs from the search's (or is not a finite number >= 0)"; M.failed = "M5"; }
    M.complete = M.m1 && M.m2 && M.m3 && M.m4 && M.m5;
    M.improving = out.improving;
    M.best_move_objective = out.improving ? out.objective : f_plan;
    return out;
}

// ---- one optimisation ---------------------------------------------------------------------------------------------------------------------
// `cand` = S+ bound to its tentative world; `base` = the plan before the insertion (RIC core). Pure function of its inputs: deterministic.
namespace detail_opt {
inline void optimise_impl(const OptConfig& cfg, const InstalledPlan& base, const InstalledPlan& cand, const Cut& cut, NodeId request, OptResult& R) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    if (!(cfg.tau_imp >= 0.0) || !std::isfinite(cfg.tau_imp)) throw std::invalid_argument("tau_imp must be a finite number >= 0");
    R.arm = cfg.arm;
    R.plan = cand.solution();
    R.f_before = R.f_after = cand.objective();
    const auto& nodes = cand.world().nodes();
    auto finish_time = [&] { R.seconds = std::chrono::duration<double>(Clock::now() - t0).count(); };
    if (cfg.arm == OptArm::R0) { R.stop = OptStop::Identity; R.region_valid = true; finish_time(); return; }

    const PlanLayout L = layout_of(cand.solution(), nodes);
    RicInputs ric{&base, &cand.world()};
    Region region;
    throw_fault(cfg.faults.throw_in_region);
    if (cfg.arm == OptArm::FT || cfg.arm == OptArm::B4) region = RegionR5().build(RegionContext{L, nodes, cut.skel_index, request});
    else region = RegionR3::ric(cfg.w).build(RegionContext{L, nodes, cut.skel_index, request, &ric});
    R.region_valid = region.valid;
    if (!region.valid) { R.stop = OptStop::NoRegion; R.note = region.why; finish_time(); return; }
    R.region_full_tail = region.lo == cut.skel_index && region.hi == L.last();
    ScopeMask mask = ScopeMask::from_region(region, cand.solution(), nodes);
    R.free_customers = mask.free_customers().size();
    std::vector<OperatorId> list = cfg.ops.empty() ? tdr::search::default_neighbourhood_list() : cfg.ops;
    if (cfg.arm == OptArm::B2) list = {OperatorId::N4};
    // Budget (operator calls). B4 is the full-tail comparator of RIC at the SAME pass budget (DM0 §12.2): its budget is DERIVED here from the corresponding RIC region
    // of this very decision (same builder, same w), not from its own larger tail and not from a caller-supplied number. An explicit calls_override still wins (tests).
    std::size_t budget = 0;
    if (cfg.calls_override > 0) { budget = cfg.calls_override; R.budget_source = "override"; }
    else if (cfg.arm == OptArm::B4) {
        const Region rr = RegionR3::ric(cfg.w).build(RegionContext{L, nodes, cut.skel_index, request, &ric});
        if (!rr.valid) { R.region_valid = false; R.stop = OptStop::NoRegion; R.note = "B4: the matching RIC region is invalid (" + rr.why + "); no budget can be derived"; finish_time(); return; }
        R.ric_free_customers = ScopeMask::from_region(rr, cand.solution(), nodes).free_customers().size();
        budget = budget_for(cfg, R.ric_free_customers);
        R.budget_source = "ric-region";
    } else { budget = budget_for(cfg, R.free_customers); R.budget_source = "own"; }
    R.budget_calls = budget;
    const tdr::search::SearchContext ctx = cand.world().context();

    tdr::model::Solution cur = cand.solution();
    double f = cand.objective();
    std::size_t calls_left = budget;
    OptStop stop = scoped_search(cur, f, ctx, mask, cfg, list, calls_left, R).stop;

    const bool certify = cfg.arm == OptArm::RIC_C;
    const NodeId bound_lo = L.sk.node[region.lo], bound_hi = L.sk.node[region.hi];
    if (certify) {
        // RIC-C certificate loop. Region boundaries are tracked by node identity (pinned nodes never leave the route).
        NodeId blo = bound_lo, bhi = bound_hi;
        for (;;) {
            const Region tail_r = detail_opt::full_tail_region(cur, nodes, cut.skel_index);
            const ScopeMask tail = ScopeMask::from_region(tail_r, cur, nodes);
            const tdr::model::Solution swept_copy = cur;   // M4: the sweep runs on a snapshot of exactly the plan about to be returned
            const tdr::model::Solution& returned = cfg.faults.sweep_other_plan ? cand.solution() : cur; // (v) fault: the plan that is returned is not the swept one
            SweepFound sf = certificate_sweep(swept_copy, f, ctx, tail, cut.skel_index, cfg, R.manifest, &returned);
            ++R.sweeps; R.sweep_operator_calls += R.manifest.operator_calls; R.sweep_evaluator_calls += R.manifest.evaluator_calls;
            if (!R.manifest.complete) { R.cert = CertStatus::BudgetStop; break; }
            if (!sf.improving) {
                if (stop == OptStop::Budget) { // the regional search was cut off by its budget before the sweep: no certificate (DM0 §8.3: exhaustion of the budget gives budget-stop)
                    R.cert = CertStatus::BudgetStop; R.manifest.failed = "BUDGET";
                    R.manifest.failure = "BUDGET: the search budget was exhausted before the sweep (the regional search did not converge); no local-optimality statement is made";
                } else R.cert = CertStatus::Certified;
                break;
            }
            if (calls_left == 0 || R.expansions >= cfg.max_expansions) {
                R.cert = CertStatus::BudgetStop; stop = OptStop::Budget;
                R.manifest.failed = calls_left == 0 ? "BUDGET" : "EXPANSIONS";
                R.manifest.failure = calls_left == 0 ? "BUDGET: an improving single move exists and the search budget is exhausted" : "EXPANSIONS: an improving single move exists and the expansion limit is reached";
                break;
            }
            // improving single move found: adopt it, hull its footprint into the region, re-close, continue the scoped search
            const tdr::model::Solution before = cur;
            cur = std::move(sf.plan); f = sf.objective;
            ++R.expansions; ++R.accepted_moves;
            throw_fault(cfg.faults.throw_in_expansion);
            const PlanLayout CL = layout_of(cur, nodes);
            if (!CL.ok) { R.cert = CertStatus::BudgetStop; R.note = "layout after expansion: " + CL.why; break; }
            const auto slo = CL.skel_of_node(blo), shi = CL.skel_of_node(bhi);
            if (!slo || !shi) { R.cert = CertStatus::BudgetStop; R.note = "region boundary node left the route"; break; }
            std::size_t lo = *slo, hi = *shi;
            detail_opt::footprint_skeletons(sf.move, before, cur, nodes, lo, hi);
            lo = std::max(lo, cut.skel_index);
            const Region grown = detail_region::finish(CL, nodes, RegionKind::Ric, cut.skel_index, lo, hi, lo, hi);
            if (!grown.valid) { R.cert = CertStatus::BudgetStop; R.note = "grown region invalid"; break; }
            blo = CL.sk.node[grown.lo]; bhi = CL.sk.node[grown.hi];
            mask = ScopeMask::from_region(grown, cur, nodes);
            stop = scoped_search(cur, f, ctx, mask, cfg, list, calls_left, R).stop;
        }
    } else if (cfg.audit_sweep) {
        const Region tail_r = detail_opt::full_tail_region(cur, nodes, cut.skel_index);
        const ScopeMask tail = ScopeMask::from_region(tail_r, cur, nodes);
        certificate_sweep(cur, f, ctx, tail, cut.skel_index, cfg, R.manifest, &cur);
        ++R.sweeps; R.sweep_operator_calls += R.manifest.operator_calls; R.sweep_evaluator_calls += R.manifest.evaluator_calls;
    }

    // guarantee: evaluator feasibility and F(S*) <= F(S+); anything else falls back to S+
    const auto e = cand.world().evaluate(cur);
    ++R.evaluator_calls;
    if (!e.feasible || e.objective_value > cand.objective() + 1e-9) { R.note += "optimised plan rejected by the final check; S+ returned"; cur = cand.solution(); R.cert = certify ? CertStatus::BudgetStop : CertStatus::NotRun; }
    else f = e.objective_value;
    R.plan = cur; R.f_after = f; R.stop = stop;
    // protected-route diagnostics: commitment vs the whole tail, region preservation vs the arm's initial region (RIC-C may grow: whole tail)
    const Region tail0 = detail_opt::full_tail_region(cand.solution(), nodes, cut.skel_index);
    if (tail0.valid) R.commitment_violations = check_region_preserved(cand.solution(), R.plan, tail0, nodes).size();
    R.protected_changes = (certify || !region.valid) ? R.commitment_violations : check_region_preserved(cand.solution(), R.plan, region, nodes).size();
    finish_time();
}
} // namespace detail_opt

// Never throws. An exception (std or not) anywhere in region building, search, sweep driver or expansion is caught here: the input plan S+ is returned and the failure is
// recorded; for RIC-C the arm is kept and the certificate status is budget-stop (diagnostic in manifest.failed/failure and failure_reason). Counters collected so far are kept.
inline OptResult optimise_plan(const OptConfig& cfg, const InstalledPlan& base, const InstalledPlan& cand, const Cut& cut, NodeId request) {
    OptResult R;
    R.arm = cfg.arm;
    R.f_before = R.f_after = cand.objective();
    auto fail = [&](const std::string& why) {
        R.failed = true; R.failure_reason = why; R.note += (R.note.empty() ? "" : "; ") + why;
        R.plan = cand.solution(); R.f_after = R.f_before; R.stop = OptStop::Error;
        R.protected_changes = R.commitment_violations = 0;
        R.cert = cfg.arm == OptArm::RIC_C ? CertStatus::BudgetStop : CertStatus::NotRun;
        if (R.cert == CertStatus::BudgetStop) { R.manifest.failed = "EXCEPTION"; R.manifest.failure = why; }
    };
    try { detail_opt::optimise_impl(cfg, base, cand, cut, request, R); }
    catch (const std::exception& e) { fail(std::string("exception: ") + e.what()); }
    catch (...) { fail("non-standard exception"); }
    return R;
}

// ---- admission hook ------------------------------------------------------------------------------------------------------------------------
// Plugs into AdmissionPolicy::decide(in, optimiser) (A0 and A-PM alike: they share the pipeline and differ only in Phi). Never throws into the simulator;
// any internal failure returns S+, so a Phi-feasible B1 insertion is never sacrificed by an optimiser failure.
struct RegionalOptimizer final : Optimizer {
    OptConfig cfg;
    mutable std::vector<OptResult> records; // one per optimise() call, in call order (OptResult::request = request index)
    explicit RegionalOptimizer(OptConfig c = {}) : cfg(std::move(c)) {}
    std::optional<tdr::model::Solution> optimise(const AdmissionInput& in, const tdr::model::Solution& s_plus) const override {
        OptResult R;
        try {
            auto cand = InstalledPlan::make(s_plus, in.tentative, in.theta, in.plan.version() + 1);
            if (!cand) { R.note = "S+ not evaluator-feasible"; R.plan = s_plus; R.arm = cfg.arm; records.push_back(R); return s_plus; }
            R = optimise_plan(cfg, in.plan, *cand, in.cut, in.cid);   // does not throw itself; the guard below covers InstalledPlan::make and the bookkeeping
        } catch (...) { R = OptResult{}; R.arm = cfg.arm; R.failed = true; R.failure_reason = "exception outside optimise_plan"; R.plan = s_plus; R.stop = OptStop::Error;
                        R.cert = cfg.arm == OptArm::RIC_C ? CertStatus::BudgetStop : CertStatus::NotRun; }
        R.request = in.request.index;
        records.push_back(R);
        return R.plan;
    }
};

} // namespace tdr::dynamic
