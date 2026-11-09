// DM9 regional local optimiser, arms/baselines and the RIC-C certificate: tests (T13 + DM9 acceptance list).
//  (A) R0 identity; (B) every arm: evaluator-feasible, F(S*) <= F(S+), same served set, prefix/commitment preserved (independent DM7 checker), region preserved,
//  (C) determinism, (D) accounting: evaluator calls observed by a spy == the optimiser's own counters, (E) certificate: certified => NO improving eligible single move
//  by an independent brute-force scan; independent anchor-domain counts; manifest identities, (F) fault injection (T13 i-vi) never certifies, (G) N4-cap and budget
//  downgrade to budget-stop, (H) simulator integration: A0/A-PM x scan x optimiser independent, U1 chain, [P15] per decision, I1-I7, deterministic replay,
//  (I) source guard: no shake / run_vns / Rng overload in the optimiser.
// The evaluator spy renames the evaluator before the first include (as in test_dyn_scope.cpp).
#define evaluate_solution evaluate_solution_real
#include "timing/timing_engine.hpp"
#undef evaluate_solution
namespace tdr::timing {
inline long g_eval_calls = 0;
inline SolutionEvaluationResult evaluate_solution(const tdr::model::Solution& s, const tdr::graph::SpatialNetwork& n,
                                                  const std::unordered_map<tdr::model::NodeId, tdr::model::CustomerData>& c, const FlpDataMap& f) {
    ++g_eval_calls;
    return evaluate_solution_real(s, n, c, f);
}
} // namespace tdr::timing

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>
#include <stdexcept>
#include <tuple>
#include <sstream>
#include <string>
#include <vector>

#include "dynamic/base_plan.hpp"
#include "dynamic/regional_optimiser.hpp"
#include "dynamic/runner.hpp"
#include "dynamic/simulator.hpp"

using namespace tdr;
using namespace tdr::dynamic;
using tdr::model::NodeId;
using tdr::model::Solution;

static long g_checks = 0, g_fail = 0, g_neg_control = 0;
#define CHECK(cond) do { ++g_checks; if (!(cond)) { ++g_fail; std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECKMSG(cond, msg) do { ++g_checks; if (!(cond)) { ++g_fail; std::printf("FAIL %s:%d  %s  [%s]\n", __FILE__, __LINE__, #cond, std::string(msg).c_str()); } } while (0)

namespace {

struct Case {
    std::string label;
    std::shared_ptr<InstalledPlan> base_plan;   // the plan before the insertion (its world does not contain the request)
    std::shared_ptr<const World> tw;
    std::optional<InstalledPlan> cand;
    Cut cut;
    NodeId cid = 0;
    Request rq;
    const InstalledPlan& base() const { return *base_plan; }
};

// Cases are consecutive cheapest insertions WITHOUT optimisation into a (mostly constructive) base plan, so the later cases start from a plan that the
// optimiser has not touched and has real room to improve.
std::vector<Case> make_cases() {
    std::vector<Case> out;
    const char* env = std::getenv("DM9_SEEDS");
    const char* env0 = std::getenv("DM9_SEED0");   // chunked full runs: DM9_SEED0=10 DM9_SEEDS=10 ...
    const int n_seeds = env ? std::atoi(env) : 10, seed0 = env0 ? std::atoi(env0) : 0;   // default 10 seeds (~40 cases); the full sweep is 30 seeds
    for (int seed = seed0; seed < seed0 + n_seeds; ++seed) {
        DynamicConfig c;
        c.regime = 'F'; apply_regime(c);
        c.base.base.instance.num_customers = 10 + seed % 9;
        c.base.base.instance.dataset_index = seed;
        c.base.base.vehicle.num_drones = 2 + seed % 2;
        c.base.vns.iter_max = 100;
        c.base_kind = (seed % 3 == 0) ? BaseKind::StaticVns : BaseKind::Constructive;
        const BasePlan bp = build_base_plan(c);
        if (!bp.ok) { std::printf("fixture %d not built: %s\n", seed, bp.error.c_str()); continue; }
        std::mt19937 g(static_cast<unsigned>(1234 + seed));
        auto cur = std::make_shared<InstalledPlan>(*bp.plan);
        for (int k = 0; k < 4; ++k) {
            Case cs;
            cs.label = "seed" + std::to_string(seed) + "-r" + std::to_string(k);
            cs.base_plan = cur;
            const double L = c.base.base.instance.area_side_m;
            cs.rq.index = static_cast<std::uint32_t>(k + 1);
            cs.rq.time = 0.0;
            cs.rq.location = {std::uniform_real_distribution<double>(0, L)(g), std::uniform_real_distribution<double>(0, L)(g)};
            cs.rq.se1 = c.base.base.instance.truck_service_time_s; cs.rq.se2 = c.base.base.instance.drone_service_time_s; cs.rq.theta = c.base.base.instance.theta;
            cs.cid = customer_id(cs.rq.index);
            const double theta = std::uniform_real_distribution<double>(0.0, 0.4 * makespan_of(*cur))(g);
            const auto cut = clean_cut(*cur, theta);
            if (!cut) break;
            cs.cut = *cut;
            cs.tw = cur->world_ptr()->with_request(cs.cid, cs.rq);
            if (!cs.tw) break;
            const InsertionResult ins = cheapest_truck_insertion(*cur, cs.cut, cs.cid, cs.tw);
            if (!ins.best) continue;
            cs.cand = InstalledPlan::make(ins.solution, cs.tw, theta, cur->version() + 1);
            if (!cs.cand) continue;
            cur = std::make_shared<InstalledPlan>(*cs.cand);
            out.push_back(std::move(cs));
        }
    }
    return out;
}

std::vector<NodeId> customers_only(const Solution& s, const tdr::model::NodeRegistry& nodes) {
    std::vector<NodeId> v;
    for (NodeId x : s.truck_route) if (nodes.at(x).type == tdr::model::NodeType::Customer) v.push_back(x);
    for (const auto& t : s.drone_trips) v.push_back(t.service_customer);
    std::sort(v.begin(), v.end());
    return v;
}

OptResult run(const Case& c, OptConfig cfg) { return optimise_plan(cfg, c.base(), *c.cand, c.cut, c.cid); }

bool same_result(const OptResult& a, const OptResult& b) {
    return same_solution(a.plan, b.plan) && a.f_after == b.f_after && a.search_calls == b.search_calls && a.accepted_moves == b.accepted_moves && a.evaluator_calls == b.evaluator_calls &&
           a.cert == b.cert && a.expansions == b.expansions && a.sweeps == b.sweeps && a.sweep_operator_calls == b.sweep_operator_calls && a.stop == b.stop &&
           a.scope.scope_rejected == b.scope.scope_rejected && a.scope.evaluator_calls_avoided == b.scope.evaluator_calls_avoided && a.protected_changes == b.protected_changes;
}

// ---- independent anchor domains (route positions only; no layout_of, no mask) ----
std::size_t indep_domain(int op_index /*0=N1..7=N8*/, const Solution& s, const tdr::model::NodeRegistry& nodes, std::size_t cut_route_pos) {
    const auto& r = s.truck_route;
    std::size_t n = 0;
    if (op_index == 3 || op_index == 6) { // N4, N7: truck customers strictly behind the cut (the end depot is not a customer)
        for (std::size_t p = cut_route_pos + 1; p < r.size(); ++p) if (nodes.at(r[p]).type == tdr::model::NodeType::Customer) ++n;
    } else {
        for (const auto& t : s.drone_trips) {
            std::size_t lp = r.size();
            for (std::size_t p = 0; p < r.size(); ++p) if (r[p] == t.launch_node) { lp = p; break; }
            if (lp >= cut_route_pos) ++n;
        }
    }
    return n;
}

// ---- brute force: every evaluator-feasible single-move candidate that keeps the tail contract (independent DM7 checker) and improves F ----
std::vector<tdr::search::CandidateRecord> logged(int op, const Solution& s, const search::SearchContext& ctx, std::size_t anchor) {
    using namespace tdr::search;
    auto rec = [](auto o) { o.record_candidates = true; return o; };
    switch (op) {
        case 0: return n1_launch_node_relocate(s, ctx, anchor, rec(N1Options{})).log;
        case 1: return n2_recover_node_relocate(s, ctx, anchor, rec(N2Options{})).log;
        case 2: return n3_drone_service_node_relocate(s, ctx, anchor, rec(N3Options{})).log;
        case 3: { N4Options o; o.record_candidates = true; o.max_reanchor_variants = 0; return n4_truck_service_node_relocate(s, ctx, static_cast<NodeId>(anchor), o).log; }
        case 4: return n5_td_service_node_swap(s, ctx, anchor, rec(N5Options{})).log;
        case 5: return n6_drone_drone_swap(s, ctx, anchor, rec(N6Options{})).log;
        case 6: return n7_truck_node_delete_drone_insert(s, ctx, static_cast<NodeId>(anchor), rec(N7Options{})).log;
        default: return n8_drone_node_delete_truck_insert(s, ctx, anchor, rec(N8Options{})).log;
    }
}

struct Brute { std::size_t improving = 0, scanned = 0; };
Brute brute_improving(const Solution& plan, double f, const Case& c, double tau) {
    Brute B;
    const auto& nodes = c.tw->nodes();
    const search::SearchContext ctx = c.tw->context(); // UNSCOPED
    const Region tail = detail_opt::full_tail_region(plan, nodes, c.cut.skel_index);
    const std::size_t cut_pos = c.cut.route_pos;
    for (int op = 0; op < 8; ++op) {
        std::vector<std::size_t> anchors;
        if (op == 3 || op == 6) { for (std::size_t p = cut_pos + 1; p < plan.truck_route.size(); ++p) if (nodes.at(plan.truck_route[p]).type == tdr::model::NodeType::Customer) anchors.push_back(plan.truck_route[p]); }
        else for (std::size_t t = 0; t < plan.drone_trips.size(); ++t) {
            std::size_t lp = plan.truck_route.size();
            for (std::size_t p = 0; p < plan.truck_route.size(); ++p) if (plan.truck_route[p] == plan.drone_trips[t].launch_node) { lp = p; break; }
            if (lp >= cut_pos) anchors.push_back(t);
        }
        for (std::size_t a : anchors) {
            ++B.scanned;
            for (const auto& rec : logged(op, plan, ctx, a)) {
                if (rec.reason != search::RejectReason::None || !rec.evaluated) continue;
                if (!(rec.objective < f - tau)) continue;
                if (!check_region_preserved(plan, rec.solution, tail, nodes).empty()) continue; // outside the tail contract
                ++B.improving;
            }
        }
    }
    return B;
}

OptConfig cfg_of(OptArm a) { OptConfig c; c.arm = a; return c; }

// ============================================================================================================================================
void test_identity_and_invariants(const std::vector<Case>& cases) {
    long improved[6] = {0, 0, 0, 0, 0, 0}, n = 0;
    double gain[6] = {0, 0, 0, 0, 0, 0};
    for (const auto& c : cases) {
        const auto& nodes = c.tw->nodes();
        ++n;
        // (A) R0: identity, zero work
        const OptResult r0 = run(c, cfg_of(OptArm::R0));
        CHECKMSG(same_solution(r0.plan, c.cand->solution()) && r0.search_calls == 0 && r0.evaluator_calls == 0 && r0.accepted_moves == 0 && r0.f_after == r0.f_before, c.label);
        // (B) every arm
        for (OptArm arm : {OptArm::FR, OptArm::RIC_C, OptArm::FT, OptArm::B2, OptArm::B4}) {
            OptConfig cfg = cfg_of(arm);
            const OptResult r = run(c, cfg);
            const std::string m = c.label + " " + opt_arm_name(arm);
            CHECKMSG(r.region_valid, m);
            if (arm == OptArm::B4) { // B4's budget is DERIVED from the matching RIC region inside optimise_plan (no calls_override): equal to FR's and RIC-C's
                const OptResult fr0 = run(c, cfg_of(OptArm::FR)), rc0 = run(c, cfg_of(OptArm::RIC_C));
                CHECKMSG(r.budget_source == "ric-region" && r.budget_calls == fr0.budget_calls && r.budget_calls == rc0.budget_calls && r.ric_free_customers == fr0.free_customers, m);
            }
            const auto e = c.tw->evaluate(r.plan);
            CHECKMSG(e.feasible, m);                                                  // the unchanged evaluator is the authority
            CHECKMSG(r.f_after <= r.f_before + 1e-9, m);
            CHECKMSG(std::fabs(e.objective_value - r.f_after) <= 1e-9 * std::max(1.0, std::fabs(r.f_after)), m);
            CHECKMSG(customers_only(r.plan, nodes) == customers_only(c.cand->solution(), nodes), m);   // same served set
            CHECKMSG(r.commitment_violations == 0 && r.protected_changes == 0, m);
            CHECKMSG(r.search_calls <= r.budget_calls, m);
            // independent DM7 checks: the committed prefix is the original one and the tail contract holds
            const Region tail = detail_opt::full_tail_region(c.cand->solution(), nodes, c.cut.skel_index);
            CHECKMSG(tail.valid && check_region_preserved(c.cand->solution(), r.plan, tail, nodes).empty(), m);
            for (std::size_t p = 0; p <= c.cut.route_pos; ++p) CHECKMSG(r.plan.truck_route[p] == c.cand->solution().truck_route[p], m);
            if (arm == OptArm::FR || arm == OptArm::B2) { // fixed region arms: the RIC region is preserved (DM7 checker, built independently here)
                const PlanLayout Lc = layout_of(c.cand->solution(), nodes);
                RicInputs ric{&c.base(), c.tw.get()};
                const Region R = RegionR3::ric(3).build(RegionContext{Lc, nodes, c.cut.skel_index, c.cid, &ric});
                CHECKMSG(R.valid && check_region_preserved(c.cand->solution(), r.plan, R, nodes).empty(), m);
            }
            { // the free-customer count against an independent tail count (truck customers behind the cut + drone customers launched at/after it)
                const std::size_t tail_free = indep_domain(3, c.cand->solution(), nodes, c.cut.route_pos) + indep_domain(0, c.cand->solution(), nodes, c.cut.route_pos);
                if (arm == OptArm::FT || arm == OptArm::B4) CHECKMSG(r.free_customers == tail_free, m + " full tail");
                else CHECKMSG(r.free_customers <= tail_free, m);
            }
            if (arm == OptArm::FT) { // negative control for the preserved-region checker: an FT result that changed anything outside the RIC region must be flagged
                const PlanLayout Lc = layout_of(c.cand->solution(), nodes);
                RicInputs ric{&c.base(), c.tw.get()};
                const Region R = RegionR3::ric(3).build(RegionContext{Lc, nodes, c.cut.skel_index, c.cid, &ric});
                if (R.valid && !check_region_preserved(c.cand->solution(), r.plan, R, nodes).empty()) ++g_neg_control;
            }
            if (arm == OptArm::B2) { for (std::size_t i = 0; i < 8; ++i) if (i != static_cast<std::size_t>(OperatorId::N4)) CHECKMSG(r.by_op[i].calls == 0, m); }
            if (r.f_after < r.f_before - 1e-9) { ++improved[static_cast<int>(arm)]; gain[static_cast<int>(arm)] += r.f_before - r.f_after; }
            if (arm == OptArm::RIC_C) CHECKMSG(r.cert == CertStatus::Certified || r.cert == CertStatus::BudgetStop, m);
            else CHECKMSG(r.cert == CertStatus::NotRun, m);
        }
        // RIC-C starts from exactly the FR search (same budget) and only ever improves on it
        const OptResult fr = run(c, cfg_of(OptArm::FR)), rc = run(c, cfg_of(OptArm::RIC_C));
        CHECKMSG(rc.f_after <= fr.f_after + 1e-9, c.label);
        // audit sweep never changes the plan
        OptConfig au = cfg_of(OptArm::FR); au.audit_sweep = true;
        const OptResult fa = run(c, au);
        CHECKMSG(same_solution(fa.plan, fr.plan) && fa.cert == CertStatus::NotRun && fa.manifest.ran, c.label);
    }
    const char* nm[6] = {"R0", "FR", "RIC-C", "FT", "B2", "B4"};
    for (int a : {1, 2, 3, 4, 5}) std::printf("   [info] %-5s improved F on %ld of %ld cases, total gain %.1f s\n", nm[a], improved[a], n, gain[a]);
    std::printf("   [info] negative control: %ld FT results change something outside the RIC region and are flagged by the DM7 checker\n", g_neg_control);
    CHECK(g_neg_control > 0);
    CHECK(improved[1] > 0 && improved[2] > 0 && improved[3] > 0);
    CHECK(improved[2] >= improved[1]);            // RIC-C (search + expansion) improves at least as many cases as FR
}

void test_determinism(const std::vector<Case>& cases) {
    for (const auto& c : cases) for (OptArm arm : {OptArm::FR, OptArm::RIC_C, OptArm::FT}) {
        const OptResult a = run(c, cfg_of(arm)), b = run(c, cfg_of(arm));
        CHECKMSG(same_result(a, b), c.label + " " + opt_arm_name(arm));
    }
}

void test_accounting_spy(const std::vector<Case>& cases) {
    std::size_t scope_events = 0;
    for (const auto& c : cases) for (OptArm arm : {OptArm::FR, OptArm::RIC_C, OptArm::FT, OptArm::B2}) {
        const long before = tdr::timing::g_eval_calls;
        const OptResult r = run(c, cfg_of(arm));
        const long delta = tdr::timing::g_eval_calls - before;
        CHECKMSG(delta == static_cast<long>(r.evaluator_calls + r.sweep_evaluator_calls), c.label + " " + opt_arm_name(arm)); // every evaluator call is counted, none is hidden
        scope_events += r.scope.scope_rejected;
    }
    std::printf("   [info] scope-rejected candidates over the arms: %zu (each caused zero evaluator calls: spy delta == counters)\n", scope_events);
    CHECK(scope_events > 0);
}

struct PartnerEvidence { std::size_t invariance_pairs = 0, n3_anchors = 0, n6_anchors = 0; };
// M1 accounting: the manifest counts ANCHORS; partners (N3, N5, N6, N4 exchange) are enumerated inside the operator. Evidence recorded here, and what it is NOT:
//  (a) for every operator and anchor of a certified plan, the scoped sweep's `generated` summed per operator equals the sum of an UNSCOPED call of the same operator/anchor/cap
//      (the DM8 identity): the scope hook never shortens the partner enumeration. This is a consistency check against the operator itself, not an independent count.
//  (b) INDEPENDENT counts, from the plan layout and the documented contracts: N3 generates exactly (has predecessor) + (has successor) in the anchor's drone order (launch position,
//      recovery position, index); N6 generates one candidate per OTHER trip whose half-step coordinate 2g+1 is within 2w of the anchor's. N4-exchange and N5 partner sets are
//      NOT independently counted.
void partner_evidence(const Case& c, const Solution& plan, const SweepManifest& M, PartnerEvidence& pe) {
    const auto& nodes = c.tw->nodes();
    const search::SearchContext ctx = c.tw->context();   // unscoped
    const PlanLayout L = layout_of(plan, nodes);
    std::size_t gen[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    for (int op = 0; op < 8; ++op) {
        std::vector<std::size_t> anchors;
        if (op == 3 || op == 6) { for (std::size_t p = c.cut.route_pos + 1; p < plan.truck_route.size(); ++p) if (nodes.at(plan.truck_route[p]).type == tdr::model::NodeType::Customer) anchors.push_back(plan.truck_route[p]); }
        else for (const auto& t : L.trips) if (t.launch_pos >= c.cut.route_pos) anchors.push_back(t.index);   // same eligibility as indep_domain: launched at/after the cut
        for (std::size_t a : anchors) {
            const search::OperatorResult r = detail_opt::apply(static_cast<OperatorId>(op), plan, ctx, a, 0);
            gen[op] += r.generated; ++pe.invariance_pairs;
            if (op == 2) { // N3
                const auto& me = L.trips[a];
                std::vector<const TripInfo*> seq;
                for (const auto& t : L.trips) if (plan.drone_trips[t.index].drone_id == plan.drone_trips[a].drone_id) seq.push_back(&t);
                std::sort(seq.begin(), seq.end(), [](const TripInfo* x, const TripInfo* y) { return std::tie(x->launch_pos, x->recovery_pos, x->index) < std::tie(y->launch_pos, y->recovery_pos, y->index); });
                std::size_t at = 0; for (std::size_t i = 0; i < seq.size(); ++i) if (seq[i]->index == me.index) at = i;
                const std::size_t expect = (at > 0 ? 1u : 0u) + (at + 1 < seq.size() ? 1u : 0u);
                CHECKMSG(r.generated == expect, c.label + " N3 partners");
                ++pe.n3_anchors;
            }
            if (op == 5) { // N6
                std::size_t expect = 0;
                const long hi = 2 * static_cast<long>(L.trips[a].launch_gap) + 1;
                for (const auto& t : L.trips) if (t.index != a && std::labs(2 * static_cast<long>(t.launch_gap) + 1 - hi) <= 2L * ctx.adjacency_window) ++expect;
                CHECKMSG(r.generated == expect, c.label + " N6 partners");
                ++pe.n6_anchors;
            }
        }
        CHECKMSG(M.ops[static_cast<std::size_t>(op)].generated == gen[op], c.label + " scope-invariance of generated, op" + std::to_string(op + 1));
    }
}

void test_certificate(const std::vector<Case>& cases) {
    std::size_t certified = 0, budget_stop = 0, brute_scanned = 0, with_expansion = 0, nonzero_domain = 0;
    PartnerEvidence pe; std::size_t dom_by_op[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    for (const auto& c : cases) {
        const auto& nodes = c.tw->nodes();
        const OptResult r = run(c, cfg_of(OptArm::RIC_C));
        const SweepManifest& M = r.manifest;
        CHECKMSG(M.ran && r.sweeps >= 1, c.label);
        if (r.expansions > 0) ++with_expansion;
        if (r.cert == CertStatus::Certified) {
            ++certified;
            CHECKMSG(M.complete && M.m1 && M.m2 && M.m3 && M.m4 && M.m5 && !M.improving && M.failed.empty() && r.stop != OptStop::Budget && M.tau_imp == M.tau_search, c.label);
            for (int i = 0; i < 8; ++i) {
                const auto& E = M.ops[static_cast<std::size_t>(i)];
                const std::size_t d = indep_domain(i, r.plan, nodes, c.cut.route_pos);   // computed here, not by the optimiser
                CHECKMSG(E.domain == d && E.anchors_swept == d, c.label + " op" + std::to_string(i + 1));
                CHECKMSG(E.accepted + E.rejected + E.scope_rejected == E.generated && E.exceptions == 0 && E.non_ok == 0 && E.cap_reached == 0, c.label);
                nonzero_domain += d; dom_by_op[i] += d;
            }
            // the certificate's claim, checked by an independent exhaustive scan of every eligible single move on the RETURNED plan
            const Brute B = brute_improving(r.plan, r.f_after, c, r.manifest.tau_imp);
            brute_scanned += B.scanned;
            CHECKMSG(B.improving == 0, c.label + " certified plan has an improving eligible single move");
            partner_evidence(c, r.plan, r.manifest, pe);
        } else {
            ++budget_stop;
            CHECKMSG(r.cert == CertStatus::BudgetStop && !r.manifest.failed.empty() && !r.manifest.failure.empty(), c.label + " budget-stop carries its failed condition");
        }
    }
    std::printf("   [info] certificate: certified %zu, budget-stop %zu, cases with footprint expansion %zu, anchors cross-checked by brute force %zu, nonzero domains %zu\n",
                certified, budget_stop, with_expansion, brute_scanned, nonzero_domain);
    CHECK(certified > 0);
    CHECK(brute_scanned > 0);
    std::printf("   [info] M1 partner accounting on certified plans: scope-invariance (sweep 'generated' == unscoped operator 'generated') checked on %zu operator-anchor pairs for all of N1-N8;\n"
                "          INDEPENDENT partner counts measured only for N3 (%zu anchors, predecessor/successor of the same drone) and N6 (%zu anchors, half-step window); N4-exchange, N5 and N1/N2/N7/N8 candidates are NOT independently counted\n",
                pe.invariance_pairs, pe.n3_anchors, pe.n6_anchors);
    std::printf("   [info] anchors swept by operator on certified plans: N1 %zu N2 %zu N3 %zu N4 %zu N5 %zu N6 %zu N7 %zu N8 %zu\n", dom_by_op[0], dom_by_op[1], dom_by_op[2], dom_by_op[3], dom_by_op[4], dom_by_op[5], dom_by_op[6], dom_by_op[7]);
    CHECK(pe.invariance_pairs > 0);
    if (cases.size() >= 30) CHECK(pe.n3_anchors > 0 && pe.n6_anchors > 0);   // drone-trip anchors are rare in these fixtures: a small DM9_SEEDS may have none
    // negative control for the brute force itself: the S+ of cases whose FT search found an improvement has improving eligible moves
    std::size_t neg = 0, neg_ok = 0;
    for (const auto& c : cases) {
        OptConfig z = cfg_of(OptArm::FT); z.calls_override = 1;
        const OptResult r = run(c, cfg_of(OptArm::FT));
        if (r.f_after < r.f_before - 1e-6) { ++neg; if (brute_improving(c.cand->solution(), c.cand->objective(), c, 1e-9).improving > 0) ++neg_ok; }
    }
    CHECK(neg > 0 && neg == neg_ok);
}

void test_faults(const std::vector<Case>& cases) {
    std::size_t caught[6] = {0, 0, 0, 0, 0, 0}, tried[6] = {0, 0, 0, 0, 0, 0}, silent_ok = 0, tried5m = 0, caught5m = 0;
    for (const auto& c : cases) {
        const OptResult base = run(c, cfg_of(OptArm::RIC_C));
        if (base.cert != CertStatus::Certified) continue;
        const auto& nodes = c.tw->nodes();
        for (int op = 0; op < 8; ++op) {
            const std::size_t d = indep_domain(op, base.plan, nodes, c.cut.route_pos);
            struct F { int kind; CertFaults f; } fs[4];
            fs[0].kind = 0; fs[0].f.drop_operator = op;
            fs[1].kind = 1; fs[1].f.skip_last_anchor_of = op;
            fs[2].kind = 2; fs[2].f.force_cap_counter_of = op;
            fs[3].kind = 3; fs[3].f.throw_in_anchor_of = op;
            for (auto& x : fs) {
                OptConfig cfg = cfg_of(OptArm::RIC_C); cfg.faults = x.f;
                const OptResult r = run(c, cfg);
                const bool detectable = x.kind == 2 ? true : d > 0;
                if (detectable) {
                    ++tried[x.kind]; if (r.cert == CertStatus::BudgetStop) ++caught[x.kind];
                    CHECKMSG(r.cert != CertStatus::Certified, c.label + " fault " + std::to_string(x.kind) + " op" + std::to_string(op + 1));
                    static const char* want[4] = {"M1", "M1", "M3", "M2"};   // each fault is caught by ITS condition (so each condition is individually necessary)
                    CHECKMSG(r.manifest.failure.rfind(want[x.kind], 0) == 0, c.label + " fault " + std::to_string(x.kind) + " reported '" + r.manifest.failure + "'");
                }
                else { if (r.cert == CertStatus::Certified) ++silent_ok; } // nothing to miss: a fault on an empty domain is not an incomplete sweep
            }
        }
        if (!same_solution(base.plan, c.cand->solution())) { // (v) sweeping a plan other than the returned one (only meaningful when the plan changed)
            OptConfig cfg = cfg_of(OptArm::RIC_C); cfg.faults.sweep_other_plan = true;
            const OptResult r = run(c, cfg);
            ++tried[4]; if (r.cert == CertStatus::BudgetStop) ++caught[4];
            CHECKMSG(r.cert != CertStatus::Certified && r.manifest.failure.rfind("M4", 0) == 0, c.label + " fault v");
        }
        { // (M5) the sweep's tolerance differs from the search's
            OptConfig cfg = cfg_of(OptArm::RIC_C); cfg.faults.sweep_tolerance_mismatch = true;
            const OptResult r = run(c, cfg);
            ++tried5m; if (r.cert == CertStatus::BudgetStop) ++caught5m;
            CHECKMSG(r.cert == CertStatus::BudgetStop && !r.manifest.m5 && r.manifest.failed == "M5" && r.manifest.failure.rfind("M5", 0) == 0, c.label + " fault M5");
        }
        { // (vi-b) a variant: the SWEEP's own call budget is exhausted before it is complete (the exact DM0 case (vi) is test_fault_vi_exact)
            OptConfig cfg = cfg_of(OptArm::RIC_C); cfg.sweep_call_budget = 1;
            const OptResult r = run(c, cfg);
            if (base.manifest.domain_total() > 1) { ++tried[5]; if (r.cert == CertStatus::BudgetStop) ++caught[5]; CHECKMSG(r.cert == CertStatus::BudgetStop && !r.manifest.complete, c.label + " fault vi"); }
        }
    }
    const char* nm[6] = {"(i) drop operator", "(ii) skip last anchor", "(iii) force cap counter", "(iv) exception in an anchor", "(v) sweep other plan", "(vi-b) sweep call budget"};
    for (int i = 0; i < 6; ++i) { std::printf("   [info] fault %-30s caught %zu of %zu\n", nm[i], caught[i], tried[i]); CHECK(tried[i] > 0 && caught[i] == tried[i]); }
    std::printf("   [info] fault %-30s caught %zu of %zu\n", "(M5) tolerance mismatch", caught5m, tried5m); CHECK(tried5m > 0 && caught5m == tried5m);
    std::printf("   [info] faults on empty anchor domains (nothing to miss): %zu stayed certified\n", silent_ok);
}

void test_budget_and_cap(const std::vector<Case>& cases) {
    std::size_t cap_downgrades = 0, cap_clean = 0, tiny_budget_stops = 0;
    for (const auto& c : cases) {
        // N4 cap in the sweep: cert with cap=1 is `certified` only if the sweep saw no cap-limited candidate; otherwise budget-stop and M3 is the named reason
        OptConfig cap = cfg_of(OptArm::RIC_C); cap.sweep_n4_cap = 1;
        const OptResult rc = run(c, cap);
        std::size_t capped = 0; for (const auto& E : rc.manifest.ops) capped += E.cap_reached;
        if (rc.cert == CertStatus::Certified) { CHECKMSG(capped == 0 && rc.manifest.m3, c.label); ++cap_clean; }
        else if (capped > 0) { CHECKMSG(rc.cert == CertStatus::BudgetStop && !rc.manifest.m3 && rc.manifest.failure.rfind("M3", 0) == 0, c.label); ++cap_downgrades; }
        // a tiny search budget: the plan is returned, F did not get worse, and a certificate is only given if the final complete sweep found nothing
        OptConfig tiny = cfg_of(OptArm::RIC_C); tiny.calls_override = 1;
        const OptResult rt = run(c, tiny);
        CHECKMSG(rt.f_after <= rt.f_before + 1e-9 && rt.search_calls <= 1, c.label);
        if (rt.cert == CertStatus::Certified) CHECKMSG(rt.manifest.complete && !rt.manifest.improving && brute_improving(rt.plan, rt.f_after, c, rt.manifest.tau_imp).improving == 0, c.label);
        else { ++tiny_budget_stops; CHECKMSG(rt.cert == CertStatus::BudgetStop, c.label); }
        // the search budget is spent (1 call): an improving move found by the sweep is reported, never applied, and the status is budget-stop
        if (rt.search_calls == 1) CHECKMSG(rt.expansions == 0, c.label);                  // no budget left: no expansion, whatever the sweep finds
        if (rt.search_calls == 1 && rt.manifest.improving) CHECKMSG(rt.cert == CertStatus::BudgetStop && rt.stop == OptStop::Budget, c.label);
        // zero budget and no expansions allowed: any improving move found by the sweep is NOT applied; status is budget-stop, never certified
        OptConfig zero = cfg_of(OptArm::RIC_C); zero.calls_override = 1; zero.max_expansions = 0;
        const OptResult rz = run(c, zero);
        if (rz.manifest.improving) CHECKMSG(rz.cert == CertStatus::BudgetStop && rz.expansions == 0, c.label);
    }
    std::printf("   [info] N4 cap=1 in the sweep: %zu downgraded to budget-stop, %zu still certified; tiny search budget: %zu budget-stops\n", cap_downgrades, cap_clean, tiny_budget_stops);
    CHECK(cap_downgrades > 0);
}

// ---- simulator integration -------------------------------------------------------------------------------------------------------------
struct P15Checker final : Optimizer { // asserts [P15] on the SAME decision input: A-PM accepts with the optimiser whenever it accepts with the identity optimiser
    const RegionalOptimizer* inner; mutable long decisions = 0, primary_failed = 0, fallback = 0;
    explicit P15Checker(const RegionalOptimizer* i) : inner(i) {}
    std::optional<Solution> optimise(const AdmissionInput& in, const Solution& s_plus) const override {
        AdmitAPM apm; const Optimizer id;
        const Decision a = apm.decide(in, id), b = apm.decide(in, *inner);
        ++decisions;
        CHECK(!(a.outcome == Outcome::Accepted) || b.outcome == Outcome::Accepted);
        if (b.outcome == Outcome::Accepted && b.stage == Stage::Fallback) ++fallback;
        if (b.outcome != Outcome::Accepted || b.stage == Stage::Fallback) ++primary_failed;
        return inner->optimise(in, s_plus);
    }
};

std::string sig(const SimResult& r) {
    std::ostringstream o; o.precision(17);
    o << r.ok << ' ' << r.final_objective;
    for (const auto& x : r.records) o << ' ' << static_cast<int>(x.outcome) << ':' << x.plan_version << ':' << x.delta_objective << ':' << static_cast<int>(x.stage);
    return o.str();
}

void test_simulation() {
    long runs = 0, decisions = 0, fallbacks = 0, rejected_promise = 0, optimised_installs = 0, better_than_r0 = 0;
    for (int seed = 0; seed < 6; ++seed) {
        DynamicConfig c;
        c.regime = 'F'; apply_regime(c);
        c.base.base.instance.num_customers = 12 + seed % 4;
        c.base.base.instance.dataset_index = 100 + seed;
        c.base.base.vehicle.num_drones = 2 + seed % 2;
        c.base.vns.iter_max = 100;
        c.rho = 0.5; c.omega = 10.0; c.m_max = 12; c.sigma0 = static_cast<std::uint64_t>(seed);
        const BasePlan bp = build_base_plan(c);
        if (!bp.ok) continue;
        const Scenario sc = generate_scenario(scenario_params(c, c.sigma0), bp.n0, bp.reference_makespan);
        if (!sc.ok) continue;
        for (const char* scan_name : {"g0", "g3"}) for (const char* adm_name : {"a0", "apm"}) {
            DynamicConfig cc = c; cc.scan = scan_name; cc.admission = adm_name;
            const auto scan = make_scan(cc); const auto adm = make_admission(cc);
            const PromiseParams pp = make_promise(cc);
            std::string sig_identity; double final_r0 = 0.0;
            for (OptArm arm : {OptArm::R0, OptArm::FR, OptArm::RIC_C, OptArm::FT}) {
                RegionalOptimizer ro(cfg_of(arm));
                P15Checker chk(&ro);
                SimOptions so; so.omega = cc.omega; so.scan = scan.get(); so.admission = adm.get(); so.promise = pp;
                so.optimizer = (std::string(adm_name) == "apm") ? static_cast<const Optimizer*>(&chk) : static_cast<const Optimizer*>(&ro);
                const SimResult r = simulate(*bp.plan, sc, so);
                ++runs;
                const std::string m = std::string(scan_name) + "/" + adm_name + "/" + opt_arm_name(arm) + " seed" + std::to_string(seed);
                CHECKMSG(r.ok, m + (r.errors.empty() ? "" : r.errors.front()));
                std::size_t total = 0; for (auto n : r.counts) total += n;
                CHECKMSG(total == sc.requests.size(), m);                                    // the six-way partition is exact
                CHECKMSG(r.final_plan && r.final_plan->consistent(), m);
                for (const auto& rec : r.records) CHECKMSG(rec.invariants_ok, m);
                const SimResult r2 = simulate(*bp.plan, sc, so);                             // deterministic replay (records accumulate; the decision results must not differ)
                CHECKMSG(sig(r) == sig(r2), m + " replay");
                if (arm == OptArm::R0) {
                    // identity optimiser: exactly the DM6 pipeline
                    SimOptions s0 = so; s0.optimizer = nullptr;
                    CHECKMSG(sig(simulate(*bp.plan, sc, s0)) == sig(r), m + " R0 == DM6 pipeline");
                    sig_identity = sig(r); final_r0 = r.final_objective;
                } else {
                    CHECKMSG(!ro.records.empty(), m + " the optimiser hook was consulted");   // A0 and A-PM both route through the optimiser
                    if (r.final_objective < final_r0 - 1e-6) ++better_than_r0;           // the optimiser's plan is what gets installed
                    for (const auto& rec : ro.records) { CHECKMSG(rec.f_after <= rec.f_before + 1e-9 && rec.commitment_violations == 0 && rec.protected_changes == 0, m); if (rec.f_after < rec.f_before - 1e-9) ++optimised_installs; }
                }
                decisions += chk.decisions; fallbacks += chk.fallback; rejected_promise += static_cast<long>(r.counts[static_cast<int>(Outcome::RejectedPromise)]);
                if (std::string(adm_name) == "apm") CHECKMSG(r.shift_base.violated == 0 && r.shift_new.violated == 0, m); // Phi is enforced for installed optimiser plans (I7 also re-checked inside)
            }
        }
    }
    std::printf("   [info] simulations %ld, A-PM decisions with [P15] checked on identical inputs %ld, U1 fallbacks %ld, rejected-promise %ld, optimiser records with F improved %ld\n",
                runs, decisions, fallbacks, rejected_promise, optimised_installs);
    CHECK(runs > 0 && decisions > 0);
    std::printf("   [info] simulation runs whose final objective beats R0 (same scenario): %ld\n", better_than_r0);
    CHECK(better_than_r0 > 0);
}


// ---- fault (vi), exactly as DM0 T13 states it: region = FULL TAIL (Lambda0 = [cut, end depot]) with the search budget exhausted before the sweep -> budget-stop, never certified ----
void test_fault_vi_exact(const std::vector<Case>& cases) {
    std::size_t full = 0, cut_off = 0, ctl_certified = 0, ctl_total = 0;
    for (const auto& c : cases) {
        OptConfig cfg = cfg_of(OptArm::RIC_C); cfg.w = 100000; cfg.calls_override = 1;   // a RIC margin that spans the whole tail; ONE operator call
        const OptResult r = run(c, cfg);
        CHECKMSG(r.region_valid && r.region_full_tail, c.label + " Lambda0 is the full tail");
        if (r.region_full_tail) ++full;
        if (r.stop == OptStop::Budget) { // the search was cut off by its budget before the sweep
            ++cut_off;
            CHECKMSG(r.cert == CertStatus::BudgetStop && r.cert != CertStatus::Certified && r.manifest.ran && !r.manifest.failed.empty(), c.label + " fault vi");
            CHECKMSG(r.manifest.failed == "BUDGET", c.label + " reported '" + r.manifest.failed + "'");
        }
        // control: the same full-tail Lambda0 with the default budget CAN be certified (the test has power), and then the search was not cut off
        OptConfig ctl = cfg_of(OptArm::RIC_C); ctl.w = 100000;
        const OptResult q = run(c, ctl);
        ++ctl_total;
        if (q.cert == CertStatus::Certified) { ++ctl_certified; CHECKMSG(q.region_full_tail && q.stop != OptStop::Budget, c.label); }
    }
    std::printf("   [info] fault (vi) exact: %zu of %zu cases have Lambda0 = full tail; %zu with the search budget exhausted before the sweep -> all budget-stop; control with default budget certified %zu of %zu\n",
                full, cases.size(), cut_off, ctl_certified, ctl_total);
    CHECK(full == cases.size() && cut_off > 0 && ctl_certified > 0);
}

// ---- tolerance (M5) ----
void test_tolerance(const std::vector<Case>& cases) {
    CHECK(OptConfig{}.tau_imp == 0.0 && tdr::search::VnsOptions{}.min_improvement == OptConfig{}.tau_imp);   // the static VNS default: strict '<'
    std::size_t big = 0, invalid = 0;
    for (const auto& c : cases) {
        OptConfig h = cfg_of(OptArm::RIC_C); h.tau_imp = 1e12;                     // a declared, huge tolerance: nothing counts as improving; it is logged
        const OptResult r = run(c, h);
        CHECKMSG(r.accepted_moves == 0 && same_solution(r.plan, c.cand->solution()) && r.manifest.tau_imp == 1e12 && r.manifest.tau_search == 1e12 && r.manifest.m5, c.label);
        ++big;
        for (double bad : {-1.0, std::nan("")}) { // an invalid tolerance is a configuration error: caught, S+ returned, RIC-C budget-stop, nothing applied
            OptConfig b = cfg_of(OptArm::RIC_C); b.tau_imp = bad;
            const OptResult q = run(c, b);
            CHECKMSG(q.failed && q.stop == OptStop::Error && q.cert == CertStatus::BudgetStop && q.arm == OptArm::RIC_C && same_solution(q.plan, c.cand->solution()) && q.failure_reason.find("tau_imp") != std::string::npos, c.label);
            ++invalid;
        }
        break; // one case is enough for the configuration checks
    }
    std::printf("   [info] tolerance: default tau_imp == VnsOptions::min_improvement == 0 (strict); logged in the manifest; huge-tau and invalid-tau configurations checked (%zu, %zu)\n", big, invalid);
}

// ---- exceptions never escape and never lose S+ ----
void test_exceptions(const std::vector<Case>& cases) {
    std::size_t triggered[3][2] = {{0, 0}, {0, 0}, {0, 0}}, seen_expansion = 0;
    for (const auto& c : cases) {
        const OptResult base = run(c, cfg_of(OptArm::RIC_C));
        for (int kind : {1, 2}) for (int site = 0; site < 3; ++site) for (OptArm arm : {OptArm::FR, OptArm::RIC_C, OptArm::FT, OptArm::B2, OptArm::B4}) {
            OptConfig cfg = cfg_of(arm);
            (site == 0 ? cfg.faults.throw_in_region : site == 1 ? cfg.faults.throw_in_search : cfg.faults.throw_in_expansion) = kind;
            const OptResult r = run(c, cfg);        // must not throw: an escaping exception terminates the test binary
            const bool expect_fail = site == 0 || (site == 1 && run(c, cfg_of(arm)).search_calls > 0) || (site == 2 && arm == OptArm::RIC_C && base.expansions > 0);
            const std::string m = c.label + " " + opt_arm_name(arm) + " site" + std::to_string(site) + " kind" + std::to_string(kind);
            CHECKMSG(r.failed == expect_fail, m);
            if (r.failed) {
                ++triggered[site][kind - 1];
                CHECKMSG(r.arm == arm && r.stop == OptStop::Error && same_solution(r.plan, c.cand->solution()) && r.f_after == r.f_before && !r.failure_reason.empty(), m);
                CHECKMSG(r.failure_reason == (kind == 1 ? "exception: injected std::exception" : "non-standard exception"), m);
                if (arm == OptArm::RIC_C) CHECKMSG(r.cert == CertStatus::BudgetStop && r.manifest.failed == "EXCEPTION" && !r.manifest.failure.empty(), m);
                else CHECKMSG(r.cert == CertStatus::NotRun, m);
                CHECKMSG(r.commitment_violations == 0 && r.protected_changes == 0, m);
                if (site == 2) ++seen_expansion;
            }
        }
    }
    for (int site = 0; site < 3; ++site) for (int k = 0; k < 2; ++k) CHECK(triggered[site][k] > 0);
    std::printf("   [info] exceptions (std and non-std) in region / search / expansion: triggered %zu/%zu, %zu/%zu, %zu/%zu times, none escaped; S+ returned, RIC-C -> budget-stop with arm kept\n",
                triggered[0][0], triggered[0][1], triggered[1][0], triggered[1][1], triggered[2][0], triggered[2][1]);
}

// ---- U1 witness: A-PM decisions verified against an independent recomputation of the chain on the SAME inputs ----
struct U1Witness final : Optimizer {
    const Optimizer* inner; mutable long decisions = 0, fallback = 0, primary = 0, rejected = 0, violating_with_feasible = 0, optimiser_threw = 0;
    explicit U1Witness(const Optimizer* i) : inner(i) {}
    std::optional<Solution> optimise(const AdmissionInput& in, const Solution& s_plus) const override {
        auto phi_ok = [&](const Solution& s) {
            const auto p = InstalledPlan::make(s, in.tentative, in.theta, in.plan.version() + 1);
            return p && phi(*p, in.ledger, in.promise, in.cid, in.request.time).ok;
        };
        // independent expectation: the cheapest evaluator-feasible insertion that satisfies Phi (stable by objective, ties to the earlier gap)
        std::vector<std::size_t> order;
        for (std::size_t i = 0; i < in.ins.trials.size(); ++i) if (in.ins.trials[i].feasible) order.push_back(i);
        std::stable_sort(order.begin(), order.end(), [&](std::size_t x, std::size_t y) { return in.ins.trials[x].objective < in.ins.trials[y].objective; });
        const Solution* expected = nullptr;
        for (std::size_t i : order) if (phi_ok(in.ins.trials[i].solution)) { expected = &in.ins.trials[i].solution; break; }
        Solution cand = s_plus; bool threw = false;
        try { const auto o = inner->optimise(in, s_plus); if (o) cand = *o; } catch (...) { threw = true; }
        const bool cand_ok = phi_ok(cand);
        AdmitAPM apm;
        const Decision d = apm.decide(in, *inner);
        ++decisions; if (threw) { ++optimiser_threw; CHECK(d.optimiser_failed); }
        CHECK(in.ins.best.has_value());                                           // the cheapest evaluator-feasible insertion is retained: nothing below may lose it
        if (cand_ok) { ++primary; CHECK(d.outcome == Outcome::Accepted && d.stage == Stage::Primary && d.solution && same_solution(*d.solution, cand)); }
        else if (expected) {                                                      // optimised plan violates Phi, a Phi-feasible insertion exists
            ++fallback; ++violating_with_feasible;
            CHECK(d.outcome == Outcome::Accepted && d.stage == Stage::Fallback && d.solution && same_solution(*d.solution, *expected));
        } else { ++rejected; CHECK(d.outcome == Outcome::RejectedPromise); }
        // the acceptance set never shrinks relative to B1 insertion only
        if (expected) CHECK(d.outcome == Outcome::Accepted);
        if (threw) return inner->optimise(in, s_plus);                            // rethrow into the simulator's own guard path
        return cand;
    }
};

struct WorstFirst final : Optimizer { // an "optimiser" that proposes the most delaying insertion: evaluator-feasible, usually violates Phi
    std::optional<Solution> optimise(const AdmissionInput& in, const Solution&) const override {
        std::size_t w = 0; double worst = -1;
        for (std::size_t i = 0; i < in.ins.trials.size(); ++i) if (in.ins.trials[i].feasible && in.ins.trials[i].objective > worst) { worst = in.ins.trials[i].objective; w = i; }
        return in.ins.trials[w].solution;
    }
};
struct NoPlan final : Optimizer { std::optional<Solution> optimise(const AdmissionInput&, const Solution&) const override { return std::nullopt; } };
struct ThrowStd final : Optimizer { std::optional<Solution> optimise(const AdmissionInput&, const Solution&) const override { throw std::runtime_error("boom"); } };
struct ThrowOther final : Optimizer { std::optional<Solution> optimise(const AdmissionInput&, const Solution&) const override { throw 7; } };

void test_u1_evidence() {
    struct Row { const char* name; const Optimizer* opt; long decisions = 0, fallback = 0, primary = 0, rejected = 0, threw = 0; };
    WorstFirst wf; NoPlan np; ThrowStd ts; ThrowOther to;
    RegionalOptimizer ric(cfg_of(OptArm::RIC_C)), ft(cfg_of(OptArm::FT));
    OptConfig fc = cfg_of(OptArm::RIC_C); fc.faults.throw_in_search = 2;            // the real optimiser itself failing with a non-std exception
    RegionalOptimizer faulty(fc);
    Row rows[] = {{"worst-first (Phi-violating)", &wf}, {"returns nothing", &np}, {"throws std::exception", &ts}, {"throws non-std", &to}, {"RIC-C", &ric}, {"FT", &ft}, {"RIC-C with injected non-std throw", &faulty}};
    long sims = 0;
    for (int seed = 0; seed < 8; ++seed) {
        DynamicConfig c;
        c.regime = 'F'; apply_regime(c);
        c.base.base.instance.num_customers = 12 + seed % 4; c.base.base.instance.dataset_index = 300 + seed; c.base.base.vehicle.num_drones = 2 + seed % 2;
        c.base.vns.iter_max = 100; c.rho = 0.8; c.omega = 10.0; c.m_max = 14; c.sigma0 = static_cast<std::uint64_t>(seed);
        const BasePlan bp = build_base_plan(c);
        if (!bp.ok) continue;
        for (double eps : {0.0, 0.25, 1.0}) {
            DynamicConfig cc = c; cc.admission = "apm"; cc.scan = "g0"; cc.eps_tilde = eps;
            const Scenario sc = generate_scenario(scenario_params(cc, cc.sigma0), bp.n0, bp.reference_makespan);
            if (!sc.ok) continue;
            const auto scan = make_scan(cc); const auto adm = make_admission(cc); const PromiseParams pp = make_promise(cc);
            SimOptions id; id.omega = cc.omega; id.scan = scan.get(); id.admission = adm.get(); id.promise = pp;     // identity optimiser
            const std::string sig_id = sig(simulate(*bp.plan, sc, id));
            for (auto& row : rows) {
                U1Witness w(row.opt);
                SimOptions so = id; so.optimizer = &w;
                const SimResult r = simulate(*bp.plan, sc, so);                        // must not throw: exceptions are absorbed by the A-PM guard
                ++sims;
                CHECKMSG(r.ok, std::string(row.name));
                row.decisions += w.decisions; row.fallback += w.fallback; row.primary += w.primary; row.rejected += w.rejected; row.threw += w.optimiser_threw;
                // an optimiser that only throws / returns nothing is exactly the identity pipeline
                if (row.opt == &np || row.opt == &ts || row.opt == &to || row.opt == &faulty) CHECKMSG(sig(r) == sig_id, std::string(row.name) + " == identity pipeline");
            }
        }
    }
    for (const auto& row : rows) {
        std::printf("   [info] U1 witness %-34s decisions %5ld  primary %5ld  FALLBACK %5ld  rejected-promise %4ld  optimiser threw %5ld\n", row.name, row.decisions, row.primary, row.fallback, row.rejected, row.threw);
        CHECKMSG(row.decisions > 0 && row.fallback > 0, std::string("fallback must be exercised for ") + row.name);   // explicit, never vacuous
    }
    CHECK(rows[2].threw > 0 && rows[3].threw > 0);   // rows[6]: RegionalOptimizer absorbs its own fault internally, so nothing reaches the A-PM guard; it is checked equal to the identity pipeline above
    std::printf("   [info] U1 witness simulations: %ld\n", sims);
}

// ---- B4 at simulator level: its budget is the RIC region's, per decision, on identical inputs ----
struct BudgetMatch final : Optimizer {
    RegionalOptimizer fr{cfg_of(OptArm::FR)}, ric{cfg_of(OptArm::RIC_C)}, b4{cfg_of(OptArm::B4)}, ft{cfg_of(OptArm::FT)};
    mutable long decisions = 0, equal = 0, ft_differs = 0, eval_differs = 0, b4_tail_larger = 0;
    mutable double b4_eval = 0, fr_eval = 0;
    std::optional<Solution> optimise(const AdmissionInput& in, const Solution& s_plus) const override {
        const auto r = fr.optimise(in, s_plus); ric.optimise(in, s_plus); b4.optimise(in, s_plus); ft.optimise(in, s_plus);
        const OptResult &a = fr.records.back(), &rc = ric.records.back(), &b = b4.records.back(), &f = ft.records.back();
        if (a.region_valid && b.region_valid) {
            ++decisions;
            CHECK(b.budget_source == "ric-region" && b.budget_calls == a.budget_calls && b.budget_calls == rc.budget_calls && b.ric_free_customers == a.free_customers);
            if (b.budget_calls == a.budget_calls) ++equal;
            if (f.budget_calls != a.budget_calls) ++ft_differs;          // FT's OWN budget would not have matched
            if (b.free_customers >= a.free_customers) ++b4_tail_larger;
            if (b.evaluator_calls != a.evaluator_calls) ++eval_differs;
            b4_eval += static_cast<double>(b.evaluator_calls); fr_eval += static_cast<double>(a.evaluator_calls);
        }
        return r;
    }
};
void test_b4_budget_simulator() {
    long decisions = 0, equal = 0, ft_differs = 0, eval_differs = 0, tail_larger = 0; double b4e = 0, fre = 0;
    for (int seed = 0; seed < 6; ++seed) for (const char* adm_name : {"a0", "apm"}) {
        DynamicConfig c;
        c.regime = 'F'; apply_regime(c);
        c.base.base.instance.num_customers = 12 + seed % 4; c.base.base.instance.dataset_index = 400 + seed; c.base.base.vehicle.num_drones = 2 + seed % 2;
        c.base.vns.iter_max = 100; c.rho = 0.8; c.omega = 10.0; c.m_max = 14; c.sigma0 = static_cast<std::uint64_t>(seed); c.admission = adm_name; c.scan = "g0";
        const BasePlan bp = build_base_plan(c);
        if (!bp.ok) continue;
        const Scenario sc = generate_scenario(scenario_params(c, c.sigma0), bp.n0, bp.reference_makespan);
        if (!sc.ok) continue;
        const auto scan = make_scan(c); const auto adm = make_admission(c);
        BudgetMatch bm;
        SimOptions so; so.omega = c.omega; so.scan = scan.get(); so.admission = adm.get(); so.promise = make_promise(c); so.optimizer = &bm;
        const SimResult r = simulate(*bp.plan, sc, so);
        CHECK(r.ok);
        decisions += bm.decisions; equal += bm.equal; ft_differs += bm.ft_differs; eval_differs += bm.eval_differs; tail_larger += bm.b4_tail_larger; b4e += bm.b4_eval; fre += bm.fr_eval;
    }
    std::printf("   [info] B4 budget at simulator level: %ld decisions, budgets equal to FR/RIC-C on %ld; FT's own budget would differ on %ld; B4 free region >= FR's on %ld; evaluator calls differ on %ld (B4 %.0f vs FR %.0f in total: equal operator-call budgets are NOT equal evaluator cost)\n",
                decisions, equal, ft_differs, tail_larger, eval_differs, b4e, fre);
    CHECK(decisions > 0 && equal == decisions && ft_differs > 0 && eval_differs > 0);
}

// ---- source guard: the regional optimiser never shakes, never runs run_vns, never uses an Rng overload ----------------------------------------------------
void test_source_guard() {
    std::string path = __FILE__;
    path = path.substr(0, path.find_last_of("/\\") + 1) + "../include/dynamic/regional_optimiser.hpp";
    std::ifstream f(path);
    CHECKMSG(f.good(), path);
    std::string line, code;
    while (std::getline(f, line)) { const auto p = line.find("//"); code += (p == std::string::npos ? line : line.substr(0, p)) + "\n"; }
    CHECK(code.size() > 1000);
    CHECK(code.find("shake(") == std::string::npos && code.find("run_vns(") == std::string::npos && code.find("Rng") == std::string::npos && code.find("rng") == std::string::npos);
    CHECK(code.find("evaluate_candidates") == std::string::npos); // the default (true) is never overridden
    // and DM8 still refuses a scoped shake / a scoped non-evaluating call
    DynamicConfig c; c.regime = 'F'; apply_regime(c); c.base.base.instance.num_customers = 10; c.base.vns.iter_max = 50;
    const BasePlan bp = build_base_plan(c);
    CHECK(bp.ok);
    if (!bp.ok) return;
    ScopeMask m = ScopeMask::full();
    search::SearchContext ctx = bp.plan->world().context(); ctx.scope = &m;
    tdr::util::Rng rng(1);
    CHECK(search::shake(bp.plan->solution(), ctx, rng, search::ShakeOptions{}).status == search::ShakeStatus::InvalidParameters);
}

} // namespace

int main() {
    std::printf("DM9 regional optimiser tests\n");
    const std::vector<Case> cases = make_cases();
    std::printf("   [info] %zu cases (candidate S+ with a clean cut)\n", cases.size());
    CHECK(cases.size() >= (std::getenv("DM9_SEEDS") ? 1u : 30u)); // default 10 seeds give > 30 cases
    auto timed = [](const char* name, auto&& fn) { const auto t = std::chrono::steady_clock::now(); fn(); std::printf("   (%s: %.1fs)\n", name, std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count()); std::fflush(stdout); };
    timed("invariants", [&] { test_identity_and_invariants(cases); });
    timed("determinism", [&] { test_determinism(cases); });
    timed("accounting", [&] { test_accounting_spy(cases); });
    timed("certificate", [&] { test_certificate(cases); });
    timed("faults", [&] { test_faults(cases); });
    timed("budget/cap", [&] { test_budget_and_cap(cases); });
    timed("simulation", [&] { test_simulation(); });
    timed("fault vi exact", [&] { test_fault_vi_exact(cases); });
    timed("tolerance", [&] { test_tolerance(cases); });
    timed("exceptions", [&] { test_exceptions(cases); });
    timed("u1 evidence", [&] { test_u1_evidence(); });
    timed("b4 budget", [&] { test_b4_budget_simulator(); });
    timed("source", [&] { test_source_guard(); });
    std::printf("%ld checks, %ld failures\n", g_checks, g_fail);
    return g_fail == 0 ? 0 : 1;
}
