// DM8 scope hook: (A) ScopeMask unit tests, (B) bit-identity of the hooked static operators against the FROZEN pre-hook copy
// (tests/reference/search_pre_hook, verbatim from static-baseline-m10) for null scope and full mask, results/logs/evaluator calls/RNG state,
// (C) scope rejects never call the evaluator (spy), accounting identities, (D) a scoped brute-force oracle (G6), (E) DM7 cross-check,
// (F) determinism, (G) shakes refusal and unchanged unscoped shakes/VNS.  No DM9 optimiser is involved.
//
// The evaluator spy: timing_engine.hpp is included with evaluate_solution renamed, and a counting forwarder takes its name.
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
#include <cstdio>
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "search_pre_hook/neighbourhood_n1.hpp"
#include "search_pre_hook/neighbourhood_n2.hpp"
#include "search_pre_hook/neighbourhood_n3.hpp"
#include "search_pre_hook/neighbourhood_n4.hpp"
#include "search_pre_hook/neighbourhood_n5.hpp"
#include "search_pre_hook/neighbourhood_n6.hpp"
#include "search_pre_hook/neighbourhood_n7.hpp"
#include "search_pre_hook/neighbourhood_n8.hpp"
#include "search_pre_hook/shakes.hpp"
#include "search_pre_hook/vns.hpp"

#include "search/neighbourhood_n1.hpp"
#include "search/neighbourhood_n2.hpp"
#include "search/neighbourhood_n3.hpp"
#include "search/neighbourhood_n4.hpp"
#include "search/neighbourhood_n5.hpp"
#include "search/neighbourhood_n6.hpp"
#include "search/neighbourhood_n7.hpp"
#include "search/neighbourhood_n8.hpp"
#include "search/shakes.hpp"
#include "search/vns.hpp"

#include "dynamic/base_plan.hpp"
#include "dynamic/region_builder.hpp"
#include "dynamic/scope_mask.hpp"

using namespace tdr;
using namespace tdr::dynamic;
using tdr::model::DroneTrip;
using tdr::model::Node;
using tdr::model::NodeType;
using tdr::model::Solution;
using tdr::search::ScopeViolation;

static int g_checks = 0, g_fail = 0;
#define CHECK(cond) do { ++g_checks; if (!(cond)) { ++g_fail; std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECKMSG(cond, msg) do { ++g_checks; if (!(cond)) { ++g_fail; std::printf("FAIL %s:%d  %s  [%s]\n", __FILE__, __LINE__, #cond, std::string(msg).c_str()); } } while (0)

namespace {

// ===================================================== (A) ScopeMask unit tests (no evaluator) =====================================================
model::NodeRegistry registry() {
    model::NodeRegistry n;
    n.add(Node{0, NodeType::Depot, {0, 0}});
    for (int i = 1; i <= 7; ++i) n.add(Node{i, NodeType::Customer, {1000.0 * i, 0}});
    for (int i : {20, 21, 22}) n.add(Node{i, NodeType::Customer, {1000.0 * (i - 19), 500}});
    for (int f = 1000; f <= 1003; ++f) n.add(Node{f, NodeType::FlexibleLaunchPoint, {1000.0 * (f - 999), 100}});
    return n;
}
// route 0 c1 c2 F1000 c3 c4 F1001 c5 c6 c7 0 ; trip A (F1000 -> 20 -> c4), trip B (F1001 -> 21 -> c6)
Solution unit_plan() {
    Solution s;
    s.truck_route = {0, 1, 2, 1000, 3, 4, 1001, 5, 6, 7, 0};
    s.drone_trips = {DroneTrip{0, 1000, 20, 4}, DroneTrip{1, 1001, 21, 6}};
    return s;
}
search::Move mv(model::NodeId customer = model::kInvalidNodeId, model::NodeId partner = model::kInvalidNodeId) { search::Move m; m.customer = customer; m.partner = partner; return m; }

void test_mask_units() {
    const auto n = registry(); const Solution s = unit_plan();
    RegionR3 r3; r3.w = 0;
    const PlanLayout L = layout_of(s, n);
    const Region R = r3.build(RegionContext{L, n, 0, 3});
    CHECK(R.valid && R.lo == 2 && R.hi == 4 && R.free_customers == (std::vector<model::NodeId>{3, 20}) && R.pinned_nodes == (std::vector<model::NodeId>{2, 4}));
    const ScopeMask M = ScopeMask::from_region(R, s, n);
    CHECK(!M.unrestricted() && !M.empty_region());
    CHECK(M.free_customers() == (std::vector<model::NodeId>{3, 20}));
    CHECK(M.is_free(3) && M.is_free(20) && !M.is_free(1) && !M.is_free(21) && !M.is_free(4));
    CHECK(M.protected_truck_customers() == (std::vector<model::NodeId>{1, 2, 4, 5, 6, 7}));
    CHECK(M.protected_trips().size() == 1 && M.protected_trips()[0].customer == 21 && M.protected_trips()[0].launch == 1001 && M.protected_trips()[0].recovery == 6 && M.protected_trips()[0].drone == 1);
    CHECK(M.pinned_order() == (std::vector<model::NodeId>{2, 4}));
    CHECK(M.free_truck_customers(s, n) == (std::vector<model::NodeId>{3}));
    CHECK(M.free_trip_indices(s) == (std::vector<std::size_t>{0}));

    // the identity candidate passes
    CHECK(M.check(s, mv()) == ScopeViolation::None);
    CHECK(M.check(s, mv(3)) == ScopeViolation::None && M.check(s, mv(3, 20)) == ScopeViolation::None);
    CHECK(M.protected_prefix() == (std::vector<model::NodeId>{0, 1, 2}) && M.protected_suffix() == (std::vector<model::NodeId>{4, 1001, 5, 6, 7, 0}));
    // (a) rule 2: free customer 3 moved behind boundary hi (node 4) enters the protected suffix -> rejected before evaluation
    Solution a = s; a.truck_route = {0, 1, 2, 1000, 4, 3, 1001, 5, 6, 7, 0};
    CHECK(M.check(a, mv(3)) == ScopeViolation::ProtectedSuffixChanged);
    CHECK(!check_region_preserved(s, a, R, n).empty());
    // (b) rule 2: free customer crossing the lower boundary lo (node 2) enters the protected prefix -> rejected; DM7 independently flags it too
    Solution b = s; b.truck_route = {0, 1, 3, 2, 1000, 4, 1001, 5, 6, 7, 0};
    CHECK(M.check(b, mv(3)) == ScopeViolation::ProtectedPrefixChanged);
    CHECK(!check_region_preserved(s, b, R, n).empty());
    // (c) protected anchor / partner
    CHECK(M.check(s, mv(5)) == ScopeViolation::MoveAnchorNotFree);
    CHECK(M.check(s, mv(3, 4)) == ScopeViolation::MoveAnchorNotFree);   // pinned customer 4 is not free
    CHECK(M.check(s, mv(21)) == ScopeViolation::MoveAnchorNotFree);
    // (d) protected truck customer missing / duplicated / drone-served
    Solution d = s; d.truck_route = {0, 1, 2, 1000, 3, 4, 1001, 6, 7, 0};
    CHECK(M.check(d, mv()) == ScopeViolation::ProtectedTruckCustomerChanged);
    d = s; d.truck_route = {0, 1, 2, 1000, 3, 4, 1001, 5, 5, 6, 7, 0};
    CHECK(M.check(d, mv()) == ScopeViolation::ProtectedTruckCustomerChanged);
    d = s; d.drone_trips.push_back(DroneTrip{0, 1002, 5, 7});
    CHECK(M.check(d, mv()) == ScopeViolation::ProtectedTruckCustomerChanged);
    // (e) protected drone customer back on the truck / trip dissolved / trip changed
    Solution e = s; e.truck_route = {0, 1, 2, 1000, 3, 4, 1001, 21, 5, 6, 7, 0};
    CHECK(M.check(e, mv()) == ScopeViolation::ProtectedDroneCustomerChanged);
    e = s; e.drone_trips.pop_back();
    CHECK(M.check(e, mv()) == ScopeViolation::ProtectedDroneCustomerChanged);
    e = s; e.drone_trips[1].drone_id = 0;
    CHECK(M.check(e, mv()) == ScopeViolation::ProtectedDroneCustomerChanged);
    e = s; e.drone_trips[1].launch_node = 1002;
    CHECK(M.check(e, mv()) == ScopeViolation::ProtectedDroneCustomerChanged);
    e = s; e.drone_trips[1].recovery_node = 7;
    CHECK(M.check(e, mv()) == ScopeViolation::ProtectedDroneCustomerChanged);
    e = s; e.drone_trips.push_back(DroneTrip{0, 1002, 21, 7});   // a second trip serving the protected customer
    CHECK(M.check(e, mv()) == ScopeViolation::ProtectedDroneCustomerChanged);
    // (f) protected trip dependency: recovery 6 moved before launch F1001 (6 stays on the route exactly once)
    Solution f = s; f.truck_route = {0, 1, 2, 1000, 3, 4, 6, 1001, 5, 7, 0};
    CHECK(M.check(f, mv()) == ScopeViolation::ProtectedTripDependencyBroken);
    f = s; f.truck_route = {0, 1, 2, 1000, 3, 4, 5, 6, 7, 0};  // launch FLP gone from the route
    CHECK(M.check(f, mv()) == ScopeViolation::ProtectedTripDependencyBroken);
    // (g) pinned order: swap pinned customers 2 and 4 (free trip A, so no protected dependency is hit)
    Solution g = s; g.truck_route = {0, 1, 4, 1000, 3, 2, 1001, 5, 6, 7, 0};
    CHECK(M.check(g, mv()) == ScopeViolation::PinnedOrderChanged);

    // (i) rule 2, trips: a trip serving a free customer must launch inside the span and be recovered no later than hi
    Solution ti = s; ti.drone_trips[0].launch_node = 1001;   // launch FLP now in the suffix
    CHECK(M.check(ti, mv()) == ScopeViolation::FreeTripOutsideRegion);
    ti = s; ti.drone_trips[0].launch_node = 1;               // launch in the prefix
    CHECK(M.check(ti, mv()) == ScopeViolation::FreeTripOutsideRegion);
    ti = s; ti.drone_trips[0].recovery_node = 5;             // recovered behind hi
    CHECK(M.check(ti, mv()) == ScopeViolation::FreeTripOutsideRegion);
    ti = s; ti.drone_trips[0].recovery_node = 4;             // recovery AT hi is the allowed extreme (unchanged plan)
    CHECK(M.check(ti, mv()) == ScopeViolation::None);
    ti = s; ti.drone_trips[0].recovery_node = 3;             // inside the span
    CHECK(M.check(ti, mv()) == ScopeViolation::None);
    // (j) rule 1 preserved: free customers cross an INTERIOR pinned node (a frozen recovery); only lo/hi are the protected boundary
    Region X = R; X.lo = 1; X.hi = 6; X.free_customers = {2, 3, 5, 20, 21}; X.pinned_nodes = {1, 4, 6};
    X.free_trips = {0, 1}; X.frozen_trips = {};
    const ScopeMask MX = ScopeMask::from_region(X, s, n);
    CHECK(MX.protected_prefix() == (std::vector<model::NodeId>{0, 1}) && MX.protected_suffix() == (std::vector<model::NodeId>{6, 7, 0}) && MX.pinned_order() == (std::vector<model::NodeId>{1, 4, 6}));
    Solution jx = s; jx.truck_route = {0, 1, 2, 1000, 3, 5, 4, 1001, 6, 7, 0};   // 5 crosses pinned 4 (order 1,4,6 kept)
    CHECK(MX.check(jx, mv(5)) == ScopeViolation::None);
    jx = s; jx.truck_route = {0, 1, 2, 1000, 4, 3, 1001, 5, 6, 7, 0};            // 3 crosses pinned 4 the other way
    CHECK(MX.check(jx, mv(3)) == ScopeViolation::None);
    jx = s; jx.truck_route = {0, 1, 2, 1000, 3, 1001, 5, 6, 4, 7, 0};            // 4 itself leaves its place relative to 6: pinned order broken
    CHECK(MX.check(jx, mv()) != ScopeViolation::None);
    jx = s; jx.truck_route = {0, 2, 1, 1000, 3, 4, 1001, 5, 6, 7, 0};            // 2 crosses lo (node 1): prefix changed
    CHECK(MX.check(jx, mv(2)) == ScopeViolation::ProtectedPrefixChanged);
    // pinned node that is not a customer: pinned depots (a hand-built region whose lo is the start depot)
    Region H = R; H.pinned_nodes = {0, 2, 4}; H.free_customers = {3, 20};
    const ScopeMask MH = ScopeMask::from_region(H, s, n);
    CHECK(MH.pinned_order() == (std::vector<model::NodeId>{0, 2, 4, 0}));
    Solution h = s; h.truck_route.pop_back();   // end depot gone
    CHECK(MH.check(h, mv()) == ScopeViolation::PinnedNodeMissing);
    CHECK(MH.check(s, mv()) == ScopeViolation::None);

    // pinned recovery customers are excluded from the free set even if the DM7 region lists them as free
    Region P = R; P.free_customers = {3, 4, 20}; P.pinned_nodes = {2, 4};
    const ScopeMask MP = ScopeMask::from_region(P, s, n);
    CHECK(!MP.is_free(4) && MP.is_free(3) && MP.free_customers() == (std::vector<model::NodeId>{3, 20}));
    CHECK(MP.check(s, mv(4)) == ScopeViolation::MoveAnchorNotFree);

    // R0 / invalid: no free customers, no anchors, every move rejected, identity still passes
    Region E; E.valid = true; E.empty = true; E.kind = RegionKind::None;
    const ScopeMask M0 = ScopeMask::from_region(E, s, n);
    CHECK(M0.empty_region() && M0.free_customers().empty() && M0.free_truck_customers(s, n).empty() && M0.free_trip_indices(s).empty());
    CHECK(M0.check(s, mv()) == ScopeViolation::None && M0.check(s, mv(3)) == ScopeViolation::MoveAnchorNotFree && M0.check(s, mv(20)) == ScopeViolation::MoveAnchorNotFree);
    CHECK(M0.check(a, mv()) == ScopeViolation::ProtectedPrefixChanged);   // R0: the whole route is the protected prefix
    CHECK(!check_region_preserved(s, a, E, n).empty());
    Region V; V.valid = false;
    const ScopeMask MV = ScopeMask::from_region(V, s, n);
    CHECK(MV.empty_region() && MV.free_customers().empty());
    // full mask: always None, everything free, no anchors restricted
    const ScopeMask MF = ScopeMask::full();
    CHECK(MF.unrestricted() && MF.is_free(1) && MF.is_free(21) && MF.check(d, mv(5)) == ScopeViolation::None && MF.check(g, mv(21, 4)) == ScopeViolation::None);
    // determinism of the mask itself
    const ScopeMask M2 = ScopeMask::from_region(R, s, n);
    CHECK(M2.free_customers() == M.free_customers() && M2.pinned_order() == M.pinned_order() && M2.protected_truck_customers() == M.protected_truck_customers());
}

// ===================================================== (B..G) operators on real plans =====================================================
struct Cfg {
    bool rec = true, eval = true;
    int pol = 1;               // N4: 0 Reject, 1 Reanchor
    std::size_t cap = 100;     // N4 re-anchor budget
    int mode = 2;              // N4: 0 Relocate, 1 Exchange, 2 both
    bool same_drone = true;    // N6
    int n8w = -1;              // N8 insertion window (-1: all gaps)
};

#define TDR_API(STRUCT, NS)                                                                                                                  \
struct STRUCT {                                                                                                                              \
    using Ctx = NS::SearchContext;                                                                                                           \
    using Res = NS::OperatorResult;                                                                                                          \
    template <class O> static void common(O& o, const Cfg& k) { o.record_candidates = k.rec; o.evaluate_candidates = k.eval; }               \
    static Res by_anchor(int op, const Solution& b, const Ctx& c, std::size_t a, const Cfg& k) {                                             \
        switch (op) {                                                                                                                        \
            case 1: { NS::N1Options o; common(o, k); return NS::n1_launch_node_relocate(b, c, a, o); }                                       \
            case 2: { NS::N2Options o; common(o, k); return NS::n2_recover_node_relocate(b, c, a, o); }                                      \
            case 3: { NS::N3Options o; common(o, k); return NS::n3_drone_service_node_relocate(b, c, a, o); }                                \
            case 4: { NS::N4Options o; common(o, k); o.mode = static_cast<NS::N4Mode>(k.mode); o.affected_trips = static_cast<NS::AffectedTripPolicy>(k.pol); \
                      o.max_reanchor_variants = k.cap; return NS::n4_truck_service_node_relocate(b, c, static_cast<tdr::model::NodeId>(a), o); }  \
            case 5: { NS::N5Options o; common(o, k); return NS::n5_td_service_node_swap(b, c, a, o); }                                       \
            case 6: { NS::N6Options o; common(o, k); o.include_same_drone = k.same_drone; return NS::n6_drone_drone_swap(b, c, a, o); }      \
            case 7: { NS::N7Options o; common(o, k); return NS::n7_truck_node_delete_drone_insert(b, c, static_cast<tdr::model::NodeId>(a), o); } \
            default: { NS::N8Options o; common(o, k); if (k.n8w >= 0) o.insertion_window = k.n8w; return NS::n8_drone_node_delete_truck_insert(b, c, a, o); } \
        }                                                                                                                                    \
    }                                                                                                                                        \
    static Res by_rng(int op, const Solution& b, const Ctx& c, tdr::util::Rng& r, const Cfg& k) {                                            \
        switch (op) {                                                                                                                        \
            case 1: { NS::N1Options o; common(o, k); return NS::n1_launch_node_relocate(b, c, r, o); }                                       \
            case 2: { NS::N2Options o; common(o, k); return NS::n2_recover_node_relocate(b, c, r, o); }                                      \
            case 3: { NS::N3Options o; common(o, k); return NS::n3_drone_service_node_relocate(b, c, r, o); }                                \
            case 4: { NS::N4Options o; common(o, k); o.mode = static_cast<NS::N4Mode>(k.mode); o.affected_trips = static_cast<NS::AffectedTripPolicy>(k.pol); \
                      o.max_reanchor_variants = k.cap; return NS::n4_truck_service_node_relocate(b, c, r, o); }                              \
            case 5: { NS::N5Options o; common(o, k); return NS::n5_td_service_node_swap(b, c, r, o); }                                       \
            case 6: { NS::N6Options o; common(o, k); o.include_same_drone = k.same_drone; return NS::n6_drone_drone_swap(b, c, r, o); }      \
            case 7: { NS::N7Options o; common(o, k); return NS::n7_truck_node_delete_drone_insert(b, c, r, o); }                             \
            default: { NS::N8Options o; common(o, k); if (k.n8w >= 0) o.insertion_window = k.n8w; return NS::n8_drone_node_delete_truck_insert(b, c, r, o); } \
        }                                                                                                                                    \
    }                                                                                                                                        \
}
TDR_API(NewApi, tdr::search);
TDR_API(RefApi, tdr::search_ref);

search::SearchContext new_ctx(const search::SearchContext& c, const search::ScopeHook* h) { search::SearchContext x = c; x.scope = h; return x; }
search_ref::SearchContext ref_ctx(const search::SearchContext& c) { return search_ref::SearchContext{c.network, c.customers, c.flp_data, c.flp_index, c.adjacency_window}; }

// ---- comparison (field by field; the two result types live in different namespaces) ----
bool same_trip(const DroneTrip& a, const DroneTrip& b) {
    return a.drone_id == b.drone_id && a.launch_node == b.launch_node && a.service_customer == b.service_customer && a.recovery_node == b.recovery_node &&
           a.launch_position_in_route == b.launch_position_in_route && a.recovery_position_in_route == b.recovery_position_in_route;
}
bool same_sol(const Solution& a, const Solution& b) {
    if (a.truck_route != b.truck_route || a.drone_trips.size() != b.drone_trips.size()) return false;
    for (std::size_t i = 0; i < a.drone_trips.size(); ++i) if (!same_trip(a.drone_trips[i], b.drone_trips[i])) return false;
    return true;
}
template <class A, class B> bool same_move(const A& a, const B& b) {
    return static_cast<int>(a.kind) == static_cast<int>(b.kind) && a.customer == b.customer && a.partner == b.partner && a.gap == b.gap && a.replaced_trip == b.replaced_trip &&
           a.replaced_node == b.replaced_node && a.partner_trip == b.partner_trip && a.launch == b.launch && a.recovery == b.recovery && a.drone_id == b.drone_id;
}
template <class A, class B> bool same_re(const std::vector<A>& a, const std::vector<B>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) if (a[i].trip != b[i].trip || a[i].from != b[i].from || a[i].to != b[i].to) return false;
    return true;
}
template <class A, class B> std::string diff_rec(const A& a, const B& b) {
    if (!same_move(a.move, b.move)) return "move";
    if (static_cast<int>(a.reason) != static_cast<int>(b.reason)) return "reason";
    if (a.evaluated != b.evaluated) return "evaluated";
    if (a.objective != b.objective) return "objective";
    if (a.affected_trips != b.affected_trips) return "affected_trips";
    if (!same_re(a.reanchored, b.reanchored)) return "reanchored";
    if (a.variants_generated != b.variants_generated || a.variants_evaluated != b.variants_evaluated || a.combinations_total != b.combinations_total || a.cap_reached != b.cap_reached) return "variants";
    if (a.flps_dropped != b.flps_dropped) return "flps_dropped";
    if (!same_sol(a.solution, b.solution)) return "solution";
    return "";
}
// "" iff equal. With_scope: also compare the (new-only) scope fields to zero/None -- used for new-vs-ref; new-vs-new compares them for equality.
template <class A, class B> std::string diff_res(const A& a, const B& b) {
    if (static_cast<int>(a.op) != static_cast<int>(b.op)) return "op";
    if (static_cast<int>(a.status) != static_cast<int>(b.status)) return "status";
    if (a.base_objective != b.base_objective) return "base_objective";
    if (a.anchor_customer != b.anchor_customer || a.anchor_trip != b.anchor_trip) return "anchor";
    if (a.generated != b.generated) return "generated";
    if (a.evaluated != b.evaluated) return "evaluated";
    if (a.accepted != b.accepted) return "accepted";
    if (a.duplicates_skipped != b.duplicates_skipped) return "duplicates_skipped";
    for (std::size_t i = 0; i < search::kRejectReasonCount; ++i) if (a.rejected[i] != b.rejected[i]) return "rejected[" + std::to_string(i) + "]";
    if (a.stale_launch_candidates != b.stale_launch_candidates || a.reanchored_candidates != b.reanchored_candidates || a.trips_reanchored != b.trips_reanchored ||
        a.variants_generated != b.variants_generated || a.cap_reached_candidates != b.cap_reached_candidates) return "n4 diagnostics";
    if (a.best.has_value() != b.best.has_value()) return "best presence";
    if (a.best) {
        if (a.best->objective != b.best->objective) return "best objective";
        if (!same_sol(a.best->solution, b.best->solution)) return "best solution";
        if (!same_move(a.best->move, b.best->move)) return "best move";
        if (a.best->flps_dropped != b.best->flps_dropped) return "best flps_dropped";
        if (!same_re(a.best->reanchored, b.best->reanchored)) return "best reanchored";
    }
    if (a.log.size() != b.log.size()) return "log size";
    for (std::size_t i = 0; i < a.log.size(); ++i) if (const std::string d = diff_rec(a.log[i], b.log[i]); !d.empty()) return "log[" + std::to_string(i) + "]." + d;
    return "";
}
bool scope_fields_zero(const search::OperatorResult& r) {
    if (r.scope_rejected || r.scope_variants_skipped || r.evaluator_calls_avoided) return false;
    for (auto v : r.scope_by_violation) if (v) return false;
    for (const auto& rec : r.log) if (rec.scope_violation != ScopeViolation::None || rec.variants_scope_skipped) return false;
    return true;
}
std::string diff_scope(const search::OperatorResult& a, const search::OperatorResult& b) {
    if (a.scope_rejected != b.scope_rejected || a.scope_variants_skipped != b.scope_variants_skipped || a.evaluator_calls_avoided != b.evaluator_calls_avoided || a.scope_by_violation != b.scope_by_violation) return "scope counters";
    for (std::size_t i = 0; i < a.log.size(); ++i) if (a.log[i].scope_violation != b.log[i].scope_violation || a.log[i].variants_scope_skipped != b.log[i].variants_scope_skipped) return "scope log[" + std::to_string(i) + "]";
    return "";
}
bool structural(search::RejectReason r) {
    using R = search::RejectReason;
    return r == R::AffectedTrips || r == R::NoFreeDrone || r == R::NoReplacementFlp || r == R::InvalidRecovery;
}
search::Move to_new(const search_ref::Move& m) {
    search::Move x; x.kind = static_cast<search::MoveKind>(static_cast<int>(m.kind)); x.customer = m.customer; x.partner = m.partner; x.gap = m.gap; x.replaced_trip = m.replaced_trip;
    x.replaced_node = m.replaced_node; x.partner_trip = m.partner_trip; x.launch = m.launch; x.recovery = m.recovery; x.drone_id = m.drone_id; return x;
}
std::uint64_t rng_probe(tdr::util::Rng r) { return r.next_u64() ^ (r.next_u64() << 1); }

// hooks used as probes
struct PassAll final : search::ScopeHook { mutable long calls = 0; ScopeViolation check(const Solution&, const search::Move&) const override { ++calls; return ScopeViolation::None; } };
struct RejectAll final : search::ScopeHook {
    mutable long calls = 0; mutable long evals_at_calls_start = -1; mutable bool evaluated_between = false; mutable long last = -1;
    ScopeViolation check(const Solution&, const search::Move&) const override { ++calls; return ScopeViolation::MoveAnchorNotFree; }
};
// records the evaluator counter at every call: a hook call that is followed by an evaluator call for the SAME candidate cannot be told apart here, but a
// reject-all hook followed by zero evaluator calls beyond the base evaluation proves rejection precedes evaluation.

// Wraps any hook and proves, per candidate, that a rejection is never followed by an evaluator call before the next candidate is presented:
// the evaluator counter must be unchanged between a rejecting check() and the next check() (or the end of the operator call).
struct SpyHook final : search::ScopeHook {
    const search::ScopeHook& inner;
    mutable long last_eval = -1, bad = 0, calls = 0;
    mutable bool last_rej = false;
    mutable std::map<int, long> by;
    explicit SpyHook(const search::ScopeHook& h) : inner(h) {}
    ScopeViolation check(const Solution& c, const search::Move& m) const override {
        const long now = tdr::timing::g_eval_calls;
        if (last_rej && now != last_eval) ++bad;
        const ScopeViolation v = inner.check(c, m);
        last_rej = v != ScopeViolation::None; last_eval = now; ++calls;
        if (last_rej) ++by[static_cast<int>(v)];
        return v;
    }
    void finish() const { if (last_rej && tdr::timing::g_eval_calls != last_eval) ++bad; last_rej = false; }
};

// deterministic hooks whose verdict depends on the candidate's node ids, so different N4 re-anchor variants of ONE candidate get different verdicts (partial skipping)
struct ModHook final : search::ScopeHook {
    unsigned m, r;
    ModHook(unsigned mod, unsigned rem) : m(mod), r(rem) {}
    ScopeViolation check(const Solution& c, const search::Move&) const override {
        unsigned long sum = 0; for (auto v : c.truck_route) sum += v;
        for (const auto& t : c.drone_trips) sum += 3UL * static_cast<unsigned long>(t.launch_node) + static_cast<unsigned long>(t.recovery_node);
        return sum % m == r ? ScopeViolation::PinnedOrderChanged : ScopeViolation::None;
    }
};

// ---- fixtures ----
struct Fixture {
    BasePlan bp;
    std::string label;
    const Solution& plan() const { return bp.plan->solution(); }
    search::SearchContext ctx() const { return bp.plan->world().context(); }
    const model::NodeRegistry& nodes() const { return bp.plan->world().nodes(); }
};

struct Stats {
    long plans = 0, plans_with_trips = 0, trips_total = 0, ref_calls = 0, op_runs = 0, candidates = 0, scope_cands = 0, variants_skipped = 0, partial_variant = 0, allskip_variant = 0;
    long boundary_cands[9] = {0}, boundary_zero_eval_cands = 0, spy_bad = 0;
    long dm7_flagged_boundary = 0, dm7_checked = 0, masks = 0, regions_invalid = 0, accepted_scoped = 0, avoided = 0, n4_stale = 0;
    std::map<std::string, long> by_violation;
} S;

// One anchor/config under one mask: all scoped invariants against the unscoped reference run `ref` (record_candidates must be on in both).
void check_scoped(const std::string& tag, int op, const Fixture& F, const search::SearchContext& base_ctx, const Solution& plan, std::size_t anchor, const Cfg& cfg,
                  const RefApi::Res& ref, const search::ScopeHook& M, const Region* region) {
    const long c0 = tdr::timing::g_eval_calls;
    const search::SearchContext sc = new_ctx(base_ctx, &M);
    const auto r = NewApi::by_anchor(op, plan, sc, anchor, cfg);
    const long calls = tdr::timing::g_eval_calls - c0;
    // determinism
    const auto r2 = NewApi::by_anchor(op, plan, sc, anchor, cfg);
    CHECKMSG(diff_res(r, r2).empty() && diff_scope(r, r2).empty(), tag + " deterministic");
    // per-candidate proof: no evaluator call happens between a rejecting check() and the next candidate (boundary classes counted per operator)
    {
        const SpyHook spy(M);
        const search::SearchContext spc = new_ctx(base_ctx, &spy);
        const auto rs = NewApi::by_anchor(op, plan, spc, anchor, cfg);
        spy.finish();
        CHECKMSG(spy.bad == 0, tag + " a rejected candidate was followed by an evaluator call");
        CHECKMSG(diff_res(rs, r).empty() && diff_scope(rs, r).empty(), tag + " spy-wrapped hook gives the same result");
        CHECKMSG(rs.status != search::OperatorStatus::Ok || spy.calls == static_cast<long>(rs.evaluated) + static_cast<long>(rs.evaluator_calls_avoided), tag + " hook consulted once per evaluator-bound candidate/variant");
        S.spy_bad += spy.bad;
        for (const ScopeViolation bv : {ScopeViolation::ProtectedPrefixChanged, ScopeViolation::ProtectedSuffixChanged, ScopeViolation::FreeTripOutsideRegion}) {
            const auto it = spy.by.find(static_cast<int>(bv));
            if (it != spy.by.end()) { S.boundary_cands[op] += it->second; S.boundary_zero_eval_cands += it->second; }
        }
    }
    // identity 3: scope rejects never reach the evaluator (only the base evaluation + the evaluated candidates/variants)
    CHECKMSG(calls == 1 + static_cast<long>(r.evaluated), tag + " evaluator calls == 1 + evaluated");
    if (r.status != search::OperatorStatus::Ok) { CHECKMSG(ref.status == static_cast<decltype(ref.status)>(static_cast<int>(r.status)), tag + " status same"); return; }
    // counters
    CHECKMSG(r.generated == ref.generated && r.duplicates_skipped == ref.duplicates_skipped && r.variants_generated == ref.variants_generated &&
                 r.cap_reached_candidates == ref.cap_reached_candidates && r.stale_launch_candidates == ref.stale_launch_candidates, tag + " generation identical to unscoped");
    CHECKMSG(r.accepted + r.rejected_total() + r.scope_rejected == r.generated, tag + " accounting: accepted + rejected + scope_rejected == generated");
    CHECKMSG(r.evaluated + r.evaluator_calls_avoided == ref.evaluated, tag + " evaluated + avoided == unscoped evaluated");
    std::size_t sum = 0; for (std::size_t i = 1; i < search::kScopeViolationCount; ++i) sum += r.scope_by_violation[i];
    CHECKMSG(sum == r.scope_rejected && r.scope_by_violation[0] == 0, tag + " per-violation counts sum to scope_rejected");
    CHECKMSG(r.scope_variants_skipped <= r.evaluator_calls_avoided, tag + " variants skipped <= avoided");
    // structural rejections are unchanged (they come before the scope check); the evaluator rejection classes can only shrink
    for (std::size_t i = 1; i < search::kRejectReasonCount; ++i)
        if (structural(static_cast<search::RejectReason>(i))) CHECKMSG(r.rejected[i] == ref.rejected[i], tag + " structural rejects identical");
    // per-candidate oracle against the unscoped log
    CHECKMSG(r.log.size() == ref.log.size(), tag + " log size");
    bool partial = false;
    std::size_t exp_scope = 0, exp_avoided = 0;
    for (std::size_t i = 0; i < std::min(r.log.size(), ref.log.size()); ++i) {
        const auto& sr = r.log[i];
        const auto& rr = ref.log[i];
        CHECKMSG(same_move(sr.move, rr.move), tag + " same candidate order");
        const bool reached = rr.evaluated;           // the unscoped run sent it (or one of its variants) to the evaluator
        if (!reached) {
            CHECKMSG(sr.scope_violation == ScopeViolation::None && sr.variants_scope_skipped == 0 && static_cast<int>(sr.reason) == static_cast<int>(rr.reason) && !sr.evaluated, tag + " structural candidate untouched");
            continue;
        }
        if (rr.variants_generated == 0) {
            const ScopeViolation v = M.check(rr.solution, to_new(rr.move));
            if (v != ScopeViolation::None) {
                CHECKMSG(sr.scope_violation == v && !sr.evaluated && sr.reason == search::RejectReason::None, tag + " scope-rejected candidate named, not evaluated");
                ++exp_scope; ++exp_avoided;
                ++S.by_violation[search::scope_violation_name(v)];
            } else {
                CHECKMSG(sr.scope_violation == ScopeViolation::None && sr.evaluated && static_cast<int>(sr.reason) == static_cast<int>(rr.reason) && sr.objective == rr.objective, tag + " passing candidate judged exactly as unscoped");
            }
        } else {
            exp_avoided += sr.variants_scope_skipped;
            if (sr.variants_scope_skipped == 0) {
                CHECKMSG(sr.scope_violation == ScopeViolation::None && sr.evaluated && static_cast<int>(sr.reason) == static_cast<int>(rr.reason) && sr.objective == rr.objective && sr.variants_evaluated == rr.variants_evaluated &&
                             same_re(sr.reanchored, rr.reanchored), tag + " variant candidate without skips identical");
            } else if (sr.variants_scope_skipped == sr.variants_generated) {
                CHECKMSG(sr.scope_violation != ScopeViolation::None && !sr.evaluated && sr.variants_evaluated == 0 && sr.reason == search::RejectReason::None, tag + " all variants skipped -> scope-rejected candidate");
                CHECKMSG(M.check(sr.solution, sr.move) == sr.scope_violation, tag + " shown skipped variant carries the recorded violation");
                ++exp_scope; ++S.allskip_variant;
                ++S.by_violation[search::scope_violation_name(sr.scope_violation)];
            } else {
                partial = true; ++S.partial_variant;
                CHECKMSG(sr.scope_violation == ScopeViolation::None && sr.evaluated && sr.variants_evaluated >= 1 && sr.variants_evaluated + sr.variants_scope_skipped == sr.variants_generated, tag + " partially skipped candidate is still judged by the evaluator on its remaining variants");
            }
            CHECKMSG(sr.variants_generated == rr.variants_generated && sr.combinations_total == rr.combinations_total && sr.cap_reached == rr.cap_reached, tag + " variant enumeration (incl. cap) identical to unscoped");
            CHECKMSG(sr.variants_evaluated + sr.variants_scope_skipped == sr.variants_generated, tag + " every generated variant is evaluated or scope-skipped, never cap-skipped");
        }
    }
    // evaluator-reject classes can only shrink -- EXCEPT for a partially skipped N4 candidate, whose reported reason is that of its first EVALUATED variant (documented), which may be another class
    if (!partial) for (std::size_t i = 1; i < search::kRejectReasonCount; ++i) CHECKMSG(r.rejected[i] <= ref.rejected[i], tag + " rejected[] never grows");
    CHECKMSG(r.scope_rejected == exp_scope, tag + " scope_rejected equals the oracle count");
    CHECKMSG(r.evaluator_calls_avoided == exp_avoided, tag + " avoided equals the oracle count");
    { std::size_t sv = 0; for (const auto& sr : r.log) sv += sr.variants_scope_skipped;
      CHECKMSG(r.scope_variants_skipped == sv, tag + " scope_variants_skipped equals the sum over the log");
      CHECKMSG(r.evaluator_calls_avoided == (r.scope_rejected - [&] { std::size_t a = 0; for (const auto& sr : r.log) if (sr.scope_violation != ScopeViolation::None && sr.variants_scope_skipped > 0) ++a; return a; }()) + r.scope_variants_skipped,
               tag + " avoided == scope-rejected non-variant candidates + skipped variants"); }
    // best: the strict-< first-wins best over the passing candidates (exact when no N4 candidate was partially skipped; otherwise a one-sided bound)
    if (r.best) {
        CHECKMSG(M.check(r.best->solution, r.best->move) == ScopeViolation::None, tag + " best passes the mask");
        CHECKMSG(!ref.best || r.best->objective >= ref.best->objective, tag + " scoped best is never better than the unscoped best");
    }
    if (!partial) {
        const model::Solution* want = nullptr; double wo = 0;
        for (std::size_t i = 0; i < ref.log.size(); ++i) {
            const auto& rr = ref.log[i];
            if (!(rr.evaluated && static_cast<int>(rr.reason) == 0)) continue;
            if (r.log[i].scope_violation != ScopeViolation::None) continue;
            if (!want || rr.objective < wo) { want = &rr.solution; wo = rr.objective; }
        }
        CHECKMSG(r.best.has_value() == (want != nullptr), tag + " best exists iff a passing accepted candidate exists");
        if (r.best && want) CHECKMSG(r.best->objective == wo && same_sol(r.best->solution, *want), tag + " best == brute-force oracle (strict <, first wins)");
    }
    // logged evaluated candidates pass the mask
    for (const auto& sr : r.log) if (sr.evaluated && sr.variants_generated == 0) CHECKMSG(M.check(sr.solution, sr.move) == ScopeViolation::None, tag + " every evaluated candidate passes the mask");
    // DM7 cross-check on the scoped best
    if (r.best && region) {
        ++S.dm7_checked; ++S.accepted_scoped;
        bool flagged = false;
        const bool base_clean = check_region_preserved(plan, plan, *region, F.nodes()).empty();   // DM7's own verdict on the unchanged plan
        for (const auto& msg : check_region_preserved(plan, r.best->solution, *region, F.nodes())) {
            // the boundary rules are enforced by the hook, so a mask-accepted best must be DM7-clean whenever DM7 accepts the unchanged plan
            CHECKMSG(!base_clean, tag + " mask-accepted best flagged by DM7 although the hook enforces the boundary: " + msg);
            const bool protected_elem = msg.rfind("protected trip", 0) == 0 || msg.rfind("the set of served customers", 0) == 0 || msg.rfind("empty region: a trip changed", 0) == 0 || msg.rfind("empty region: the trip set", 0) == 0;
            CHECKMSG(!protected_elem, tag + " mask-accepted best violates a protected element: " + msg);
            flagged = true;
        }
        if (flagged) ++S.dm7_flagged_boundary;
    }
    S.scope_cands += static_cast<long>(r.scope_rejected); S.variants_skipped += static_cast<long>(r.scope_variants_skipped); S.avoided += static_cast<long>(r.evaluator_calls_avoided);
}

std::vector<Fixture> make_fixtures() {
    std::vector<Fixture> out;
    for (int seed = 0; seed < 40; ++seed) {
        DynamicConfig c;
        c.regime = 'F'; apply_regime(c);
        c.base.base.instance.num_customers = 9 + seed % 9;
        c.base.base.instance.dataset_index = seed;
        c.base.base.vehicle.num_drones = 2 + seed % 2;
        c.base.vns.iter_max = 100;
        c.base_kind = (seed % 3 == 2) ? BaseKind::Constructive : BaseKind::StaticVns;
        Fixture F;
        F.bp = build_base_plan(c);
        if (!F.bp.ok) { std::printf("fixture %d not built: %s\n", seed, F.bp.error.c_str()); continue; }
        F.label = "seed" + std::to_string(seed) + (c.base_kind == BaseKind::StaticVns ? "-vns" : "-ctor");
        out.push_back(std::move(F));
    }
    return out;
}

std::vector<Cfg> cfgs_for(int op) {
    std::vector<Cfg> v;
    v.push_back(Cfg{});
    if (op == 4) {
        Cfg a; a.pol = 0; v.push_back(a);
        Cfg b; b.cap = 2; b.mode = 0; v.push_back(b);
        Cfg d; d.cap = 7; d.mode = 1; v.push_back(d);
    }
    if (op == 6) { Cfg a; a.same_drone = false; v.push_back(a); }
    if (op == 8) { Cfg a; a.n8w = 1; v.push_back(a); }
    return v;
}

std::vector<std::size_t> anchors_for(int op, const Solution& s, const model::NodeRegistry& nodes) {
    std::vector<std::size_t> a;
    if (op == 4 || op == 7) {
        for (auto v : s.truck_route) if (nodes.contains(v) && nodes.at(v).type == NodeType::Customer) a.push_back(static_cast<std::size_t>(v));
        a.push_back(9999); // not a truck customer: InvalidAnchor
    } else {
        for (std::size_t i = 0; i <= s.drone_trips.size(); ++i) a.push_back(i); // the last one is out of range: InvalidAnchor
    }
    return a;
}

struct MaskCase { std::string name; Region region; ScopeMask mask; bool has_region; };

std::vector<MaskCase> make_masks(const Fixture& F, std::mt19937& g) {
    std::vector<MaskCase> out;
    const Solution& s = F.plan();
    const auto& nodes = F.nodes();
    const PlanLayout L = layout_of(s, nodes);
    std::vector<NodeId> truck;
    for (std::size_t k = 1; k + 1 < L.sk.size(); ++k) if (nodes.contains(L.sk.node[k]) && nodes.at(L.sk.node[k]).type == NodeType::Customer) truck.push_back(L.sk.node[k]);
    auto add = [&](const std::string& name, const RegionBuilder& b, std::size_t cut, NodeId r) {
        Region R = b.build(RegionContext{L, nodes, cut, r});
        if (!R.valid) { ++S.regions_invalid; return; }
        out.push_back(MaskCase{name, R, ScopeMask::from_region(R, s, nodes), true});
    };
    RegionR3 r3a, r3b; r3a.w = 0; r3b.w = 2;
    RegionR4 r4; r4.h = 1;
    RegionR1 r1; r1.k = 2;
    RegionR5 r5;
    const RegionBuilder* bs[] = {&r3a, &r3b, &r4, &r1, &r5};
    for (int q = 0; q < 3 && !truck.empty(); ++q) {
        const NodeId r = truck[g() % truck.size()];
        std::size_t rs = 0; for (std::size_t k = 0; k < L.sk.size(); ++k) if (L.sk.node[k] == r) rs = k;
        const std::size_t cut = (q == 0 || rs <= 1) ? 0 : g() % rs;
        add("R" + std::to_string(q) + "a", *bs[(q * 2) % 5], cut, r);
        add("R" + std::to_string(q) + "b", *bs[(q * 2 + 1) % 5], cut, r);
    }
    if (!truck.empty()) { RegionR3 r3c; r3c.w = 3; add("R3w3", r3c, 0, truck[g() % truck.size()]); add("R5@0", r5, 0, truck[g() % truck.size()]); }
    RegionR0 r0;
    Region E = r0.build(RegionContext{L, nodes, 0, truck.empty() ? 0 : truck[0]});
    if (E.valid) out.push_back(MaskCase{"R0", E, ScopeMask::from_region(E, s, nodes), true});
    return out;
}

void test_operators() {
    const auto fixtures = make_fixtures();
    CHECK(fixtures.size() >= 30);
    std::mt19937 g(20261110);
    long identity_runs = 0, probe_runs = 0;
    for (const auto& F : fixtures) {
        const Solution& plan = F.plan();
        const search::SearchContext base_ctx = F.ctx();
        const auto rctx = ref_ctx(base_ctx);
        ++S.plans; S.trips_total += static_cast<long>(plan.drone_trips.size()); if (!plan.drone_trips.empty()) ++S.plans_with_trips;
        const ScopeMask full = ScopeMask::full();
        const auto masks = make_masks(F, g);
        S.masks += static_cast<long>(masks.size());
        PassAll pass; RejectAll rej;
        const ModHook mod2(2, 1), mod3(3, 0);
        for (int op = 1; op <= 8; ++op) {
            for (const Cfg& cfg : cfgs_for(op)) {
                for (std::size_t anchor : anchors_for(op, plan, F.nodes())) {
                    const std::string tag = F.label + " N" + std::to_string(op) + " a" + std::to_string(anchor) + " pol" + std::to_string(cfg.pol) + "/cap" + std::to_string(cfg.cap) + "/m" + std::to_string(cfg.mode);
                    // ---- reference (frozen pre-hook) ----
                    long c0 = tdr::timing::g_eval_calls;
                    const auto ref = RefApi::by_anchor(op, plan, rctx, anchor, cfg);
                    const long ref_calls = tdr::timing::g_eval_calls - c0;
                    ++S.op_runs; S.ref_calls += ref_calls; S.candidates += static_cast<long>(ref.generated); S.n4_stale += static_cast<long>(ref.stale_launch_candidates);
                    // ---- B1: null scope == reference, evaluator calls included ----
                    c0 = tdr::timing::g_eval_calls;
                    const auto nul = NewApi::by_anchor(op, plan, base_ctx, anchor, cfg);
                    CHECKMSG(tdr::timing::g_eval_calls - c0 == ref_calls, tag + " null scope: evaluator calls identical");
                    CHECKMSG(diff_res(nul, ref).empty(), tag + " null scope: bit-identical (" + diff_res(nul, ref) + ")");
                    CHECKMSG(scope_fields_zero(nul), tag + " null scope: scope counters zero");
                    // ---- B2: full mask == reference ----
                    c0 = tdr::timing::g_eval_calls;
                    const auto fu = NewApi::by_anchor(op, plan, new_ctx(base_ctx, &full), anchor, cfg);
                    CHECKMSG(tdr::timing::g_eval_calls - c0 == ref_calls, tag + " full mask: evaluator calls identical");
                    CHECKMSG(diff_res(fu, ref).empty(), tag + " full mask: bit-identical (" + diff_res(fu, ref) + ")");
                    CHECKMSG(scope_fields_zero(fu), tag + " full mask: scope counters zero");
                    // ---- B3: a hook that passes everything == reference (the hook is consulted, changes nothing) ----
                    pass.calls = 0; c0 = tdr::timing::g_eval_calls;
                    const auto pa = NewApi::by_anchor(op, plan, new_ctx(base_ctx, &pass), anchor, cfg);
                    CHECKMSG(tdr::timing::g_eval_calls - c0 == ref_calls && diff_res(pa, ref).empty() && scope_fields_zero(pa), tag + " pass-all hook: identical to unscoped");
                    // hook is consulted once per evaluator-bound candidate / variant (never for structural rejects)
                    CHECKMSG(pass.calls == static_cast<long>(ref.evaluated), tag + " hook consulted exactly once per evaluator call of the unscoped run");
                    // ---- B4: no evaluation flag: the hook is not consulted and nothing is counted ----
                    { Cfg ne = cfg; ne.eval = false;
                      // unscoped: unchanged behaviour
                      const auto y = RefApi::by_anchor(op, plan, rctx, anchor, ne);
                      const auto x0 = NewApi::by_anchor(op, plan, base_ctx, anchor, ne);
                      CHECKMSG(diff_res(x0, y).empty() && scope_fields_zero(x0), tag + " evaluate_candidates=false, no scope: identical to the reference");
                      // scoped: refused by precondition before any work: no generation, no hook call, zero evaluator calls
                      for (const search::ScopeHook* h : {static_cast<const search::ScopeHook*>(&pass), static_cast<const search::ScopeHook*>(&full)}) {
                          pass.calls = 0; const long e0 = tdr::timing::g_eval_calls;
                          const auto x = NewApi::by_anchor(op, plan, new_ctx(base_ctx, h), anchor, ne);
                          CHECKMSG(x.status == search::OperatorStatus::ScopeRequiresEvaluation && x.generated == 0 && x.evaluated == 0 && !x.best && x.log.empty() && pass.calls == 0 && tdr::timing::g_eval_calls == e0,
                                   tag + " scoped call with evaluate_candidates=false is refused before any work");
                      } }
                    // ---- C1: reject-all hook: the evaluator is called ONLY for the base solution ----
                    rej.calls = 0; c0 = tdr::timing::g_eval_calls;
                    const auto ra = NewApi::by_anchor(op, plan, new_ctx(base_ctx, &rej), anchor, cfg);
                    CHECKMSG(tdr::timing::g_eval_calls - c0 == 1, tag + " reject-all: exactly one evaluator call (the base)");
                    CHECKMSG(ra.evaluated == 0 && ra.accepted == 0 && !ra.best.has_value(), tag + " reject-all: nothing evaluated or accepted");
                    CHECKMSG(ra.generated == ref.generated && ra.accepted + ra.rejected_total() + ra.scope_rejected == ra.generated, tag + " reject-all: accounting");
                    CHECKMSG(ra.evaluator_calls_avoided == ref.evaluated, tag + " reject-all: all evaluator calls avoided");
                    ++probe_runs;
                    // ---- scoped runs under region masks (D, E, F) ----
                    for (const auto& mc : masks) check_scoped(tag + " " + mc.name, op, F, base_ctx, plan, anchor, cfg, ref, mc.mask, mc.has_region ? &mc.region : nullptr);
                    for (const ModHook* h : {&mod2, &mod3}) check_scoped(tag + " modhook", op, F, base_ctx, plan, anchor, cfg, ref, *h, nullptr);
                    ++identity_runs;
                }
            }
        }
        // ---- Rng overloads: identical results AND RNG state for null scope / full mask / any mask (selection consumes the same draws) ----
        for (int op = 1; op <= 8; ++op) {
            for (std::uint64_t seed = 1; seed <= 6; ++seed) {
                const Cfg cfg;
                tdr::util::Rng r0(seed * 7919), r1(seed * 7919), r2(seed * 7919), r3(seed * 7919);
                const auto ref = RefApi::by_rng(op, plan, rctx, r0, cfg);
                const auto nul = NewApi::by_rng(op, plan, base_ctx, r1, cfg);
                const auto fu = NewApi::by_rng(op, plan, new_ctx(base_ctx, &full), r2, cfg);
                CHECKMSG(diff_res(nul, ref).empty() && rng_probe(r0) == rng_probe(r1), F.label + " rng overload N" + std::to_string(op) + ": null scope identical incl. RNG state");
                CHECKMSG(diff_res(fu, ref).empty() && rng_probe(r0) == rng_probe(r2), F.label + " rng overload N" + std::to_string(op) + ": full mask identical incl. RNG state");
                for (const auto& mc : masks) {
                    tdr::util::Rng rm(seed * 7919);
                    const long c0 = tdr::timing::g_eval_calls;
                    const auto sc = NewApi::by_rng(op, plan, new_ctx(base_ctx, &mc.mask), rm, cfg);
                    CHECKMSG(rng_probe(rm) == rng_probe(r0), F.label + " rng overload N" + std::to_string(op) + " " + mc.name + ": RNG consumption unchanged under a scope");
                    CHECKMSG(sc.anchor_customer == ref.anchor_customer && sc.anchor_trip == ref.anchor_trip && sc.generated == ref.generated, F.label + " rng overload: same anchor and candidates");
                    CHECKMSG(tdr::timing::g_eval_calls - c0 == 1 + static_cast<long>(sc.evaluated) || sc.status != search::OperatorStatus::Ok, F.label + " rng overload: scope rejects cost no evaluator call");
                }
            }
        }
    }
    std::printf("operators: %ld anchor/config runs, %ld probe runs\n", identity_runs, probe_runs);
}


// ---- ScopeMask against an independent, naive re-implementation of the rules (from the Region and the base plan only), on random edits ----
ScopeViolation naive_oracle(const Region& R, const Solution& plan, const model::NodeRegistry& nodes, const Solution& cand, const search::Move& m) {
    using V = ScopeViolation;
    auto in = [](const std::vector<NodeId>& v, NodeId x) { return std::find(v.begin(), v.end(), x) != v.end(); };
    auto is_free = [&](NodeId c) { return !R.empty && in(R.free_customers, c) && !in(R.pinned_nodes, c); };
    auto is_cust = [&](NodeId v) { return nodes.contains(v) && nodes.at(v).type == NodeType::Customer; };
    if (m.customer != model::kInvalidNodeId && !is_free(m.customer)) return V::MoveAnchorNotFree;
    if (m.partner != model::kInvalidNodeId && !is_free(m.partner)) return V::MoveAnchorNotFree;
    std::vector<NodeId> prot_truck, prot_drone;
    for (NodeId v : plan.truck_route) if (is_cust(v) && !is_free(v)) prot_truck.push_back(v);
    for (const auto& t : plan.drone_trips) if (!is_free(t.service_customer)) prot_drone.push_back(t.service_customer);
    for (NodeId v : cand.truck_route) if (in(prot_drone, v) && !in(prot_truck, v)) return V::ProtectedDroneCustomerChanged;
    for (NodeId c : prot_truck) {
        if (std::count(cand.truck_route.begin(), cand.truck_route.end(), c) != std::count(plan.truck_route.begin(), plan.truck_route.end(), c)) return V::ProtectedTruckCustomerChanged;
        for (const auto& t : cand.drone_trips) if (t.service_customer == c) return V::ProtectedTruckCustomerChanged;
    }
    for (const auto& pt : plan.drone_trips) {
        if (is_free(pt.service_customer)) continue;
        int n = 0; const DroneTrip* f = nullptr;
        for (const auto& t : cand.drone_trips) if (t.service_customer == pt.service_customer) { ++n; f = &t; }
        if (n != 1 || f->drone_id != pt.drone_id || f->launch_node != pt.launch_node || f->recovery_node != pt.recovery_node) return V::ProtectedDroneCustomerChanged;
        std::size_t lp = cand.truck_route.size();
        for (std::size_t i = 0; i < cand.truck_route.size(); ++i) if (cand.truck_route[i] == pt.launch_node) { lp = i; break; }
        bool ok = false;
        if (lp < cand.truck_route.size()) for (std::size_t i = lp + 1; i < cand.truck_route.size(); ++i) if (cand.truck_route[i] == pt.recovery_node) ok = true;
        if (!ok) return V::ProtectedTripDependencyBroken;
    }
    std::vector<NodeId> po, co;
    if (!R.empty) {
        for (NodeId v : plan.truck_route) if (in(R.pinned_nodes, v)) po.push_back(v);
        for (NodeId v : cand.truck_route) if (in(R.pinned_nodes, v)) co.push_back(v);
        if (co != po) { auto a = co, b = po; std::sort(a.begin(), a.end()); std::sort(b.begin(), b.end()); return a != b ? V::PinnedNodeMissing : V::PinnedOrderChanged; }
    }
    // boundary: prefix through lo / suffix from hi, from the plan's skeleton, compared position by position
    std::size_t P = 0, Sx = 0;
    const auto sk = tdr::search::make_skeleton(plan.truck_route, nodes);
    const bool whole = R.empty || !R.valid || R.hi >= sk.size() || R.lo >= R.hi;
    if (whole) P = plan.truck_route.size(); else { P = sk.route_position[R.lo] + 1; Sx = plan.truck_route.size() - sk.route_position[R.hi]; }
    if (cand.truck_route.size() < P + Sx) return V::ProtectedPrefixChanged;
    for (std::size_t i = 0; i < P; ++i) if (cand.truck_route[i] != plan.truck_route[i]) return V::ProtectedPrefixChanged;
    for (std::size_t i = 0; i < Sx; ++i) if (cand.truck_route[cand.truck_route.size() - Sx + i] != plan.truck_route[plan.truck_route.size() - Sx + i]) return V::ProtectedSuffixChanged;
    const std::size_t endp = cand.truck_route.size() - Sx;
    for (const auto& t : cand.drone_trips) {
        if (!is_free(t.service_customer)) continue;
        std::size_t li = cand.truck_route.size(), ri = cand.truck_route.size();
        for (std::size_t i = 0; i < cand.truck_route.size(); ++i) if (cand.truck_route[i] == t.launch_node) { li = i; break; }
        if (li == cand.truck_route.size() || li < P || li >= endp) return V::FreeTripOutsideRegion;
        for (std::size_t i = li + 1; i < cand.truck_route.size(); ++i) if (cand.truck_route[i] == t.recovery_node) { ri = i; break; }
        if (ri == cand.truck_route.size() || ri > endp) return V::FreeTripOutsideRegion;
    }
    return V::None;
}

void test_mask_fuzz() {
    const auto fixtures = make_fixtures();
    std::mt19937 g(777);
    std::map<int, long> seen;
    long cases = 0;
    for (const auto& F : fixtures) {
        const Solution& plan = F.plan();
        const auto& nodes = F.nodes();
        const auto masks = make_masks(F, g);
        std::vector<NodeId> cust, all_route = plan.truck_route;
        for (NodeId v : plan.truck_route) if (nodes.contains(v) && nodes.at(v).type == NodeType::Customer) cust.push_back(v);
        for (const auto& t : plan.drone_trips) cust.push_back(t.service_customer);
        for (int it = 0; it < 400; ++it) {
            Solution c = plan;
            const int kind = static_cast<int>(g() % 10);
            auto rp = [&](std::size_t n) { return n ? g() % n : 0; };
            switch (kind) {
                case 0: if (c.truck_route.size() > 2) std::swap(c.truck_route[1 + rp(c.truck_route.size() - 2)], c.truck_route[1 + rp(c.truck_route.size() - 2)]); break;
                case 1: if (c.truck_route.size() > 2) c.truck_route.erase(c.truck_route.begin() + 1 + static_cast<std::ptrdiff_t>(rp(c.truck_route.size() - 2))); break;
                case 8: if (c.truck_route.size() > 2) { if (g() % 2) c.truck_route.erase(c.truck_route.begin()); else c.truck_route.pop_back(); } break;
                case 2: if (c.truck_route.size() > 2) { const NodeId v = c.truck_route[1 + rp(c.truck_route.size() - 2)]; c.truck_route.insert(c.truck_route.begin() + 1 + static_cast<std::ptrdiff_t>(rp(c.truck_route.size() - 1)), v); } break;
                case 3: if (!cust.empty()) c.truck_route.insert(c.truck_route.begin() + 1 + static_cast<std::ptrdiff_t>(rp(c.truck_route.size() - 1)), cust[rp(cust.size())]); break;
                case 4: if (!c.drone_trips.empty()) { auto& t = c.drone_trips[rp(c.drone_trips.size())]; const int f = static_cast<int>(g() % 3);
                            if (f == 0) t.drone_id = 1 - t.drone_id; else if (f == 1) t.launch_node = all_route[rp(all_route.size())]; else t.recovery_node = all_route[rp(all_route.size())]; } break;
                case 5: if (!c.drone_trips.empty()) c.drone_trips.erase(c.drone_trips.begin() + static_cast<std::ptrdiff_t>(rp(c.drone_trips.size()))); break;
                case 6: if (!cust.empty()) c.drone_trips.push_back(DroneTrip{0, all_route[rp(all_route.size())], cust[rp(cust.size())], all_route[rp(all_route.size())]}); break;
                case 7: { // legal-looking move of a customer between gaps
                    if (c.truck_route.size() > 3) { const std::size_t a = 1 + rp(c.truck_route.size() - 2); const NodeId v = c.truck_route[a]; c.truck_route.erase(c.truck_route.begin() + static_cast<std::ptrdiff_t>(a));
                        c.truck_route.insert(c.truck_route.begin() + 1 + static_cast<std::ptrdiff_t>(rp(c.truck_route.size() - 1)), v); } break; }
                default: break;
            }
            search::Move m;
            const int mk = static_cast<int>(g() % 4);
            if (mk >= 1 && !cust.empty()) m.customer = cust[rp(cust.size())];
            if (mk >= 2 && !cust.empty()) m.partner = cust[rp(cust.size())];
            for (const auto& mc : masks) {
                const ScopeViolation got = mc.mask.check(c, m), want = naive_oracle(mc.region, plan, nodes, c, m);
                CHECKMSG(got == want, F.label + " " + mc.name + " mask vs naive oracle: " + search::scope_violation_name(got) + " / " + search::scope_violation_name(want));
                ++seen[static_cast<int>(got)]; ++cases;
            }
            CHECK(ScopeMask::full().check(c, m) == ScopeViolation::None);
        }
    }
    for (std::size_t v = 0; v < search::kScopeViolationCount; ++v) CHECKMSG(seen[static_cast<int>(v)] > 0, std::string("mask fuzz reaches verdict ") + search::scope_violation_name(static_cast<ScopeViolation>(v)));
    std::printf("mask fuzz: %ld cases; verdicts:", cases);
    for (std::size_t v = 0; v < search::kScopeViolationCount; ++v) std::printf(" %s=%ld", search::scope_violation_name(static_cast<ScopeViolation>(v)), seen[static_cast<int>(v)]);
    std::printf("\n");
}

// ===================================================== (G) shakes and VNS =====================================================
#define SHK_API(STRUCT, NS)                                                                                                 \
struct STRUCT {                                                                                                             \
    template <class C> static NS::ShakeResult run(int kind, const Solution& b, const C& c, tdr::util::Rng& r) {            \
        switch (kind) {                                                                                                     \
            case 0: return NS::shake_random_reverse(b, c, r);                                                               \
            case 1: return NS::shake_random_exchange(b, c, r);                                                              \
            case 2: return NS::shake_random_shift(b, c, r);                                                                 \
            case 3: return NS::shake_random_drone_remove(b, c, r);                                                          \
            case 4: return NS::shake_reverse(b, c, 1, 2, r);                                                                \
            case 5: return NS::shake_exchange(b, c, 1, 3, 1, r);                                                            \
            case 6: return NS::shake_shift(b, c, 1, 1, 1, r);                                                               \
            case 7: return NS::shake_drone_remove(b, c, 0, r);                                                              \
            default: return NS::shake(b, c, r);                                                                             \
        }                                                                                                                   \
    }                                                                                                                       \
}
SHK_API(NewShk, tdr::search);
SHK_API(RefShk, tdr::search_ref);

template <class A, class B> bool same_shake(const A& a, const B& b) {
    return static_cast<int>(a.kind) == static_cast<int>(b.kind) && static_cast<int>(a.status) == static_cast<int>(b.status) && same_sol(a.solution, b.solution) && a.seg_a == b.seg_a && a.seg_b == b.seg_b &&
           a.seg_len == b.seg_len && a.shift == b.shift && a.drone == b.drone && a.flps_lost_with_arcs == b.flps_lost_with_arcs && a.flps_dropped == b.flps_dropped && a.preserved == b.preserved &&
           a.base_objective == b.base_objective && a.dissolved.size() == b.dissolved.size() && a.removed.size() == b.removed.size() && a.reinsertions.size() == b.reinsertions.size();
}

void test_shakes_and_vns() {
    const auto fixtures = make_fixtures();
    long shake_runs = 0, refused = 0;
    for (const auto& F : fixtures) {
        const Solution& plan = F.plan();
        const search::SearchContext base_ctx = F.ctx();
        const auto rctx = ref_ctx(base_ctx);
        const ScopeMask full = ScopeMask::full();
        std::mt19937 g(5);
        const auto masks = make_masks(F, g);
        for (int kind = 0; kind <= 8; ++kind) {
            for (std::uint64_t seed = 1; seed <= 3; ++seed) {
                tdr::util::Rng a(seed * 104729), b(seed * 104729), c(seed * 104729);
                long c0 = tdr::timing::g_eval_calls;
                const auto ref = RefShk::run(kind, plan, rctx, a);
                const long ref_calls = tdr::timing::g_eval_calls - c0;
                c0 = tdr::timing::g_eval_calls;
                const auto nul = NewShk::run(kind, plan, base_ctx, b);
                CHECKMSG(same_shake(nul, ref) && tdr::timing::g_eval_calls - c0 == ref_calls && rng_probe(a) == rng_probe(b), F.label + " shake " + std::to_string(kind) + ": unscoped identical (result, evaluator calls, RNG)");
                // a full mask is still a scope: shakes have no scope support, so even a full mask is refused (never silently unscoped)
                c0 = tdr::timing::g_eval_calls;
                const auto fu = NewShk::run(kind, plan, new_ctx(base_ctx, &full), c);
                CHECKMSG(fu.status == search::ShakeStatus::InvalidParameters && fu.solution.truck_route.empty() && tdr::timing::g_eval_calls == c0, F.label + " shake " + std::to_string(kind) + ": scoped call refused before any evaluation");
                ++shake_runs; ++refused;
                for (const auto& mc : masks) {
                    tdr::util::Rng d(seed * 104729);
                    c0 = tdr::timing::g_eval_calls;
                    const auto sc = NewShk::run(kind, plan, new_ctx(base_ctx, &mc.mask), d);
                    CHECKMSG(sc.status == search::ShakeStatus::InvalidParameters && tdr::timing::g_eval_calls == c0, F.label + " shake " + std::to_string(kind) + " " + mc.name + ": refused, zero evaluator calls");
                }
            }
        }
        // VNS: unscoped bit-identity (trajectory statistics, best solution, evaluator calls)
        search_ref::VnsOptions ro; search::VnsOptions no;
        ro.iter_max = no.iter_max = 25; ro.base_seed = no.base_seed = 4242; ro.run_index = no.run_index = 3; ro.record_trace = no.record_trace = true;
        long c0 = tdr::timing::g_eval_calls;
        const auto rv = search_ref::run_vns(plan, rctx, ro);
        const long rv_calls = tdr::timing::g_eval_calls - c0;
        c0 = tdr::timing::g_eval_calls;
        const auto nv = search::run_vns(plan, base_ctx, no);
        CHECKMSG(tdr::timing::g_eval_calls - c0 == rv_calls, F.label + " VNS unscoped: evaluator calls identical");
        CHECKMSG(static_cast<int>(rv.status) == static_cast<int>(nv.status) && same_sol(rv.best, nv.best) && same_sol(rv.current, nv.current) && rv.best_objective == nv.best_objective && rv.current_objective == nv.current_objective &&
                     rv.iterations == nv.iterations && rv.local_steps_total == nv.local_steps_total && rv.shakes_applied == nv.shakes_applied && rv.shakes_not_applicable == nv.shakes_not_applicable && rv.shakes_infeasible == nv.shakes_infeasible &&
                     rv.improving_moves == nv.improving_moves && rv.sa_accepted_moves == nv.sa_accepted_moves && rv.best_updates == nv.best_updates && rv.trace.size() == nv.trace.size(), F.label + " VNS unscoped: identical trajectory");
        for (std::size_t o = 0; o < 8; ++o) CHECKMSG(rv.ops[o].calls == nv.ops[o].calls && rv.ops[o].improving == nv.ops[o].improving && rv.ops[o].sa_accepted == nv.ops[o].sa_accepted && rv.ops[o].no_candidate == nv.ops[o].no_candidate && rv.ops[o].no_anchor == nv.ops[o].no_anchor, F.label + " VNS ops stats identical");
        // VNS under a scope: shakes are refused, so no shake is applied and the best stays the input (documented DM9 dependency: no scoped VNS)
        search::VnsOptions so = no; so.iter_max = 4; so.record_trace = false;
        const auto sv = search::run_vns(plan, new_ctx(base_ctx, &full), so);
        CHECKMSG(sv.shakes_applied == 0 && sv.best_objective == sv.initial_objective, F.label + " VNS with a scoped context applies no shake and keeps the input");
    }
    std::printf("shakes: %ld runs (%ld refused under scope)\n", shake_runs, refused);
}

} // namespace

int main() {
    test_mask_units();
    test_mask_fuzz();
    test_operators();
    test_shakes_and_vns();
    std::printf("fixtures: %ld plans (%ld with drone trips, %ld trips total), %ld masks (%ld invalid regions skipped)\n", S.plans, S.plans_with_trips, S.trips_total, S.masks, S.regions_invalid);
    std::printf("unscoped reference: %ld operator runs, %ld candidates, %ld evaluator calls, %ld N4 candidates needing re-anchoring\n", S.op_runs, S.candidates, S.ref_calls, S.n4_stale);
    std::printf("scoped: %ld candidates scope-rejected, %ld N4 variants skipped (%ld candidates partially, %ld fully skipped), %ld evaluator calls avoided\n", S.scope_cands, S.variants_skipped, S.partial_variant, S.allskip_variant, S.avoided);
    for (const auto& kv : S.by_violation) std::printf("  violation %-34s %ld\n", kv.first.c_str(), kv.second);
    std::printf("boundary rejections before evaluation (prefix/suffix/free-trip-outside), per operator, N1..N8:");
    for (int o = 1; o <= 8; ++o) std::printf(" N%d=%ld", o, S.boundary_cands[o]);
    std::printf("; a rejected candidate followed by an evaluator call: %ld\n", S.spy_bad);
    CHECK(S.spy_bad == 0);
    CHECK(S.boundary_cands[4] > 0 && S.boundary_cands[8] > 0);   // regression: boundary-crossing N4 and N8 candidates exist in the fixtures and are all rejected pre-evaluation
    std::printf("DM7 cross-check: %ld scoped bests checked, %ld flagged by DM7 (must be 0 when DM7 accepts the unchanged plan), %ld scoped bests found\n", S.dm7_checked, S.dm7_flagged_boundary, S.accepted_scoped);
    std::printf("%d checks, %d failures\n", g_checks, g_fail);
    if (g_fail == 0) std::printf("ALL TESTS PASSED\n");
    return g_fail == 0 ? 0 : 1;
}
