// DM8 gate G5 (INFORMATIONAL, not a test; wall-clock is machine dependent): cost of the unscoped hook path (null scope) vs the frozen pre-hook copy,
// and evaluator calls / time saved by a region mask. Not registered in CMake. Build: g++ -O2 -I include -I tests/reference tests/bench_dyn_scope.cpp tsp.a
#include <chrono>
#include <cstdio>
#include "search_pre_hook/neighbourhood_n1.hpp"
#include "search_pre_hook/neighbourhood_n2.hpp"
#include "search_pre_hook/neighbourhood_n3.hpp"
#include "search_pre_hook/neighbourhood_n4.hpp"
#include "search_pre_hook/neighbourhood_n5.hpp"
#include "search_pre_hook/neighbourhood_n6.hpp"
#include "search_pre_hook/neighbourhood_n7.hpp"
#include "search_pre_hook/neighbourhood_n8.hpp"
#include "search/neighbourhood_n1.hpp"
#include "search/neighbourhood_n2.hpp"
#include "search/neighbourhood_n3.hpp"
#include "search/neighbourhood_n4.hpp"
#include "search/neighbourhood_n5.hpp"
#include "search/neighbourhood_n6.hpp"
#include "search/neighbourhood_n7.hpp"
#include "search/neighbourhood_n8.hpp"
#include "dynamic/base_plan.hpp"
#include "dynamic/scope_mask.hpp"
using namespace tdr; using namespace tdr::dynamic;
using Clock = std::chrono::steady_clock;

#define DEFINE_SWEEP(NAME, NS, CTX)                                                                                                                                  \
std::size_t NAME(const model::Solution& s, const CTX& c, const model::NodeRegistry& nodes, std::size_t& evals, std::size_t& gen) {                                    \
    std::size_t acc = 0;                                                                                                                                              \
    auto take = [&](const auto& r) { evals += r.evaluated; gen += r.generated; acc += r.accepted; };                                                                  \
    for (std::size_t t = 0; t < s.drone_trips.size(); ++t) {                                                                                                          \
        take(NS::n1_launch_node_relocate(s, c, t)); take(NS::n2_recover_node_relocate(s, c, t)); take(NS::n3_drone_service_node_relocate(s, c, t));                        \
        take(NS::n5_td_service_node_swap(s, c, t)); take(NS::n6_drone_drone_swap(s, c, t)); take(NS::n8_drone_node_delete_truck_insert(s, c, t));                          \
    }                                                                                                                                                                 \
    for (auto v : s.truck_route) if (nodes.contains(v) && nodes.at(v).type == model::NodeType::Customer) { take(NS::n4_truck_service_node_relocate(s, c, v)); take(NS::n7_truck_node_delete_drone_insert(s, c, v)); } \
    return acc;                                                                                                                                                       \
}
DEFINE_SWEEP(sweep_ref, tdr::search_ref, tdr::search_ref::SearchContext)
DEFINE_SWEEP(sweep_new, tdr::search, tdr::search::SearchContext)

int main() {
    BasePlan B;   // first seed whose plan has several drone trips (the static VNS often ends with none at n=30)
    for (int seed = 0; seed < 30 && !(B.ok && B.plan->solution().drone_trips.size() >= 4); ++seed) {
        DynamicConfig cfg; cfg.regime = 'F'; apply_regime(cfg);
        cfg.base.base.instance.num_customers = 30; cfg.base.base.instance.dataset_index = seed; cfg.base.vns.iter_max = 300; cfg.base.base.vehicle.num_drones = 3;
        cfg.base_kind = BaseKind::Constructive;
        B = build_base_plan(cfg);
    }
    if (!B.ok) { std::printf("fixture failed: %s\n", B.error.c_str()); return 1; }
    const auto& s = B.plan->solution(); const auto& nodes = B.plan->world().nodes();
    const search::SearchContext ctx = B.plan->world().context();
    const search_ref::SearchContext rctx{ctx.network, ctx.customers, ctx.flp_data, ctx.flp_index, ctx.adjacency_window};
    std::printf("plan: n=30, %zu drone trips, %zu route nodes\n", s.drone_trips.size(), s.truck_route.size());
    double tr = 0, tn = 0; std::size_t er = 0, en = 0, gr = 0, gn = 0, a1 = 0, a2 = 0;
    for (int rep = 0; rep < 6; ++rep) {          // alternate the order to cancel warm-up / frequency effects
        for (int k = 0; k < 2; ++k) {
            const bool ref_first = (rep % 2 == 0) == (k == 0);
            const auto t0 = Clock::now();
            if (ref_first) a1 = sweep_ref(s, rctx, nodes, er = 0, gr = 0); else a2 = sweep_new(s, ctx, nodes, en = 0, gn = 0);
            const double dt = std::chrono::duration<double>(Clock::now() - t0).count();
            (ref_first ? tr : tn) += dt;
        }
    }
    std::printf("unscoped sweep (all anchors, N1-N8): reference %.3f s, hooked/null-scope %.3f s, ratio %.3f; evaluator calls %zu vs %zu, candidates %zu vs %zu, accepted %zu vs %zu\n", tr, tn, tn / tr, er, en, gr, gn, a1, a2);
    // scoped sweep: R3(w=1) region around a middle customer
    std::vector<model::NodeId> cust; for (auto v : s.truck_route) if (nodes.contains(v) && nodes.at(v).type == model::NodeType::Customer) cust.push_back(v);
    const PlanLayout L = layout_of(s, nodes); RegionR3 r3; r3.w = 1;
    const Region R = r3.build(RegionContext{L, nodes, 0, cust[cust.size() / 2]});
    const ScopeMask M = ScopeMask::from_region(R, s, nodes);
    search::SearchContext sc = ctx; sc.scope = &M;
    std::size_t es = 0, gs = 0; const auto t0 = Clock::now(); const std::size_t a3 = sweep_new(s, sc, nodes, es, gs); const double ts = std::chrono::duration<double>(Clock::now() - t0).count();
    std::printf("scoped sweep over ALL anchors (region: %zu free customers of %zu): %.3f s (one pass) vs %.3f s unscoped; evaluator calls %zu vs %zu per pass; accepted %zu\n", M.free_customers().size(), cust.size() + s.drone_trips.size(), ts, tn / 6, es, en, a3);
    return 0;
}
