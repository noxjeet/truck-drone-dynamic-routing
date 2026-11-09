// M6 oracle test. For seeded random instances and random-walk bases (feasible shake results and N1/N3/N4/N5/N7 moves, so trips of
// many shapes occur) every shake is compared EXACTLY (route, trips, dissolve/removal records) with an independent re-implementation
// written here from docs/static/shakes.md: own skeleton arithmetic, FLPs of an arc re-derived from the FlpData map in base-route
// order, own validity test, own free-gap / fallback reinsertion. The documented RNG draw orders are replayed on a fresh Rng; the
// production code's helpers (make_skeleton, drop_unused_flps, rebuild, settle) are not used. Verdicts come from the FROZEN
// reference evaluator (tdr::timing_ref).
#include "config/static_config.hpp"
#include "flp/flp_arc_index.hpp"
#include "init/initial_solution_builder.hpp"
#include "instance/static_instance.hpp"
#include "reference/timing_engine_reference.hpp"
#include "search/neighbourhood_n1.hpp"
#include "search/neighbourhood_n3.hpp"
#include "search/neighbourhood_n4.hpp"
#include "search/neighbourhood_n5.hpp"
#include "search/neighbourhood_n7.hpp"
#include "search/shakes.hpp"
#include "util/rng.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace tdr::model;
using namespace tdr::search;

namespace {

int g_failures = 0;
long g_checks = 0;
void check(bool ok, const std::string& what) {
    ++g_checks;
    if (!ok) { if (g_failures < 40) std::cout << "[FAIL] " << what << "\n"; ++g_failures; }
}

struct World {
    const NodeRegistry* nodes;
    const tdr::graph::SpatialNetwork* net;
    const std::unordered_map<NodeId, CustomerData>* cd;
    const tdr::timing::FlpDataMap* flp;
    int U;
};
bool is_flp(const World& w, NodeId v) { return w.nodes->at(v).type == NodeType::FlexibleLaunchPoint; }

struct Expect {
    bool applicable = true;
    TruckRoute route;
    std::vector<DroneTrip> trips;
    std::vector<std::pair<NodeId, int>> dissolved; // (customer, reason as int)
    std::vector<NodeId> removed;
    std::vector<NodeId> dropped, lost;
    std::vector<std::pair<std::size_t, bool>> reins; // (free_gaps, fallback)
    std::vector<NodeId> reins_after;
};

// Reinsertion + validity, independent. `route` = skeleton with each gap's FLPs; trips = all input trips (with origin index)
Expect finish(const World& w, std::vector<NodeId> sk, const Solution& base, int remove_drone, tdr::util::Rng& rng) {
    Expect x;
    // FLPs by directed arc, in base-route order
    std::map<std::pair<NodeId, NodeId>, std::vector<NodeId>> arcs;
    for (NodeId v : base.truck_route) if (is_flp(w, v)) { const auto& d = w.flp->at(v); arcs[{d.arc_from, d.arc_to}].push_back(v); }
    // gaps: vector of FLP lists per gap
    std::vector<std::vector<NodeId>> gap(sk.size() - 1);
    std::set<NodeId> alive;
    for (std::size_t g = 0; g + 1 < sk.size(); ++g) {
        auto it = arcs.find({sk[g], sk[g + 1]});
        if (it != arcs.end()) { gap[g] = it->second; for (NodeId f : it->second) alive.insert(f); }
    }
    for (NodeId v : base.truck_route) if (is_flp(w, v) && !alive.count(v)) x.lost.push_back(v);
    auto position = [&](NodeId v, std::size_t& skel_idx, bool& is_f, std::size_t& gap_idx, std::size_t& in_gap) {
        for (std::size_t g = 0; g < sk.size(); ++g) if (sk[g] == v) { skel_idx = g; is_f = false; return true; }
        for (std::size_t g = 0; g < gap.size(); ++g) for (std::size_t k = 0; k < gap[g].size(); ++k) if (gap[g][k] == v) { gap_idx = g; in_gap = k; is_f = true; return true; }
        return false;
    };
    // order key of a node in the route: 2*skeleton index for skeleton nodes; 2*g+1 + k/1000 for gap FLPs
    auto key = [&](NodeId v, double& out) {
        std::size_t si = 0, gi = 0, ik = 0; bool f = false;
        if (!position(v, si, f, gi, ik)) return false;
        out = f ? 2.0 * static_cast<double>(gi) + 1.0 + static_cast<double>(ik + 1) / 1000.0 : 2.0 * static_cast<double>(si);
        return true;
    };
    std::vector<DroneTrip> trips = base.drone_trips;
    std::vector<bool> gone(trips.size(), false);
    std::vector<std::size_t> origin(trips.size());
    for (std::size_t i = 0; i < origin.size(); ++i) origin[i] = i;
    std::vector<NodeId> queue;
    std::vector<std::size_t> qorigin;
    for (std::size_t i = 0; i < trips.size(); ++i) {
        if (remove_drone >= 0) {
            if (trips[i].drone_id == remove_drone) { gone[i] = true; x.removed.push_back(trips[i].service_customer); queue.push_back(trips[i].service_customer); }
            continue;
        }
        double kl = 0, kr = 0;
        const bool l_ok = key(trips[i].launch_node, kl);
        int why = -1;
        if (!l_ok) why = 0;
        else {
            // the recovery must occur strictly after the launch; the depot occurs at both ends of the skeleton
            bool after = false;
            if (trips[i].recovery_node == sk.back()) { kr = 2.0 * static_cast<double>(sk.size() - 1); after = kr > kl; }
            if (!after && key(trips[i].recovery_node, kr)) after = kr > kl;
            if (!after) why = 1;
        }
        if (why >= 0) { gone[i] = true; x.dissolved.push_back({trips[i].service_customer, why}); queue.push_back(trips[i].service_customer); }
    }
    auto drop_idle = [&]() {
        for (std::size_t g = 0; g < gap.size(); ++g) {
            std::vector<NodeId> keep;
            for (NodeId f : gap[g]) {
                bool used = false;
                for (std::size_t i = 0; i < trips.size(); ++i) used = used || (!gone[i] && trips[i].launch_node == f);
                if (used) keep.push_back(f); else x.dropped.push_back(f);
            }
            gap[g] = keep;
        }
    };
    drop_idle();
    for (std::size_t qi = 0; qi < queue.size(); ++qi) {
        std::vector<std::size_t> free;
        for (std::size_t g = 0; g < gap.size(); ++g) if (gap[g].empty()) free.push_back(g);
        std::size_t g;
        bool fb = false;
        const std::size_t nfree = free.size();
        if (!free.empty()) g = free[rng.uniform_index(free.size())];
        else {
            fb = true;
            g = rng.uniform_index(gap.size());
            for (std::size_t i = 0; i < trips.size(); ++i) {
                if (gone[i]) continue;
                if (std::find(gap[g].begin(), gap[g].end(), trips[i].launch_node) == gap[g].end()) continue;
                gone[i] = true;
                x.dissolved.push_back({trips[i].service_customer, 2});
                queue.push_back(trips[i].service_customer);
            }
            for (NodeId f : gap[g]) x.dropped.push_back(f);
            gap[g].clear();
        }
        x.reins.push_back({nfree, fb});
        x.reins_after.push_back(sk[g]);
        sk.insert(sk.begin() + static_cast<std::ptrdiff_t>(g + 1), queue[qi]);
        gap.insert(gap.begin() + static_cast<std::ptrdiff_t>(g + 1), std::vector<NodeId>{});
    }
    for (std::size_t g = 0; g < sk.size(); ++g) {
        x.route.push_back(sk[g]);
        if (g < gap.size()) for (NodeId f : gap[g]) x.route.push_back(f);
    }
    for (std::size_t i = 0; i < trips.size(); ++i) if (!gone[i]) x.trips.push_back(trips[i]);
    return x;
}

bool same_trips(const std::vector<DroneTrip>& a, const std::vector<DroneTrip>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (a[i].drone_id != b[i].drone_id || a[i].launch_node != b[i].launch_node || a[i].service_customer != b[i].service_customer || a[i].recovery_node != b[i].recovery_node) return false;
    return true;
}

struct Stats {
    long bases = 0, bases_with_trips = 0, calls[4] = {0, 0, 0, 0}, ok[4] = {0, 0, 0, 0}, na[4] = {0, 0, 0, 0};
    long dissolved[3] = {0, 0, 0}, removed = 0, reins = 0, fallbacks = 0, no_dissolve = 0, with_dissolve = 0, feasible = 0, infeasible = 0, endurance = 0, flp_drops = 0, flp_lost = 0;
    long same_objective = 0, improved = 0;
    std::uint64_t digest = 14695981039346656037ULL;
    void mix(std::uint64_t v) { digest = (digest ^ v) * 1099511628211ULL; }
};

std::vector<NodeId> skeleton_of(const World& w, const TruckRoute& r) {
    std::vector<NodeId> s;
    for (NodeId v : r) if (!is_flp(w, v)) s.push_back(v);
    return s;
}

void one(const World& w, const SearchContext& ctx, const Solution& base, ShakeKind kind, std::uint64_t seed, Stats& st, const std::string& tag) {
    const ShakeOptions opt;
    tdr::util::Rng prod(seed), rep(seed);
    ShakeResult r;
    switch (kind) {
        case ShakeKind::Reverse: r = shake_random_reverse(base, ctx, prod, opt); break;
        case ShakeKind::Exchange: r = shake_random_exchange(base, ctx, prod, opt); break;
        case ShakeKind::Shift: r = shake_random_shift(base, ctx, prod, opt); break;
        default: r = shake_random_drone_remove(base, ctx, prod, opt); break;
    }
    ++st.calls[static_cast<int>(kind)];
    const auto sk0 = skeleton_of(w, base.truck_route);
    const long m = static_cast<long>(sk0.size()) - 2;
    std::vector<NodeId> mid(sk0.begin() + 1, sk0.end() - 1);
    int remove_drone = -1;
    bool applicable = true;
    if (kind == ShakeKind::DroneRemove) {
        std::set<int> ids;
        for (const auto& t : base.drone_trips) ids.insert(t.drone_id);
        if (ids.empty()) applicable = false;
        else { auto it = ids.begin(); std::advance(it, static_cast<long>(rep.uniform_index(ids.size()))); remove_drone = *it; }
    } else if (m < 2) applicable = false;
    else if (kind == ShakeKind::Reverse) {
        const long x = static_cast<long>(rep.uniform_index(static_cast<std::size_t>(m)));
        long y = static_cast<long>(rep.uniform_index(static_cast<std::size_t>(m - 1)));
        if (y >= x) ++y;
        std::reverse(mid.begin() + std::min(x, y), mid.begin() + std::max(x, y) + 1);
    } else if (kind == ShakeKind::Exchange) {
        const long L = rep.uniform_int(1, std::min<long>(3, m / 2));
        std::vector<std::pair<long, long>> all; // lexicographic (a, b), 1-based
        for (long a = 1; a + 2 * L - 1 <= m; ++a) for (long b = a + L; b + L - 1 <= m; ++b) all.push_back({a, b});
        const auto ab = all[rep.uniform_index(all.size())];
        for (long k = 0; k < L; ++k) std::swap(mid[static_cast<std::size_t>(ab.first - 1 + k)], mid[static_cast<std::size_t>(ab.second - 1 + k)]);
    } else {
        const long L = rep.uniform_int(1, std::min<long>(3, m - 1));
        const long a = rep.uniform_int(1, m - L + 1);
        std::vector<int> dirs;
        if (a > 1) dirs.push_back(-1);
        if (a + L - 1 < m) dirs.push_back(1);
        const int dir = dirs[rep.uniform_index(dirs.size())];
        const long room = dir < 0 ? a - 1 : m - (a + L - 1);
        const long d = rep.uniform_int(1, std::min<long>(3, room));
        // move by explicit removal and re-insertion
        std::vector<NodeId> seg(mid.begin() + (a - 1), mid.begin() + (a - 1 + L));
        std::vector<NodeId> rest;
        for (long i = 0; i < m; ++i) if (i < a - 1 || i >= a - 1 + L) rest.push_back(mid[static_cast<std::size_t>(i)]);
        const long at = (dir > 0 ? a - 1 + d : a - 1 - d);
        rest.insert(rest.begin() + at, seg.begin(), seg.end());
        mid = rest;
    }
    if (!applicable) {
        ++st.na[static_cast<int>(kind)];
        check(r.status == ShakeStatus::NotApplicable, tag + ": not applicable expected");
        check(prod.next_u64() == rep.next_u64(), tag + ": a NotApplicable shake consumes exactly the draws before the decision");
        return;
    }
    ++st.ok[static_cast<int>(kind)];
    std::vector<NodeId> sk;
    sk.push_back(sk0.front());
    sk.insert(sk.end(), mid.begin(), mid.end());
    sk.push_back(sk0.back());
    const Expect x = finish(w, sk, base, remove_drone, rep);
    check(r.status == ShakeStatus::Ok, tag + ": status Ok");
    if (r.status != ShakeStatus::Ok) return;
    if (r.solution.truck_route != x.route && g_failures == 0) {
        auto pr = [](const char* n, const TruckRoute& t) { std::cout << n; for (NodeId v : t) std::cout << " " << v; std::cout << "\n"; };
        pr("base ", base.truck_route); pr("prod ", r.solution.truck_route); pr("orac ", x.route);
        for (const auto& t : base.drone_trips) std::cout << "trip d" << t.drone_id << " " << t.launch_node << " " << t.service_customer << " " << t.recovery_node << "\n";
    }
    check(r.solution.truck_route == x.route, tag + ": route equals the oracle's");
    check(same_trips(r.solution.drone_trips, x.trips), tag + ": trips equal the oracle's (order, drone ids, launch, customer, recovery)");
    check(prod.next_u64() == rep.next_u64(), tag + ": the same number of draws is consumed");
    bool rec = r.dissolved.size() == x.dissolved.size() && r.removed.size() == x.removed.size() && r.reinsertions.size() == x.reins.size();
    for (std::size_t i = 0; rec && i < x.dissolved.size(); ++i) rec = r.dissolved[i].customer == x.dissolved[i].first && static_cast<int>(r.dissolved[i].reason) == x.dissolved[i].second;
    for (std::size_t i = 0; rec && i < x.removed.size(); ++i) rec = r.removed[i].customer == x.removed[i];
    for (std::size_t i = 0; rec && i < x.reins.size(); ++i) rec = r.reinsertions[i].free_gaps == x.reins[i].first && r.reinsertions[i].fallback == x.reins[i].second && r.reinsertions[i].after == x.reins_after[i];
    check(rec, tag + ": dissolution, removal and reinsertion records equal the oracle's (customers, reasons, free-gap counts, fallback flags, anchors)");
    check(r.flps_dropped == x.dropped && r.flps_lost_with_arcs == x.lost, tag + ": FLP accounting equals the oracle's");
    check(std::fabs(r.base_objective - tdr::timing_ref::evaluate_solution(base, *w.net, *w.cd, *w.flp).objective_value) <= 1e-6 * std::max(1.0, r.base_objective), tag + ": base objective equals the frozen reference's");
    // independent evaluation
    const auto e = tdr::timing_ref::evaluate_solution(r.solution, *w.net, *w.cd, *w.flp);
    check(r.evaluated && r.feasible == e.feasible && (!e.feasible || std::fabs(r.objective - e.objective_value) <= 1e-6 * std::max(1.0, e.objective_value)), tag + ": verdict and objective equal the frozen reference's");
    check(!r.evaluated || (r.feasible == (r.reason == RejectReason::None)), tag + ": a feasible result has reason None, an infeasible one an evaluator reason");
    // coverage
    std::set<NodeId> seen;
    bool once = true;
    for (NodeId v : r.solution.truck_route) if (w.nodes->at(v).type == NodeType::Customer) once = once && seen.insert(v).second;
    for (const auto& t : r.solution.drone_trips) once = once && seen.insert(t.service_customer).second;
    check(once && seen.size() == w.cd->size(), tag + ": every customer served exactly once");
    // stats
    for (const auto& d : x.dissolved) ++st.dissolved[d.second];
    st.removed += static_cast<long>(x.removed.size());
    st.reins += static_cast<long>(x.reins.size());
    for (const auto& q : x.reins) st.fallbacks += q.second;
    (x.dissolved.empty() && x.removed.empty()) ? ++st.no_dissolve : ++st.with_dissolve;
    st.flp_drops += static_cast<long>(x.dropped.size());
    st.flp_lost += static_cast<long>(x.lost.size());
    if (e.feasible) { ++st.feasible; if (e.objective_value < r.base_objective - 1e-9) ++st.improved; if (std::fabs(e.objective_value - r.base_objective) <= 1e-9) ++st.same_objective; }
    else { ++st.infeasible; if (r.reason == RejectReason::EvalEndurance) ++st.endurance; }
    st.mix(static_cast<std::uint64_t>(seed)); st.mix(static_cast<std::uint64_t>(kind));
    for (NodeId v : r.solution.truck_route) st.mix(static_cast<std::uint64_t>(v));
    for (const auto& t : r.solution.drone_trips) { st.mix(static_cast<std::uint64_t>(t.drone_id)); st.mix(static_cast<std::uint64_t>(t.launch_node)); st.mix(static_cast<std::uint64_t>(t.recovery_node)); }
    st.mix(r.feasible ? 1 : 0);
}

Stats campaign(int seeds) {
    Stats st;
    for (int seed = 0; seed < seeds; ++seed) {
        tdr::util::Rng fz(tdr::util::derive_seed(6272, tdr::util::SeedStream::Fuzz, static_cast<std::uint64_t>(seed)));
        tdr::config::StaticConfig cfg;
        cfg.seed.base_seed = 6272;
        cfg.instance.num_customers = static_cast<int>(fz.uniform_int(6, 14));
        cfg.instance.dataset_index = seed;
        cfg.vehicle.num_drones = static_cast<int>(fz.uniform_int(seed % 5 == 0 ? 1 : 2, 3));
        switch (seed % 4) {
            case 1: cfg.vehicle.endurance_s *= 3.0; break;
            case 2: cfg.instance.area_side_m = 15000.0; break;
            case 3: cfg.instance.area_side_m = 15000.0; cfg.vehicle.endurance_s *= 3.0; break;
            default: break;
        }
        const auto gen = tdr::instance::generate_static_instance(cfg);
        check(gen.ok, "instance generation");
        if (!gen.ok) continue;
        const auto& inst = gen.instance;
        const auto net0 = inst.make_network();
        tdr::init::InitialSolutionPolicy pol;
        pol.flp_policy.unit_division_length = cfg.flp.unit_division_length_m;
        const auto init = tdr::init::build_initial_solution(inst.depot, inst.customers, inst.customer_data, net0, pol);
        if (!init.feasible) continue;
        const tdr::graph::SpatialNetwork net(init.node_registry, inst.truck, inst.drone);
        const tdr::flp::FlpArcIndex idx(init.flp_data);
        const World w{&init.node_registry, &net, &inst.customer_data, &init.flp_data, cfg.vehicle.num_drones};
        const SearchContext ctx{net, inst.customer_data, init.flp_data, idx, 3};

        Solution cur = init.solution;
        for (int step = 0; step < 10; ++step) {
            ++st.bases;
            if (!cur.drone_trips.empty()) ++st.bases_with_trips;
            const std::string tag = "seed " + std::to_string(seed) + " step " + std::to_string(step);
            for (int k = 0; k < 4; ++k)
                for (std::uint64_t s = 1; s <= 12; ++s) one(w, ctx, cur, static_cast<ShakeKind>(k), s * 1009 + static_cast<std::uint64_t>(seed * 31 + step), st, tag + " kind " + std::to_string(k) + " s" + std::to_string(s));
            {   // shake(): first draw picks the kind
                tdr::util::Rng a(seed * 13 + step), b(seed * 13 + step);
                const auto r = shake(cur, ctx, a);
                const std::size_t kk = b.uniform_index(4);
                check(static_cast<std::size_t>(r.kind) == kk, tag + ": shake() kind equals the first uniform_index(4) draw");
            }
            // advance the base: feasible shake result, or an N-move, so that bases of many shapes occur
            tdr::util::Rng walk(tdr::util::derive_seed(6273, tdr::util::SeedStream::Fuzz, static_cast<std::uint64_t>(seed * 100 + step)));
            const double pick = walk.uniform01();
            if (pick < 0.45) {
                const auto r = shake(cur, ctx, walk);
                if (r.status == ShakeStatus::Ok && r.feasible) cur = r.solution;
                continue;
            }
            OperatorResult r;
            if (!cur.drone_trips.empty() && pick < 0.6) { N1Options o; o.record_candidates = true; r = n1_launch_node_relocate(cur, ctx, walk, o); }
            else if (!cur.drone_trips.empty() && pick < 0.72) { N3Options o; o.record_candidates = true; r = n3_drone_service_node_relocate(cur, ctx, walk, o); }
            else if (!cur.drone_trips.empty() && pick < 0.82) { N5Options o; o.record_candidates = true; r = n5_td_service_node_swap(cur, ctx, walk, o); }
            else if (pick < 0.92) { N7Options o; o.record_candidates = true; r = n7_truck_node_delete_drone_insert(cur, ctx, walk, o); }
            else { N4Options o; o.record_candidates = true; r = n4_truck_service_node_relocate(cur, ctx, walk, o); }
            std::vector<const CandidateRecord*> acc;
            for (const auto& rec : r.log) if (rec.reason == RejectReason::None) acc.push_back(&rec);
            if (!acc.empty()) cur = acc[walk.uniform_index(acc.size())]->solution;
        }
    }
    return st;
}

} // namespace

int main() {
    constexpr int kSeeds = 80;
    const Stats a = campaign(kSeeds);
    const Stats b = campaign(kSeeds);
    const char* names[4] = {"Reverse", "Exchange", "Shift", "DroneRemove"};
    for (int k = 0; k < 4; ++k) std::cout << names[k] << ": calls=" << a.calls[k] << " ok=" << a.ok[k] << " not_applicable=" << a.na[k] << "\n";
    std::cout << "bases=" << a.bases << " with_trips=" << a.bases_with_trips << "\n"
              << "dissolved: launch-arc-vanished=" << a.dissolved[0] << " recovery-not-after-launch=" << a.dissolved[1] << " arc-split-by-reinsertion=" << a.dissolved[2] << " removed_by_S4=" << a.removed << "\n"
              << "reinsertions=" << a.reins << " fallback_draws=" << a.fallbacks << " shakes_without_dissolution=" << a.no_dissolve << " with=" << a.with_dissolve << "\n"
              << "flps_dropped_idle=" << a.flp_drops << " flps_lost_with_arcs=" << a.flp_lost << "\n"
              << "results: feasible=" << a.feasible << " infeasible=" << a.infeasible << " (endurance=" << a.endurance << ") improved=" << a.improved << " equal_objective=" << a.same_objective << "\n";

    check(a.bases > 500 && a.bases_with_trips > 300, "non-vacuity: enough bases, many with trips");
    for (int k = 0; k < 4; ++k) check(a.ok[k] > 1000, std::string("non-vacuity: ") + names[k] + " acts often");
    check(a.na[3] > 0, "non-vacuity: Random_Drone_Remove NotApplicable (no trip) occurs");
    check(a.dissolved[0] > 200 && a.dissolved[1] > 20, "non-vacuity: launch-arc-vanished and recovery-not-after-launch dissolutions occur");
    check(a.dissolved[2] > 0 && a.fallbacks > 0, "non-vacuity: the no-free-gap fallback (ArcSplitByReinsertion) occurs in random walks");
    check(a.removed > 300 && a.reins > 1000, "non-vacuity: S4 removals and reinsertions occur");
    check(a.no_dissolve > 500 && a.with_dissolve > 500, "non-vacuity: shakes with and without any dissolution occur");
    check(a.feasible > 500 && a.infeasible > 100 && a.endurance > 0, "non-vacuity: feasible and infeasible (incl. endurance) results occur");
    check(a.flp_drops > 200 && a.flp_lost > 200, "non-vacuity: idle FLP drops and FLPs lost with their arcs occur");
    check(a.digest == b.digest, "deterministic: identical digest over the whole campaign on a second full run (oracle on both)");

    std::cout << g_checks << " checks, " << g_failures << " failures\n";
    if (g_failures == 0) std::cout << "ALL TESTS PASSED\n";
    return g_failures == 0 ? 0 : 1;
}
