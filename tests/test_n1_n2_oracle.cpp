// M5 oracle test. For seeded random instances (and random-walk neighbours of their initial solutions, so that trips of many
// shapes occur) N1 (launch node relocate) and N2 (recover node relocate) are compared RECORD BY RECORD with an independent
// brute-force enumeration written here with its own route/skeleton/gap arithmetic. It calls none of the production helpers
// (not detail_relocate, not flp_insertion_position, not remove_from_route, not trips_launching_from, not gaps_within_window,
// not FlpArcIndex::flps_on_arc: the FLPs of an arc are re-derived from the FlpData map), and is judged by the FROZEN
// reference evaluator (tdr::timing_ref).
//
// Oracle contract (docs/static/operators_N1_N2.md):
//   Skeleton = the route's non-FLP nodes s0 (depot) .. s_last (end depot); gap g lies between s_g and s_{g+1}.
//   N1, trip i with launch FLP in gap g0, window w: candidate gaps g = max(0,g0-w) .. min(last-1,g0+w) ascending; in each gap
//       every FLP of the arc (s_g, s_{g+1}) (from the FlpData map) in (position_ratio, id) order except the current launch;
//       candidate = base with the trip's launch replaced, the new FLP inserted into its gap if absent (gap FLPs stay in
//       (ratio, id) order), the old FLP removed iff no trip launches from it any more. If the recovery is not strictly after
//       the launch in the candidate route -> InvalidRecovery (not evaluated, nothing repaired); else judged by the reference.
//   N2, trip i with recovery at skeleton index s_k, window w: s = max(1,s_k-w) .. min(last,s_k+w), s != s_k ascending; candidate =
//       base with the trip's recovery replaced by s_s (customer or END depot); the route is untouched. InvalidRecovery iff
//       s < (gap of the launch) + 1; else judged by the reference.
//   Judged candidates: reference-feasible -> accepted with the reference objective; infeasible -> an evaluator reason
//   (EvalEndurance iff some trip violates endurance in the reference). Best = the first candidate attaining the minimum.

#include "config/static_config.hpp"
#include "flp/flp_arc_index.hpp"
#include "init/initial_solution_builder.hpp"
#include "instance/static_instance.hpp"
#include "reference/timing_engine_reference.hpp"
#include "search/neighbourhood_n1.hpp"
#include "search/neighbourhood_n2.hpp"
#include "search/neighbourhood_n3.hpp"
#include "search/neighbourhood_n4.hpp"
#include "search/neighbourhood_n5.hpp"
#include "search/neighbourhood_n6.hpp"
#include "search/neighbourhood_n7.hpp"
#include "search/neighbourhood_n8.hpp"
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
    if (!ok) {
        if (g_failures < 40) std::cout << "[FAIL] " << what << "\n";
        ++g_failures;
    }
}

struct World {
    const NodeRegistry* nodes;
    const tdr::graph::SpatialNetwork* net;
    const std::unordered_map<NodeId, CustomerData>* cd;
    const tdr::timing::FlpDataMap* flp;
    const tdr::flp::FlpArcIndex* idx;
    int U;
};

bool is_flp(const World& w, NodeId v) { return w.nodes->at(v).type == NodeType::FlexibleLaunchPoint; }

bool covers_once(const World& w, const Solution& s) {
    std::set<NodeId> seen;
    for (NodeId v : s.truck_route) if (w.nodes->at(v).type == NodeType::Customer && !seen.insert(v).second) return false;
    for (const auto& t : s.drone_trips) if (!seen.insert(t.service_customer).second) return false;
    return seen.size() == w.cd->size();
}

bool no_foreign_flp(const World& w, const TruckRoute& r) {
    NodeId last = kInvalidNodeId;
    std::vector<NodeId> pending;
    for (NodeId v : r) {
        if (is_flp(w, v)) { pending.push_back(v); continue; }
        for (NodeId p : pending) { const auto& d = w.flp->at(p); if (d.arc_from != last || d.arc_to != v) return false; }
        pending.clear();
        last = v;
    }
    return pending.empty();
}

struct Ref { bool feasible; double objective; bool endurance_violated; };
Ref ref_eval(const World& w, const Solution& s) {
    const auto e = tdr::timing_ref::evaluate_solution(s, *w.net, *w.cd, *w.flp);
    Ref r{e.feasible, e.objective_value, false};
    for (const auto& t : e.drone_trip_timings)
        if (t.structurally_valid && t.on_route && t.has_customer_data && t.drone_id_valid && !t.endurance_feasible) r.endurance_violated = true;
    return r;
}

bool close(double a, double b) { return std::fabs(a - b) <= 1e-6 * std::max(1.0, std::fabs(b)); }

struct Stats {
    long bases = 0, bases_with_trips = 0, bases_with_two_drones_used = 0, bases_shared_launch = 0, scrambles = 0;
    long n1_calls = 0, n1_cand = 0, n1_acc = 0, n1_invalid = 0, n1_endurance = 0, n1_seq = 0, n1_fleet = 0, n1_eval_other = 0, n1_improving = 0, n1_ties = 0, n1_empty = 0;
    long n1_new_flp = 0, n1_existing_flp = 0, n1_shared_target = 0, n1_dropped_old = 0, n1_kept_old_shared = 0, n1_depot_arc = 0, n1_other_gap = 0, n1_same_gap = 0;
    long n2_calls = 0, n2_cand = 0, n2_acc = 0, n2_invalid = 0, n2_endurance = 0, n2_seq = 0, n2_fleet = 0, n2_eval_other = 0, n2_improving = 0, n2_ties = 0, n2_empty = 0;
    long n2_depot = 0, n2_clipped = 0, n2_to_customer = 0, n2_depot_anchor = 0;
    long bad_flp_arc = 0, bad_flp_unused = 0, bad_construction = 0;
    std::uint64_t digest = 14695981039346656037ULL;
    void mix(std::uint64_t v) { digest = (digest ^ v) * 1099511628211ULL; }
};

void tally(Stats& st, const OperatorResult& r) {
    st.bad_flp_arc += static_cast<long>(r.rejected[static_cast<std::size_t>(RejectReason::EvalFlpArc)]);
    st.bad_flp_unused += static_cast<long>(r.rejected[static_cast<std::size_t>(RejectReason::EvalFlpUnused)]);
    for (RejectReason x : {RejectReason::AffectedTrips, RejectReason::NoFreeDrone, RejectReason::NoReplacementFlp})
        st.bad_construction += static_cast<long>(r.rejected[static_cast<std::size_t>(x)]);
}

void mix_result(Stats& st, const OperatorResult& r) {
    st.mix(r.generated); st.mix(r.evaluated); st.mix(r.accepted); st.mix(r.duplicates_skipped);
    for (auto x : r.rejected) st.mix(x);
    if (r.best) {
        st.mix(static_cast<std::uint64_t>(r.best->move.launch));
        st.mix(static_cast<std::uint64_t>(r.best->move.recovery));
        for (NodeId v : r.best->solution.truck_route) st.mix(static_cast<std::uint64_t>(v));
    }
}

// ---- independent route arithmetic ----------------------------------------------------------------------------
std::size_t route_pos(const TruckRoute& r, NodeId v, std::size_t from = 0) {
    for (std::size_t i = from; i < r.size(); ++i) if (r[i] == v) return i;
    return r.size();
}

std::vector<NodeId> skeleton(const World& w, const TruckRoute& r) {
    std::vector<NodeId> s;
    for (NodeId v : r) if (!is_flp(w, v)) s.push_back(v);
    return s;
}

// gap index of an FLP on the route = (skeleton nodes before it) - 1
long gap_of(const World& w, const TruckRoute& r, NodeId flp) {
    long seen = 0;
    for (NodeId v : r) { if (v == flp) return seen - 1; if (!is_flp(w, v)) ++seen; }
    return -2;
}

// Rebuild a route from the skeleton and a set of FLPs: the FLPs of each gap in (position_ratio, id) order.
TruckRoute rebuild(const World& w, const std::vector<NodeId>& skel, const std::set<NodeId>& flps) {
    TruckRoute out;
    for (std::size_t s = 0; s < skel.size(); ++s) {
        out.push_back(skel[s]);
        if (s + 1 == skel.size()) break;
        std::vector<std::pair<double, NodeId>> here;
        for (NodeId f : flps) {
            const auto& d = w.flp->at(f);
            if (d.arc_from == skel[s] && d.arc_to == skel[s + 1]) here.emplace_back(d.position_ratio, f);
        }
        std::sort(here.begin(), here.end());
        for (const auto& p : here) out.push_back(p.second);
    }
    return out;
}

// All FLPs of the arc (a, b) in (ratio, id) order, straight from the FlpData map.
std::vector<NodeId> arc_flps(const World& w, NodeId a, NodeId b) {
    std::vector<std::pair<double, NodeId>> v;
    for (const auto& kv : *w.flp) if (kv.second.arc_from == a && kv.second.arc_to == b) v.emplace_back(kv.second.position_ratio, kv.first);
    std::sort(v.begin(), v.end());
    std::vector<NodeId> out;
    for (const auto& p : v) out.push_back(p.second);
    return out;
}

bool recovery_after_launch(const TruckRoute& r, const DroneTrip& t) {
    const std::size_t lp = route_pos(r, t.launch_node);
    if (lp == r.size()) return false;
    return route_pos(r, t.recovery_node, lp + 1) != r.size();
}

struct Want {
    NodeId launch, recovery;
    Solution sol;
    bool valid;
    std::vector<NodeId> dropped;
};

// Judge the production result against the brute-force candidate list.
void judge(const World& w, const Solution& base, const OperatorResult& res, const std::vector<Want>& want, MoveKind kind, OperatorId op, std::size_t anchor, NodeId replaced,
           const std::string& where, long& cand, long& acc, long& inval, long& endu, long& seq, long& fleet, long& other, long& improving, long& ties) {
    const auto base_ref = ref_eval(w, base);
    check(res.status == OperatorStatus::Ok && res.op == op && res.anchor_trip == anchor && res.anchor_customer == base.drone_trips[anchor].service_customer, where + ": status, operator id, anchor");
    check(close(res.base_objective, base_ref.objective), where + ": base objective == frozen reference");
    check(res.generated == want.size() && res.log.size() == want.size(), where + ": candidate count == brute force (" + std::to_string(res.generated) + " vs " + std::to_string(want.size()) + ")");
    std::size_t n_valid = 0;
    for (const auto& x : want) n_valid += x.valid;
    check(res.duplicates_skipped == 0 && res.evaluated == n_valid && res.accepted + res.rejected_total() == res.generated, where + ": one outcome per candidate; evaluated == valid candidates; no duplicates");
    if (res.log.size() != want.size()) return;

    std::optional<std::size_t> best_i;
    double best_obj = 0;
    std::size_t feasible_count = 0;
    for (std::size_t k = 0; k < want.size(); ++k) {
        const auto& rec = res.log[k];
        const auto& x = want[k];
        ++cand;
        const std::string cw = where + " cand " + std::to_string(k);
        check(rec.move.kind == kind && rec.move.replaced_trip == anchor && rec.move.customer == base.drone_trips[anchor].service_customer && rec.move.replaced_node == replaced &&
                  rec.move.launch == x.launch && rec.move.recovery == x.recovery && rec.move.drone_id == base.drone_trips[anchor].drone_id, cw + ": move descriptor");
        bool same = rec.solution.truck_route == x.sol.truck_route && rec.solution.drone_trips.size() == x.sol.drone_trips.size();
        for (std::size_t t = 0; same && t < x.sol.drone_trips.size(); ++t) {
            const auto &a = rec.solution.drone_trips[t], &b = x.sol.drone_trips[t];
            same = a.drone_id == b.drone_id && a.launch_node == b.launch_node && a.service_customer == b.service_customer && a.recovery_node == b.recovery_node;
        }
        check(same, cw + ": candidate solution == brute-force construction (route, launches, recoveries, drone ids)");
        check(rec.flps_dropped == x.dropped, cw + ": flps_dropped == brute force");
        check(covers_once(w, rec.solution) && no_foreign_flp(w, rec.solution.truck_route), cw + ": exactly-once coverage, no foreign FLPs");
        if (!x.valid) {
            ++inval;
            check(rec.reason == RejectReason::InvalidRecovery && !rec.evaluated && rec.affected_trips == std::vector<std::size_t>{anchor}, cw + ": recovery not after launch -> InvalidRecovery, not evaluated, trip named");
            continue;
        }
        check(rec.evaluated, cw + ": evaluated");
        const auto v = ref_eval(w, x.sol);
        if (v.feasible) {
            ++acc;
            ++feasible_count;
            check(rec.reason == RejectReason::None && close(rec.objective, v.objective), cw + ": reference-feasible -> accepted with the reference objective");
            if (!best_i || v.objective < best_obj) { best_i = k; best_obj = v.objective; }
            else if (v.objective == best_obj) ++ties;
        } else {
            const std::string nm = reject_reason_name(rec.reason);
            check(nm.rfind("eval-", 0) == 0, cw + ": reference-infeasible -> rejected with an evaluator reason (" + nm + ")");
            check((rec.reason == RejectReason::EvalEndurance) == v.endurance_violated, cw + ": EvalEndurance <=> the reference sees an endurance violation");
            if (rec.reason == RejectReason::EvalEndurance) ++endu;
            else if (rec.reason == RejectReason::EvalDroneSequencing) ++seq;
            else if (rec.reason == RejectReason::EvalFleetCapacity) ++fleet;
            else ++other;
        }
    }
    check(res.accepted == feasible_count, where + ": accepted count == number of reference-feasible candidates");
    check(static_cast<bool>(best_i) == res.best.has_value(), where + ": best exists iff some candidate is feasible");
    if (best_i && res.best) {
        check(close(res.best->objective, best_obj) && res.best->move.launch == want[*best_i].launch && res.best->move.recovery == want[*best_i].recovery && res.best->solution.truck_route == want[*best_i].sol.truck_route,
              where + ": best = the FIRST candidate attaining the minimum (strict <)");
        if (res.improves()) ++improving;
    }
}

// ---- N1 ----
void check_n1(const World& w, const Solution& base, int window, Stats& st, const std::string& tag) {
    const auto skel = skeleton(w, base.truck_route);
    const long G = static_cast<long>(skel.size()) - 1; // number of gaps
    for (std::size_t i = 0; i < base.drone_trips.size(); ++i) {
        const DroneTrip& bt = base.drone_trips[i];
        const long g0 = gap_of(w, base.truck_route, bt.launch_node);
        std::vector<Want> want;
        std::set<NodeId> route_flps;
        for (NodeId v : base.truck_route) if (is_flp(w, v)) route_flps.insert(v);
        bool old_shared = false;
        for (std::size_t k = 0; k < base.drone_trips.size(); ++k) if (k != i && base.drone_trips[k].launch_node == bt.launch_node) old_shared = true;
        for (long g = std::max(0L, g0 - window); g <= std::min(G - 1, g0 + window); ++g) {
            for (NodeId f : arc_flps(w, skel[static_cast<std::size_t>(g)], skel[static_cast<std::size_t>(g + 1)])) {
                if (f == bt.launch_node) continue;
                Want x;
                x.launch = f;
                x.recovery = bt.recovery_node;
                x.sol = base;
                x.sol.drone_trips[i].launch_node = f;
                std::set<NodeId> fl = route_flps;
                fl.insert(f);
                if (!old_shared) { fl.erase(bt.launch_node); x.dropped = {bt.launch_node}; }
                x.sol.truck_route = rebuild(w, skel, fl);
                x.valid = recovery_after_launch(x.sol.truck_route, x.sol.drone_trips[i]);
                want.push_back(x);
                if (route_flps.count(f)) { ++st.n1_existing_flp; for (const auto& t : base.drone_trips) if (t.launch_node == f) { ++st.n1_shared_target; break; } }
                else ++st.n1_new_flp;
                if (!old_shared) ++st.n1_dropped_old; else ++st.n1_kept_old_shared;
                if (g == 0) ++st.n1_depot_arc;
                if (g == g0) ++st.n1_same_gap; else ++st.n1_other_gap;
            }
        }
        N1Options opt;
        opt.record_candidates = true;
        const auto res = n1_launch_node_relocate(base, SearchContext{*w.net, *w.cd, *w.flp, *w.idx, window}, i, opt);
        const std::string where = tag + " N1 trip " + std::to_string(i) + " w=" + std::to_string(window);
        ++st.n1_calls;
        tally(st, res);
        mix_result(st, res);
        if (want.empty()) ++st.n1_empty;
        long c = 0, a = 0, inv = 0, en = 0, sq = 0, fl = 0, ot = 0, im = 0, ti = 0;
        judge(w, base, res, want, MoveKind::N1LaunchRelocate, OperatorId::N1, i, bt.launch_node, where, c, a, inv, en, sq, fl, ot, im, ti);
        st.n1_cand += c; st.n1_acc += a; st.n1_invalid += inv; st.n1_endurance += en; st.n1_seq += sq; st.n1_fleet += fl; st.n1_eval_other += ot; st.n1_improving += im; st.n1_ties += ti;
    }
}

// ---- N2 ----
void check_n2(const World& w, const Solution& base, int window, Stats& st, const std::string& tag) {
    const auto skel = skeleton(w, base.truck_route);
    const long last = static_cast<long>(skel.size()) - 1;
    for (std::size_t i = 0; i < base.drone_trips.size(); ++i) {
        const DroneTrip& bt = base.drone_trips[i];
        const long g0 = gap_of(w, base.truck_route, bt.launch_node);
        // skeleton index of the recovery: the depot recovers at the END (index last); a customer at its single occurrence
        long sk = -1;
        for (long s = 1; s <= last; ++s) if (skel[static_cast<std::size_t>(s)] == bt.recovery_node) sk = s;
        check(sk >= 1, tag + ": the recovery node is on the skeleton");
        std::vector<Want> want;
        bool clipped = false;
        for (long s = std::max(1L, sk - window); s <= std::min(last, sk + window); ++s) {
            if (s == sk) continue;
            Want x;
            x.launch = bt.launch_node;
            x.recovery = skel[static_cast<std::size_t>(s)];
            x.sol = base;
            x.sol.drone_trips[i].recovery_node = x.recovery;
            x.valid = s >= g0 + 1;
            check(x.valid == recovery_after_launch(x.sol.truck_route, x.sol.drone_trips[i]), tag + ": s >= gap+1 agrees with the route-position test");
            want.push_back(x);
            if (x.recovery == 0) ++st.n2_depot; else ++st.n2_to_customer;
        }
        if (sk - window < 1 || sk + window > last) clipped = true;
        if (clipped) ++st.n2_clipped;
        if (bt.recovery_node == 0) ++st.n2_depot_anchor;
        N2Options opt;
        opt.record_candidates = true;
        const auto res = n2_recover_node_relocate(base, SearchContext{*w.net, *w.cd, *w.flp, *w.idx, window}, i, opt);
        const std::string where = tag + " N2 trip " + std::to_string(i) + " w=" + std::to_string(window);
        ++st.n2_calls;
        tally(st, res);
        mix_result(st, res);
        if (want.empty()) ++st.n2_empty;
        long c = 0, a = 0, inv = 0, en = 0, sq = 0, fl = 0, ot = 0, im = 0, ti = 0;
        judge(w, base, res, want, MoveKind::N2RecoveryRelocate, OperatorId::N2, i, bt.recovery_node, where, c, a, inv, en, sq, fl, ot, im, ti);
        st.n2_cand += c; st.n2_acc += a; st.n2_invalid += inv; st.n2_endurance += en; st.n2_seq += sq; st.n2_fleet += fl; st.n2_eval_other += ot; st.n2_improving += im; st.n2_ties += ti;
    }
}

// ------------------------------------------------------------------ campaign
Stats campaign(int seeds) {
    Stats st;
    for (int seed = 0; seed < seeds; ++seed) {
        tdr::util::Rng fz(tdr::util::derive_seed(6262, tdr::util::SeedStream::Fuzz, static_cast<std::uint64_t>(seed)));
        tdr::config::StaticConfig cfg;
        cfg.seed.base_seed = 6262;
        cfg.instance.num_customers = static_cast<int>(fz.uniform_int(8, 14));
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
        const World w{&init.node_registry, &net, &inst.customer_data, &init.flp_data, &idx, cfg.vehicle.num_drones};

        Solution cur = init.solution;
        for (int step = 0; step < 8; ++step) {
            ++st.bases;
            if (!cur.drone_trips.empty()) ++st.bases_with_trips;
            {
                std::set<int> used;
                std::map<NodeId, int> launches;
                for (const auto& t : cur.drone_trips) { used.insert(t.drone_id); ++launches[t.launch_node]; }
                if (used.size() >= 2) ++st.bases_with_two_drones_used;
                for (const auto& kv : launches) if (kv.second >= 2) { ++st.bases_shared_launch; break; }
            }
            const std::string tag = "seed " + std::to_string(seed) + " step " + std::to_string(step);
            if (!cur.drone_trips.empty() && cfg.vehicle.num_drones >= 2 && step % 2 == 1) {
                tdr::util::Rng sc(tdr::util::derive_seed(6264, tdr::util::SeedStream::Fuzz, static_cast<std::uint64_t>(seed * 100 + step)));
                for (int attempt = 0; attempt < 6; ++attempt) {
                    Solution trial = cur;
                    for (auto& t : trial.drone_trips) t.drone_id = static_cast<int>(sc.uniform_int(0, cfg.vehicle.num_drones - 1));
                    if (ref_eval(w, trial).feasible) { cur = trial; ++st.scrambles; break; }
                }
            }
            if (!cur.drone_trips.empty()) {
                for (int win : {0, 1, 2, 3}) { check_n1(w, cur, win, st, tag); check_n2(w, cur, win, st, tag); }
                // the randomised overloads: one uniform_index draw, anchor = vector index
                tdr::util::Rng a(seed * 7 + step), b(seed * 7 + step);
                const SearchContext ctx{net, inst.customer_data, init.flp_data, idx, 2};
                const auto r1 = n1_launch_node_relocate(cur, ctx, a);
                const auto r2 = n2_recover_node_relocate(cur, ctx, b);
                tdr::util::Rng c(seed * 7 + step);
                const std::size_t idx_draw = c.uniform_index(cur.drone_trips.size());
                check(r1.anchor_trip == idx_draw && r2.anchor_trip == idx_draw && a.next_u64() == c.next_u64() , tag + ": Rng overloads draw exactly once, anchor = uniform_index(#trips)");
            }
            // random walk over all eight operators so that trips of many shapes (shared launches, trips recovered anywhere) occur
            tdr::util::Rng walk(tdr::util::derive_seed(6263, tdr::util::SeedStream::Fuzz, static_cast<std::uint64_t>(seed * 100 + step)));
            const SearchContext ctx{net, inst.customer_data, init.flp_data, idx, 3};
            OperatorResult r;
            const double pick = walk.uniform01();
            if (!cur.drone_trips.empty() && pick < 0.12) {
                N8Options o; o.record_candidates = true;
                r = n8_drone_node_delete_truck_insert(cur, ctx, walk, o);
            } else if (pick < 0.42) {
                N7Options o; o.record_candidates = true;
                r = n7_truck_node_delete_drone_insert(cur, ctx, walk, o);
            } else if (!cur.drone_trips.empty() && pick < 0.52) {
                N5Options o; o.record_candidates = true;
                r = n5_td_service_node_swap(cur, ctx, walk, o);
            } else if (!cur.drone_trips.empty() && pick < 0.62) {
                N3Options o; o.record_candidates = true;
                r = n3_drone_service_node_relocate(cur, ctx, walk, o);
            } else if (!cur.drone_trips.empty() && pick < 0.70) {
                N6Options o; o.record_candidates = true;
                r = n6_drone_drone_swap(cur, ctx, walk, o);
            } else if (!cur.drone_trips.empty() && pick < 0.85) {
                N1Options o; o.record_candidates = true;
                r = n1_launch_node_relocate(cur, ctx, walk, o);
            } else if (!cur.drone_trips.empty() && pick < 0.95) {
                N2Options o; o.record_candidates = true;
                r = n2_recover_node_relocate(cur, ctx, walk, o);
            } else {
                N4Options o; o.record_candidates = true;
                r = n4_truck_service_node_relocate(cur, ctx, walk, o);
            }
            std::vector<const CandidateRecord*> acc;
            for (const auto& rec : r.log) if (rec.reason == RejectReason::None) acc.push_back(&rec);
            mix_result(st, r);
            if (acc.empty()) continue;
            cur = acc[walk.uniform_index(acc.size())]->solution;
        }
    }
    return st;
}

} // namespace

int main() {
    constexpr int kSeeds = 80;
    const Stats a = campaign(kSeeds);
    const Stats b = campaign(kSeeds);

    std::cout << "drone-id re-draws accepted=" << a.scrambles << "\n"
              << "bases=" << a.bases << " with_trips=" << a.bases_with_trips << " two_drones_used=" << a.bases_with_two_drones_used << " shared_launch_bases=" << a.bases_shared_launch << "\n"
              << "N1: calls(w=0..3)=" << a.n1_calls << " candidates=" << a.n1_cand << " accepted=" << a.n1_acc << " invalid_recovery=" << a.n1_invalid << " improving_calls=" << a.n1_improving
              << " endurance=" << a.n1_endurance << " sequencing=" << a.n1_seq << " fleet=" << a.n1_fleet << " other_eval=" << a.n1_eval_other << " empty_calls=" << a.n1_empty << " ties=" << a.n1_ties << "\n"
              << "    new_flp_targets=" << a.n1_new_flp << " existing_flp_targets=" << a.n1_existing_flp << " shared_target(used by another trip)=" << a.n1_shared_target << " old_dropped=" << a.n1_dropped_old
              << " old_kept(shared)=" << a.n1_kept_old_shared << " depot_arc_targets=" << a.n1_depot_arc << " same_gap=" << a.n1_same_gap << " other_gap=" << a.n1_other_gap << "\n"
              << "N2: calls(w=0..3)=" << a.n2_calls << " candidates=" << a.n2_cand << " accepted=" << a.n2_acc << " invalid_recovery=" << a.n2_invalid << " improving_calls=" << a.n2_improving
              << " endurance=" << a.n2_endurance << " sequencing=" << a.n2_seq << " fleet=" << a.n2_fleet << " other_eval=" << a.n2_eval_other << " empty_calls=" << a.n2_empty << " ties=" << a.n2_ties << "\n"
              << "    depot_candidates=" << a.n2_depot << " customer_candidates=" << a.n2_to_customer << " window_clipped_calls=" << a.n2_clipped << " depot_recovery_anchors=" << a.n2_depot_anchor << "\n";

    check(a.bases > 300 && a.bases_with_trips > 200, "non-vacuity: enough bases, many with trips");
    check(a.bases_with_two_drones_used > 30, "non-vacuity: bases with two drones in use occur");
    check(a.n1_cand > 500 && a.n1_acc > 50, "non-vacuity: N1 candidates and accepted candidates occur");
    check(a.n1_invalid > 50, "non-vacuity: N1 InvalidRecovery candidates occur");
    check(a.n1_endurance > 0, "non-vacuity: N1 endurance rejections occur");
    check(a.n1_improving > 0, "non-vacuity: N1 finds strict improvements");
    check(a.n1_new_flp > 100 && a.n1_existing_flp > 10 && a.n1_dropped_old > 100, "non-vacuity: N1 inserts new FLPs, reuses existing ones, and drops old launches");
    check(a.n1_empty > 5, "non-vacuity: N1 calls with no candidate occur");
    check(a.n1_depot_arc > 10 && a.n1_other_gap > 100 && a.n1_same_gap > 50, "non-vacuity: N1 candidates on the depot arc, the same gap and other gaps occur");
    check(a.n2_cand > 500 && a.n2_acc > 50, "non-vacuity: N2 candidates and accepted candidates occur");
    check(a.n2_invalid > 20, "non-vacuity: N2 InvalidRecovery candidates occur");
    check(a.n2_endurance > 0, "non-vacuity: N2 endurance rejections occur");
    check(a.n2_improving > 0, "non-vacuity: N2 finds strict improvements");
    check(a.n2_depot > 20 && a.n2_to_customer > 200, "non-vacuity: N2 candidates at the end depot and at customers occur");
    check(a.n2_clipped > 20 && a.n2_empty > 0, "non-vacuity: windows clipped by the route ends and empty calls occur");
    check(a.bad_flp_arc == 0 && a.bad_flp_unused == 0 && a.bad_construction == 0,
          "N1/N2 never produce an FLP-arc or idle-FLP rejection, nor any AffectedTrips/NoFreeDrone/NoReplacementFlp rejection");
    check(a.digest == b.digest, "deterministic: identical digest over the whole campaign on a second full run (oracle on both)");

    std::cout << g_checks << " checks, " << g_failures << " failures\n";
    if (g_failures == 0) std::cout << "ALL TESTS PASSED\n";
    return g_failures == 0 ? 0 : 1;
}
