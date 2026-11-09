// M3 oracle test. For seeded random instances (and random-walk neighbours of their
// initial solutions, so that trips of many shapes occur) the operators N7 and N8 are compared
// RECORD BY RECORD with an independent brute-force enumeration written here with its own
// route/skeleton/arc arithmetic (it does not call the production operators' helpers or the
// FlpArcIndex), judged by the FROZEN reference evaluator (tdr::timing_ref).
//
// Oracle contract (see docs/static/operators_N7_N8.md):
//   N7, per interior truck customer t: remove t from the route; if any other trip then has its
//       launch FLP off its own arc or recovers at t -> ONE AffectedTrips record (trips named);
//       else, for every gap g of the resulting skeleton whose ORIGINAL half-step coordinate
//       (left + right original skeleton index) is within 2w of 2 s_t, and every FLP of that arc
//       (read from the ownership data, ratio then id): trip (f, t, arc end), f inserted in ratio
//       order or reused, lowest reference-feasible drone id (none -> NoFreeDrone or an evaluator
//       reason). No FLP in any eligible arc -> ONE NoReplacementFlp record.
//   N8, per trip: dissolve it, drop FLPs no remaining trip launches from, insert its customer
//       before skeleton node g+1 for EVERY gap g (and, with insertion_window W, only the gaps with
//       |2g+1-h| <= 2W, h = half-step of the customer): a gap that leaves another trip's launch FLP
//       off its arc -> AffectedTrips (trips named, base indices); else judged by the reference.

#include "config/static_config.hpp"
#include "flp/flp_arc_index.hpp"
#include "init/initial_solution_builder.hpp"
#include "instance/static_instance.hpp"
#include "reference/timing_engine_reference.hpp"
#include "search/neighbourhood_n4.hpp"
#include "search/neighbourhood_n5.hpp"
#include "search/neighbourhood_n7.hpp"
#include "search/neighbourhood_n8.hpp"
#include "util/rng.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <set>
#include <string>
#include <tuple>
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

std::vector<std::size_t> skel_positions(const World& w, const TruckRoute& r) {
    std::vector<std::size_t> p;
    for (std::size_t i = 0; i < r.size(); ++i) if (!is_flp(w, r[i])) p.push_back(i);
    return p;
}

// independent "an FLP is not between the endpoints of its own arc"
std::vector<NodeId> foreign_flps(const World& w, const TruckRoute& r) {
    std::vector<NodeId> out;
    NodeId last = kInvalidNodeId;
    std::vector<NodeId> pending;
    for (NodeId v : r) {
        if (is_flp(w, v)) { pending.push_back(v); continue; }
        for (NodeId p : pending) { const auto& d = w.flp->at(p); if (d.arc_from != last || d.arc_to != v) out.push_back(p); }
        pending.clear();
        last = v;
    }
    for (NodeId p : pending) out.push_back(p);
    return out;
}

bool covers_once(const World& w, const Solution& s) {
    std::set<NodeId> seen;
    for (NodeId v : s.truck_route) if (w.nodes->at(v).type == NodeType::Customer && !seen.insert(v).second) return false;
    for (const auto& t : s.drone_trips) if (!seen.insert(t.service_customer).second) return false;
    std::size_t customers = 0;
    for (const auto& kv : *w.cd) { (void)kv; ++customers; }
    return seen.size() == customers;
}


struct RefVerdict { bool feasible; double objective; };
RefVerdict ref_eval(const World& w, const Solution& s) {
    const auto e = tdr::timing_ref::evaluate_solution(s, *w.net, *w.cd, *w.flp);
    return {e.feasible, e.objective_value};
}

bool close(double a, double b) { return std::fabs(a - b) <= 1e-6 * std::max(1.0, std::fabs(b)); }

struct Stats {
    long bases = 0, bases_with_trips = 0;
    long n7_calls = 0, n7_candidates = 0, n7_accepted = 0, n7_affected = 0, n7_noflp = 0, n7_nofree = 0, n7_endurance = 0, n7_eval_other = 0, n7_reuse = 0, n7_depot_recovery = 0, n7_second_drone = 0, n7_improving = 0;
    long n8_calls = 0, n8_candidates = 0, n8_accepted = 0, n8_affected = 0, n8_endurance = 0, n8_eval_other = 0, n8_dropped = 0, n8_improving = 0, n8_windowed_calls = 0, n8_outside_default_window = 0;
    long bad_reason_flp_arc = 0, bad_reason_flp_unused = 0, bad_reason_fleet = 0;
    std::uint64_t digest = 14695981039346656037ULL;
    void mix(std::uint64_t v) { digest = (digest ^ v) * 1099511628211ULL; }
};


void tally(Stats& st, const OperatorResult& r) {
    st.bad_reason_flp_arc += static_cast<long>(r.rejected[static_cast<std::size_t>(RejectReason::EvalFlpArc)]);
    st.bad_reason_flp_unused += static_cast<long>(r.rejected[static_cast<std::size_t>(RejectReason::EvalFlpUnused)]);
    st.bad_reason_fleet += static_cast<long>(r.rejected[static_cast<std::size_t>(RejectReason::EvalFleetCapacity)]);
}

void mix_result(Stats& st, const OperatorResult& r) {
    st.mix(r.generated); st.mix(r.evaluated); st.mix(r.accepted); st.mix(r.duplicates_skipped);
    for (auto x : r.rejected) st.mix(x);
    if (r.best) {
        for (NodeId v : r.best->solution.truck_route) st.mix(static_cast<std::uint64_t>(v));
        for (const auto& t : r.best->solution.drone_trips) { st.mix(static_cast<std::uint64_t>(t.drone_id)); st.mix(static_cast<std::uint64_t>(t.launch_node)); st.mix(static_cast<std::uint64_t>(t.recovery_node)); }
    }
}

bool is_eval_reason(RejectReason r) { return std::string(reject_reason_name(r)).rfind("eval-", 0) == 0; }

// FLPs of arc (x,y) from the ownership data, ratio ascending then id (independent of FlpArcIndex).
std::vector<NodeId> arc_flps(const World& w, NodeId x, NodeId y) {
    std::vector<std::pair<std::pair<double, NodeId>, NodeId>> v;
    for (const auto& kv : *w.flp) if (kv.second.arc_from == x && kv.second.arc_to == y) v.push_back({{kv.second.position_ratio, kv.first}, kv.first});
    std::sort(v.begin(), v.end());
    std::vector<NodeId> out;
    for (const auto& e : v) out.push_back(e.second);
    return out;
}

// ------------------------------------------------------------------ N7 oracle
void check_n7(const World& w, const Solution& base, int window, Stats& st, const std::string& tag) {
    const auto sp = skel_positions(w, base.truck_route);
    for (std::size_t s = 1; s + 1 < sp.size(); ++s) {
        const NodeId t = base.truck_route[sp[s]];
        if (w.nodes->at(t).type != NodeType::Customer) continue;

        TruckRoute r = base.truck_route;
        r.erase(r.begin() + static_cast<long>(sp[s]));
        const auto foreign = foreign_flps(w, r);
        std::vector<std::size_t> affected;
        for (std::size_t k = 0; k < base.drone_trips.size(); ++k) {
            const auto& x = base.drone_trips[k];
            if (x.recovery_node == t || std::find(foreign.begin(), foreign.end(), x.launch_node) != foreign.end()) affected.push_back(k);
        }

        N7Options opt;
        opt.record_candidates = true;
        const auto res = n7_truck_node_delete_drone_insert(base, SearchContext{*w.net, *w.cd, *w.flp, *w.idx, window}, t, opt);
        ++st.n7_calls;
        tally(st, res);
        mix_result(st, res);
        const std::string where = tag + " N7 anchor C" + std::to_string(t) + " w=" + std::to_string(window);
        check(res.status == OperatorStatus::Ok && res.anchor_customer == t, where + ": status ok, anchor");
        check(res.duplicates_skipped == 0 && res.accepted + res.rejected_total() == res.generated && res.log.size() == res.generated, where + ": one outcome per candidate, one record each, no duplicates");

        if (!affected.empty()) {
            ++st.n7_candidates;
            ++st.n7_affected;
            check(res.generated == 1 && res.log.size() == 1 && res.log[0].reason == RejectReason::AffectedTrips && !res.log[0].evaluated && res.evaluated == 0 && !res.best, where + ": disturbed trips -> ONE AffectedTrips record, not evaluated");
            if (res.log.size() == 1) check(res.log[0].affected_trips == affected, where + ": affected trip list");
            continue;
        }

        const auto sp2 = skel_positions(w, r);
        struct Opt { std::size_t g; NodeId x, y, f; };
        std::vector<Opt> options;
        for (std::size_t g = 0; g + 1 < sp2.size(); ++g) {
            // original skeleton indices of the endpoints of resulting gap g
            const std::size_t left = (g < s) ? g : g + 1;
            const std::size_t right = (g + 1 < s) ? g + 1 : g + 2;
            const long h_old = static_cast<long>(left + right);
            if (std::labs(h_old - 2L * static_cast<long>(s)) > 2L * window) continue;
            const NodeId x = r[sp2[g]], y = r[sp2[g + 1]];
            for (NodeId f : arc_flps(w, x, y)) options.push_back(Opt{g, x, y, f});
        }
        if (options.empty()) {
            ++st.n7_candidates;
            ++st.n7_noflp;
            check(res.generated == 1 && res.log.size() == 1 && res.log[0].reason == RejectReason::NoReplacementFlp && !res.log[0].evaluated && res.evaluated == 0 && !res.best, where + ": no FLP on any eligible arc -> ONE NoReplacementFlp record");
            continue;
        }
        check(res.generated == options.size() && res.log.size() == options.size(), where + ": candidate count == brute force (" + std::to_string(res.generated) + " vs " + std::to_string(options.size()) + ")");
        if (res.log.size() != options.size()) continue;

        std::optional<std::size_t> best_i;
        double best_obj = 0;
        for (std::size_t i = 0; i < options.size(); ++i) {
            const auto& op = options[i];
            const auto& rec = res.log[i];
            ++st.n7_candidates;
            TruckRoute rr = r;
            const bool on_route = std::find(rr.begin(), rr.end(), op.f) != rr.end();
            if (on_route) ++st.n7_reuse;
            if (!on_route) {
                std::size_t at = sp2[op.g] + 1;
                const auto key = [&](NodeId v) { return std::make_pair(w.flp->at(v).position_ratio, v); };
                while (at < sp2[op.g + 1] && key(rr[at]) < key(op.f)) ++at;
                rr.insert(rr.begin() + static_cast<long>(at), op.f);
            }
            std::optional<int> lowest;
            double obj = 0;
            for (int id = 0; id < w.U && !lowest; ++id) {
                Solution cand;
                cand.truck_route = rr;
                cand.drone_trips = base.drone_trips;
                DroneTrip nt; nt.drone_id = id; nt.launch_node = op.f; nt.service_customer = t; nt.recovery_node = op.y;
                cand.drone_trips.push_back(nt);
                const auto v = ref_eval(w, cand);
                if (v.feasible) { lowest = id; obj = v.objective; }
            }
            const std::string cw = where + " gap " + std::to_string(op.g) + " launch " + std::to_string(op.f);
            check(rec.move.customer == t && rec.move.launch == op.f && rec.move.recovery == op.y && rec.move.gap == op.g, cw + ": move descriptor");
            check(rec.solution.truck_route == rr, cw + ": candidate route equals brute force");
            check(rec.solution.drone_trips.size() == base.drone_trips.size() + 1, cw + ": exactly one trip added, none removed");
            bool kept = rec.solution.drone_trips.size() == base.drone_trips.size() + 1;
            for (std::size_t k = 0; kept && k < base.drone_trips.size(); ++k) {
                const auto& a = rec.solution.drone_trips[k]; const auto& b = base.drone_trips[k];
                kept = a.drone_id == b.drone_id && a.launch_node == b.launch_node && a.service_customer == b.service_customer && a.recovery_node == b.recovery_node;
            }
            check(kept, cw + ": the existing trips are untouched (ids, launches, customers, recoveries)");
            if (op.y == 0) ++st.n7_depot_recovery;
            if (lowest) {
                ++st.n7_accepted;
                check(rec.reason == RejectReason::None && rec.evaluated && close(rec.objective, obj) && rec.move.drone_id == *lowest, cw + ": feasible -> accepted with the lowest feasible drone id (op " + std::to_string(rec.move.drone_id) + ", oracle " + std::to_string(*lowest) + ")");
                if (rec.move.drone_id > 0) ++st.n7_second_drone;
                check(covers_once(w, rec.solution), cw + ": exactly-once coverage");
                check(foreign_flps(w, rec.solution.truck_route).empty(), cw + ": no foreign FLPs");
                if (!best_i || obj < best_obj) { best_i = i; best_obj = obj; }
            } else {
                check(rec.reason != RejectReason::None && rec.reason != RejectReason::AffectedTrips, cw + ": infeasible for every drone id -> rejected (reason " + reject_reason_name(rec.reason) + ")");
                if (rec.reason == RejectReason::NoFreeDrone) { ++st.n7_nofree; check(!rec.evaluated, cw + ": NoFreeDrone is never evaluated"); }
                else if (rec.reason == RejectReason::EvalEndurance) { ++st.n7_endurance; check(is_eval_reason(rec.reason), cw + ": endurance is an evaluator reason"); }
                else { ++st.n7_eval_other; check(is_eval_reason(rec.reason), cw + ": other rejections are evaluator reasons"); }
            }
        }
        check(static_cast<bool>(best_i) == res.best.has_value(), where + ": best exists iff some candidate is feasible");
        if (best_i && res.best) {
            check(close(res.best->objective, best_obj), where + ": best objective = minimum");
            check(res.best->move.launch == options[*best_i].f && res.best->move.gap == options[*best_i].g, where + ": best = the FIRST candidate attaining the minimum (strict <)");
            check(res.best->move.kind == MoveKind::N7TruckToDrone && res.best->move.customer == t && res.best->move.recovery == options[*best_i].y && res.best->move.drone_id == res.log[*best_i].move.drone_id, where + ": the best candidate's move descriptor (kind, customer, recovery, drone id)");
            if (res.improves()) ++st.n7_improving;
        }
    }
}

// ------------------------------------------------------------------ N8 oracle
void check_n8_one(const World& w, const Solution& base, std::size_t ti, int window, std::optional<int> ins_window, Stats& st, const std::string& tag) {
    const auto& old_trip = base.drone_trips[ti];
    const NodeId c = old_trip.service_customer;
    std::vector<DroneTrip> rest;
    std::vector<std::size_t> rest_index;
    for (std::size_t k = 0; k < base.drone_trips.size(); ++k) if (k != ti) { rest.push_back(base.drone_trips[k]); rest_index.push_back(k); }
    std::set<NodeId> used;
    for (const auto& x : rest) used.insert(x.launch_node);
    TruckRoute r0;
    std::vector<NodeId> dropped;
    for (NodeId v : base.truck_route) {
        if (is_flp(w, v) && used.count(v) == 0) dropped.push_back(v);
        else r0.push_back(v);
    }
    const auto sp = skel_positions(w, r0);
    // half-step of c in the base: 2 * (gap holding its launch FLP) + 1
    const auto spb = skel_positions(w, base.truck_route);
    std::size_t lp = 0;
    for (std::size_t i = 0; i < base.truck_route.size(); ++i) if (base.truck_route[i] == old_trip.launch_node) lp = i;
    std::size_t gd = 0;
    for (std::size_t k = 0; k < spb.size(); ++k) if (spb[k] < lp) gd = k;
    const long hc = static_cast<long>(2 * gd + 1);

    std::vector<std::size_t> gaps;
    for (std::size_t g = 0; g + 1 < sp.size(); ++g) {
        if (ins_window && std::labs(static_cast<long>(2 * g + 1) - hc) > 2L * *ins_window) continue;
        gaps.push_back(g);
    }

    N8Options opt;
    opt.record_candidates = true;
    opt.insertion_window = ins_window;
    const auto res = n8_drone_node_delete_truck_insert(base, SearchContext{*w.net, *w.cd, *w.flp, *w.idx, window}, ti, opt);
    ++st.n8_calls;
    if (ins_window) ++st.n8_windowed_calls;
    tally(st, res);
    mix_result(st, res);
    const std::string where = tag + " N8 trip " + std::to_string(ti) + " (C" + std::to_string(c) + ") w=" + std::to_string(window) + (ins_window ? " W=" + std::to_string(*ins_window) : std::string(" all-gaps"));
    check(res.status == OperatorStatus::Ok && res.anchor_customer == c && res.anchor_trip == ti, where + ": status ok, anchor");
    check(res.duplicates_skipped == 0 && res.accepted + res.rejected_total() == res.generated && res.log.size() == res.generated, where + ": one outcome per candidate, one record each, no duplicates");
    check(res.generated == gaps.size() && res.log.size() == gaps.size(), where + ": candidate count == brute force (" + std::to_string(res.generated) + " vs " + std::to_string(gaps.size()) + ")");
    if (res.log.size() != gaps.size()) return;

    std::optional<std::size_t> best_i;
    double best_obj = 0;
    for (std::size_t i = 0; i < gaps.size(); ++i) {
        const std::size_t g = gaps[i];
        const auto& rec = res.log[i];
        ++st.n8_candidates;
        if (!ins_window && std::labs(static_cast<long>(2 * g + 1) - hc) > 2L * window) ++st.n8_outside_default_window;
        TruckRoute rr = r0;
        rr.insert(rr.begin() + static_cast<long>(sp[g + 1]), c);
        const auto foreign = foreign_flps(w, rr);
        std::vector<std::size_t> affected;
        for (std::size_t k = 0; k < rest.size(); ++k) if (std::find(foreign.begin(), foreign.end(), rest[k].launch_node) != foreign.end()) affected.push_back(rest_index[k]);
        const std::string cw = where + " gap " + std::to_string(g);
        check(rec.move.customer == c && rec.move.gap == g && rec.move.replaced_trip == ti && rec.move.kind == MoveKind::N8DroneToTruck, cw + ": move descriptor");
        check(rec.solution.truck_route == rr, cw + ": candidate route equals brute force");
        check(rec.flps_dropped == dropped, cw + ": dropped FLPs equal brute force");
        bool kept = rec.solution.drone_trips.size() == rest.size();
        for (std::size_t k = 0; kept && k < rest.size(); ++k) {
            const auto& a = rec.solution.drone_trips[k]; const auto& b = rest[k];
            kept = a.drone_id == b.drone_id && a.launch_node == b.launch_node && a.service_customer == b.service_customer && a.recovery_node == b.recovery_node;
        }
        check(kept, cw + ": the other trips are untouched (ids, launches, customers, recoveries), the dissolved one is gone");
        if (!affected.empty()) {
            ++st.n8_affected;
            check(rec.reason == RejectReason::AffectedTrips && !rec.evaluated && rec.affected_trips == affected, cw + ": a gap holding another trip's FLP -> AffectedTrips naming exactly those trips, not evaluated");
            continue;
        }
        Solution cand;
        cand.truck_route = rr;
        cand.drone_trips = rest;
        const auto v = ref_eval(w, cand);
        if (v.feasible) {
            ++st.n8_accepted;
            check(rec.reason == RejectReason::None && rec.evaluated && close(rec.objective, v.objective), cw + ": feasible -> accepted with the reference objective");
            check(covers_once(w, rec.solution) && foreign_flps(w, rec.solution.truck_route).empty(), cw + ": exactly-once coverage, no foreign FLPs");
            if (!best_i || v.objective < best_obj) { best_i = i; best_obj = v.objective; }
        } else {
            check(rec.evaluated && is_eval_reason(rec.reason), cw + ": reference-infeasible -> rejected with an evaluator reason (" + reject_reason_name(rec.reason) + ")");
            if (rec.reason == RejectReason::EvalEndurance) ++st.n8_endurance; else ++st.n8_eval_other;
        }
    }
    st.n8_dropped += static_cast<long>(!dropped.empty());
    check(static_cast<bool>(best_i) == res.best.has_value(), where + ": best exists iff some candidate is feasible");
    if (best_i && res.best) {
        check(close(res.best->objective, best_obj) && res.best->move.gap == gaps[*best_i], where + ": best = the FIRST candidate attaining the minimum (strict <)");
        check(res.best->move.kind == MoveKind::N8DroneToTruck && res.best->move.replaced_trip == ti && res.best->move.customer == c && res.best->flps_dropped == dropped, where + ": the best candidate's move descriptor (kind, dissolved trip, customer, dropped FLPs)");
        if (res.improves()) ++st.n8_improving;
    }
}

void check_n8(const World& w, const Solution& base, int window, Stats& st, const std::string& tag) {
    for (std::size_t ti = 0; ti < base.drone_trips.size(); ++ti) {
        check_n8_one(w, base, ti, window, std::nullopt, st, tag);
        check_n8_one(w, base, ti, window, 1, st, tag);
    }
}

// ------------------------------------------------------------------ campaign
Stats campaign(bool run_oracle, int seeds) {
    Stats st;
    for (int seed = 0; seed < seeds; ++seed) {
        tdr::util::Rng fz(tdr::util::derive_seed(4242, tdr::util::SeedStream::Fuzz, static_cast<std::uint64_t>(seed)));
        tdr::config::StaticConfig cfg;
        cfg.seed.base_seed = 4242;
        cfg.instance.num_customers = static_cast<int>(fz.uniform_int(6, 11));
        cfg.instance.dataset_index = seed;
        cfg.vehicle.num_drones = static_cast<int>(fz.uniform_int(1, 3));
        if (seed % 3 == 2) cfg.vehicle.endurance_s = cfg.vehicle.endurance_s * 0.7;
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
        for (int step = 0; step < 6; ++step) {
            ++st.bases;
            if (!cur.drone_trips.empty()) ++st.bases_with_trips;
            const std::string tag = "seed " + std::to_string(seed) + " step " + std::to_string(step);
            for (int win : {1, 3}) {
                if (run_oracle) {
                    check_n7(w, cur, win, st, tag);
                    check_n8(w, cur, win, st, tag);
                }
            }
            // random walk over all four operators that can change the trip structure
            tdr::util::Rng walk(tdr::util::derive_seed(4243, tdr::util::SeedStream::Fuzz, static_cast<std::uint64_t>(seed * 100 + step)));
            const SearchContext ctx{net, inst.customer_data, init.flp_data, idx, 3};
            OperatorResult r;
            const double pick = walk.uniform01();
            if (!cur.drone_trips.empty() && pick < 0.35) {
                N8Options o; o.record_candidates = true;
                r = n8_drone_node_delete_truck_insert(cur, ctx, walk, o);
            } else if (pick < 0.7) {
                N7Options o; o.record_candidates = true;
                r = n7_truck_node_delete_drone_insert(cur, ctx, walk, o);
            } else if (!cur.drone_trips.empty() && pick < 0.85) {
                N5Options o; o.record_candidates = true;
                r = n5_td_service_node_swap(cur, ctx, walk, o);
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
    constexpr int kSeeds = 60;
    const Stats a = campaign(true, kSeeds);
    const Stats b = campaign(true, kSeeds);

    std::cout << "bases=" << a.bases << " with_trips=" << a.bases_with_trips << "\n"
              << "N7: calls=" << a.n7_calls << " candidates=" << a.n7_candidates << " accepted=" << a.n7_accepted << " improving_calls=" << a.n7_improving << " affected=" << a.n7_affected
              << " no_replacement=" << a.n7_noflp << " no_free_drone=" << a.n7_nofree << " endurance=" << a.n7_endurance << " other_eval=" << a.n7_eval_other
              << " reuse_of_onroute_flp=" << a.n7_reuse << " depot_recovery=" << a.n7_depot_recovery << " second_drone=" << a.n7_second_drone << "\n"
              << "N8: calls=" << a.n8_calls << " (windowed " << a.n8_windowed_calls << ") candidates=" << a.n8_candidates << " accepted=" << a.n8_accepted << " improving_calls=" << a.n8_improving
              << " affected=" << a.n8_affected << " endurance=" << a.n8_endurance << " other_eval=" << a.n8_eval_other << " flp_dropped_calls=" << a.n8_dropped
              << " candidates_outside_the_default_window=" << a.n8_outside_default_window << "\n";

    check(a.bases > 150 && a.bases_with_trips > 100, "non-vacuity: enough bases, many with trips");
    check(a.n7_candidates > 1000 && a.n7_accepted > 100 && a.n7_affected > 20, "non-vacuity: N7 accepted and affected candidates occur");
    check(a.n7_endurance > 0, "non-vacuity: N7 endurance rejections occur");
    check(a.n7_reuse > 0 && a.n7_depot_recovery > 0 && a.n7_second_drone > 0, "non-vacuity: N7 launch-FLP reuse, depot recovery and a non-zero drone id occur");
    check(a.n7_improving > 0, "non-vacuity: N7 finds strict improvements");
    check(a.n8_candidates > 300 && a.n8_accepted > 50 && a.n8_affected > 20, "non-vacuity: N8 accepted and affected candidates occur");
    check(a.n8_dropped > 0 && a.n8_outside_default_window > 0, "non-vacuity: N8 drops FLPs and generates candidates outside the D1 window (D5)");
    check(a.n8_windowed_calls > 0, "non-vacuity: the windowed N8 variant is exercised");
    check(a.n8_improving > 0, "non-vacuity: N8 finds strict improvements");
    check(a.bad_reason_flp_arc == 0 && a.bad_reason_flp_unused == 0, "operators never produce an FLP-off-arc or idle-FLP candidate that reaches the evaluator");
    check(a.bad_reason_fleet == 0, "no evaluator fleet-capacity rejection occurs (NoFreeDrone catches N7 first; N8 removes a trip)");
    check(a.digest == b.digest, "deterministic: identical digest over the whole campaign on a second full run (oracle on both)");

    std::cout << g_checks << " checks, " << g_failures << " failures\n";
    if (g_failures == 0) std::cout << "ALL TESTS PASSED\n";
    return g_failures == 0 ? 0 : 1;
}
