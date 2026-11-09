// M4 oracle test. For seeded random instances (and random-walk neighbours of their initial solutions, so that
// trips of many shapes occur) the operators N3 and N6 are compared RECORD BY RECORD with an independent brute-force
// enumeration written here with its own route/skeleton arithmetic (it calls none of the production operators'
// helpers: not detail_n3::drone_sequence, not PositionIndex, not trips_serving), judged by the FROZEN reference
// evaluator (tdr::timing_ref).
//
// Oracle contract (see docs/static/operators_N3_N6.md):
//   N3, per trip i of drone u: the trips of u ordered by (route position of the launch FLP, route position of the
//       recovery node after it, trip index); expected candidates = [predecessor, successor] (those that exist), each
//       the exchange of the two trips' drone customers. The order is cross-checked against the reference
//       evaluator's launch times (constraint (28): route order is time order).
//   N6, per trip i and window w: the half-step coordinate h of every drone customer is 2g+1, g = (number of skeleton
//       nodes up to and including the one before the launch FLP) - 1; partners = trips j != i with |h_i-h_j| <= 2w,
//       (default) of a different drone, ordered by (|h_i-h_j|, customer id); each the same exchange.
//   Every candidate is judged by the reference: feasible -> accepted with the reference objective; infeasible ->
//   an evaluator reason (EvalEndurance iff some trip violates endurance in the reference). Best = the first
//   candidate attaining the minimum feasible objective (strict <).

#include "config/static_config.hpp"
#include "flp/flp_arc_index.hpp"
#include "init/initial_solution_builder.hpp"
#include "instance/static_instance.hpp"
#include "reference/timing_engine_reference.hpp"
#include "search/neighbourhood_n3.hpp"
#include "search/neighbourhood_n4.hpp"
#include "search/neighbourhood_n5.hpp"
#include "search/neighbourhood_n6.hpp"
#include "search/neighbourhood_n7.hpp"
#include "search/neighbourhood_n8.hpp"
#include "util/rng.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
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

struct Ref {
    bool feasible;
    double objective;
    bool endurance_violated;
    std::vector<double> launch_time;
};
Ref ref_eval(const World& w, const Solution& s) {
    const auto e = tdr::timing_ref::evaluate_solution(s, *w.net, *w.cd, *w.flp);
    Ref r{e.feasible, e.objective_value, false, {}};
    for (const auto& t : e.drone_trip_timings) {
        r.launch_time.push_back(t.launch_time);
        if (t.structurally_valid && t.on_route && t.has_customer_data && t.drone_id_valid && !t.endurance_feasible) r.endurance_violated = true;
    }
    return r;
}

bool close(double a, double b) { return std::fabs(a - b) <= 1e-6 * std::max(1.0, std::fabs(b)); }

struct Stats {
    long scrambles = 0, bases = 0, bases_with_trips = 0, bases_with_two_drones_used = 0, bases_with_3_same_drone = 0;
    long n3_calls = 0, n3_candidates = 0, n3_accepted = 0, n3_endurance = 0, n3_eval_other = 0, n3_improving = 0, n3_both_neighbours = 0, n3_lone = 0, n3_noncontiguous_vector = 0, n3_ties = 0, n3_other_drone_between = 0;
    long n6_calls = 0, n6_candidates = 0, n6_accepted = 0, n6_endurance = 0, n6_eval_other = 0, n6_improving = 0, n6_same_drone_candidates = 0, n6_window_limited = 0, n6_truck_in_window = 0, n6_ties = 0, n6_empty = 0;
    long bad_flp_arc = 0, bad_flp_unused = 0, bad_fleet = 0, bad_seq = 0, bad_construction = 0;
    std::uint64_t digest = 14695981039346656037ULL;
    void mix(std::uint64_t v) { digest = (digest ^ v) * 1099511628211ULL; }
};

void tally(Stats& st, const OperatorResult& r) {
    st.bad_flp_arc += static_cast<long>(r.rejected[static_cast<std::size_t>(RejectReason::EvalFlpArc)]);
    st.bad_flp_unused += static_cast<long>(r.rejected[static_cast<std::size_t>(RejectReason::EvalFlpUnused)]);
    st.bad_fleet += static_cast<long>(r.rejected[static_cast<std::size_t>(RejectReason::EvalFleetCapacity)]);
    st.bad_seq += static_cast<long>(r.rejected[static_cast<std::size_t>(RejectReason::EvalDroneSequencing)]);
    for (RejectReason x : {RejectReason::AffectedTrips, RejectReason::NoFreeDrone, RejectReason::NoReplacementFlp, RejectReason::InvalidRecovery})
        st.bad_construction += static_cast<long>(r.rejected[static_cast<std::size_t>(x)]);
}

void mix_result(Stats& st, const OperatorResult& r) {
    st.mix(r.generated); st.mix(r.evaluated); st.mix(r.accepted); st.mix(r.duplicates_skipped);
    for (auto x : r.rejected) st.mix(x);
    if (r.best) {
        st.mix(static_cast<std::uint64_t>(r.best->move.partner_trip));
        for (const auto& t : r.best->solution.drone_trips) { st.mix(static_cast<std::uint64_t>(t.drone_id)); st.mix(static_cast<std::uint64_t>(t.service_customer)); }
    }
}

// ---- independent route arithmetic -------------------------------------------------------------------------
std::size_t route_pos(const TruckRoute& r, NodeId v, std::size_t from = 0) {
    for (std::size_t i = from; i < r.size(); ++i) if (r[i] == v) return i;
    return r.size();
}

// h(customer): truck customer at skeleton index s -> 2s; drone customer launched from a FLP in gap g -> 2g+1.
std::map<NodeId, long> half_steps(const World& w, const Solution& s) {
    std::map<NodeId, long> h;
    long skeleton_seen = 0;
    std::map<NodeId, long> gap_of_flp;
    for (NodeId v : s.truck_route) {
        if (is_flp(w, v)) {
            gap_of_flp[v] = skeleton_seen - 1;
        } else {
            if (w.nodes->at(v).type == NodeType::Customer) h[v] = 2 * skeleton_seen;
            ++skeleton_seen;
        }
    }
    for (const auto& t : s.drone_trips) h[t.service_customer] = 2 * gap_of_flp.at(t.launch_node) + 1;
    return h;
}

// Judge one expected exchange against the production record.
struct Expect { std::size_t i, j; };

void judge_result(const World& w, const Solution& base, const OperatorResult& res, const std::vector<Expect>& want, MoveKind kind, OperatorId op, std::size_t anchor,
                  const std::string& where, long& cand, long& acc, long& endu, long& other, long& improving, long& ties) {
    const auto base_ref = ref_eval(w, base);
    check(res.status == OperatorStatus::Ok && res.op == op && res.anchor_trip == anchor && res.anchor_customer == base.drone_trips[anchor].service_customer, where + ": status, operator id, anchor");
    check(close(res.base_objective, base_ref.objective), where + ": base objective == frozen reference");
    check(res.generated == want.size() && res.log.size() == want.size(), where + ": candidate count == brute force (" + std::to_string(res.generated) + " vs " + std::to_string(want.size()) + ")");
    check(res.duplicates_skipped == 0 && res.evaluated == res.generated && res.accepted + res.rejected_total() == res.generated, where + ": one outcome per candidate, all evaluated, no duplicates");
    if (res.log.size() != want.size()) return;

    std::optional<std::size_t> best_i;
    double best_obj = 0;
    std::size_t feasible_count = 0;
    for (std::size_t k = 0; k < want.size(); ++k) {
        const auto& rec = res.log[k];
        ++cand;
        Solution exp = base;
        std::swap(exp.drone_trips[want[k].i].service_customer, exp.drone_trips[want[k].j].service_customer);
        const std::string cw = where + " cand " + std::to_string(k);
        check(rec.move.kind == kind && rec.move.replaced_trip == want[k].i && rec.move.partner_trip == want[k].j && rec.move.customer == base.drone_trips[want[k].i].service_customer &&
                  rec.move.partner == base.drone_trips[want[k].j].service_customer, cw + ": move descriptor");
        bool same = rec.solution.truck_route == exp.truck_route && rec.solution.drone_trips.size() == exp.drone_trips.size();
        for (std::size_t t = 0; same && t < exp.drone_trips.size(); ++t) {
            const auto &a = rec.solution.drone_trips[t], &b = exp.drone_trips[t];
            same = a.drone_id == b.drone_id && a.launch_node == b.launch_node && a.service_customer == b.service_customer && a.recovery_node == b.recovery_node;
        }
        check(same, cw + ": candidate = base with the two customers exchanged (route, launch, recovery, drone ids unchanged)");
        check(covers_once(w, rec.solution) && no_foreign_flp(w, rec.solution.truck_route), cw + ": exactly-once coverage, no foreign FLPs");
        const auto v = ref_eval(w, exp);
        check(rec.evaluated, cw + ": evaluated");
        if (v.feasible) {
            ++acc;
            ++feasible_count;
            check(rec.reason == RejectReason::None && close(rec.objective, v.objective), cw + ": reference-feasible -> accepted with the reference objective");
            if (!best_i || v.objective < best_obj) { best_i = k; best_obj = v.objective; }
            else if (v.objective == best_obj) ++ties;
        } else {
            const bool is_eval = std::string(reject_reason_name(rec.reason)).rfind("eval-", 0) == 0;
            check(is_eval, cw + ": reference-infeasible -> rejected with an evaluator reason (" + reject_reason_name(rec.reason) + ")");
            check((rec.reason == RejectReason::EvalEndurance) == v.endurance_violated, cw + ": EvalEndurance <=> the reference sees an endurance violation");
            if (rec.reason == RejectReason::EvalEndurance) ++endu; else ++other;
        }
    }
    check(res.accepted == feasible_count, where + ": accepted count == number of reference-feasible candidates");
    check(static_cast<bool>(best_i) == res.best.has_value(), where + ": best exists iff some candidate is feasible");
    if (best_i && res.best) {
        check(close(res.best->objective, best_obj) && res.best->move.partner_trip == want[*best_i].j, where + ": best = the FIRST candidate attaining the minimum (strict <)");
        if (res.improves()) ++improving;
    }
}

// ---- N3 ----
void check_n3(const World& w, const Solution& base, Stats& st, const std::string& tag) {
    const auto base_ref = ref_eval(w, base);
    for (std::size_t i = 0; i < base.drone_trips.size(); ++i) {
        const int u = base.drone_trips[i].drone_id;
        std::vector<std::tuple<std::size_t, std::size_t, std::size_t>> seq; // (launch pos, recovery pos, index)
        for (std::size_t k = 0; k < base.drone_trips.size(); ++k) {
            if (base.drone_trips[k].drone_id != u) continue;
            const std::size_t lp = route_pos(base.truck_route, base.drone_trips[k].launch_node);
            const std::size_t rp = route_pos(base.truck_route, base.drone_trips[k].recovery_node, lp + 1);
            seq.emplace_back(lp, rp, k);
        }
        std::sort(seq.begin(), seq.end());
        // constraint (28): route order is time order -> the reference's launch times must increase along the sequence
        for (std::size_t q = 1; q < seq.size(); ++q)
            check(base_ref.launch_time[std::get<2>(seq[q - 1])] < base_ref.launch_time[std::get<2>(seq[q])], tag + ": drone sequence is in strictly increasing reference launch time");
        std::size_t at = 0;
        while (std::get<2>(seq[at]) != i) ++at;
        std::vector<Expect> want;
        if (at > 0) want.push_back({i, std::get<2>(seq[at - 1])});
        if (at + 1 < seq.size()) want.push_back({i, std::get<2>(seq[at + 1])});

        N3Options opt;
        opt.record_candidates = true;
        // N3 must not depend on the window
        for (int win : {0, 3}) {
            const auto res = n3_drone_service_node_relocate(base, SearchContext{*w.net, *w.cd, *w.flp, *w.idx, win}, i, opt);
            const std::string where = tag + " N3 trip " + std::to_string(i) + " w=" + std::to_string(win);
            if (win == 3) {
                ++st.n3_calls;
                tally(st, res);
                mix_result(st, res);
                if (want.empty()) ++st.n3_lone;
                if (want.size() == 2) ++st.n3_both_neighbours;
                for (const auto& e : want) if (e.j + 1 != i && e.j != i + 1) ++st.n3_noncontiguous_vector;
                for (const auto& e : want) {
                    // another drone's trip launches between the two trips in the route
                    const std::size_t a = std::min(route_pos(base.truck_route, base.drone_trips[e.i].launch_node), route_pos(base.truck_route, base.drone_trips[e.j].launch_node));
                    const std::size_t b = std::max(route_pos(base.truck_route, base.drone_trips[e.i].launch_node), route_pos(base.truck_route, base.drone_trips[e.j].launch_node));
                    for (const auto& t : base.drone_trips) {
                        const std::size_t p = route_pos(base.truck_route, t.launch_node);
                        if (t.drone_id != u && p > a && p < b) { ++st.n3_other_drone_between; break; }
                    }
                }
            }
            judge_result(w, base, res, want, MoveKind::N3Exchange, OperatorId::N3, i, where, st.n3_candidates, st.n3_accepted, st.n3_endurance, st.n3_eval_other, st.n3_improving, st.n3_ties);
        }
    }
}

// ---- N6 ----
void check_n6(const World& w, const Solution& base, int window, Stats& st, const std::string& tag) {
    const auto h = half_steps(w, base);
    for (std::size_t i = 0; i < base.drone_trips.size(); ++i) {
        for (bool same_drone : {false, true}) {
            std::vector<std::tuple<long, NodeId, std::size_t>> found; // (|dh|, customer id, trip)
            long skipped_same = 0;
            for (std::size_t j = 0; j < base.drone_trips.size(); ++j) {
                if (j == i) continue;
                const long d = std::labs(h.at(base.drone_trips[i].service_customer) - h.at(base.drone_trips[j].service_customer));
                if (d > 2L * window) continue;
                if (!same_drone && base.drone_trips[j].drone_id == base.drone_trips[i].drone_id) { ++skipped_same; continue; }
                found.emplace_back(d, base.drone_trips[j].service_customer, j);
            }
            std::sort(found.begin(), found.end());
            std::vector<Expect> want;
            for (const auto& f : found) want.push_back({i, std::get<2>(f)});

            N6Options opt;
            opt.record_candidates = true;
            opt.include_same_drone = same_drone;
            const auto res = n6_drone_drone_swap(base, SearchContext{*w.net, *w.cd, *w.flp, *w.idx, window}, i, opt);
            const std::string where = tag + " N6 trip " + std::to_string(i) + " w=" + std::to_string(window) + (same_drone ? " +same" : "");
            if (!same_drone) {
                ++st.n6_calls;
                tally(st, res);
                mix_result(st, res);
                if (want.empty()) ++st.n6_empty;
                if (skipped_same > 0) ++st.n6_window_limited; // reused as: same-drone partners inside the window that were excluded
                for (const auto& kv : h) {
                    bool is_truck = true;
                    for (const auto& t : base.drone_trips) if (t.service_customer == kv.first) is_truck = false;
                    if (is_truck && std::labs(h.at(base.drone_trips[i].service_customer) - kv.second) <= 2L * window) { ++st.n6_truck_in_window; break; }
                }
            } else {
                for (const auto& f : found) if (base.drone_trips[std::get<2>(f)].drone_id == base.drone_trips[i].drone_id) ++st.n6_same_drone_candidates;
                // A8 (revised at the M4 sign-off): the DEFAULT options are the unrestricted ones
                N6Options dflt;
                dflt.record_candidates = true;
                const auto rd = n6_drone_drone_swap(base, SearchContext{*w.net, *w.cd, *w.flp, *w.idx, window}, i, dflt);
                bool same_list = rd.generated == res.generated && rd.accepted == res.accepted && rd.log.size() == res.log.size();
                for (std::size_t k = 0; same_list && k < rd.log.size(); ++k) same_list = rd.log[k].move.partner_trip == res.log[k].move.partner_trip;
                check(same_list, where + ": default N6Options == include_same_drone = true");
            }
            long c = 0, a = 0, e = 0, o = 0, im = 0, ti = 0;
            judge_result(w, base, res, want, MoveKind::N6Swap, OperatorId::N6, i, where, c, a, e, o, im, ti);
            if (!same_drone) {
                st.n6_candidates += c; st.n6_accepted += a; st.n6_endurance += e; st.n6_eval_other += o; st.n6_improving += im; st.n6_ties += ti;
            }
        }
    }
}

// ------------------------------------------------------------------ campaign
Stats campaign(int seeds) {
    Stats st;
    for (int seed = 0; seed < seeds; ++seed) {
        tdr::util::Rng fz(tdr::util::derive_seed(5252, tdr::util::SeedStream::Fuzz, static_cast<std::uint64_t>(seed)));
        tdr::config::StaticConfig cfg;
        cfg.seed.base_seed = 5252;
        cfg.instance.num_customers = static_cast<int>(fz.uniform_int(8, 14));
        cfg.instance.dataset_index = seed;
        cfg.vehicle.num_drones = static_cast<int>(fz.uniform_int(seed % 5 == 0 ? 1 : 2, 3));
        // variants so that feasible exchanges occur (paper-scale 60 km / 30 min is almost always infeasible at this n)
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
                std::map<int, int> per;
                for (const auto& t : cur.drone_trips) { used.insert(t.drone_id); ++per[t.drone_id]; }
                if (used.size() >= 2) ++st.bases_with_two_drones_used;
                for (const auto& kv : per) if (kv.second >= 3) { ++st.bases_with_3_same_drone; break; }
            }
            const std::string tag = "seed " + std::to_string(seed) + " step " + std::to_string(step);
            if (!cur.drone_trips.empty() && cfg.vehicle.num_drones >= 2 && step % 2 == 1) {
                // any drone-id assignment that the reference evaluator accepts is a legal solution; the lowest-free
                // assignment of the builders rarely uses a second drone, so re-draw the ids (a few tries)
                tdr::util::Rng sc(tdr::util::derive_seed(5254, tdr::util::SeedStream::Fuzz, static_cast<std::uint64_t>(seed * 100 + step)));
                for (int attempt = 0; attempt < 6; ++attempt) {
                    Solution trial = cur;
                    for (auto& t : trial.drone_trips) t.drone_id = static_cast<int>(sc.uniform_int(0, cfg.vehicle.num_drones - 1));
                    if (ref_eval(w, trial).feasible) { cur = trial; ++st.scrambles; break; }
                }
            }
            if (!cur.drone_trips.empty()) {
                check_n3(w, cur, st, tag);
                for (int win : {0, 1, 3}) check_n6(w, cur, win, st, tag);
            }
            // random walk over all six operators so that trips of many shapes occur
            tdr::util::Rng walk(tdr::util::derive_seed(5253, tdr::util::SeedStream::Fuzz, static_cast<std::uint64_t>(seed * 100 + step)));
            const SearchContext ctx{net, inst.customer_data, init.flp_data, idx, 3};
            OperatorResult r;
            const double pick = walk.uniform01();
            if (!cur.drone_trips.empty() && pick < 0.15) {
                N8Options o; o.record_candidates = true;
                r = n8_drone_node_delete_truck_insert(cur, ctx, walk, o);
            } else if (pick < 0.55) {
                N7Options o; o.record_candidates = true;
                r = n7_truck_node_delete_drone_insert(cur, ctx, walk, o);
            } else if (!cur.drone_trips.empty() && pick < 0.65) {
                N5Options o; o.record_candidates = true;
                r = n5_td_service_node_swap(cur, ctx, walk, o);
            } else if (!cur.drone_trips.empty() && pick < 0.8) {
                N3Options o; o.record_candidates = true;
                r = n3_drone_service_node_relocate(cur, ctx, walk, o);
            } else if (!cur.drone_trips.empty() && pick < 0.95) {
                N6Options o; o.record_candidates = true;
                o.include_same_drone = walk.uniform01() < 0.3;
                r = n6_drone_drone_swap(cur, ctx, walk, o);
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
              << "bases=" << a.bases << " with_trips=" << a.bases_with_trips << " two_drones_used=" << a.bases_with_two_drones_used << " three_trips_on_one_drone=" << a.bases_with_3_same_drone << "\n"
              << "N3: calls=" << a.n3_calls << " candidates=" << a.n3_candidates << " accepted=" << a.n3_accepted << " improving_calls=" << a.n3_improving << " endurance=" << a.n3_endurance
              << " other_eval=" << a.n3_eval_other << " both_neighbours=" << a.n3_both_neighbours << " lone=" << a.n3_lone << " partner_not_vector_adjacent=" << a.n3_noncontiguous_vector
              << " other_drone_launches_between=" << a.n3_other_drone_between << " candidate_ties=" << a.n3_ties << "\n"
              << "N6: calls(w=0,1,3)=" << a.n6_calls << " candidates=" << a.n6_candidates << " accepted=" << a.n6_accepted << " improving_calls=" << a.n6_improving << " endurance=" << a.n6_endurance
              << " other_eval=" << a.n6_eval_other << " empty_calls=" << a.n6_empty << " calls_excluding_same_drone_partner=" << a.n6_window_limited << " same_drone_candidates(+same)=" << a.n6_same_drone_candidates
              << " calls_with_truck_customer_in_window=" << a.n6_truck_in_window << " candidate_ties=" << a.n6_ties << "\n";

    check(a.bases > 300 && a.bases_with_trips > 200, "non-vacuity: enough bases, many with trips");
    check(a.bases_with_two_drones_used > 30 && a.bases_with_3_same_drone > 5, "non-vacuity: bases with two drones in use and with three trips on one drone occur");
    check(a.n3_candidates > 200 && a.n3_accepted > 30, "non-vacuity: N3 candidates and accepted candidates occur");
    check(a.n3_endurance > 0, "non-vacuity: N3 endurance rejections occur");
    check(a.n3_both_neighbours > 5 && a.n3_lone > 20, "non-vacuity: N3 anchors with both neighbours and lone anchors occur");
    check(a.n3_improving > 0, "non-vacuity: N3 finds strict improvements");
    check(a.n3_other_drone_between > 0, "non-vacuity: N3 pairs with another drone's launch between them (consecutive on the drone, not in the route) occur");
    check(a.n6_candidates > 300 && a.n6_accepted > 50, "non-vacuity: N6 candidates and accepted candidates occur");
    check(a.n6_endurance > 0, "non-vacuity: N6 endurance rejections occur");
    check(a.n6_improving > 0, "non-vacuity: N6 finds strict improvements");
    check(a.n6_window_limited > 10 && a.n6_same_drone_candidates > 10, "non-vacuity: same-drone partners inside the window occur (excluded by default, admitted with include_same_drone)");
    check(a.n6_truck_in_window > 50, "non-vacuity: truck customers inside the window occur (never partners)");
    check(a.n6_empty > 20, "non-vacuity: N6 anchors with no partner occur");
    check(a.bad_flp_arc == 0 && a.bad_flp_unused == 0 && a.bad_fleet == 0 && a.bad_seq == 0 && a.bad_construction == 0,
          "an exchange never produces an FLP-arc, idle-FLP, fleet-capacity or drone-sequencing rejection, nor any construction-time rejection");
    check(a.digest == b.digest, "deterministic: identical digest over the whole campaign on a second full run (oracle on both)");

    std::cout << g_checks << " checks, " << g_failures << " failures\n";
    if (g_failures == 0) std::cout << "ALL TESTS PASSED\n";
    return g_failures == 0 ? 0 : 1;
}
