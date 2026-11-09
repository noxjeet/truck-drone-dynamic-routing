// M2 oracle test. For seeded random instances (and random-walk neighbours of
// their initial solutions, so that trips of many shapes occur) the operators
// N4 and N5 are compared RECORD BY RECORD with an independent brute-force
// enumeration written here with its own route/skeleton arithmetic, judged by
// the FROZEN reference evaluator (tdr::timing_ref), not the production one.
//
// Oracle contract (see docs/static/operators_N4_N5.md):
//   N4: candidates = relocations (gaps in the window, incident gaps excluded,
//       ascending) followed by exchanges (truck partners within the window,
//       nearest first, ties by id); duplicates by route equality removed.
//       Policy Reject (check_n4): a candidate with an FLP off its arc is
//       AffectedTrips; otherwise it is judged by the evaluator with the base
//       trips unchanged.
//       Policy Reanchor, the default (check_n4_reanchor): recovery relationships
//       are checked first (a recovery not after its launch -> InvalidRecovery);
//       trips whose launch FLP is off its arc are re-anchored on the FLPs of the
//       arc they now sit on (ratio order), jointly over all stale trips (last
//       stale trip fastest), each variant judged by the reference evaluator, the
//       first minimum wins; no FLP on that arc -> NoReplacementFlp. Drone id,
//       customer and recovery never change; unaffected trips are untouched.
//   N5: per partner t (truck customers within the window of the trip's
//       half-step, nearest first, ties by id): d replaces t on the route, the
//       dissolved trip and then-unused FLPs are dropped; if a foreign FLP or a
//       trip recovering at t remains -> ONE AffectedTrips record; else one
//       candidate per (gap in the window of 2*s_t) x (FLP of that arc) with
//       recovery at the next truck node and the lowest drone id that is
//       reference-feasible.

#include "config/static_config.hpp"
#include "flp/flp_arc_index.hpp"
#include "init/initial_solution_builder.hpp"
#include "instance/static_instance.hpp"
#include "reference/timing_engine_reference.hpp"
#include "search/neighbourhood_n4.hpp"
#include "search/neighbourhood_n5.hpp"
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

struct Stats {
    long bases = 0, bases_with_trips = 0;
    long n4_calls = 0, n4_candidates = 0, n4_accepted = 0, n4_affected = 0, n4_eval_rejected = 0, n4_trip_order = 0;
    long n5_calls = 0, n5_candidates = 0, n5_accepted = 0, n5_affected = 0, n5_nofree = 0, n5_endurance = 0, n5_eval_rejected = 0, n5_reuse = 0, n5_depot_recovery = 0;
    long n5_second_drone = 0;
    long n4r_candidates = 0, n4r_stale = 0, n4r_reanchored = 0, n4r_trips = 0, n4r_variants = 0, n4r_noflp = 0, n4r_invrec = 0, n4r_eval_rejected = 0, n4r_accepted = 0, n4r_multi = 0, n4r_capped = 0, n4r_ties = 0;
    long bad_reason_flp_arc = 0, bad_reason_flp_unused = 0;
    std::uint64_t digest = 14695981039346656037ULL;
    void mix(std::uint64_t v) { digest = (digest ^ v) * 1099511628211ULL; }
};

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

using TripKey = std::tuple<int, NodeId, NodeId, NodeId>;
std::vector<TripKey> trip_keys(const Solution& s) {
    std::vector<TripKey> k;
    for (const auto& t : s.drone_trips) k.emplace_back(t.drone_id, t.launch_node, t.service_customer, t.recovery_node);
    std::sort(k.begin(), k.end());
    return k;
}

struct RefVerdict { bool feasible; double objective; };
RefVerdict ref_eval(const World& w, const Solution& s) {
    const auto e = tdr::timing_ref::evaluate_solution(s, *w.net, *w.cd, *w.flp);
    return {e.feasible, e.objective_value};
}

bool close(double a, double b) { return std::fabs(a - b) <= 1e-6 * std::max(1.0, std::fabs(b)); }

void tally(Stats& st, const OperatorResult& r) {
    st.bad_reason_flp_arc += static_cast<long>(r.rejected[static_cast<std::size_t>(RejectReason::EvalFlpArc)]);
    st.bad_reason_flp_unused += static_cast<long>(r.rejected[static_cast<std::size_t>(RejectReason::EvalFlpUnused)]);
}

void mix_result(Stats& st, const OperatorResult& r) {
    st.mix(r.generated); st.mix(r.evaluated); st.mix(r.accepted); st.mix(r.duplicates_skipped);
    for (auto x : r.rejected) st.mix(x);
    if (r.best) {
        for (NodeId v : r.best->solution.truck_route) st.mix(static_cast<std::uint64_t>(v));
        for (const auto& t : r.best->solution.drone_trips) { st.mix(static_cast<std::uint64_t>(t.drone_id)); st.mix(static_cast<std::uint64_t>(t.launch_node)); st.mix(static_cast<std::uint64_t>(t.recovery_node)); }
    }
}

// ------------------------------------------------------------------ N4 oracle
void check_n4(const World& w, const Solution& base, int window, Stats& st, const std::string& tag) {
    const auto sp = skel_positions(w, base.truck_route);
    for (std::size_t s = 1; s + 1 < sp.size(); ++s) {
        const NodeId c = base.truck_route[sp[s]];
        if (w.nodes->at(c).type != NodeType::Customer) continue;

        std::vector<TruckRoute> routes;
        std::vector<int> kind; // 0 relocate, 1 exchange
        std::vector<std::size_t> gap_of;
        std::vector<NodeId> partner_of;
        const auto add = [&](const TruckRoute& r, int k, std::size_t g, NodeId p) {
            if (std::find(routes.begin(), routes.end(), r) != routes.end()) return;
            routes.push_back(r); kind.push_back(k); gap_of.push_back(g); partner_of.push_back(p);
        };
        for (std::size_t g = 0; g + 1 < sp.size(); ++g) {
            if (g + 1 == s || g == s) continue;
            const long dh = std::labs(static_cast<long>(2 * g + 1) - static_cast<long>(2 * s));
            if (dh > 2L * window) continue;
            TruckRoute r = base.truck_route;
            r.erase(r.begin() + static_cast<long>(sp[s]));
            const auto sp2 = skel_positions(w, r);
            const std::size_t g2 = (g + 1 <= s) ? g : g - 1; // g <= s-2 -> g ; g >= s+1 -> g-1
            r.insert(r.begin() + static_cast<long>(sp2[g2 + 1]), c);
            add(r, 0, g, kInvalidNodeId);
        }
        std::vector<std::pair<long, NodeId>> partners;
        for (std::size_t k = 1; k + 1 < sp.size(); ++k) {
            if (k == s) continue;
            const NodeId p = base.truck_route[sp[k]];
            const long d = std::labs(static_cast<long>(k) - static_cast<long>(s));
            if (d <= window) partners.emplace_back(d, p);
        }
        std::sort(partners.begin(), partners.end());
        for (const auto& pp : partners) {
            TruckRoute r = base.truck_route;
            std::size_t pk = 0;
            for (std::size_t k = 0; k < sp.size(); ++k) if (base.truck_route[sp[k]] == pp.second) pk = k;
            std::swap(r[sp[s]], r[sp[pk]]);
            add(r, 1, kNoIndex, pp.second);
        }

        N4Options opt;
        opt.record_candidates = true;
        opt.affected_trips = AffectedTripPolicy::Reject; // the original M2 contract, unchanged
        const auto res = n4_truck_service_node_relocate(base, SearchContext{*w.net, *w.cd, *w.flp, *w.idx, window}, c, opt);
        ++st.n4_calls;
        tally(st, res);
        mix_result(st, res);
        const std::string where = tag + " N4 anchor C" + std::to_string(c) + " w=" + std::to_string(window);
        check(res.status == OperatorStatus::Ok, where + ": status ok");
        check(res.generated == routes.size() && res.log.size() == routes.size(), where + ": candidate count == brute force (" + std::to_string(res.generated) + " vs " + std::to_string(routes.size()) + ")");
        if (res.log.size() != routes.size()) continue;
        std::optional<std::size_t> best;
        double best_obj = 0;
        for (std::size_t i = 0; i < routes.size(); ++i) {
            const auto& rec = res.log[i];
            ++st.n4_candidates;
            check(rec.solution.truck_route == routes[i], where + ": candidate " + std::to_string(i) + " route equals the brute-force route (same order)");
            check(static_cast<int>(rec.move.kind) == (kind[i] == 0 ? static_cast<int>(MoveKind::N4Relocate) : static_cast<int>(MoveKind::N4Exchange)), where + ": move kind");
            check(trip_keys(rec.solution) == trip_keys(base), where + ": trips untouched in candidate " + std::to_string(i));
            check(covers_once(w, rec.solution), where + ": exactly-once coverage");
            const auto foreign = foreign_flps(w, routes[i]);
            if (!foreign.empty()) {
                ++st.n4_affected;
                check(rec.reason == RejectReason::AffectedTrips && !rec.evaluated, where + ": foreign FLP -> AffectedTrips, not evaluated");
                std::vector<std::size_t> expect;
                for (std::size_t t = 0; t < base.drone_trips.size(); ++t)
                    if (std::find(foreign.begin(), foreign.end(), base.drone_trips[t].launch_node) != foreign.end()) expect.push_back(t);
                check(rec.affected_trips == expect, where + ": affected trip list == trips launching from foreign FLPs");
                bool ref_feasible = true;
                { const auto v = ref_eval(w, rec.solution); ref_feasible = v.feasible; }
                check(!ref_feasible, where + ": the reference evaluator also rejects an affected candidate");
            } else {
                const auto v = ref_eval(w, routes[i] == base.truck_route ? base : rec.solution);
                check(rec.evaluated, where + ": unaffected candidate is evaluated");
                if (v.feasible) {
                    ++st.n4_accepted;
                    check(rec.reason == RejectReason::None && close(rec.objective, v.objective), where + ": reference-feasible -> accepted, objective equal (" + std::to_string(rec.objective) + " vs " + std::to_string(v.objective) + ")");
                    if (!best || v.objective < best_obj) { best = i; best_obj = v.objective; }
                } else {
                    ++st.n4_eval_rejected;
                    if (rec.reason == RejectReason::EvalTripOrder) ++st.n4_trip_order;
                    check(rec.reason != RejectReason::None && rec.reason != RejectReason::AffectedTrips, where + ": reference-infeasible -> rejected by an evaluator reason");
                }
            }
        }
        check(static_cast<bool>(best) == res.best.has_value(), where + ": best exists iff some candidate is feasible");
        if (best && res.best) {
            check(res.best->solution.truck_route == routes[*best] && close(res.best->objective, best_obj), where + ": best = first-generated minimum");
        }
    }
}


bool exactly_once_ok(const World& w, const Solution& s) { return covers_once(w, s); }

// ------------------------------------------------------- N4 oracle, Reanchor policy
// Everything below is written from the contract in the header, with its own route arithmetic.
bool is_eval_reason(RejectReason r) { return std::string(reject_reason_name(r)).rfind("eval-", 0) == 0; }

void check_n4_reanchor(const World& w, const Solution& base, int window, Stats& st, const std::string& tag, std::optional<std::size_t> cap = std::nullopt) {
    const auto sp = skel_positions(w, base.truck_route);
    for (std::size_t s = 1; s + 1 < sp.size(); ++s) {
        const NodeId c = base.truck_route[sp[s]];
        if (w.nodes->at(c).type != NodeType::Customer) continue;

        // --- the same candidate routes as check_n4 (independent construction) ---
        std::vector<TruckRoute> routes;
        const auto add = [&](const TruckRoute& r) { if (std::find(routes.begin(), routes.end(), r) == routes.end()) routes.push_back(r); };
        for (std::size_t g = 0; g + 1 < sp.size(); ++g) {
            if (g + 1 == s || g == s) continue;
            if (std::labs(static_cast<long>(2 * g + 1) - static_cast<long>(2 * s)) > 2L * window) continue;
            TruckRoute r = base.truck_route;
            r.erase(r.begin() + static_cast<long>(sp[s]));
            const auto sp2 = skel_positions(w, r);
            const std::size_t g2 = (g + 1 <= s) ? g : g - 1;
            r.insert(r.begin() + static_cast<long>(sp2[g2 + 1]), c);
            add(r);
        }
        std::vector<std::pair<long, NodeId>> partners;
        for (std::size_t k = 1; k + 1 < sp.size(); ++k) {
            if (k == s) continue;
            const long d = std::labs(static_cast<long>(k) - static_cast<long>(s));
            if (d <= window) partners.emplace_back(d, base.truck_route[sp[k]]);
        }
        std::sort(partners.begin(), partners.end());
        for (const auto& pp : partners) {
            TruckRoute r = base.truck_route;
            std::size_t pk = 0;
            for (std::size_t k = 0; k < sp.size(); ++k) if (base.truck_route[sp[k]] == pp.second) pk = k;
            std::swap(r[sp[s]], r[sp[pk]]);
            add(r);
        }

        N4Options opt; // default policy: Reanchor, default budget 100
        opt.record_candidates = true;
        if (cap) opt.max_reanchor_variants = *cap;
        check(opt.affected_trips == AffectedTripPolicy::Reanchor, "default policy is Reanchor");
        // the oracle's own reading of the budget: 100 by default (approved), 0 = unlimited
        const std::size_t budget = cap ? (*cap == 0 ? static_cast<std::size_t>(-1) : *cap) : 100;
        const auto res = n4_truck_service_node_relocate(base, SearchContext{*w.net, *w.cd, *w.flp, *w.idx, window}, c, opt);
        ++st.n4_calls;
        mix_result(st, res);
        const std::string where = tag + " N4r anchor C" + std::to_string(c) + " w=" + std::to_string(window);
        check(res.status == OperatorStatus::Ok && res.generated == routes.size() && res.log.size() == routes.size(), where + ": candidate count == brute force");
        if (res.log.size() != routes.size()) continue;

        long o_stale = 0, o_reanch = 0, o_trips = 0, o_variants = 0, o_acc = 0, o_evaluated = 0, o_capped = 0;
        std::array<long, kRejectReasonCount> o_rej{};
        std::optional<std::size_t> best;
        double best_obj = 0;

        for (std::size_t i = 0; i < routes.size(); ++i) {
            const auto& rec = res.log[i];
            const TruckRoute& rt = routes[i];
            ++st.n4r_candidates;
            check(static_cast<bool>(rec.move.kind == MoveKind::N4Relocate || rec.move.kind == MoveKind::N4Exchange), where + ": move kind");

            // stale trips (launch FLP off its arc), ascending by trip index
            const auto foreign = foreign_flps(w, rt);
            std::vector<std::size_t> stale;
            for (std::size_t t = 0; t < base.drone_trips.size(); ++t)
                if (std::find(foreign.begin(), foreign.end(), base.drone_trips[t].launch_node) != foreign.end()) stale.push_back(t);
            if (!foreign.empty()) ++o_stale;

            // recovery relationship: the recovery must occur at a route position after the launch's position
            std::vector<std::size_t> bad_rec;
            for (std::size_t t = 0; t < base.drone_trips.size(); ++t) {
                const auto& tr = base.drone_trips[t];
                const auto lp = std::find(rt.begin(), rt.end(), tr.launch_node);
                bool ok = lp != rt.end();
                if (ok) ok = std::find(lp + 1, rt.end(), tr.recovery_node) != rt.end();
                if (!ok) bad_rec.push_back(t);
            }

            check(exactly_once_ok(w, rec.solution), where + ": exactly-once coverage in the shown solution");
            if (!bad_rec.empty()) {
                ++st.n4r_invrec; ++o_rej[static_cast<std::size_t>(RejectReason::InvalidRecovery)];
                check(rec.reason == RejectReason::InvalidRecovery && !rec.evaluated && rec.variants_generated == 0 && rec.reanchored.empty(), where + ": recovery not after launch -> InvalidRecovery, not evaluated, nothing re-anchored");
                check(rec.affected_trips == bad_rec, where + ": affected trips == trips with an invalid recovery");
                check(rec.solution.truck_route == rt && trip_keys(rec.solution) == trip_keys(base), where + ": invalid-recovery candidate shown UNREPAIRED");
                check(!ref_eval(w, rec.solution).feasible, where + ": reference also rejects the unrepaired candidate");
                continue;
            }
            if (stale.empty()) {
                check(rec.reanchored.empty() && rec.variants_generated == 0 && rec.evaluated, where + ": nothing stale -> evaluated as is, no re-anchoring");
                check(trip_keys(rec.solution) == trip_keys(base) && rec.solution.truck_route == rt, where + ": unaffected candidate carries the base trips verbatim");
                ++o_evaluated;
                Solution cand{rt, base.drone_trips};
                const auto v = ref_eval(w, cand);
                if (v.feasible) {
                    ++o_acc; ++st.n4r_accepted;
                    check(rec.reason == RejectReason::None && close(rec.objective, v.objective), where + ": feasible -> accepted with the reference objective");
                    if (!best || v.objective < best_obj) { best = i; best_obj = v.objective; }
                } else {
                    ++st.n4r_eval_rejected;
                    check(is_eval_reason(rec.reason), where + ": infeasible -> an evaluator reason");
                    ++o_rej[static_cast<std::size_t>(rec.reason)];
                }
                continue;
            }

            // stale: replacement options per stale trip, from the FLP data (ratio ascending, ties by id)
            std::vector<std::vector<NodeId>> opts(stale.size());
            std::vector<std::pair<NodeId, NodeId>> arcs(stale.size());
            for (std::size_t k = 0; k < stale.size(); ++k) {
                const NodeId f = base.drone_trips[stale[k]].launch_node;
                const std::size_t fp = static_cast<std::size_t>(std::find(rt.begin(), rt.end(), f) - rt.begin());
                NodeId a = kInvalidNodeId, b = kInvalidNodeId;
                for (std::size_t q = fp; q-- > 0;) if (!is_flp(w, rt[q])) { a = rt[q]; break; }
                for (std::size_t q = fp + 1; q < rt.size(); ++q) if (!is_flp(w, rt[q])) { b = rt[q]; break; }
                arcs[k] = {a, b};
                for (const auto& kv : *w.flp) if (kv.second.arc_from == a && kv.second.arc_to == b) opts[k].push_back(kv.first);
                std::sort(opts[k].begin(), opts[k].end(), [&](NodeId x, NodeId y) {
                    const double rx = w.flp->at(x).position_ratio, ry = w.flp->at(y).position_ratio;
                    return rx != ry ? rx < ry : x < y;
                });
            }
            ++st.n4r_stale;
            std::vector<std::size_t> noflp;
            for (std::size_t k = 0; k < stale.size(); ++k) if (opts[k].empty()) noflp.push_back(stale[k]);
            if (!noflp.empty()) {
                ++st.n4r_noflp; ++o_rej[static_cast<std::size_t>(RejectReason::NoReplacementFlp)];
                check(rec.reason == RejectReason::NoReplacementFlp && !rec.evaluated && rec.variants_generated == 0 && rec.affected_trips == noflp,
                      where + ": a stale trip's arc has no FLP -> NoReplacementFlp (names those trips), not evaluated");
                check(trip_keys(rec.solution) == trip_keys(base), where + ": trips left untouched (never dissolved)");
                continue;
            }
            std::size_t combos = 1;
            for (const auto& o : opts) combos *= o.size();
            const bool capped = combos > budget; // the budget stops the enumeration before all combinations are reached
            if (capped) { ++o_capped; ++st.n4r_capped; }
            const std::size_t expect_vars = capped ? budget : combos;
            if (stale.size() > 1) ++st.n4r_multi;

            // enumerate (last stale trip fastest), judge with the reference evaluator, first minimum wins
            std::vector<std::size_t> idx(stale.size(), 0);
            bool any = false, first = true;
            (void)first;
            double bo = 0;
            Solution bestvar;
            std::vector<NodeId> best_to;
            std::size_t tie_count = 0, generated_here = 0;
            for (;;) {
                // variant route: drop the stale FLPs, put each chosen FLP into the gap of its arc in ratio order (unless already on the route)
                TruckRoute vr;
                for (NodeId v : rt) if (std::find(foreign.begin(), foreign.end(), v) == foreign.end()) vr.push_back(v);
                Solution var;
                var.drone_trips = base.drone_trips;
                std::vector<NodeId> to(stale.size());
                for (std::size_t k = 0; k < stale.size(); ++k) {
                    to[k] = opts[k][idx[k]];
                    var.drone_trips[stale[k]].launch_node = to[k];
                    if (std::find(vr.begin(), vr.end(), to[k]) != vr.end()) continue;
                    std::size_t pos = vr.size();
                    for (std::size_t q = 0; q + 1 < vr.size(); ++q) {
                        if (is_flp(w, vr[q])) continue;
                        // find the next non-FLP after q
                        std::size_t nq = q + 1;
                        while (nq < vr.size() && is_flp(w, vr[nq])) ++nq;
                        if (nq >= vr.size() || vr[q] != arcs[k].first || vr[nq] != arcs[k].second) continue;
                        std::size_t at = nq;
                        for (std::size_t m = q + 1; m < nq; ++m)
                            if (w.flp->at(vr[m]).position_ratio > w.flp->at(to[k]).position_ratio) { at = m; break; }
                        pos = at;
                        break;
                    }
                    vr.insert(vr.begin() + static_cast<long>(pos), to[k]);
                }
                var.truck_route = vr;
                ++o_variants; ++o_evaluated; ++generated_here;
                const auto v = ref_eval(w, var);
                if (v.feasible) {
                    if (!any || v.objective < bo) { any = true; bo = v.objective; bestvar = var; best_to = to; tie_count = 1; }
                    else if (v.objective == bo) ++tie_count;
                }
                first = false;
                if (generated_here >= budget) break; // budget used up
                std::size_t k = stale.size();
                bool done = false;
                while (true) {
                    if (k == 0) { done = true; break; }
                    --k;
                    if (++idx[k] < opts[k].size()) break;
                    idx[k] = 0;
                }
                if (done) break;
            }
            check(rec.variants_generated == expect_vars && generated_here == expect_vars && rec.variants_evaluated == expect_vars && rec.evaluated,
                  where + ": combinations generated == min(product of the replacement counts, budget), all evaluated");
            check(rec.cap_reached == capped && rec.combinations_total == combos, where + ": cap_reached / combinations_total");
            if (any) {
                ++o_acc; ++o_reanch; o_trips += static_cast<long>(stale.size());
                ++st.n4r_accepted; ++st.n4r_reanchored; st.n4r_trips += static_cast<long>(stale.size());
                if (tie_count > 1) ++st.n4r_ties;
                check(rec.reason == RejectReason::None && close(rec.objective, bo), where + ": re-anchored candidate accepted with the best variant's reference objective");
                check(rec.solution.truck_route == bestvar.truck_route, where + ": shown route == independently built best variant (first minimum)");
                check(trip_keys(rec.solution) == trip_keys(bestvar), where + ": shown trips == best variant");
                bool listed = rec.reanchored.size() == stale.size();
                for (std::size_t k = 0; listed && k < stale.size(); ++k)
                    listed = rec.reanchored[k].trip == stale[k] && rec.reanchored[k].from == base.drone_trips[stale[k]].launch_node && rec.reanchored[k].to == best_to[k];
                check(listed, where + ": re-anchor list == (stale trips ascending, old FLP, chosen FLP)");
                // unaffected trips and all drone ids / customers / recoveries preserved
                bool kept = rec.solution.drone_trips.size() == base.drone_trips.size();
                for (std::size_t t = 0; kept && t < base.drone_trips.size(); ++t) {
                    const auto &x = base.drone_trips[t], &y = rec.solution.drone_trips[t];
                    kept = x.drone_id == y.drone_id && x.service_customer == y.service_customer && x.recovery_node == y.recovery_node;
                    if (std::find(stale.begin(), stale.end(), t) == stale.end()) kept = kept && x.launch_node == y.launch_node;
                }
                check(kept, where + ": drone id, customer, recovery preserved; unaffected trips byte-identical");
                check(foreign_flps(w, rec.solution.truck_route).empty(), where + ": no FLP off its arc after re-anchoring");
                if (!best || bo < best_obj) { best = i; best_obj = bo; }
            } else {
                ++st.n4r_eval_rejected;
                check(is_eval_reason(rec.reason) && !(rec.reason == RejectReason::None), where + ": no feasible variant -> evaluator reason");
                ++o_rej[static_cast<std::size_t>(rec.reason)];
            }
        }
        // operator-level counters == independent tallies
        check(static_cast<long>(res.stale_launch_candidates) == o_stale, where + ": stale_launch_candidates");
        check(static_cast<long>(res.reanchored_candidates) == o_reanch && static_cast<long>(res.trips_reanchored) == o_trips, where + ": reanchored_candidates / trips_reanchored");
        check(static_cast<long>(res.variants_generated) == o_variants && static_cast<long>(res.accepted) == o_acc, where + ": variants_generated / accepted");
        check(static_cast<long>(res.evaluated) == o_evaluated, where + ": evaluator calls == independent count");
        check(static_cast<long>(res.cap_reached_candidates) == o_capped, where + ": cap_reached_candidates == independent count");
        bool rej_ok = true;
        long rej_sum = 0;
        for (std::size_t k = 0; k < o_rej.size(); ++k) {
            // evaluator reasons are compared as a group (first-variant attribution is checked per record above)
            if (k == static_cast<std::size_t>(RejectReason::InvalidRecovery) || k == static_cast<std::size_t>(RejectReason::NoReplacementFlp))
                rej_ok = rej_ok && static_cast<long>(res.rejected[k]) == o_rej[k];
            rej_sum += o_rej[k];
        }
        check(rej_ok, where + ": per-reason counters for invalid-recovery / no-replacement-flp");
        check(static_cast<long>(res.rejected_total()) == rej_sum && res.rejected_total() + res.accepted == res.generated, where + ": every candidate counted exactly once (accepted or one rejection reason)");
        check(static_cast<bool>(best) == res.best.has_value(), where + ": best exists iff some candidate is feasible");
        if (best && res.best) check(close(res.best->objective, best_obj) && res.best->solution.truck_route == res.log[*best].solution.truck_route, where + ": best = first-generated minimum (strict <)");
    }
}


// ------------------------------------------------------------------ cap 100 vs cap 10000 on the benchmark fixtures
// The default budget (100) is an empirical choice. On the fixtures of bench/bench_operators.cpp (base seed 31415, dataset 0,
// n = 10, 20, 50, initial solution, window 3) it finds the same BEST candidate per anchor as the former budget 10000.
// This is a statement about THESE fixtures only; the budget does not guarantee equality in general.
void check_cap_on_bench_fixtures() {
    for (int n : {10, 20, 50}) {
        tdr::config::StaticConfig cfg;
        cfg.seed.base_seed = 31415;
        cfg.instance.num_customers = n;
        cfg.instance.dataset_index = 0;
        const auto gen = tdr::instance::generate_static_instance(cfg);
        const auto& inst = gen.instance;
        const auto net0 = inst.make_network();
        tdr::init::InitialSolutionPolicy pol;
        pol.flp_policy.unit_division_length = cfg.flp.unit_division_length_m;
        const auto init = tdr::init::build_initial_solution(inst.depot, inst.customers, inst.customer_data, net0, pol);
        check(init.feasible, "bench fixture n=" + std::to_string(n) + ": initial solution feasible");
        if (!init.feasible) continue;
        const tdr::graph::SpatialNetwork net(init.node_registry, inst.truck, inst.drone);
        const tdr::flp::FlpArcIndex idx(init.flp_data);
        const SearchContext ctx{net, inst.customer_data, init.flp_data, idx, 3};
        const Solution& base = init.solution;
        const auto sp = skel_positions(World{&init.node_registry, &net, &inst.customer_data, &init.flp_data, &idx, 0}, base.truck_route);
        std::size_t anchors = 0, same_best = 0, capped = 0, same_cand = 0, cands = 0;
        for (std::size_t s = 1; s + 1 < sp.size(); ++s) {
            const NodeId a = base.truck_route[sp[s]];
            N4Options o100, o10k;
            o100.record_candidates = o10k.record_candidates = true;
            o100.max_reanchor_variants = 100;
            o10k.max_reanchor_variants = 10000;
            const auto r1 = n4_truck_service_node_relocate(base, ctx, a, o100), r2 = n4_truck_service_node_relocate(base, ctx, a, o10k);
            ++anchors;
            capped += r1.cap_reached_candidates;
            const bool eq = r1.best.has_value() == r2.best.has_value() && (!r1.best || (r1.best->objective == r2.best->objective && r1.best->solution.truck_route == r2.best->solution.truck_route && trip_keys(r1.best->solution) == trip_keys(r2.best->solution)));
            if (eq) ++same_best;
            check(r1.generated == r2.generated && r1.log.size() == r2.log.size(), "bench fixture n=" + std::to_string(n) + ": same candidates generated under both budgets");
            for (std::size_t i = 0; i < r1.log.size() && i < r2.log.size(); ++i) {
                ++cands;
                if (r1.log[i].reason == RejectReason::None ? (r2.log[i].reason == RejectReason::None && r1.log[i].objective == r2.log[i].objective) : r2.log[i].reason != RejectReason::None) ++same_cand;
            }
            check(r2.cap_reached_candidates == 0, "bench fixture n=" + std::to_string(n) + ": budget 10000 is never reached (exhaustive)");
        }
        check(same_best == anchors, "bench fixture n=" + std::to_string(n) + ": cap 100 finds the same best candidate as cap 10000 for every one of the " + std::to_string(anchors) + " anchors");
        check(same_cand == cands, "bench fixture n=" + std::to_string(n) + ": candidate by candidate the same accepted/rejected status and objective (observed on these fixtures, not guaranteed in general)");
        std::cout << "cap 100 vs 10000, n=" << n << ": " << anchors << " anchors, " << cands << " candidates, cap reached in " << capped << " candidates, best identical for " << same_best << "/" << anchors << " anchors, candidates identical " << same_cand << "/" << cands << "\n";
    }
}

// ------------------------------------------------------------------ N5 oracle
void check_n5(const World& w, const Solution& base, int window, Stats& st, const std::string& tag) {
    const auto sp = skel_positions(w, base.truck_route);
    for (std::size_t ti = 0; ti < base.drone_trips.size(); ++ti) {
        const auto& old_trip = base.drone_trips[ti];
        const NodeId d = old_trip.service_customer;
        std::size_t lp = 0;
        for (std::size_t i = 0; i < base.truck_route.size(); ++i) if (base.truck_route[i] == old_trip.launch_node) lp = i;
        std::size_t gd = 0; // gap holding the launch FLP
        for (std::size_t k = 0; k < sp.size(); ++k) if (sp[k] < lp) gd = k;
        const long hd = static_cast<long>(2 * gd + 1);

        std::vector<std::pair<long, NodeId>> partners;
        for (std::size_t k = 1; k + 1 < sp.size(); ++k) {
            const NodeId p = base.truck_route[sp[k]];
            if (w.nodes->at(p).type != NodeType::Customer) continue;
            const long dh = std::labs(static_cast<long>(2 * k) - hd);
            if (dh <= 2L * window) partners.emplace_back(dh, p);
        }
        std::sort(partners.begin(), partners.end());

        N5Options opt;
        opt.record_candidates = true;
        const auto res = n5_td_service_node_swap(base, SearchContext{*w.net, *w.cd, *w.flp, *w.idx, window}, ti, opt);
        ++st.n5_calls;
        tally(st, res);
        mix_result(st, res);
        const std::string where = tag + " N5 trip " + std::to_string(ti) + " (C" + std::to_string(d) + ") w=" + std::to_string(window);
        check(res.status == OperatorStatus::Ok && res.anchor_customer == d, where + ": status ok, anchor customer");

        std::size_t rec_i = 0;
        std::optional<std::size_t> best_i;
        double best_obj = 0;
        bool shape_ok = true;
        for (const auto& pp : partners) {
            const NodeId t = pp.second;
            std::size_t ts = 0;
            for (std::size_t k = 0; k < sp.size(); ++k) if (base.truck_route[sp[k]] == t) ts = k;
            TruckRoute r = base.truck_route;
            r[sp[ts]] = d;
            std::vector<TripKey> keys_dummy;
            std::vector<DroneTrip> rest;
            for (std::size_t k = 0; k < base.drone_trips.size(); ++k) if (k != ti) rest.push_back(base.drone_trips[k]);
            std::set<NodeId> used;
            for (const auto& x : rest) used.insert(x.launch_node);
            TruckRoute r2;
            for (NodeId v : r) if (!is_flp(w, v) || used.count(v)) r2.push_back(v);
            r = r2;
            const auto foreign = foreign_flps(w, r);
            std::vector<std::size_t> affected;
            for (std::size_t k = 0; k < base.drone_trips.size(); ++k) {
                if (k == ti) continue;
                const auto& x = base.drone_trips[k];
                if (x.recovery_node == t || std::find(foreign.begin(), foreign.end(), x.launch_node) != foreign.end()) affected.push_back(k);
            }
            if (!affected.empty() || !foreign.empty()) {
                if (rec_i >= res.log.size()) { shape_ok = false; break; }
                const auto& rec = res.log[rec_i++];
                ++st.n5_candidates;
                ++st.n5_affected;
                check(rec.reason == RejectReason::AffectedTrips && !rec.evaluated && rec.move.customer == t && rec.move.partner == d, where + ": swap with C" + std::to_string(t) + " leaves trips invalid -> one AffectedTrips record");
                check(rec.affected_trips == affected, where + ": affected trip list for partner C" + std::to_string(t));
                continue;
            }
            // unaffected: one candidate per (gap in window of 2*ts) x (FLP of that arc)
            const auto sp2 = skel_positions(w, r);
            for (std::size_t g = 0; g + 1 < sp2.size(); ++g) {
                const long dh = std::labs(static_cast<long>(2 * g + 1) - static_cast<long>(2 * ts));
                if (dh > 2L * window) continue;
                const NodeId x = r[sp2[g]], y = r[sp2[g + 1]];
                for (NodeId f : w.idx->flps_on_arc(x, y)) {
                    if (rec_i >= res.log.size()) { shape_ok = false; break; }
                    const auto& rec = res.log[rec_i++];
                    ++st.n5_candidates;
                    const bool on_route = std::find(r.begin(), r.end(), f) != r.end();
                    if (on_route) ++st.n5_reuse;
                    TruckRoute rr = r;
                    if (!on_route) {
                        std::size_t at = sp2[g] + 1;
                        const auto key = [&](NodeId v) { return std::make_pair(w.flp->at(v).position_ratio, v); };
                        while (at < sp2[g + 1] && key(rr[at]) < key(f)) ++at;
                        rr.insert(rr.begin() + static_cast<long>(at), f);
                    }
                    std::optional<int> lowest;
                    double obj = 0;
                    for (int id = 0; id < w.U && !lowest; ++id) {
                        Solution cand;
                        cand.truck_route = rr;
                        cand.drone_trips = rest;
                        DroneTrip nt; nt.drone_id = id; nt.launch_node = f; nt.service_customer = t; nt.recovery_node = y;
                        cand.drone_trips.push_back(nt);
                        const auto v = ref_eval(w, cand);
                        if (v.feasible) { lowest = id; obj = v.objective; }
                    }
                    const std::string cw = where + " partner C" + std::to_string(t) + " launch " + std::to_string(f);
                    check(rec.move.customer == t && rec.move.partner == d && rec.move.launch == f && rec.move.recovery == y && rec.move.replaced_trip == ti, cw + ": move descriptor");
                    check(rec.solution.truck_route.empty() || rec.solution.truck_route == rr, cw + ": candidate route equals brute force");
                    if (y == 0) ++st.n5_depot_recovery;
                    if (lowest) {
                        ++st.n5_accepted;
                        check(rec.reason == RejectReason::None && rec.evaluated && close(rec.objective, obj) && rec.move.drone_id == *lowest, cw + ": feasible -> accepted with the lowest feasible drone id (op " + std::to_string(rec.move.drone_id) + ", oracle " + std::to_string(*lowest) + ")");
                        if (rec.move.drone_id > 0) ++st.n5_second_drone;
                        check(covers_once(w, rec.solution), cw + ": exactly-once coverage");
                        check(foreign_flps(w, rec.solution.truck_route).empty(), cw + ": no foreign FLPs");
                        if (!best_i || obj < best_obj) { best_i = rec_i - 1; best_obj = obj; }
                    } else {
                        check(rec.reason != RejectReason::None && rec.reason != RejectReason::AffectedTrips, cw + ": infeasible for every drone id -> rejected (reason " + reject_reason_name(rec.reason) + ")");
                        if (rec.reason == RejectReason::NoFreeDrone) { ++st.n5_nofree; check(!rec.evaluated, cw + ": NoFreeDrone is never evaluated"); }
                        else if (rec.reason == RejectReason::EvalEndurance) ++st.n5_endurance;
                        else ++st.n5_eval_rejected;
                    }
                }
            }
        }
        check(shape_ok && rec_i == res.log.size() && res.generated == res.log.size(), where + ": record count == brute force (" + std::to_string(res.log.size()) + " vs " + std::to_string(rec_i) + ")");
        check(static_cast<bool>(best_i) == res.best.has_value(), where + ": best exists iff some candidate is feasible");
        if (best_i && res.best) check(close(res.best->objective, best_obj), where + ": best objective = minimum");
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
        // a third of the seeds get a short endurance so that endurance rejections occur
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
        for (int step = 0; step < 4; ++step) {
            ++st.bases;
            if (!cur.drone_trips.empty()) ++st.bases_with_trips;
            const std::string tag = "seed " + std::to_string(seed) + " step " + std::to_string(step);
            for (int win : {1, 3}) {
                if (run_oracle) {
                    check_n4(w, cur, win, st, tag);
                    check_n4_reanchor(w, cur, win, st, tag);       // default budget (100)
                    check_n4_reanchor(w, cur, win, st, tag + " cap4", 4); // a small budget: the capped path is exercised often
                    check_n5(w, cur, win, st, tag);
                }
            }
            // random-walk to a different feasible base (either operator, random pick of an accepted candidate)
            tdr::util::Rng walk(tdr::util::derive_seed(4243, tdr::util::SeedStream::Fuzz, static_cast<std::uint64_t>(seed * 100 + step)));
            const SearchContext ctx{net, inst.customer_data, init.flp_data, idx, 3};
            OperatorResult r;
            if (!cur.drone_trips.empty() && walk.bernoulli(0.7)) {
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
    check_cap_on_bench_fixtures();
    const Stats a = campaign(true, kSeeds);
    const Stats b = campaign(true, kSeeds);

    std::cout << "bases=" << a.bases << " with_trips=" << a.bases_with_trips << "\n"
              << "N4: calls=" << a.n4_calls << " candidates=" << a.n4_candidates << " accepted=" << a.n4_accepted << " affected=" << a.n4_affected
              << " eval_rejected=" << a.n4_eval_rejected << " (trip-order " << a.n4_trip_order << ")\n"
              << "N5: calls=" << a.n5_calls << " candidates=" << a.n5_candidates << " accepted=" << a.n5_accepted << " affected=" << a.n5_affected
              << " no_free_drone=" << a.n5_nofree << " endurance=" << a.n5_endurance << " other_eval=" << a.n5_eval_rejected
              << " reuse_of_onroute_flp=" << a.n5_reuse << " depot_recovery=" << a.n5_depot_recovery << " second_drone=" << a.n5_second_drone << "\n";

    std::cout << "N4 Reanchor: candidates=" << a.n4r_candidates << " stale=" << a.n4r_stale << " reanchored=" << a.n4r_reanchored << " trips_reanchored=" << a.n4r_trips
              << " multi_trip=" << a.n4r_multi << " accepted=" << a.n4r_accepted << " no_replacement=" << a.n4r_noflp << " invalid_recovery=" << a.n4r_invrec
              << " eval_rejected=" << a.n4r_eval_rejected << " cap_reached=" << a.n4r_capped << " tie_groups=" << a.n4r_ties << "\n";

    // The oracle only means something if it exercised every branch.
    check(a.bases > 100 && a.bases_with_trips > 50, "non-vacuity: enough bases, many with trips");
    // N4 evaluator rejections are NOT required: every trip built by the initial builder or by N5 recovers at the
    // next truck node (D4), so any N4 move that disturbs a recovery node also pulls the launch FLP off its arc and is
    // caught earlier as AffectedTrips. The one reachable evaluator rejection (recovery moved ahead of the launch,
    // reported as eval-trip-order) is covered by the hand fixture in test_n4_n5.cpp.
    check(a.n4_candidates > 500 && a.n4_accepted > 100 && a.n4_affected > 20, "non-vacuity: N4 accepted and affected candidates occur");
    check(a.n4r_candidates > 500 && a.n4r_stale > 100 && a.n4r_reanchored > 50, "non-vacuity: Reanchor produces stale candidates and re-anchors some");
    check(a.n4r_multi > 0, "non-vacuity: candidates with SEVERAL stale trips occur (joint enumeration)");
    check(a.n4r_invrec > 0 && a.n4r_noflp > 0, "non-vacuity: invalid-recovery and no-replacement-flp rejections occur");
    check(a.n4r_eval_rejected > 0, "non-vacuity: evaluator rejections of re-anchored / plain candidates occur");
    check(a.n5_candidates > 500 && a.n5_accepted > 100 && a.n5_affected > 10, "non-vacuity: N5 accepted and affected candidates occur");
    check(a.n5_endurance > 0, "non-vacuity: N5 endurance rejections occur");
    check(a.n5_nofree > 0, "non-vacuity: N5 NoFreeDrone rejections occur");
    check(a.n5_reuse > 0 && a.n5_depot_recovery > 0 && a.n5_second_drone > 0, "non-vacuity: launch-FLP reuse, depot recovery and a non-zero drone id occur");
    // Construction bugs would show up as these two reasons: they must never be produced.
    check(a.bad_reason_flp_arc == 0 && a.bad_reason_flp_unused == 0, "operators never produce an FLP-off-arc or idle-FLP candidate that reaches the evaluator");
    // Determinism: a second pass over the same seeds (no oracle) yields the same digest of every result.
    check(a.digest == b.digest, "deterministic: identical digest over the whole campaign on a second full run (oracle on both)");

    std::cout << g_checks << " checks, " << g_failures << " failures\n";
    if (g_failures == 0) std::cout << "ALL TESTS PASSED\n";
    return g_failures == 0 ? 0 : 1;
}
