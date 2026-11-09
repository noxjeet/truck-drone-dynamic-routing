// M1 performance follow-up (P1/P2): differential test of the production
// evaluator tdr::timing::evaluate_solution against the FROZEN reference
// tdr::timing_ref::evaluate_solution (tests/reference/timing_engine_reference.hpp,
// verbatim copy of the evaluator at commit cc37e49).
//
//   default        : the accepted P1+P2 contract. Everything is bitwise
//                    identical except the three things P2 is allowed to change:
//                      (a) the ORDER of customer_records,
//                      (b) the ORDER of customer-level violation reasons,
//                      (c) the last bits of objective_value (summation order),
//                    and (c) must stay inside the rigorous bound
//                    |dObj| <= 2 * n * eps * sum|t_i|.
//   --strict       : bitwise identical in EVERYTHING (the P1-only contract).
//
// Deterministic: all randomness comes from tdr::util::Rng with the fixed seeds
// printed in the summary (kBaseSeed). No wall-clock, no addresses.

#include "config/static_config.hpp"
#include "flp/flp_arc_index.hpp"
#include "init/initial_solution_builder.hpp"
#include "instance/static_instance.hpp"
#include "reference/timing_engine_reference.hpp"
#include "search/drone_assignment.hpp"
#include "search/route_ops.hpp"
#include "timing/timing_engine.hpp"
#include "util/rng.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

using namespace tdr::model;
using namespace tdr::search;

namespace {

constexpr std::uint64_t kBaseSeed = 31415;
bool g_strict = false;
int g_failures = 0;
long g_checks = 0;

void check(bool condition, const std::string& description) {
    ++g_checks;
    if (!condition) {
        std::cout << "[FAIL] " << description << "\n";
        ++g_failures;
    }
}

bool beq(double a, double b) { return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b); }

template <class A, class B>
bool vec_beq(const std::vector<A>& a, const std::vector<B>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) if (!beq(a[i], b[i])) return false;
    return true;
}

template <class A, class B>
bool trip_timing_equal(const A& a, const B& b) {
    return a.structurally_valid == b.structurally_valid && a.on_route == b.on_route && a.has_customer_data == b.has_customer_data &&
           a.drone_id_valid == b.drone_id_valid && beq(a.launch_time, b.launch_time) &&
           beq(a.outbound_flight_time_raw, b.outbound_flight_time_raw) && beq(a.customer_arrival_time, b.customer_arrival_time) &&
           beq(a.customer_departure_time, b.customer_departure_time) && beq(a.return_flight_time_raw, b.return_flight_time_raw) &&
           beq(a.recovery_arrival_time, b.recovery_arrival_time) && beq(a.truck_arrival_at_recovery, b.truck_arrival_at_recovery) &&
           beq(a.recovery_complete_time, b.recovery_complete_time) && beq(a.theta_used, b.theta_used) &&
           beq(a.endurance_value_36, b.endurance_value_36) && beq(a.endurance_value_37, b.endurance_value_37) &&
           beq(a.generalized_flight_time, b.generalized_flight_time) && a.endurance_feasible == b.endurance_feasible &&
           a.feasible == b.feasible && a.violation_reason == b.violation_reason;
}

template <class A, class B>
bool record_equal(const A& a, const B& b) {
    return a.customer_id == b.customer_id && a.served_by_truck == b.served_by_truck && a.served_by_drone == b.served_by_drone &&
           beq(a.service_arrival_time, b.service_arrival_time) && a.feasible == b.feasible && a.violation_reason == b.violation_reason;
}

template <class A, class B>
bool sync_equal(const A& a, const B& b) {
    return a.recovery_node == b.recovery_node && beq(a.truck_arrival, b.truck_arrival) && beq(a.drone_arrival, b.drone_arrival) &&
           a.truck_waited == b.truck_waited && a.drone_waited == b.drone_waited && beq(a.wait_time, b.wait_time);
}

struct Stats {
    long cases = 0, feasible = 0, infeasible = 0, objective_bit_diff = 0, order_diff_records = 0, order_diff_reasons = 0;
    long multi_trip = 0, drone_id_gt0 = 0, no_trips = 0, simultaneous_recovery = 0, missing_cd_reason = 0, invalid_cd_entry_reason = 0;
    long map_order_cases = 0, ref_map_order_dependent = 0, prod_map_order_dependent = 0;
    double max_abs = 0.0, max_rel = 0.0, max_bound_ratio = 0.0;
    long pairs = 0, ties_ref = 0, ties_new = 0, flips = 0, flips_nonfeasible_only = 0;
    std::map<int, long> by_tamper;
    std::map<int, long> by_edit;
};
Stats g_stats;

template <class P, class R>
void compare(const P& p, const R& r, const std::string& label) {
    ++g_stats.cases;
    const std::string where = " [" + label + "]";
    check(p.feasible == r.feasible, "feasibility verdict identical" + where);
    check(p.max_simultaneous_drones == r.max_simultaneous_drones, "max_simultaneous_drones identical" + where);
    check(vec_beq(p.truck_timing.arrival_time, r.truck_timing.arrival_time), "truck arrival times bitwise identical" + where);
    check(vec_beq(p.truck_timing.departure_time, r.truck_timing.departure_time), "truck departure times bitwise identical" + where);
    bool trips_ok = p.drone_trip_timings.size() == r.drone_trip_timings.size();
    for (std::size_t i = 0; trips_ok && i < p.drone_trip_timings.size(); ++i) trips_ok = trip_timing_equal(p.drone_trip_timings[i], r.drone_trip_timings[i]);
    check(trips_ok, "every drone-trip timing field bitwise identical" + where);
    bool sync_ok = p.synchronization_records.size() == r.synchronization_records.size();
    for (std::size_t i = 0; sync_ok && i < p.synchronization_records.size(); ++i) sync_ok = sync_equal(p.synchronization_records[i], r.synchronization_records[i]);
    check(sync_ok, "synchronization records bitwise identical" + where);

    // customer_records: same multiset; identical ORDER only required in strict mode.
    check(p.customer_records.size() == r.customer_records.size(), "same number of customer records" + where);
    {
        auto pa = p.customer_records;
        auto ra = r.customer_records;
        const bool same_order = [&] {
            if (pa.size() != ra.size()) return false;
            for (std::size_t i = 0; i < pa.size(); ++i) if (!record_equal(pa[i], ra[i])) return false;
            return true;
        }();
        if (!same_order) ++g_stats.order_diff_records;
        if (g_strict) check(same_order, "customer_records identical incl. order (strict)" + where);
        std::sort(pa.begin(), pa.end(), [](const auto& x, const auto& y) { return x.customer_id < y.customer_id; });
        std::sort(ra.begin(), ra.end(), [](const auto& x, const auto& y) { return x.customer_id < y.customer_id; });
        bool set_ok = pa.size() == ra.size();
        for (std::size_t i = 0; set_ok && i < pa.size(); ++i) set_ok = record_equal(pa[i], ra[i]);
        check(set_ok, "customer_records equal as an id-keyed set (bitwise)" + where);
        if (!g_strict) {
            bool ascending = true;
            for (std::size_t i = 1; i < p.customer_records.size(); ++i) ascending = ascending && p.customer_records[i - 1].customer_id < p.customer_records[i].customer_id;
            check(ascending, "customer_records are in ascending customer id order (P2)" + where);
        }
    }

    // violation reasons
    {
        const bool same_order = p.violation_reasons == r.violation_reasons;
        if (!same_order) ++g_stats.order_diff_reasons;
        if (g_strict) check(same_order, "violation_reasons identical incl. order (strict)" + where);
        auto pa = p.violation_reasons;
        auto ra = r.violation_reasons;
        std::sort(pa.begin(), pa.end());
        std::sort(ra.begin(), ra.end());
        check(pa == ra, "violation_reasons equal as a multiset" + where);
        for (const auto& s : p.violation_reasons) {
            if (s.find("has no CustomerData") != std::string::npos) { ++g_stats.missing_cd_reason; break; }
        }
        for (const auto& s : p.violation_reasons) {
            if (s.find("CustomerData entry") != std::string::npos) { ++g_stats.invalid_cd_entry_reason; break; }
        }
    }

    // objective
    {
        const double po = p.objective_value, ro = r.objective_value;
        if (g_strict) check(beq(po, ro), "objective bitwise identical (strict)" + where);
        if (!beq(po, ro)) ++g_stats.objective_bit_diff;
        const double ad = std::fabs(po - ro);
        g_stats.max_abs = std::max(g_stats.max_abs, ad);
        if (ro != 0.0) g_stats.max_rel = std::max(g_stats.max_rel, ad / std::fabs(ro));
        double sum_abs = 0.0;
        for (const auto& rec : r.customer_records) sum_abs += std::fabs(rec.service_arrival_time);
        const double n = static_cast<double>(r.customer_records.size());
        const double bound = 2.0 * n * std::numeric_limits<double>::epsilon() * sum_abs;
        check(ad <= bound, "objective difference inside the rigorous summation bound 2*n*eps*sum|t|" + where);
        if (bound > 0.0) g_stats.max_bound_ratio = std::max(g_stats.max_bound_ratio, ad / bound);
        // P2 contract: the objective IS the left-to-right sum of customer_records in their (ascending-id) order, bit for bit.
        {
            double acc = 0.0;
            for (const auto& rec : p.customer_records) acc += rec.service_arrival_time;
            check(beq(acc, po), "objective == left-to-right sum of customer_records in record order, bitwise (P2 summation contract)" + where);
        }
        // the objective is exactly the sum of the service arrival times of the serving vehicle (depot return excluded)
        long double exact = 0.0L;
        for (const auto& rec : p.customer_records) exact += static_cast<long double>(rec.service_arrival_time);
        check(std::fabs(static_cast<long double>(po) - exact) <= 2.0L * static_cast<long double>(n) * std::numeric_limits<double>::epsilon() * sum_abs,
              "production objective equals the sum of its customer_records (depot return excluded)" + where);
    }
    if (p.feasible) ++g_stats.feasible; else ++g_stats.infeasible;
}

tdr::init::InitialSolutionPolicy policy_for(const tdr::config::StaticConfig& cfg) {
    tdr::init::InitialSolutionPolicy pol;
    pol.flp_policy.unit_division_length = cfg.flp.unit_division_length_m;
    return pol;
}

using CustomerMap = std::unordered_map<NodeId, CustomerData>;

CustomerMap rebuilt_reverse(const CustomerMap& m) {
    std::vector<NodeId> keys;
    for (const auto& kv : m) keys.push_back(kv.first);
    std::sort(keys.begin(), keys.end(), std::greater<NodeId>());
    CustomerMap out;
    out.reserve(m.size() * 3 + 11); // different bucket count too
    for (NodeId k : keys) out.emplace(k, m.at(k));
    return out;
}

// Edit kinds (see apply_edit).
constexpr int kEditKinds = 11;
bool apply_edit(Solution& s, int kind, tdr::util::Rng& fz, const NodeRegistry& nodes, const Solution& base, const TruckRoute& truck_only) {
    switch (kind) {
        case 0: { // relocate a truck customer to a random gap
            const ServiceMap sm = classify_service(s, nodes);
            if (sm.truck_customers.empty()) return false;
            const NodeId c = sm.truck_customers[fz.uniform_index(sm.truck_customers.size())];
            remove_from_route(s.truck_route, c);
            const Skeleton sk = make_skeleton(s.truck_route, nodes);
            if (sk.gap_count() == 0) return false;
            const auto pos = insertion_position_for_gap(sk, fz.uniform_index(sk.gap_count()));
            if (!pos) return false;
            s.truck_route.insert(s.truck_route.begin() + static_cast<std::ptrdiff_t>(*pos), c);
            return true;
        }
        case 1: { // drone customer to the truck
            if (s.drone_trips.empty()) return false;
            const std::size_t ti = fz.uniform_index(s.drone_trips.size());
            const NodeId c = s.drone_trips[ti].service_customer;
            erase_trip(s, ti);
            const Skeleton sk = make_skeleton(s.truck_route, nodes);
            const auto pos = insertion_position_for_gap(sk, fz.uniform_index(sk.gap_count()));
            if (!pos) return false;
            s.truck_route.insert(s.truck_route.begin() + static_cast<std::ptrdiff_t>(*pos), c);
            return true;
        }
        case 2: { // reverse a slice
            const Skeleton sk = make_skeleton(s.truck_route, nodes);
            if (sk.size() < 4) return false;
            std::size_t a = 1 + fz.uniform_index(sk.size() - 2), b = 1 + fz.uniform_index(sk.size() - 2);
            if (a > b) std::swap(a, b);
            std::reverse(s.truck_route.begin() + static_cast<std::ptrdiff_t>(sk.route_position[a]),
                         s.truck_route.begin() + static_cast<std::ptrdiff_t>(sk.route_position[b]) + 1);
            return true;
        }
        case 3: { // stretch: recover later (overlapping flights)
            if (s.drone_trips.empty()) return false;
            const std::size_t ti = fz.uniform_index(s.drone_trips.size());
            const auto lp = tdr::timing::detail::find_route_position(s.truck_route, s.drone_trips[ti].launch_node);
            if (!lp) return false;
            std::vector<NodeId> later;
            for (std::size_t p = *lp + 1; p < s.truck_route.size(); ++p)
                if (!is_flp(nodes, s.truck_route[p]) && s.truck_route[p] != s.drone_trips[ti].service_customer) later.push_back(s.truck_route[p]);
            if (later.empty()) return false;
            s.drone_trips[ti].recovery_node = later[fz.uniform_index(later.size())];
            return true;
        }
        case 4: { // drop a trip, customer unserved
            if (s.drone_trips.empty()) return false;
            erase_trip(s, fz.uniform_index(s.drone_trips.size()));
            return true;
        }
        case 5: { // simultaneous recoveries: trip A recovers where trip B recovers
            if (s.drone_trips.size() < 2) return false;
            const std::size_t a = fz.uniform_index(s.drone_trips.size());
            std::size_t b = fz.uniform_index(s.drone_trips.size());
            if (a == b) b = (b + 1) % s.drone_trips.size();
            const auto lp = tdr::timing::detail::find_route_position(s.truck_route, s.drone_trips[a].launch_node);
            const auto rp = tdr::timing::detail::find_route_position(s.truck_route, s.drone_trips[b].recovery_node);
            if (!lp || !rp || *rp <= *lp) return false;
            s.drone_trips[a].recovery_node = s.drone_trips[b].recovery_node;
            return true;
        }
        case 6: // all-truck solution (no trips, no FLPs)
            s.truck_route = truck_only;
            s.drone_trips.clear();
            return true;
        case 7: { // duplicate a trip (customer served twice by drones, FLP reuse)
            if (s.drone_trips.empty()) return false;
            s.drone_trips.push_back(s.drone_trips[fz.uniform_index(s.drone_trips.size())]);
            return true;
        }
        case 8: { // corrupt a drone id
            if (s.drone_trips.empty()) return false;
            static const int bad[] = {-1, 99};
            s.drone_trips[fz.uniform_index(s.drone_trips.size())].drone_id = bad[fz.uniform_index(2)];
            return true;
        }
        case 9: { // swap launch and recovery of a trip
            if (s.drone_trips.empty()) return false;
            auto& t = s.drone_trips[fz.uniform_index(s.drone_trips.size())];
            std::swap(t.launch_node, t.recovery_node);
            return true;
        }
        default: // no edit: the builder's own solution
            s = base;
            return true;
    }
}

constexpr int kTamperKinds = 6;
CustomerMap tamper(const CustomerMap& cd, int kind, tdr::util::Rng& fz, const std::vector<NodeId>& customers, NodeId a_flp_id) {
    CustomerMap out = cd;
    switch (kind) {
        case 1: out.erase(customers[fz.uniform_index(customers.size())]); break;                         // missing entry
        case 2: { CustomerData d = cd.at(customers[0]); d.node_id = 987654; out.emplace(987654, d); } break; // unknown node id
        case 3: { CustomerData d = cd.at(customers[0]); d.node_id = a_flp_id; out.emplace(a_flp_id, d); } break; // entry for an FLP
        case 4: out.at(customers[fz.uniform_index(customers.size())]).node_id = 424242; break;           // node_id != key
        case 5: out = rebuilt_reverse(cd); break;                                                         // same content, other insertion order
        default: break;
    }
    return out;
}

template <class SolutionT>
void evaluate_and_compare(const SolutionT& s, const tdr::graph::SpatialNetwork& net, const CustomerMap& cd, const tdr::timing::FlpDataMap& flp,
                          const std::string& label) {
    const auto p = tdr::timing::evaluate_solution(s, net, cd, flp);
    const auto r = tdr::timing_ref::evaluate_solution(s, net, cd, flp);
    compare(p, r, label);
}

// ---- exact-arithmetic oracle (Shewchuk partials, as in Python's math.fsum) ---
// Returns the sign (-1, 0, +1) of the EXACT real-number sum of `xs`.
int exact_sign(const std::vector<double>& xs) {
    std::vector<double> partials;
    for (double x : xs) {
        std::size_t i = 0;
        for (double y : partials) {
            if (std::fabs(x) < std::fabs(y)) std::swap(x, y);
            const double hi = x + y;
            const double lo = y - (hi - x);
            if (lo != 0.0) partials[i++] = lo;
            x = hi;
        }
        partials.resize(i);
        partials.push_back(x);
    }
    while (!partials.empty() && partials.back() == 0.0) partials.pop_back();
    if (partials.empty()) return 0;
    return partials.back() > 0.0 ? 1 : -1; // non-overlapping partials: the largest one decides the sign
}

constexpr int kOrders = 6; // reference evaluated under this many CustomerData insertion orders

struct Obj {
    std::vector<double> times;       // production customer_records service times (== reference's as a set)
    double ref[kOrders];             // reference objective under each insertion order (ref[0] = the natural one)
    double prod;
    bool feasible;
};

int sgn(double a, double b) { return a < b ? -1 : (a > b ? 1 : 0); }

struct FlipStats {
    long pairs = 0, ff_pairs = 0;
    long flips = 0, flips_ff = 0;
    long flips_ref_unstable = 0, flips_ref_stable = 0;
    long flips_exact_tie = 0;           // the two solutions are EXACTLY equal as real numbers
    long ref_wrong = 0, prod_wrong = 0; // sign differs from the exact-arithmetic sign (ties counted as wrong if reported strict)
    long ref_unstable_pairs = 0;        // pairs whose reference ordering depends on map insertion order
    long prod_ties_exact = 0;           // production reports equal and the exact answer is equal
    double max_flip_exact_gap = 0.0;    // largest |obj difference| (production) among flipped pairs
};
FlipStats g_flip;

void flip_analysis(const std::vector<Obj>& v) {
    for (std::size_t i = 0; i < v.size(); ++i)
        for (std::size_t j = i + 1; j < v.size(); ++j) {
            ++g_stats.pairs;
            const int sr = sgn(v[i].ref[0], v[j].ref[0]), sp = sgn(v[i].prod, v[j].prod);
            if (sr == 0) ++g_stats.ties_ref;
            if (sp == 0) ++g_stats.ties_new;
            const bool ff = v[i].feasible && v[j].feasible;
            ++g_flip.pairs;
            if (ff) ++g_flip.ff_pairs;
            bool ref_unstable = false;
            for (int k = 1; k < kOrders; ++k) ref_unstable = ref_unstable || sgn(v[i].ref[k], v[j].ref[k]) != sr;
            if (ref_unstable) ++g_flip.ref_unstable_pairs;
            std::vector<double> diff = v[i].times;
            for (double t : v[j].times) diff.push_back(-t);
            const int truth = exact_sign(diff);
            if (sr != truth) ++g_flip.ref_wrong;
            if (sp != truth) ++g_flip.prod_wrong;
            if (sp == 0 && truth == 0) ++g_flip.prod_ties_exact;
            if (sr != sp) {
                ++g_stats.flips;
                ++g_flip.flips;
                if (ff) ++g_flip.flips_ff;
                if (!ff) ++g_stats.flips_nonfeasible_only;
                if (ref_unstable) ++g_flip.flips_ref_unstable; else ++g_flip.flips_ref_stable;
                if (truth == 0) ++g_flip.flips_exact_tie;
                g_flip.max_flip_exact_gap = std::max(g_flip.max_flip_exact_gap, std::fabs(v[i].prod - v[j].prod));
            }
        }
}

CustomerMap rebuilt_shuffled(const CustomerMap& m, tdr::util::Rng& rng, std::size_t reserve) {
    std::vector<NodeId> keys;
    for (const auto& kv : m) keys.push_back(kv.first);
    std::sort(keys.begin(), keys.end());
    for (std::size_t i = keys.size(); i > 1; --i) std::swap(keys[i - 1], keys[rng.uniform_index(i)]);
    CustomerMap out;
    out.reserve(reserve);
    for (NodeId k : keys) out.emplace(k, m.at(k));
    return out;
}

void run_env(std::uint64_t seed_idx, int n_customers, int edits, int num_drones, const std::string& tag) {
    tdr::util::Rng fz(tdr::util::derive_seed(kBaseSeed, tdr::util::SeedStream::Fuzz, seed_idx));
    tdr::config::StaticConfig cfg;
    cfg.seed.base_seed = kBaseSeed;
    cfg.instance.num_customers = n_customers;
    cfg.instance.dataset_index = static_cast<int>(seed_idx);
    cfg.vehicle.num_drones = num_drones;
    const auto gen = tdr::instance::generate_static_instance(cfg);
    check(gen.ok, "instance generation succeeds " + tag);
    if (!gen.ok) return;
    const auto& inst = gen.instance;
    const auto pol = policy_for(cfg);
    const auto res = tdr::init::build_initial_solution(inst.depot, inst.customers, inst.customer_data, inst.make_network(), pol);
    check(res.node_registry.size() > inst.customers.size() + 1, "builder produced a node registry with an FLP pool " + tag);
    const tdr::graph::SpatialNetwork full(res.node_registry, inst.truck, inst.drone);
    const NodeRegistry& nodes = res.node_registry;
    const NodeId some_flp = res.candidate_flps.empty() ? 987655 : res.candidate_flps.front().node.id;

    std::vector<Obj> objs;
    for (int m = 0; m < edits; ++m) {
        Solution s = res.solution;
        const int kind = (m == 0) ? 10 : static_cast<int>(fz.uniform_int(0, kEditKinds - 1));
        if (!apply_edit(s, kind, fz, nodes, res.solution, res.truck_only_route)) continue;
        if (fz.uniform_int(0, 1) == 1) (void)assign_drone_ids_by_route_order(s, num_drones);
        const int tk = (fz.uniform_int(0, 9) < 6) ? 0 : static_cast<int>(fz.uniform_int(1, kTamperKinds - 1));
        const CustomerMap cd = tamper(inst.customer_data, tk, fz, inst.customers, some_flp);
        ++g_stats.by_edit[kind];
        ++g_stats.by_tamper[tk];
        const std::string label = tag + " edit " + std::to_string(kind) + " tamper " + std::to_string(tk) + " #" + std::to_string(m);

        const auto p = tdr::timing::evaluate_solution(s, full, cd, res.flp_data);
        const auto r = tdr::timing_ref::evaluate_solution(s, full, cd, res.flp_data);
        compare(p, r, label);
        if (tk == 0 || tk == 5) { // content-identical CustomerData: the objective comparison is apples-to-apples
            Obj o;
            for (const auto& rec : p.customer_records) o.times.push_back(rec.service_arrival_time);
            o.ref[0] = r.objective_value;
            o.prod = p.objective_value;
            o.feasible = p.feasible;
            const CustomerMap rev = rebuilt_reverse(cd);
            o.ref[1] = tdr::timing_ref::evaluate_solution(s, full, rev, res.flp_data).objective_value;
            for (int k = 2; k < kOrders; ++k) {
                const CustomerMap sh = rebuilt_shuffled(cd, fz, 7 + static_cast<std::size_t>(fz.uniform_int(0, 3)) * 97);
                o.ref[k] = tdr::timing_ref::evaluate_solution(s, full, sh, res.flp_data).objective_value;
                if (!g_strict) check(beq(tdr::timing::evaluate_solution(s, full, sh, res.flp_data).objective_value, p.objective_value),
                                     "production objective identical under a shuffled insertion order " + label);
            }
            objs.push_back(o);
        }

        if (s.drone_trips.size() >= 2) ++g_stats.multi_trip;
        if (s.drone_trips.empty()) ++g_stats.no_trips;
        for (const auto& t : s.drone_trips) if (t.drone_id > 0 && t.drone_id < num_drones) { ++g_stats.drone_id_gt0; break; }
        {
            std::map<NodeId, int> rec;
            for (const auto& t : s.drone_trips) ++rec[t.recovery_node];
            for (const auto& kv : rec) if (kv.second >= 2) { ++g_stats.simultaneous_recovery; break; }
        }

        // map-insertion-order sensitivity (the reproducibility argument for P2)
        if (tk == 0 || tk == 5) {
            ++g_stats.map_order_cases;
            const CustomerMap rev = rebuilt_reverse(cd);
            const auto p2 = tdr::timing::evaluate_solution(s, full, rev, res.flp_data);
            const auto r2 = tdr::timing_ref::evaluate_solution(s, full, rev, res.flp_data);
            if (!beq(r.objective_value, r2.objective_value)) ++g_stats.ref_map_order_dependent;
            if (!beq(p.objective_value, p2.objective_value)) ++g_stats.prod_map_order_dependent;
            if (!g_strict) {
                check(beq(p.objective_value, p2.objective_value), "production objective does not depend on CustomerData map insertion order (P2) " + label);
                check(p.violation_reasons == p2.violation_reasons, "production violation list does not depend on map insertion order (P2) " + label);
            }
        }
    }
    flip_analysis(objs);
}

// ---- hand-built edge cases --------------------------------------------------
void edge_cases() {
    tdr::config::StaticConfig cfg;
    cfg.seed.base_seed = kBaseSeed;
    cfg.instance.num_customers = 3;
    const auto gen = tdr::instance::generate_static_instance(cfg);
    const auto& inst = gen.instance;

    { // depot-only network, empty customer data, several degenerate routes
        NodeRegistry reg;
        reg.add(Node{0, NodeType::Depot, {0.0, 0.0}});
        const tdr::graph::SpatialNetwork net(reg, inst.truck, inst.drone);
        const CustomerMap empty;
        const tdr::timing::FlpDataMap flp;
        for (const TruckRoute& route : {TruckRoute{}, TruckRoute{0}, TruckRoute{0, 0}}) {
            Solution s;
            s.truck_route = route;
            evaluate_and_compare(s, net, empty, flp, "depot-only route size " + std::to_string(route.size()));
        }
        check(net.customer_ids().empty(), "depot-only network has no customer ids");
    }
    { // customers present but no CustomerData, and a route that skips them
        Solution s;
        s.truck_route = {0, 0};
        evaluate_and_compare(s, inst.make_network(), CustomerMap{}, {}, "customers without CustomerData, empty route");
        s.truck_route = {};
        evaluate_and_compare(s, inst.make_network(), inst.customer_data, {}, "empty route with full CustomerData");
    }
    { // all-truck on a generated instance, all drone speeds
        const auto res = tdr::init::build_initial_solution(inst.depot, inst.customers, inst.customer_data, inst.make_network(), policy_for(cfg));
        Solution s;
        s.truck_route = res.truck_only_route;
        const tdr::graph::SpatialNetwork full(res.node_registry, inst.truck, inst.drone);
        evaluate_and_compare(s, full, inst.customer_data, res.flp_data, "all-truck, 3 customers");
        const auto e = tdr::timing::evaluate_solution(s, full, inst.customer_data, res.flp_data);
        check(e.feasible && e.customer_records.size() == 3, "all-truck solution is feasible with 3 records");
        double sum = 0.0;
        for (std::size_t p = 1; p + 1 < s.truck_route.size(); ++p) sum += e.truck_timing.arrival_time[p];
        check(std::fabs(sum - e.objective_value) <= 1e-9 * std::max(1.0, sum), "all-truck objective = sum of truck arrival times, depot return excluded");
    }
}

// ---- P1: SpatialNetwork::customer_ids() ------------------------------------
void p1_cache_tests() {
    tdr::config::StaticConfig cfg;
    cfg.seed.base_seed = kBaseSeed;
    cfg.instance.num_customers = 12;
    const auto gen = tdr::instance::generate_static_instance(cfg);
    const auto& inst = gen.instance;
    const auto res = tdr::init::build_initial_solution(inst.depot, inst.customers, inst.customer_data, inst.make_network(), policy_for(cfg));

    const auto scan = [](const NodeRegistry& r) {
        std::vector<NodeId> v;
        for (const auto& kv : r.all()) if (kv.second.type == NodeType::Customer) v.push_back(kv.first);
        std::sort(v.begin(), v.end());
        return v;
    };

    static_assert(std::is_same_v<decltype(std::declval<const tdr::graph::SpatialNetwork&>().customer_ids()), const std::vector<NodeId>&>,
                  "customer_ids() must expose the cache read-only");

    // construction: equals a fresh scan+sort, FLP pool excluded, ascending
    const tdr::graph::SpatialNetwork small = inst.make_network();
    check(small.customer_ids() == scan(small.nodes()), "cache == sorted scan of the registry (customers only)");
    check(small.customer_ids().size() == 12, "cache holds exactly the 12 customers");
    check(std::is_sorted(small.customer_ids().begin(), small.customer_ids().end()), "cache is ascending");
    const tdr::graph::SpatialNetwork full(res.node_registry, inst.truck, inst.drone);
    check(res.node_registry.size() > 1000, "full registry carries the eager FLP pool (precondition of the test)");
    check(full.customer_ids() == scan(full.nodes()) && full.customer_ids() == inst.customers, "FLP pool does not leak into the cache; cache == instance customers");

    // repeated access: stable address and content, no recomputation side effects
    const auto* addr1 = &full.customer_ids();
    const auto copy1 = full.customer_ids();
    for (int i = 0; i < 5; ++i) (void)tdr::timing::evaluate_solution(res.solution, full, inst.customer_data, res.flp_data);
    check(&full.customer_ids() == addr1 && full.customer_ids() == copy1, "repeated access (and repeated evaluation) leaves the cache untouched");

    // mutation/invalidation: the registry handed to the constructor is COPIED; later mutation of the source cannot reach the network
    NodeRegistry source;
    source.add(Node{0, NodeType::Depot, {0, 0}});
    source.add(Node{5, NodeType::Customer, {1, 1}});
    source.add(Node{3, NodeType::Customer, {2, 2}});
    const tdr::graph::SpatialNetwork net(source, inst.truck, inst.drone);
    check((net.customer_ids() == std::vector<NodeId>{3, 5}), "cache is ascending regardless of insertion order");
    source.add(Node{9, NodeType::Customer, {3, 3}});
    check((net.customer_ids() == std::vector<NodeId>{3, 5}) && net.nodes().size() == 3 && !net.nodes().contains(9),
          "mutating the source registry after construction does not reach the network (by-value copy), and the cache stays consistent with nodes()");
    check(net.customer_ids() == scan(net.nodes()), "cache still equals a fresh scan after the source was mutated");

    // copy / move / assignment keep cache and registry consistent
    tdr::graph::SpatialNetwork copied = net;
    check(copied.customer_ids() == net.customer_ids() && copied.customer_ids() == scan(copied.nodes()), "copy-constructed network has a consistent cache");
    tdr::graph::SpatialNetwork moved = std::move(copied);
    check(moved.customer_ids() == scan(moved.nodes()) && moved.customer_ids().size() == 2, "move-constructed network has a consistent cache");
    NodeRegistry other;
    other.add(Node{0, NodeType::Depot, {0, 0}});
    other.add(Node{7, NodeType::Customer, {1, 0}});
    tdr::graph::SpatialNetwork assigned(other, inst.truck, inst.drone);
    assigned = net;
    check(assigned.customer_ids() == scan(assigned.nodes()) && assigned.customer_ids() == (std::vector<NodeId>{3, 5}), "assignment replaces registry and cache together");

    // evaluator behaviour through the cache: missing CustomerData reported once per customer, in ascending id order
    {
        Solution s;
        s.truck_route = {0, 0};
        const auto e = tdr::timing::evaluate_solution(s, net, CustomerMap{}, {});
        std::vector<std::string> missing;
        for (const auto& r : e.violation_reasons) if (r.find("has no CustomerData") != std::string::npos) missing.push_back(r);
        check(missing.size() == 2 && missing[0].find("network customer 3 ") != std::string::npos && missing[1].find("network customer 5 ") != std::string::npos,
              "evaluator reports each network customer without CustomerData, ascending, exactly once");
        const auto r = tdr::timing_ref::evaluate_solution(s, net, CustomerMap{}, {});
        check(e.violation_reasons == r.violation_reasons, "the P1 path reproduces the reference violation list exactly (incl. order)");
    }

    // the builder's augmented network (new object) gets its own correct cache
    check(full.customer_ids() == small.customer_ids(), "augmented network and base network agree on the customer set");
}

} // namespace

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) if (std::string(argv[i]) == "--strict") g_strict = true;
    std::cout << "mode: " << (g_strict ? "STRICT (bitwise identical incl. order and objective)" : "P1+P2 contract") << ", base seed " << kBaseSeed << "\n";

    p1_cache_tests();
    edge_cases();

    // Small instances (the fuzz-sized regime): 40 seeds x 40 edits.
    for (int seed = 0; seed < 40; ++seed) {
        tdr::util::Rng pick(tdr::util::derive_seed(kBaseSeed, tdr::util::SeedStream::Instance, static_cast<std::uint64_t>(seed)));
        run_env(static_cast<std::uint64_t>(seed), static_cast<int>(pick.uniform_int(3, 14)), 40, static_cast<int>(pick.uniform_int(1, 3)),
                "small seed " + std::to_string(seed));
    }
    // Benchmark sizes: n = 10, 20, 50 with 3 seeds each.
    for (int n : {10, 20, 50})
        for (int k = 0; k < 3; ++k)
            run_env(static_cast<std::uint64_t>(1000 + n * 10 + k), n, 25, 2, "n=" + std::to_string(n) + " seed " + std::to_string(1000 + n * 10 + k));

    const Stats& st = g_stats;
    std::cout << "differential summary (" << (g_strict ? "strict" : "P1+P2") << "):\n"
              << "  cases=" << st.cases << " feasible=" << st.feasible << " infeasible=" << st.infeasible << " multi_trip=" << st.multi_trip
              << " drone_id>0=" << st.drone_id_gt0 << " no_trips=" << st.no_trips << " simultaneous_recovery=" << st.simultaneous_recovery << "\n"
              << "  tampered CustomerData: missing-data reason=" << st.missing_cd_reason << " invalid-entry reason=" << st.invalid_cd_entry_reason << "\n"
              << "  objective bit-differs=" << st.objective_bit_diff << " max_abs=" << st.max_abs << " max_rel=" << st.max_rel
              << " max(|d|/bound)=" << st.max_bound_ratio << "\n"
              << "  order differences: customer_records=" << st.order_diff_records << " violation_reasons=" << st.order_diff_reasons << "\n"
              << "  map-insertion-order sensitivity over " << st.map_order_cases << " cases: reference objective changes=" << st.ref_map_order_dependent
              << " production objective changes=" << st.prod_map_order_dependent << "\n"
              << "  pairwise ranking: pairs=" << st.pairs << " exact ties(ref)=" << st.ties_ref << " exact ties(new)=" << st.ties_new
              << " sign flips=" << st.flips << " (of which involving an infeasible solution=" << st.flips_nonfeasible_only << ")\n"
              << "  assertions=" << g_checks << "\n"
              << "  ranking vs exact arithmetic (content-identical maps only): pairs=" << g_flip.pairs << " (feasible-feasible " << g_flip.ff_pairs << ")\n"
              << "    reference ordering depends on map insertion order in " << g_flip.ref_unstable_pairs << " pairs\n"
              << "    sign flips ref->prod=" << g_flip.flips << " (feasible-feasible " << g_flip.flips_ff << "): of which ref-unstable=" << g_flip.flips_ref_unstable
              << " ref-stable=" << g_flip.flips_ref_stable << ", exactly-equal-as-reals=" << g_flip.flips_exact_tie << ", max |objective gap| among flips=" << g_flip.max_flip_exact_gap << "\n"
              << "    orderings disagreeing with the exact sign: reference=" << g_flip.ref_wrong << " production=" << g_flip.prod_wrong
              << " (production reports an exact tie correctly in " << g_flip.prod_ties_exact << " pairs)\n";
    std::cout << "  edits:";
    for (const auto& kv : st.by_edit) std::cout << " k" << kv.first << "=" << kv.second;
    std::cout << "\n  tampers:";
    for (const auto& kv : st.by_tamper) std::cout << " t" << kv.first << "=" << kv.second;
    std::cout << "\n";

    // non-vacuity
    check(st.cases >= 1000, "at least 1000 differential cases");
    check(st.feasible >= 100 && st.infeasible >= 300, "both feasible (>=100) and infeasible (>=300) cases are well represented");
    check(st.multi_trip >= 100, "at least 100 cases with >= 2 trips");
    check(st.drone_id_gt0 >= 30, "at least 30 cases with a drone id > 0");
    check(st.no_trips >= 30, "at least 30 trip-free (all-truck) cases");
    check(st.simultaneous_recovery >= 10, "at least 10 cases with simultaneous recoveries");
    check(st.missing_cd_reason >= 20, "at least 20 cases exercised the P1 'network customer has no CustomerData' path");
    check(st.invalid_cd_entry_reason >= 40, "at least 40 cases exercised invalid CustomerData entries");
    check(st.map_order_cases >= 500, "at least 500 map-insertion-order cases");

    if (g_failures == 0) std::cout << "\nALL TESTS PASSED (" << g_checks << " assertions)\n";
    else std::cout << "\nSOME TESTS FAILED (" << g_failures << " of " << g_checks << ")\n";
    return g_failures == 0 ? 0 : 1;
}
