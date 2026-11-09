// M1 fuzz harness. Generates seeded instances through the NEW generator,
// builds initial solutions with the EXISTING builder, then applies random
// structural edits through the new route helpers and checks invariants
// against the EXISTING evaluator (the oracle). It asserts:
//   * exactly-once service is preserved by edits that preserve it, and the
//     structural service map exposes the damage of edits that do not;
//   * flps_off_their_arc() agrees with the evaluator on every edited route
//     (the helper reports; the evaluator decides -- they must never differ);
//   * drop_unused_flps() removes exactly the rule-(6) offenders and nothing
//     a trip uses;
//   * assign_drone_ids_by_route_order()/lowest_free_drone_for_trip() never
//     leave a (28) or |U| violation behind when they report success;
//   * a solution the evaluator accepts has no foreign/unused FLPs and covers
//     every customer exactly once;
//   * the whole pipeline is deterministic.
// The seeds are fixed, so any failure is reproducible.

#include "config/static_config.hpp"
#include "flp/flp_arc_index.hpp"
#include "init/initial_solution_builder.hpp"
#include "instance/static_instance.hpp"
#include "search/candidate_window.hpp"
#include "search/drone_assignment.hpp"
#include "search/route_ops.hpp"
#include "timing/timing_engine.hpp"
#include "util/rng.hpp"

#include <algorithm>
#include <iostream>
#include <set>
#include <string>
#include <vector>

using namespace tdr::model;
using namespace tdr::search;

namespace {

int g_failures = 0;
long g_checks = 0;

void check(bool condition, const std::string& description) {
    ++g_checks;
    if (!condition) {
        std::cout << "[FAIL] " << description << "\n";
        ++g_failures;
    }
}

int count_reasons(const std::vector<std::string>& reasons, const std::string& needle) {
    int c = 0;
    for (const auto& r : reasons) if (r.find(needle) != std::string::npos) ++c;
    return c;
}

// Each network customer is served by exactly one of: the truck route, or
// exactly one drone trip. Structural only.
bool exactly_once(const Solution& s, const NodeRegistry& nodes, const std::vector<NodeId>& customers) {
    const ServiceMap m = classify_service(s, nodes);
    for (NodeId c : customers) {
        const int by_truck = static_cast<int>(std::count(m.truck_customers.begin(), m.truck_customers.end(), c));
        const int by_drone = static_cast<int>(trips_serving(s, c).size());
        if (by_truck + by_drone != 1) return false;
    }
    return true;
}

int missing_customers(const Solution& s, const NodeRegistry& nodes, const std::vector<NodeId>& customers) {
    const ServiceMap m = classify_service(s, nodes);
    int miss = 0;
    for (NodeId c : customers) {
        const bool by_truck = std::count(m.truck_customers.begin(), m.truck_customers.end(), c) > 0;
        if (!by_truck && trips_serving(s, c).empty()) ++miss;
    }
    return miss;
}

} // namespace

int main() {
    constexpr int kSeeds = 30;
    constexpr int kMutationsPerSeed = 30;

    long cases = 0, cases_with_foreign = 0, cases_feasible = 0, initial_feasible = 0, cases_unused = 0;
    long assign_success = 0, lowest_free_checks = 0, cases_unplaced = 0, stretches = 0, nonzero_ids = 0, nonzero_free_results = 0;
    long drone_to_truck_moves = 0, relocations = 0, reversals = 0, trip_drops = 0;

    for (int seed = 0; seed < kSeeds; ++seed) {
        tdr::util::Rng fz(tdr::util::derive_seed(777, tdr::util::SeedStream::Fuzz, static_cast<std::uint64_t>(seed)));

        tdr::config::StaticConfig cfg;
        cfg.seed.base_seed = 777;
        cfg.instance.num_customers = static_cast<int>(fz.uniform_int(5, 12));
        cfg.instance.dataset_index = seed;
        cfg.vehicle.num_drones = static_cast<int>(fz.uniform_int(1, 3));
        const auto gen = tdr::instance::generate_static_instance(cfg);
        check(gen.ok, "instance generation succeeds (seed " + std::to_string(seed) + ")");
        const auto& inst = gen.instance;
        const auto net = inst.make_network();

        tdr::init::InitialSolutionPolicy pol;
        pol.flp_policy.unit_division_length = cfg.flp.unit_division_length_m;
        const auto res = tdr::init::build_initial_solution(inst.depot, inst.customers, inst.customer_data, net, pol);
        const auto res_again = tdr::init::build_initial_solution(inst.depot, inst.customers, inst.customer_data, net, pol);
        check(res.solution.truck_route == res_again.solution.truck_route && res.solution.drone_trips.size() == res_again.solution.drone_trips.size() &&
                  res.objective_value == res_again.objective_value,
              "generator + builder pipeline is deterministic (seed " + std::to_string(seed) + ")");
        if (res.feasible) ++initial_feasible;

        const tdr::graph::SpatialNetwork full(res.node_registry, inst.truck, inst.drone);
        const NodeRegistry& nodes = res.node_registry;
        const tdr::flp::FlpArcIndex index(res.flp_data);
        const int U = cfg.vehicle.num_drones;

        // ---- invariants of the initial solution --------------------------------
        check(exactly_once(res.solution, nodes, inst.customers), "initial solution serves every customer exactly once (seed " + std::to_string(seed) + ")");
        check(flps_off_their_arc(res.solution.truck_route, nodes, res.flp_data).empty(), "initial solution has no foreign FLPs (seed " + std::to_string(seed) + ")");
        check(unused_flps_in_route(res.solution, nodes).empty(), "initial solution has no unused FLPs (seed " + std::to_string(seed) + ")");
        for (const auto& t : res.solution.drone_trips) {
            const auto owner = index.arc_of(t.launch_node);
            check(owner.has_value(), "every launch node of the initial solution is in the arc index");
        }
        {
            const PositionIndex pi(res.solution, nodes);
            bool all_placed = true;
            for (NodeId c : inst.customers) all_placed = all_placed && pi.half_step(c).has_value();
            check(all_placed, "every customer of the initial solution gets a half-step position (seed " + std::to_string(seed) + ")");
            for (int w : {1, 2, 3}) {
                bool brute_ok = true, sym_ok = true;
                for (NodeId c : inst.customers) {
                    std::set<NodeId> expect;
                    for (NodeId o : inst.customers) if (o != c && pi.adjacent(c, o, w)) expect.insert(o);
                    const auto got = pi.neighbours(c, w);
                    brute_ok = brute_ok && std::set<NodeId>(got.begin(), got.end()) == expect && got.size() == expect.size();
                    for (NodeId o : inst.customers) sym_ok = sym_ok && (pi.adjacent(c, o, w) == pi.adjacent(o, c, w));
                }
                check(brute_ok, "neighbours() equals the brute-force adjacency set (w=" + std::to_string(w) + ", seed " + std::to_string(seed) + ")");
                check(sym_ok, "adjacency is symmetric (w=" + std::to_string(w) + ")");
            }
        }

        // ---- random edits ---------------------------------------------------------
        for (int m = 0; m < kMutationsPerSeed; ++m) {
            Solution s = res.solution;
            const int kind = static_cast<int>(fz.uniform_int(0, 4));
            bool serves_all_expected = true;   // does this edit preserve exactly-once by construction?
            int expected_missing = 0;

            if (kind == 0) { // relocate a random truck-served customer to a random gap (FLPs of the gaps NOT handled)
                ++relocations;
                const ServiceMap sm = classify_service(s, nodes);
                if (sm.truck_customers.empty()) continue;
                const NodeId c = sm.truck_customers[fz.uniform_index(sm.truck_customers.size())];
                remove_from_route(s.truck_route, c);
                const Skeleton sk = make_skeleton(s.truck_route, nodes);
                if (sk.gap_count() == 0) continue;
                const std::size_t g = fz.uniform_index(sk.gap_count());
                const auto pos = insertion_position_for_gap(sk, g);
                check(pos.has_value(), "a chosen gap has an insertion position");
                s.truck_route.insert(s.truck_route.begin() + static_cast<std::ptrdiff_t>(*pos), c);
            } else if (kind == 1) { // drone customer back to the truck (N8-like), with explicit FLP clean-up
                if (s.drone_trips.empty()) continue;
                ++drone_to_truck_moves;
                const std::size_t ti = fz.uniform_index(s.drone_trips.size());
                const NodeId c = s.drone_trips[ti].service_customer;
                erase_trip(s, ti);
                const Skeleton sk = make_skeleton(s.truck_route, nodes);
                const std::size_t g = fz.uniform_index(sk.gap_count());
                const auto pos = insertion_position_for_gap(sk, g);
                s.truck_route.insert(s.truck_route.begin() + static_cast<std::ptrdiff_t>(*pos), c);
            } else if (kind == 2) { // reverse a random slice of the route between interior skeleton nodes
                ++reversals;
                const Skeleton sk = make_skeleton(s.truck_route, nodes);
                if (sk.size() < 4) continue;
                std::size_t a = 1 + fz.uniform_index(sk.size() - 2);
                std::size_t b = 1 + fz.uniform_index(sk.size() - 2);
                if (a > b) std::swap(a, b);
                std::reverse(s.truck_route.begin() + static_cast<std::ptrdiff_t>(sk.route_position[a]),
                             s.truck_route.begin() + static_cast<std::ptrdiff_t>(sk.route_position[b]) + 1);
            } else if (kind == 4) { // stretch a trip: recover it at a later truck node, creating overlapping flights
                if (s.drone_trips.empty()) continue;
                ++stretches;
                const std::size_t ti = fz.uniform_index(s.drone_trips.size());
                const auto lp = tdr::timing::detail::find_route_position(s.truck_route, s.drone_trips[ti].launch_node);
                if (!lp) continue;
                std::vector<NodeId> later;
                for (std::size_t p = *lp + 1; p < s.truck_route.size(); ++p) {
                    if (!is_flp(nodes, s.truck_route[p]) && s.truck_route[p] != s.drone_trips[ti].service_customer) later.push_back(s.truck_route[p]);
                }
                if (later.empty()) continue;
                s.drone_trips[ti].recovery_node = later[fz.uniform_index(later.size())];
            } else { // drop a random trip WITHOUT re-serving its customer: damage must be visible
                if (s.drone_trips.empty()) continue;
                ++trip_drops;
                erase_trip(s, fz.uniform_index(s.drone_trips.size()));
                serves_all_expected = false;
                expected_missing = 1;
            }
            ++cases;

            // I4: service structure.
            if (serves_all_expected) {
                check(exactly_once(s, nodes, inst.customers), "edit kind " + std::to_string(kind) + " preserves exactly-once service structurally");
            } else {
                check(missing_customers(s, nodes, inst.customers) == expected_missing, "dropping a trip without re-serving leaves exactly one customer unserved (visible to the map)");
            }

            // I1: evaluator determinism and no-throw.
            const auto e1 = tdr::timing::evaluate_solution(s, full, inst.customer_data, res.flp_data);
            const auto e2 = tdr::timing::evaluate_solution(s, full, inst.customer_data, res.flp_data);
            check(e1.feasible == e2.feasible && e1.objective_value == e2.objective_value && e1.violation_reasons == e2.violation_reasons,
                  "evaluator is deterministic on edited solutions");

            // I2: helper vs evaluator on FLP arc ownership.
            const auto foreign = flps_off_their_arc(s.truck_route, nodes, res.flp_data);
            const int ev_arc = count_reasons(e1.violation_reasons, "belongs to L_(") + count_reasons(e1.violation_reasons, "has no arc-ownership data");
            check(static_cast<int>(foreign.size()) == ev_arc,
                  "flps_off_their_arc count == evaluator arc-ownership violations (helper " + std::to_string(foreign.size()) +
                      ", evaluator " + std::to_string(ev_arc) + ", seed " + std::to_string(seed) + ", kind " + std::to_string(kind) + ")");
            if (!foreign.empty()) ++cases_with_foreign;

            // I3: rule (6) and drop_unused_flps.
            // unused_flps_in_route() means "no trip NAMES this FLP as its launch".
            // The evaluator's rule (6) is stricter: it counts an FLP as unused
            // also when the only trips naming it cannot be placed on the route
            // (e.g. recovery now before launch) -- those trips are themselves
            // reported infeasible. So: equal when every trip is placed,
            // otherwise the helper's set is a subset (count <=).
            const int ev_unused = count_reasons(e1.violation_reasons, "(constraint (6))");
            const auto unused = unused_flps_in_route(s, nodes);
            bool all_trips_placed = true;
            for (const auto& tt : e1.drone_trip_timings) all_trips_placed = all_trips_placed && tt.structurally_valid && tt.on_route;
            if (all_trips_placed) {
                check(static_cast<int>(unused.size()) == ev_unused, "all trips placed: unused_flps_in_route count == evaluator rule-(6) violations");
            } else {
                ++cases_unplaced;
                check(static_cast<int>(unused.size()) <= ev_unused, "some trip unplaced: helper never reports MORE unused FLPs than the evaluator");
                check(!e1.feasible, "a solution with an unplaced trip is never evaluator-feasible");
            }
            if (!unused.empty()) ++cases_unused;
            Solution cleaned = s;
            std::set<NodeId> launch_nodes;
            for (const auto& t : cleaned.drone_trips) launch_nodes.insert(t.launch_node);
            const auto removed = drop_unused_flps(cleaned, nodes);
            check(removed == unused, "drop_unused_flps removes exactly unused_flps_in_route");
            bool used_kept = true;
            for (NodeId f : launch_nodes) used_kept = used_kept && std::count(cleaned.truck_route.begin(), cleaned.truck_route.end(), f) >= std::count(s.truck_route.begin(), s.truck_route.end(), f);
            check(used_kept, "drop_unused_flps keeps every FLP a trip launches from");
            check(cleaned.drone_trips.size() == s.drone_trips.size(), "drop_unused_flps does not change the trips");
            const auto e3 = tdr::timing::evaluate_solution(cleaned, full, inst.customer_data, res.flp_data);
            if (all_trips_placed) {
                check(count_reasons(e3.violation_reasons, "(constraint (6))") == 0, "all trips placed: after drop_unused_flps the evaluator reports no rule-(6) violation");
            } else {
                check(count_reasons(e3.violation_reasons, "(constraint (6))") <= ev_unused && !e3.feasible,
                      "some trip unplaced: clean-up never adds rule-(6) violations and the solution stays rejected (the unplaced trip is not hidden)");
            }
            check(e3.objective_value == e1.objective_value || !e1.feasible, "dropping unused FLPs does not move any arrival time of an otherwise-feasible solution");

            // I6/I7: drone id assignment.
            Solution assigned = cleaned;
            if (assign_drone_ids_by_route_order(assigned, U)) {
                ++assign_success;
                const auto e4 = tdr::timing::evaluate_solution(assigned, full, inst.customer_data, res.flp_data);
                check(count_reasons(e4.violation_reasons, "constraint (28)") == 0 && count_reasons(e4.violation_reasons, "simultaneously in flight") == 0,
                      "successful assignment leaves no (28) / |U| violation");
                for (std::size_t ti = 0; ti < assigned.drone_trips.size(); ++ti) { // EVERY trip, not a random one
                    ++lowest_free_checks;
                    if (assigned.drone_trips[ti].drone_id > 0) ++nonzero_ids;
                    Solution moved = assigned;
                    const int before = moved.drone_trips[ti].drone_id;
                    const auto fr = lowest_free_drone_for_trip(moved, ti, U);
                    if (fr) {
                        if (*fr > 0) ++nonzero_free_results;
                        check(*fr <= before || !(moved.drone_trips[ti].drone_id >= 0), "lowest free drone <= the id the successful assignment gave");
                        moved.drone_trips[ti].drone_id = *fr;
                        const auto e5 = tdr::timing::evaluate_solution(moved, full, inst.customer_data, res.flp_data);
                        check(count_reasons(e5.violation_reasons, "constraint (28)") == 0 && count_reasons(e5.violation_reasons, "simultaneously in flight") == 0,
                              "re-assigning one trip to its lowest free drone keeps (28) and |U| satisfied");
                    } else {
                        // After a successful assignment a PLACED trip always has a free drone (the one it holds).
                        // So "nothing" is only legitimate for a trip whose launch or recovery cannot be located.
                        const auto lp = tdr::timing::detail::find_route_position(moved.truck_route, moved.drone_trips[ti].launch_node);
                        const bool placeable = lp.has_value() && tdr::timing::detail::find_route_position_from(
                                                                      moved.truck_route, moved.drone_trips[ti].recovery_node, *lp + 1).has_value();
                        check(!placeable, "lowest_free_drone_for_trip returned nothing only for a trip whose launch/recovery cannot be located");
                    }
                }
            }

            // I5: feasible => structurally clean.
            if (e1.feasible) {
                ++cases_feasible;
                check(foreign.empty() && unused.empty() && exactly_once(s, nodes, inst.customers),
                      "an evaluator-feasible edited solution has no foreign/unused FLPs and serves every customer exactly once");
            }
        }
    }

    std::cout << "fuzz summary: seeds=" << kSeeds << " initial_feasible=" << initial_feasible << " edited_cases=" << cases
              << " (relocations=" << relocations << " drone->truck=" << drone_to_truck_moves << " reversals=" << reversals
              << " trip_drops=" << trip_drops << " stretches=" << stretches << ")\n"
              << "              cases_with_foreign_flps=" << cases_with_foreign << " cases_with_unused_flps=" << cases_unused
              << " evaluator_feasible_after_edit=" << cases_feasible << " assignments_succeeded=" << assign_success
              << " lowest_free_checks=" << lowest_free_checks << " cases_with_unplaced_trip=" << cases_unplaced << " assigned_trips_with_id>0=" << nonzero_ids
              << " lowest_free_results>0=" << nonzero_free_results << " assertions=" << g_checks << "\n";

    // Non-vacuity: the interesting branches must actually have been exercised.
    check(cases >= 200, "at least 200 edited cases were generated");
    check(cases_with_foreign >= 20, "at least 20 cases exercised the foreign-FLP comparison with a non-empty set");
    check(cases_unused >= 5, "at least 5 cases exercised unused-FLP clean-up");
    check(assign_success >= 50, "at least 50 successful drone-id assignments were verified");
    check(lowest_free_checks >= 50, "at least 50 lowest_free_drone_for_trip checks ran");
    check(nonzero_ids >= 30, "at least 30 assigned trips carry drone id > 0 (overlapping flights were really created)");
    check(nonzero_free_results >= 20, "at least 20 lowest_free_drone_for_trip results were > 0 (the overlap branch was exercised)");
    check(cases_unplaced >= 20, "at least 20 cases exercised the unplaced-trip branch");
    check(initial_feasible >= kSeeds * 2 / 3, "most initial solutions are evaluator-feasible (builder sanity)");

    if (g_failures == 0) std::cout << "\nALL TESTS PASSED (" << g_checks << " assertions)\n";
    else std::cout << "\nSOME TESTS FAILED (" << g_failures << " of " << g_checks << ")\n";
    return g_failures == 0 ? 0 : 1;
}
