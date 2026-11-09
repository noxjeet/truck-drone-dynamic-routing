#pragma once

// Route and trip bookkeeping shared by the Phase G neighbourhoods and the
// Phase H shakes (M1, 9 Nov 2026).
//
// RULES (from the Phase G brief, enforced by construction here):
//  * NOTHING in this file decides feasibility. Timing, endurance, SR/SL,
//    (28), |U| and the objective come from tdr::timing::evaluate_solution
//    and only from there. These helpers locate things, apply mechanical
//    edits, and REPORT the consequences of an edit.
//  * NOTHING repairs silently. When an edit leaves the solution in a state
//    the evaluator would reject (e.g. a launch point left on a foreign arc),
//    the helper either does not do that edit or returns the affected
//    entities to the caller. The only mechanical clean-ups offered are
//    explicit, return what they changed, and are named for it
//    (drop_unused_flps).
//  * The FLP arc rule (p9: a launch point of L_(a,b) is usable only while
//    the truck traverses (a,b)) is a property of the route's *skeleton*:
//    the sequence of non-FLP nodes. Any insertion of a node into, or
//    removal of a node from, a skeleton gap changes the arcs the truck
//    traverses, so every FLP sitting in the affected gap(s) becomes foreign.
//    flps_in_gap / flps_off_their_arc expose exactly that.
//
// Terminology: skeleton index s = rank among the route's non-FLP entries
// (0 = start depot, S-1 = end depot). Gap g = the arc between skeleton nodes
// g and g+1; the FLPs "in gap g" are the FLP entries between them.

#include <algorithm>
#include <cstddef>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "model/drone_trip.hpp"
#include "model/flexible_launch_point.hpp"
#include "model/node.hpp"
#include "model/solution.hpp"
#include "model/truck_route.hpp"
#include "timing/timing_engine.hpp"

namespace tdr::search {

inline bool is_flp(const tdr::model::NodeRegistry& nodes, tdr::model::NodeId id) {
    return nodes.contains(id) && nodes.at(id).type == tdr::model::NodeType::FlexibleLaunchPoint;
}

// The non-FLP entries of a route, with their raw route positions.
struct Skeleton {
    std::vector<std::size_t> route_position; // route_position[s] = raw index of skeleton node s
    std::vector<tdr::model::NodeId> node;    // node[s]
    std::size_t size() const { return node.size(); }
    std::size_t gap_count() const { return node.empty() ? 0 : node.size() - 1; }
};

inline Skeleton make_skeleton(const tdr::model::TruckRoute& route, const tdr::model::NodeRegistry& nodes) {
    Skeleton sk;
    for (std::size_t p = 0; p < route.size(); ++p) {
        if (!is_flp(nodes, route[p])) {
            sk.route_position.push_back(p);
            sk.node.push_back(route[p]);
        }
    }
    return sk;
}

// Skeleton index of the gap containing raw route position p (an FLP entry),
// or nullopt if p is itself a skeleton node / out of range.
inline std::optional<std::size_t> gap_of_route_position(const Skeleton& sk, std::size_t p) {
    if (sk.size() < 2) return std::nullopt;
    for (std::size_t g = 0; g + 1 < sk.size(); ++g) {
        if (sk.route_position[g] < p && p < sk.route_position[g + 1]) return g;
    }
    return std::nullopt;
}

// FLP ids currently sitting between skeleton nodes g and g+1, in route order.
inline std::vector<tdr::model::NodeId> flps_in_gap(const tdr::model::TruckRoute& route, const Skeleton& sk, std::size_t g) {
    std::vector<tdr::model::NodeId> out;
    if (g + 1 >= sk.size()) return out;
    for (std::size_t p = sk.route_position[g] + 1; p < sk.route_position[g + 1]; ++p) out.push_back(route[p]);
    return out;
}

// One FLP entry whose route placement disagrees with its owning arc.
struct ForeignFlp {
    std::size_t route_position = 0;
    tdr::model::NodeId flp = tdr::model::kInvalidNodeId;
    tdr::model::NodeId actual_from = tdr::model::kInvalidNodeId; // nearest non-FLP node before it
    tdr::model::NodeId actual_to = tdr::model::kInvalidNodeId;   // nearest non-FLP node after it
    bool has_owner = false;                                       // false: no arc-ownership data at all
    tdr::model::NodeId owner_from = tdr::model::kInvalidNodeId;
    tdr::model::NodeId owner_to = tdr::model::kInvalidNodeId;
};

// All FLP entries of `route` that are not on their own arc (or have no
// ownership data). This is a *report* of a mechanical fact; it is the same
// condition the evaluator rejects (p9, (13)-(15)), and tests/test_search_infra
// asserts the two agree on every fuzzed route. Use it after an edit to learn
// what the edit invalidated -- never to "fix" it.
inline std::vector<ForeignFlp> flps_off_their_arc(
    const tdr::model::TruckRoute& route, const tdr::model::NodeRegistry& nodes,
    const std::unordered_map<tdr::model::NodeId, tdr::model::FlexibleLaunchPointData>& flp_data) {
    std::vector<ForeignFlp> out;
    tdr::model::NodeId prev = tdr::model::kInvalidNodeId;           // nearest non-FLP before
    std::vector<std::size_t> pending;                                // FLP entries awaiting their "next"
    std::vector<tdr::model::NodeId> pending_prev;
    for (std::size_t p = 0; p < route.size(); ++p) {
        if (is_flp(nodes, route[p])) {
            pending.push_back(p);
            pending_prev.push_back(prev);
        } else {
            for (std::size_t k = 0; k < pending.size(); ++k) {
                ForeignFlp f;
                f.route_position = pending[k];
                f.flp = route[pending[k]];
                f.actual_from = pending_prev[k];
                f.actual_to = route[p];
                const auto it = flp_data.find(f.flp);
                if (it == flp_data.end()) {
                    out.push_back(f);
                } else if (it->second.arc_from != f.actual_from || it->second.arc_to != f.actual_to) {
                    f.has_owner = true;
                    f.owner_from = it->second.arc_from;
                    f.owner_to = it->second.arc_to;
                    out.push_back(f);
                }
            }
            pending.clear();
            pending_prev.clear();
            prev = route[p];
        }
    }
    for (std::size_t k = 0; k < pending.size(); ++k) { // trailing FLPs with no following truck node
        ForeignFlp f;
        f.route_position = pending[k];
        f.flp = route[pending[k]];
        f.actual_from = pending_prev[k];
        f.actual_to = tdr::model::kInvalidNodeId;
        const auto it = flp_data.find(f.flp);
        if (it != flp_data.end()) { f.has_owner = true; f.owner_from = it->second.arc_from; f.owner_to = it->second.arc_to; }
        out.push_back(f);
    }
    return out;
}

// ---- trip lookups --------------------------------------------------------

inline std::vector<std::size_t> trips_launching_from(const tdr::model::Solution& s, tdr::model::NodeId flp) {
    std::vector<std::size_t> out;
    for (std::size_t i = 0; i < s.drone_trips.size(); ++i) if (s.drone_trips[i].launch_node == flp) out.push_back(i);
    return out;
}

inline std::vector<std::size_t> trips_serving(const tdr::model::Solution& s, tdr::model::NodeId customer) {
    std::vector<std::size_t> out;
    for (std::size_t i = 0; i < s.drone_trips.size(); ++i) if (s.drone_trips[i].service_customer == customer) out.push_back(i);
    return out;
}

inline std::vector<std::size_t> trips_recovering_at(const tdr::model::Solution& s, tdr::model::NodeId node) {
    std::vector<std::size_t> out;
    for (std::size_t i = 0; i < s.drone_trips.size(); ++i) if (s.drone_trips[i].recovery_node == node) out.push_back(i);
    return out;
}

// How each customer of the solution is currently served, from structure
// alone (no timing). A customer may appear in both lists or neither if the
// solution is malformed; the evaluator reports that.
struct ServiceMap {
    std::vector<tdr::model::NodeId> truck_customers;                       // route order
    std::unordered_map<tdr::model::NodeId, std::size_t> drone_trip_of;      // customer -> first trip index serving it
};

inline ServiceMap classify_service(const tdr::model::Solution& s, const tdr::model::NodeRegistry& nodes) {
    ServiceMap m;
    for (tdr::model::NodeId v : s.truck_route) {
        if (nodes.contains(v) && nodes.at(v).type == tdr::model::NodeType::Customer) m.truck_customers.push_back(v);
    }
    for (std::size_t i = 0; i < s.drone_trips.size(); ++i) {
        m.drone_trip_of.emplace(s.drone_trips[i].service_customer, i);
    }
    return m;
}

// ---- explicit mechanical edits ---------------------------------------------

// Removes `id` from the route; returns its former raw position. Does NOT
// touch trips or FLPs: the caller owns the consequences, which it can read
// from flps_off_their_arc() and trips_*().
inline std::optional<std::size_t> remove_from_route(tdr::model::TruckRoute& route, tdr::model::NodeId id) {
    const auto pos = tdr::timing::detail::find_route_position(route, id);
    if (!pos) return std::nullopt;
    route.erase(route.begin() + static_cast<std::ptrdiff_t>(*pos));
    return pos;
}

// Raw route position at which a node inserted "into gap g" lands when it is
// placed immediately BEFORE skeleton node g+1 (i.e. after any FLPs of the
// gap). Returns nullopt if g is not a valid gap of `sk`. The gap's FLPs, if
// any, remain in the route and become foreign -- the caller must dissolve or
// re-home the trips using them (see flps_in_gap).
inline std::optional<std::size_t> insertion_position_for_gap(const Skeleton& sk, std::size_t g) {
    if (g + 1 >= sk.size()) return std::nullopt;
    return sk.route_position[g + 1];
}

// Raw route position at which FLP `flp` must be inserted into gap g so that the
// gap's FLPs stay in increasing position_ratio order (ties by id) -- the order
// the evaluator requires for FLPs of one arc. Pure position arithmetic: the
// caller guarantees `flp` belongs to arc (sk.node[g], sk.node[g+1]) (the
// evaluator re-checks it independently). nullopt if g is not a gap of `sk` or
// `flp` has no ownership data. [M2]
inline std::optional<std::size_t> flp_insertion_position(
    const tdr::model::TruckRoute& route, const Skeleton& sk, std::size_t g, tdr::model::NodeId flp,
    const std::unordered_map<tdr::model::NodeId, tdr::model::FlexibleLaunchPointData>& flp_data) {
    if (g + 1 >= sk.size()) return std::nullopt;
    const auto mine = flp_data.find(flp);
    if (mine == flp_data.end()) return std::nullopt;
    std::size_t pos = sk.route_position[g] + 1;
    for (std::size_t p = pos; p < sk.route_position[g + 1]; ++p) {
        const auto other = flp_data.find(route[p]);
        if (other == flp_data.end()) break;
        const bool before = other->second.position_ratio < mine->second.position_ratio ||
                            (other->second.position_ratio == mine->second.position_ratio && route[p] < flp);
        if (!before) break;
        pos = p + 1;
    }
    return pos;
}

// Erases trip `index` (order of the remaining trips is preserved). Does not
// change the route; a launch FLP left with no trip is reported by
// unused_flps_in_route() / removed by drop_unused_flps().
inline void erase_trip(tdr::model::Solution& s, std::size_t index) {
    s.drone_trips.erase(s.drone_trips.begin() + static_cast<std::ptrdiff_t>(index));
}

// FLP entries of the route that no trip names as its launch node (rule (6)
// violations if left in place), in route order. NOTE the deliberate
// asymmetry with the evaluator: the evaluator's rule (6) also counts an FLP
// whose only trips cannot be placed on the route (e.g. recovery now before
// launch). Those trips are reported infeasible by the evaluator anyway; this
// helper does not delete an FLP that a (broken) trip still names, so a broken
// trip is never hidden by clean-up. Equal to the evaluator's set whenever
// every trip is placed (checked in tests/test_fuzz_search_infra.cpp).
inline std::vector<tdr::model::NodeId> unused_flps_in_route(const tdr::model::Solution& s, const tdr::model::NodeRegistry& nodes) {
    std::unordered_set<tdr::model::NodeId> used;
    for (const auto& t : s.drone_trips) used.insert(t.launch_node);
    std::vector<tdr::model::NodeId> out;
    for (tdr::model::NodeId v : s.truck_route) if (is_flp(nodes, v) && used.count(v) == 0) out.push_back(v);
    return out;
}

// EXPLICIT clean-up: removes FLP entries that no trip launches from and
// returns them (route order). It never removes an FLP that a trip uses and
// never touches customers or trips. Callers must count/log the result
// (D6: "record how many ... and why").
inline std::vector<tdr::model::NodeId> drop_unused_flps(tdr::model::Solution& s, const tdr::model::NodeRegistry& nodes) {
    const std::vector<tdr::model::NodeId> unused = unused_flps_in_route(s, nodes);
    if (unused.empty()) return unused;
    const std::unordered_set<tdr::model::NodeId> dead(unused.begin(), unused.end());
    s.truck_route.erase(std::remove_if(s.truck_route.begin(), s.truck_route.end(),
                                       [&](tdr::model::NodeId v) { return dead.count(v) != 0; }),
                        s.truck_route.end());
    return unused;
}

// ---- drone identity ----------------------------------------------------------

// For a single changed trip: the lowest drone id u in [0, num_drones) such
// that no OTHER trip flown by u has a route-position interval
// [launch, recovery] overlapping this trip's interval (closed intervals;
// constraint (28) requires the next launch strictly after the previous
// recovery). Other trips keep their ids (drone identity is preserved);
// trips whose launch/recovery cannot be located are ignored. Returns
// nullopt if this trip's own interval cannot be located or every drone is
// busy. This chooses an id; whether the result is feasible is still the
// evaluator's call.
inline std::optional<int> lowest_free_drone_for_trip(const tdr::model::Solution& s, std::size_t trip_index, int num_drones) {
    const auto& route = s.truck_route;
    const auto interval = [&](const tdr::model::DroneTrip& t) -> std::optional<std::pair<std::size_t, std::size_t>> {
        const auto lp = tdr::timing::detail::find_route_position(route, t.launch_node);
        if (!lp) return std::nullopt;
        const auto rp = tdr::timing::detail::find_route_position_from(route, t.recovery_node, *lp + 1);
        if (!rp) return std::nullopt;
        return std::make_pair(*lp, *rp);
    };
    const auto mine = interval(s.drone_trips[trip_index]);
    if (!mine) return std::nullopt;
    std::vector<bool> busy(static_cast<std::size_t>(std::max(0, num_drones)), false);
    for (std::size_t i = 0; i < s.drone_trips.size(); ++i) {
        if (i == trip_index) continue;
        const int u = s.drone_trips[i].drone_id;
        if (u < 0 || u >= num_drones) continue;
        const auto other = interval(s.drone_trips[i]);
        if (!other) continue;
        const bool disjoint = other->second < mine->first || mine->second < other->first;
        if (!disjoint) busy[static_cast<std::size_t>(u)] = true;
    }
    for (int u = 0; u < num_drones; ++u) if (!busy[static_cast<std::size_t>(u)]) return u;
    return std::nullopt;
}

} // namespace tdr::search
