#pragma once

// Dynamic extension (DM3): clean cut and editable gaps (math_spec §2, DM0 §4.2). Everything is computed from the cached evaluation of
// an InstalledPlan, never from a "literal truck position".
//   spans(B):  a trip spans skeleton position B iff launch_pos <= B < recovery_pos (recovery at the end depot = last route index)
//   clean cut: B = smallest skeleton position (excluding the end depot) with truck arrival >= theta that no trip spans.
//   nullopt   <=> no editable gap remains.

#include <algorithm>
#include <limits>
#include <optional>
#include <vector>

#include "dynamic/plan.hpp"
#include "search/route_ops.hpp"

namespace tdr::dynamic {

inline constexpr std::size_t kNoPos = std::numeric_limits<std::size_t>::max();

struct TripSpan {
    std::size_t launch = kNoPos;   // route position of the launch FLP
    std::size_t recovery = kNoPos; // route position of the recovery node (end depot -> last index)
    std::size_t trip_index = 0;
    tdr::model::NodeId customer = tdr::model::kInvalidNodeId;
};

inline std::vector<TripSpan> trip_spans(const tdr::model::Solution& s) {
    const auto& route = s.truck_route;
    auto find = [&](tdr::model::NodeId id) {
        for (std::size_t i = 0; i < route.size(); ++i) if (route[i] == id) return i;
        return kNoPos;
    };
    std::vector<TripSpan> out;
    for (std::size_t t = 0; t < s.drone_trips.size(); ++t) {
        const auto& d = s.drone_trips[t];
        TripSpan sp;
        sp.trip_index = t;
        sp.customer = d.service_customer;
        sp.launch = find(d.launch_node);
        sp.recovery = (!route.empty() && d.recovery_node == route.front()) ? route.size() - 1 : find(d.recovery_node);
        out.push_back(sp);
    }
    return out;
}

struct Cut {
    std::size_t skel_index = 0; // skeleton index B_s
    std::size_t route_pos = 0;  // raw route position of B
    double node_time = 0.0;     // truck arrival at B
};

inline bool spans(const std::vector<TripSpan>& trips, std::size_t pos) {
    for (const auto& t : trips) if (t.launch <= pos && pos < t.recovery) return true;
    return false;
}

inline std::optional<Cut> clean_cut(const InstalledPlan& P, double theta) {
    const auto& route = P.solution().truck_route;
    const auto sk = tdr::search::make_skeleton(route, P.world().nodes());
    const auto& arr = P.eval().truck_timing.arrival_time;
    const auto trips = trip_spans(P.solution());
    for (std::size_t s = 0; s + 1 < sk.size(); ++s) { // the end depot (last skeleton node) has no gap behind it
        const std::size_t pos = sk.route_position[s];
        if (arr[pos] < theta) continue;
        if (spans(trips, pos)) continue;
        return Cut{s, pos, arr[pos]};
    }
    return std::nullopt;
}

struct Gap {
    std::size_t skel = 0;  // skeleton index of a
    std::size_t a_pos = 0; // raw route positions of a and b
    std::size_t b_pos = 0;
};

inline std::vector<Gap> editable_gaps(const InstalledPlan& P, const Cut& cut) {
    const auto sk = tdr::search::make_skeleton(P.solution().truck_route, P.world().nodes());
    std::vector<Gap> out;
    for (std::size_t s = cut.skel_index; s + 1 < sk.size(); ++s) out.push_back(Gap{s, sk.route_position[s], sk.route_position[s + 1]});
    return out;
}

// math_spec §2: g is straddled iff some trip has launch < P(b) <= recovery. (A launch FLP on the gap's own arc is included.)
inline bool is_straddled(const std::vector<TripSpan>& trips, const Gap& g) {
    for (const auto& t : trips) if (t.launch < g.b_pos && g.b_pos <= t.recovery) return true;
    return false;
}

// True iff the gap carries FLP entries (a stale launch FLP after a truck insertion, CE-2).
inline bool gap_has_flps(const tdr::model::Solution&, const Gap& g) { return g.b_pos > g.a_pos + 1; }

} // namespace tdr::dynamic
