#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "model/flexible_launch_point.hpp"
#include "model/node.hpp"
#include "model/truck_route.hpp"

namespace tdr::flp {

// Flexible-launch-point (FLP) generation.
//
// [PAPER] What the paper states about FLP placement:
//   - p3: launch points are "evenly distributed across the truck's route";
//   - p8: launches are allowed "at any two customer locations or between a
//     customer location and the depot", the method "divides a series of
//     feasible launch points by equal proportions of the distances between
//     two locations";
//   - p9 / Table 2: L_{i,j} is the set of launch points between node i and
//     node j (i in C0, j in C_E) -- every FLP belongs to exactly one arc --
//     and "If the truck does not pass through arc (i,j), it cannot access
//     the launch point in set L_ij";
//   - p18: "the number of flexible points inserted is proportional to the
//     truck's driving distance".
// So the number of points on an arc depends on that arc's length, and
// depot arcs are included.
//
// [PAPER, sensitivity analysis only] Sec 6.3.4 (p33) gives the only
// explicit counting formula, eq. (41): n = floor(D/d), with D the distance
// between two successive locations and d the "length of unit division",
// swept over [1000, 5000] m; Sec 6.3.1-6.3.3 use 2 km intervals. The main
// experiments (Sec 6.1, Tables 3/4) state NO value of d.
//
// UNRESOLVED (professor decision pending): the main-experiment value of d.
// This module therefore has NO default for d: FlpCountRule::UnitDivisionLength
// requires the caller to set unit_division_length > 0 explicitly, and
// generation fails loudly (FlpGenerationResult::ok == false) otherwise.
// Table 4's counts (214/837/3493) are NOT used as a calibration target.
//
// [ASSUMPTION] Placement within an arc: the n points sit at ratios
// m/(n+1), m = 1..n, i.e. they divide the arc into n+1 equal parts
// ("equal proportions", p8; "equitably divided", p33). The paper does not
// say whether points are instead placed at multiples of d from the arc's
// start. The eq. (41) form n = floor(D/d) is used for the count.
//
// FlpCountRule::FixedCountPerArc (points_per_arc points on every arc,
// regardless of length) is the pre-29-Sep-2026 baseline. It is NOT
// supported by the paper (it ignores p18's distance proportionality) and is
// kept only as an explicit, opt-in comparison/test mode -- never a default.
enum class FlpCountRule {
    UnitDivisionLength, // n = floor(D / d)  [paper structure; d must be configured]
    FixedCountPerArc,   // n = points_per_arc  [NOT paper-supported; opt-in only]
};

struct FlpGenerationPolicy {
    FlpCountRule count_rule = FlpCountRule::UnitDivisionLength;

    // d, in the same distance units as node coordinates. 0 = "not
    // configured": deliberately no default, because the paper does not
    // specify the main-experiment value (see above).
    double unit_division_length = 0.0;

    // Only used by FlpCountRule::FixedCountPerArc.
    int points_per_arc = 1;

    // An arc shorter than this is treated as zero-length / coincident (e.g.
    // two distinct nodes at identical coordinates) and receives no points.
    double min_arc_length = 1e-9;
};

// One generated flexible launch point: the Node itself (id, type,
// coordinates) plus the FlexibleLaunchPointData recording which arc L_{i,j}
// it belongs to and where on that arc it sits. route_arc_index is the index
// of the arc within the arc list passed to the generator (for the
// route-based overload: the arc between truck_route[route_arc_index] and
// truck_route[route_arc_index + 1]).
struct GeneratedFlexibleLaunchPoint {
    tdr::model::Node node;
    tdr::model::FlexibleLaunchPointData data;
    std::size_t route_arc_index = 0;
};

struct FlpGenerationResult {
    std::vector<GeneratedFlexibleLaunchPoint> points;
    // One past the highest node id assigned during this call.
    tdr::model::NodeId next_available_id = tdr::model::kInvalidNodeId;
    // false iff the policy is not usable (e.g. UnitDivisionLength with
    // d <= 0 or non-finite); `points` is then empty and `error` explains.
    bool ok = true;
    std::string error;
};

using Arc = std::pair<tdr::model::NodeId, tdr::model::NodeId>;

// Number of points to place on an arc of the given length under `policy`.
// Returns -1 if the policy itself is invalid.
inline int flp_count_for_arc(double arc_length, const FlpGenerationPolicy& policy) {
    if (arc_length < policy.min_arc_length) return 0;
    switch (policy.count_rule) {
        case FlpCountRule::UnitDivisionLength: {
            const double d = policy.unit_division_length;
            if (!(d > 0.0) || !std::isfinite(d)) return -1;
            // eq. (41): n = floor(D/d). The tiny epsilon only protects exact
            // multiples (e.g. D = 20, d = 10) against floating-point
            // representation error; it never changes a genuine fraction.
            return static_cast<int>(std::floor(arc_length / d + 1e-9));
        }
        case FlpCountRule::FixedCountPerArc:
            return std::max(0, policy.points_per_arc);
    }
    return -1;
}

// Generates L_{a,b} for every arc (a, b) in `arcs`, in the given order.
// Pure: no mutation of `nodes`, no I/O, no randomness. Preconditions (not
// checked): every arc endpoint is registered in `nodes`, and `next_id`
// does not collide with any id already in use.
inline FlpGenerationResult generate_flexible_launch_points_on_arcs(
    const std::vector<Arc>& arcs,
    const tdr::model::NodeRegistry& nodes,
    const FlpGenerationPolicy& policy,
    tdr::model::NodeId next_id) {

    using tdr::model::FlexibleLaunchPointData;
    using tdr::model::Node;
    using tdr::model::NodeType;
    using tdr::model::Point2D;

    FlpGenerationResult result;
    result.next_available_id = next_id;

    if (policy.count_rule == FlpCountRule::UnitDivisionLength &&
        (!(policy.unit_division_length > 0.0) || !std::isfinite(policy.unit_division_length))) {
        result.ok = false;
        result.error =
            "FLP unit division length d is not configured (must be > 0). The paper does not specify "
            "the main-experiment value of d (eq. (41) appears only in the Sec 6.3.4 sensitivity "
            "analysis), so no default is assumed; set FlpGenerationPolicy::unit_division_length.";
        return result;
    }

    for (std::size_t arc_index = 0; arc_index < arcs.size(); ++arc_index) {
        const auto [u, v] = arcs[arc_index];
        const Point2D& pu = nodes.at(u).coordinates;
        const Point2D& pv = nodes.at(v).coordinates;
        const double dx = pv.x - pu.x;
        const double dy = pv.y - pu.y;
        const double arc_length = std::sqrt(dx * dx + dy * dy);

        const int n = flp_count_for_arc(arc_length, policy);
        for (int m = 1; m <= n; ++m) {
            const double ratio = static_cast<double>(m) / static_cast<double>(n + 1);

            Node node;
            node.id = result.next_available_id;
            node.type = NodeType::FlexibleLaunchPoint;
            node.coordinates = Point2D{pu.x + ratio * dx, pu.y + ratio * dy};

            FlexibleLaunchPointData data;
            data.node_id = node.id;
            data.arc_from = u;
            data.arc_to = v;
            data.position_ratio = ratio;

            result.points.push_back(GeneratedFlexibleLaunchPoint{node, data, arc_index});
            ++result.next_available_id;
        }
    }
    return result;
}

// Route-based convenience overload: generates L_{a,b} for every consecutive
// pair (truck_route[i], truck_route[i+1]) -- including the depot arcs at
// both ends -- and for nothing else.
inline FlpGenerationResult generate_flexible_launch_points(
    const tdr::model::TruckRoute& truck_route,
    const tdr::model::NodeRegistry& nodes,
    const FlpGenerationPolicy& policy,
    tdr::model::NodeId next_id) {
    std::vector<Arc> arcs;
    if (truck_route.size() >= 2) {
        arcs.reserve(truck_route.size() - 1);
        for (std::size_t i = 0; i + 1 < truck_route.size(); ++i) arcs.emplace_back(truck_route[i], truck_route[i + 1]);
    }
    return generate_flexible_launch_points_on_arcs(arcs, nodes, policy, next_id);
}

} // namespace tdr::flp
