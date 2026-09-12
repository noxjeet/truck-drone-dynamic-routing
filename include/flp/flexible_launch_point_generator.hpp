#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

#include "model/flexible_launch_point.hpp"
#include "model/node.hpp"
#include "model/truck_route.hpp"

namespace tdr::flp {

// Baseline flexible-launch-point (FLP) placement policy.
//
// [PAPER] Sec 2.1/4.1: flexible launch points exist along the truck
// route, partitioned per traversed arc (i, j) as L_ij; a launch point is
// only usable if the truck's route actually traverses that arc
// (constraints 5-6, 13-15). Sec 2.3 additionally describes them
// qualitatively as dividing each arc "by equal proportions of the
// distances between two locations" -- i.e. evenly spaced along the arc.
//
// [ASSUMPTION] Everything about *how many* points to place, and at
// exactly which ratios, is our own choice. The paper does not specify a
// numeric rule for the main synthetic experiments: Table 4 gives the
// resulting counts (214/837/3493 launch points for the 10/20/50-customer
// instances) but not the generating rule, and the only fully numeric
// rule the paper gives (eq. 41, a fixed unit-division length swept over
// [1000, 5000] m) is explicitly scoped to the Sec 6.3.4 sensitivity
// analysis on a separate 7-customer/10-node instance -- NOT the main
// experiments. Transferring eq. 41 into this baseline would misrepresent
// it as the main-experiment rule, which the source does not support.
// See DECISIONS.md ("FLP baseline generation policy") for the complete
// reasoning.
//
// This baseline instead places a fixed, configurable number of points
// per traversed arc, evenly spaced along it (matching the qualitative
// "equal proportions" description without claiming to match Table 4's
// counts). It is deliberately isolated behind this policy struct and
// generate_flexible_launch_points() below so it can be swapped for a
// different rule later without touching anything that consumes its
// output.
struct FlpGenerationPolicy {
    // Number of launch points to place on each traversed arc, at ratios
    // 1/(n+1), 2/(n+1), ..., n/(n+1) along the arc (n = points_per_arc).
    // [ASSUMPTION] default of 1 (a single midpoint per arc) is the
    // simplest defensible baseline; it is a configuration value, not a
    // paper-derived constant.
    int points_per_arc = 1;

    // An arc shorter than this is treated as zero-length / coincident
    // (e.g. the route visits the same physical location twice in a row)
    // and receives no generated points at all -- there is nowhere new to
    // place a launch point that would not simply coincide with an
    // endpoint already on the route.
    double min_arc_length = 1e-9;
};

// One generated flexible launch point: the Node itself (id, type,
// coordinates) plus the FlexibleLaunchPointData recording which arc it
// belongs to and where on that arc it sits (Phase B types, unchanged).
// route_arc_index additionally records the position of this arc within
// the input truck_route (the arc is between truck_route[route_arc_index]
// and truck_route[route_arc_index + 1]), so a repeated arc (the same
// node pair appearing more than once in a route) does not become
// ambiguous -- FlexibleLaunchPointData::arc_from/arc_to alone identify
// the node pair, not the specific occurrence.
struct GeneratedFlexibleLaunchPoint {
    tdr::model::Node node;
    tdr::model::FlexibleLaunchPointData data;
    std::size_t route_arc_index = 0;
};

struct FlpGenerationResult {
    std::vector<GeneratedFlexibleLaunchPoint> points;
    // One past the highest node id assigned during this call -- pass
    // this back in as next_id on a subsequent call (e.g. generating for
    // another route in the same instance) to keep ids from colliding.
    tdr::model::NodeId next_available_id = tdr::model::kInvalidNodeId;
};

// Generates flexible launch points strictly on the arcs `truck_route`
// actually traverses: only consecutive pairs (truck_route[i],
// truck_route[i+1]) define an arc; nothing is generated on any other
// pair of nodes, traversed or not. This intentionally covers the
// depot-to-first-node and last-node-to-depot arcs the same way as every
// other arc -- truck_route is expected to already include the depot at
// both ends (as every route built elsewhere in this repository does), so
// no special-casing is needed here.
//
// Preconditions (not defensively checked, to keep this function purely
// geometric): every id in truck_route must already be registered in
// `nodes`, and `next_id` must not collide with any node id already in
// use anywhere in the instance (by `nodes` or by a prior generation
// call). Violating either is a caller bug, not a data condition this
// function is responsible for detecting.
//
// This function is pure: it does not mutate `nodes`, perform any I/O, or
// use randomness. The same arguments always produce the same result,
// which both makes it trivially safe to call repeatedly and matches the
// paper's own description of a fixed, non-random partition rule (as
// opposed to, say, the shake neighborhoods' explicit randomness, which
// belongs to a much later phase).
inline FlpGenerationResult generate_flexible_launch_points(
    const tdr::model::TruckRoute& truck_route,
    const tdr::model::NodeRegistry& nodes,
    const FlpGenerationPolicy& policy,
    tdr::model::NodeId next_id) {

    using tdr::model::FlexibleLaunchPointData;
    using tdr::model::Node;
    using tdr::model::NodeId;
    using tdr::model::NodeType;
    using tdr::model::Point2D;

    FlpGenerationResult result;
    result.next_available_id = next_id;

    if (truck_route.size() < 2 || policy.points_per_arc <= 0) {
        return result;
    }

    for (std::size_t arc_index = 0; arc_index + 1 < truck_route.size(); ++arc_index) {
        const NodeId u = truck_route[arc_index];
        const NodeId v = truck_route[arc_index + 1];
        const Point2D& pu = nodes.at(u).coordinates;
        const Point2D& pv = nodes.at(v).coordinates;

        const double dx = pv.x - pu.x;
        const double dy = pv.y - pu.y;
        const double arc_length = std::sqrt(dx * dx + dy * dy);
        if (arc_length < policy.min_arc_length) {
            continue; // zero-length/coincident arc: no new point to place
        }

        const int n = policy.points_per_arc;
        for (int k = 1; k <= n; ++k) {
            const double ratio = static_cast<double>(k) / static_cast<double>(n + 1);

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

} // namespace tdr::flp
