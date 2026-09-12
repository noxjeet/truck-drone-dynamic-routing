#include "fixtures/small_graph_fixture.hpp"
#include "flp/flexible_launch_point_generator.hpp"
#include "model/node.hpp"
#include "model/truck_route.hpp"

#include <cmath>
#include <iostream>
#include <set>
#include <string>

using namespace tdr::flp;
using namespace tdr::model;

namespace {

int g_failures = 0;

void check(bool condition, const std::string& description) {
    if (condition) {
        std::cout << "[PASS] " << description << "\n";
    } else {
        std::cout << "[FAIL] " << description << "\n";
        ++g_failures;
    }
}

void check_near(double a, double b, double tol, const std::string& description) {
    check(std::fabs(a - b) <= tol,
          description + " (got " + std::to_string(a) + ", expected " + std::to_string(b) + ")");
}

// Checks that `point` lies on the segment from `from` to `to` at the
// given ratio, using plain linear interpolation -- an independent
// recomputation from the generator's own formula, for cross-checking.
bool lies_on_segment_at_ratio(const Point2D& point, const Point2D& from, const Point2D& to, double ratio, double tol) {
    const double expected_x = from.x + ratio * (to.x - from.x);
    const double expected_y = from.y + ratio * (to.y - from.y);
    return std::fabs(point.x - expected_x) <= tol && std::fabs(point.y - expected_y) <= tol;
}

} // namespace

int main() {
    NodeRegistry nodes = tdr::test_fixtures::build_small_graph_fixture();
    // Fixture reminder: 0=Depot(0,0), 1=Customer(3,4), 2=Customer(6,8),
    // 3=Customer(-3,4), 4=FlexibleLaunchPoint(0,4). Generated points
    // below use fresh ids starting at 100+ so they never collide with
    // these.

    // 1. A simple straight truck-route arc produces launch points lying on that arc.
    {
        TruckRoute route = {0, 1}; // (0,0) -> (3,4), length 5
        FlpGenerationPolicy policy;
        policy.points_per_arc = 1;
        FlpGenerationResult result = generate_flexible_launch_points(route, nodes, policy, /*next_id=*/100);

        check(result.points.size() == 1, "Single arc with points_per_arc=1 produces exactly one point");
        const auto& p = result.points.front();
        check(p.data.arc_from == 0 && p.data.arc_to == 1, "Generated point records the correct arc endpoints");
        check_near(p.data.position_ratio, 0.5, 1e-12, "Midpoint ratio is 0.5 for a single point per arc");
        check_near(p.node.coordinates.x, 1.5, 1e-9, "Midpoint x-coordinate is correct");
        check_near(p.node.coordinates.y, 2.0, 1e-9, "Midpoint y-coordinate is correct");
        check(p.node.type == NodeType::FlexibleLaunchPoint, "Generated node has type FlexibleLaunchPoint");
        check(p.node.id == 100, "Generated node id starts at the requested next_id");
        check(result.next_available_id == 101, "next_available_id advances by the number of points generated");
    }

    // 2. Multiple consecutive truck-route arcs produce launch points only on those arcs.
    // 3. No launch point is generated on an untraversed edge (here, (0,2) is never an arc).
    {
        TruckRoute route = {0, 1, 2}; // arcs (0,1) and (1,2), NOT (0,2)
        FlpGenerationPolicy policy;
        policy.points_per_arc = 1;
        FlpGenerationResult result = generate_flexible_launch_points(route, nodes, policy, 100);

        check(result.points.size() == 2, "Two consecutive arcs each produce one point");

        bool saw_arc_0_1 = false, saw_arc_1_2 = false, saw_untraversed_0_2 = false;
        for (const auto& p : result.points) {
            if (p.data.arc_from == 0 && p.data.arc_to == 1) saw_arc_0_1 = true;
            if (p.data.arc_from == 1 && p.data.arc_to == 2) saw_arc_1_2 = true;
            if (p.data.arc_from == 0 && p.data.arc_to == 2) saw_untraversed_0_2 = true;
        }
        check(saw_arc_0_1, "A point was generated on traversed arc (0,1)");
        check(saw_arc_1_2, "A point was generated on traversed arc (1,2)");
        check(!saw_untraversed_0_2, "No point was generated on the untraversed edge (0,2)");

        check(result.points[0].route_arc_index == 0, "First point is tagged with route_arc_index 0");
        check(result.points[1].route_arc_index == 1, "Second point is tagged with route_arc_index 1");
    }

    // 4. Generated launch points have valid unique IDs and correct NodeType.
    {
        TruckRoute route = {0, 1, 2, 3, 0};
        FlpGenerationPolicy policy;
        policy.points_per_arc = 2;
        FlpGenerationResult result = generate_flexible_launch_points(route, nodes, policy, 100);

        std::set<NodeId> ids;
        bool all_flex_type = true;
        for (const auto& p : result.points) {
            ids.insert(p.node.id);
            if (p.node.type != NodeType::FlexibleLaunchPoint) all_flex_type = false;
        }
        check(ids.size() == result.points.size(), "All generated node ids are unique");
        check(all_flex_type, "Every generated node has type FlexibleLaunchPoint");
    }

    // 5. Generated coordinates are geometrically consistent with their associated arc.
    {
        TruckRoute route = {0, 3}; // (0,0) -> (-3,4), length 5
        FlpGenerationPolicy policy;
        policy.points_per_arc = 1;
        FlpGenerationResult result = generate_flexible_launch_points(route, nodes, policy, 100);

        check(result.points.size() == 1, "Single-arc route produces one point for geometric check");
        const auto& p = result.points.front();
        const Point2D& from = nodes.at(p.data.arc_from).coordinates;
        const Point2D& to = nodes.at(p.data.arc_to).coordinates;
        check(lies_on_segment_at_ratio(p.node.coordinates, from, to, p.data.position_ratio, 1e-9),
              "Generated point's coordinates match independent linear interpolation at its recorded ratio");
    }

    // 6. A route containing the depot and multiple customers is handled correctly
    // (including the last-node-to-depot arc, treated the same as any other).
    {
        TruckRoute route = {0, 1, 2, 3, 0}; // 4 arcs: (0,1),(1,2),(2,3),(3,0)
        FlpGenerationPolicy policy;
        policy.points_per_arc = 1;
        FlpGenerationResult result = generate_flexible_launch_points(route, nodes, policy, 100);

        check(result.points.size() == 4, "A 4-arc depot-customers-depot loop produces one point per arc");
        std::set<std::size_t> arc_indices_seen;
        for (const auto& p : result.points) arc_indices_seen.insert(p.route_arc_index);
        check(arc_indices_seen.size() == 4, "All four route arc positions (including the closing depot arc) are represented");
        check(arc_indices_seen.count(3) == 1, "The last-node-to-depot arc (index 3) produced a point, same as any other arc");
    }

    // 7. Endpoint behavior is deterministic (repeated calls agree).
    // 10. The same deterministic input and configuration produces the same launch points.
    {
        TruckRoute route = {0, 1, 2};
        FlpGenerationPolicy policy;
        policy.points_per_arc = 2;

        FlpGenerationResult first = generate_flexible_launch_points(route, nodes, policy, 100);
        FlpGenerationResult second = generate_flexible_launch_points(route, nodes, policy, 100);

        check(first.points.size() == second.points.size(), "Repeated calls produce the same number of points");
        check(first.next_available_id == second.next_available_id, "Repeated calls agree on next_available_id");
        bool all_match = true;
        for (std::size_t i = 0; i < first.points.size() && i < second.points.size(); ++i) {
            const auto& a = first.points[i];
            const auto& b = second.points[i];
            if (a.node.id != b.node.id || a.node.type != b.node.type ||
                a.node.coordinates.x != b.node.coordinates.x || a.node.coordinates.y != b.node.coordinates.y ||
                a.data.arc_from != b.data.arc_from || a.data.arc_to != b.data.arc_to ||
                a.data.position_ratio != b.data.position_ratio || a.route_arc_index != b.route_arc_index) {
                all_match = false;
                break;
            }
        }
        check(all_match, "Repeated calls with identical inputs produce field-for-field identical points");
    }

    // 8. A zero-length/coincident route arc is handled explicitly.
    {
        // Same node id twice in a row.
        TruckRoute route_same_id = {0, 0, 1};
        FlpGenerationPolicy policy;
        policy.points_per_arc = 1;
        FlpGenerationResult result_same_id = generate_flexible_launch_points(route_same_id, nodes, policy, 100);
        check(result_same_id.points.size() == 1,
              "A repeated-node (zero-length) arc contributes zero points; only the real arc (0,1) produces one");
        check(result_same_id.points.front().data.arc_from == 0 && result_same_id.points.front().data.arc_to == 1,
              "The single generated point belongs to the genuine arc, not the zero-length one");

        // Two distinct node ids that happen to share the same coordinates.
        NodeRegistry coincident_nodes;
        coincident_nodes.add(Node{10, NodeType::Depot, Point2D{5.0, 5.0}});
        coincident_nodes.add(Node{11, NodeType::Customer, Point2D{5.0, 5.0}});
        TruckRoute route_coincident = {10, 11};
        FlpGenerationResult result_coincident = generate_flexible_launch_points(route_coincident, coincident_nodes, policy, 200);
        check(result_coincident.points.empty(),
              "An arc between two distinct-id but coincident-coordinate nodes produces zero points");
        check(result_coincident.next_available_id == 200,
              "next_available_id is unchanged when no points are generated");
    }

    // 9. The configured density/count policy produces the expected number of launch points.
    {
        TruckRoute route = {0, 1};
        FlpGenerationPolicy policy_three;
        policy_three.points_per_arc = 3;
        FlpGenerationResult result_three = generate_flexible_launch_points(route, nodes, policy_three, 100);

        check(result_three.points.size() == 3, "points_per_arc=3 on one arc produces exactly 3 points");
        check_near(result_three.points[0].data.position_ratio, 0.25, 1e-12, "First of 3 points sits at ratio 1/4");
        check_near(result_three.points[1].data.position_ratio, 0.50, 1e-12, "Second of 3 points sits at ratio 2/4");
        check_near(result_three.points[2].data.position_ratio, 0.75, 1e-12, "Third of 3 points sits at ratio 3/4");

        FlpGenerationPolicy policy_zero;
        policy_zero.points_per_arc = 0;
        FlpGenerationResult result_zero = generate_flexible_launch_points(route, nodes, policy_zero, 100);
        check(result_zero.points.empty(), "points_per_arc=0 generates no points at all");
        check(result_zero.next_available_id == 100, "next_available_id is unchanged when points_per_arc=0");

        FlpGenerationPolicy policy_negative;
        policy_negative.points_per_arc = -5;
        FlpGenerationResult result_negative = generate_flexible_launch_points(route, nodes, policy_negative, 100);
        check(result_negative.points.empty(), "A negative points_per_arc is treated as generating no points, not a crash");
    }

    std::cout << "\n" << (g_failures == 0 ? "ALL TESTS PASSED" : std::to_string(g_failures) + " TEST(S) FAILED") << "\n";
    return g_failures == 0 ? 0 : 1;
}
