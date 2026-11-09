#pragma once

// Dynamic extension (DM3): installation invariants I1-I6 (DM0 §4.7) for a SINGLE TRUCK INSERTION of one new customer (the vertical
// slice's only move). I7 (the promise predicate) is a policy outcome, not a bug, and is not part of the slice.
//   I1 evaluator-feasible          I2 prefix identity (route [0..B], trips launched <= B, timing arrays [0..B], committed ETAs: bitwise)
//   I3 coverage (exactly the previous customers + the new one, each served once)
//   I4 outside the edit unchanged (removing the new customer from the route gives the previous route; trips equal)
//   I5 tail shift constant (checked only for non-straddled gaps; to 1e-9 relative)
//   I6 every changed or new service has ETA >= theta = s + omega

#include <cmath>
#include <cstring>
#include <set>
#include <string>
#include <vector>

#include "dynamic/cursor.hpp"

namespace tdr::dynamic {

struct InvariantReport {
    bool ok = true;
    std::vector<std::string> violations;
    void fail(const std::string& m) { ok = false; violations.push_back(m); }
};

namespace detail_inv {
inline bool bits(double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0; }
inline double eta_of(const tdr::timing::SolutionEvaluationResult& r, tdr::model::NodeId c, bool* found) {
    for (const auto& cr : r.customer_records) if (cr.customer_id == c) { *found = true; return cr.service_arrival_time; }
    *found = false;
    return 0.0;
}
inline bool trip_eq(const tdr::model::DroneTrip& a, const tdr::model::DroneTrip& b) {
    return a.drone_id == b.drone_id && a.launch_node == b.launch_node && a.service_customer == b.service_customer && a.recovery_node == b.recovery_node;
}
} // namespace detail_inv

inline InvariantReport check_single_insertion(const InstalledPlan& prev, const InstalledPlan& next, const Cut& cut, std::size_t gap_skel,
                                              tdr::model::NodeId new_customer, double theta) {
    using namespace detail_inv;
    InvariantReport R;
    const auto& pr = prev.solution().truck_route;
    const auto& nr = next.solution().truck_route;

    // I1
    if (!next.eval().feasible) R.fail("I1: candidate is not evaluator-feasible");
    if (!next.consistent()) R.fail("I1: cached evaluation does not match its solution/world");

    // locate the new customer
    std::size_t np = kNoPos;
    for (std::size_t i = 0; i < nr.size(); ++i) if (nr[i] == new_customer) np = i;
    if (np == kNoPos) { R.fail("I3: new customer is not on the truck route (slice inserts as a truck customer)"); return R; }

    // I3 coverage: customers of next world == customers of prev world + new; each appears exactly once in route or trips
    {
        std::set<tdr::model::NodeId> expect;
        for (const auto& kv : prev.world().customers()) expect.insert(kv.first);
        expect.insert(new_customer);
        std::set<tdr::model::NodeId> have;
        for (const auto& kv : next.world().customers()) have.insert(kv.first);
        if (have != expect) R.fail("I3: next world's customers differ from previous customers + new request");
        std::multiset<tdr::model::NodeId> served;
        for (auto id : nr) if (next.world().nodes().at(id).type == tdr::model::NodeType::Customer) served.insert(id);
        for (const auto& t : next.solution().drone_trips) served.insert(t.service_customer);
        for (auto id : expect) if (served.count(id) != 1) R.fail("I3: customer " + std::to_string(id) + " served " + std::to_string(served.count(id)) + " times");
        if (served.size() != expect.size()) R.fail("I3: served multiset has foreign entries");
    }

    // I4: removing the new customer reproduces the previous route; trips unchanged
    {
        auto tmp = nr;
        tmp.erase(tmp.begin() + static_cast<std::ptrdiff_t>(np));
        if (tmp != pr) R.fail("I4: route differs from previous route outside the inserted customer");
        const auto& a = prev.solution().drone_trips;
        const auto& b = next.solution().drone_trips;
        bool same = a.size() == b.size();
        for (std::size_t i = 0; same && i < a.size(); ++i) same = trip_eq(a[i], b[i]);
        if (!same) R.fail("I4: drone trips changed (slice inserts truck-only)");
    }

    // I2 prefix identity [0..B]
    {
        bool ok = nr.size() > cut.route_pos && np > cut.route_pos;
        for (std::size_t q = 0; ok && q <= cut.route_pos; ++q) ok = nr[q] == pr[q];
        if (!ok) R.fail("I2: route prefix [0..B] changed or the insertion lies at/before the cut");
        const auto& pa = prev.eval().truck_timing; const auto& na = next.eval().truck_timing;
        for (std::size_t q = 0; ok && q <= cut.route_pos; ++q)
            if (!bits(pa.arrival_time[q], na.arrival_time[q]) || !bits(pa.departure_time[q], na.departure_time[q])) { R.fail("I2: timing arrays [0..B] are not bit-identical"); break; }
        // committed ETAs: truck customers at positions <= B and customers of trips launched <= B
        const auto trips = trip_spans(prev.solution());
        std::vector<tdr::model::NodeId> committed;
        for (std::size_t q = 0; q <= cut.route_pos && q < pr.size(); ++q) if (prev.world().nodes().at(pr[q]).type == tdr::model::NodeType::Customer) committed.push_back(pr[q]);
        for (const auto& t : trips) if (t.launch <= cut.route_pos) committed.push_back(t.customer);
        for (auto c : committed) {
            bool f1, f2;
            const double e1 = eta_of(prev.eval(), c, &f1), e2 = eta_of(next.eval(), c, &f2);
            if (!f1 || !f2 || !bits(e1, e2)) { R.fail("I2: committed customer " + std::to_string(c) + " ETA changed"); break; }
        }
    }

    // I5 tail shift (non-straddled gaps only)
    {
        const auto sk = tdr::search::make_skeleton(pr, prev.world().nodes());
        if (gap_skel + 1 < sk.size()) {
            Gap g{gap_skel, sk.route_position[gap_skel], sk.route_position[gap_skel + 1]};
            if (!is_straddled(trip_spans(prev.solution()), g) && np == g.b_pos) {
                const auto& pd = prev.eval().truck_timing.departure_time; const auto& nd = next.eval().truck_timing.departure_time;
                const auto& pa = prev.eval().truck_timing.arrival_time; const auto& na = next.eval().truck_timing.arrival_time;
                const double D = nd[g.b_pos + 1] - pd[g.b_pos];
                for (std::size_t q = g.b_pos + 1; q < pr.size(); ++q) {
                    const double s1 = nd[q + 1] - pd[q] - D, s2 = na[q + 1] - pa[q] - D;
                    if (std::fabs(s1) > 1e-9 * std::max(1.0, std::fabs(pd[q])) || std::fabs(s2) > 1e-9 * std::max(1.0, std::fabs(pa[q]))) { R.fail("I5: tail shift is not constant"); break; }
                }
            }
        } else R.fail("I5: gap index out of range");
    }

    // I6 changed or new service has ETA >= theta
    {
        bool f;
        const double en = eta_of(next.eval(), new_customer, &f);
        if (!f || en < theta - 1e-9) R.fail("I6: new customer's ETA precedes s + omega");
        for (const auto& cr : prev.eval().customer_records) {
            bool f2;
            const double e2 = eta_of(next.eval(), cr.customer_id, &f2);
            if (f2 && std::fabs(e2 - cr.service_arrival_time) > 1e-9 * std::max(1.0, std::fabs(cr.service_arrival_time)) && e2 < theta - 1e-9)
                R.fail("I6: customer " + std::to_string(cr.customer_id) + " changed service before s + omega");
        }
    }
    return R;
}

} // namespace tdr::dynamic
