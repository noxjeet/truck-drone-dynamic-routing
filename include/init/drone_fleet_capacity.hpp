#pragma once

// Phase F post-approval correctness patch (drone-fleet capacity across
// the WHOLE initial-solution construction, not just within one cluster).
// See DECISIONS.md, "Initial-solution construction (Phase F)" ->
// "Post-approval correctness patch" for the full write-up; this header's
// comments are a summary.
//
// [PAPER, constraint 28] "prevents relaunching a drone before it's
// actually been recovered" (paper1-understanding.md Sec 2.4). The exact
// MILP assigns each single-flight route s=(i,j,k) to a specific drone
// index w in U via drone-indexed variables (t_iw, l_iw); constraint 28
// forbids assigning a new flight to drone w before w's previous flight
// has been recovered. |U| (Table 2) is the fixed, finite number of such
// drone indices that exist at all.
//
// A drone's "busy interval" is therefore [launch_time,
// recovery_complete_time] -- both already computed, unmodified, by
// Phase E's tdr::timing::evaluate_solution() (DroneTripTiming). This
// file does NOT recompute either value; it only takes the finished
// values as input and answers a pure scheduling question about them:
// can these intervals be partitioned across at most `num_drones`
// physical drones such that no drone's two intervals overlap?
//
// [INFERENCE] That scheduling question is answered via the standard
// interval-scheduling / interval-graph-coloring fact: a set of closed
// intervals can be partitioned into <= k non-overlapping-per-group sets
// if and only if no time instant is covered by more than k intervals
// simultaneously ("maximum overlap" == the minimum number of groups
// needed, for interval graphs specifically). This is a textbook
// combinatorial fact, not a paper-specified algorithm or an invented
// heuristic -- computing "maximum simultaneous overlap" and comparing it
// to |U| is both NECESSARY and SUFFICIENT to determine whether some
// valid assignment of the given trips to <= |U| physical drones exists.
// It is classified [INFERENCE] (not [PAPER]) only because Paper 1 itself
// never states this equivalence explicitly -- it follows from combining
// [PAPER] facts (constraint 28's non-overlap rule, |U|'s fixed size)
// with an uncontroversial, well-known scheduling result, not from
// inventing new problem-specific machinery.
//
// [ASSUMPTION] A boundary touch (one trip's recovery_complete_time
// exactly equal to another's launch_time) is treated as NON-overlapping
// -- the drone is available for a new launch at the exact instant it
// finishes its previous recovery. This mirrors the half-open-interval /
// kTimeEpsilon-guarded "<=" convention already used throughout
// timing_engine.hpp for every other feasibility boundary.

#include <algorithm>
#include <cstddef>
#include <vector>

#include "timing/timing_engine.hpp"

namespace tdr::init {

struct DroneFleetCapacityResult {
    // The largest number of drone trips ever required to be
    // simultaneously "in flight" (launched but not yet recovered) at any
    // single instant, across all given trips.
    int max_simultaneous_drones = 0;
    // true iff max_simultaneous_drones <= num_drones (vacuously true for
    // zero trips).
    bool feasible = true;
};

// `timings` must be Phase E's own already-computed DroneTripTiming
// values (e.g. SolutionEvaluationResult::drone_trip_timings) -- this
// function reads only launch_time/recovery_complete_time from each and
// performs no timing computation of its own. Every entry is included in
// the sweep regardless of its own .feasible flag; callers are expected
// to only ever pass trips they already intend to keep (F4 never builds
// a trial including a trip it would reject for other reasons), but nothing
// here special-cases an infeasible entry -- capacity is purely a function
// of the given intervals.
inline DroneFleetCapacityResult check_drone_fleet_capacity(
    const std::vector<tdr::timing::DroneTripTiming>& timings, int num_drones) {
    DroneFleetCapacityResult out;
    if (timings.empty()) return out;

    struct Event {
        double time;
        int delta; // +1 at a trip's launch_time, -1 at its recovery_complete_time
    };
    std::vector<Event> events;
    events.reserve(timings.size() * 2);
    for (const auto& t : timings) {
        events.push_back(Event{t.launch_time, +1});
        events.push_back(Event{t.recovery_complete_time, -1});
    }

    // Sort by time; at an exactly-tied instant, process every -1 (a
    // drone becoming free) before any +1 (a drone being newly required),
    // so a trip ending exactly when another begins is correctly treated
    // as non-overlapping (see the [ASSUMPTION] above).
    std::sort(events.begin(), events.end(), [](const Event& a, const Event& b) {
        if (std::abs(a.time - b.time) > tdr::timing::kTimeEpsilon) return a.time < b.time;
        return a.delta < b.delta;
    });

    int running = 0;
    int max_running = 0;
    for (const auto& e : events) {
        running += e.delta;
        max_running = std::max(max_running, running);
    }
    out.max_simultaneous_drones = max_running;
    out.feasible = max_running <= num_drones;
    return out;
}

} // namespace tdr::init
