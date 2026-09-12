#pragma once

#include <limits>

#include "model/parameters.hpp"

namespace tdr::graph {

// Generic distance -> time conversion at a constant speed. Not paper
// terminology by itself -- it is the shared arithmetic that both
// truck_travel_time() and drone_travel_time() below are built from,
// since [PAPER, Assumption 2] states both vehicle types move at a
// constant, own speed (time = distance / speed).
//
// A non-positive speed has no physical meaning here (a misconfigured
// TruckParameters/DroneParameters, e.g. a still-default-constructed
// speed of 0.0). Rather than dividing by zero/returning a silently wrong
// number, this returns +infinity -- an unmistakably "unusable" travel
// time that will not be confused with any real result. This is an
// arithmetic safety guard, not a feasibility rule: it makes no judgement
// about routes, endurance, or timing.
inline double travel_time_for_distance(double distance, double speed) {
    if (speed <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    return distance / speed;
}

// [PAPER] Table 2: d_ij, the truck's travel time on arc (i, j).
inline double truck_travel_time(double distance, const tdr::model::TruckParameters& truck) {
    return travel_time_for_distance(distance, truck.speed);
}

// [PAPER] Table 2: d'_ij, the drone's travel time on arc (i, j).
//
// IMPORTANT: this is the raw / physical flight time only. Paper 1
// separately defines a *generalized* flight time (eq. 35:
// t_j = f_j * (1 + theta_j)) that inflates a drone's actual flight time
// f_j by a load-dependent coefficient theta_j when checking battery
// endurance and downstream timing. This function computes f_j (the
// "d'_ij" of Table 2), not t_j -- applying theta_j is deliberately left
// to the timing/feasibility engine (Phase E/F), which already has
// access to the customer's theta via CustomerData
// (model/customer.hpp::generalized_flight_time_theta). Folding that
// transform in here would blur exactly the distinction the plan's
// timing reconstruction (paper1-implementation-plan.md Sec. 11) is
// built around.
inline double drone_travel_time(double distance, const tdr::model::DroneParameters& drone) {
    return travel_time_for_distance(distance, drone.speed);
}

} // namespace tdr::graph
