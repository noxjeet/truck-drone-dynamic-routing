#pragma once

#include <vector>

#include "model/drone_trip.hpp"
#include "model/truck_route.hpp"

namespace tdr::model {

// A complete static truck-and-drone solution: the truck's route plus
// every drone trip flown alongside it.
//
// [PAPER] Sec 5.1's "independent-interactive" representation: one truck
// route plus a separate collection of drone "blocks" (trips), linked
// back into the route via their launch/recovery node ids rather than
// being embedded in the route sequence itself.
//
// Deliberately minimal at this phase: no cached timing, no auxiliary
// lookup indices (e.g. "which trip launches from this node"), and no
// derived/validity state. Per the implementation plan (Sec 5, Sec 25),
// auxiliary indices are Phase H's job, once the neighborhoods that
// actually need them exist and their real access patterns are known;
// building them now would be guessing. Likewise, whether a Solution is
// complete, feasible, or optimal is entirely the timing/feasibility
// engine's job (Phase F) -- this struct only has to be *capable* of
// holding a full solution, not evaluate one.
struct Solution {
    TruckRoute truck_route;
    std::vector<DroneTrip> drone_trips;
};

} // namespace tdr::model
