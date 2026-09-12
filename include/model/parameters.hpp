#pragma once

namespace tdr::model {

// [PAPER] Table 2. Truck-side operating parameters. Per-customer service
// time (Se1_i) is NOT here -- it lives on CustomerData (customer.hpp)
// because the paper indexes it by customer, not globally.
struct TruckParameters {
    double speed = 0.0; // [PAPER, Assumption 2] truck travel time on an arc = distance / speed
};

// [PAPER] Table 2. Drone-side operating parameters, shared by every
// drone in the fleet. Paper 1 describes drones as homogeneous (a single
// speed, a single endurance) -- it does not discuss drone-to-drone
// variation, so a single shared DroneParameters plus a fleet size is
// sufficient; nothing here forecloses giving a specific drone its own
// parameters later if that ever becomes necessary, but the paper gives
// no reason to build that now.
struct DroneParameters {
    double speed = 0.0;               // [PAPER, Assumption 2] d'_ij = distance / speed
    double endurance_seconds = 0.0;   // e [PAPER, Table 2] battery flight-time cap (constraints 29/30, 36/37)
    double launch_prep_time_sl = 0.0; // SL [PAPER, Table 2] consumed at the launch point before departure (constraint 21)
    double recovery_time_sr = 0.0;    // SR [PAPER, Table 2] consumed at the recovery node after landing
    int num_drones = 0;               // |U| [PAPER, Table 2] fleet size / concurrency cap
};

} // namespace tdr::model
