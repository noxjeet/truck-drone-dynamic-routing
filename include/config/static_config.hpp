#pragma once

// Centralised configuration for the STATIC paper reproduction (M1,
// 9 Nov 2026). Nothing that an experiment varies is hard-coded in the
// algorithm headers: it lives here, with its provenance. Units are SI
// (metres, seconds, metres/second) everywhere; reports show minutes
// alongside seconds (DECISIONS D10).
//
// Tag legend (same as the rest of the repository):
//   [PAPER]       stated by Han, Chi & Sun 2026 for this quantity;
//   [ASSUMPTION]  our choice where the paper is silent or contradictory;
//                 every such default is listed in DECISIONS.md ("M1").
//
// Scope in M1: instance generation, vehicle parameters, FLP spacing,
// neighbourhood candidate window and the seed policy. VNS parameters,
// shake parameters and stopping criteria are added in M6/M7, runner and
// CLI overrides in M8 -- deliberately not here yet.

#include <cstdint>
#include <string>
#include <vector>

namespace tdr::config {

enum class DepotPlacement {
    Centre,   // centre of the square (default, DECISION D9)
    Corner,   // (0, 0)
    Explicit, // (depot_x_m, depot_y_m)
};

struct InstanceConfig {
    int num_customers = 10;             // [PAPER, Sec 6.1] 10 / 20 / 50 for small / medium / large
    double area_side_m = 60000.0;       // [PAPER, Sec 6.1] 60 x 60 km square
    DepotPlacement depot_placement = DepotPlacement::Centre; // [ASSUMPTION, D9] the paper does not say where the depot is
    double depot_x_m = 0.0;             // only used when depot_placement == Explicit
    double depot_y_m = 0.0;
    int dataset_index = 0;              // which of the "four datasets" per size; selects the instance seed
    double truck_service_time_s = 30.0; // [PAPER, Sec 6.1] customer service time 30 s (Se1)
    double drone_service_time_s = 30.0; // [PAPER, Sec 6.1] same value used for drone service (Se2)
    double theta = 1.0;                 // [PAPER, Sec 6.1] load coefficient theta = 1
};

struct VehicleConfig {
    double truck_speed_mps = 15.0;      // [PAPER, Sec 6.1]
    double drone_speed_mps = 12.0;      // [PAPER, Sec 6.1] (Sec 6.3 uses other values for its own sweeps)
    double endurance_s = 1800.0;        // [PAPER, Sec 6.1] 30 minutes
    double launch_prep_time_s = 60.0;   // [PAPER, Sec 6.1] SL
    double recovery_time_s = 40.0;      // [PAPER, Sec 6.1] SR
    int num_drones = 2;                 // [PAPER, Sec 6.1] default fleet
};

struct FlpConfig {
    // d in eq. (41), n = floor(D/d). [ASSUMPTION] The main experiments state
    // no value; 2000 m is the spacing the paper uses in its Sec 6.3 sensitivity
    // analyses. The count rule itself and the equal-ratio placement are the
    // generator's documented assumptions (flp/flexible_launch_point_generator.hpp).
    double unit_division_length_m = 2000.0;
};

struct NeighbourhoodConfig {
    // [ASSUMPTION, D1] "adjacent" candidates for N1-N8: two route entities are
    // adjacent iff their skeleton positions differ by at most this many truck
    // nodes (exact definition: search/candidate_window.hpp and DECISIONS.md "M1").
    int adjacency_window = 3;
    int n1_repeats = 2;                 // [ASSUMPTION, D2] paper: "several times"
    int n2_repeats = 2;                 // [ASSUMPTION, D2]
};

struct SeedConfig {
    // [ASSUMPTION] the paper gives no seed policy. Streams for instance,
    // initial solution, VNS and shakes are derived from this one value
    // (util/rng.hpp: derive_seed), so one number reproduces a whole experiment.
    std::uint64_t base_seed = 20261109ULL;
};

struct StaticConfig {
    InstanceConfig instance;
    VehicleConfig vehicle;
    FlpConfig flp;
    NeighbourhoodConfig neighbourhood;
    SeedConfig seed;
};

// Returns every problem found (empty = valid). Never throws.
inline std::vector<std::string> validate(const StaticConfig& c) {
    std::vector<std::string> e;
    const auto positive = [&](double v, const char* name) {
        if (!(v > 0.0) || v != v) e.push_back(std::string(name) + " must be a positive finite number");
    };
    const auto non_negative = [&](double v, const char* name) {
        if (!(v >= 0.0) || v != v) e.push_back(std::string(name) + " must be a non-negative finite number");
    };
    if (c.instance.num_customers < 1) e.push_back("instance.num_customers must be >= 1");
    positive(c.instance.area_side_m, "instance.area_side_m");
    non_negative(c.instance.truck_service_time_s, "instance.truck_service_time_s");
    non_negative(c.instance.drone_service_time_s, "instance.drone_service_time_s");
    non_negative(c.instance.theta, "instance.theta");
    if (c.instance.dataset_index < 0) e.push_back("instance.dataset_index must be >= 0");
    if (c.instance.depot_placement == DepotPlacement::Explicit) {
        if (!(c.instance.depot_x_m >= 0.0 && c.instance.depot_x_m <= c.instance.area_side_m) ||
            !(c.instance.depot_y_m >= 0.0 && c.instance.depot_y_m <= c.instance.area_side_m)) {
            e.push_back("explicit depot must lie inside the square [0, area_side_m]^2");
        }
    }
    positive(c.vehicle.truck_speed_mps, "vehicle.truck_speed_mps");
    positive(c.vehicle.drone_speed_mps, "vehicle.drone_speed_mps");
    positive(c.vehicle.endurance_s, "vehicle.endurance_s");
    non_negative(c.vehicle.launch_prep_time_s, "vehicle.launch_prep_time_s");
    non_negative(c.vehicle.recovery_time_s, "vehicle.recovery_time_s");
    if (c.vehicle.num_drones < 1) e.push_back("vehicle.num_drones must be >= 1");
    positive(c.flp.unit_division_length_m, "flp.unit_division_length_m");
    if (c.neighbourhood.adjacency_window < 1) e.push_back("neighbourhood.adjacency_window must be >= 1");
    if (c.neighbourhood.n1_repeats < 1) e.push_back("neighbourhood.n1_repeats must be >= 1");
    if (c.neighbourhood.n2_repeats < 1) e.push_back("neighbourhood.n2_repeats must be >= 1");
    return e;
}

} // namespace tdr::config
