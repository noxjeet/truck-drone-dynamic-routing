#pragma once

// Shared vocabulary for the Phase G neighbourhood operators (M2, 9 Nov 2026).
//
// An operator takes an evaluator-feasible Solution and a SearchContext and
// returns an OperatorResult: the best FEASIBLE candidate (lowest evaluator
// objective, first-generated wins ties), plus complete statistics: how many
// candidates were generated, evaluated and accepted, and for every rejected
// candidate WHY. Operators never mutate their input (copy-and-modify), never
// decide timing/endurance/(28)/|U| feasibility themselves (the evaluator does,
// and its verdict is classified, not interpreted), and never repair a trip
// silently: a move that would leave a drone trip on a vanished arc is reported
// as RejectReason::AffectedTrips with the trips named.
//
// No hidden state: randomness enters only through an explicit tdr::util::Rng
// (used solely to pick the anchor customer / trip), and every enumeration is
// in a fixed order, so equal inputs + equal RNG state give equal results.

#include <array>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "flp/flp_arc_index.hpp"
#include "graph/spatial_network.hpp"
#include "model/customer.hpp"
#include "model/node.hpp"
#include "model/solution.hpp"
#include "timing/timing_engine.hpp"

namespace tdr::search {

// Everything an operator needs to read. References only; the caller owns the
// data and keeps it alive (and unchanged) for the duration of the call.
struct SearchContext {
    const tdr::graph::SpatialNetwork& network;                                      // the FULL eager network (depot + customers + FLP pool)
    const std::unordered_map<tdr::model::NodeId, tdr::model::CustomerData>& customers;
    const tdr::timing::FlpDataMap& flp_data;                                        // arc ownership of every FLP (evaluator input)
    const tdr::flp::FlpArcIndex& flp_index;                                         // arc -> FLPs (built from flp_data)
    int adjacency_window = 3;                                                       // D1: NeighbourhoodConfig::adjacency_window

    const tdr::model::NodeRegistry& nodes() const { return network.nodes(); }
    int num_drones() const { return network.drone_parameters().num_drones; }
};

enum class OperatorId { N1, N2, N3, N4, N5, N6, N7, N8 };

enum class MoveKind {
    N4Relocate, // truck customer moved to another gap of the truck route
    N4Exchange, // truck customer exchanged with another truck customer
    N5Swap,     // truck customer <-> drone customer, with a re-selected trip
    N7TruckToDrone, // [M3] truck customer deleted from the truck route and served by a new drone trip
    N8DroneToTruck, // [M3] drone trip dissolved and its customer inserted into a gap of the truck route
    N3Exchange,     // [M4] the drone customers of two consecutive trips of the SAME drone are exchanged (launch, recovery, drone ids fixed)
    N1LaunchRelocate,   // [M5] one trip's launch FLP replaced by another FLP (customer, recovery node, drone id fixed)
    N2RecoveryRelocate, // [M5] one trip's recovery node replaced by another truck node (route, launch, customer, drone id fixed)
    N6Swap,         // [M4] the drone customers of two adjacent trips (other drones by default AND, since the A8 sign-off revision, the same drone) are exchanged (launch, recovery, drone ids fixed)
};

inline constexpr std::size_t kNoIndex = std::numeric_limits<std::size_t>::max();

// A fully specified move. Fields not used by a kind keep their defaults.
struct Move {
    MoveKind kind = MoveKind::N4Relocate;
    tdr::model::NodeId customer = tdr::model::kInvalidNodeId; // N3, N6: the anchor trip's drone customer (before the exchange); N4: the moved/anchor customer; N5, N7: the TRUCK customer that becomes drone-served; N8: the DRONE customer that moves onto the truck
    tdr::model::NodeId partner = tdr::model::kInvalidNodeId;  // N3, N6: the partner trip's drone customer (before the exchange); N4Exchange: the other truck customer; N5: the DRONE customer that moves onto the truck
    std::size_t gap = kNoIndex;                               // N4Relocate / N8: skeleton gap the customer is inserted into (N8: gap of the unchanged skeleton); N7: gap of the RESULTING skeleton whose arc holds the launch FLP
    std::size_t replaced_trip = kNoIndex;                     // N5: index (in the input) of the trip that is replaced; N8: index of the dissolved trip; N3, N6: index of the ANCHOR trip
    tdr::model::NodeId replaced_node = tdr::model::kInvalidNodeId; // [M5] N1: the OLD launch FLP; N2: the OLD recovery node
    std::size_t partner_trip = kNoIndex;                      // [M4] N3, N6: index (in the input) of the partner trip
    tdr::model::NodeId launch = tdr::model::kInvalidNodeId;   // N5, N7: launch FLP of the new trip
    tdr::model::NodeId recovery = tdr::model::kInvalidNodeId; // N5, N7: recovery node of the new trip
    int drone_id = -1;                                        // N5, N7: drone flying the new trip
};

// Why a candidate was not accepted. `None` = accepted (evaluator-feasible).
enum class RejectReason : std::size_t {
    None = 0,
    AffectedTrips,       // the route change leaves a trip's launch FLP on a vanished arc, or removes a trip's recovery node (N4 policy Reject, N5, N7, N8)
    NoFreeDrone,         // N5, N7: no drone can fly the new trip without overlapping its other trips (|U| / (28) at construction time)
    NoReplacementFlp,    // N4 re-anchoring: a trip with a stale launch FLP has no FLP on its resulting arc; N7: no FLP on any eligible arc for the customer that must become drone-served
    InvalidRecovery,     // N4 re-anchoring: a trip's recovery node no longer lies after its launch (re-anchoring the launch cannot fix that)
    EvalEndurance,       // evaluator: endurance (36)/(37)
    EvalFleetCapacity,   // evaluator: more than |U| drones simultaneously in flight
    EvalDroneSequencing, // evaluator: constraint (28) (a drone relaunched before it is recovered)
    EvalTripOrder,       // evaluator: a trip's recovery does not follow its launch in truck-route order (root cause; rule (6) on its FLP is only a cascade)
    EvalFlpArc,          // evaluator: FLP not on its own arc  (indicates an operator construction bug -- must stay at 0)
    EvalFlpUnused,       // evaluator: rule (6) idle FLP        (indicates an operator construction bug -- must stay at 0)
    EvalOther,           // any other evaluator violation (timing/sequence/synchronisation...)
};
inline constexpr std::size_t kRejectReasonCount = 12;

inline const char* reject_reason_name(RejectReason r) {
    switch (r) {
        case RejectReason::None: return "accepted";
        case RejectReason::AffectedTrips: return "affected-trips";
        case RejectReason::NoFreeDrone: return "no-free-drone";
        case RejectReason::NoReplacementFlp: return "no-replacement-flp";
        case RejectReason::InvalidRecovery: return "invalid-recovery";
        case RejectReason::EvalEndurance: return "eval-endurance";
        case RejectReason::EvalFleetCapacity: return "eval-fleet-capacity";
        case RejectReason::EvalDroneSequencing: return "eval-drone-sequencing";
        case RejectReason::EvalTripOrder: return "eval-trip-order";
        case RejectReason::EvalFlpArc: return "eval-flp-arc";
        case RejectReason::EvalFlpUnused: return "eval-flp-unused";
        case RejectReason::EvalOther: return "eval-other";
    }
    return "unknown";
}

// [M2] What N4 does with drone trips that its route change disturbs.
//   Reject   -- the original M2 policy: a candidate that leaves any launch FLP off its arc is rejected
//               (AffectedTrips), trips named, never evaluated. Kept selectable; it invents nothing.
//   Reanchor -- [APPROVED 9 Nov 2026, conservative re-anchoring] each trip whose launch FLP became stale
//               gets a replacement FLP on the resulting arc; its drone customer, recovery node and drone id
//               are preserved; unaffected trips are not touched; a trip whose recovery no longer lies after
//               its launch is NOT repaired (InvalidRecovery); everything is then judged by the evaluator.
// N5 has its own fixed behaviour (reject) and is not governed by this enum.
enum class AffectedTripPolicy { Reject, Reanchor };

struct OperatorOptions {
    bool record_candidates = false;   // keep every generated candidate (solution + verdict) -- tests/diagnostics; off in search
    bool evaluate_candidates = true;  // false: stop after construction and the structural checks (benchmarking generation cost); no candidate is then accepted
};

// One launch FLP replaced for one trip (N4 re-anchoring).
struct Reanchor {
    std::size_t trip = kNoIndex;                                   // index into the INPUT solution's trips
    tdr::model::NodeId from = tdr::model::kInvalidNodeId;          // the stale launch FLP
    tdr::model::NodeId to = tdr::model::kInvalidNodeId;            // the replacement, on the resulting arc
};

// One generated candidate (kept only when OperatorOptions::record_candidates).
struct CandidateRecord {
    Move move;
    RejectReason reason = RejectReason::None;
    bool evaluated = false;
    double objective = 0.0;                         // valid iff evaluated
    std::vector<std::size_t> affected_trips;        // indices into the INPUT solution's trips (AffectedTrips / InvalidRecovery / NoReplacementFlp)
    std::vector<Reanchor> reanchored;               // N4 Reanchor: the replacements of the shown solution (the best variant, or the first variant if none was feasible)
    std::size_t variants_generated = 0;             // N4 Reanchor: replacement combinations constructed for this candidate
    std::size_t variants_evaluated = 0;             // ... of which sent to the evaluator
    std::size_t combinations_total = 0;             // N4 Reanchor: all joint replacement combinations of this candidate (saturating); variants_generated <= min(this, cap)
    bool cap_reached = false;                       // N4 Reanchor: the search budget (N4Options::max_reanchor_variants) stopped the enumeration before all combinations were reached
    std::vector<tdr::model::NodeId> flps_dropped;   // FLPs removed because their only trip was replaced or dissolved (N5, N8)
    tdr::model::Solution solution;                  // the candidate (empty for pre-construction rejections)
};

struct Candidate {
    tdr::model::Solution solution;
    double objective = 0.0;
    Move move;
    std::vector<tdr::model::NodeId> flps_dropped;
    std::vector<Reanchor> reanchored;  // N4 Reanchor: the replacements applied in this candidate (empty if none were needed)
};

enum class OperatorStatus {
    Ok,
    InputInfeasible,  // the input solution is not evaluator-feasible: nothing is generated
    InvalidAnchor,    // the given customer / trip index is not a valid anchor for this operator
    NoEligibleAnchor, // random selection found nothing to anchor on
};

inline const char* operator_status_name(OperatorStatus s) {
    switch (s) {
        case OperatorStatus::Ok: return "ok";
        case OperatorStatus::InputInfeasible: return "input-infeasible";
        case OperatorStatus::InvalidAnchor: return "invalid-anchor";
        case OperatorStatus::NoEligibleAnchor: return "no-eligible-anchor";
    }
    return "unknown";
}

struct OperatorResult {
    OperatorId op = OperatorId::N4;
    OperatorStatus status = OperatorStatus::Ok;
    double base_objective = 0.0;     // evaluator objective of the input (valid unless status == InputInfeasible)
    tdr::model::NodeId anchor_customer = tdr::model::kInvalidNodeId; // N4, N7: the truck customer; N3, N5, N6, N8: the drone customer of the chosen trip
    std::size_t anchor_trip = kNoIndex;                               // N3, N5, N6, N8: index of the chosen trip

    // "generated" and "accepted" count CANDIDATES (N4: distinct routes/moves; N5: swap or new-trip candidates).
    // "evaluated" counts EVALUATOR CALLS: for an N4 candidate that needed re-anchoring it is the number of replacement
    // combinations tried, so it can exceed "generated". Each candidate is rejected for exactly one reason.
    std::size_t generated = 0;       // fully specified candidates produced (after duplicate removal)
    std::size_t evaluated = 0;       // evaluator calls
    std::size_t accepted = 0;        // candidates with an evaluator-feasible outcome
    std::size_t duplicates_skipped = 0;
    std::array<std::size_t, kRejectReasonCount> rejected{}; // by RejectReason (index 0 unused), one per rejected candidate

    // N4 Reanchor diagnostics (all zero for N5 and for policy Reject).
    std::size_t stale_launch_candidates = 0;  // candidates in which at least one launch FLP became stale (needed re-anchoring)
    std::size_t reanchored_candidates = 0;    // ... of which accepted after re-anchoring
    std::size_t trips_reanchored = 0;         // trips re-anchored, summed over those accepted candidates
    std::size_t variants_generated = 0;       // replacement combinations constructed (all stale candidates that reached enumeration)
    std::size_t cap_reached_candidates = 0;   // candidates whose enumeration was cut short by the budget (an orthogonal flag, NOT an outcome: such a candidate is still accepted or rejected exactly once);
                                              // while it is 0 the Reanchor search was exhaustive, otherwise it may have missed a better combination

    std::optional<Candidate> best;   // lowest objective among accepted; first generated wins ties (strict <)
    std::vector<CandidateRecord> log; // all candidates, iff OperatorOptions::record_candidates

    bool has_candidate() const { return best.has_value(); }
    // Strict comparison on the raw objective: no tolerance (see DECISIONS.md, "P2" note for M7).
    bool improves() const { return best && best->objective < base_objective; }
    std::size_t rejected_total() const {
        std::size_t n = 0;
        for (std::size_t i = 1; i < kRejectReasonCount; ++i) n += rejected[i];
        return n;
    }
};

// Verdict of the evaluator on one candidate, classified from its STRUCTURED
// fields where possible. Never overrides the evaluator.
struct EvalVerdict {
    bool feasible = false;
    double objective = 0.0;
    RejectReason reason = RejectReason::None;
};

inline EvalVerdict classify_evaluation(const tdr::timing::SolutionEvaluationResult& e, int num_drones) {
    EvalVerdict v;
    v.objective = e.objective_value;
    if (e.feasible) {
        v.feasible = true;
        return v;
    }
    for (const auto& t : e.drone_trip_timings) {
        if (t.structurally_valid && t.on_route && t.has_customer_data && t.drone_id_valid && !t.endurance_feasible) {
            v.reason = RejectReason::EvalEndurance;
            return v;
        }
    }
    const auto has = [&](const char* needle) {
        for (const auto& r : e.violation_reasons) if (r.find(needle) != std::string::npos) return true;
        return false;
    };
    if (e.max_simultaneous_drones > num_drones) { v.reason = RejectReason::EvalFleetCapacity; return v; }
    if (has("constraint (28)")) { v.reason = RejectReason::EvalDroneSequencing; return v; }
    // Checked BEFORE rule (6): a trip whose recovery precedes its launch also leaves its launch FLP "unused" (cascade).
    if (has("not reachable in truck-route order")) { v.reason = RejectReason::EvalTripOrder; return v; }
    if (has("belongs to L_(") || has("has no arc-ownership data")) { v.reason = RejectReason::EvalFlpArc; return v; }
    if (has("(constraint (6))")) { v.reason = RejectReason::EvalFlpUnused; return v; }
    v.reason = RejectReason::EvalOther;
    return v;
}

} // namespace tdr::search
