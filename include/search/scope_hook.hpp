#pragma once

// [DM8] Optional scope hook for the static neighbourhood operators (additive; absent hook = the original code path, unchanged).
//
// A ScopeHook is a pure predicate on ONE fully constructed candidate: "may this candidate be sent to the evaluator?".
// The operators call it immediately BEFORE tdr::timing::evaluate_solution (after every structural rejection they already
// make), so a scope-rejected candidate costs no evaluator call. The hook never repairs and never edits a candidate; it
// only names the first violated rule. It is NOT the feasibility authority (the evaluator still is) and it is NOT the DM7
// region validator (tdr::dynamic::validate_region / check_region_preserved stay independent and unchanged).
//
// The hook is reached through SearchContext::scope (nullptr = unscoped = static behaviour, bit for bit).

#include <cstddef>

#include "model/solution.hpp"

namespace tdr::search {

struct Move;

enum class ScopeViolation : std::size_t {
    None = 0,
    MoveAnchorNotFree,             // the move's anchor / partner customer is not free (protected)
    ProtectedTruckCustomerChanged, // a protected truck customer left the truck route, was duplicated, or became drone-served
    ProtectedDroneCustomerChanged, // a protected drone customer lost its trip, or its trip's drone / launch / recovery changed, or it moved to the truck
    ProtectedTripDependencyBroken, // a protected trip's launch or recovery node is missing from the route, or recovery no longer follows launch
    PinnedNodeMissing,             // a pinned node (identity) is absent from the candidate's truck route
    PinnedOrderChanged,            // pinned nodes no longer appear in their original relative order
    ProtectedPrefixChanged,        // the protected prefix (truck route through boundary lo, as node identities) is not the candidate's route prefix
    ProtectedSuffixChanged,        // the protected suffix (boundary hi and everything behind it) is not the candidate's route suffix
    FreeTripOutsideRegion,         // a trip serving a free customer launches outside, or is recovered outside, the span between prefix and suffix
};
inline constexpr std::size_t kScopeViolationCount = 10; // number of enumerators INCLUDING None (array size of the per-violation counters)
static_assert(kScopeViolationCount == static_cast<std::size_t>(ScopeViolation::FreeTripOutsideRegion) + 1, "kScopeViolationCount must cover every ScopeViolation");

inline const char* scope_violation_name(ScopeViolation v) {
    switch (v) {
        case ScopeViolation::None: return "none";
        case ScopeViolation::MoveAnchorNotFree: return "move-anchor-not-free";
        case ScopeViolation::ProtectedTruckCustomerChanged: return "protected-truck-customer-changed";
        case ScopeViolation::ProtectedDroneCustomerChanged: return "protected-drone-customer-changed";
        case ScopeViolation::ProtectedTripDependencyBroken: return "protected-trip-dependency-broken";
        case ScopeViolation::PinnedNodeMissing: return "pinned-node-missing";
        case ScopeViolation::PinnedOrderChanged: return "pinned-order-changed";
        case ScopeViolation::ProtectedPrefixChanged: return "protected-prefix-changed";
        case ScopeViolation::ProtectedSuffixChanged: return "protected-suffix-changed";
        case ScopeViolation::FreeTripOutsideRegion: return "free-trip-outside-region";
    }
    return "unknown";
}

class ScopeHook {
public:
    virtual ~ScopeHook() = default;
    // Pure and deterministic: the verdict depends only on (candidate, move) and the hook's own immutable snapshot.
    virtual ScopeViolation check(const tdr::model::Solution& candidate, const Move& move) const = 0;
};

} // namespace tdr::search
