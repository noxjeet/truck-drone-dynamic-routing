#pragma once

// [DM8] ScopeMask: an identity-based snapshot of what a regional local search may NOT change, implementing tdr::search::ScopeHook.
//
// Built once from (Region, candidate plan S+). It stores node/customer/trip IDENTITIES only (no skeleton indices, no route positions),
// so it stays valid while an iterated search replaces the base plan, as long as the protected identities are respected.
//   free customers      Region::free_customers MINUS the pinned nodes (a pinned frozen-recovery customer is never free; DM7 lists it as free, left unchanged there)
//   protected truck     every truck-served customer of S+ that is not free      -> must stay on the truck route (exactly once), never become drone-served
//   protected trips     every drone trip of S+ whose customer is not free        -> must exist unchanged (drone id, launch FLP, recovery node); launch must precede recovery
//   pinned nodes        Region::pinned_nodes (boundary lo, hi, frozen recoveries) -> must stay in the truck route in their original relative order
//   protected prefix    the plan's route nodes 0..lo (as a node-id sequence) -> must be the candidate route's prefix, verbatim
//   protected suffix    the plan's route nodes hi..end (node-id sequence)     -> must be the candidate route's suffix, verbatim
//   free trips          a trip serving a free customer must launch strictly between prefix and suffix and be recovered after its launch, at or before the suffix start
// Two rules, kept apart. Rule 1: pinned nodes keep identity, trip assignment and relative order among themselves; FREE customers may be placed before/between/
// after the INTERIOR pinned nodes (frozen recoveries). Rule 2: the boundary itself is protected: nothing may enter or leave the prefix/suffix, so a free customer
// that would cross lo or hi, or a trip that would launch/recover outside the span, is rejected BEFORE the evaluator. All of this is identity (node-id sequences),
// no skeleton index or route position is stored. DM7's validate_region / check_region_preserved stay independent post-acceptance checks.
//
// R0 (empty or invalid region): no free customers, therefore no anchors and every move is rejected (MoveAnchorNotFree).
// full(): everything free, no pinned nodes: check() is always None (the hook is then a pure no-op).

#include <algorithm>
#include <cstddef>
#include <unordered_set>
#include <vector>

#include "dynamic/region_builder.hpp"
#include "search/move_types.hpp"
#include "search/scope_hook.hpp"

namespace tdr::dynamic {

class ScopeMask final : public tdr::search::ScopeHook {
public:
    using NodeId = tdr::model::NodeId;
    struct TripKey { int drone = -1; NodeId launch = tdr::model::kInvalidNodeId, customer = tdr::model::kInvalidNodeId, recovery = tdr::model::kInvalidNodeId; };

    static ScopeMask full() { ScopeMask m; m.unrestricted_ = true; return m; }

    // `plan` is the plan the region was built on (S+). An invalid or empty region gives the empty mask.
    static ScopeMask from_region(const Region& R, const tdr::model::Solution& plan, const tdr::model::NodeRegistry& nodes) {
        ScopeMask m;
        const auto sk = tdr::search::make_skeleton(plan.truck_route, nodes);
        if (!R.valid || R.empty || R.hi >= sk.size() || R.lo >= R.hi) {
            m.empty_region_ = true;
            m.prefix_ = plan.truck_route;    // nothing may change: the whole route is protected prefix
            m.fill_protected(plan, nodes);
            return m;
        }
        const std::size_t plo = sk.route_position[R.lo], phi = sk.route_position[R.hi];
        m.prefix_.assign(plan.truck_route.begin(), plan.truck_route.begin() + static_cast<std::ptrdiff_t>(plo) + 1);
        m.suffix_.assign(plan.truck_route.begin() + static_cast<std::ptrdiff_t>(phi), plan.truck_route.end());
        const std::unordered_set<NodeId> pinned(R.pinned_nodes.begin(), R.pinned_nodes.end());
        for (NodeId c : R.free_customers) if (pinned.count(c) == 0) m.free_.push_back(c);
        std::sort(m.free_.begin(), m.free_.end());
        m.free_.erase(std::unique(m.free_.begin(), m.free_.end()), m.free_.end());
        m.pinned_set_ = pinned;
        for (NodeId v : plan.truck_route) if (pinned.count(v) != 0) m.pinned_order_.push_back(v);
        m.fill_protected(plan, nodes);
        return m;
    }

    bool unrestricted() const { return unrestricted_; }
    bool empty_region() const { return empty_region_; }
    bool is_free(NodeId c) const { return unrestricted_ || std::binary_search(free_.begin(), free_.end(), c); }
    const std::vector<NodeId>& free_customers() const { return free_; }
    const std::vector<NodeId>& pinned_order() const { return pinned_order_; }
    const std::vector<NodeId>& protected_prefix() const { return prefix_; }
    const std::vector<NodeId>& protected_suffix() const { return suffix_; }
    const std::vector<NodeId>& protected_truck_customers() const { return protected_truck_; }
    const std::vector<TripKey>& protected_trips() const { return protected_trips_; }

    // ---- anchor helpers: the ONLY customers/trips an operator may be anchored on (explicit-anchor overloads) ----
    std::vector<NodeId> free_truck_customers(const tdr::model::Solution& s, const tdr::model::NodeRegistry& nodes) const {
        std::vector<NodeId> out;
        for (NodeId v : s.truck_route) if (is_customer(nodes, v) && is_free(v)) out.push_back(v);
        return out;
    }
    std::vector<std::size_t> free_trip_indices(const tdr::model::Solution& s) const {
        std::vector<std::size_t> out;
        for (std::size_t i = 0; i < s.drone_trips.size(); ++i) if (is_free(s.drone_trips[i].service_customer)) out.push_back(i);
        return out;
    }

    tdr::search::ScopeViolation check(const tdr::model::Solution& cand, const tdr::search::Move& mv) const override {
        using V = tdr::search::ScopeViolation;
        if (unrestricted_) return V::None;
        // 1. the move's anchor and partner must be free
        if (mv.customer != tdr::model::kInvalidNodeId && !is_free(mv.customer)) return V::MoveAnchorNotFree;
        if (mv.partner != tdr::model::kInvalidNodeId && !is_free(mv.partner)) return V::MoveAnchorNotFree;
        const auto& route = cand.truck_route;
        // 2. protected truck customers: still on the route exactly once, not drone-served
        if (!protected_truck_.empty() || !protected_trips_.empty()) {
            std::vector<NodeId> hit;
            for (NodeId v : route) {
                if (std::binary_search(protected_truck_.begin(), protected_truck_.end(), v)) hit.push_back(v);
                else if (std::binary_search(protected_drone_.begin(), protected_drone_.end(), v)) return V::ProtectedDroneCustomerChanged; // a protected drone customer put on the truck route
            }
            std::sort(hit.begin(), hit.end());
            if (hit != protected_truck_) return V::ProtectedTruckCustomerChanged; // missing or duplicated
            for (const auto& t : cand.drone_trips)
                if (std::binary_search(protected_truck_.begin(), protected_truck_.end(), t.service_customer)) return V::ProtectedTruckCustomerChanged;
        }
        // 3. protected drone customers: exactly one trip each, unchanged
        for (const auto& k : protected_trips_) {
            std::size_t n = 0; const tdr::model::DroneTrip* found = nullptr;
            for (const auto& t : cand.drone_trips) if (t.service_customer == k.customer) { ++n; found = &t; }
            if (n != 1 || found->drone_id != k.drone || found->launch_node != k.launch || found->recovery_node != k.recovery) return V::ProtectedDroneCustomerChanged;
            // 4. dependency: launch present, recovery after it
            const auto lp = std::find(route.begin(), route.end(), k.launch);
            if (lp == route.end() || std::find(lp + 1, route.end(), k.recovery) == route.end()) return V::ProtectedTripDependencyBroken;
        }
        // 5/6. pinned nodes present, in their original relative order
        if (!pinned_order_.empty()) {
            std::vector<NodeId> seq;
            for (NodeId v : route) if (pinned_set_.count(v) != 0) seq.push_back(v);
            if (seq != pinned_order_) {
                std::vector<NodeId> a = seq, b = pinned_order_;
                std::sort(a.begin(), a.end()); std::sort(b.begin(), b.end());
                return a != b ? V::PinnedNodeMissing : V::PinnedOrderChanged;
            }
        }
        // 7/8. protected prefix and suffix: node-identity sequences, verbatim at the two ends of the candidate route
        if (route.size() < prefix_.size() + suffix_.size() || !std::equal(prefix_.begin(), prefix_.end(), route.begin())) return V::ProtectedPrefixChanged;
        if (!std::equal(suffix_.begin(), suffix_.end(), route.end() - static_cast<std::ptrdiff_t>(suffix_.size()))) return V::ProtectedSuffixChanged;
        // 9. a trip serving a free customer lives inside the span: launch in [prefix, n - suffix), recovery after it and at most the first suffix node
        const std::size_t P = prefix_.size(), end = route.size() - suffix_.size();
        for (const auto& t : cand.drone_trips) {
            if (!is_free(t.service_customer)) continue;
            const auto lp = std::find(route.begin(), route.end(), t.launch_node);
            const std::size_t li = static_cast<std::size_t>(lp - route.begin());
            if (lp == route.end() || li < P || li >= end) return V::FreeTripOutsideRegion;
            const auto rp = std::find(lp + 1, route.end(), t.recovery_node);
            if (rp == route.end() || static_cast<std::size_t>(rp - route.begin()) > end) return V::FreeTripOutsideRegion;
        }
        return V::None;
    }

private:
    static bool is_customer(const tdr::model::NodeRegistry& nodes, NodeId v) { return nodes.contains(v) && nodes.at(v).type == tdr::model::NodeType::Customer; }

    void fill_protected(const tdr::model::Solution& plan, const tdr::model::NodeRegistry& nodes) {
        for (NodeId v : plan.truck_route) if (is_customer(nodes, v) && !is_free(v)) protected_truck_.push_back(v);
        for (const auto& t : plan.drone_trips) if (!is_free(t.service_customer)) {
            protected_trips_.push_back(TripKey{t.drone_id, t.launch_node, t.service_customer, t.recovery_node});
            protected_drone_.push_back(t.service_customer);
        }
        std::sort(protected_truck_.begin(), protected_truck_.end());
        std::sort(protected_drone_.begin(), protected_drone_.end());
    }

    bool unrestricted_ = false, empty_region_ = false;
    std::vector<NodeId> free_;                 // sorted
    std::vector<NodeId> protected_truck_;      // sorted; one entry per occurrence in the plan's route
    std::vector<NodeId> protected_drone_;      // sorted
    std::vector<TripKey> protected_trips_;
    std::vector<NodeId> pinned_order_;         // base-route order, one entry per occurrence
    std::vector<NodeId> prefix_, suffix_;      // protected route prefix (through lo) / suffix (from hi); full mask: both empty
    std::unordered_set<NodeId> pinned_set_;
};

// Aggregates the scope counters of OperatorResults (for DM9 diagnostics and the DM8 tests).
struct ScopeStats {
    std::size_t scope_rejected = 0, scope_variants_skipped = 0, evaluator_calls_avoided = 0;
    std::array<std::size_t, tdr::search::kScopeViolationCount> by_violation{};
    void add(const tdr::search::OperatorResult& r) {
        scope_rejected += r.scope_rejected; scope_variants_skipped += r.scope_variants_skipped; evaluator_calls_avoided += r.evaluator_calls_avoided;
        for (std::size_t i = 0; i < by_violation.size(); ++i) by_violation[i] += r.scope_by_violation[i];
    }
};

} // namespace tdr::dynamic
