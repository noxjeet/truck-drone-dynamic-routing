#pragma once

// The shared "adjacent" candidate rule for N1-N8 (DECISION D1, M1,
// 9 Nov 2026).
//
// [PAPER] Sec 5.3 describes every neighbourhood in terms of "adjacent"
// nodes/customers/trips but never defines adjacency or a radius. This file
// supplies ONE definition used by all eight neighbourhoods, so a single
// configurable number (NeighbourhoodConfig::adjacency_window = w, default
// 3) controls them all.
// [ASSUMPTION] Definition. Place every customer on a "half-step" line over
// the route's skeleton (non-FLP nodes, route_ops.hpp):
//   - a truck-served customer at skeleton index s has h = 2 s;
//   - a drone-served customer, whose trip launches from an FLP lying in gap
//     g (between skeleton nodes g and g+1), has h = 2 g + 1, i.e. midway
//     between the two truck nodes it was launched between;
//   - a gap g (an insertion point for a truck node) has h = 2 g + 1.
// Two entities are ADJACENT iff |h1 - h2| <= 2 w. For truck customers this
// is "within w truck-nodes of each other"; a drone customer is adjacent to
// the truck nodes within w of its launch gap.
// Why skeleton and not raw route positions: raw positions count FLP entries,
// so an FLP-dense route would shrink the neighbourhood to one or two
// customers for the same w. The skeleton is independent of how many FLPs
// the current solution happens to use.
//
// This is a candidate PRE-FILTER only. It never decides feasibility.

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "model/node.hpp"
#include "model/solution.hpp"
#include "search_pre_hook/route_ops.hpp"

namespace tdr::search_ref {

// Gaps g of `sk` (insertion points between skeleton g and g+1) whose half-step
// coordinate 2g+1 lies within `window` truck nodes of half-step coordinate h,
// ascending: |2g + 1 - h| <= 2 * window. Same rule as PositionIndex::gaps_within,
// usable on any skeleton (e.g. one built from an intermediate route). [M2]
inline std::vector<std::size_t> gaps_within_window(const Skeleton& sk, int h, int window) {
    std::vector<std::size_t> out;
    for (std::size_t g = 0; g < sk.gap_count(); ++g) {
        if (std::abs(2 * static_cast<int>(g) + 1 - h) <= 2 * window) out.push_back(g);
    }
    return out;
}

class PositionIndex {
public:
    PositionIndex(const tdr::model::Solution& s, const tdr::model::NodeRegistry& nodes)
        : skeleton_(make_skeleton(s.truck_route, nodes)) {
        for (std::size_t k = 0; k < skeleton_.size(); ++k) {
            const tdr::model::NodeId v = skeleton_.node[k];
            if (nodes.contains(v) && nodes.at(v).type == tdr::model::NodeType::Customer) half_[v] = 2 * static_cast<int>(k);
        }
        for (const auto& t : s.drone_trips) {
            const auto lp = tdr::timing::detail::find_route_position(s.truck_route, t.launch_node);
            if (!lp) continue;
            const auto g = gap_of_route_position(skeleton_, *lp);
            if (!g) continue;
            half_.emplace(t.service_customer, 2 * static_cast<int>(*g) + 1);
        }
    }

    const Skeleton& skeleton() const { return skeleton_; }

    // Half-step coordinate of a customer; nullopt if it is nowhere on the
    // route (e.g. a drone customer whose launch FLP is missing).
    std::optional<int> half_step(tdr::model::NodeId customer) const {
        const auto it = half_.find(customer);
        if (it == half_.end()) return std::nullopt;
        return it->second;
    }

    bool adjacent(tdr::model::NodeId x, tdr::model::NodeId y, int window) const {
        const auto hx = half_step(x), hy = half_step(y);
        return hx && hy && std::abs(*hx - *hy) <= 2 * window;
    }

    // Customers adjacent to `customer` (excluding itself), nearest first;
    // ties broken by ascending id. Deterministic.
    std::vector<tdr::model::NodeId> neighbours(tdr::model::NodeId customer, int window) const {
        std::vector<std::pair<int, tdr::model::NodeId>> found;
        const auto hc = half_step(customer);
        if (!hc) return {};
        for (const auto& kv : half_) {
            if (kv.first == customer) continue;
            const int d = std::abs(kv.second - *hc);
            if (d <= 2 * window) found.emplace_back(d, kv.first);
        }
        std::sort(found.begin(), found.end());
        std::vector<tdr::model::NodeId> out;
        out.reserve(found.size());
        for (const auto& f : found) out.push_back(f.second);
        return out;
    }

    // Gaps g (insertion points between skeleton g and g+1) within the window
    // of half-step coordinate h, ascending.
    std::vector<std::size_t> gaps_within(int h, int window) const {
        std::vector<std::size_t> out;
        for (std::size_t g = 0; g < skeleton_.gap_count(); ++g) {
            if (std::abs(2 * static_cast<int>(g) + 1 - h) <= 2 * window) out.push_back(g);
        }
        return out;
    }

private:
    Skeleton skeleton_;
    std::unordered_map<tdr::model::NodeId, int> half_;
};

} // namespace tdr::search_ref
