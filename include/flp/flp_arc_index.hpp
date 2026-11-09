#pragma once

// Arc -> flexible-launch-point lookup over the EAGER static pool (M1,
// 9 Nov 2026).
//
// The static reproduction keeps the pool exactly as Phase D/F produce it:
// L_(a,b) is generated once for every ordered pair of truck nodes, with the
// ids assigned then. This class adds no generation and no ids; it only
// answers "which FLPs belong to arc (a, b), nearest-to-a first?" and "which
// arc does this FLP belong to?" in O(1), which the neighbourhoods need
// after every route change (a route change alters which directed arcs the
// truck traverses, hence which FLPs are launchable). No lazy cache and no
// capacity-based id formula: those were for the (paused) dynamic work.
//
// Built from the arc-ownership map that evaluate_solution() already takes
// (FlpDataMap), so the index and the evaluator can never disagree about
// ownership.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "flp/flexible_launch_point_generator.hpp"
#include "model/flexible_launch_point.hpp"
#include "model/node.hpp"

namespace tdr::flp {

class FlpArcIndex {
public:
    FlpArcIndex() = default;

    explicit FlpArcIndex(const std::unordered_map<tdr::model::NodeId, tdr::model::FlexibleLaunchPointData>& flp_data) {
        struct Entry { tdr::model::NodeId flp; double ratio; };
        std::unordered_map<std::uint64_t, std::vector<Entry>> tmp;
        for (const auto& kv : flp_data) {
            const auto& d = kv.second;
            tmp[key(d.arc_from, d.arc_to)].push_back(Entry{kv.first, d.position_ratio});
            owner_[kv.first] = Arc{d.arc_from, d.arc_to};
        }
        for (auto& kv : tmp) {
            // Deterministic: ascending ratio (nearest to the arc start first),
            // ties by id. The input map's iteration order is unspecified.
            std::sort(kv.second.begin(), kv.second.end(), [](const Entry& x, const Entry& y) {
                if (x.ratio != y.ratio) return x.ratio < y.ratio;
                return x.flp < y.flp;
            });
            auto& out = by_arc_[kv.first];
            out.reserve(kv.second.size());
            for (const Entry& e : kv.second) out.push_back(e.flp);
        }
    }

    // L_(a,b), ordered by position along the arc from a; empty if none.
    const std::vector<tdr::model::NodeId>& flps_on_arc(tdr::model::NodeId a, tdr::model::NodeId b) const {
        const auto it = by_arc_.find(key(a, b));
        return it == by_arc_.end() ? empty_ : it->second;
    }

    std::optional<Arc> arc_of(tdr::model::NodeId flp) const {
        const auto it = owner_.find(flp);
        if (it == owner_.end()) return std::nullopt;
        return it->second;
    }

    bool contains(tdr::model::NodeId flp) const { return owner_.find(flp) != owner_.end(); }
    std::size_t flp_count() const { return owner_.size(); }
    std::size_t nonempty_arc_count() const { return by_arc_.size(); }

private:
    static std::uint64_t key(tdr::model::NodeId a, tdr::model::NodeId b) {
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32) |
               static_cast<std::uint64_t>(static_cast<std::uint32_t>(b));
    }
    std::unordered_map<std::uint64_t, std::vector<tdr::model::NodeId>> by_arc_;
    std::unordered_map<tdr::model::NodeId, Arc> owner_;
    std::vector<tdr::model::NodeId> empty_;
};

} // namespace tdr::flp
