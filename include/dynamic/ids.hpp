#pragma once

// Dynamic extension (DM3): content-addressed ids (DM0 §10.2, decision D8). Pure functions; no state, no counters.
//   customer id of request r      = kC0 + r              (r = 1-based scenario request index; independent of acceptance)
//   FLP id on a dynamic-endpoint arc = kF0 + ((iota(a)*K + iota(b))*64 + (m-1)),  m = 1..64
// Base-pool (static) ids are far below kC0 and keep their static values.

#include <cstdint>
#include <optional>

#include "model/node.hpp"

namespace tdr::dynamic {

using tdr::model::NodeId;

inline constexpr NodeId kC0 = 1 << 24;   // first dynamic customer id
inline constexpr NodeId kF0 = 1 << 27;   // first dynamic-arc FLP id (kC0 + r stays < kF0 for r < 2^27 - 2^24)
inline constexpr int kMaxFlpPerArc = 64; // Q-F validator rule [PROPOSED]

inline NodeId customer_id(std::uint32_t request_index) { return kC0 + static_cast<NodeId>(request_index); }
inline bool is_dynamic_customer_id(NodeId id) { return id > kC0 && id < kF0; }
inline std::uint32_t request_index_of(NodeId id) { return static_cast<std::uint32_t>(id - kC0); }

// Dense index: depot 0, base customer i -> i, request r -> n0 + r.  K = n0 + M_max + 1 must be <= 1024.
struct DenseIndex {
    int n0 = 0;
    int m_max = 0;
    int K() const { return n0 + m_max + 1; }
    bool valid() const { return n0 >= 0 && m_max >= 0 && K() <= 1024; }
    int of(NodeId id) const { return id == 0 ? 0 : (id >= kC0 ? n0 + static_cast<int>(request_index_of(id)) : id); }
    NodeId node_of(int iota) const { return iota == 0 ? 0 : (iota <= n0 ? iota : customer_id(static_cast<std::uint32_t>(iota - n0))); }
};

struct FlpCode { int iota_a, iota_b, m; };

// nullopt when the arguments are out of range (a validator should have prevented this; never silently wrap).
inline std::optional<NodeId> flp_id(const DenseIndex& d, int iota_a, int iota_b, int m) {
    if (!d.valid() || iota_a < 0 || iota_b < 0 || iota_a >= d.K() || iota_b >= d.K() || iota_a == iota_b || m < 1 || m > kMaxFlpPerArc) return std::nullopt;
    const std::int64_t id = static_cast<std::int64_t>(kF0) + (static_cast<std::int64_t>(iota_a) * d.K() + iota_b) * kMaxFlpPerArc + (m - 1);
    if (id >= (std::int64_t{1} << 31)) return std::nullopt;
    return static_cast<NodeId>(id);
}

inline std::optional<FlpCode> decode_flp_id(const DenseIndex& d, NodeId id) {
    if (!d.valid() || id < kF0) return std::nullopt;
    const std::int64_t v = static_cast<std::int64_t>(id) - kF0;
    const std::int64_t arc = v / kMaxFlpPerArc;
    const int m = static_cast<int>(v % kMaxFlpPerArc) + 1;
    const int a = static_cast<int>(arc / d.K()), b = static_cast<int>(arc % d.K());
    if (a >= d.K() || a == b) return std::nullopt;
    return FlpCode{a, b, m};
}

} // namespace tdr::dynamic
