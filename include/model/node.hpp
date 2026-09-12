#pragma once

#include <unordered_map>

namespace tdr::model {

// A node identifier. All node types (depot, customer, flexible launch
// point) share one identifier space, matching Paper 1's graph G=(V,A)
// where V is a single vertex set (Sec 2.1). It is a plain integer, not a
// stronger type, to stay consistent with the existing tsp/ module, whose
// DistanceMatrix is also indexed by plain int.
using NodeId = int;

inline constexpr NodeId kInvalidNodeId = -1;

// [PAPER] Paper 1 distinguishes exactly these three kinds of location in
// its graph: the depot (Sec 2.1, D = {0, c+1}), customers (C), and
// flexible launch points (L, partitioned per-arc as L_ij). Note: the
// paper's depot is really two *roles* at one physical location -- a
// start node (0) and an end node (c+1) -- because the truck route is a
// path, not a cycle, in its own formulation. This Phase-B model does not
// introduce a fourth NodeType for that: a start and an end depot are
// simply represented as two separate Node entries that both carry
// NodeType::Depot and (normally) the same coordinates. Which id plays
// which role is left to whatever module builds a concrete instance
// (Phase C/D), not to this type.
enum class NodeType {
    Depot,
    Customer,
    FlexibleLaunchPoint,
};

// [PAPER] Sec 2.1: nodes are located in the Euclidean plane (60x60 km
// square for the synthetic instances). Plain x/y, no units baked in here
// -- units are an instance-generation concern (Phase D).
struct Point2D {
    double x = 0.0;
    double y = 0.0;
};

// A single node in the problem graph. This is intentionally the same
// shape for every NodeType -- depot, customer, and flexible launch point
// all have an id, a type, and coordinates. Type-specific extra data
// (customer service times, a flex point's arc/offset) lives in separate
// structs (customer.hpp, flexible_launch_point.hpp) that reference a
// Node by its id, rather than being folded into Node itself. That keeps
// Node minimal and reusable everywhere a plain "which node, what kind,
// where" answer is needed (e.g. Phase C's distance calculations), while
// keeping type-specific fields out of the common case.
struct Node {
    NodeId id = kInvalidNodeId;
    NodeType type = NodeType::Depot;
    Point2D coordinates;
};

// A minimal id -> Node lookup table. This is not an "Instance" (Phase
// C/D's job): it holds no distances, no generation logic, nothing about
// how the nodes came to exist -- only identity, type, and coordinates,
// so that later modules can look a node up by id and ask "what kind is
// this, and where is it." Rejects duplicate ids rather than silently
// overwriting, since a repeated node id would be an internal
// inconsistency, not a legitimate model state.
class NodeRegistry {
public:
    // Returns false (and leaves the registry unchanged) if a node with
    // this id is already registered.
    bool add(const Node& node) {
        return nodes_.emplace(node.id, node).second;
    }

    bool contains(NodeId id) const {
        return nodes_.find(id) != nodes_.end();
    }

    const Node& at(NodeId id) const {
        return nodes_.at(id);
    }

    std::size_t size() const {
        return nodes_.size();
    }

    const std::unordered_map<NodeId, Node>& all() const {
        return nodes_;
    }

private:
    std::unordered_map<NodeId, Node> nodes_;
};

} // namespace tdr::model
