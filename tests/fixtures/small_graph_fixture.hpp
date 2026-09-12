#pragma once

// A small, deterministic, hand-checkable node layout for tests. Not part
// of the shipped library (it lives under tests/, not include/) and not
// an instance-generation mechanism -- it exists purely so this phase's
// tests, and later phases' tests, can share one fixed set of nodes with
// exactly known pairwise distances instead of each re-deriving their own
// coordinates.
//
// Layout (all distances are exact integers, built from 3-4-5 right
// triangles so nothing needs sqrt() to verify by hand):
//
//   id 0: Depot,    (0, 0)
//   id 1: Customer, (3, 4)     -- distance to depot = 5
//   id 2: Customer, (6, 8)     -- distance to depot = 10, to id1 = 5
//   id 3: Customer, (-3, 4)    -- distance to depot = 5, to id1 = 6
//   id 4: FlexibleLaunchPoint, (0, 4) -- distance to depot = 4, to id1 = 3

#include "model/node.hpp"

namespace tdr::test_fixtures {

inline tdr::model::NodeRegistry build_small_graph_fixture() {
    using tdr::model::Node;
    using tdr::model::NodeType;
    using tdr::model::Point2D;

    tdr::model::NodeRegistry nodes;
    nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
    nodes.add(Node{1, NodeType::Customer, Point2D{3.0, 4.0}});
    nodes.add(Node{2, NodeType::Customer, Point2D{6.0, 8.0}});
    nodes.add(Node{3, NodeType::Customer, Point2D{-3.0, 4.0}});
    nodes.add(Node{4, NodeType::FlexibleLaunchPoint, Point2D{0.0, 4.0}});
    return nodes;
}

} // namespace tdr::test_fixtures
