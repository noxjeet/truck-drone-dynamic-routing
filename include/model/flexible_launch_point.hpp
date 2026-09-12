#pragma once

#include "model/node.hpp"

namespace tdr::model {

// Flexible-launch-point-specific data, keyed back to a Node of type
// NodeType::FlexibleLaunchPoint via node_id.
//
// [PAPER] Sec 2.1/4.1: flexible launch points are partitioned per arc,
// L_ij for arc (i, j) -- a point only exists "on" one specific directed
// arc, and (constraints 5-6, 13-15) is only usable if the truck's chosen
// route actually traverses that arc.
//
// [ASSUMPTION: representation shape only, not a generation rule] this
// struct records *where* an already-generated point sits on its arc, as
// a ratio in [0, 1] from arc_from to arc_from's endpoint arc_to. It does
// not decide how many points exist per arc or at which ratios -- that
// generation rule is explicitly out of scope here and belongs to Phase
// E (see paper1-implementation-plan.md Sec 8, which documents that the
// paper does not specify the main-experiment generation rule precisely
// enough to reproduce exactly).
struct FlexibleLaunchPointData {
    NodeId node_id = kInvalidNodeId;

    NodeId arc_from = kInvalidNodeId;  // i in arc (i, j) this point lies on
    NodeId arc_to = kInvalidNodeId;    // j
    double position_ratio = 0.0;       // 0 = coincides with arc_from, 1 = coincides with arc_to
};

} // namespace tdr::model
