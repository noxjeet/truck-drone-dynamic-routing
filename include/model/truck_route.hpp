#pragma once

#include <vector>

#include "model/node.hpp"

namespace tdr::model {

// The truck's route: a single ordered sequence of node ids, starting and
// ending at a depot node, and including every truck-served customer plus
// whichever flexible launch points are currently active (i.e. actually
// used as a launch or recovery site) on this route.
//
// [PAPER] Sec 5.1 describes the truck route as one ordered path
// depot -> ... -> depot containing depot, customer, and flexible-launch
// nodes in sequence. A plain ordered list of NodeIds is a direct,
// unembellished representation of that -- there is no reason for a
// dedicated wrapper class here (no invariant beyond "it's a sequence" is
// enforced at this phase; anything about which flex points may
// legitimately appear, or in what order, belongs to whichever module
// actually builds or edits routes).
using TruckRoute = std::vector<NodeId>;

} // namespace tdr::model
