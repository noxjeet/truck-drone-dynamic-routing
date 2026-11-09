#pragma once

#include <cstdint>

#include "model/customer.hpp"
#include "model/node.hpp"

namespace tdr::dynamic {

// One exogenous demand request (DM0 §5). Index is 1-based; the customer id is kC0 + index (ids.hpp). The thinning mark of DM0 §5.6
// is reserved and deliberately NOT stored in v1.
struct Request {
    std::uint32_t index = 0;
    double time = 0.0;                 // arrival time tau_r (simulated seconds)
    tdr::model::Point2D location;
    double se1 = 0.0, se2 = 0.0, theta = 0.0;
};

} // namespace tdr::dynamic
