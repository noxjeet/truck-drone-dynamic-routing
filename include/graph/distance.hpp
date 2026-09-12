#pragma once

#include <cmath>

#include "model/node.hpp"

namespace tdr::graph {

// [PAPER] Sec 2.1: nodes sit in a Euclidean plane (the synthetic
// instances are drawn uniformly in a 60x60 km square). This is the only
// distance model used anywhere in this repository -- no road network,
// no travel-time API, nothing beyond straight-line distance.
//
//   d(i, j) = sqrt((x_i - x_j)^2 + (y_i - y_j)^2)
//
// This is symmetric (d(i,j) == d(j,i)) and zero exactly when the two
// points coincide, both of which later phases can rely on.
inline double euclidean_distance(const tdr::model::Point2D& a, const tdr::model::Point2D& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

inline double euclidean_distance(const tdr::model::Node& a, const tdr::model::Node& b) {
    return euclidean_distance(a.coordinates, b.coordinates);
}

} // namespace tdr::graph
