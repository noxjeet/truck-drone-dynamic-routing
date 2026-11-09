#pragma once

// Dynamic extension (DM4 slice): exogenous scenario generator (DM0 §5). Uniform spatial model, homogeneous Poisson or fixed-count arrivals.
// The scenario is generated in full BEFORE any simulation from policy-independent streams (Arrivals, Locations). Fixed draw order:
// count -> times -> locations -> attributes. Clustered/corridor models, non-homogeneous profiles and thinning marks are not in the slice.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "dynamic/ids.hpp"
#include "dynamic/request.hpp"
#include "runner/result_writer.hpp" // fnv1a64_hex only
#include "util/rng.hpp"

namespace tdr::dynamic {

enum class ArrivalCount { Poisson, Fixed };

struct ScenarioParams {
    double rho = 0.5;       // degree of dynamism: E[M] / n0
    double zeta = 1.0;      // request horizon T_A = zeta * T0_ref
    ArrivalCount count = ArrivalCount::Poisson;
    double area_side_m = 60000.0;
    double se1 = 30.0, se2 = 30.0, theta = 1.0;
    int m_max = 200;        // n0 + m_max + 1 <= 1024 is checked by the runner (DenseIndex)
    std::uint64_t base_seed = 20261109ULL;
    std::uint64_t sigma = 0;
};

struct Scenario {
    ScenarioParams params;
    int n0 = 0;
    double t0_ref = 0.0;
    double horizon = 0.0; // T_A
    std::vector<Request> requests; // sorted by (time, index); index 1..M
    std::string digest;            // FNV-1a over the exact bit-level content
    bool ok = true;
    std::string error;
};

inline Scenario generate_scenario(const ScenarioParams& p, int n0, double t0_ref) {
    Scenario S;
    S.params = p;
    S.n0 = n0;
    S.t0_ref = t0_ref;
    S.horizon = p.zeta * t0_ref;
    if (!(p.rho >= 0.0) || !(p.zeta > 0.0) || !(t0_ref > 0.0) || !(p.area_side_m > 0.0) || p.m_max < 0 || n0 < 0) { S.ok = false; S.error = "invalid scenario parameters"; return S; }
    DenseIndex d{n0, p.m_max};
    if (!d.valid()) { S.ok = false; S.error = "n0 + m_max + 1 exceeds 1024"; return S; }

    tdr::util::Rng arr(tdr::util::derive_seed(p.base_seed, tdr::util::SeedStream::Arrivals, p.sigma));
    const double lambda_total = p.rho * n0;
    int M = 0;
    if (p.count == ArrivalCount::Fixed) {
        M = static_cast<int>(std::llround(lambda_total));
    } else { // Poisson(Lambda) as the number of unit-rate exponential partial sums not exceeding Lambda
        double acc = 0.0;
        for (;;) {
            acc += -std::log(1.0 - arr.uniform01());
            if (acc > lambda_total) break;
            ++M;
            if (M > p.m_max) break; // fail loudly below; never truncate
        }
    }
    if (M > p.m_max) { S.ok = false; S.error = "scenario has more than m_max requests (never silently truncated)"; return S; }

    std::vector<double> times(static_cast<std::size_t>(M));
    for (auto& t : times) t = arr.uniform_real(0.0, S.horizon);
    std::stable_sort(times.begin(), times.end());

    tdr::util::Rng loc(tdr::util::derive_seed(p.base_seed, tdr::util::SeedStream::Locations, p.sigma));
    for (int i = 0; i < M; ++i) {
        Request r;
        r.index = static_cast<std::uint32_t>(i + 1);
        r.time = times[static_cast<std::size_t>(i)];
        const double x = loc.uniform_real(0.0, p.area_side_m);
        const double y = loc.uniform_real(0.0, p.area_side_m);
        r.location = tdr::model::Point2D{x, y};
        r.se1 = p.se1; r.se2 = p.se2; r.theta = p.theta;
        S.requests.push_back(r);
    }

    std::string blob;
    char b[160];
    std::snprintf(b, sizeof b, "n0=%d M=%d TA=%.17g\n", n0, M, S.horizon);
    blob += b;
    for (const auto& r : S.requests) { std::snprintf(b, sizeof b, "%u %.17g %.17g %.17g\n", r.index, r.time, r.location.x, r.location.y); blob += b; }
    S.digest = tdr::runner::fnv1a64_hex(blob);
    return S;
}

} // namespace tdr::dynamic
