#pragma once

// Dynamic extension (DM4 slice): configuration of the vertical slice. A separate struct (Q10, proposed); the static 48-key table and
// ExperimentConfig are reused unchanged for everything static (instance, vehicle, FLP, VNS) and are not extended.

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "config/experiment_config.hpp"
#include "dynamic/scenario.hpp"

namespace tdr::dynamic {

enum class BaseKind { StaticVns, Constructive };

inline constexpr double kPaperEnduranceS = 1800.0; // regime P; regime F = 2x this, everything else identical (math_spec §7)

struct DynamicConfig {
    tdr::config::ExperimentConfig base;
    char regime = 'P';
    double rho = 0.5, zeta = 1.0;
    ArrivalCount count = ArrivalCount::Poisson;
    double omega = 10.0;       // simulated decision latency (s)
    std::string scan = "g0";   // g0 | none | g3 | g3c
    std::string admission = "a0"; // a0 (PM0) | apm (A-PM: PM1+PM2, U1 chain)
    // A-PM parameters, dimensionless (DM0 §7.2). Placeholders, NOT frozen: the grid W~ in {1,2,4}, eps~ in {0,.5,1,2,4,8} is placed on pilot seeds (§7.2).
    double w_tilde = 2.0;      // W = w_tilde * tau_L, tau_L = L / v_T            [PROPOSED]
    double eps_tilde = 1.0;    // E = eps_tilde * tau_bar, tau_bar = (L/sqrt(n0)) / v_T   [PROPOSED]
    double eps_base_tilde = -1.0; // < 0: E_base = E_new (D4 default); >= 0: separate class tolerance (sensitivity)
    double varsigma = 1.0;     // G3c tube radius in units of l = L/sqrt(n0)      [PROPOSED placeholder]
    BaseKind base_kind = BaseKind::StaticVns;
    int m_max = 200;
    std::uint64_t sigma0 = 0;
    int scenarios = 1;
    std::string out;
    bool overwrite = false;
};

// instance-derived scales (DM0 7.2): tau_L = L / v_T (size independent); tau_bar = (L / sqrt(n0)) / v_T
inline double promise_tau_L(const DynamicConfig& c) { return c.base.base.instance.area_side_m / c.base.base.vehicle.truck_speed_mps; }
inline double promise_ell(const DynamicConfig& c) { return c.base.base.instance.area_side_m / std::sqrt(static_cast<double>(std::max(1, c.base.base.instance.num_customers))); }
inline double promise_tau_bar(const DynamicConfig& c) { return promise_ell(c) / c.base.base.vehicle.truck_speed_mps; }

// Applies the regime to the static vehicle config. F doubles endurance only. Idempotent.
inline void apply_regime(DynamicConfig& c) {
    c.base.base.vehicle.endurance_s = (c.regime == 'F') ? 2.0 * kPaperEnduranceS : c.base.base.vehicle.endurance_s;
}

inline std::vector<std::string> validate(const DynamicConfig& c) {
    std::vector<std::string> e = tdr::config::validate(c.base);
    if (c.regime != 'P' && c.regime != 'F') e.push_back("regime must be P or F");
    if (!(c.rho >= 0.0)) e.push_back("rho must be >= 0");
    if (!(c.zeta > 0.0)) e.push_back("zeta must be > 0");
    if (!(c.omega >= 0.0)) e.push_back("omega must be >= 0");
    if (c.scan != "g0" && c.scan != "none" && c.scan != "g3" && c.scan != "g3c") e.push_back("scan must be g0, none, g3 or g3c (G1/G2 are not implemented)");
    if (c.admission != "a0" && c.admission != "apm") e.push_back("admission must be a0 or apm");
    if (!(c.w_tilde > 0.0)) e.push_back("w_tilde must be > 0");
    if (!(c.eps_tilde >= 0.0)) e.push_back("eps_tilde must be >= 0");
    if (!(c.varsigma > 0.0)) e.push_back("varsigma must be > 0");
    if (c.admission == "apm" && c.w_tilde * promise_tau_L(c) <= c.omega) e.push_back("W must exceed omega (feasibility floor, DM0 7.2): every decided request would be expired or rejected");
    if (c.m_max < 0 || c.base.base.instance.num_customers + c.m_max + 1 > 1024) e.push_back("n0 + m_max + 1 must be <= 1024");
    if (c.scenarios < 1) e.push_back("scenarios must be >= 1");
    return e;
}

} // namespace tdr::dynamic
