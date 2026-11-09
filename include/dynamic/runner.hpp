#pragma once

// Dynamic extension (DM4 slice): runner = base plan -> scenarios -> simulation -> outputs.

#include <memory>
#include <string>
#include <vector>

#include "dynamic/writer.hpp"

namespace tdr::dynamic {

struct DynamicRun {
    bool ok = false;
    std::vector<std::string> errors;
    DynamicConfig cfg;   // the EFFECTIVE configuration (regime applied); writers and printers must use this one
    BasePlan base;
    std::vector<ScenarioRun> runs;
    PromiseParams promise;              // effective W, E_base, E_new in seconds (PM0: disabled)
    std::vector<std::string> warnings;  // non-fatal advisories (e.g. queue load)
};

inline std::unique_ptr<ScanRegion> make_scan(const DynamicConfig& c) {
    if (c.scan == "none") return std::make_unique<ScanNone>();
    if (c.scan == "g3") return std::make_unique<ScanG3>();
    if (c.scan == "g3c") { auto g = std::make_unique<ScanG3c>(); g->varsigma = c.varsigma; g->ell = promise_ell(c); return g; }
    return std::make_unique<ScanAll>();
}

inline std::unique_ptr<AdmissionPolicy> make_admission(const DynamicConfig& c) {
    if (c.admission == "apm") return std::make_unique<AdmitAPM>();
    return std::make_unique<AdmitA0>();
}

inline PromiseParams make_promise(const DynamicConfig& c) {
    PromiseParams p;
    if (c.admission != "apm") return p; // PM0
    p.enabled = true;
    p.W = c.w_tilde * promise_tau_L(c);
    p.E_new = c.eps_tilde * promise_tau_bar(c);
    p.E_base = c.eps_base_tilde < 0.0 ? p.E_new : c.eps_base_tilde * promise_tau_bar(c);
    return p;
}

inline ScenarioParams scenario_params(const DynamicConfig& c, std::uint64_t sigma) {
    ScenarioParams p;
    p.rho = c.rho; p.zeta = c.zeta; p.count = c.count; p.m_max = c.m_max;
    p.area_side_m = c.base.base.instance.area_side_m;
    p.se1 = c.base.base.instance.truck_service_time_s; p.se2 = c.base.base.instance.drone_service_time_s; p.theta = c.base.base.instance.theta;
    p.base_seed = c.base.base.seed.base_seed; p.sigma = sigma;
    return p;
}

inline DynamicRun run_dynamic(DynamicConfig cfg) {
    DynamicRun R;
    apply_regime(cfg);
    R.cfg = cfg;
    R.errors = validate(cfg);
    if (!R.errors.empty()) return R;
    R.base = build_base_plan(cfg);
    if (!R.base.ok) { R.errors.push_back(R.base.error); return R; }
    const auto scan = make_scan(cfg);
    const auto admit_p = make_admission(cfg);
    const AdmissionPolicy& admit = *admit_p;
    R.promise = make_promise(cfg);
    bool all_ok = true;
    for (int k = 0; k < cfg.scenarios; ++k) {
        ScenarioRun sr;
        // the scenario scale ALWAYS comes from the static-VNS reference makespan, whichever base plan is executed (U6, proposed)
        sr.scenario = generate_scenario(scenario_params(cfg, cfg.sigma0 + static_cast<std::uint64_t>(k)), R.base.n0, R.base.reference_makespan);
        if (!sr.scenario.ok) { R.errors.push_back("scenario " + std::to_string(cfg.sigma0 + k) + ": " + sr.scenario.error); return R; }
        sr.scan = scan->name(); sr.admission = admit.name();
        SimOptions so; so.omega = cfg.omega; so.scan = scan.get(); so.admission = &admit; so.promise = R.promise;
        sr.sim = simulate(*R.base.plan, sr.scenario, so);
        { // queue-load estimate rho_q = (arrivals * omega) / T_A (DM0 7.2); advisory only
            const double rq = sr.scenario.horizon > 0 ? static_cast<double>(sr.scenario.requests.size()) * cfg.omega / sr.scenario.horizon : 0.0;
            if (rq >= 0.5) R.warnings.push_back("sigma " + std::to_string(sr.scenario.params.sigma) + ": queue load rho_q = " + std::to_string(rq) + " >= 0.5 (FIFO waiting will consume W)");
        }
        if (!sr.sim.ok) { all_ok = false; for (auto& e : sr.sim.errors) R.errors.push_back("sigma " + std::to_string(sr.scenario.params.sigma) + ": " + e); }
        R.runs.push_back(std::move(sr));
    }
    R.ok = all_ok;
    return R;
}

} // namespace tdr::dynamic
