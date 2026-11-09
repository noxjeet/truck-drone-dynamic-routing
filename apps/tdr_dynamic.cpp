// tdr_dynamic: executable dynamic pipeline (DM3/DM4 vertical slice). Static VNS base plan -> exogenous Poisson/uniform requests ->
// FIFO decisions with latency omega -> clean-cut truck-only cheapest insertion (evaluator-verified) -> A0 admission -> results.
// Usage: tdr_dynamic [--n=N] [--dataset=K] [--seed=S] [--regime=P|F] [--rho=R] [--zeta=Z] [--fixed-count] [--omega=W] [--scan=g0|none|g3|g3c] [--admission=a0|apm]
//                    [--w-tilde=W] [--eps-tilde=E] [--eps-base-tilde=E] [--varsigma=S]
//                    [--base=vns|constructive] [--vns-iters=I] [--sigma=S0] [--scenarios=K] [--m-max=M] [--set=static.key=value]
//                    [--out=DIR] [--overwrite] [--trace]   (--trace prints a human-readable plan/event trace of every scenario; read-only)
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "dynamic/runner.hpp"
#include "dynamic/trace.hpp"

using namespace tdr::dynamic;

namespace {
bool take(const std::string& a, const char* key, std::string& v) {
    const std::string k = std::string("--") + key + "=";
    if (a.rfind(k, 0) != 0) return false;
    v = a.substr(k.size());
    return true;
}
} // namespace

int main(int argc, char** argv) {
    DynamicConfig c;
    bool trace = false;
    std::vector<std::string> errs;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        std::string v;
        if (take(a, "n", v)) c.base.base.instance.num_customers = std::atoi(v.c_str());
        else if (take(a, "dataset", v)) c.base.base.instance.dataset_index = std::atoi(v.c_str());
        else if (take(a, "seed", v)) c.base.base.seed.base_seed = std::strtoull(v.c_str(), nullptr, 10);
        else if (take(a, "regime", v)) c.regime = v.empty() ? '?' : v[0];
        else if (take(a, "rho", v)) c.rho = std::atof(v.c_str());
        else if (take(a, "zeta", v)) c.zeta = std::atof(v.c_str());
        else if (a == "--fixed-count") c.count = ArrivalCount::Fixed;
        else if (take(a, "omega", v)) c.omega = std::atof(v.c_str());
        else if (take(a, "scan", v)) c.scan = v;
        else if (take(a, "admission", v)) c.admission = v;
        else if (take(a, "w-tilde", v)) c.w_tilde = std::atof(v.c_str());
        else if (take(a, "eps-tilde", v)) c.eps_tilde = std::atof(v.c_str());
        else if (take(a, "eps-base-tilde", v)) c.eps_base_tilde = std::atof(v.c_str());
        else if (take(a, "varsigma", v)) c.varsigma = std::atof(v.c_str());
        else if (take(a, "base", v)) { if (v == "vns") c.base_kind = BaseKind::StaticVns; else if (v == "constructive") c.base_kind = BaseKind::Constructive; else errs.push_back("--base must be vns or constructive"); }
        else if (take(a, "vns-iters", v)) c.base.vns.iter_max = static_cast<std::size_t>(std::strtoull(v.c_str(), nullptr, 10));
        else if (take(a, "sigma", v)) c.sigma0 = std::strtoull(v.c_str(), nullptr, 10);
        else if (take(a, "scenarios", v)) c.scenarios = std::atoi(v.c_str());
        else if (take(a, "m-max", v)) c.m_max = std::atoi(v.c_str());
        else if (take(a, "out", v)) c.out = v;
        else if (a == "--overwrite") c.overwrite = true;
        else if (a == "--trace") trace = true;
        else if (take(a, "set", v)) {
            const auto eq = v.find('=');
            if (eq == std::string::npos) { errs.push_back("--set needs key=value"); continue; }
            const std::string e = tdr::config::set_config_value(c.base, v.substr(0, eq), v.substr(eq + 1));
            if (!e.empty()) errs.push_back(e);
        } else errs.push_back("unknown option " + a);
    }
    if (!errs.empty()) { for (auto& e : errs) std::cerr << "error: " << e << "\n"; return 2; }

    DynamicRun R = run_dynamic(c);
    if (R.base.plan) {
        std::cout << "base plan: n0=" << R.base.n0 << " executed objective " << R.base.plan->objective() << ", reference makespan " << R.base.reference_makespan << " s (regime "
                  << R.cfg.regime << ", endurance " << R.cfg.base.base.vehicle.endurance_s << " s)\n";
    }
    for (const auto& r : R.runs) {
        std::cout << "scenario sigma=" << r.scenario.params.sigma << " digest " << r.scenario.digest << ": generated " << r.scenario.requests.size();
        for (int o = 0; o < 6; ++o) std::cout << ", " << outcome_name(static_cast<Outcome>(o)) << " " << r.sim.counts[o];
        if (R.promise.enabled) std::cout << " | stage primary " << r.sim.stage_primary << " fallback " << r.sim.stage_fallback;
        std::cout << " | objective " << r.sim.base_objective << " -> " << r.sim.final_objective << " | invariant checks " << r.sim.invariant_checks << (r.sim.ok ? "" : " | FAILED") << "\n";
    }
    if (trace) for (const auto& r : R.runs) std::cout << "\n" << trace_text(R.cfg, R.base, r, R.promise) << "\n";
    for (auto& w : R.warnings) std::cerr << "warning: " << w << "\n";
    if (!R.errors.empty()) { for (auto& e : R.errors) std::cerr << "error: " << e << "\n"; return 1; }
    if (!c.out.empty()) {
        const std::string e = write_outputs(c.out, render_outputs(R.cfg, R.base, R.runs, R.promise), c.overwrite);
        if (!e.empty()) { std::cerr << "error: " << e << "\n"; return 1; }
        std::cout << "wrote " << c.out << "\n";
    }
    return 0;
}
