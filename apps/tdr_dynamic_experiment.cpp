// tdr_dynamic_experiment: reproducible paired comparison of scan/admission arms on identical scenarios (DM6 demo).
// Arms: B0 (scan none), G0/A0 [baseline], and for each eps~ in --eps-list: G0/A-PM, G3/A-PM, G3c/A-PM (G3c = explicitly NON-conservative comparator).
// Optimiser = identity (arm B1): no optimisation improvement is measured or claimed. All arms share the base plan, scenario seeds and digests.
// Outputs (in --out): experiment.csv (per arm x scenario, deterministic), summary.csv (per arm, means over scenarios, deterministic),
// runtime.csv (wall-clock, NOT deterministic, kept separate), config.txt. Usage:
//   tdr_dynamic_experiment [--n=20] [--scenarios=10] [--rho=1] [--omega=60] [--w-tilde=2] [--eps-list=0.5,2,8] [--varsigma=1] [--vns-iters=150]
//                          [--regime=P|F] [--sigma=0] [--zeta=1] [--seed=S] [--out=DIR] [--overwrite]
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>

#include "dynamic/runner.hpp"

using namespace tdr::dynamic;

namespace {
bool take(const std::string& a, const char* key, std::string& v) {
    const std::string k = std::string("--") + key + "=";
    if (a.rfind(k, 0) != 0) return false;
    v = a.substr(k.size());
    return true;
}
struct Arm { std::string label, scan, adm; double eps = 0.0; };

// fraction of the area inside the scan region for the BASE plan at tau = 0 (4096 fixed-seed uniform points; Monte Carlo, ~0.8% standard error)
double area_fraction(const DynamicRun& R, const DynamicConfig& c, const ScanRegion& scan) {
    std::mt19937_64 g(12345);
    std::uniform_real_distribution<double> U(0.0, c.base.base.instance.area_side_m);
    const PromiseLedger L = PromiseLedger::from_base_plan(*R.base.plan);
    int in = 0; const int N = 4096;
    for (int k = 0; k < N; ++k) {
        Request rq; rq.index = 1; rq.time = 0.0; rq.location = {U(g), U(g)}; rq.se1 = 30; rq.se2 = 30; rq.theta = 1;
        in += scan.contains(ScanContext{*R.base.plan, L, R.promise, c.omega, rq}) ? 1 : 0;
    }
    return static_cast<double>(in) / N;
}
} // namespace

int main(int argc, char** argv) {
    DynamicConfig c;
    c.base.base.instance.num_customers = 20; c.base.vns.iter_max = 150; c.rho = 1.0; c.omega = 60.0; c.scenarios = 10; c.w_tilde = 2.0; c.varsigma = 1.0;
    std::vector<double> eps = {0.5, 2.0, 8.0};
    std::string out = "dyn_experiment"; bool overwrite = false;
    std::vector<std::string> errs;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i]; std::string v;
        if (take(a, "n", v)) c.base.base.instance.num_customers = std::atoi(v.c_str());
        else if (take(a, "scenarios", v)) c.scenarios = std::atoi(v.c_str());
        else if (take(a, "rho", v)) c.rho = std::atof(v.c_str());
        else if (take(a, "zeta", v)) c.zeta = std::atof(v.c_str());
        else if (take(a, "omega", v)) c.omega = std::atof(v.c_str());
        else if (take(a, "w-tilde", v)) c.w_tilde = std::atof(v.c_str());
        else if (take(a, "varsigma", v)) c.varsigma = std::atof(v.c_str());
        else if (take(a, "vns-iters", v)) c.base.vns.iter_max = static_cast<std::size_t>(std::strtoull(v.c_str(), nullptr, 10));
        else if (take(a, "regime", v)) c.regime = v.empty() ? '?' : v[0];
        else if (take(a, "sigma", v)) c.sigma0 = std::strtoull(v.c_str(), nullptr, 10);
        else if (take(a, "seed", v)) c.base.base.seed.base_seed = std::strtoull(v.c_str(), nullptr, 10);
        else if (take(a, "out", v)) out = v;
        else if (a == "--overwrite") overwrite = true;
        else if (take(a, "eps-list", v)) { eps.clear(); std::stringstream ss(v); std::string t; while (std::getline(ss, t, ',')) eps.push_back(std::atof(t.c_str())); }
        else errs.push_back("unknown option " + a);
    }
    if (!errs.empty()) { for (auto& e : errs) std::cerr << "error: " << e << "\n"; return 2; }

    std::vector<Arm> arms = {{"B0", "none", "a0", 0}, {"G0/A0", "g0", "a0", 0}};
    for (double e : eps) {
        std::ostringstream s; s << "/APM(eps=" << e << ")";
        arms.push_back({"G0" + s.str(), "g0", "apm", e}); arms.push_back({"G3" + s.str(), "g3", "apm", e}); arms.push_back({"G3c" + s.str(), "g3c", "apm", e});
    }

    std::string per, sum, rt, cfgtxt;
    per = "arm,scan,admission,eps_tilde,sigma,digest,generated,out_of_scan,expired,stale,rejected_no_insertion,rejected_promise,accepted,acc_over_generated,acc_over_considered,base_objective,final_objective,"
          "mean_queue_wait_s,max_queue,evaluator_calls,stage_primary,stage_fallback,base_eta_shift_max_s,new_eta_shift_max_s,tolerance_violations,sim_ok,mean_response_s\n";
    sum = "arm,scan,admission,eps_tilde,scenarios,generated,out_of_scan,expired,stale,rejected_no_insertion,rejected_promise,accepted,acc_over_generated,acc_over_considered,"
          "mean_final_objective_per_accepted_delta,mean_queue_wait_s,mean_evaluator_calls,scan_area_fraction_base_plan,paired_digests_match\n";
    rt = "arm,wall_clock_s_including_base_plan_build\n";
    std::vector<std::string> ref_digests; bool all_ok = true; std::string cfg_line;
    for (const Arm& arm : arms) {
        DynamicConfig k = c; k.scan = arm.scan; k.admission = arm.adm; k.eps_tilde = arm.eps;
        const auto t0 = std::chrono::steady_clock::now();
        const DynamicRun R = run_dynamic(k);
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (!R.ok) { for (auto& e : R.errors) std::cerr << "error (" << arm.label << "): " << e << "\n"; return 1; }
        if (cfg_line.empty()) {
            std::ostringstream s; s << "n0=" << R.base.n0 << " regime=" << R.cfg.regime << " endurance_s=" << R.cfg.base.base.vehicle.endurance_s << " rho=" << k.rho << " zeta=" << k.zeta << " omega_s=" << k.omega << " scenarios=" << k.scenarios
                << " sigma0=" << k.sigma0 << " seed=" << k.base.base.seed.base_seed << " vns_iters=" << k.base.vns.iter_max << " w_tilde=" << k.w_tilde << " eps_list=";
            for (double e : eps) s << e << ";";
            s << " varsigma=" << k.varsigma << " base_objective=" << R.base.plan->objective() << " reference_makespan_s=" << R.base.reference_makespan << " optimiser=identity(B1) tool=" << kDynToolVersion;
            cfg_line = s.str();
        }
        bool match = true;
        if (ref_digests.empty()) for (const auto& r : R.runs) ref_digests.push_back(r.scenario.digest);
        for (std::size_t i = 0; i < R.runs.size(); ++i) match = match && R.runs[i].scenario.digest == ref_digests[i];
        all_ok = all_ok && match;
        const auto scan = make_scan(R.cfg);
        const double af = area_fraction(R, R.cfg, *scan);
        double S[6] = {0, 0, 0, 0, 0, 0}, gen = 0, qw = 0, ec = 0, dobj = 0; std::size_t ndec = 0, nacc = 0;
        char b[1024];
        for (const auto& r : R.runs) {
            const auto& m = r.sim; const double g = static_cast<double>(r.scenario.requests.size());
            double wait = 0; std::size_t nd = 0; int calls = 0;
            for (const auto& d : m.records) { calls += d.evaluator_calls; if (d.outcome != Outcome::OutOfScan && d.outcome != Outcome::Expired) { wait += d.start - d.arrival; ++nd; } }
            for (int o = 0; o < 6; ++o) S[o] += static_cast<double>(m.counts[o]);
            gen += g; qw += wait; ndec += nd; ec += calls; nacc += m.counts[5]; dobj += m.final_objective - m.base_objective;
            const double acc = static_cast<double>(m.counts[5]), cons = g - static_cast<double>(m.counts[0]);
            double resp = 0; std::size_t nr = 0; for (const auto& d : m.records) if (d.outcome != Outcome::OutOfScan && d.outcome != Outcome::Expired) { resp += d.install - d.arrival; ++nr; }
            std::snprintf(b, sizeof b, "%s,%s,%s,%g,%llu,%s,%.0f,%zu,%zu,%zu,%zu,%zu,%zu,%.6g,%.6g,%.10g,%.10g,%.6g,%zu,%d,%zu,%zu,%.6g,%.6g,%zu,%d,%.6g\n", arm.label.c_str(), arm.scan.c_str(), arm.adm.c_str(), arm.eps,
                          static_cast<unsigned long long>(r.scenario.params.sigma), r.scenario.digest.c_str(), g, m.counts[0], m.counts[1], m.counts[2], m.counts[3], m.counts[4], m.counts[5], g > 0 ? acc / g : 0.0,
                          cons > 0 ? acc / cons : 0.0, m.base_objective, m.final_objective, nd ? wait / static_cast<double>(nd) : 0.0, m.max_queue, calls, m.stage_primary, m.stage_fallback, m.shift_base.max, m.shift_new.max,
                          m.shift_base.violated + m.shift_new.violated, m.ok ? 1 : 0, nr ? resp / static_cast<double>(nr) : 0.0);
            per += b;
        }
        const double ns = static_cast<double>(R.runs.size());
        std::snprintf(b, sizeof b, "%s,%s,%s,%g,%zu,%.0f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6g,%.6g,%.10g,%.6g,%.6g,%.4f,%d\n", arm.label.c_str(), arm.scan.c_str(), arm.adm.c_str(), arm.eps, R.runs.size(), gen / ns,
                      S[0] / ns, S[1] / ns, S[2] / ns, S[3] / ns, S[4] / ns, S[5] / ns, gen > 0 ? S[5] / gen : 0.0, (gen - S[0]) > 0 ? S[5] / (gen - S[0]) : 0.0, nacc ? dobj / static_cast<double>(nacc) : 0.0,
                      ndec ? qw / static_cast<double>(ndec) : 0.0, ec / ns, af, match ? 1 : 0);
        sum += b;
        std::snprintf(b, sizeof b, "%s,%.3f\n", arm.label.c_str(), wall);
        rt += b;
    }
    std::filesystem::create_directories(out);
    if (!overwrite && std::filesystem::exists(std::filesystem::path(out) / "summary.csv")) { std::cerr << "error: " << out << "/summary.csv exists (use --overwrite)\n"; return 1; }
    auto w = [&](const char* name, const std::string& s) { std::ofstream(std::filesystem::path(out) / name, std::ios::binary) << s; };
    w("experiment.csv", per); w("summary.csv", sum); w("runtime.csv", rt); w("config.txt", cfg_line + "\n");
    std::cout << cfg_line << "\n\n" << sum << "\n(wall-clock in runtime.csv; paired digests " << (all_ok ? "match across all arms" : "MISMATCH") << ")\n";
    return all_ok ? 0 : 1;
}
