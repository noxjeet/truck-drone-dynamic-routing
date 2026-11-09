#pragma once

// Dynamic extension (DM4 slice): result files, schema "tdr.dynamic.results/1" (draft, vertical-slice subset). No wall-clock values are
// written, so identical (config, scenario) runs produce byte-identical files (replay check). Reuses JsonWriter / fnv1a64_hex.

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "dynamic/base_plan.hpp"
#include "dynamic/simulator.hpp"

namespace tdr::dynamic {

inline constexpr const char* kDynSchema = "tdr.dynamic.results/1"; // DM5 adds fields only (additive)
inline constexpr const char* kDynToolVersion = "tdr_dynamic 0.2 (DM3/DM4 slice + DM5 G3/A-PM)";

struct ScenarioRun {
    Scenario scenario;
    SimResult sim;
    std::string scan, admission;
};

inline std::string requests_csv(const ScenarioRun& r) {
    std::string s = "request,customer_id,arrival,x,y,outcome,start,install,cut_skel,cut_route_pos,cut_time,gaps_tried,gaps_feasible,gaps_straddled,delta_objective,eta_at_install,plan_version,queue_at_arrival,invariants_ok,stage,phi_primary_failed\n";
    char b[512];
    for (std::size_t i = 0; i < r.sim.records.size(); ++i) {
        const auto& d = r.sim.records[i];
        const auto& q = r.scenario.requests[i];
        std::snprintf(b, sizeof b, "%u,%d,%.17g,%.17g,%.17g,%s,%.17g,%.17g,%zu,%zu,%.17g,%zu,%zu,%zu,%.17g,%.17g,%u,%zu,%d,%s,%d\n", d.request, customer_id(d.request), q.time, q.location.x, q.location.y,
                      outcome_name(d.outcome), d.start, d.install, d.had_cut ? d.cut_skel : 0, d.had_cut ? d.cut_route_pos : 0, d.cut_time, d.gaps_tried, d.gaps_feasible, d.gaps_straddled,
                      d.delta_objective, d.eta_at_install, d.plan_version, d.queue_len_at_arrival, d.invariants_ok ? 1 : 0, stage_name(d.stage), d.phi_primary_failed ? 1 : 0);
        s += b;
    }
    return s;
}

inline std::string results_json(const DynamicConfig& cfg, const BasePlan& base, const std::vector<ScenarioRun>& runs, const PromiseParams& pp = PromiseParams{}) {
    tdr::runner::JsonWriter j;
    j.begin_object();
    j.field("schema", kDynSchema);
    j.field("tool_version", kDynToolVersion);
    j.field("static_baseline_tag", "static-baseline-m10");
    j.key("config"); j.begin_object();
    j.field("regime", std::string(1, cfg.regime)); j.field("endurance_s", cfg.base.base.vehicle.endurance_s);
    j.field("n0", base.n0); j.field("dataset_index", cfg.base.base.instance.dataset_index); j.field("seed", static_cast<unsigned long long>(cfg.base.base.seed.base_seed));
    j.field("rho", cfg.rho); j.field("zeta", cfg.zeta); j.field("arrival_count", cfg.count == ArrivalCount::Poisson ? "poisson" : "fixed");
    j.field("omega_s", cfg.omega); j.field("scan", cfg.scan); j.field("admission", cfg.admission);
    j.field("promise_enabled", pp.enabled);
    if (pp.enabled) { j.field("W_s", pp.W); j.field("E_base_s", pp.E_base); j.field("E_new_s", pp.E_new); j.field("w_tilde", cfg.w_tilde); j.field("eps_tilde", cfg.eps_tilde); }
    if (cfg.scan == "g3c") j.field("varsigma", cfg.varsigma);
    j.field("base_plan", cfg.base_kind == BaseKind::StaticVns ? "static-vns" : "constructive"); j.field("vns_iter_max", static_cast<unsigned long long>(cfg.base.vns.iter_max));
    j.end_object();
    j.key("base_plan"); j.begin_object();
    j.field("executed_objective", base.plan->objective()); j.field("executed_makespan_s", makespan_of(*base.plan));
    j.field("reference_makespan_s", base.reference_makespan); j.field("constructive_objective", base.constructive_objective); j.field("vns_objective", base.vns_objective);
    j.end_object();
    j.key("scenarios"); j.begin_array();
    for (const auto& r : runs) {
        j.begin_object();
        j.field("sigma", static_cast<unsigned long long>(r.scenario.params.sigma));
        j.field("digest", r.scenario.digest);
        j.field("horizon_s", r.scenario.horizon);
        j.field("generated", r.scenario.requests.size());
        j.field("sim_ok", r.sim.ok);
        j.key("errors"); j.begin_array(true); for (const auto& e : r.sim.errors) j.value(e); j.end_array();
        j.key("outcomes"); j.begin_object();
        for (int o = 0; o < 6; ++o) j.field(outcome_name(static_cast<Outcome>(o)), r.sim.counts[o]);
        j.end_object();
        const double gen = static_cast<double>(r.scenario.requests.size());
        const double considered = gen - static_cast<double>(r.sim.counts[static_cast<int>(Outcome::OutOfScan)]);
        const double acc = static_cast<double>(r.sim.counts[static_cast<int>(Outcome::Accepted)]);
        j.key("summary"); j.begin_object();
        j.field("accepted_over_generated", gen > 0 ? acc / gen : 0.0);
        j.field("accepted_over_considered", considered > 0 ? acc / considered : 0.0);
        j.field("base_objective", r.sim.base_objective); j.field("final_objective", r.sim.final_objective);
        j.field("base_makespan_s", r.sim.base_makespan); j.field("final_makespan_s", r.sim.final_makespan);
        j.field("max_queue", r.sim.max_queue); j.field("invariant_checks", r.sim.invariant_checks);
        double resp = 0.0; std::size_t nresp = 0;
        for (const auto& d : r.sim.records) if (d.outcome != Outcome::OutOfScan) { resp += d.install - d.arrival; ++nresp; }
        j.field("mean_response_s", nresp ? resp / static_cast<double>(nresp) : 0.0);
        j.field("stage_primary", r.sim.stage_primary); j.field("stage_fallback", r.sim.stage_fallback);
        for (int k = 0; k < 2; ++k) {
            const ClassShift& c = k == 0 ? r.sim.shift_base : r.sim.shift_new;
            const std::string pre = k == 0 ? "base_" : "new_";
            j.field(pre + "promised", c.n); j.field(pre + "eta_shift_mean_s", c.mean); j.field(pre + "eta_shift_max_s", c.max); j.field(pre + "tolerance_violations", c.violated);
        }
        j.field("final_plan_version", r.sim.final_plan ? r.sim.final_plan->version() : 0u);
        j.end_object();
        j.end_object();
    }
    j.end_array();
    j.end_object();
    return j.finish();
}

struct OutFile { std::string name, content; };

inline std::vector<OutFile> render_outputs(const DynamicConfig& cfg, const BasePlan& base, const std::vector<ScenarioRun>& runs, const PromiseParams& pp = PromiseParams{}) {
    std::vector<OutFile> out;
    out.push_back({"results.json", results_json(cfg, base, runs, pp)});
    for (const auto& r : runs) out.push_back({"requests_sigma" + std::to_string(r.scenario.params.sigma) + ".csv", requests_csv(r)});
    tdr::runner::JsonWriter m;
    m.begin_object();
    m.field("schema", std::string(kDynSchema) + ".manifest");
    m.key("files"); m.begin_array();
    for (const auto& f : out) { m.begin_object(); m.field("name", f.name); m.field("bytes", f.content.size()); m.field("fnv1a64", tdr::runner::fnv1a64_hex(f.content)); m.end_object(); }
    m.end_array();
    m.end_object();
    out.push_back({"manifest.json", m.finish()});
    return out;
}

// Refuses to overwrite an existing results.json unless asked; returns "" on success.
inline std::string write_outputs(const std::string& dir, const std::vector<OutFile>& files, bool overwrite) {
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec) return "cannot create " + dir + ": " + ec.message();
    if (!overwrite && fs::exists(fs::path(dir) / "results.json")) return dir + "/results.json exists (use --overwrite)";
    for (const auto& f : files) {
        std::ofstream o(fs::path(dir) / f.name, std::ios::binary);
        if (!o) return "cannot write " + f.name;
        o << f.content;
    }
    return "";
}

} // namespace tdr::dynamic
