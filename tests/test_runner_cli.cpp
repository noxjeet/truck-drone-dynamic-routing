// M8: runner, result writer and CLI (include/runner/*.hpp). The runner is checked against a hand-built pipeline that calls the
// unchanged M7 run_vns directly (VNS behaviour and seed handling preserved), the outputs are parsed back by an independent JSON
// reader, and the CLI is driven in-process.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "reference/timing_engine_reference.hpp"
#include "runner/cli.hpp"

using namespace tdr;
using namespace tdr::runner;
using tdr::config::ExperimentConfig;

namespace {

int g_failures = 0;
int g_passes = 0;

void check(bool condition, const std::string& description) {
    if (condition) { ++g_passes; std::cout << "[PASS] " << description << "\n"; }
    else { ++g_failures; std::cout << "[FAIL] " << description << "\n"; }
}

// ------------------------------------------------------------------------------------------------ independent JSON reader
struct Json {
    enum T { Null, Bool, Num, Str, Arr, Obj } t = Null;
    bool b = false;
    double n = 0;
    std::string s, raw;
    std::vector<Json> a;
    std::vector<std::pair<std::string, Json>> o;
    const Json& at(const std::string& k) const {
        static const Json none;
        for (const auto& kv : o) if (kv.first == k) return kv.second;
        return none;
    }
    bool has(const std::string& k) const { for (const auto& kv : o) if (kv.first == k) return true; return false; }
};

struct JParser {
    const std::string& s;
    std::size_t i = 0;
    bool ok = true;
    explicit JParser(const std::string& str) : s(str) {}
    void ws() { while (i < s.size() && (s[i] == ' ' || s[i] == '\n' || s[i] == '\t' || s[i] == '\r')) ++i; }
    bool lit(const char* w) { const std::size_t L = std::char_traits<char>::length(w); if (s.compare(i, L, w) == 0) { i += L; return true; } return false; }
    Json value() {
        ws();
        Json j;
        if (i >= s.size()) { ok = false; return j; }
        const char c = s[i];
        if (c == '{') {
            ++i; j.t = Json::Obj; ws();
            if (i < s.size() && s[i] == '}') { ++i; return j; }
            for (;;) {
                ws(); Json k = value(); if (k.t != Json::Str) { ok = false; return j; }
                ws(); if (i >= s.size() || s[i] != ':') { ok = false; return j; } ++i;
                Json v = value(); j.o.push_back({k.s, v}); ws();
                if (i < s.size() && s[i] == ',') { ++i; continue; }
                if (i < s.size() && s[i] == '}') { ++i; return j; }
                ok = false; return j;
            }
        }
        if (c == '[') {
            ++i; j.t = Json::Arr; ws();
            if (i < s.size() && s[i] == ']') { ++i; return j; }
            for (;;) {
                j.a.push_back(value()); ws();
                if (i < s.size() && s[i] == ',') { ++i; continue; }
                if (i < s.size() && s[i] == ']') { ++i; return j; }
                ok = false; return j;
            }
        }
        if (c == '"') {
            ++i; j.t = Json::Str;
            while (i < s.size() && s[i] != '"') {
                if (s[i] == '\\') {
                    ++i; if (i >= s.size()) { ok = false; return j; }
                    switch (s[i]) { case 'n': j.s += '\n'; break; case 't': j.s += '\t'; break; case 'r': j.s += '\r'; break; case '"': j.s += '"'; break; case '\\': j.s += '\\'; break;
                        case 'u': { j.s += static_cast<char>(std::strtol(s.substr(i + 1, 4).c_str(), nullptr, 16)); i += 4; break; } default: ok = false; }
                } else j.s += s[i];
                ++i;
            }
            if (i >= s.size()) { ok = false; return j; }
            ++i; return j;
        }
        if (lit("true")) { j.t = Json::Bool; j.b = true; return j; }
        if (lit("false")) { j.t = Json::Bool; return j; }
        if (lit("null")) { return j; }
        const std::size_t st = i;
        while (i < s.size() && (std::isdigit(static_cast<unsigned char>(s[i])) || s[i] == '-' || s[i] == '+' || s[i] == '.' || s[i] == 'e' || s[i] == 'E')) ++i;
        if (st == i) { ok = false; return j; }
        j.t = Json::Num; j.raw = s.substr(st, i - st); j.n = std::strtod(j.raw.c_str(), nullptr);
        return j;
    }
};

bool parse_json(const std::string& text, Json& out) {
    JParser p(text);
    out = p.value();
    p.ws();
    return p.ok && p.i == text.size();
}

std::vector<std::string> split(const std::string& s, char d) {
    std::vector<std::string> v;
    std::string cur;
    for (char c : s) { if (c == d) { v.push_back(cur); cur.clear(); } else cur += c; }
    v.push_back(cur);
    return v;
}

std::string slurp(const std::filesystem::path& p) {
    std::ifstream f(p, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

ExperimentConfig small_cfg(int n = 10, std::size_t iters = 60, int runs = 3, std::uint64_t seed = 31415) {
    ExperimentConfig c;
    c.base.seed.base_seed = seed;
    c.base.instance.num_customers = n;
    c.vns.iter_max = iters;
    c.experiment.runs = runs;
    return c;
}

// the pipeline of the runner, written out by hand (as bench_vns and the M7 tests do it)
struct Pipe {
    instance::StaticInstance inst;
    init::InitialSolutionResult init;
    std::unique_ptr<graph::SpatialNetwork> net;
    std::unique_ptr<flp::FlpArcIndex> idx;
    bool ok = false;
    Pipe(const ExperimentConfig& cfg, int dataset, const tsp::TSPSolverFactory& fac) {
        config::StaticConfig sc = cfg.base;
        sc.instance.dataset_index = dataset;
        const auto gen = instance::generate_static_instance(sc);
        if (!gen.ok) return;
        inst = gen.instance;
        init::InitialSolutionPolicy pol;
        pol.kmeans_policy = cfg.initial.kmeans;
        pol.flp_policy.unit_division_length = sc.flp.unit_division_length_m;
        const auto net0 = inst.make_network();
        init = init::build_initial_solution(inst.depot, inst.customers, inst.customer_data, net0, pol, fac);
        if (!init.feasible) return;
        net = std::make_unique<graph::SpatialNetwork>(init.node_registry, inst.truck, inst.drone);
        idx = std::make_unique<flp::FlpArcIndex>(init.flp_data);
        ok = true;
    }
    search::SearchContext ctx(int window) const { return search::SearchContext{*net, inst.customer_data, init.flp_data, *idx, window}; }
};

bool same_solution(const model::Solution& a, const model::Solution& b) {
    if (a.truck_route != b.truck_route || a.drone_trips.size() != b.drone_trips.size()) return false;
    for (std::size_t i = 0; i < a.drone_trips.size(); ++i) {
        const auto &x = a.drone_trips[i], &y = b.drone_trips[i];
        if (x.drone_id != y.drone_id || x.launch_node != y.launch_node || x.service_customer != y.service_customer || x.recovery_node != y.recovery_node) return false;
    }
    return true;
}

bool same_run(const search::VnsResult& a, const search::VnsResult& b) {
    if (a.trace.size() != b.trace.size()) return false;
    for (std::size_t i = 0; i < a.trace.size(); ++i)
        if (a.trace[i].temperature != b.trace[i].temperature || a.trace[i].current != b.trace[i].current || a.trace[i].best != b.trace[i].best || a.trace[i].moves != b.trace[i].moves || a.trace[i].shake_kind != b.trace[i].shake_kind) return false;
    return same_solution(a.best, b.best) && same_solution(a.current, b.current) && a.best_objective == b.best_objective && a.current_objective == b.current_objective && a.iterations == b.iterations &&
           a.improving_moves == b.improving_moves && a.sa_accepted_moves == b.sa_accepted_moves && a.shakes_applied == b.shakes_applied && a.passes_truncated == b.passes_truncated;
}

int cli(const std::vector<std::string>& a, std::string* out = nullptr, std::string* err = nullptr) {
    std::ostringstream o, e;
    const int rc = cli_main(a, o, e);
    if (out) *out = o.str();
    if (err) *err = e.str();
    return rc;
}

std::filesystem::path fresh_dir(const std::string& name) {
    const auto d = std::filesystem::temp_directory_path() / ("tdr_m8_" + name);
    std::filesystem::remove_all(d);
    return d;
}

} // namespace

int main() {
    const tsp::TSPSolverFactory default_factory;

    // ------------------------------------------------------------------ the runner preserves the unchanged M7 behaviour and its seed handling
    {
        ExperimentConfig cfg = small_cfg(10, 80, 3);
        cfg.output.record_trace = true;
        const ExperimentResult X = run_experiment(cfg);
        check(X.ok && X.errors.empty() && X.datasets.size() == 1 && X.datasets[0].runs.size() == 3, "a 3-run experiment completes without errors");
        Pipe P(cfg, 0, default_factory);
        check(P.ok, "reference pipeline built by hand");
        if (P.ok && X.ok) {
            const auto ctx = P.ctx(cfg.base.neighbourhood.adjacency_window);
            check(same_solution(P.init.solution, X.datasets[0].initial) && P.init.objective_value == X.datasets[0].initial_objective,
                  "the initial solution built from the configuration equals the one built with the default TSPSolverFactory{} (F1/F2 defaults unchanged)");
            bool all_same = true, all_verified = true, ref_ok = true;
            for (int r = 0; r < 3; ++r) {
                search::VnsOptions o;                 // plain M7 defaults, set by hand
                o.iter_max = 80;
                o.base_seed = 31415;
                o.run_index = static_cast<std::uint64_t>(r);
                o.record_trace = true;
                const auto direct = search::run_vns(P.init.solution, ctx, o);
                all_same = all_same && same_run(direct, X.datasets[0].runs[r].vns);
                all_verified = all_verified && X.datasets[0].runs[r].verified;
                const auto e = timing_ref::evaluate_solution(X.datasets[0].runs[r].vns.best, *P.net, P.inst.customer_data, P.init.flp_data);
                ref_ok = ref_ok && e.feasible && std::fabs(e.objective_value - X.datasets[0].runs[r].vns.best_objective) <= 1e-9 * std::max(1.0, e.objective_value);
            }
            check(all_same, "every runner run is bit-identical (trace, best, current, counters) to a direct run_vns call with the same options and run index");
            check(all_verified && ref_ok, "every returned best solution passes the runner's own verification and the frozen reference evaluator");
        }
    }

    // ------------------------------------------------------------------ runner-level failure: reported, no crash, no runs
    {
        ExperimentConfig cfg = small_cfg(10, 10, 2);
        cfg.base.vehicle.drone_speed_mps = 3e-308;
        const ExperimentResult X = run_experiment(cfg);
        check(!X.ok && X.errors.size() == 1 && X.datasets.size() == 1 && !X.datasets[0].initial_feasible && X.datasets[0].runs.empty() && !X.datasets[0].diagnostics.empty(), "non-finite travel times (drone speed 3e-308) are reported by the preflight: experiment not ok, no runs, diagnostic kept");
    }

    // ------------------------------------------------------------------ non-default F1/F2 values reach the builder
    {
        ExperimentConfig cfg = small_cfg(25, 5, 1);
        cfg.initial.tsp_max_exact_n = 5;   // force the heuristic TSP
        cfg.initial.tsp_heuristic_restarts = 3;
        cfg.initial.tsp_heuristic_seed = 7;
        cfg.initial.kmeans.max_iterations = 11;
        const ExperimentResult X = run_experiment(cfg);
        Pipe P(cfg, 0, tsp::TSPSolverFactory(5, 3, 7));
        check(X.ok && P.ok && same_solution(P.init.solution, X.datasets[0].initial), "F1/F2 configuration values (TSP exact limit / restarts / seed, k-means) are passed to the initial-solution builder");
    }

    // ------------------------------------------------------------------ F1/F2 values that change the outcome reach the builder (non-vacuous)
    {
        ExperimentConfig base = small_cfg(40, 3, 1);
        Pipe dflt(base, 0, default_factory);
        // TSP: find an instance and a heuristic seed whose initial solution differs from the default seed's
        int found_seed = -1, found_n = 0, found_ds = 0;
        for (int n : {40, 60, 80}) for (int ds = 0; ds < 3 && found_seed < 0; ++ds) for (int sd = 1; sd <= 25 && found_seed < 0; ++sd) {
            ExperimentConfig c = small_cfg(n, 3, 1);
            c.base.instance.dataset_index = ds;
            c.initial.tsp_max_exact_n = 5; c.initial.tsp_heuristic_restarts = 3; c.initial.tsp_heuristic_seed = static_cast<std::uint64_t>(sd);
            Pipe a(c, ds, tsp::TSPSolverFactory(5, 3, static_cast<std::uint64_t>(sd)));
            Pipe b(c, ds, tsp::TSPSolverFactory(5, 3, 42));
            if (a.ok && b.ok && !same_solution(a.init.solution, b.init.solution)) { found_seed = sd; found_n = n; found_ds = ds; }
        }
        check(found_seed > 0, "a discriminating heuristic-TSP seed exists for the test (non-vacuity): n = " + std::to_string(found_n) + ", dataset " + std::to_string(found_ds) + ", seed " + std::to_string(found_seed));
        if (found_seed > 0) {
            ExperimentConfig c = small_cfg(found_n, 3, 1);
            c.base.instance.dataset_index = found_ds;
            c.initial.tsp_max_exact_n = 5; c.initial.tsp_heuristic_restarts = 3; c.initial.tsp_heuristic_seed = static_cast<std::uint64_t>(found_seed);
            const ExperimentResult X = run_experiment(c);
            Pipe P(c, found_ds, tsp::TSPSolverFactory(5, 3, static_cast<std::uint64_t>(found_seed)));
            Pipe P42(c, found_ds, tsp::TSPSolverFactory(5, 3, 42));
            check(X.ok && P.ok && P42.ok && same_solution(P.init.solution, X.datasets[0].initial) && !same_solution(P42.init.solution, X.datasets[0].initial), "initial.tsp_* values reach the TSP factory (the seed changes the initial solution, the runner follows the configured one)");
        }
        // k-means: find a setting that changes the initial solution
        double found_eps = -1; int found_it = 0;
        for (int it : {1, 2, 3}) {
            ExperimentConfig c = base;
            c.initial.kmeans.max_iterations = it; c.initial.kmeans.convergence_epsilon = 1e12;
            Pipe a(c, 0, default_factory);
            if (a.ok && dflt.ok && !same_solution(a.init.solution, dflt.init.solution)) { found_it = it; found_eps = 1e12; break; }
        }
        check(found_it > 0, "a discriminating k-means setting exists for the test (non-vacuity)");
        if (found_it > 0) {
            ExperimentConfig c = base;
            c.initial.kmeans.max_iterations = found_it; c.initial.kmeans.convergence_epsilon = found_eps;
            const ExperimentResult X = run_experiment(c);
            Pipe P(c, 0, default_factory);
            check(X.ok && P.ok && same_solution(P.init.solution, X.datasets[0].initial) && !same_solution(dflt.init.solution, X.datasets[0].initial), "initial.kmeans_* values reach the clustering step");
        }
    }

    // ------------------------------------------------------------------ the runner's verification rejects what it must reject (negative tests)
    {
        ExperimentConfig cfg = small_cfg(10, 40, 1);
        Pipe P(cfg, 0, default_factory);
        if (P.ok && !P.init.solution.drone_trips.empty()) {
            const auto ctx = P.ctx(3);
            const model::Solution& good = P.init.solution;
            check(verify_best(good, P.init.objective_value, ctx, P.inst.customers).empty(), "verify_best accepts a feasible solution with its exact objective");
            check(!verify_best(good, P.init.objective_value + 1e-3, ctx, P.inst.customers).empty(), "...rejects a stated objective that differs from the evaluator's");
            model::Solution dup = good;
            for (auto id : good.truck_route) if (P.init.node_registry.at(id).type == model::NodeType::Customer) { dup.drone_trips[0].service_customer = id; break; }
            check(!verify_best(dup, P.init.objective_value, ctx, P.inst.customers).empty(), "...rejects a solution that serves a customer twice and another not at all");
            model::Solution twice = good;
            twice.drone_trips.push_back(good.drone_trips[0]);
            check(verify_best(twice, P.init.objective_value, ctx, P.inst.customers).find("not served exactly once") != std::string::npos, "...names the structural reason when a customer is served by two trips (reported before the evaluator is consulted)");
            model::Solution missing = good;
            missing.drone_trips.pop_back();
            check(!verify_best(missing, P.init.objective_value, ctx, P.inst.customers).empty(), "...rejects a solution with an unserved customer");
            bool found = false, rejected = false;
            for (std::size_t t = 0; t < good.drone_trips.size() && !found; ++t)
                for (auto id : good.truck_route) {
                    if (P.init.node_registry.at(id).type != model::NodeType::Customer || id == good.drone_trips[t].recovery_node) continue;
                    model::Solution bad = good;
                    bad.drone_trips[t].recovery_node = id;
                    const auto e = timing::evaluate_solution(bad, *P.net, P.inst.customer_data, P.init.flp_data);
                    if (!search::classify_evaluation(e, ctx.num_drones()).feasible) { found = true; rejected = !verify_best(bad, e.objective_value, ctx, P.inst.customers).empty(); break; }
                }
            check(found && rejected, "...rejects a structurally complete solution that the evaluator declares infeasible");
            RunRecord good_rec, bad_rec, dead_rec;
            good_rec.vns.status = search::VnsStatus::Ok; good_rec.vns.best = good; good_rec.vns.best_objective = P.init.objective_value;
            bad_rec = good_rec; bad_rec.vns.best = dup;
            dead_rec.vns.status = search::VnsStatus::InvalidOptions; dead_rec.vns.message = "boom";
            verify_record(good_rec, ctx, P.inst.customers); verify_record(bad_rec, ctx, P.inst.customers); verify_record(dead_rec, ctx, P.inst.customers);
            check(good_rec.verified && good_rec.verify_note.empty() && !bad_rec.verified && !bad_rec.verify_note.empty() && !dead_rec.verified && dead_rec.verify_note == "boom", "verify_record: verified only when the run started and its best solution passes; the reason is kept");
        } else check(false, "verification fixture built");
    }

    // ------------------------------------------------------------------ VNS, shake, N4 and neighbourhood keys reach the algorithm
    {
        ExperimentConfig cfg = small_cfg(10, 80, 1);
        config::set_config_value(cfg, "vns.reference", "shaken-state");
        config::set_config_value(cfg, "vns.neighbourhoods", "N4,N5");
        config::set_config_value(cfg, "shake.max_segment_length", "2");
        config::set_config_value(cfg, "n4.max_reanchor_variants", "7");
        config::set_config_value(cfg, "vns.cooling_rate", "0.9");
        config::set_config_value(cfg, "vns.t0_absolute", "2.5");
        cfg.output.record_trace = true;
        const ExperimentResult X = run_experiment(cfg);
        Pipe P(cfg, 0, default_factory);
        search::VnsOptions o;
        o.iter_max = 80; o.base_seed = 31415; o.run_index = 0; o.record_trace = true;
        o.reference = search::AcceptanceReference::ShakenState; o.neighbourhoods = {search::OperatorId::N4, search::OperatorId::N5};
        o.shake.max_segment_length = 2; o.n4.max_reanchor_variants = 7; o.cooling_rate = 0.9; o.t0_absolute = 2.5;
        if (P.ok && X.ok) {
            const auto direct = search::run_vns(P.init.solution, P.ctx(3), o);
            const auto& v = X.datasets[0].runs[0].vns;
            check(same_run(direct, v), "reference, neighbourhood list, shake, N4 budget, cooling and T0 settings are honoured exactly as in a direct call");
            std::size_t other = 0;
            for (std::size_t k : {0u, 1u, 2u, 5u, 6u, 7u}) other += v.ops[k].calls;
            check(v.ops[3].calls > 0 && other == 0 && v.t0 == 2.5, "only N4 and N5 ran (custom list), T0 = 2.5");
        }
        ExperimentConfig w = small_cfg(10, 40, 1);
        w.base.neighbourhood.adjacency_window = 1;
        w.base.neighbourhood.n1_repeats = 1;
        w.base.neighbourhood.n2_repeats = 0 + 1;
        const ExperimentResult XW = run_experiment(w);
        Pipe PW(w, 0, default_factory);
        search::VnsOptions ow;
        ow.iter_max = 40; ow.base_seed = 31415; ow.run_index = 0; ow.n1_repeats = 1; ow.n2_repeats = 1;
        if (PW.ok && XW.ok) check(same_run(search::run_vns(PW.init.solution, PW.ctx(1), ow), XW.datasets[0].runs[0].vns), "adjacency window and N1/N2 repeats come from neighbourhood.* (window 1, one repeat each)");
    }

    // ------------------------------------------------------------------ run-index and dataset independence
    {
        ExperimentConfig full = small_cfg(10, 50, 4);
        full.output.record_trace = true;
        const ExperimentResult XF = run_experiment(full);
        bool ok = XF.ok;
        for (int r = 0; r < 4 && ok; ++r) {
            ExperimentConfig one = full;
            one.experiment.runs = 1;
            one.experiment.first_run_index = static_cast<std::uint64_t>(r);
            const ExperimentResult X1 = run_experiment(one);
            ok = ok && X1.ok && same_run(X1.datasets[0].runs[0].vns, XF.datasets[0].runs[static_cast<std::size_t>(r)].vns) && X1.datasets[0].runs[0].run_index == static_cast<std::uint64_t>(r);
        }
        check(ok, "run r of a 4-run experiment equals the single-run experiment with first_run_index = r (runs do not influence each other)");
        bool differ = false;
        for (int r = 1; r < 4; ++r) differ = differ || XF.datasets[0].runs[static_cast<std::size_t>(r)].vns.best_objective != XF.datasets[0].runs[0].vns.best_objective;
        check(differ, "different run indices give different trajectories");

        ExperimentConfig two = small_cfg(10, 30, 2);
        two.experiment.num_datasets = 2;
        const ExperimentResult X2 = run_experiment(two);
        ExperimentConfig d1 = two;
        d1.experiment.num_datasets = 1;
        d1.base.instance.dataset_index = 1;
        const ExperimentResult X3 = run_experiment(d1);
        check(X2.ok && X3.ok && X2.datasets.size() == 2 && X2.datasets[1].dataset_index == 1 && X2.datasets[1].instance_seed != X2.datasets[0].instance_seed &&
              same_solution(X2.datasets[1].initial, X3.datasets[0].initial) && X2.datasets[1].runs[1].vns.best_objective == X3.datasets[0].runs[1].vns.best_objective,
              "dataset d of a 2-dataset experiment equals the single experiment with dataset_index = d; the instance seeds differ");
    }

    // ------------------------------------------------------------------ determinism of the rendered outputs
    {
        ExperimentConfig cfg = small_cfg(10, 60, 3);
        cfg.output.record_trace = true;
        const ExperimentResult A = run_experiment(cfg), B = run_experiment(cfg);
        const auto fa = render_outputs(A), fb = render_outputs(B);
        bool same = fa.size() == fb.size();
        std::string nondet;
        for (std::size_t i = 0; same && i < fa.size(); ++i) {
            same = fa[i].name == fb[i].name;
            if (fa[i].deterministic) same = same && fa[i].content == fb[i].content; else nondet += fa[i].name;
        }
        check(same && nondet == "timing.json", "two runs of one configuration render byte-identical files; timing.json is the only non-deterministic file");
        ExperimentConfig other = cfg;
        other.base.seed.base_seed = 31416;
        const ExperimentResult C = run_experiment(other);
        check(results_json(C) != results_json(A), "another base seed gives different results");
        check(results_json(A).find("\"seconds\":") == std::string::npos && results_json(A).find("total_seconds") == std::string::npos && runs_csv(A).find("seconds") == std::string::npos && traces_csv(A).find("seconds") == std::string::npos,
              "no wall-clock quantity appears in the deterministic outputs");
        ExperimentConfig tl = cfg;
        tl.vns.max_seconds = 100.0;
        Json jr;
        parse_json(results_json(run_experiment(tl)), jr);
        check(jr.at("experiment").at("deterministic").t == Json::Bool && !jr.at("experiment").at("deterministic").b, "vns.max_seconds > 0 is flagged non-deterministic in the results");
    }

    // ------------------------------------------------------------------ the JSON and CSV outputs parse back and agree with the in-memory result
    {
        ExperimentConfig cfg = small_cfg(10, 300, 3);
        cfg.output.record_trace = true;
        const ExperimentResult X = run_experiment(cfg);
        Json r;
        const bool parsed = parse_json(results_json(X), r);
        check(parsed && r.at("schema").s == "tdr.static.results/2", "results.json is well-formed JSON with the schema tag");
        if (parsed) {
            const Json& d0 = r.at("datasets").a[0];
            check(r.at("datasets").a.size() == 1 && d0.at("runs").a.size() == 3 && d0.at("num_customers").n == 10 && d0.at("initial").at("feasible").b && r.at("experiment").at("config_digest").s == fnv1a64_hex(config::config_to_text(cfg)),
                  "structure: 1 dataset, 3 runs, customers, feasible initial solution, config digest = FNV-1a of the resolved configuration");
            bool vals = true, sols = true, guard = true, units = true;
            for (std::size_t k = 0; k < 3; ++k) {
                const Json& jr = d0.at("runs").a[k];
                const auto& v = X.datasets[0].runs[k].vns;
                vals = vals && std::strtod(jr.at("best_objective_s").raw.c_str(), nullptr) == v.best_objective && jr.at("iterations").n == static_cast<double>(v.iterations) && jr.at("status").s == "ok" && jr.at("verified").b &&
                       jr.at("stop").s == "iter-max" && jr.at("run_index").n == static_cast<double>(k) && jr.at("improving_moves").n == static_cast<double>(v.improving_moves);
                units = units && std::fabs(jr.at("best_objective_min").n - v.best_objective / 60.0) <= 1e-12 * v.best_objective;
                const Json& bs = jr.at("best_solution");
                sols = sols && bs.at("truck_route").a.size() == v.best.truck_route.size() && bs.at("drone_trips").a.size() == v.best.drone_trips.size();
                for (std::size_t q = 0; sols && q < v.best.truck_route.size(); ++q) sols = static_cast<long long>(bs.at("truck_route").a[q].n) == v.best.truck_route[q];
                for (std::size_t q = 0; sols && q < v.best.drone_trips.size(); ++q) sols = static_cast<int>(bs.at("drone_trips").a[q].at("customer").n) == v.best.drone_trips[q].service_customer && static_cast<int>(bs.at("drone_trips").a[q].at("launch").n) == v.best.drone_trips[q].launch_node;
                const Json& p = jr.at("passes");
                guard = guard && p.at("completed").n + p.at("skipped").n + p.at("guard_truncated").n == static_cast<double>(v.iterations) && !p.at("guard_triggered").b && jr.at("operators").a.size() == 8;
            }
            check(vals && units, "per-run values parse back bit-exactly (objective %.17g, counters, status, stop reason); minutes = seconds / 60 (D10)");
            check(sols, "best solutions (truck route, drone trips) parse back identical to the in-memory solutions");
            check(guard, "pass outcomes: completed + skipped + guard_truncated = iterations, guard_triggered false at the defaults, 8 operator rows");
            double mn = 1e300, mx = 0, sum = 0;
            for (const auto& R : X.datasets[0].runs) { mn = std::min(mn, R.vns.best_objective); mx = std::max(mx, R.vns.best_objective); sum += R.vns.best_objective; }
            const Json& sm = d0.at("summary");
            check(std::strtod(sm.at("best_objective_s").raw.c_str(), nullptr) == mn && std::strtod(sm.at("worst_best_objective_s").raw.c_str(), nullptr) == mx && std::fabs(sm.at("mean_best_objective_s").n - sum / 3.0) <= 1e-9 && sm.at("runs_ok").n == 3,
                  "dataset summary: min / mean / max of the best objectives");
            const double mean3 = sum / 3.0;
            double ss3 = 0.0;
            std::size_t arg = 0;
            for (std::size_t k = 0; k < 3; ++k) { const double v = X.datasets[0].runs[k].vns.best_objective; ss3 += (v - mean3) * (v - mean3); if (v == mn) { arg = k; break; } }
            ss3 = 0.0;
            for (const auto& R : X.datasets[0].runs) ss3 += (R.vns.best_objective - mean3) * (R.vns.best_objective - mean3);
            check(ss3 > 0.0, "the three runs end at different best objectives (the standard-deviation check is not vacuous)");
            check(std::fabs(sm.at("std_best_objective_s").n - std::sqrt(ss3 / 2.0)) <= 1e-9 * std::max(1.0, std::sqrt(ss3 / 2.0)) && sm.at("best_run_index").n == static_cast<double>(arg) &&
                  std::fabs(sm.at("mean_improvement_pct").n - (100.0 * (X.datasets[0].initial_objective - mean3) / X.datasets[0].initial_objective)) <= 1e-9,
                  "dataset summary: sample standard deviation (n - 1), index of the best run, mean improvement in percent");
        }
        // CSV
        const auto rows = split(runs_csv(X), '\n');
        const auto head = split(rows[0], ',');
        bool shape = rows.size() == 5 && rows.back().empty();  // header + 3 rows + trailing empty
        for (std::size_t k = 1; k <= 3 && shape; ++k) shape = split(rows[k], ',').size() == head.size();
        const auto c1 = split(rows[1], ',');
        check(shape && c1[0] == "0" && c1[1] == "0" && std::strtod(c1[6].c_str(), nullptr) == X.datasets[0].runs[0].vns.best_objective && c1[2] == "ok" && c1[3] == "iter-max", "runs.csv: header, one row per run, every row has all columns, values equal the result");
        const auto tr = split(traces_csv(X), '\n');
        std::size_t total_iters = 0;
        for (const auto& R : X.datasets[0].runs) total_iters += R.vns.iterations;
        check(tr.size() == total_iters + 2 && tr[0].find("pass_end") != std::string::npos && tr[1].find("completed") != std::string::npos, "traces.csv: one row per pass of every run, pass outcome included");
        Json t, m;
        const bool tp = parse_json(timing_json(X), t);
        check(tp && t.at("runs").a.size() == 3 && t.at("total_seconds").t == Json::Num, "timing.json is well-formed");
        (void)m;
    }

    // ------------------------------------------------------------------ the documented schema is exactly what is written (docs/static/output_schema.md)
    {
        ExperimentConfig cfg = small_cfg(10, 80, 2);
        cfg.output.record_trace = true;
        const ExperimentResult X = run_experiment(cfg);
        auto keys = [](const Json& j) { std::string k; for (const auto& kv : j.o) k += (k.empty() ? "" : ",") + kv.first; return k; };
        Json r, tm, mf;
        const bool ok = parse_json(results_json(X), r) && parse_json(timing_json(X), tm) && parse_json(manifest_json(X, render_outputs(X)), mf);
        check(ok, "schema fixture parsed");
        if (ok) {
            const Json& d = r.at("datasets").a[0];
            const Json& run = d.at("runs").a[0];
            check(keys(r) == "schema,tool,units,experiment,datasets" && keys(r.at("units")) == "time,coordinates,objective" &&
                  keys(r.at("experiment")) == "name,runs_per_dataset,num_datasets,first_dataset_index,first_run_index,base_seed,config_digest,deterministic,ok,errors", "results.json: top level, units and experiment keys are exactly the documented ones, in order");
            check(keys(d) == "dataset_index,instance_seed,num_customers,num_flp_candidates,initial,nodes,runs,summary" && keys(d.at("initial")) == "feasible,objective_s,objective_min,num_trips,diagnostics,solution" &&
                  keys(d.at("initial").at("solution")) == "truck_route,drone_trips" && keys(d.at("initial").at("solution").at("drone_trips").a[0]) == "drone,launch,customer,recovery" &&
                  keys(d.at("summary")) == "runs_ok,best_objective_s,best_objective_min,best_run_index,mean_best_objective_s,mean_best_objective_min,worst_best_objective_s,worst_best_objective_min,std_best_objective_s,mean_improvement_pct", "dataset, initial, solution, trip and summary keys");
            check(keys(run) == "run_index,status,stop,verified,initial_objective_s,initial_objective_min,best_objective_s,best_objective_min,final_current_objective_s,final_current_objective_min,improvement_pct,iterations,t0,final_temperature,best_updates,last_best_iteration,improving_moves,sa_accepted_moves,passes,shakes,operators,num_trips,best_solution" &&
                  keys(run.at("passes")) == "completed,skipped,guard_truncated,guard_triggered,local_steps_total,max_local_steps_in_pass" && keys(run.at("shakes")) == "applied,not_applicable_redraws,infeasible_redraws,applied_by_kind" &&
                  keys(run.at("shakes").at("applied_by_kind")) == "reverse,exchange,shift,drone_remove" && keys(run.at("operators").a[0]) == "op,calls,no_anchor,no_candidate,improving,sa_accepted,sa_rejected", "run, passes, shakes and operator-row keys");
            bool flp = false, cust = false, dep = false;
            for (const auto& n : d.at("nodes").a) {
                if (n.at("type").s == "flp") { flp = true; if (keys(n) != "id,type,x_m,y_m,arc_from,arc_to,position_ratio") flp = false; }
                if (n.at("type").s == "customer") { cust = true; if (keys(n) != "id,type,x_m,y_m") cust = false; }
                if (n.at("type").s == "depot") dep = keys(n) == "id,type,x_m,y_m";
            }
            check(flp && cust && dep, "node rows: depot / customer (id,type,x_m,y_m) and flp (+ arc_from, arc_to, position_ratio)");
                        check(keys(tm) == "schema,note,total_seconds,runs" && keys(tm.at("runs").a[0]) == "dataset_index,run_index,seconds" && keys(mf) == "schema,tool,experiment,deterministic,ok,files" && keys(mf.at("files").a[0]) == "name,bytes,fnv1a64" && tm.at("schema").s == "tdr.static.timing/1" && mf.at("schema").s == "tdr.static.manifest/1", "timing.json and manifest.json keys and schema tags");
            const auto head = split(runs_csv(X), '\n')[0], thead = split(traces_csv(X), '\n')[0];
            check(head == "dataset_index,run_index,status,stop,verified,initial_objective_s,best_objective_s,best_objective_min,improvement_pct,iterations,passes_completed,passes_skipped,passes_guard_truncated,guard_triggered,shakes_applied,shakes_not_applicable,shakes_infeasible,improving_moves,sa_accepted_moves,best_updates,last_best_iteration,t0,final_temperature,num_trips" &&
                  thead == "dataset_index,run_index,iter,temperature,shaken,shake_kind,pass_end,local_steps,moves,current_objective_s,best_objective_s", "runs.csv and traces.csv headers");
            // interpretation: the node table is complete and agrees with the generator
            Pipe P(cfg, 0, default_factory);
            std::map<long long, const Json*> byid;
            for (const auto& n : d.at("nodes").a) byid[static_cast<long long>(n.at("id").n)] = &n;
            bool complete = P.ok && byid.count(0) && byid.size() >= 11, coords = P.ok, refs = true;
            if (P.ok) {
                for (auto c : P.inst.customers) {
                    const auto it = byid.find(c);
                    complete = complete && it != byid.end() && it->second->at("type").s == "customer";
                    coords = coords && it != byid.end() && it->second->at("x_m").n == P.inst.nodes.at(c).coordinates.x && it->second->at("y_m").n == P.inst.nodes.at(c).coordinates.y;
                }
                auto check_sol = [&](const Json& sol) {
                    for (const auto& id : sol.at("truck_route").a) refs = refs && byid.count(static_cast<long long>(id.n)) > 0;
                    for (const auto& t : sol.at("drone_trips").a) {
                        refs = refs && byid.count(static_cast<long long>(t.at("launch").n)) && byid.at(static_cast<long long>(t.at("launch").n))->at("type").s == "flp" && byid.count(static_cast<long long>(t.at("customer").n)) && byid.count(static_cast<long long>(t.at("recovery").n));
                        const auto& fl = *byid.at(static_cast<long long>(t.at("launch").n));
                        const auto& fd = P.init.flp_data.at(static_cast<model::NodeId>(t.at("launch").n));
                        refs = refs && fl.at("arc_from").n == fd.arc_from && fl.at("arc_to").n == fd.arc_to && fl.at("position_ratio").n == fd.position_ratio && fl.at("x_m").n == P.init.node_registry.at(fd.node_id).coordinates.x;
                    }
                };
                check_sol(d.at("initial").at("solution"));
                for (const auto& rr : d.at("runs").a) check_sol(rr.at("best_solution"));
            }
            check(complete && coords && refs, "interpretation: the node table holds the depot, every customer (coordinates equal the generator's) and every FLP any written solution uses (arc and ratio equal the pool's); every id in a written solution resolves");
        }
    }

    // ------------------------------------------------------------------ the A38 guard is visible end to end
    {
        ExperimentConfig cfg = small_cfg(10, 40, 1);
        cfg.vns.max_local_steps = 1;
        const ExperimentResult X = run_experiment(cfg);
        Json r;
        parse_json(results_json(X), r);
        const Json& p = r.at("datasets").a[0].at("runs").a[0].at("passes");
        check(X.ok && p.at("guard_triggered").b && p.at("guard_truncated").n > 0 && p.at("max_local_steps_in_pass").n == 1 && p.at("completed").n + p.at("skipped").n + p.at("guard_truncated").n == 40,
              "guard-truncated passes are reported in results.json (guard_triggered, guard_truncated, max_local_steps_in_pass) -- never silent");
        const auto lines = split(runs_csv(X), '\n');
        check(lines[1].find(",true,") != std::string::npos, "...and in runs.csv");
    }

    // ------------------------------------------------------------------ digest known answers
    check(fnv1a64_hex("") == "cbf29ce484222325" && fnv1a64_hex("a") == "af63dc4c8601ec8c" && fnv1a64_hex("foobar") == "85944171f73967e8", "FNV-1a 64 matches the published test vectors (\"\", \"a\", \"foobar\")");

    // ------------------------------------------------------------------ JSON writer
    {
        JsonWriter j;
        j.begin_object();
        j.field("s", "a\"b\\c\nd\te\x01");
        j.field("nan", std::nan(""));
        j.field("big", static_cast<unsigned long long>(18446744073709551615ULL));
        j.field("neg", -5);
        j.field("pi", 3.141592653589793);
        j.key("arr"); j.begin_array(true); j.value(1); j.value(2); j.end_array();
        j.key("empty"); j.begin_array(); j.end_array();
        j.key("nested"); j.begin_object(); j.key("x"); j.begin_object(); j.field("t", true); j.end_object(); j.end_object();
        j.end_object();
        Json r;
        const bool ok = parse_json(j.finish(), r);
        check(ok && r.at("s").s == std::string("a\"b\\c\nd\te\x01") && r.at("nan").t == Json::Null && r.at("big").raw == "18446744073709551615" && r.at("neg").n == -5 && r.at("pi").n == 3.141592653589793 &&
              r.at("arr").a.size() == 2 && r.at("empty").a.empty() && r.at("nested").at("x").at("t").b, "JsonWriter: escaping, non-finite -> null, 64-bit integers, round-trip doubles, inline arrays, nesting");
    }

    // ------------------------------------------------------------------ writer: files, manifest, overwrite protection
    {
        ExperimentConfig cfg = small_cfg(10, 40, 2);
        cfg.output.record_trace = true;
        const ExperimentResult X = run_experiment(cfg);
        const auto dir = fresh_dir("writer");
        const WriteResult W = write_experiment(dir.string(), X, false);
        check(W.ok && W.files.size() == 6, "write_experiment writes config, results, runs, traces, timing and manifest");
        bool exist = true, notmp = true;
        for (const char* f : {"config.resolved.cfg", "results.json", "runs.csv", "traces.csv", "timing.json", "manifest.json"}) exist = exist && std::filesystem::exists(dir / f);
        for (const auto& e : std::filesystem::directory_iterator(dir)) notmp = notmp && e.path().extension() != ".tmp";
        check(exist && notmp, "all files present, no temporary files left behind");
        Json m;
        const bool mp = parse_json(slurp(dir / "manifest.json"), m);
        bool digests = mp && m.at("files").a.size() == 5;
        for (const auto& f : m.at("files").a) {
            const std::string name = f.at("name").s, content = slurp(dir / name);
            digests = digests && f.at("bytes").n == static_cast<double>(content.size()) && (name == "timing.json" ? f.at("fnv1a64").s == "not-reproducible" : f.at("fnv1a64").s == fnv1a64_hex(content));
        }
        check(digests, "manifest: sizes and FNV-1a digests match the files on disk; timing.json is marked not-reproducible");
        check(config::config_to_text(X.config) == slurp(dir / "config.resolved.cfg"), "config.resolved.cfg is the canonical configuration");
        const std::string before = slurp(dir / "results.json");
        ExperimentConfig other = cfg;
        other.base.seed.base_seed = 1;
        const WriteResult W2 = write_experiment(dir.string(), run_experiment(other), false);
        check(!W2.ok && W2.error.find("already exists") != std::string::npos && slurp(dir / "results.json") == before, "an existing results.json is not overwritten without the overwrite flag (and is left untouched)");
        const WriteResult W3 = write_experiment(dir.string(), run_experiment(other), true);
        check(W3.ok && slurp(dir / "results.json") != before, "with overwrite the results are replaced");
        ExperimentConfig nt = other;
        nt.output.record_trace = false;
        const WriteResult W4 = write_experiment(dir.string(), run_experiment(nt), true);
        check(W4.ok && !std::filesystem::exists(dir / "traces.csv"), "a stale traces.csv is removed when the new configuration records no traces");
        const WriteResult W5 = write_experiment((dir / "results.json" / "x").string(), X, false);
        check(!W5.ok && !W5.error.empty(), "an unwritable location is reported, not ignored");
        std::filesystem::remove_all(dir);
    }

    // ------------------------------------------------------------------ CLI
    {
        std::string out, err;
        check(cli({"version"}, &out) == 0 && out.find("tdr_static") != std::string::npos && cli({"help"}, &out) == 0 && out.find("usage") != std::string::npos && cli({"--help"}) == 0, "version / help");
        check(cli({"list-keys"}, &out) == 0 && split(out, '\n').size() == config::config_fields().size() + 1 && out.find("vns.iter_max = 2000") != std::string::npos, "list-keys prints every key with its default");
        check(cli({"frobnicate"}, nullptr, &err) == 2 && err.find("unknown command") != std::string::npos && cli({"check", "--bogus"}, nullptr, &err) == 2 && err.find("unknown argument") != std::string::npos && cli({"run", "--out"}, nullptr, &err) == 2 && cli({"run"}, nullptr, &err) == 2 &&
              err.find("--out") != std::string::npos, "usage errors (unknown command / flag, missing value, missing --out) -> exit 2");
        const auto dir = fresh_dir("cli");
        check(cli({"run", "--out", dir.string(), "--set", "vns.iter_max=abc"}, nullptr, &err) == 2 && err.find("vns.iter_max") != std::string::npos && !std::filesystem::exists(dir) &&
              cli({"run", "--out", dir.string(), "--set", "novalue"}, nullptr, &err) == 2 && cli({"run", "--out", dir.string(), "--set", "experiment.runs=0"}, nullptr, &err) == 2 && err.find("experiment.runs") != std::string::npos &&
              cli({"run", "--out", dir.string(), "--config", (dir / "missing.cfg").string()}, nullptr, &err) == 2 && !std::filesystem::exists(dir),
              "bad --set / invalid values / missing config -> exit 2 with the reason, nothing written");
        check(cli({"check", "--set", "instance.num_customers=20"}, &out) == 0 && out.find("configuration is valid") != std::string::npos && out.find("instance.num_customers = 20") != std::string::npos &&
              cli({"check", "--set", "vns.cooling_rate=0"}, nullptr, &err) == 2, "check validates and echoes the resolved configuration");

        // precedence: defaults < file < --set
        const auto cdir = fresh_dir("cfgfile");
        std::filesystem::create_directories(cdir);
        { std::ofstream(cdir / "e.cfg") << "[vns]\niter_max = 90\ncooling_rate = 0.9\n[experiment]\nruns = 2\n"; }
        check(cli({"print-config", "--config", (cdir / "e.cfg").string(), "--set", "vns.iter_max=70"}, &out) == 0 && out.find("vns.iter_max = 70") != std::string::npos && out.find("vns.cooling_rate = 0.90000000000000002") != std::string::npos &&
              out.find("experiment.runs = 2") != std::string::npos && out.find("vns.t_min = 1.0000000000000001e-09") != std::string::npos, "precedence: built-in defaults < --config file < --set (later --set wins)");

        // end to end, then reproduce from the echoed configuration
        const std::vector<std::string> base = {"--set", "instance.num_customers=10", "--set", "experiment.runs=2", "--set", "vns.iter_max=50", "--set", "seed.base_seed=31415", "--set", "output.record_trace=true", "--quiet"};
        std::vector<std::string> a1 = {"run", "--out", (dir / "a").string()};
        a1.insert(a1.end(), base.begin(), base.end());
        check(cli(a1, &out, &err) == 0 && out.empty() && err.empty() && std::filesystem::exists(dir / "a" / "results.json"), "run --quiet: exit 0, no console output, files written");
        check(cli({"run", "--out", (dir / "r").string(), "--config", (dir / "a" / "config.resolved.cfg").string(), "--quiet"}) == 0, "the resolved configuration of a run is accepted as a --config file");
        bool same = true;
        for (const char* f : {"config.resolved.cfg", "results.json", "runs.csv", "traces.csv"}) same = same && slurp(dir / "a" / f) == slurp(dir / "r" / f);
        check(same, "REPRODUCIBILITY: re-running from config.resolved.cfg gives byte-identical config, results.json, runs.csv and traces.csv");
        const std::string m1 = slurp(dir / "a" / "manifest.json"), m2 = slurp(dir / "r" / "manifest.json");
        Json j1, j2;
        parse_json(m1, j1); parse_json(m2, j2);
        bool md = true;
        for (std::size_t i = 0; i < j1.at("files").a.size(); ++i) md = md && j1.at("files").a[i].at("fnv1a64").s == j2.at("files").a[i].at("fnv1a64").s;
        check(md, "the manifest digests of the two runs are equal");
        check(cli(a1, nullptr, &err) == 4 && err.find("already exists") != std::string::npos, "second run into the same directory without --overwrite -> exit 4");
        std::vector<std::string> a2 = a1;
        a2.push_back("--overwrite");
        check(cli(a2) == 0, "--overwrite replaces the results");
        std::vector<std::string> a3 = {"run", "--out", (dir / "v").string(), "--set", "instance.num_customers=10", "--set", "experiment.runs=1", "--set", "vns.iter_max=10"};
        // paths with spaces, a Windows-style configuration file (BOM + CRLF), and an experiment that fails (exit code 1)
        const auto sp = fresh_dir("dir with spaces");
        std::filesystem::create_directories(sp);
        { std::ofstream f(sp / "my config.cfg", std::ios::binary); f << "\xEF\xBB\xBF[instance]\r\nnum_customers = 10\r\n[experiment]\r\nruns = 1\r\n[vns]\r\niter_max = 20\r\n"; }
        check(cli({"run", "--config", (sp / "my config.cfg").string(), "--out", (sp / "out dir" / "run 1").string(), "--quiet"}, &out, &err) == 0 && std::filesystem::exists(sp / "out dir" / "run 1" / "results.json"),
              "paths containing spaces (configuration file, nested output directory) and a CRLF + BOM configuration file work");
        bool no_tmp = true;
        for (const auto& e : std::filesystem::directory_iterator(sp / "out dir" / "run 1")) no_tmp = no_tmp && e.path().extension() != ".tmp";
        check(no_tmp && cli({"run", "--config", (sp / "my config.cfg").string(), "--out", (sp / "out dir" / "run 1").string(), "--quiet", "--overwrite"}) == 0, "replacing existing result files (rename over an existing file) works and leaves no temporary files");

        const auto fl = fresh_dir("failing");
        const int rc_fail = cli({"run", "--out", fl.string(), "--set", "instance.num_customers=10", "--set", "experiment.runs=1", "--set", "vehicle.truck_speed_mps=3e-308", "--quiet"}, &out, &err);
        Json fj, fm;
        const bool pj = parse_json(slurp(fl / "results.json"), fj), pm = parse_json(slurp(fl / "manifest.json"), fm);
        check(rc_fail == 1 && err.find("travel times are not finite") != std::string::npos, "an experiment that fails (travel times overflow to infinity) exits with code 1 and names the reason on stderr");
        check(pj && pm && !fj.at("experiment").at("ok").b && fj.at("experiment").at("errors").a.size() == 1 && !fj.at("datasets").a[0].at("initial").at("feasible").b && fj.at("datasets").a[0].at("runs").a.empty() && !fm.at("ok").b &&
              std::filesystem::exists(fl / "runs.csv") && std::filesystem::exists(fl / "config.resolved.cfg"), "...and still writes a complete, parseable record of the failure (ok = false, the error, no runs, manifest ok = false)");
        std::filesystem::remove_all(fl);
        std::filesystem::remove_all(sp);
        check(cli(a3, &out) == 0 && out.find("dataset 0 run 0 done") != std::string::npos && out.find("wrote 5 files") != std::string::npos, "without --quiet progress and a summary line are printed");
        std::filesystem::remove_all(dir);
        std::filesystem::remove_all(cdir);
    }

    std::cout << "\n" << g_passes << " passed, " << g_failures << " failed\n";
    if (g_failures == 0) std::cout << "ALL TESTS PASSED\n";
    return g_failures == 0 ? 0 : 1;
}
