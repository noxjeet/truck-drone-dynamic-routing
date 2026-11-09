#pragma once

// Experiment runner for the STATIC reproduction (M8, 9 Nov 2026). Pure computation: no file or console output (the writer and the CLI
// are separate headers). For every dataset index it generates the instance, builds the initial solution (F1-F3, unchanged since
// M1), then performs `runs` independent VNS runs with run indices first_run_index.. through the unchanged `run_vns` of M7.
//
// Determinism: the result depends only on the configuration. Every VNS run takes its seeds from derive_seed(base_seed, stream,
// run_index) inside run_vns; the runner adds no randomness and runs do not influence each other (each starts from the same initial
// solution), so run r of an experiment equals the single run with first_run_index = r (tested). The only non-deterministic
// quantities are wall-clock seconds, which live in separate fields and files; vns.max_seconds > 0 makes a run itself
// non-reproducible and is flagged in the manifest.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "config/experiment_config.hpp"
#include "flp/flp_arc_index.hpp"
#include "graph/spatial_network.hpp"
#include "init/initial_solution_builder.hpp"
#include "instance/static_instance.hpp"
#include "model/solution.hpp"
#include "search/vns.hpp"
#include "timing/timing_engine.hpp"
#include "tsp/tsp_solver_factory.hpp"

namespace tdr::runner {

struct RunRecord {
    std::uint64_t run_index = 0;
    tdr::search::VnsResult vns;
    // Independent re-check of the returned best solution with the production evaluator + structural check (every customer exactly once).
    bool verified = false;
    std::string verify_note; // empty when verified
};

// A node referenced by the results: the depot, every customer, and every FLP used by the initial solution or a returned best solution
// (coordinates in metres; FLPs also carry the arc they lie on and their position ratio), so a result can be interpreted without regenerating the instance.
struct NodeRecord {
    tdr::model::NodeId id = tdr::model::kInvalidNodeId;
    std::string type; // "depot" | "customer" | "flp"
    double x = 0.0, y = 0.0;
    tdr::model::NodeId arc_from = tdr::model::kInvalidNodeId, arc_to = tdr::model::kInvalidNodeId; // flp only
    double position_ratio = 0.0;                                                                   // flp only
};

struct DatasetRecord {
    int dataset_index = 0;
    std::uint64_t instance_seed = 0;
    int num_customers = 0;
    std::size_t num_flp_candidates = 0;
    bool initial_feasible = false;
    double initial_objective = 0.0;
    tdr::model::Solution initial;
    std::vector<std::string> diagnostics;
    std::vector<NodeRecord> nodes;
    std::vector<RunRecord> runs;
};

struct ExperimentResult {
    bool ok = true;                   // false: a configuration problem, an infeasible initial solution or a run that could not start
    std::vector<std::string> errors;
    tdr::config::ExperimentConfig config; // exactly what was run
    std::vector<DatasetRecord> datasets;
    double total_seconds = 0.0;       // wall clock (non-deterministic; kept out of the deterministic outputs)
};

using ProgressFn = std::function<void(const std::string&)>;

// Every customer must be served exactly once (truck route interior or one drone trip) and the evaluator must accept the solution with
// exactly the stated objective. Returns "" when all hold.
inline std::string verify_best(const tdr::model::Solution& s, double stated_objective, const tdr::search::SearchContext& ctx,
                               const std::vector<tdr::model::NodeId>& customers) {
    std::unordered_map<tdr::model::NodeId, int> count;
    for (auto id : s.truck_route) if (ctx.nodes().contains(id) && ctx.nodes().at(id).type == tdr::model::NodeType::Customer) ++count[id];
    for (const auto& t : s.drone_trips) ++count[t.service_customer];
    for (auto c : customers) {
        const auto it = count.find(c);
        if (it == count.end() || it->second != 1) return "customer " + std::to_string(c) + " is not served exactly once";
    }
    if (count.size() != customers.size()) return "a non-customer node is served";
    const auto e = tdr::timing::evaluate_solution(s, ctx.network, ctx.customers, ctx.flp_data);
    const auto v = tdr::search::classify_evaluation(e, ctx.num_drones());
    if (!v.feasible) return "the evaluator rejects the returned best solution";
    if (v.objective != stated_objective) return "evaluator objective differs from the stated objective";
    return "";
}

// Fills R.verified / R.verify_note: a run that did not start is not verified (its message is the note); otherwise verify_best decides.
inline void verify_record(RunRecord& R, const tdr::search::SearchContext& ctx, const std::vector<tdr::model::NodeId>& customers) {
    if (R.vns.status != tdr::search::VnsStatus::Ok) { R.verified = false; R.verify_note = R.vns.message.empty() ? "the VNS run did not start" : R.vns.message; return; }
    R.verify_note = verify_best(R.vns.best, R.vns.best_objective, ctx, customers);
    R.verified = R.verify_note.empty();
}

inline DatasetRecord run_dataset(const tdr::config::ExperimentConfig& cfg, int dataset_index, std::vector<std::string>& errors, const ProgressFn& progress = nullptr) {
    DatasetRecord D;
    D.dataset_index = dataset_index;
    tdr::config::StaticConfig sc = cfg.base;
    sc.instance.dataset_index = dataset_index;
    const auto gen = tdr::instance::generate_static_instance(sc);
    if (!gen.ok) {
        for (const auto& m : gen.errors) errors.push_back("dataset " + std::to_string(dataset_index) + ": " + m);
        return D;
    }
    const auto& inst = gen.instance;
    D.instance_seed = inst.instance_seed;
    D.num_customers = static_cast<int>(inst.customers.size());

    // Preflight (runner-level input check, no algorithm change): extreme vehicle speeds can make travel times overflow to infinity, which
    // the Stage-1 heuristic TSP does not guard against (it would index with -1). Such a configuration is reported as an experiment failure.
    {
        const auto probe = inst.make_network();
        std::vector<tdr::model::NodeId> ids{inst.depot};
        ids.insert(ids.end(), inst.customers.begin(), inst.customers.end());
        bool finite = true;
        for (auto x : ids) for (auto y : ids) finite = finite && std::isfinite(probe.truck_time(x, y)) && std::isfinite(probe.drone_time(x, y));
        if (!finite) {
            D.diagnostics.push_back("preflight: a truck or drone travel time is not finite (vehicle speed too small for the instance size)");
            errors.push_back("dataset " + std::to_string(dataset_index) + ": travel times are not finite; check vehicle.truck_speed_mps and vehicle.drone_speed_mps");
            return D;
        }
    }

    tdr::init::InitialSolutionPolicy pol;
    pol.kmeans_policy = cfg.initial.kmeans;
    pol.flp_policy.unit_division_length = sc.flp.unit_division_length_m;
    const tdr::tsp::TSPSolverFactory factory(cfg.initial.tsp_max_exact_n, cfg.initial.tsp_heuristic_restarts, cfg.initial.tsp_heuristic_seed);
    const auto net0 = inst.make_network();
    const auto init = tdr::init::build_initial_solution(inst.depot, inst.customers, inst.customer_data, net0, pol, factory);
    D.diagnostics = init.diagnostics;
    D.num_flp_candidates = init.flp_data.size();
    D.initial_feasible = init.feasible;
    D.initial_objective = init.objective_value;
    D.initial = init.solution;
    if (!init.feasible) {
        errors.push_back("dataset " + std::to_string(dataset_index) + ": the initial solution is not feasible");
        return D;
    }
    const tdr::graph::SpatialNetwork net(init.node_registry, inst.truck, inst.drone);
    const tdr::flp::FlpArcIndex idx(init.flp_data);
    const tdr::search::SearchContext ctx{net, inst.customer_data, init.flp_data, idx, sc.neighbourhood.adjacency_window};

    const tdr::search::VnsOptions base_opts = tdr::config::effective_vns_options(cfg);
    for (int r = 0; r < cfg.experiment.runs; ++r) {
        tdr::search::VnsOptions o = base_opts;
        o.run_index = cfg.experiment.first_run_index + static_cast<std::uint64_t>(r);
        RunRecord R;
        R.run_index = o.run_index;
        R.vns = tdr::search::run_vns(init.solution, ctx, o);
        verify_record(R, ctx, inst.customers);
        if (!R.verified) errors.push_back("dataset " + std::to_string(dataset_index) + " run " + std::to_string(o.run_index) + ": " + (R.vns.status != tdr::search::VnsStatus::Ok ? R.verify_note : "verification failed: " + R.verify_note));
        D.runs.push_back(std::move(R));
        if (progress) progress("dataset " + std::to_string(dataset_index) + " run " + std::to_string(o.run_index) + " done");
    }
    // node table for interpretation (after the runs: includes the FLPs the returned solutions use)
    std::vector<tdr::model::NodeId> flp_ids;
    auto note = [&](const tdr::model::Solution& sol) { for (const auto& t : sol.drone_trips) flp_ids.push_back(t.launch_node); };
    note(D.initial);
    for (const auto& R : D.runs) if (R.vns.status == tdr::search::VnsStatus::Ok) { note(R.vns.best); note(R.vns.current); }
    std::sort(flp_ids.begin(), flp_ids.end());
    flp_ids.erase(std::unique(flp_ids.begin(), flp_ids.end()), flp_ids.end());
    auto add = [&](tdr::model::NodeId id, const char* type) {
        NodeRecord n;
        n.id = id; n.type = type;
        const auto& nd = net.nodes().at(id);
        n.x = nd.coordinates.x; n.y = nd.coordinates.y;
        if (std::string(type) == "flp") {
            const auto& f = init.flp_data.at(id);
            n.arc_from = f.arc_from; n.arc_to = f.arc_to; n.position_ratio = f.position_ratio;
        }
        D.nodes.push_back(n);
    };
    add(inst.depot, "depot");
    for (auto c : inst.customers) add(c, "customer");
    for (auto f : flp_ids) add(f, "flp");
    return D;
}

inline ExperimentResult run_experiment(const tdr::config::ExperimentConfig& cfg, const ProgressFn& progress = nullptr) {
    ExperimentResult X;
    X.config = cfg;
    X.errors = tdr::config::validate(cfg);
    if (!X.errors.empty()) { X.ok = false; return X; }
    const auto t0 = std::chrono::steady_clock::now();
    try {
        for (int d = 0; d < cfg.experiment.num_datasets; ++d)
            X.datasets.push_back(run_dataset(cfg, cfg.base.instance.dataset_index + d, X.errors, progress));
    } catch (const std::exception& e) {
        X.errors.push_back(std::string("unexpected exception: ") + e.what());
    }
    X.total_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    X.ok = X.errors.empty();
    return X;
}

} // namespace tdr::runner
