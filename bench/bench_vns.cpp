// VNS pilot / benchmark for M7 (D7: parameters calibrated in pilot runs, NOT tuned to the paper's numbers). NOT part of the default build or
// of CTest (cmake -DTDR_BUILD_BENCH=ON). Fixture: base seed 31415, dataset_index 0, the initial solution of the paper-scale pipeline, window 3.
// usage: bench_vns <n> <iter_max> <runs> [endurance_multiplier=1.0] [grid=t0|cool|all|curve|default]
// For every setting `runs` independent runs (run_index 0..runs-1, same base seed): final best-so-far improvement over the initial objective
// (mean / min / max in %), wall-clock per run, and the mean per-run shake/move statistics. Settings: acceptance reference (printed f(x)
// "Incumbent" vs "ShakenState"), T0 as a fraction of f(initial), cooling rate. Nothing here is compared with the paper's Table 3.
#include "config/static_config.hpp"
#include "flp/flp_arc_index.hpp"
#include "init/initial_solution_builder.hpp"
#include "instance/static_instance.hpp"
#include "search/vns.hpp"
#include "util/rng.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace tdr::model;
using namespace tdr::search;

int main(int argc, char** argv) {
    if (argc < 4) { std::cerr << "usage: bench_vns <n> <iter_max> <runs> [endurance_multiplier [t0|cool|all]]\n"; return 2; }
    const int n = std::stoi(argv[1]);
    const std::size_t iters = static_cast<std::size_t>(std::stoul(argv[2]));
    const int runs = std::stoi(argv[3]);
    const double em = argc > 4 ? std::stod(argv[4]) : 1.0;
    const std::string grid = argc > 5 ? argv[5] : "all";
    tdr::config::StaticConfig cfg;
    cfg.seed.base_seed = 31415;
    cfg.instance.num_customers = n;
    cfg.instance.dataset_index = 0;
    cfg.vehicle.endurance_s *= em;
    const auto gen = tdr::instance::generate_static_instance(cfg);
    const auto& inst = gen.instance;
    const auto net0 = inst.make_network();
    tdr::init::InitialSolutionPolicy pol;
    pol.flp_policy.unit_division_length = cfg.flp.unit_division_length_m;
    const auto init = tdr::init::build_initial_solution(inst.depot, inst.customers, inst.customer_data, net0, pol);
    if (!init.feasible) { std::cout << "initial solution infeasible\n"; return 1; }
    const tdr::graph::SpatialNetwork net(init.node_registry, inst.truck, inst.drone);
    const tdr::flp::FlpArcIndex idx(init.flp_data);
    const SearchContext ctx{net, inst.customer_data, init.flp_data, idx, 3};
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "VNS pilot: n=" << n << " endurance x" << em << " iter_max=" << iters << " runs=" << runs << " |pool|=" << init.flp_data.size() << " initial trips=" << init.solution.drone_trips.size() << " drones=" << inst.drone.num_drones << "\n";

    struct Setting { std::string name; AcceptanceReference ref; double t0; double alpha; };
    std::vector<Setting> ss;
    if (grid == "t0" || grid == "all")
        for (auto ref : {AcceptanceReference::Incumbent, AcceptanceReference::ShakenState})
            for (double t0 : {1e-4, 1e-3, 1e-2, 1e-1})
                ss.push_back({"", ref, t0, 0.98});
    if (grid == "default") ss.push_back({"", VnsOptions{}.reference, VnsOptions{}.t0_fraction, VnsOptions{}.cooling_rate});
    if (grid == "cool" || grid == "all")
        for (auto ref : {AcceptanceReference::Incumbent, AcceptanceReference::ShakenState})
            for (double a : {0.9, 0.95, 0.995})
                ss.push_back({"", ref, 1e-2, a});
    if (grid == "curve") {
        // best-so-far gain (%) at checkpoints, mean over runs, for the two acceptance references at T0 = 1e-2, alpha = 0.98 (alpha^iter reaches the floor quickly, so the late part is greedy)
        for (auto ref : {AcceptanceReference::Incumbent, AcceptanceReference::ShakenState})
            for (double alpha : {0.95, 0.995}) {
                std::vector<std::size_t> cps;
                for (std::size_t c : {50u, 100u, 200u, 500u, 1000u, 2000u, 5000u, 10000u}) if (c <= iters) cps.push_back(c);
                std::vector<double> sum(cps.size(), 0.0);
                double secs = 0;
                for (int r = 0; r < runs; ++r) {
                    VnsOptions o;
                    o.iter_max = iters; o.t0_fraction = 1e-2; o.cooling_rate = alpha; o.reference = ref; o.run_index = static_cast<std::uint64_t>(r); o.base_seed = 31415; o.record_trace = true;
                    const auto res = run_vns(init.solution, ctx, o);
                    secs += res.seconds;
                    for (std::size_t i = 0; i < cps.size(); ++i) sum[i] += 100.0 * (res.initial_objective - res.trace[cps[i] - 1].best) / res.initial_objective;
                }
                std::cout << (ref == AcceptanceReference::Incumbent ? "Incumbent  " : "ShakenState") << " T0=1e-02 alpha=" << std::setprecision(3) << alpha << std::setprecision(2) << "  s/run " << secs / runs << "  mean best-gain %:";
                for (std::size_t i = 0; i < cps.size(); ++i) std::cout << "  @" << cps[i] << " " << sum[i] / runs;
                std::cout << "\n";
            }
        return 0;
    }
    for (const auto& s : ss) {
        std::vector<double> gains;
        double secs = 0, applied = 0, na = 0, inf = 0, skipped = 0, impm = 0, sam = 0, upd = 0, trunc = 0;
        std::array<double, 8> calls{};
        for (int r = 0; r < runs; ++r) {
            VnsOptions o;
            o.iter_max = iters;
            o.t0_fraction = s.t0;
            o.cooling_rate = s.alpha;
            o.reference = s.ref;
            o.run_index = static_cast<std::uint64_t>(r);
            o.base_seed = 31415;
            const auto res = run_vns(init.solution, ctx, o);
            gains.push_back(100.0 * (res.initial_objective - res.best_objective) / res.initial_objective);
            secs += res.seconds; applied += static_cast<double>(res.shakes_applied); na += static_cast<double>(res.shakes_not_applicable); inf += static_cast<double>(res.shakes_infeasible);
            skipped += static_cast<double>(res.iterations_skipped); impm += static_cast<double>(res.improving_moves); sam += static_cast<double>(res.sa_accepted_moves); upd += static_cast<double>(res.best_updates); trunc += static_cast<double>(res.passes_truncated);
            for (std::size_t k = 0; k < 8; ++k) calls[k] += static_cast<double>(res.ops[k].calls);
        }
        double mean = 0;
        for (double g : gains) mean += g;
        mean /= runs;
        const double R = runs;
        std::cout << (s.ref == AcceptanceReference::Incumbent ? "Incumbent  " : "ShakenState") << " T0=" << std::scientific << std::setprecision(0) << s.t0 << std::fixed << std::setprecision(3) << " alpha=" << s.alpha
                  << std::setprecision(2) << "  gain% mean " << mean << " [min " << *std::min_element(gains.begin(), gains.end()) << ", max " << *std::max_element(gains.begin(), gains.end()) << "]  s/run " << secs / R
                  << "  per run: shakes " << applied / R << " (na " << na / R << ", infeas " << inf / R << ", skipped " << skipped / R << ") improving " << impm / R << " SA-acc " << sam / R << " best-updates " << upd / R << " truncated " << trunc / R << "\n";
    }
    return 0;
}
