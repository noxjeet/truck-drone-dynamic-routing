// Evaluator micro-benchmark for the M1 performance follow-up (P1/P2).
// NOT part of the default build or of CTest (cmake -DTDR_BUILD_BENCH=ON).
//
// Variants (all evaluate the SAME instance, solution and CustomerData):
//   A  reference, full registry   : frozen evaluator from cc37e49 on the full eager-FLP-pool network
//   B  reference, used-node regs. : same frozen evaluator, network registry restricted to the nodes the
//                                   solution touches (depot, customers, FLPs on the route / launch nodes)
//   C  P1 only                    : evaluator after P1 (customer-id cache), BEFORE P2   [optional, needs
//                                   -DTDR_BENCH_P1_HEADER=<path of the saved P1-only header, namespace tdr::timing_p1>]
//   D  P1 + P2 (production)       : include/timing/timing_engine.hpp as it is now
//
// Method: before timing, every variant is checked to give the same verdict and an objective equal to A's
// (A, B, C bitwise; D within the 2*n*eps*sum|t| bound). Timing: W warm-up calls, then R repetitions; in each
// repetition every variant runs one batch of B calls (round-robin over the variants, so slow drift hits all
// of them alike). The batch size is calibrated once per variant to ~kBatchMs. Reported: per-call median over
// the R repetitions, min/max, and the relative spread (max-min)/median. Wall clock via steady_clock.

#include "config/static_config.hpp"
#include "init/initial_solution_builder.hpp"
#include "instance/static_instance.hpp"
#include "reference/timing_engine_reference.hpp"
#include "timing/timing_engine.hpp"
#ifdef TDR_BENCH_P1_HEADER
#include TDR_BENCH_P1_HEADER
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <utility>
#include <string>
#include <vector>

using namespace tdr::model;
using Clock = std::chrono::steady_clock;

namespace {

constexpr int kReps = 15;
constexpr double kBatchMs = 15.0;
volatile double g_sink = 0.0;

struct Variant {
    Variant(std::string n, std::function<double()> f) : name(std::move(n)), run(std::move(f)) {}
    std::string name;
    std::function<double()> run; // one evaluation; returns the objective
    int batch = 1;
    std::vector<double> per_call_us;
};

double now_us() { return std::chrono::duration<double, std::micro>(Clock::now().time_since_epoch()).count(); }

} // namespace

int main(int argc, char** argv) {
    const std::uint64_t base_seed = 31415;
    std::vector<int> sizes = {10, 20, 50};
    int seeds_per_size = 3;
    if (argc > 1) seeds_per_size = std::atoi(argv[1]);

    std::cout << "evaluator benchmark: base seed " << base_seed << ", " << seeds_per_size << " instances per size, " << kReps
              << " repetitions, ~" << kBatchMs << " ms batches\n"
              << "columns: median us/call [min .. max] (rel. spread)  speedup vs A\n";
    bool all_equivalent = true;

    for (int n : sizes) {
        for (int k = 0; k < seeds_per_size; ++k) {
            tdr::config::StaticConfig cfg;
            cfg.seed.base_seed = base_seed;
            cfg.instance.num_customers = n;
            cfg.instance.dataset_index = 5000 + n * 10 + k;
            cfg.vehicle.num_drones = 2;
            const auto gen = tdr::instance::generate_static_instance(cfg);
            const auto& inst = gen.instance;
            tdr::init::InitialSolutionPolicy pol;
            pol.flp_policy.unit_division_length = cfg.flp.unit_division_length_m;
            const auto res = tdr::init::build_initial_solution(inst.depot, inst.customers, inst.customer_data, inst.make_network(), pol);

            const tdr::graph::SpatialNetwork full(res.node_registry, inst.truck, inst.drone);
            NodeRegistry used;
            {
                std::vector<NodeId> keep(res.solution.truck_route.begin(), res.solution.truck_route.end());
                for (const auto& t : res.solution.drone_trips) { keep.push_back(t.launch_node); keep.push_back(t.recovery_node); keep.push_back(t.service_customer); }
                for (const auto& kv : res.node_registry.all()) if (kv.second.type != NodeType::FlexibleLaunchPoint) used.add(kv.second);
                for (NodeId id : keep) if (!used.contains(id)) used.add(res.node_registry.at(id));
            }
            const tdr::graph::SpatialNetwork used_net(used, inst.truck, inst.drone);

            const Solution& sol = res.solution;
            Solution all_truck;
            all_truck.truck_route = res.truck_only_route;
            const Solution* const candidates[2] = {&sol, &all_truck};
            for (const Solution* sp : candidates) {
                const bool is_all_truck = (sp == &all_truck);
                const Solution& s = *sp;
                std::vector<Variant> vs;
                vs.reserve(4);
                vs.emplace_back("A ref/full-registry", [&] { return tdr::timing_ref::evaluate_solution(s, full, inst.customer_data, res.flp_data).objective_value; });
                vs.emplace_back("B ref/used-nodes", [&] { return tdr::timing_ref::evaluate_solution(s, used_net, inst.customer_data, res.flp_data).objective_value; });
#ifdef TDR_BENCH_P1_HEADER
                vs.emplace_back("C P1 only", [&] { return tdr::timing_p1::evaluate_solution(s, full, inst.customer_data, res.flp_data).objective_value; });
#endif
                vs.emplace_back("D P1+P2 (prod)", [&] { return tdr::timing::evaluate_solution(s, full, inst.customer_data, res.flp_data).objective_value; });

                // equivalence first
                const auto ra = tdr::timing_ref::evaluate_solution(s, full, inst.customer_data, res.flp_data);
                const auto rb = tdr::timing_ref::evaluate_solution(s, used_net, inst.customer_data, res.flp_data);
                const auto rd = tdr::timing::evaluate_solution(s, full, inst.customer_data, res.flp_data);
                bool eq = ra.feasible == rb.feasible && ra.feasible == rd.feasible && ra.objective_value == rb.objective_value;
                double sum_abs = 0.0;
                for (const auto& rec : ra.customer_records) sum_abs += std::fabs(rec.service_arrival_time);
                eq = eq && std::fabs(rd.objective_value - ra.objective_value) <= 2.0 * static_cast<double>(n) * std::numeric_limits<double>::epsilon() * sum_abs;
#ifdef TDR_BENCH_P1_HEADER
                const auto rc = tdr::timing_p1::evaluate_solution(s, full, inst.customer_data, res.flp_data);
                eq = eq && rc.feasible == ra.feasible && rc.objective_value == ra.objective_value;
#endif
                all_equivalent = all_equivalent && eq;

                // calibrate + warm up
                for (auto& v : vs) {
                    for (int i = 0; i < 50; ++i) g_sink = g_sink + v.run();
                    const double t0 = now_us();
                    constexpr int kProbe = 20;
                    for (int i = 0; i < kProbe; ++i) g_sink = g_sink + v.run();
                    const double per = (now_us() - t0) / kProbe;
                    v.batch = std::max(1, static_cast<int>(kBatchMs * 1000.0 / std::max(per, 0.05)));
                }
                for (int rep = 0; rep < kReps; ++rep)
                    for (auto& v : vs) {
                        const double t0 = now_us();
                        for (int i = 0; i < v.batch; ++i) g_sink = g_sink + v.run();
                        v.per_call_us.push_back((now_us() - t0) / v.batch);
                    }

                std::cout << "n=" << std::setw(2) << n << " inst " << k << (is_all_truck ? " all-truck   " : " initial-sol  ") << "feas=" << ra.feasible
                          << " trips=" << s.drone_trips.size() << " |V|=" << res.node_registry.size() << " equivalent=" << (eq ? "yes" : "NO") << "\n";
                double base = 0.0;
                for (auto& v : vs) {
                    std::sort(v.per_call_us.begin(), v.per_call_us.end());
                    const double med = v.per_call_us[v.per_call_us.size() / 2];
                    if (&v == &vs.front()) base = med;
                    std::cout << "    " << std::left << std::setw(22) << v.name << std::right << std::fixed << std::setprecision(2) << std::setw(9) << med << " us  ["
                              << v.per_call_us.front() << " .. " << v.per_call_us.back() << "] (" << std::setprecision(1)
                              << 100.0 * (v.per_call_us.back() - v.per_call_us.front()) / med << "%)  x" << std::setprecision(1) << base / med << "\n";
                }
            }
        }
    }
    std::cout << (all_equivalent ? "ALL VARIANTS EQUIVALENT\n" : "EQUIVALENCE CHECK FAILED\n");
    return all_equivalent ? 0 : 1;
}
