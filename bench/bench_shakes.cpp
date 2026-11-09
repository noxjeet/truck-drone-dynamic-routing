// Shake micro-benchmark for M6 (Random_Reverse, Random_Exchange, Random_Shift, Random_Drone_Remove). NOT part of the default build or of
// CTest (cmake -DTDR_BUILD_BENCH=ON). Same fixtures as bench_operators / bench_n1_n2: base seed 31415, dataset_index 0, initial solution,
// window 3. For each shake: cost per call with evaluate_result=false (construction: permutation, FLP follow, dissolution, reinsertion)
// and with evaluate_result=true (adds the evaluator verdict on the result; one base evaluation per call is always made to establish
// that the input is feasible), and OUTCOME STATISTICS over 4000 seeded draws: feasible rate of the shaken solution, trips dissolved
// by reason, trips removed, fallback reinsertions, objective change. Usage: bench_shakes [endurance_multiplier=1.0 [n|all]]
#include "config/static_config.hpp"
#include "flp/flp_arc_index.hpp"
#include "init/initial_solution_builder.hpp"
#include "instance/static_instance.hpp"
#include "search/shakes.hpp"
#include "timing/timing_engine.hpp"
#include "util/rng.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <new>
#include <string>
#include <vector>

using namespace tdr::model;
using namespace tdr::search;
using Clock = std::chrono::steady_clock;

namespace {
constexpr int kReps = 15;
constexpr double kBatchMs = 20.0;
constexpr int kDraws = 4000;
volatile std::size_t g_sink = 0;
std::size_t g_allocs = 0;
double now_us() { return std::chrono::duration<double, std::micro>(Clock::now().time_since_epoch()).count(); }

template <class F>
std::vector<double> time_it(F f, int& batch_out) {
    int b = 1;
    for (;;) {
        const double t0 = now_us();
        for (int i = 0; i < b; ++i) f(i);
        if (now_us() - t0 >= kBatchMs * 1000.0 || b > (1 << 20)) break;
        b *= 2;
    }
    for (int i = 0; i < 3; ++i) f(i);
    std::vector<double> us;
    for (int r = 0; r < kReps; ++r) {
        const double t0 = now_us();
        for (int i = 0; i < b; ++i) f(i);
        us.push_back((now_us() - t0) / b);
    }
    batch_out = b;
    std::sort(us.begin(), us.end());
    return us;
}
} // namespace

__attribute__((noinline)) static void* raw_alloc(std::size_t n) { return std::malloc(n); }
__attribute__((noinline)) static void raw_free(void* p) noexcept { std::free(p); }
void* operator new(std::size_t n) { ++g_allocs; if (void* p = raw_alloc(n)) return p; throw std::bad_alloc(); }
void operator delete(void* p) noexcept { raw_free(p); }
void operator delete(void* p, std::size_t) noexcept { raw_free(p); }

int main(int argc, char** argv) {
    const double endurance_mult = argc > 1 ? std::stod(argv[1]) : 1.0;
    const std::vector<int> sizes = (argc > 2 && std::string(argv[2]) != "all") ? std::vector<int>{std::stoi(argv[2])} : std::vector<int>{10, 20, 50};
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Shake benchmark, endurance x" << endurance_mult << ", defaults L_max = D_max = 3, window=3, seeds: base 31415, dataset_index 0; " << kReps << " reps, batch >= " << kBatchMs << " ms, " << kDraws << " outcome draws\n\n";
    const char* names[4] = {"Random_Reverse", "Random_Exchange", "Random_Shift", "Random_Drone_Remove"};
    for (int n : sizes) {
        tdr::config::StaticConfig cfg;
        cfg.seed.base_seed = 31415;
        cfg.instance.num_customers = n;
        cfg.instance.dataset_index = 0;
        cfg.vehicle.endurance_s *= endurance_mult;
        const auto gen = tdr::instance::generate_static_instance(cfg);
        const auto& inst = gen.instance;
        const auto net0 = inst.make_network();
        tdr::init::InitialSolutionPolicy pol;
        pol.flp_policy.unit_division_length = cfg.flp.unit_division_length_m;
        const auto init = tdr::init::build_initial_solution(inst.depot, inst.customers, inst.customer_data, net0, pol);
        if (!init.feasible) { std::cout << "n=" << n << ": initial solution infeasible, skipped\n"; continue; }
        const tdr::graph::SpatialNetwork net(init.node_registry, inst.truck, inst.drone);
        const tdr::flp::FlpArcIndex idx(init.flp_data);
        const SearchContext ctx{net, inst.customer_data, init.flp_data, idx, 3};
        const Solution& base = init.solution;
        std::size_t flps_on_route = 0;
        for (NodeId v : base.truck_route) flps_on_route += init.node_registry.at(v).type == NodeType::FlexibleLaunchPoint;
        std::cout << "n=" << n << "  |pool|=" << init.flp_data.size() << "  trips=" << base.drone_trips.size() << "  route FLPs=" << flps_on_route << "  route length=" << base.truck_route.size() << "\n";
        int b0 = 0;
        const auto ev = time_it([&](int) { const auto e = tdr::timing::evaluate_solution(base, net, inst.customer_data, init.flp_data); g_sink = g_sink + static_cast<std::size_t>(e.feasible); }, b0);
        std::cout << "  base evaluate_solution (1 call)       " << std::setw(10) << ev[ev.size() / 2] << " us\n";
        ShakeOptions og, of;
        og.evaluate_result = false;
        of.evaluate_result = true;
        for (int k = 0; k < 4; ++k) {
            const auto run = [&](std::uint64_t seed, const ShakeOptions& o) {
                tdr::util::Rng rng(seed);
                switch (k) {
                    case 0: return shake_random_reverse(base, ctx, rng, o);
                    case 1: return shake_random_exchange(base, ctx, rng, o);
                    case 2: return shake_random_shift(base, ctx, rng, o);
                    default: return shake_random_drone_remove(base, ctx, rng, o);
                }
            };
            int bg = 0, bf = 0;
            const auto tg = time_it([&](int i) { const auto r = run(static_cast<std::uint64_t>(i) + 1, og); g_sink = g_sink + r.solution.truck_route.size(); }, bg);
            const auto tf = time_it([&](int i) { const auto r = run(static_cast<std::uint64_t>(i) + 1, of); g_sink = g_sink + r.solution.truck_route.size(); }, bf);
            g_allocs = 0;
            for (int i = 0; i < 200; ++i) { const auto r = run(static_cast<std::uint64_t>(i) + 1, of); g_sink = g_sink + r.solution.truck_route.size(); }
            const double allocs = static_cast<double>(g_allocs) / 200.0;
            long ok = 0, na = 0, feas = 0, infeas = 0, endur = 0, imp = 0, none_d = 0, dis[3] = {0, 0, 0}, rem = 0, fb = 0, reins = 0, trips_left = 0, lost = 0, drops = 0;
            double dsum = 0;
            for (int i = 0; i < kDraws; ++i) {
                const auto r = run(static_cast<std::uint64_t>(i) + 100000, of);
                if (r.status == ShakeStatus::NotApplicable) { ++na; continue; }
                ++ok;
                if (r.feasible) { ++feas; dsum += r.objective - r.base_objective; imp += r.objective < r.base_objective - 1e-9; }
                else { ++infeas; endur += r.reason == RejectReason::EvalEndurance; }
                for (const auto& d : r.dissolved) ++dis[static_cast<int>(d.reason)];
                rem += static_cast<long>(r.removed.size());
                none_d += r.dissolved.empty() && r.removed.empty();
                for (const auto& q : r.reinsertions) { ++reins; fb += q.fallback; }
                trips_left += static_cast<long>(r.solution.drone_trips.size());
                lost += static_cast<long>(r.flps_lost_with_arcs.size());
                drops += static_cast<long>(r.flps_dropped.size());
            }
            std::cout << "  " << std::left << std::setw(20) << names[k] << std::right << " gen " << std::setw(9) << tg[tg.size() / 2] << " us   full " << std::setw(9) << tf[tf.size() / 2] << " us  (eval share " << 100.0 * (tf[tf.size() / 2] - tg[tg.size() / 2]) / tf[tf.size() / 2] << " %, base evals alone " << ev[ev.size() / 2] << " us)   allocs/call " << allocs << "\n";
            if (ok == 0) { std::cout << "      not applicable in all " << na << " draws\n"; continue; }
            const double d = static_cast<double>(ok);
            std::cout << "      outcomes over " << ok << " applicable draws (" << na << " not applicable): feasible " << 100.0 * static_cast<double>(feas) / d << " % (infeasible " << infeas << ", of which endurance " << endur << "), improving among all " << 100.0 * static_cast<double>(imp) / d
                      << " %, mean objective change of feasible results " << (feas ? dsum / static_cast<double>(feas) : 0.0) << "\n"
                      << "      per shake: trips dissolved launch-arc-vanished " << static_cast<double>(dis[0]) / d << ", recovery-not-after-launch " << static_cast<double>(dis[1]) / d << ", arc-split " << static_cast<double>(dis[2]) / d
                      << ", removed " << static_cast<double>(rem) / d << ", shakes with no dissolution/removal " << 100.0 * static_cast<double>(none_d) / d << " %, reinsertions " << static_cast<double>(reins) / d << " (fallback " << fb << "), trips left " << static_cast<double>(trips_left) / d
                      << ", FLPs lost with arcs " << static_cast<double>(lost) / d << ", idle FLPs dropped " << static_cast<double>(drops) / d << "\n";
        }
        std::cout << "\n";
    }
    return static_cast<int>(g_sink == static_cast<std::size_t>(-1));
}
