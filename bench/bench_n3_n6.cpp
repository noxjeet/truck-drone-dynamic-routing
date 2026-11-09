// Operator micro-benchmark for M4 (N3, N6). NOT part of the default build or of CTest (cmake -DTDR_BUILD_BENCH=ON).
// Same methodology and fixtures as bench_operators.cpp (M2) and bench_n7_n8.cpp (M3): base seed 31415, dataset_index 0, initial
// solution, window 3, record_candidates = false. A "sweep" calls the operator once for EVERY drone trip of the base solution
// (N3: consecutive trips of the same drone; N6: adjacent other trips, same-drone ones included by default (A8 revised 9 Nov 2026), plus the labelled restricted variant include_same_drone = false).
// gen  = evaluate_candidates=false (construction + affected-trip check + free-drone search; one base evaluation per call is still made),
// full = evaluate_candidates=true. Derived: generation cost per candidate (net of the base evaluations), marginal cost per evaluated
// candidate = (full - gen)/evaluated, evaluator calls, outcomes by reason, allocations per sweep.

#include "config/static_config.hpp"
#include "flp/flp_arc_index.hpp"
#include "init/initial_solution_builder.hpp"
#include "instance/static_instance.hpp"
#include "search/neighbourhood_n3.hpp"
#include "search/neighbourhood_n6.hpp"
#include "timing/timing_engine.hpp"
#include "util/rng.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <new>
#include <cstdlib>
#include <string>
#include <vector>

using namespace tdr::model;
using namespace tdr::search;
using Clock = std::chrono::steady_clock;

namespace {
constexpr int kReps = 15;
constexpr double kBatchMs = 20.0;
volatile std::size_t g_sink = 0;

double now_us() { return std::chrono::duration<double, std::micro>(Clock::now().time_since_epoch()).count(); }

struct Counts {
    std::size_t calls = 0, generated = 0, evaluated = 0, accepted = 0, duplicates = 0;
    std::array<std::size_t, kRejectReasonCount> rej{};
};

// allocation counter (global operator new), to report allocations per sweep
std::size_t g_allocs = 0, g_alloc_bytes = 0;

struct Series {
    std::string name;
    std::vector<double> us; // per sweep
    int batch = 1;
    double median() const { auto v = us; std::sort(v.begin(), v.end()); return v[v.size() / 2]; }
    double lo() const { return *std::min_element(us.begin(), us.end()); }
    double hi() const { return *std::max_element(us.begin(), us.end()); }
};

template <class F>
void run_series(std::vector<Series>& all, std::vector<F>& fs) {
    for (std::size_t k = 0; k < all.size(); ++k) {   // calibrate the batch size
        int b = 1;
        for (;;) {
            const double t0 = now_us();
            for (int i = 0; i < b; ++i) fs[k]();
            if ((now_us() - t0) >= kBatchMs * 1000.0 || b > (1 << 20)) break;
            b *= 2;
        }
        all[k].batch = b;
        for (int i = 0; i < 3; ++i) fs[k]();         // warm-up
    }
    for (int rep = 0; rep < kReps; ++rep)
        for (std::size_t k = 0; k < all.size(); ++k) {
            const double t0 = now_us();
            for (int i = 0; i < all[k].batch; ++i) fs[k]();
            all[k].us.push_back((now_us() - t0) / all[k].batch);
        }
}
} // namespace


__attribute__((noinline)) static void* raw_alloc(std::size_t n) { return std::malloc(n); }
__attribute__((noinline)) static void raw_free(void* p) noexcept { std::free(p); }
void* operator new(std::size_t n) { ++g_allocs; g_alloc_bytes += n; if (void* p = raw_alloc(n)) return p; throw std::bad_alloc(); }
void operator delete(void* p) noexcept { raw_free(p); }
void operator delete(void* p, std::size_t) noexcept { raw_free(p); }

int main(int argc, char** argv) {
    // usage: bench_n3_n6 [endurance_multiplier=1.0 [n]]   (a multiplier > 1 only changes the instance parameter vehicle.endurance_s,
    // to exercise the accepted-candidate path when the paper's 1800 s makes almost every candidate endurance-infeasible; clearly labelled, not a paper parameter)
    // argv[3] = "alt": LABELLED VARIANT -- the base trips are re-assigned alternately to drones 0, 1, 0, 1, ... in launch order (kept only if
    // the evaluator still accepts the solution). The paper-scale initial solution flies every trip with drone 0, so N6's default (other-drone
    // partners only) has no candidate there; this variant is the only way to measure N6's candidate path. Not a paper parameter.
    const double endurance_mult = argc > 1 ? std::stod(argv[1]) : 1.0;
    const std::vector<int> sizes = (argc > 2 && std::string(argv[2]) != "all") ? std::vector<int>{std::stoi(argv[2])} : std::vector<int>{10, 20, 50};
    const bool alternate = argc > 3 && std::string(argv[3]) == "alt";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "N3/N6 operator benchmark, endurance x" << endurance_mult << (alternate ? ", ALTERNATE-DRONE base (labelled variant)" : "") << ", window=3, seeds: base 31415, dataset_index 0; " << kReps << " reps, batch >= " << kBatchMs << " ms\n\n";
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
        Solution base_store = init.solution;
        if (alternate) {
            // launch order = position of the launch FLP in the route
            std::vector<std::pair<std::size_t, std::size_t>> order;
            for (std::size_t k = 0; k < base_store.drone_trips.size(); ++k) {
                std::size_t p = 0;
                while (p < base_store.truck_route.size() && base_store.truck_route[p] != base_store.drone_trips[k].launch_node) ++p;
                order.emplace_back(p, k);
            }
            std::sort(order.begin(), order.end());
            Solution trial = base_store;
            for (std::size_t r = 0; r < order.size(); ++r) trial.drone_trips[order[r].second].drone_id = static_cast<int>(r % 2);
            if (tdr::timing::evaluate_solution(trial, net, inst.customer_data, init.flp_data).feasible) base_store = trial;
            else std::cout << "  (alternate-drone assignment is not feasible for this base; the original assignment is kept)\n";
        }
        const Solution& base = base_store;

        const std::size_t trips = base.drone_trips.size();
        std::cout << "n=" << n << "  |pool|=" << init.flp_data.size() << "  trips=" << trips << "  drones=" << inst.drone.num_drones << "\n";

        N3Options o3g, o3f; o3g.evaluate_candidates = false; o3f.evaluate_candidates = true;
        N6Options o6g, o6f; o6g.evaluate_candidates = false; o6f.evaluate_candidates = true;
        N6Options o6sg = o6g, o6sf = o6f; o6sg.include_same_drone = false; o6sf.include_same_drone = false; // restricted variant (different-drone partners only), labelled separately
        const auto add = [](Counts& c, const OperatorResult& r) {
            ++c.calls; c.generated += r.generated; c.evaluated += r.evaluated; c.accepted += r.accepted; c.duplicates += r.duplicates_skipped;
            for (std::size_t i = 0; i < kRejectReasonCount; ++i) c.rej[i] += r.rejected[i];
            g_sink = g_sink + r.generated;
        };
        const auto sweep3 = [&](const N3Options& o, Counts& c) { for (std::size_t t = 0; t < trips; ++t) add(c, n3_drone_service_node_relocate(base, ctx, t, o)); };
        const auto sweep6 = [&](const N6Options& o, Counts& c) { for (std::size_t t = 0; t < trips; ++t) add(c, n6_drone_drone_swap(base, ctx, t, o)); };
        Counts c3g, c3f, c6g, c6f, c6sg, c6sf;
        std::size_t al3 = 0, ab3 = 0, al6 = 0, ab6 = 0, al6s = 0, ab6s = 0;
        sweep3(o3g, c3g); sweep6(o6g, c6g); sweep6(o6sg, c6sg);
        {
            g_allocs = 0; g_alloc_bytes = 0; sweep3(o3f, c3f); al3 = g_allocs; ab3 = g_alloc_bytes;
            g_allocs = 0; g_alloc_bytes = 0; sweep6(o6f, c6f); al6 = g_allocs; ab6 = g_alloc_bytes;
            g_allocs = 0; g_alloc_bytes = 0; sweep6(o6sf, c6sf); al6s = g_allocs; ab6s = g_alloc_bytes;
        }
        {   // drone usage of the base solution (N6 default needs trips of different drones)
            std::vector<int> used;
            for (const auto& t : base.drone_trips) if (std::find(used.begin(), used.end(), t.drone_id) == used.end()) used.push_back(t.drone_id);
            std::cout << "  distinct drones used by the base trips: " << used.size() << "\n";
        }

        std::vector<Series> s(7);
        s[0].name = "base evaluate_solution (1 call)";
        s[1].name = "N3 gen  (sweep over all trips)";
        s[2].name = "N3 full (sweep over all trips)";
        s[3].name = "N6 gen  (sweep over all trips)";
        s[4].name = "N6 full (sweep over all trips)";
        s[5].name = "N6 restricted gen  (different drones only)";
        s[6].name = "N6 restricted full (different drones only)";
        std::vector<std::function<void()>> fs;
        fs.push_back([&] { const auto e = tdr::timing::evaluate_solution(base, net, inst.customer_data, init.flp_data); g_sink = g_sink + static_cast<std::size_t>(e.feasible); });
        Counts d;
        fs.push_back([&] { sweep3(o3g, d); });
        fs.push_back([&] { sweep3(o3f, d); });
        fs.push_back([&] { sweep6(o6g, d); });
        fs.push_back([&] { sweep6(o6f, d); });
        fs.push_back([&] { sweep6(o6sg, d); });
        fs.push_back([&] { sweep6(o6sf, d); });
        run_series(s, fs);

        const double base_us = s[0].median();
        for (const auto& x : s)
            std::cout << "  " << std::left << std::setw(36) << x.name << std::right << std::setw(12) << x.median() << " us  [min " << x.lo() << ", max " << x.hi() << ", spread " << 100.0 * (x.hi() - x.lo()) / x.median() << " %]\n";
        const auto pct = [](std::size_t x, std::size_t of) { return of ? 100.0 * static_cast<double>(x) / static_cast<double>(of) : 0.0; };
        const auto derive = [&](const char* op, const Counts& cg, const Counts& cf, const Series& g, const Series& f, std::size_t allocs, std::size_t bytes) {
            if (cg.calls == 0) { std::cout << "  " << op << ": no trips in this base solution\n"; return; }
            const double gen_net = g.median() - static_cast<double>(cg.calls) * base_us;
            std::cout << "  " << op << ": calls/sweep=" << cg.calls << " generated=" << cg.generated << " evaluator-calls=" << cf.evaluated << " feasible=" << cf.accepted << " (" << pct(cf.accepted, cf.generated) << " % of generated, "
                      << pct(cf.accepted, cf.evaluated) << " % of evaluated) duplicates=" << cf.duplicates << "\n";
            std::cout << "      rejected (full): ";
            bool any = false;
            for (std::size_t i = 1; i < kRejectReasonCount; ++i) if (cf.rej[i]) { std::cout << reject_reason_name(static_cast<RejectReason>(i)) << "=" << cf.rej[i] << " "; any = true; }
            if (!any) std::cout << "none";
            std::cout << "  (accepted + rejected == generated: " << ((cf.accepted + [&] { std::size_t t = 0; for (std::size_t i = 1; i < kRejectReasonCount; ++i) t += cf.rej[i]; return t; }() == cf.generated) ? "yes" : "NO") << ")\n";
            if (cg.generated > 0)
                std::cout << "      generation (net of " << cg.calls << " base evals): " << gen_net << " us per sweep = " << gen_net / static_cast<double>(cg.generated) << " us per generated candidate -> "
                          << static_cast<double>(cg.generated) / (gen_net * 1e-6) << " candidates/s\n";
            if (cf.evaluated > 0)
                std::cout << "      evaluation (full - gen): " << (f.median() - g.median()) << " us per sweep = " << (f.median() - g.median()) / static_cast<double>(cf.evaluated) << " us per evaluated candidate (evaluator alone: " << base_us << " us per call)\n";
            std::cout << "      full sweep total per generated candidate: " << f.median() / static_cast<double>(std::max<std::size_t>(cf.generated, 1)) << " us; allocations per full sweep: " << allocs << " (" << bytes / 1024 << " KiB total, "
                      << static_cast<double>(allocs) / static_cast<double>(std::max<std::size_t>(cf.generated, 1)) << " per generated candidate)\n";
        };
        derive("N3", c3g, c3f, s[1], s[2], al3, ab3);
        derive("N6", c6g, c6f, s[3], s[4], al6, ab6);
        derive("N6 restricted (variant, include_same_drone=false)", c6sg, c6sf, s[5], s[6], al6s, ab6s);
        std::cout << "\n";
    }
    return static_cast<int>(g_sink == static_cast<std::size_t>(-1));
}
