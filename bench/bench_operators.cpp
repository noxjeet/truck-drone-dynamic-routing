// Operator micro-benchmark for M2 (N4, N5). NOT part of the default build or of CTest (cmake -DTDR_BUILD_BENCH=ON).
//
// For each customer count the instance and the initial solution are generated from fixed seeds. A "sweep" calls
// the operator once for EVERY anchor of the base solution (N4: each truck customer; N5: each drone trip) with
// the default window (3), record_candidates = false (search mode).
//
// Two modes per operator, measured separately:
//   gen   evaluate_candidates = false : candidate construction + duplicate removal + affected-trip check
//         (N5: + dissolution + free-drone search). One base evaluation per call is still made (input check).
//   full  evaluate_candidates = true  : the same plus the evaluator on every constructed, unaffected candidate.
// Derived: base-evaluation cost per call (measured on its own), generation cost per candidate (net of the base
// evaluation), marginal evaluation cost per evaluated candidate = (full - gen) / evaluated, and throughput.
//
// Timing: R repetitions; in each repetition every mode runs enough sweeps to last >= kBatchMs (batch calibrated
// once), round-robin over the modes so drift hits all alike. Reported: per-sweep median, min, max and relative
// spread (max-min)/median. steady_clock wall time. The evaluator is the production P1+P2 one.

#include "config/static_config.hpp"
#include "flp/flp_arc_index.hpp"
#include "init/initial_solution_builder.hpp"
#include "instance/static_instance.hpp"
#include "search/neighbourhood_n4.hpp"
#include "search/neighbourhood_n5.hpp"
#include "timing/timing_engine.hpp"
#include "util/rng.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
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
    std::size_t calls = 0, generated = 0, evaluated = 0, accepted = 0, affected = 0, nofree = 0;
    std::size_t stale = 0, reanchored = 0, trips_re = 0, variants = 0, noflp = 0, invrec = 0, capped = 0, eval_rej = 0; // N4 Reanchor diagnostics (capped = candidates whose enumeration hit the budget)
};

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

int main(int argc, char** argv) {
    const std::vector<int> sizes = argc > 1 ? std::vector<int>{std::stoi(argv[1])} : std::vector<int>{10, 20, 50};
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "operator benchmark, window=3, seeds: base 31415, dataset_index 0; " << kReps << " reps, batch >= " << kBatchMs << " ms\n\n";
    for (int n : sizes) {
        tdr::config::StaticConfig cfg;
        cfg.seed.base_seed = 31415;
        cfg.instance.num_customers = n;
        cfg.instance.dataset_index = 0;
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

        std::vector<NodeId> anchors4;
        {
            const Skeleton sk = make_skeleton(base.truck_route, net.nodes());
            for (std::size_t s = 1; s + 1 < sk.size(); ++s) anchors4.push_back(sk.node[s]);
        }
        const std::size_t trips = base.drone_trips.size();
        std::cout << "n=" << n << "  |pool|=" << init.flp_data.size() << "  truck customers=" << anchors4.size() << "  trips=" << trips << "  drones=" << inst.drone.num_drones << "\n";

        // base evaluation on its own
        Counts c4rg, c4rf, c4g, c4f, c4eg, c4ef, c5g, c5f;
        // N4 is measured under BOTH affected-trip policies: Reject (the original M2 behaviour = "before") and Reanchor (the default = "after").
        N4Options o4rg, o4rf, o4g, o4f, o4eg, o4ef; // Reject / Reanchor (default cap 100) / Reanchor with the former cap 10000
        o4rg.affected_trips = o4rf.affected_trips = AffectedTripPolicy::Reject;
        o4rg.evaluate_candidates = false; o4rf.evaluate_candidates = true; o4g.evaluate_candidates = false; o4f.evaluate_candidates = true;
        o4eg.max_reanchor_variants = o4ef.max_reanchor_variants = 10000; o4eg.evaluate_candidates = false; o4ef.evaluate_candidates = true;
        N5Options o5g, o5f; o5g.evaluate_candidates = false; o5f.evaluate_candidates = true;
        const auto rj = [](const OperatorResult& r, RejectReason w) { return static_cast<std::size_t>(r.rejected[static_cast<std::size_t>(w)]); };
        const auto sweep4 = [&](const N4Options& o, Counts& c) {
            for (NodeId a : anchors4) {
                const auto r = n4_truck_service_node_relocate(base, ctx, a, o);
                ++c.calls; c.generated += r.generated; c.evaluated += r.evaluated; c.accepted += r.accepted; c.affected += rj(r, RejectReason::AffectedTrips);
                c.stale += r.stale_launch_candidates; c.reanchored += r.reanchored_candidates; c.trips_re += r.trips_reanchored; c.variants += r.variants_generated;
                c.noflp += rj(r, RejectReason::NoReplacementFlp); c.invrec += rj(r, RejectReason::InvalidRecovery); c.capped += r.cap_reached_candidates;
                c.eval_rej += r.rejected_total() - rj(r, RejectReason::AffectedTrips) - rj(r, RejectReason::NoReplacementFlp) - rj(r, RejectReason::InvalidRecovery);
                g_sink = g_sink + r.generated;
            }
        };
        const auto sweep5 = [&](const N5Options& o, Counts& c) {
            for (std::size_t t = 0; t < trips; ++t) { const auto r = n5_td_service_node_swap(base, ctx, t, o); ++c.calls; c.generated += r.generated; c.evaluated += r.evaluated; c.accepted += r.accepted; c.affected += r.rejected[static_cast<std::size_t>(RejectReason::AffectedTrips)]; c.nofree += r.rejected[static_cast<std::size_t>(RejectReason::NoFreeDrone)]; g_sink = g_sink + r.generated; }
        };
        // counts for one sweep (not timed)
        sweep4(o4rg, c4rg); sweep4(o4rf, c4rf); sweep4(o4g, c4g); sweep4(o4f, c4f); sweep4(o4eg, c4eg); sweep4(o4ef, c4ef); sweep5(o5g, c5g); sweep5(o5f, c5f);

        std::vector<Series> s(9);
        s[0].name = "base evaluate_solution (1 call)";
        s[1].name = "N4 Reject gen          (all anchors)";
        s[2].name = "N4 Reject full         (all anchors)";
        s[3].name = "N4 Reanchor c100 gen   (all anchors)";
        s[4].name = "N4 Reanchor c100 full  (all anchors)";
        s[5].name = "N4 Reanchor c10000 gen (all anchors)";
        s[6].name = "N4 Reanchor c10000 full(all anchors)";
        s[7].name = "N5 gen  (sweep over all trips)";
        s[8].name = "N5 full (sweep over all trips)";
        std::vector<std::function<void()>> fs;
        fs.push_back([&] { const auto e = tdr::timing::evaluate_solution(base, net, inst.customer_data, init.flp_data); g_sink = g_sink + static_cast<std::size_t>(e.feasible); });
        Counts d;
        fs.push_back([&] { sweep4(o4rg, d); });
        fs.push_back([&] { sweep4(o4rf, d); });
        fs.push_back([&] { sweep4(o4g, d); });
        fs.push_back([&] { sweep4(o4f, d); });
        fs.push_back([&] { sweep4(o4eg, d); });
        fs.push_back([&] { sweep4(o4ef, d); });
        fs.push_back([&] { sweep5(o5g, d); });
        fs.push_back([&] { sweep5(o5f, d); });
        run_series(s, fs);

        const double base_us = s[0].median();
        const auto line = [&](const Series& x) {
            std::cout << "  " << std::left << std::setw(36) << x.name << std::right << std::setw(12) << x.median() << " us  [min " << x.lo() << ", max " << x.hi() << ", spread " << 100.0 * (x.hi() - x.lo()) / x.median() << " %]\n";
        };
        for (const auto& x : s) line(x);
        const auto derive = [&](const char* op, const Counts& cg, const Counts& cf, const Series& g, const Series& f) {
            if (cg.calls == 0) { std::cout << "  " << op << ": no anchors in this base solution\n"; return; }
            const double gen_net = g.median() - static_cast<double>(cg.calls) * base_us;
            std::cout << "  " << op << ": calls/sweep=" << cg.calls << " generated=" << cg.generated << " evaluated(full)=" << cf.evaluated << " accepted=" << cf.accepted
                      << " affected=" << cg.affected << (cg.nofree ? " no-free-drone=" + std::to_string(cg.nofree) : std::string()) << "\n";
            if (cg.generated > 0)
                std::cout << "      generation (net of " << cg.calls << " base evals): " << gen_net << " us per sweep = " << gen_net / static_cast<double>(cg.generated) << " us per generated candidate  -> "
                          << static_cast<double>(cg.generated) / (gen_net * 1e-6) << " candidates/s\n";
            if (cf.evaluated > 0)
                std::cout << "      evaluation (full - gen): " << (f.median() - g.median()) << " us per sweep = " << (f.median() - g.median()) / static_cast<double>(cf.evaluated) << " us per evaluated candidate (evaluator alone: " << base_us << " us per call)\n";
        };
        derive("N4 Reject      ", c4rg, c4rf, s[1], s[2]);
        derive("N4 Reanchor c100  ", c4g, c4f, s[3], s[4]);
        derive("N4 Reanchor c10000", c4eg, c4ef, s[5], s[6]);
        derive("N5", c5g, c5f, s[7], s[8]);
        if (c4g.generated > 0) {
            const auto pct = [](std::size_t x, std::size_t of) { return of ? 100.0 * static_cast<double>(x) / static_cast<double>(of) : 0.0; };
            const auto row = [&](const char* name, const Counts& c) {
                std::cout << "      " << std::left << std::setw(16) << name << std::right << " feasible " << c.accepted << " (" << pct(c.accepted, c.generated) << " %), re-anchored " << c.reanchored << " (" << pct(c.reanchored, c.generated) << " %, " << c.trips_re
                          << " trips), stale " << c.stale << ", rejected " << (c.generated - c.accepted) << " (" << pct(c.generated - c.accepted, c.generated) << " %): invalid-recovery " << c.invrec << ", no-replacement-flp " << c.noflp << ", evaluator " << c.eval_rej
                          << "; combinations generated " << c.variants << ", evaluator calls " << c.evaluated << " (" << (c.stale ? static_cast<double>(c.variants) / static_cast<double>(c.stale) : 0.0) << " per stale candidate), cap reached in " << c.capped
                          << " candidates (" << pct(c.capped, c.stale) << " % of stale, " << pct(c.capped, c.generated) << " % of all)\n";
            };
            std::cout << "  N4 outcomes per sweep (" << c4g.generated << " candidates):\n";
            { std::cout << "      " << std::left << std::setw(16) << "Reject" << std::right << " feasible " << c4rf.accepted << " (" << pct(c4rf.accepted, c4rf.generated) << " %), affected-trips rejected " << c4rf.affected << " (" << pct(c4rf.affected, c4rf.generated) << " %)\n"; }
            row("Reanchor c100", c4f);
            row("Reanchor c10000", c4ef);
        }
        std::cout << "\n";
    }
    return static_cast<int>(g_sink == static_cast<std::size_t>(-1));
}
