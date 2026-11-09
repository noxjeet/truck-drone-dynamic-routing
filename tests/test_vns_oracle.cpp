// M7 oracle test. For seeded random instances the VNS driver (run_vns) is compared EXACTLY (per-pass trace, final solutions, every
// counter) with an independent re-implementation of docs/static/vns.md written here: its own state variables, its own temperature,
// acceptance and best-so-far arithmetic, its own seed-stream construction, its own neighbourhood list. It uses only the already
// validated building blocks (shake(), the eight operators) through their public entry points, and replays the documented draw
// order (shakes from the Shake stream; operator anchors and the acceptance draw from the Vns stream). Every state the oracle
// accepts is re-judged by the FROZEN reference evaluator (feasible, same objective).
#include "config/static_config.hpp"
#include "flp/flp_arc_index.hpp"
#include "init/initial_solution_builder.hpp"
#include "instance/static_instance.hpp"
#include "reference/timing_engine_reference.hpp"
#include "search/vns.hpp"
#include "util/rng.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace tdr::model;
using namespace tdr::search;

namespace {

int g_failures = 0;
long g_checks = 0;
void check(bool ok, const std::string& what) {
    ++g_checks;
    if (!ok) { if (g_failures < 40) std::cout << "[FAIL] " << what << "\n"; ++g_failures; }
}

struct Ref { bool feasible; double objective; };
Ref ref_eval(const tdr::graph::SpatialNetwork& net, const std::unordered_map<NodeId, CustomerData>& cd, const tdr::timing::FlpDataMap& flp, const Solution& s) {
    const auto e = tdr::timing_ref::evaluate_solution(s, net, cd, flp);
    return {e.feasible, e.objective_value};
}

bool same_sol(const Solution& a, const Solution& b) {
    if (a.truck_route != b.truck_route || a.drone_trips.size() != b.drone_trips.size()) return false;
    for (std::size_t i = 0; i < a.drone_trips.size(); ++i)
        if (a.drone_trips[i].drone_id != b.drone_trips[i].drone_id || a.drone_trips[i].launch_node != b.drone_trips[i].launch_node || a.drone_trips[i].service_customer != b.drone_trips[i].service_customer || a.drone_trips[i].recovery_node != b.drone_trips[i].recovery_node) return false;
    return true;
}

struct Row { double T, cur, best; bool shaken; int kind; std::size_t moves; int end; std::size_t steps; };
struct Out {
    std::vector<Row> rows;
    Solution best, cur;
    double fbest = 0, fcur = 0, T = 0;
    std::size_t applied = 0, na = 0, inf = 0, skipped = 0, best_updates = 0, last_best = 0, truncated = 0, completed = 0, steps_total = 0, steps_max = 0;
    std::array<std::size_t, 8> calls{}, noanchor{}, nocand{}, impr{}, saacc{}, sarej{};
};

// independent loop. mode 0 = printed reference f(x) (Incumbent), 1 = f(w) (ShakenState)
Out oracle(const Solution& init, const SearchContext& ctx, const VnsOptions& o, const tdr::graph::SpatialNetwork& net, const std::unordered_map<NodeId, CustomerData>& cd, const tdr::timing::FlpDataMap& flp) {
    Out out;
    // the driver's own objective is the production evaluator's (last-bit differences from the frozen reference are the documented P2 ordering effect)
    const Ref r0{true, tdr::timing::evaluate_solution(init, net, cd, flp).objective_value};
    check(std::fabs(r0.objective - ref_eval(net, cd, flp, init).objective) <= 1e-9 * std::max(1.0, r0.objective), "initial objective: production equals frozen reference within the P2 bound");
    // streams
    tdr::util::Rng vr(tdr::util::derive_seed(o.base_seed, tdr::util::SeedStream::Vns, o.run_index));
    tdr::util::Rng sr(tdr::util::derive_seed(o.base_seed, tdr::util::SeedStream::Shake, o.run_index));
    std::vector<OperatorId> order = {OperatorId::N4, OperatorId::N5, OperatorId::N8, OperatorId::N3, OperatorId::N6, OperatorId::N7};
    if (!o.neighbourhoods.empty()) order = o.neighbourhoods;
    else for (int r = 0; r < std::max(o.n1_repeats, o.n2_repeats); ++r) { if (r < o.n1_repeats) order.push_back(OperatorId::N1); if (r < o.n2_repeats) order.push_back(OperatorId::N2); }
    Solution x = init, best = init;
    double fx = r0.objective, fb = r0.objective;
    double T = o.t0_absolute > 0 ? o.t0_absolute : std::max(o.t_min, o.t0_fraction * r0.objective);
    std::size_t nobest = 0;
    for (std::size_t it = 1; it <= o.iter_max; ++it) {
        Row row{T, 0, 0, false, -1, 0, 0, 0};
        bool improved_best = false;
        Solution w;
        double fw = 0;
        bool got = false;
        int kind = -1;
        for (std::size_t a = 0; a < o.max_shake_attempts && !got; ++a) {
            ShakeResult s = shake(x, ctx, sr, o.shake);
            if (s.status != ShakeStatus::Ok) { ++out.na; continue; }
            if (!s.feasible) { ++out.inf; continue; }
            // independent verdict on the shaken solution
            const Ref rs = ref_eval(net, cd, flp, s.solution);
            check(rs.feasible && std::fabs(rs.objective - s.objective) <= 1e-9 * std::max(1.0, rs.objective), "accepted shake result: frozen-reference feasible and equal objective");
            w = s.solution; fw = s.objective; got = true; kind = static_cast<int>(s.kind);
        }
        row.end = 1; row.steps = 0;
        if (!got) ++out.skipped;
        else {
            ++out.applied;
            row.shaken = true; row.kind = kind;
            if (fw < fb) { best = w; fb = fw; ++out.best_updates; out.last_best = it; improved_best = true; }
            std::size_t idx = 0, steps = 0;
            bool cut = false;
            for (;;) {
                if (idx >= order.size()) break;
                if (o.max_local_steps > 0 && steps >= o.max_local_steps) { ++out.truncated; cut = true; break; }
                ++steps;
                const auto op = static_cast<std::size_t>(order[idx]);
                OperatorResult res;
                switch (order[idx]) {
                    case OperatorId::N1: res = n1_launch_node_relocate(w, ctx, vr); break;
                    case OperatorId::N2: res = n2_recover_node_relocate(w, ctx, vr); break;
                    case OperatorId::N3: res = n3_drone_service_node_relocate(w, ctx, vr); break;
                    case OperatorId::N4: res = n4_truck_service_node_relocate(w, ctx, vr, o.n4); break;
                    case OperatorId::N5: res = n5_td_service_node_swap(w, ctx, vr); break;
                    case OperatorId::N6: res = n6_drone_drone_swap(w, ctx, vr); break;
                    case OperatorId::N7: res = n7_truck_node_delete_drone_insert(w, ctx, vr); break;
                    default: res = n8_drone_node_delete_truck_insert(w, ctx, vr); break;
                }
                ++out.calls[op];
                if (res.status != OperatorStatus::Ok) { ++out.noanchor[op]; ++idx; continue; }
                if (!res.best.has_value()) { ++out.nocand[op]; ++idx; continue; }
                const double fz = res.best->objective;
                const double ref = o.reference == AcceptanceReference::Incumbent ? fx : fw;
                bool take = false, better = false;
                if (fz < ref - o.min_improvement) { take = true; better = true; ++out.impr[op]; }
                else {
                    const double u = vr.uniform01();
                    const double p = std::exp((ref - fz) / T);
                    if (u <= p) { take = true; ++out.saacc[op]; } else ++out.sarej[op];
                }
                if (take) {
                    const Ref rz = ref_eval(net, cd, flp, res.best->solution);
                    check(rz.feasible && std::fabs(rz.objective - fz) <= 1e-9 * std::max(1.0, rz.objective), "accepted operator result: frozen-reference feasible and equal objective");
                    w = res.best->solution; fw = fz; ++row.moves;
                    if (o.reference == AcceptanceReference::Incumbent) { x = w; fx = fw; }
                    if (fw < fb) { best = w; fb = fw; ++out.best_updates; out.last_best = it; improved_best = true; }
                }
                idx = better ? 0 : idx + 1;
            }
            if (o.reference == AcceptanceReference::ShakenState) { x = w; fx = fw; }
            row.steps = steps; out.steps_total += steps; out.steps_max = std::max(out.steps_max, steps);
            if (cut) row.end = 2; else { row.end = 0; ++out.completed; }
        }
        row.cur = fx; row.best = fb;
        out.rows.push_back(row);
        nobest = improved_best ? 0 : nobest + 1;
        T = std::max(o.t_min, o.cooling_rate * T);
        if (o.max_no_improve > 0 && nobest >= o.max_no_improve) break;
    }
    out.best = best; out.cur = x; out.fbest = fb; out.fcur = fx; out.T = T;
    return out;
}

struct Stats {
    long runs = 0, passes = 0, skipped = 0, applied = 0, na = 0, inf = 0, moves_imp = 0, moves_sa = 0, best_updates = 0, trunc = 0, noimp_stops = 0, runs_improved = 0, shaken_state = 0;
    std::array<long, 4> kinds{};
    std::uint64_t digest = 14695981039346656037ULL;
    void mix(std::uint64_t v) { digest = (digest ^ v) * 1099511628211ULL; }
};

void compare(const Solution& init, const SearchContext& ctx, const VnsOptions& o, const tdr::graph::SpatialNetwork& net, const std::unordered_map<NodeId, CustomerData>& cd, const tdr::timing::FlpDataMap& flp, Stats& st, const std::string& tag) {
    VnsOptions oo = o;
    oo.record_trace = true;
    const VnsResult r = run_vns(init, ctx, oo);
    const Out e = oracle(init, ctx, o, net, cd, flp);
    check(r.status == VnsStatus::Ok, tag + ": status Ok");
    bool rows = r.trace.size() == e.rows.size();
    for (std::size_t i = 0; rows && i < e.rows.size(); ++i) {
        const VnsTraceRow& a = r.trace[i]; const Row& b = e.rows[i];
        rows = static_cast<int>(a.end) == b.end && a.local_steps == b.steps && a.iter == i + 1 && a.temperature == b.T && a.current == b.cur && a.best == b.best && a.shaken == b.shaken && a.shake_kind == b.kind && a.moves == b.moves;
    }
    if (!rows && g_failures == 0) for (std::size_t i = 0; i < std::min(r.trace.size(), e.rows.size()); ++i) { const auto& a = r.trace[i]; const auto& b = e.rows[i]; if (a.temperature != b.T || a.current != b.cur || a.best != b.best || a.shaken != b.shaken || a.shake_kind != b.kind || a.moves != b.moves) { std::cout << std::setprecision(17) << "first diff pass " << i + 1 << " T " << a.temperature << "/" << b.T << " cur " << a.current << "/" << b.cur << " best " << a.best << "/" << b.best << " shaken " << a.shaken << "/" << b.shaken << " kind " << a.shake_kind << "/" << b.kind << " moves " << a.moves << "/" << b.moves << " sizes " << r.trace.size() << "/" << e.rows.size() << "\n"; break; } }
    check(rows, tag + ": per-pass trace (T, f(x), f(best), shake kind, moves) equals the oracle's");
    check(same_sol(r.best, e.best) && same_sol(r.current, e.cur) && r.best_objective == e.fbest && r.current_objective == e.fcur && r.final_temperature == e.T, tag + ": best, current and final temperature equal the oracle's");
    check(r.shakes_applied == e.applied && r.shakes_not_applicable == e.na && r.shakes_infeasible == e.inf && r.iterations_skipped == e.skipped && r.best_updates == e.best_updates && r.last_best_iteration == e.last_best && r.passes_truncated == e.truncated && r.passes_completed == e.completed && r.local_steps_total == e.steps_total && r.max_local_steps_in_pass == e.steps_max && r.guard_triggered() == (e.truncated > 0) && r.iterations == r.passes_completed + r.iterations_skipped + r.passes_truncated, tag + ": shake, skip and best-so-far counters equal the oracle's");
    bool ops = true;
    for (std::size_t k = 0; k < 8; ++k) ops = ops && r.ops[k].calls == e.calls[k] && r.ops[k].no_anchor == e.noanchor[k] && r.ops[k].no_candidate == e.nocand[k] && r.ops[k].improving == e.impr[k] && r.ops[k].sa_accepted == e.saacc[k] && r.ops[k].sa_rejected == e.sarej[k];
    check(ops, tag + ": per-operator statistics equal the oracle's");
    const Ref rb = ref_eval(net, cd, flp, r.best), rc = ref_eval(net, cd, flp, r.current);
    check(rb.feasible && rc.feasible && std::fabs(rb.objective - r.best_objective) <= 1e-9 * std::max(1.0, rb.objective), tag + ": final best/current are frozen-reference feasible");
    check(r.best_objective <= r.initial_objective, tag + ": best <= initial");
    ++st.runs; st.passes += static_cast<long>(r.iterations); st.skipped += static_cast<long>(r.iterations_skipped); st.applied += static_cast<long>(r.shakes_applied); st.na += static_cast<long>(r.shakes_not_applicable); st.inf += static_cast<long>(r.shakes_infeasible);
    st.moves_imp += static_cast<long>(r.improving_moves); st.moves_sa += static_cast<long>(r.sa_accepted_moves); st.best_updates += static_cast<long>(r.best_updates); st.trunc += static_cast<long>(r.passes_truncated);
    for (std::size_t k = 0; k < 4; ++k) st.kinds[k] += static_cast<long>(r.shakes_applied_by_kind[k]);
    if (r.best_objective < r.initial_objective) ++st.runs_improved;
    if (r.stop == VnsStop::NoImprovement) ++st.noimp_stops;
    if (o.reference == AcceptanceReference::ShakenState) ++st.shaken_state;
    st.mix(static_cast<std::uint64_t>(r.iterations)); st.mix(static_cast<std::uint64_t>(r.shakes_applied)); st.mix(static_cast<std::uint64_t>(r.best_updates));
    for (NodeId v : r.best.truck_route) st.mix(static_cast<std::uint64_t>(v));
    for (const auto& t : r.best.drone_trips) { st.mix(static_cast<std::uint64_t>(t.launch_node)); st.mix(static_cast<std::uint64_t>(t.recovery_node)); st.mix(static_cast<std::uint64_t>(t.drone_id)); }
}

Stats campaign(int seeds) {
    Stats st;
    for (int seed = 0; seed < seeds; ++seed) {
        tdr::util::Rng fz(tdr::util::derive_seed(7272, tdr::util::SeedStream::Fuzz, static_cast<std::uint64_t>(seed)));
        tdr::config::StaticConfig cfg;
        cfg.seed.base_seed = 7272;
        cfg.instance.num_customers = static_cast<int>(fz.uniform_int(6, 13));
        cfg.instance.dataset_index = seed;
        cfg.vehicle.num_drones = static_cast<int>(fz.uniform_int(seed % 5 == 0 ? 1 : 2, 3));
        switch (seed % 4) {
            case 1: cfg.vehicle.endurance_s *= 3.0; break;
            case 2: cfg.instance.area_side_m = 15000.0; break;
            case 3: cfg.instance.area_side_m = 15000.0; cfg.vehicle.endurance_s *= 3.0; break;
            default: break;
        }
        const auto gen = tdr::instance::generate_static_instance(cfg);
        check(gen.ok, "instance generation");
        if (!gen.ok) continue;
        const auto& inst = gen.instance;
        const auto net0 = inst.make_network();
        tdr::init::InitialSolutionPolicy pol;
        pol.flp_policy.unit_division_length = cfg.flp.unit_division_length_m;
        const auto init = tdr::init::build_initial_solution(inst.depot, inst.customers, inst.customer_data, net0, pol);
        if (!init.feasible) continue;
        const tdr::graph::SpatialNetwork net(init.node_registry, inst.truck, inst.drone);
        const tdr::flp::FlpArcIndex idx(init.flp_data);
        const SearchContext ctx{net, inst.customer_data, init.flp_data, idx, 3};
        const std::string tag = "seed " + std::to_string(seed);
        VnsOptions o;
        o.iter_max = 14;
        o.base_seed = 9000 + static_cast<std::uint64_t>(seed);
        o.run_index = static_cast<std::uint64_t>(seed % 3);
        compare(init.solution, ctx, o, net, inst.customer_data, init.flp_data, st, tag + " default");
        VnsOptions s2 = o;
        s2.reference = AcceptanceReference::ShakenState;
        s2.t0_fraction = 0.01;
        s2.cooling_rate = 0.9;
        compare(init.solution, ctx, s2, net, inst.customer_data, init.flp_data, st, tag + " shaken-state");
        if (seed % 2 == 0) {
            VnsOptions hot = o;
            hot.t0_fraction = 0.05;
            hot.max_local_steps = 25;
            hot.cooling_rate = 0.95;
            compare(init.solution, ctx, hot, net, inst.customer_data, init.flp_data, st, tag + " warm+guard");
        }
        if (seed % 3 == 0) {
            VnsOptions ni = o;
            ni.iter_max = 60;
            ni.max_no_improve = 4;
            ni.t0_absolute = 1e-9;
            ni.neighbourhoods = {OperatorId::N7, OperatorId::N4, OperatorId::N1, OperatorId::N2, OperatorId::N8};
            ni.min_improvement = 0.5;
            ni.n4.affected_trips = AffectedTripPolicy::Reject;
            compare(init.solution, ctx, ni, net, inst.customer_data, init.flp_data, st, tag + " custom list, delta, no-improvement");
        }
        if (seed % 4 == 1) {
            VnsOptions tiny = o;
            tiny.max_shake_attempts = 1;
            tiny.n4.max_reanchor_variants = 2;
            tiny.shake.max_segment_length = 1;
            tiny.shake.max_shift = 1;
            compare(init.solution, ctx, tiny, net, inst.customer_data, init.flp_data, st, tag + " one shake attempt, L = D = 1");
        }
    }
    return st;
}

} // namespace

int main() {
    constexpr int kSeeds = 100;
    const Stats a = campaign(kSeeds);
    const Stats b = campaign(kSeeds);
    std::cout << "runs=" << a.runs << " passes=" << a.passes << " shakes_applied=" << a.applied << " (reverse " << a.kinds[0] << ", exchange " << a.kinds[1] << ", shift " << a.kinds[2] << ", drone-remove " << a.kinds[3] << ")"
              << " not_applicable=" << a.na << " infeasible=" << a.inf << " skipped_passes=" << a.skipped << "\n"
              << "improving_moves=" << a.moves_imp << " sa_accepted_moves=" << a.moves_sa << " best_updates=" << a.best_updates << " runs_with_better_best=" << a.runs_improved << " truncated_passes=" << a.trunc << " no_improvement_stops=" << a.noimp_stops << " shaken_state_runs=" << a.shaken_state << "\n";
    check(a.runs > 150 && a.passes > 2000, "non-vacuity: enough runs and passes");
    check(a.applied > 1500 && a.kinds[0] > 50 && a.kinds[1] > 50 && a.kinds[2] > 50 && a.kinds[3] > 50, "non-vacuity: all four shake kinds are applied");
    check(a.na > 0, "non-vacuity: NotApplicable shakes occur and are redrawn");
    check(a.moves_imp > 20 && a.moves_sa > 20, "non-vacuity: improving and SA-accepted moves occur");
    check(a.best_updates > 20 && a.runs_improved > 10, "non-vacuity: the best-so-far improves in some runs");
    check(a.trunc > 0, "non-vacuity: the local-step guard triggers in the warm runs");
    check(a.noimp_stops > 0, "non-vacuity: the no-improvement stop triggers");
    check(a.shaken_state > 50, "non-vacuity: ShakenState runs");
    check(a.digest == b.digest, "deterministic: identical digest over the whole campaign on a second full run (oracle on both)");
    std::cout << g_checks << " checks, " << g_failures << " failures\n";
    if (g_failures == 0) std::cout << "ALL TESTS PASSED\n";
    return g_failures == 0 ? 0 : 1;
}
