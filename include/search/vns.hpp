#pragma once

// VNS driver of Algorithm 1 (M7, 9 Nov 2026). Contract: docs/static/vns.md (written before this file). Summary:
//  * pass = shake (redrawn when not applicable / evaluator-infeasible, else the pass is skipped) + sequential local search over the
//    neighbourhood list with SA acceptance + one cooling step; `iter` counts every pass (D8 fix of the printed pseudocode).
//  * accepted candidates are carried forward (the next neighbourhood searches from them); a separate best-so-far is kept (D8).
//  * the evaluator is the only judge: every stored solution is the evaluator-feasible result of a shake or an operator; nothing is repaired.
//  * two seed streams (Vns, Shake) derived from base seed + run index: bit-identical reproducibility.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "search/move_types.hpp"
#include "search/neighbourhood_n1.hpp"
#include "search/neighbourhood_n2.hpp"
#include "search/neighbourhood_n3.hpp"
#include "search/neighbourhood_n4.hpp"
#include "search/neighbourhood_n5.hpp"
#include "search/neighbourhood_n6.hpp"
#include "search/neighbourhood_n7.hpp"
#include "search/neighbourhood_n8.hpp"
#include "search/shakes.hpp"
#include "timing/timing_engine.hpp"
#include "util/rng.hpp"

namespace tdr::search {

enum class AcceptanceReference { Incumbent, ShakenState };
enum class VnsStatus { Ok, InvalidOptions, InitialInfeasible };
enum class VnsStop { None, IterMax, NoImprovement, TimeLimit };
// How one pass ended (A38): Completed = the neighbourhood list was exhausted normally; Skipped = no feasible shake
// (no local search ran); GuardTruncated = the local search was cut off by max_local_steps (never silent).
enum class VnsPassEnd { Completed, Skipped, GuardTruncated };

inline const char* vns_pass_end_name(VnsPassEnd e) {
    switch (e) {
        case VnsPassEnd::Completed: return "completed";
        case VnsPassEnd::Skipped: return "skipped";
        case VnsPassEnd::GuardTruncated: return "guard-truncated";
    }
    return "?";
}

inline const char* vns_stop_name(VnsStop s) {
    switch (s) {
        case VnsStop::None: return "none";
        case VnsStop::IterMax: return "iter-max";
        case VnsStop::NoImprovement: return "no-improvement";
        case VnsStop::TimeLimit: return "time-limit";
    }
    return "?";
}

// Sec 6.1 order, then N1 and N2 alternately (assumption A32, D2).
inline std::vector<OperatorId> default_neighbourhood_list(int n1_repeats = 2, int n2_repeats = 2) {
    std::vector<OperatorId> v = {OperatorId::N4, OperatorId::N5, OperatorId::N8, OperatorId::N3, OperatorId::N6, OperatorId::N7};
    for (int r = 0; r < std::max(n1_repeats, n2_repeats); ++r) {
        if (r < n1_repeats) v.push_back(OperatorId::N1);
        if (r < n2_repeats) v.push_back(OperatorId::N2);
    }
    return v;
}

struct VnsOptions {
    std::size_t iter_max = 2000;      // [ASSUMPTION A33, pilot-based default, D7]
    double t0_fraction = 0.01;        // T0 = max(t_min, t0_fraction * f(initial)) unless t0_absolute > 0 [A33, pilot]
    double t0_absolute = 0.0;
    double cooling_rate = 0.995;      // update(T): T <- max(t_min, cooling_rate * T) once per pass [A31, pilot]
    double t_min = 1e-9;              // temperature floor (> 0)
    std::size_t max_no_improve = 0;   // stop after this many passes without a new best-so-far; 0 = off [A36]
    double max_seconds = 0.0;         // wall-clock limit; 0 = off (the only non-reproducible criterion)
    std::size_t max_shake_attempts = 10; // attempts per pass before the pass is skipped [A35]
    std::size_t max_local_steps = 1000; // operator calls per pass; 0 = unlimited. Guard only: at a very high T non-improving moves can alternate with improvements forever [A38]
    double min_improvement = 0.0;     // delta: improvement iff f(z) < ref - delta; 0 = strict '<' [A37]
    AcceptanceReference reference = AcceptanceReference::Incumbent; // [A30]
    std::vector<OperatorId> neighbourhoods; // empty -> default_neighbourhood_list(n1_repeats, n2_repeats)
    int n1_repeats = 2;               // D2 (NeighbourhoodConfig)
    int n2_repeats = 2;
    ShakeOptions shake;               // L_max = D_max = 3 (M6)
    N4Options n4;                     // Reanchor, budget 100 (M2)
    std::uint64_t base_seed = 20261109ULL;
    std::uint64_t run_index = 0;
    bool record_trace = false;
};

inline std::vector<std::string> validate(const VnsOptions& o) {
    std::vector<std::string> e;
    if (!(o.t0_fraction >= 0.0) || !std::isfinite(o.t0_fraction)) e.push_back("t0_fraction must be a finite number >= 0");
    if (!(o.t0_absolute >= 0.0) || !std::isfinite(o.t0_absolute)) e.push_back("t0_absolute must be a finite number >= 0");
    if (!(o.cooling_rate > 0.0 && o.cooling_rate <= 1.0)) e.push_back("cooling_rate must be in (0, 1]");
    if (!(o.t_min > 0.0) || !std::isfinite(o.t_min)) e.push_back("t_min must be a finite number > 0");
    if (!(o.max_seconds >= 0.0) || !std::isfinite(o.max_seconds)) e.push_back("max_seconds must be a finite number >= 0");
    if (o.max_shake_attempts < 1) e.push_back("max_shake_attempts must be >= 1");
    if (!(o.min_improvement >= 0.0) || !std::isfinite(o.min_improvement)) e.push_back("min_improvement must be a finite number >= 0");
    if (o.neighbourhoods.empty() && (o.n1_repeats < 0 || o.n2_repeats < 0)) e.push_back("n1_repeats and n2_repeats must be >= 0");
    return e;
}

struct VnsOpStats {
    std::size_t calls = 0, no_anchor = 0, no_candidate = 0, improving = 0, sa_accepted = 0, sa_rejected = 0;
};

struct VnsTraceRow {
    std::size_t iter = 0;
    double temperature = 0.0; // the T used in this pass
    bool shaken = false;      // a feasible shake was found
    int shake_kind = -1;
    double current = 0.0;     // f(x) at the end of the pass
    double best = 0.0;        // f(best) at the end of the pass
    std::size_t moves = 0;    // accepted moves in this pass
    VnsPassEnd end = VnsPassEnd::Completed; // how the pass ended [A38]
    std::size_t local_steps = 0; // operator calls made in this pass
};

struct VnsResult {
    VnsStatus status = VnsStatus::Ok;
    VnsStop stop = VnsStop::None;
    std::string message;
    tdr::model::Solution best;
    tdr::model::Solution current;
    double initial_objective = 0.0, best_objective = 0.0, current_objective = 0.0;
    double t0 = 0.0, final_temperature = 0.0;
    std::size_t iterations = 0;
    std::size_t passes_truncated = 0; // passes whose local search was cut off by max_local_steps (A38; never silent)
    std::size_t passes_completed = 0; // passes whose local search ran to the end of the list; iterations == passes_completed + iterations_skipped + passes_truncated
    std::size_t local_steps_total = 0, max_local_steps_in_pass = 0; // operator calls overall / in the busiest pass
    bool guard_triggered() const { return passes_truncated > 0; }
    std::size_t shakes_applied = 0, shakes_not_applicable = 0, shakes_infeasible = 0, iterations_skipped = 0;
    std::array<std::size_t, 4> shakes_applied_by_kind{};
    std::array<VnsOpStats, 8> ops{}; // indexed by OperatorId
    std::size_t best_updates = 0, last_best_iteration = 0;
    std::size_t improving_moves = 0, sa_accepted_moves = 0;
    double seconds = 0.0;
    std::vector<VnsTraceRow> trace;
};

namespace detail_vns {

inline OperatorResult apply(OperatorId op, const tdr::model::Solution& w, const SearchContext& ctx, tdr::util::Rng& rng, const VnsOptions& o) {
    switch (op) {
        case OperatorId::N1: return n1_launch_node_relocate(w, ctx, rng);
        case OperatorId::N2: return n2_recover_node_relocate(w, ctx, rng);
        case OperatorId::N3: return n3_drone_service_node_relocate(w, ctx, rng);
        case OperatorId::N4: return n4_truck_service_node_relocate(w, ctx, rng, o.n4);
        case OperatorId::N5: return n5_td_service_node_swap(w, ctx, rng);
        case OperatorId::N6: return n6_drone_drone_swap(w, ctx, rng);
        case OperatorId::N7: return n7_truck_node_delete_drone_insert(w, ctx, rng);
        default: return n8_drone_node_delete_truck_insert(w, ctx, rng);
    }
}

} // namespace detail_vns

inline VnsResult run_vns(const tdr::model::Solution& initial, const SearchContext& ctx, const VnsOptions& o = {}) {
    using Clock = std::chrono::steady_clock;
    VnsResult R;
    const auto errors = validate(o);
    if (!errors.empty()) {
        R.status = VnsStatus::InvalidOptions;
        for (const auto& m : errors) R.message += (R.message.empty() ? "" : "; ") + m;
        return R;
    }
    const auto e0 = tdr::timing::evaluate_solution(initial, ctx.network, ctx.customers, ctx.flp_data);
    const EvalVerdict v0 = classify_evaluation(e0, ctx.num_drones());
    if (!v0.feasible) {
        R.status = VnsStatus::InitialInfeasible;
        R.message = "the initial solution is not evaluator-feasible";
        return R;
    }
    const std::vector<OperatorId> list = o.neighbourhoods.empty() ? default_neighbourhood_list(o.n1_repeats, o.n2_repeats) : o.neighbourhoods;
    tdr::util::Rng vns_rng(tdr::util::derive_seed(o.base_seed, tdr::util::SeedStream::Vns, o.run_index));
    tdr::util::Rng shake_rng(tdr::util::derive_seed(o.base_seed, tdr::util::SeedStream::Shake, o.run_index));

    tdr::model::Solution x = initial;
    double fx = v0.objective;
    tdr::model::Solution best = initial;
    double fbest = v0.objective;
    R.initial_objective = v0.objective;
    const double T0 = o.t0_absolute > 0.0 ? o.t0_absolute : std::max(o.t_min, o.t0_fraction * v0.objective);
    double T = T0;
    R.t0 = T0;
    std::size_t since_best = 0;
    const auto start = Clock::now();
    R.stop = VnsStop::IterMax;

    for (std::size_t iter = 1; iter <= o.iter_max; ++iter) {
        VnsTraceRow row;
        row.iter = iter;
        row.temperature = T;
        bool best_improved = false;

        ShakeResult s;
        bool have = false;
        for (std::size_t a = 0; a < o.max_shake_attempts; ++a) {
            s = shake(x, ctx, shake_rng, o.shake);
            if (s.status != ShakeStatus::Ok) { ++R.shakes_not_applicable; continue; }
            if (!s.feasible) { ++R.shakes_infeasible; continue; }
            have = true;
            break;
        }
        if (!have) {
            ++R.iterations_skipped;
            row.end = VnsPassEnd::Skipped;
        } else {
            ++R.shakes_applied;
            ++R.shakes_applied_by_kind[static_cast<std::size_t>(s.kind)];
            row.shaken = true;
            row.shake_kind = static_cast<int>(s.kind);
            tdr::model::Solution w = std::move(s.solution);
            double fw = s.objective;
            if (fw < fbest) { best = w; fbest = fw; ++R.best_updates; R.last_best_iteration = iter; best_improved = true; }
            std::size_t k = 0, steps = 0;
            bool truncated = false;
            while (k < list.size()) {
                if (o.max_local_steps > 0 && steps >= o.max_local_steps) { ++R.passes_truncated; truncated = true; break; }
                ++steps;
                const OperatorId op = list[k];
                OperatorResult r = detail_vns::apply(op, w, ctx, vns_rng, o);
                VnsOpStats& st = R.ops[static_cast<std::size_t>(op)];
                ++st.calls;
                if (r.status != OperatorStatus::Ok) { ++st.no_anchor; ++k; continue; }
                if (!r.best) { ++st.no_candidate; ++k; continue; }
                const double fz = r.best->objective;
                const double ref = o.reference == AcceptanceReference::Incumbent ? fx : fw;
                bool accepted = false, improving = false;
                if (fz < ref - o.min_improvement) {
                    accepted = true;
                    improving = true;
                    ++st.improving;
                    ++R.improving_moves;
                } else {
                    const double u = vns_rng.uniform01();
                    if (u <= std::exp((ref - fz) / T)) { accepted = true; ++st.sa_accepted; ++R.sa_accepted_moves; }
                    else ++st.sa_rejected;
                }
                if (accepted) {
                    w = std::move(r.best->solution);
                    fw = fz;
                    ++row.moves;
                    if (o.reference == AcceptanceReference::Incumbent) { x = w; fx = fw; }
                    if (fw < fbest) { best = w; fbest = fw; ++R.best_updates; R.last_best_iteration = iter; best_improved = true; }
                }
                k = improving ? 0 : k + 1;
            }
            if (o.reference == AcceptanceReference::ShakenState) { x = std::move(w); fx = fw; }
            row.local_steps = steps;
            R.local_steps_total += steps;
            R.max_local_steps_in_pass = std::max(R.max_local_steps_in_pass, steps);
            if (truncated) row.end = VnsPassEnd::GuardTruncated; else ++R.passes_completed;
        }
        R.iterations = iter;
        row.current = fx;
        row.best = fbest;
        if (o.record_trace) R.trace.push_back(row);
        since_best = best_improved ? 0 : since_best + 1;
        T = std::max(o.t_min, o.cooling_rate * T);
        if (o.max_no_improve > 0 && since_best >= o.max_no_improve) { R.stop = VnsStop::NoImprovement; break; }
        if (o.max_seconds > 0.0 && std::chrono::duration<double>(Clock::now() - start).count() >= o.max_seconds) { R.stop = VnsStop::TimeLimit; break; }
    }
    R.seconds = std::chrono::duration<double>(Clock::now() - start).count();
    R.final_temperature = T;
    R.best = std::move(best);
    R.best_objective = fbest;
    R.current = std::move(x);
    R.current_objective = fx;
    return R;
}

} // namespace tdr::search
