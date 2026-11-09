// DM9 informational benchmark (NOT a gate): optimiser arms on paired scenarios, scan g0.
//   * A0 (admit whenever a feasible insertion exists) isolates OPTIMISATION QUALITY: the served set is driven only by feasibility, so served sets mostly coincide across arms.
//   * A-PM (promise-gated admission) measures ADMISSION PERFORMANCE: an arm that changes a plan changes which later requests are feasible / promise-compatible, so served sets
//     diverge. The two are reported in separate sections and never pooled.
//   * Final objectives are compared ONLY between runs that served the same set of requests (a plan serving more customers has a larger objective by construction).
//   * Per scenario: accepted counts, served-set overlap with R0 (Jaccard of accepted request indices), identical-set flags, final objectives. The aggregate is followed by the number of
//     scenarios in which FR, RIC-C and FT end with IDENTICAL final objectives: where that count equals the number of comparable scenarios, the aggregate does NOT distinguish them.
// Relative numbers only. Usage: bench_dyn_opt [seeds=8] [n_customers=16]
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "dynamic/base_plan.hpp"
#include "dynamic/regional_optimiser.hpp"
#include "dynamic/runner.hpp"
#include "dynamic/simulator.hpp"

using namespace tdr::dynamic;

int main(int argc, char** argv) {
    const int seeds = argc > 1 ? std::atoi(argv[1]) : 8, n = argc > 2 ? std::atoi(argv[2]) : 16;
    const OptArm arms[5] = {OptArm::R0, OptArm::FR, OptArm::RIC_C, OptArm::FT, OptArm::B2};
    const char* nm[5] = {"R0", "FR", "RIC-C", "FT", "B2"};
    struct Agg { long runs = 0, decisions = 0, evals = 0, sweep_evals = 0, calls = 0, scope_rej = 0, cert = 0, budget_stop = 0, failed = 0, improved = 0, accepted = 0, expansions = 0; double secs = 0, gain = 0; };
    for (const char* adm_name : {"a0", "apm"}) {
        std::printf("\n================ admission %s (%s), scan g0, n0=%d, %d paired scenarios ================\n", adm_name,
                    std::string(adm_name) == "a0" ? "OPTIMISATION QUALITY: served sets driven by feasibility only" : "ADMISSION PERFORMANCE: served sets may diverge", n, seeds);
        Agg A[5];
        double rel_sum[5] = {0, 0, 0, 0, 0}; long rel_n[5] = {0, 0, 0, 0, 0}, diff_set[5] = {0, 0, 0, 0, 0};
        long scen = 0, scen_nonzero = 0, frfttie = 0, frfttie_comparable = 0, frfttie_all_three_comparable = 0;
        std::printf("%-5s %-6s | %s\n", "seed", "arm", "accepted  jaccard-vs-R0  same-set  final-objective  vs-R0(%) [same-set only]");
        for (int seed = 0; seed < seeds; ++seed) {
            DynamicConfig c;
            c.regime = 'F'; apply_regime(c);
            c.base.base.instance.num_customers = n; c.base.base.instance.dataset_index = 200 + seed; c.base.base.vehicle.num_drones = 2 + seed % 2;
            c.base.vns.iter_max = 100; c.rho = 3.0; c.omega = 10.0; c.m_max = 60; c.sigma0 = static_cast<std::uint64_t>(seed);
            c.admission = adm_name; c.scan = "g0"; c.eps_tilde = 2.0; c.w_tilde = 4.0;
            const BasePlan bp = build_base_plan(c);
            if (!bp.ok) { std::printf("%-5d base plan not built\n", seed); continue; }
            const Scenario sc = generate_scenario(scenario_params(c, c.sigma0), bp.n0, bp.reference_makespan);
            if (!sc.ok) { std::printf("%-5d scenario not generated\n", seed); continue; }
            ++scen;
            const auto scan = make_scan(c); const auto adm = make_admission(c); const PromiseParams pp = make_promise(c);
            std::vector<std::vector<std::uint32_t>> sets; std::vector<double> fin; std::vector<bool> ok_run;
            for (int ai = 0; ai < 5; ++ai) {
                RegionalOptimizer ro(OptConfig{}); ro.cfg.arm = arms[ai];
                SimOptions so; so.omega = c.omega; so.scan = scan.get(); so.admission = adm.get(); so.promise = pp; so.optimizer = &ro;
                const auto t0 = std::chrono::steady_clock::now();
                const SimResult r = simulate(*bp.plan, sc, so);
                A[ai].secs += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                ++A[ai].runs;
                std::vector<std::uint32_t> set; for (const auto& rec : r.records) if (rec.outcome == Outcome::Accepted) set.push_back(rec.request);
                std::sort(set.begin(), set.end());
                A[ai].accepted += static_cast<long>(set.size());
                for (const auto& o : ro.records) {
                    ++A[ai].decisions; A[ai].evals += static_cast<long>(o.evaluator_calls); A[ai].sweep_evals += static_cast<long>(o.sweep_evaluator_calls);
                    A[ai].calls += static_cast<long>(o.search_calls); A[ai].scope_rej += static_cast<long>(o.scope.scope_rejected);
                    A[ai].cert += o.cert == CertStatus::Certified; A[ai].budget_stop += o.cert == CertStatus::BudgetStop; A[ai].failed += o.failed;
                    A[ai].expansions += static_cast<long>(o.expansions);
                    if (o.f_after < o.f_before) { ++A[ai].improved; A[ai].gain += o.f_before - o.f_after; }
                }
                sets.push_back(set); fin.push_back(r.final_objective); ok_run.push_back(r.ok);
            }
            if (!sets[0].empty()) ++scen_nonzero;
            for (int ai = 0; ai < 5; ++ai) {
                std::vector<std::uint32_t> inter, uni;
                std::set_intersection(sets[ai].begin(), sets[ai].end(), sets[0].begin(), sets[0].end(), std::back_inserter(inter));
                std::set_union(sets[ai].begin(), sets[ai].end(), sets[0].begin(), sets[0].end(), std::back_inserter(uni));
                const double jac = uni.empty() ? 1.0 : static_cast<double>(inter.size()) / static_cast<double>(uni.size());
                const bool same = sets[ai] == sets[0];
                if (ai > 0) { if (same) { rel_sum[ai] += fin[ai] / fin[0] - 1.0; ++rel_n[ai]; } else ++diff_set[ai]; }
                std::printf("%-5d %-6s | %8zu  %12.3f  %8s  %15.1f  ", seed, nm[ai], sets[ai].size(), jac, ai == 0 ? "-" : (same ? "yes" : "NO"), fin[ai]);
                if (ai > 0 && same) std::printf("%+9.3f\n", 100.0 * (fin[ai] / fin[0] - 1.0)); else std::printf("%9s\n", "-");
            }
            // FR / RIC-C / FT indistinguishable in this scenario? (identical final objective AND identical served set)
            const bool tie = fin[1] == fin[2] && fin[2] == fin[3] && sets[1] == sets[2] && sets[2] == sets[3];
            if (tie) ++frfttie;
            if (sets[1] == sets[0] && sets[2] == sets[0] && sets[3] == sets[0]) { ++frfttie_all_three_comparable; if (fin[1] == fin[2] && fin[2] == fin[3]) ++frfttie_comparable; }
        }
        std::printf("\n-- aggregate (%s) --\n", adm_name);
        std::printf("%-6s %8s %9s %9s %9s %9s %8s %8s %6s %6s %6s %9s %8s\n", "arm", "decide", "improved", "gain(s)", "eval/dec", "sweep/dec", "calls/d", "scope/d", "cert", "bstop", "fail", "accepted", "sec/run");
        for (int ai = 0; ai < 5; ++ai) {
            const Agg& g = A[ai]; const double d = static_cast<double>(std::max(1L, g.decisions));
            std::printf("%-6s %8ld %9ld %9.0f %9.1f %9.1f %8.1f %8.1f %6ld %6ld %6ld %9ld %8.2f\n", nm[ai], g.decisions, g.improved, g.gain, g.evals / d, g.sweep_evals / d, g.calls / d, g.scope_rej / d, g.cert, g.budget_stop, g.failed, g.accepted, g.secs / static_cast<double>(std::max(1L, g.runs)));
        }
        std::printf("scenarios: %ld run, %ld with at least one accepted request under R0 (a scenario that accepts nothing is trivial and says nothing)\n", scen, scen_nonzero);
        std::printf("final objective vs R0 on IDENTICAL served sets only (relative):\n");
        for (int ai = 1; ai < 5; ++ai)
            std::printf("   %-6s %+.3f%% over %ld comparable runs of %ld (%ld not comparable: served set differs from R0)\n", nm[ai], rel_n[ai] ? 100.0 * rel_sum[ai] / static_cast<double>(rel_n[ai]) : 0.0, rel_n[ai], scen, diff_set[ai]);
        std::printf("FR, RIC-C and FT end with identical final objective AND served set in %ld of %ld scenarios (%ld of the %ld in which all three share R0's served set).\n", frfttie, scen, frfttie_comparable, frfttie_all_three_comparable);
        std::printf("READING GUIDE: when the tie count equals the number of comparable scenarios, the aggregate percentage above does NOT distinguish FR, RIC-C and FT; it only separates the\n"
                    "regional arms from R0 (and B2 from the rest). Differences among FR/RIC-C/FT need decision-level replay on identical inputs (DM11) and many more scenarios.\n");
        if (std::string(adm_name) == "apm")
            std::printf("A-PM note: served sets diverge once any arm changes a plan, so few runs are comparable; use this section for ADMISSION (accepted counts, rejected-promise, fallbacks), not for objective.\n");
    }
    return 0;
}
