#pragma once

// Dynamic extension (DM4 slice): event-driven simulator (DM0 §11, DM2 §5). FIFO sequential decisions; three times per request
// (arrival tau, decision start s >= tau, install s + omega). Event ties: ARRIVAL before INSTALL, so an arrival at the same instant as an
// installation is scanned against the plan installed just before it (DM0 §5.6 predictability).
// CONVENTIONS (documented, DM3): (C1) every decision that is actually made - including a rejected one - occupies the optimiser for omega
// simulated seconds; (C2) if the clean cut leaves no editable gap the request is classified `Stale` and still consumes omega.
// DM5: scan = G0/none/G3/G3c evaluated ONCE at arrival against the last installed plan + ledger; admission = A0 (PM0) or A-PM (U1 chain).
// `Expired` (DM5, PM1 only): a queued request whose window can no longer be met - decision start s with s + omega > tau_r + W, since every
// insertion is behind the cut and so has ETA > s + omega - is dropped at dequeue and consumes NO optimiser time (C3). With PM0 it never occurs.
// Wall-clock time never influences a decision.

#include <algorithm>
#include <deque>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "dynamic/admission.hpp"
#include "dynamic/cursor.hpp"
#include "dynamic/inserter.hpp"
#include "dynamic/invariants.hpp"
#include "dynamic/outcome.hpp"
#include "dynamic/promise.hpp"
#include "dynamic/scan_region.hpp"
#include "dynamic/scenario.hpp"

namespace tdr::dynamic {

struct DecisionRecord {
    std::uint32_t request = 0;
    Outcome outcome = Outcome::OutOfScan;
    double arrival = 0.0, start = 0.0, install = 0.0; // start/install = arrival for out-of-scan
    bool had_cut = false;
    std::size_t cut_skel = 0, cut_route_pos = 0;
    double cut_time = 0.0;
    std::size_t gaps_tried = 0, gaps_feasible = 0, gaps_straddled = 0, gaps_infeasible_straddled = 0, gaps_infeasible_with_flps = 0;
    int evaluator_calls = 0;
    double delta_objective = 0.0;     // accepted only
    double eta_at_install = 0.0;      // accepted only: the new customer's ETA in the installed plan
    std::uint32_t plan_version = 0;   // version after this decision
    std::size_t queue_len_at_arrival = 0;
    bool invariants_ok = true;
    Stage stage = Stage::None;        // accepted only: primary (optimised/cheapest plan satisfied Phi) or fallback
    bool phi_primary_failed = false;  // the first candidate failed Phi (outcome is then Fallback-accepted or rejected-promise)
    std::string note;
};

struct SimOptions {
    double omega = 10.0;
    const ScanRegion* scan = nullptr;
    const AdmissionPolicy* admission = nullptr;
    const Optimizer* optimizer = nullptr; // nullptr = identity (arm B1)
    PromiseParams promise;                // default: PM0 (disabled)
};

struct ClassShift { std::size_t n = 0, violated = 0; double mean = 0.0, max = 0.0; }; // ETA drift (final - first promised) over a promise class

struct SimResult {
    bool ok = true;                        // false iff an invariant violation or an internal error occurred
    std::vector<std::string> errors;
    std::vector<DecisionRecord> records;   // one per request, index order (partition: exactly one outcome each)
    std::optional<InstalledPlan> final_plan;
    double base_objective = 0.0, final_objective = 0.0;
    double base_makespan = 0.0, final_makespan = 0.0;
    std::size_t counts[6] = {0, 0, 0, 0, 0, 0};
    std::size_t max_queue = 0;
    int invariant_checks = 0;
    std::size_t stage_primary = 0, stage_fallback = 0;
    ClassShift shift_base, shift_new; // measured against the ledger whether or not promises are enforced
    std::size_t ledger_size = 0;
};

inline SimResult simulate(const InstalledPlan& base, const Scenario& sc, const SimOptions& opt) {
    SimResult R;
    const std::size_t M = sc.requests.size();
    R.records.resize(M);
    for (std::size_t i = 0; i < M; ++i) { R.records[i].request = sc.requests[i].index; R.records[i].arrival = sc.requests[i].time; }
    InstalledPlan plan = base;
    PromiseLedger ledger = PromiseLedger::from_base_plan(base); // always maintained (drift metrics); enforced only if opt.promise.enabled
    const Optimizer identity;
    const Optimizer& optimizer = opt.optimizer ? *opt.optimizer : identity;
    R.base_objective = base.objective();
    R.base_makespan = base.eval().truck_timing.arrival_time.back();

    struct Pending {
        double t = 0.0;
        std::optional<InstalledPlan> next;
        std::optional<std::pair<NodeId, PromiseEntry>> promise;
        std::size_t rec = 0;
    };
    std::optional<Pending> pending;
    std::deque<std::size_t> queue; // indices into sc.requests (FIFO by (time, index): the scenario is already sorted that way)
    std::size_t next_arrival = 0;
    constexpr double kInf = std::numeric_limits<double>::infinity();

    auto start_next = [&](double s) {
        while (!queue.empty()) {
            const std::size_t ri = queue.front();
            queue.pop_front();
            const Request& rq = sc.requests[ri];
            DecisionRecord& rec = R.records[ri];
            if (opt.promise.enabled && s + opt.omega > rq.time + opt.promise.W) { // C3: dropped at dequeue, consumes no omega
                rec.outcome = Outcome::Expired; rec.start = rec.install = s; rec.plan_version = plan.version();
                rec.note = "response window cannot be met after the queue wait";
                continue;
            }
            rec.start = s;
            rec.install = s + opt.omega;
            Pending P;
            P.t = s + opt.omega;
            P.rec = ri;
            const double theta = s + opt.omega;
            const auto cut = clean_cut(plan, theta);
            if (!cut) {
                rec.outcome = Outcome::Stale; // C2: nothing editable remains behind the cut; omega is still consumed
                rec.note = "no editable gap after the clean cut";
            } else {
                rec.had_cut = true; rec.cut_skel = cut->skel_index; rec.cut_route_pos = cut->route_pos; rec.cut_time = cut->node_time;
                const NodeId cid = customer_id(rq.index);
                const auto tentative = plan.world_ptr()->with_request(cid, rq);
                if (!tentative) {
                    R.ok = false; R.errors.push_back("id collision for request " + std::to_string(rq.index));
                    rec.outcome = Outcome::RejectedNoInsertion; rec.invariants_ok = false; rec.note = "id collision";
                } else {
                    const InsertionResult ins = cheapest_truck_insertion(plan, *cut, cid, tentative);
                    rec.evaluator_calls = ins.evaluator_calls;
                    rec.gaps_tried = ins.trials.size();
                    for (const auto& t : ins.trials) {
                        if (t.feasible) ++rec.gaps_feasible;
                        if (t.straddled) { ++rec.gaps_straddled; if (!t.feasible) ++rec.gaps_infeasible_straddled; }
                        if (!t.feasible && t.has_flps) ++rec.gaps_infeasible_with_flps;
                    }
                    const AdmissionInput in{plan, *cut, tentative, cid, rq, theta, ins, ledger, opt.promise};
                    const Decision dec = opt.admission->decide(in, optimizer);
                    rec.evaluator_calls += dec.extra_evaluator_calls;
                    rec.note = dec.note;
                    rec.phi_primary_failed = (dec.outcome == Outcome::RejectedPromise) || (dec.outcome == Outcome::Accepted && dec.stage == Stage::Fallback);
                    if (dec.outcome != Outcome::Accepted) rec.outcome = dec.outcome;
                    else {
                        std::string why;
                        auto next = InstalledPlan::make(*dec.solution, tentative, theta, plan.version() + 1, &why);
                        if (!next) {
                            R.ok = false; R.errors.push_back("request " + std::to_string(rq.index) + ": evaluator-verified candidate failed to install: " + why);
                            rec.outcome = Outcome::RejectedNoInsertion; rec.invariants_ok = false; rec.note = "install failure";
                        } else {
                            bool ok = true;
                            // I1-I6 are defined for a single truck insertion; plans produced by an optimiser are covered by Phi + the evaluator only.
                            if (dec.trial_index != static_cast<std::size_t>(-1)) {
                                const auto rep = check_single_insertion(plan, *next, *cut, ins.trials[dec.trial_index].gap.skel, cid, theta);
                                ++R.invariant_checks;
                                if (!rep.ok) { ok = false; for (const auto& v : rep.violations) R.errors.push_back("request " + std::to_string(rq.index) + ": " + v); }
                            }
                            if (ok && opt.admission->uses_promises()) { // I7: the installed plan satisfies Phi (independent re-check)
                                const PhiResult pr = phi(*next, ledger, opt.promise, cid, rq.time);
                                ++R.invariant_checks;
                                if (!pr.ok) { ok = false; R.errors.push_back("request " + std::to_string(rq.index) + ": I7 Phi violated by installed plan: " + pr.reason); }
                            }
                            if (!ok) {
                                R.ok = false;
                                rec.outcome = Outcome::RejectedNoInsertion; rec.invariants_ok = false; rec.note = "INVARIANT VIOLATION (rolled back)";
                            } else {
                                rec.outcome = Outcome::Accepted;
                                rec.stage = dec.stage;
                                rec.delta_objective = next->objective() - plan.objective();
                                bool f = false;
                                rec.eta_at_install = detail_inv::eta_of(next->eval(), cid, &f);
                                rec.plan_version = next->version();
                                P.promise = std::make_pair(cid, PromiseEntry{Klass::New, rec.eta_at_install, rq.time});
                                P.next = std::move(next);
                            }
                        }
                    }
                }
            }
            if (rec.plan_version == 0) rec.plan_version = plan.version();
            pending = std::move(P);
            return;
        }
    };

    while (next_arrival < M || pending) {
        const double ta = next_arrival < M ? sc.requests[next_arrival].time : kInf;
        const double ti = pending ? pending->t : kInf;
        if (ta <= ti) { // ARRIVAL (ties go to the arrival)
            const std::size_t ri = next_arrival++;
            const Request& rq = sc.requests[ri];
            DecisionRecord& rec = R.records[ri];
            rec.queue_len_at_arrival = queue.size() + (pending ? 1 : 0);
            R.max_queue = std::max(R.max_queue, rec.queue_len_at_arrival);
            const ScanContext sctx{plan, ledger, opt.promise, opt.omega, rq};
            if (!opt.scan->contains(sctx)) {
                rec.outcome = Outcome::OutOfScan; rec.start = rec.install = rq.time; rec.plan_version = plan.version();
            } else {
                queue.push_back(ri);
                if (!pending) start_next(rq.time);
            }
        } else { // INSTALL
            Pending P = std::move(*pending);
            pending.reset();
            if (P.next) plan = std::move(*P.next);
            if (P.promise) ledger.set(P.promise->first, P.promise->second.klass, P.promise->second.first_promised_eta, P.promise->second.tau_arrival);
            start_next(P.t);
        }
    }

    for (const auto& r : R.records) {
        ++R.counts[static_cast<int>(r.outcome)];
        if (r.outcome == Outcome::Accepted && opt.admission->uses_promises()) (r.stage == Stage::Fallback ? R.stage_fallback : R.stage_primary)++;
    }
    if (!plan.consistent()) { R.ok = false; R.errors.push_back("final plan's cached evaluation does not match its solution/world"); }
    R.final_objective = plan.objective();
    R.final_makespan = plan.eval().truck_timing.arrival_time.back();
    // drift of every promised customer against its first promised ETA, split by class (PM2 outcome metric)
    const auto eta = eta_map(plan.eval());
    for (const auto& kv : ledger.all()) {
        const auto it = eta.find(kv.first);
        if (it == eta.end()) continue;
        ClassShift& c = kv.second.klass == Klass::Base ? R.shift_base : R.shift_new;
        const double d = it->second - kv.second.first_promised_eta;
        ++c.n; c.mean += d; c.max = c.n == 1 ? d : std::max(c.max, d);
        if (opt.promise.enabled && d > opt.promise.tolerance(kv.second.klass) + 1e-9 * std::max(1.0, std::fabs(kv.second.first_promised_eta))) ++c.violated;
    }
    for (ClassShift* c : {&R.shift_base, &R.shift_new}) if (c->n) c->mean /= static_cast<double>(c->n);
    R.ledger_size = ledger.size();
    R.final_plan = plan;
    return R;
}

} // namespace tdr::dynamic
