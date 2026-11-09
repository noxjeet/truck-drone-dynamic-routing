#pragma once

// Dynamic extension: admission (DM0 §7.6-7.7).
//   A0   : accept the cheapest evaluator-feasible insertion; no promises (PM0).                         [slice baseline, unchanged behaviour]
//   A-PM : the U1 four-step chain (APPROVED):
//            (1) S+ = cheapest-F evaluator-feasible insertion;
//            (2) S* = optimiser(S+)   (DM5: the identity optimiser, i.e. arm B1; the DM9 local optimiser plugs in here);
//            (3) if Phi(S*) holds install S*; otherwise take the cheapest-F evaluator-feasible insertion plan that satisfies Phi (stage Fallback);
//            (4) if none exists: `rejected-promise`.
//        Property [P15] (tested): the acceptance set of A-PM with ANY optimiser contains the acceptance set of A-PM with the identity optimiser.
// Every installed plan is still checked by the I1-I6 invariants in the simulator (I7 = Phi is the policy outcome).

#include <algorithm>
#include <optional>
#include <vector>

#include "dynamic/inserter.hpp"
#include "dynamic/outcome.hpp"
#include "dynamic/promise.hpp"

namespace tdr::dynamic {

enum class Stage { None, Primary, Fallback };
inline const char* stage_name(Stage s) { return s == Stage::Primary ? "primary" : (s == Stage::Fallback ? "fallback" : "none"); }

struct AdmissionInput {
    const InstalledPlan& plan;
    const Cut& cut;
    const std::shared_ptr<const World>& tentative;
    tdr::model::NodeId cid;
    const Request& request;
    double theta;
    const InsertionResult& ins;
    const PromiseLedger& ledger;
    const PromiseParams& promise;
};

struct Decision {
    Outcome outcome = Outcome::RejectedNoInsertion;
    std::optional<tdr::model::Solution> solution; // iff Accepted
    Stage stage = Stage::None;
    std::size_t trial_index = static_cast<std::size_t>(-1); // index into ins.trials of the accepted single insertion; -1 if the plan came from the optimiser
    std::string note;
    int extra_evaluator_calls = 0;
    bool optimiser_failed = false; // [DM9] the optimiser threw (std or not): treated as `no plan`, S+ is used, the U1 chain continues
};

struct Optimizer { // post-insertion optimiser hook (DM9 will implement the real one); returns the plan S* to try first
    virtual ~Optimizer() = default;
    virtual std::optional<tdr::model::Solution> optimise(const AdmissionInput&, const tdr::model::Solution& s_plus) const { return s_plus; }
};

// [DM9] An optimiser exception must never bypass the U1 chain or escape into the simulator: it is treated exactly like an optimiser that returned nothing (S+ is used).
inline std::optional<tdr::model::Solution> guarded_optimise(const Optimizer& opt, const AdmissionInput& in, const tdr::model::Solution& s_plus, Decision& d) {
    try { return opt.optimise(in, s_plus); }
    catch (const std::exception& e) { d.optimiser_failed = true; d.note = std::string("optimiser exception: ") + e.what(); }
    catch (...) { d.optimiser_failed = true; d.note = "optimiser exception: non-standard"; }
    return std::nullopt;
}

struct AdmissionPolicy {
    virtual ~AdmissionPolicy() = default;
    virtual Decision decide(const AdmissionInput& in, const Optimizer& opt) const = 0;
    virtual const char* name() const = 0;
    virtual bool uses_promises() const = 0;
};

inline bool same_solution(const tdr::model::Solution& a, const tdr::model::Solution& b) {
    if (a.truck_route != b.truck_route || a.drone_trips.size() != b.drone_trips.size()) return false;
    for (std::size_t i = 0; i < a.drone_trips.size(); ++i) {
        const auto &x = a.drone_trips[i], &y = b.drone_trips[i];
        if (x.drone_id != y.drone_id || x.launch_node != y.launch_node || x.service_customer != y.service_customer || x.recovery_node != y.recovery_node) return false;
    }
    return true;
}

struct AdmitA0 final : AdmissionPolicy {
    // [DM9] A0 shares the insertion/optimisation pipeline of A-PM and differs only in Phi (none here): S* = optimiser(S+) is installed as is (the optimiser
    // returns an evaluator-feasible plan with F(S*) <= F(S+) or S+ itself). With the identity optimiser the result is exactly the DM4-DM6 one.
    Decision decide(const AdmissionInput& in, const Optimizer& opt) const override {
        Decision d;
        if (!in.ins.best) return d; // RejectedNoInsertion
        const auto sstar = guarded_optimise(opt, in, in.ins.solution, d);
        d.outcome = Outcome::Accepted;
        d.stage = Stage::None;
        if (sstar && !same_solution(*sstar, in.ins.solution)) { d.solution = *sstar; return d; } // trial_index stays -1: I1-I6 are defined for single insertions only
        d.solution = in.ins.solution;
        d.trial_index = *in.ins.best;
        return d;
    }
    const char* name() const override { return "a0"; }
    bool uses_promises() const override { return false; }
};

struct AdmitAPM final : AdmissionPolicy {
    Decision decide(const AdmissionInput& in, const Optimizer& opt) const override {
        Decision d;
        if (!in.ins.best) { d.note = "no evaluator-feasible insertion"; return d; } // RejectedNoInsertion
        const auto& s_plus = in.ins.solution;
        auto phi_of = [&](const tdr::model::Solution& s, std::string* why) {
            auto p = InstalledPlan::make(s, in.tentative, in.theta, in.plan.version() + 1, why);
            ++d.extra_evaluator_calls;
            if (!p) return false;
            const PhiResult r = phi(*p, in.ledger, in.promise, in.cid, in.request.time);
            if (!r.ok && why) *why = r.reason;
            return r.ok;
        };
        // step 2-3: S* = optimiser(S+); install it if Phi holds
        const auto sstar = guarded_optimise(opt, in, s_plus, d);
        const std::string opt_note = d.note;   // empty unless the optimiser threw
        std::string why;
        const tdr::model::Solution& cand = sstar ? *sstar : s_plus;
        if (phi_of(cand, &why)) {
            d.outcome = Outcome::Accepted; d.solution = cand; d.stage = Stage::Primary;
            if (same_solution(cand, s_plus)) d.trial_index = *in.ins.best;
            return d;
        }
        d.note = opt_note.empty() ? why : opt_note + "; " + why;
        // step 3 (fallback): cheapest evaluator-feasible insertion plan that satisfies Phi
        std::vector<std::size_t> order;
        for (std::size_t i = 0; i < in.ins.trials.size(); ++i) if (in.ins.trials[i].feasible) order.push_back(i);
        std::stable_sort(order.begin(), order.end(), [&](std::size_t x, std::size_t y) { return in.ins.trials[x].objective < in.ins.trials[y].objective; });
        for (std::size_t i : order) {
            if (same_solution(in.ins.trials[i].solution, cand)) continue; // already tried
            if (phi_of(in.ins.trials[i].solution, nullptr)) { d.outcome = Outcome::Accepted; d.solution = in.ins.trials[i].solution; d.stage = Stage::Fallback; d.trial_index = i; return d; }
        }
        d.outcome = Outcome::RejectedPromise; // step 4
        return d;
    }
    const char* name() const override { return "apm"; }
    bool uses_promises() const override { return true; }
};

} // namespace tdr::dynamic
