#pragma once

// Dynamic extension (DM5): promise model PM1 + PM2 and the predicate Phi (DM0 §7).
//   PM1 response window W:     ETA_r <= tau_r + W            (tau_r = ARRIVAL time: queue waiting consumes W)
//   PM2 tolerance E_class:     ETA'_i <= P_i + E_class(i)    for every promised customer i
//   P_i is the FIRST promised ETA (no ratchet, D5): base customers = ETA in the executed base plan at t = 0; a new customer = its ETA at
//   acceptance. Base and new customers carry E_base / E_new (default equal, D4). Reporting is always split by class.
// With PM disabled (A0, PM0) W = E = +infinity and Phi reduces to evaluator feasibility.

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <unordered_map>

#include "dynamic/plan.hpp"

namespace tdr::dynamic {

inline constexpr double kInfinity = std::numeric_limits<double>::infinity();

enum class Klass : std::uint8_t { Base, New };

struct PromiseParams {
    bool enabled = false;       // false = PM0
    double W = kInfinity;
    double E_base = kInfinity, E_new = kInfinity;
    double tolerance(Klass k) const { return !enabled ? kInfinity : (k == Klass::Base ? E_base : E_new); }
};

struct PromiseEntry {
    Klass klass = Klass::Base;
    double first_promised_eta = 0.0; // P_i
    double tau_arrival = 0.0;
};

class PromiseLedger {
public:
    void set(tdr::model::NodeId id, Klass k, double first_promised_eta, double tau) { m_[id] = PromiseEntry{k, first_promised_eta, tau}; }
    const PromiseEntry* find(tdr::model::NodeId id) const { const auto it = m_.find(id); return it == m_.end() ? nullptr : &it->second; }
    const std::map<tdr::model::NodeId, PromiseEntry>& all() const { return m_; }
    std::size_t size() const { return m_.size(); }

    // Base promises: every customer's ETA in the executed base plan at t = 0.
    static PromiseLedger from_base_plan(const InstalledPlan& base) {
        PromiseLedger L;
        for (const auto& cr : base.eval().customer_records) L.set(cr.customer_id, Klass::Base, cr.service_arrival_time, 0.0);
        return L;
    }

private:
    std::map<tdr::model::NodeId, PromiseEntry> m_;
};

inline std::unordered_map<tdr::model::NodeId, double> eta_map(const tdr::timing::SolutionEvaluationResult& e) {
    std::unordered_map<tdr::model::NodeId, double> m;
    for (const auto& cr : e.customer_records) m[cr.customer_id] = cr.service_arrival_time;
    return m;
}

// slack_i = P_i + E_class - ETA_i in plan `p` (+inf with PM0 or if i has no promise).
inline double slack_of(const std::unordered_map<tdr::model::NodeId, double>& eta, const PromiseLedger& L, const PromiseParams& pp, tdr::model::NodeId id) {
    if (!pp.enabled) return kInfinity;
    const PromiseEntry* e = L.find(id);
    const auto it = eta.find(id);
    if (!e || it == eta.end()) return kInfinity;
    return e->first_promised_eta + pp.tolerance(e->klass) - it->second;
}

struct PhiResult {
    bool ok = true;
    std::string reason;
};

// Phi(S*; r): evaluator-feasible (guaranteed by InstalledPlan), commitments I2 (checked separately by check_single_insertion), the response
// window for the new customer, and the tolerance for every promised customer. `eps` absorbs floating-point rounding only.
inline PhiResult phi(const InstalledPlan& cand, const PromiseLedger& L, const PromiseParams& pp, tdr::model::NodeId new_id, double tau_r, double eps = 1e-9) {
    PhiResult R;
    if (!cand.eval().feasible) { R.ok = false; R.reason = "evaluator-infeasible"; return R; }
    if (!pp.enabled) return R;
    const auto eta = eta_map(cand.eval());
    const auto it = eta.find(new_id);
    if (it == eta.end()) { R.ok = false; R.reason = "new customer missing"; return R; }
    if (it->second > tau_r + pp.W + eps * std::max(1.0, std::fabs(tau_r + pp.W))) { R.ok = false; R.reason = "response window W violated"; return R; }
    for (const auto& kv : L.all()) {
        const auto e = eta.find(kv.first);
        if (e == eta.end()) { R.ok = false; R.reason = "promised customer " + std::to_string(kv.first) + " missing"; return R; }
        const double bound = kv.second.first_promised_eta + pp.tolerance(kv.second.klass);
        if (e->second > bound + eps * std::max(1.0, std::fabs(bound))) {
            R.ok = false;
            R.reason = std::string("tolerance violated for ") + (kv.second.klass == Klass::Base ? "base" : "new") + " customer " + std::to_string(kv.first);
            return R;
        }
    }
    return R;
}

} // namespace tdr::dynamic
