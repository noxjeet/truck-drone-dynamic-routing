#pragma once

// Dynamic extension (DM6 demo): human-readable plan and event trace, rendered AFTER the simulation from its records. Read-only: it never
// influences a decision, and it is deterministic (no wall-clock values). Optimiser = identity; prototype = truck-only insertion.

#include <algorithm>
#include <cstdio>
#include <string>
#include <tuple>
#include <vector>

#include "dynamic/writer.hpp"

namespace tdr::dynamic {

inline std::string node_label(const InstalledPlan& p, tdr::model::NodeId id) {
    const auto& n = p.world().nodes().at(id);
    char b[48];
    if (n.type == tdr::model::NodeType::Depot) return "D";
    if (n.type == tdr::model::NodeType::FlexibleLaunchPoint) { std::snprintf(b, sizeof b, "f%d", id - 0); return std::string("f") + std::to_string(id); }
    if (id >= kC0) { std::snprintf(b, sizeof b, "r%d", id - kC0); return b; }
    std::snprintf(b, sizeof b, "c%d", id);
    return b;
}

inline std::string plan_text(const InstalledPlan& p, const char* title) {
    std::string s = std::string(title) + " (version " + std::to_string(p.version()) + ", objective " + std::to_string(p.objective()) + ")\n  truck route: ";
    const auto& r = p.solution().truck_route;
    const auto eta = eta_map(p.eval());
    for (std::size_t i = 0; i < r.size(); ++i) {
        s += node_label(p, r[i]);
        const auto it = eta.find(r[i]);
        char b[40];
        if (it != eta.end()) { std::snprintf(b, sizeof b, "[%.0fs]", it->second); s += b; }
        if (i + 1 < r.size()) s += " -> ";
    }
    s += "\n  drone trips: ";
    if (p.solution().drone_trips.empty()) s += "none";
    for (const auto& t : p.solution().drone_trips) {
        const auto it = eta.find(t.service_customer);
        char b[64];
        std::snprintf(b, sizeof b, "[%.0fs]", it != eta.end() ? it->second : 0.0);
        s += "(launch " + node_label(p, t.launch_node) + " -> " + node_label(p, t.service_customer) + b + " -> recover " + node_label(p, t.recovery_node) + ")  ";
    }
    return s + "\n  (bracketed value = service-arrival time ETA in seconds)\n";
}

inline std::string trace_text(const DynamicConfig& cfg, const BasePlan& base, const ScenarioRun& run, const PromiseParams& pp) {
    const SimResult& R = run.sim;
    std::string s;
    char b[640];
    std::snprintf(b, sizeof b, "=== DYNAMIC TRUCK-DRONE DEMO TRACE ===\nprototype: truck-only insertion of new requests into the editable part of the plan; optimiser: IDENTITY (no re-optimisation, no improvement claimed)\n"
                  "scan region: %s | admission: %s | decision latency omega = %.0f s | scenario sigma=%llu digest %s | n0=%d, regime %c (endurance %.0f s)\n",
                  run.scan.c_str(), run.admission.c_str(), cfg.omega, static_cast<unsigned long long>(run.scenario.params.sigma), run.scenario.digest.c_str(), base.n0, cfg.regime, cfg.base.base.vehicle.endurance_s);
    s += b;
    if (pp.enabled) { std::snprintf(b, sizeof b, "promises: response window W = %.0f s (ETA_r <= arrival + W); tolerance E = %.0f s (every promised ETA' <= first promised ETA + E)\n", pp.W, pp.E_new); s += b; }
    else s += "promises: none (PM0)\n";
    s += "conventions: every decision (even a rejected one) occupies omega; no editable gap => STALE (still occupies omega); EXPIRED = window already unmeetable at dequeue, dropped, no omega\n\n";
    s += plan_text(*base.plan, "INITIAL PLAN") + "\n";
    s += "REQUESTS (arrival time, location):\n";
    for (const auto& q : run.scenario.requests) { std::snprintf(b, sizeof b, "  r%-3u t=%9.1f s  at (%7.0f, %7.0f) m\n", q.index, q.time, q.location.x, q.location.y); s += b; }
    s += "\nEVENTS (FIFO, one decision at a time):\n";
    struct Ev { double t; int ord; std::size_t i; std::string text; };
    std::vector<Ev> ev;
    for (std::size_t i = 0; i < R.records.size(); ++i) {
        const auto& d = R.records[i];
        std::snprintf(b, sizeof b, "ARRIVE   r%u  (requests waiting or in decision: %zu)%s", d.request, d.queue_len_at_arrival, d.outcome == Outcome::OutOfScan ? "  -> OUT OF SCAN: lost, never queued" : "  -> in scan, queued");
        ev.push_back({d.arrival, 0, i, b});
        if (d.outcome == Outcome::OutOfScan) continue;
        if (d.outcome == Outcome::Expired) {
            std::snprintf(b, sizeof b, "EXPIRED  r%u  dequeued after waiting %.0f s: arrival+W = %.0f s cannot be met (decision would end at %.0f s); dropped, no omega consumed", d.request, d.start - d.arrival, d.arrival + pp.W, d.start + cfg.omega);
            ev.push_back({d.start, 2, i, b});
            continue;
        }
        std::snprintf(b, sizeof b, "START    r%u  after waiting %.0f s in queue; cut after truck position %zu (t=%.0f s), %zu editable gaps tried, %zu evaluator-feasible", d.request, d.start - d.arrival, d.had_cut ? d.cut_route_pos : 0, d.cut_time, d.gaps_tried, d.gaps_feasible);
        if (!d.had_cut) std::snprintf(b, sizeof b, "START    r%u  after waiting %.0f s in queue; clean cut leaves no editable gap", d.request, d.start - d.arrival);
        ev.push_back({d.start, 2, i, b});
        std::string o;
        switch (d.outcome) {
            case Outcome::Accepted: std::snprintf(b, sizeof b, "ACCEPTED (%s): ETA %.0f s (promised, window bound %.0f s), objective +%.0f, plan -> version %u", stage_name(d.stage), d.eta_at_install, pp.enabled ? d.arrival + pp.W : 0.0, d.delta_objective, d.plan_version); break;
            case Outcome::RejectedPromise: std::snprintf(b, sizeof b, "REJECTED-PROMISE: feasible insertions exist but none satisfies the promises (first failure: %s)", d.note.c_str()); break;
            case Outcome::RejectedNoInsertion: std::snprintf(b, sizeof b, "REJECTED-NO-INSERTION: no evaluator-feasible insertion%s", d.note.empty() ? "" : (" (" + d.note + ")").c_str()); break;
            case Outcome::Stale: std::snprintf(b, sizeof b, "STALE: nothing editable remains behind the cut (the truck is too far along)"); break;
            default: std::snprintf(b, sizeof b, "?"); break;
        }
        std::string line = std::string("INSTALL  r") + std::to_string(d.request) + "  decision ends: " + b;
        ev.push_back({d.install, 1, i, line});
    }
    std::stable_sort(ev.begin(), ev.end(), [](const Ev& a, const Ev& c) { return std::tie(a.t, a.ord, a.i) < std::tie(c.t, c.ord, c.i); });
    for (const auto& e : ev) { std::snprintf(b, sizeof b, "  t=%9.1f  ", e.t); s += b + e.text + "\n"; }
    s += "\n" + plan_text(*R.final_plan, "FINAL PLAN") + "\n";
    std::snprintf(b, sizeof b, "SUMMARY: generated %zu | out-of-scan %zu | expired %zu | stale %zu | rejected-no-insertion %zu | rejected-promise %zu | accepted %zu (primary %zu, fallback %zu)\n"
                  "objective %.0f -> %.0f | makespan %.0f -> %.0f s | max queue %zu | promised-ETA drift: base customers max %.0f s, new customers max %.0f s | tolerance violations %zu | invariant checks %d | sim ok: %s\n",
                  run.scenario.requests.size(), R.counts[0], R.counts[1], R.counts[2], R.counts[3], R.counts[4], R.counts[5], R.stage_primary, R.stage_fallback, R.base_objective, R.final_objective, R.base_makespan, R.final_makespan,
                  R.max_queue, R.shift_base.max, R.shift_new.max, R.shift_base.violated + R.shift_new.violated, R.invariant_checks, R.ok ? "yes" : "NO");
    return s + b;
}

} // namespace tdr::dynamic
