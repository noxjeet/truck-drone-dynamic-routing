// M7 unit tests: the VNS driver (include/search/vns.hpp), contract docs/static/vns.md. Line-world fixtures for the exact cases (no applicable shake, iteration counting,
// temperature schedule, statistics) and generated instances for behavioural properties; every result is re-judged by the FROZEN reference evaluator.
#include "config/static_config.hpp"
#include "flp/flexible_launch_point_generator.hpp"
#include "flp/flp_arc_index.hpp"
#include "graph/spatial_network.hpp"
#include "init/initial_solution_builder.hpp"
#include "instance/static_instance.hpp"
#include "model/customer.hpp"
#include "model/drone_trip.hpp"
#include "model/node.hpp"
#include "model/parameters.hpp"
#include "model/solution.hpp"
#include "reference/timing_engine_reference.hpp"
#include "search/candidate_window.hpp"
#include "search/vns.hpp"
#include "search/route_ops.hpp"
#include "timing/timing_engine.hpp"
#include "util/rng.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

using namespace tdr::model;
using namespace tdr::search;

namespace {

int g_failures = 0;
int g_passes = 0;

void check(bool condition, const std::string& description) {
    if (condition) {
        ++g_passes;
        std::cout << "[PASS] " << description << "\n";
    } else {
        ++g_failures;
        std::cout << "[FAIL] " << description << "\n";
    }
}


struct Fix {
    NodeRegistry nodes;
    std::unordered_map<NodeId, CustomerData> cd;
    tdr::timing::FlpDataMap flp;
    std::optional<tdr::graph::SpatialNetwork> net;
    tdr::flp::FlpArcIndex idx;
    int n = 0;

    SearchContext ctx(int w) const { return SearchContext{*net, cd, flp, idx, w}; }
    NodeId flp_at(NodeId a, NodeId b, std::size_t k) const { return idx.flps_on_arc(a, b).at(k); }
    double flp_x(NodeId f) const { return nodes.at(f).coordinates.x; }
};

std::unique_ptr<Fix> make_line(int n, int drones, double endurance = 3600.0, double d = 600.0, double spacing = 1000.0, double drone_speed = 20.0) {
    auto f = std::make_unique<Fix>();
    f->n = n;
    f->nodes.add(Node{0, NodeType::Depot, Point2D{0, 0}});
    for (int i = 1; i <= n; ++i) f->nodes.add(Node{i, NodeType::Customer, Point2D{spacing * i, 0}});
    std::vector<tdr::flp::Arc> arcs;
    for (int a = 0; a <= n; ++a)
        for (int b = 0; b <= n; ++b)
            if (a != b) arcs.emplace_back(a, b);
    tdr::flp::FlpGenerationPolicy pol;
    pol.unit_division_length = d;
    const auto gen = tdr::flp::generate_flexible_launch_points_on_arcs(arcs, f->nodes, pol, 100);
    for (const auto& p : gen.points) {
        f->nodes.add(p.node);
        f->flp[p.node.id] = p.data;
    }
    for (int i = 1; i <= n; ++i) {
        CustomerData c;
        c.node_id = i;
        c.truck_service_time = 0;
        c.drone_service_time = 0;
        c.generalized_flight_time_theta = 0;
        f->cd[i] = c;
    }
    TruckParameters tp;
    tp.speed = 10;
    DroneParameters dp;
    dp.speed = drone_speed;
    dp.endurance_seconds = endurance;
    dp.launch_prep_time_sl = 0;
    dp.recovery_time_sr = 0;
    dp.num_drones = drones;
    f->net.emplace(f->nodes, tp, dp);
    f->idx = tdr::flp::FlpArcIndex(f->flp);
    return f;
}

DroneTrip trip(int drone, NodeId launch, NodeId customer, NodeId recovery) {
    DroneTrip t;
    t.drone_id = drone;
    t.launch_node = launch;
    t.service_customer = customer;
    t.recovery_node = recovery;
    return t;
}

// Each customer 1..n appears exactly once, either on the truck route or as a trip's service customer.
bool exactly_once(const Solution& s, int n) {
    std::vector<int> count(static_cast<std::size_t>(n) + 1, 0);
    for (NodeId v : s.truck_route) if (v >= 1 && v <= n) ++count[static_cast<std::size_t>(v)];
    for (const auto& t : s.drone_trips) if (t.service_customer >= 1 && t.service_customer <= n) ++count[static_cast<std::size_t>(t.service_customer)];
    for (int i = 1; i <= n; ++i) if (count[static_cast<std::size_t>(i)] != 1) return false;
    return true;
}

double ref_objective(const Solution& s, const Fix& f, bool* feasible) {
    const auto e = tdr::timing_ref::evaluate_solution(s, *f.net, f.cd, f.flp);
    if (feasible) *feasible = e.feasible;
    return e.objective_value;
}


} // namespace

namespace {

// A generated instance with its initial solution (the paper-scale pipeline), owned in one place.
struct Gen {
    tdr::instance::StaticInstance inst;
    tdr::init::InitialSolutionResult init;
    std::optional<tdr::graph::SpatialNetwork> net;
    tdr::flp::FlpArcIndex idx;
    bool ok = false;
    SearchContext ctx(int w = 3) const { return SearchContext{*net, inst.customer_data, init.flp_data, idx, w}; }
};

std::unique_ptr<Gen> make_gen(int n, int dataset, double endurance_mult = 1.0, double side = 0.0) {
    auto g = std::make_unique<Gen>();
    tdr::config::StaticConfig cfg;
    cfg.seed.base_seed = 7777;
    cfg.instance.num_customers = n;
    cfg.instance.dataset_index = dataset;
    cfg.vehicle.endurance_s *= endurance_mult;
    if (side > 0) cfg.instance.area_side_m = side;
    const auto r = tdr::instance::generate_static_instance(cfg);
    if (!r.ok) return g;
    g->inst = r.instance;
    const auto net0 = g->inst.make_network();
    tdr::init::InitialSolutionPolicy pol;
    pol.flp_policy.unit_division_length = cfg.flp.unit_division_length_m;
    g->init = tdr::init::build_initial_solution(g->inst.depot, g->inst.customers, g->inst.customer_data, net0, pol);
    if (!g->init.feasible) return g;
    g->net.emplace(g->init.node_registry, g->inst.truck, g->inst.drone);
    g->idx = tdr::flp::FlpArcIndex(g->init.flp_data);
    g->ok = true;
    return g;
}

double ref_obj(const Gen& g, const Solution& s, bool* feasible) {
    const auto e = tdr::timing_ref::evaluate_solution(s, *g.net, g.inst.customer_data, g.init.flp_data);
    if (feasible) *feasible = e.feasible;
    return e.objective_value;
}

bool same_route(const Solution& a, const Solution& b) {
    if (a.truck_route != b.truck_route || a.drone_trips.size() != b.drone_trips.size()) return false;
    for (std::size_t i = 0; i < a.drone_trips.size(); ++i) {
        const auto &x = a.drone_trips[i], &y = b.drone_trips[i];
        if (x.drone_id != y.drone_id || x.launch_node != y.launch_node || x.service_customer != y.service_customer || x.recovery_node != y.recovery_node) return false;
    }
    return true;
}

bool same_trace(const VnsResult& a, const VnsResult& b) {
    if (a.trace.size() != b.trace.size()) return false;
    for (std::size_t i = 0; i < a.trace.size(); ++i) {
        const auto &x = a.trace[i], &y = b.trace[i];
        if (x.iter != y.iter || x.temperature != y.temperature || x.shaken != y.shaken || x.shake_kind != y.shake_kind || x.current != y.current || x.best != y.best || x.moves != y.moves) return false;
    }
    return true;
}

// internal consistency of the statistics of one result
bool consistent(const VnsResult& r) {
    std::size_t kinds = 0, moves = 0;
    for (auto k : r.shakes_applied_by_kind) kinds += k;
    bool ok = kinds == r.shakes_applied && r.iterations == r.shakes_applied + r.iterations_skipped;
    std::size_t imp = 0, sa = 0;
    for (const auto& o : r.ops) {
        ok = ok && o.calls == o.no_anchor + o.no_candidate + o.improving + o.sa_accepted + o.sa_rejected;
        imp += o.improving;
        sa += o.sa_accepted;
    }
    ok = ok && imp == r.improving_moves && sa == r.sa_accepted_moves;
    for (const auto& t : r.trace) moves += t.moves;
    if (!r.trace.empty()) ok = ok && moves == imp + sa && r.trace.size() == r.iterations;
    return ok;
}

void check_result(const Gen& g, const VnsResult& r, const std::string& tag) {
    bool fb = false, fc = false;
    const double ob = ref_obj(g, r.best, &fb), oc = ref_obj(g, r.current, &fc);
    check(fb && fc, tag + ": best and current are feasible for the frozen reference");
    check(std::fabs(ob - r.best_objective) <= 1e-9 * std::max(1.0, ob) && std::fabs(oc - r.current_objective) <= 1e-9 * std::max(1.0, oc), tag + ": best and current objectives equal the frozen reference's");
    check(r.best_objective <= r.initial_objective, tag + ": best-so-far is never worse than the initial solution");
    bool mono = true;
    for (std::size_t i = 1; i < r.trace.size(); ++i) mono = mono && r.trace[i].best <= r.trace[i - 1].best;
    check(mono, tag + ": the best-so-far objective never increases");
    check(consistent(r), tag + ": statistics are internally consistent");
    check(exactly_once(r.best, static_cast<int>(g.inst.customer_data.size())) && exactly_once(r.current, static_cast<int>(g.inst.customer_data.size())), tag + ": every customer served exactly once in best and current");
}

} // namespace

int main() {
    // ---------------------------------------------------------------------------------------------- list and options
    {
        const auto d = default_neighbourhood_list(2, 2);
        check(d == std::vector<OperatorId>({OperatorId::N4, OperatorId::N5, OperatorId::N8, OperatorId::N3, OperatorId::N6, OperatorId::N7, OperatorId::N1, OperatorId::N2, OperatorId::N1, OperatorId::N2}), "default neighbourhood list: Sec 6.1 order then N1, N2, N1, N2 (A32, D2)");
        check(default_neighbourhood_list(3, 1) == std::vector<OperatorId>({OperatorId::N4, OperatorId::N5, OperatorId::N8, OperatorId::N3, OperatorId::N6, OperatorId::N7, OperatorId::N1, OperatorId::N2, OperatorId::N1, OperatorId::N1}), "list with n1 = 3, n2 = 1");
        check(default_neighbourhood_list(0, 0).size() == 6 && default_neighbourhood_list(1, 0).size() == 7 && default_neighbourhood_list(1, 0).back() == OperatorId::N1, "list with no / one repeat");
        const VnsOptions o;
        check(o.iter_max == 2000 && o.cooling_rate == 0.995 && o.t0_fraction == 0.01 && o.max_shake_attempts == 10 && o.min_improvement == 0.0 && o.reference == AcceptanceReference::Incumbent && o.max_no_improve == 0 && o.max_seconds == 0.0 && o.shake.max_segment_length == 3 && o.n4.max_reanchor_variants == 100, "default options (pilot-based values, labelled assumptions; printed acceptance reference)");
        check(validate(o).empty(), "default options are valid");
        auto bad = [&](auto mut) { VnsOptions x; mut(x); return !validate(x).empty(); };
        check(bad([](VnsOptions& x) { x.cooling_rate = 0; }) && bad([](VnsOptions& x) { x.cooling_rate = 1.5; }) && bad([](VnsOptions& x) { x.t_min = 0; }) && bad([](VnsOptions& x) { x.t0_fraction = -1; }) && bad([](VnsOptions& x) { x.t0_absolute = -1; }) &&
              bad([](VnsOptions& x) { x.max_seconds = -1; }) && bad([](VnsOptions& x) { x.max_shake_attempts = 0; }) && bad([](VnsOptions& x) { x.min_improvement = -1; }) && bad([](VnsOptions& x) { x.n1_repeats = -1; }) && bad([](VnsOptions& x) { x.cooling_rate = std::nan(""); }),
              "invalid options are reported");
        check(!bad([](VnsOptions& x) { x.cooling_rate = 1.0; }) && !bad([](VnsOptions& x) { x.iter_max = 0; }), "cooling_rate = 1 and iter_max = 0 are valid");
    }

    // ---------------------------------------------------------------------------------------------- statuses on the line fixtures
    auto P = make_line(7, 2);
    Solution base;
    {
        const NodeId F23 = P->flp_at(2, 3, 0), F56 = P->flp_at(5, 6, 0);
        base.truck_route = {0, 1, 2, F23, 3, 5, F56, 6, 0};
        base.drone_trips = {trip(0, F23, 4, 5), trip(1, F56, 7, 6)};
    }
    {
        VnsOptions o;
        o.cooling_rate = 0;
        const auto r = run_vns(base, P->ctx(3), o);
        check(r.status == VnsStatus::InvalidOptions && !r.message.empty() && r.iterations == 0, "invalid options: InvalidOptions with a message, nothing run");
        Solution omitted;
        omitted.truck_route = {0, 1, 2, 0};
        const auto r2 = run_vns(omitted, P->ctx(3));
        check(r2.status == VnsStatus::InitialInfeasible && r2.iterations == 0, "an infeasible initial solution is InitialInfeasible, nothing run");
        VnsOptions z;
        z.iter_max = 0;
        const auto r3 = run_vns(base, P->ctx(3), z);
        bool fe = false;
        const double ob = ref_objective(base, *P, &fe);
        check(r3.status == VnsStatus::Ok && r3.iterations == 0 && r3.stop == VnsStop::IterMax && same_route(r3.best, base) && same_route(r3.current, base) && std::fabs(r3.best_objective - ob) < 1e-9 && r3.t0 == r3.final_temperature && fe, "iter_max = 0: the initial solution comes back unchanged, no pass");
    }

    // ---------------------------------------------------------------------------------------------- iteration count and temperature schedule
    {
        VnsOptions o;
        o.iter_max = 25;
        o.record_trace = true;
        o.t0_fraction = 0.01;
        o.cooling_rate = 0.9;
        const auto r = run_vns(base, P->ctx(3), o);
        bool fe = false;
        const double f0 = ref_objective(base, *P, &fe);
        check(r.status == VnsStatus::Ok && r.iterations == 25 && r.trace.size() == 25 && r.stop == VnsStop::IterMax, "exactly iter_max passes are run (D8: the counter advances) and the trace has one row per pass");
        bool rows = true, temps = true;
        double T = 0.01 * f0;
        for (std::size_t i = 0; i < r.trace.size(); ++i) {
            rows = rows && r.trace[i].iter == i + 1;
            temps = temps && std::fabs(r.trace[i].temperature - T) <= 1e-12 * T;
            T = std::max(1e-9, 0.9 * T);
        }
        check(rows && temps, "T of pass i is T0 * alpha^(i-1) with T0 = t0_fraction * f(initial): one cooling step per pass");
        check(std::fabs(r.t0 - 0.01 * f0) <= 1e-12 * f0 && std::fabs(r.final_temperature - T) <= 1e-12 * T, "T0 and the final temperature are reported");
        VnsOptions a;
        a.iter_max = 3;
        a.t0_absolute = 500.0;
        a.cooling_rate = 0.5;
        a.record_trace = true;
        const auto ra = run_vns(base, P->ctx(3), a);
        check(ra.t0 == 500.0 && ra.trace[0].temperature == 500.0 && ra.trace[1].temperature == 250.0 && ra.trace[2].temperature == 125.0 && ra.final_temperature == 62.5, "absolute T0 and geometric cooling: 500, 250, 125 -> 62.5");
        VnsOptions fl;
        fl.iter_max = 5;
        fl.t0_absolute = 1e-8;
        fl.t_min = 1e-9;
        fl.cooling_rate = 0.01;
        const auto rf = run_vns(base, P->ctx(3), fl);
        check(rf.final_temperature == 1e-9, "the temperature never falls below t_min");
        check(consistent(r) && consistent(ra), "statistics consistent");
    }

    // ---------------------------------------------------------------------------------------------- shakes that cannot act: the pass is skipped
    {
        auto one = make_line(1, 1);
        Solution s;
        s.truck_route = {0, 1, 0};
        VnsOptions o;
        o.iter_max = 7;
        o.max_shake_attempts = 4;
        o.record_trace = true;
        const auto r = run_vns(s, one->ctx(3), o);
        check(r.status == VnsStatus::Ok && r.iterations == 7 && r.iterations_skipped == 7 && r.shakes_applied == 0 && r.shakes_not_applicable == 28 && r.shakes_infeasible == 0, "no applicable shake (one truck customer, no trip): every pass is skipped after max_shake_attempts NotApplicable draws; passes still count");
        bool rows = true;
        for (const auto& t : r.trace) rows = rows && !t.shaken && t.moves == 0 && t.current == r.initial_objective && t.best == r.initial_objective;
        check(rows && same_route(r.best, s) && same_route(r.current, s) && r.final_temperature <= r.t0, "a skipped pass leaves x and best unchanged, the temperature cools anyway");
        for (const auto& op : r.ops) check(op.calls == 0, "no local search runs in a skipped pass");
    }

    // ---------------------------------------------------------------------------------------------- generated instances: behavioural properties
    {
        auto g = make_gen(12, 0, 3.0, 15000.0);
        check(g->ok, "generated instance (n = 12, larger area, endurance x3: trips and accepted moves occur)");
        if (g->ok) {
            const auto ctx = g->ctx(3);
            // determinism and run index
            VnsOptions o;
            o.iter_max = 40;
            o.record_trace = true;
            const auto a = run_vns(g->init.solution, ctx, o), b = run_vns(g->init.solution, ctx, o);
            check(same_trace(a, b) && same_route(a.best, b.best) && same_route(a.current, b.current) && a.shakes_not_applicable == b.shakes_not_applicable, "same options, seed and run index: bit-identical trace and result");
            VnsOptions o1 = o;
            o1.run_index = 1;
            const auto c = run_vns(g->init.solution, ctx, o1);
            VnsOptions o2 = o;
            o2.base_seed = 5;
            const auto d = run_vns(g->init.solution, ctx, o2);
            check(!same_trace(a, c) && !same_trace(a, d), "another run index or base seed gives another trajectory");
            check_result(*g, a, "default");
            check(a.improving_moves + a.sa_accepted_moves > 0 && a.shakes_applied > 20, "non-vacuity: moves are accepted and shakes applied");
            // pure descent (T tiny): x never worsens, x == best
            VnsOptions pd = o;
            pd.t0_absolute = 1e-9;
            pd.cooling_rate = 1.0;
            const auto p = run_vns(g->init.solution, ctx, pd);
            bool desc = true;
            for (const auto& t : p.trace) desc = desc && t.current == t.best;
            for (std::size_t i = 1; i < p.trace.size(); ++i) desc = desc && p.trace[i].current <= p.trace[i - 1].current;
            check(desc && p.current_objective == p.best_objective, "T ~ 0 (Incumbent): the accepted solution never worsens (only improvements and exact ties), x == best-so-far");
            check_result(*g, p, "pure descent");
            // T huge: every non-improving candidate with a feasible candidate is accepted
            VnsOptions hot = o;
            hot.t0_absolute = 1e30;
            hot.cooling_rate = 1.0;
            hot.iter_max = 15;
            hot.max_local_steps = 60;
            const auto h = run_vns(g->init.solution, ctx, hot);
            std::size_t rej = 0;
            for (const auto& s : h.ops) rej += s.sa_rejected;
            check(rej == 0 && h.sa_accepted_moves > 0, "T huge: no non-improving candidate is rejected (u <= exp(-small) ~ 1)");
            check(h.passes_truncated > 0 || h.iterations == 15, "T huge: the local-search step guard bounds the pass (A38)");
            VnsOptions g1 = hot;
            g1.max_local_steps = 1;
            const auto gr = run_vns(g->init.solution, ctx, g1);
            std::size_t calls = 0;
            for (const auto& s2 : gr.ops) calls += s2.calls;
            check(calls <= gr.shakes_applied && gr.passes_truncated + 0 <= gr.iterations, "max_local_steps = 1: at most one operator call per applied shake");
            check_result(*g, h, "hot");
            // threshold: nothing counts as improving
            VnsOptions th = o;
            th.min_improvement = 1e30;
            const auto t = run_vns(g->init.solution, ctx, th);
            check(t.improving_moves == 0, "min_improvement huge: no move is an improvement");
            check_result(*g, t, "threshold");
            // ShakenState
            VnsOptions ss = o;
            ss.reference = AcceptanceReference::ShakenState;
            const auto s1 = run_vns(g->init.solution, ctx, ss), s2 = run_vns(g->init.solution, ctx, ss);
            check(same_trace(s1, s2) && !same_trace(a, s1), "ShakenState: deterministic and a different trajectory from Incumbent");
            check_result(*g, s1, "shaken-state");
            check(s1.shakes_applied > 0 && s1.current_objective >= s1.best_objective, "ShakenState: the incumbent is the searched shaken solution (never better than the best-so-far)");
            // custom list
            VnsOptions only4 = o;
            only4.neighbourhoods = {OperatorId::N4};
            const auto f = run_vns(g->init.solution, ctx, only4);
            std::size_t others = 0;
            for (std::size_t i = 0; i < f.ops.size(); ++i) if (i != static_cast<std::size_t>(OperatorId::N4)) others += f.ops[i].calls;
            check(others == 0 && f.ops[static_cast<std::size_t>(OperatorId::N4)].calls > 0, "a custom neighbourhood list is honoured");
            // restart on improvement: with the default list, after an improving move the next call is N4 again => N4 is called at least as often as N7
            const auto& os = a.ops;
            check(os[static_cast<std::size_t>(OperatorId::N4)].calls >= os[static_cast<std::size_t>(OperatorId::N7)].calls && os[static_cast<std::size_t>(OperatorId::N1)].calls >= os[static_cast<std::size_t>(OperatorId::N7)].calls, "sequential list with restart: earlier neighbourhoods are called at least as often as the last ones");
            // stopping
            VnsOptions ni = o;
            ni.iter_max = 400;
            ni.max_no_improve = 3;
            ni.t0_absolute = 1e-9;
            const auto n = run_vns(g->init.solution, ctx, ni);
            check(n.stop == VnsStop::NoImprovement && n.iterations == n.last_best_iteration + 3 && n.iterations < 400, "no-improvement stop: exactly max_no_improve passes after the last best-so-far update");
            VnsOptions tl = o;
            tl.max_seconds = 1e-12;
            const auto q = run_vns(g->init.solution, ctx, tl);
            check(q.stop == VnsStop::TimeLimit && q.iterations == 1, "time limit: stops after the pass that exceeds it");
            // the printed iteration semantic: iter_max passes with nothing else limiting
            check(a.stop == VnsStop::IterMax && a.iterations == 40, "iter_max passes are completed");
            // seed streams are independent: changing shake options changes shakes but a run without any shake use of vns stream is unaffected
            VnsOptions sh = o;
            sh.shake.max_segment_length = 1;
            sh.shake.max_shift = 1;
            const auto sr = run_vns(g->init.solution, ctx, sh);
            check_result(*g, sr, "L_max = D_max = 1");
        }
    }

    // ---------------------------------------------------------------------------------------------- an instance with an eligible-trip-free start still works
    {
        auto g = make_gen(8, 3);
        if (g->ok) {
            Solution alltruck;
            alltruck.truck_route.push_back(0);
            for (int i = 1; i <= 8; ++i) alltruck.truck_route.push_back(i);
            alltruck.truck_route.push_back(0);
            bool fe = false;
            ref_obj(*g, alltruck, &fe);
            if (fe) {
                VnsOptions o;
                o.iter_max = 30;
                o.record_trace = true;
                const auto r = run_vns(alltruck, g->ctx(3), o);
                check_result(*g, r, "all-truck start");
                check(r.shakes_not_applicable > 0 || r.shakes_applied > 0, "all-truck start: Drone_Remove is NotApplicable and redrawn, route shakes apply");
            }
        }
    }

    // ---------------------------------------------------------------------------------------------- A38: the step guard is reported, never silent
    {
        auto g = make_gen(12, 0, 3.0, 15000.0);
        check(g->ok, "guard fixture generated");
        if (g->ok) {
            const auto ctx = g->ctx(3);
            auto pass_ends_ok = [](const VnsResult& r, std::size_t limit) {
                std::size_t c = 0, sk = 0, tr = 0, steps = 0, mx = 0;
                bool ok = r.trace.size() == r.iterations;
                for (const auto& t : r.trace) {
                    steps += t.local_steps;
                    mx = std::max(mx, t.local_steps);
                    if (t.end == VnsPassEnd::Completed) { ++c; ok = ok && t.shaken; }
                    else if (t.end == VnsPassEnd::Skipped) { ++sk; ok = ok && !t.shaken && t.local_steps == 0; }
                    else { ++tr; ok = ok && t.shaken && limit > 0 && t.local_steps == limit; }
                    if (limit > 0) ok = ok && t.local_steps <= limit;
                }
                return ok && c == r.passes_completed && sk == r.iterations_skipped && tr == r.passes_truncated && steps == r.local_steps_total && mx == r.max_local_steps_in_pass && r.iterations == c + sk + tr;
            };
            std::size_t calls_of = 0;
            auto total_calls = [&](const VnsResult& r) { calls_of = 0; for (const auto& st : r.ops) calls_of += st.calls; return calls_of; };
            // default guard on the default schedule: nothing is truncated, the result says so
            VnsOptions n;
            n.iter_max = 40;
            n.record_trace = true;
            const auto nr = run_vns(g->init.solution, ctx, n);
            check(!nr.guard_triggered() && nr.passes_truncated == 0 && nr.passes_completed == nr.shakes_applied && pass_ends_ok(nr, n.max_local_steps), "normal run: no guard trigger, every applied pass is Completed, outcomes consistent with the counters");
            check(total_calls(nr) == nr.local_steps_total && nr.max_local_steps_in_pass <= n.max_local_steps, "local_steps_total equals the operator calls made");
            // guard = 1: every applied pass (the list has 10 entries) is cut off after one call and says so
            VnsOptions g1 = n;
            g1.max_local_steps = 1;
            const auto r1 = run_vns(g->init.solution, ctx, g1);
            check(r1.guard_triggered() && r1.shakes_applied > 0 && r1.passes_truncated == r1.shakes_applied && r1.passes_completed == 0, "max_local_steps = 1: every applied pass is reported GuardTruncated, none Completed");
            check(pass_ends_ok(r1, 1) && total_calls(r1) == r1.shakes_applied && r1.max_local_steps_in_pass == 1, "guard-truncated passes made exactly max_local_steps calls (outcome rows, counters, call totals agree)");
            check(r1.stop == VnsStop::IterMax && r1.iterations == g1.iter_max, "a guard trigger ends one pass's local search, not the run (stop reason stays IterMax; the trigger is carried by the counters)");
            check_result(*g, r1, "guard=1");
            // effect: the truncated search is shorter and weaker, never invalid
            check(total_calls(r1) < nr.local_steps_total, "the guard shortens the search (fewer operator calls than the unguarded run)");
            // guard exactly large enough: a pass that ends normally in <= limit calls is not flagged
            VnsOptions one = n;
            one.neighbourhoods = {OperatorId::N4};
            one.max_local_steps = 1;
            const auto ro = run_vns(g->init.solution, ctx, one);
            check(pass_ends_ok(ro, 1) && ro.passes_completed + ro.passes_truncated == ro.shakes_applied, "single-operator list, guard 1: a non-improving call ends the pass normally (Completed), an improving one (restart) is reported GuardTruncated");
            // 0 = unlimited: never truncates
            VnsOptions u = n;
            u.max_local_steps = 0;
            const auto ru = run_vns(g->init.solution, ctx, u);
            check(!ru.guard_triggered() && pass_ends_ok(ru, 0), "max_local_steps = 0 (unlimited): never reports a trigger");
            // very hot run with a modest guard: the trigger count is the diagnostic
            VnsOptions hot = n;
            hot.t0_absolute = 1e30;
            hot.cooling_rate = 1.0;
            hot.iter_max = 15;
            hot.max_local_steps = 25;
            const auto rh = run_vns(g->init.solution, ctx, hot);
            check(pass_ends_ok(rh, 25), "hot run: outcomes consistent under the guard");
            check_result(*g, rh, "hot guard");
            // skipped passes are distinguishable from the others
            auto one_c = make_line(1, 1);
            Solution s1;
            s1.truck_route = {0, 1, 0};
            VnsOptions sk;
            sk.iter_max = 5;
            sk.max_shake_attempts = 2;
            sk.record_trace = true;
            const auto rs = run_vns(s1, one_c->ctx(3), sk);
            check(rs.iterations_skipped == 5 && rs.passes_completed == 0 && rs.passes_truncated == 0 && !rs.guard_triggered() && pass_ends_ok(rs, sk.max_local_steps), "skipped passes: outcome Skipped, zero local steps, not counted as completed or truncated");
        }
    }

    std::cout << "\n" << g_passes << " passed, " << g_failures << " failed\n";
    if (g_failures == 0) std::cout << "ALL TESTS PASSED\n";
    return g_failures == 0 ? 0 : 1;
}
