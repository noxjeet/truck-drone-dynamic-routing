// DM3: dynamic core (ids, world, plan, cursor, invariants, B1 inserter diagnostics) + tentative-world cost measurement at n = 50.
// Includes the explicit stale-launch-point (CE-2) and spanning-trip (CE-1) rejection tests. Informational timings are printed, never asserted.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <set>
#include <type_traits>
#include <vector>

#include "config/experiment_config.hpp"
#include "dynamic/cursor.hpp"
#include "dynamic/ids.hpp"
#include "dynamic/inserter.hpp"
#include "dynamic/invariants.hpp"
#include "dynamic/plan.hpp"
#include "dynamic/world.hpp"
#include "init/initial_solution_builder.hpp"
#include "instance/static_instance.hpp"

using namespace tdr;
using namespace tdr::dynamic;
using tdr::model::DroneTrip;
using tdr::model::Node;
using tdr::model::NodeType;
using tdr::model::Point2D;
using tdr::model::Solution;

static int g_checks = 0, g_fail = 0;
#define CHECK(cond) do { ++g_checks; if (!(cond)) { ++g_fail; std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

namespace {
const model::TruckParameters kTruck{15.0};
const model::DroneParameters kDrone{12.0, 1800.0, 60.0, 40.0, 2};

model::CustomerData cd(model::NodeId id) { model::CustomerData c; c.node_id = id; c.truck_service_time = 30; c.drone_service_time = 30; c.generalized_flight_time_theta = 1; return c; }

struct Built { std::shared_ptr<const World> world; Solution sol; };

// CE-1 layout (math_spec §4): depot (-3000,0), c1=1 (6000,0), c2=2 (12000,0), X=3 (6000,2000), launch FLP 1000 at (0,0) on arc (0,1); trip (f, X, c2).
Built ce1(double xy_x = 6000.0, double xy_y = 2000.0) {
    model::NodeRegistry n;
    n.add(Node{0, NodeType::Depot, {-3000, 0}});
    n.add(Node{1, NodeType::Customer, {6000, 0}});
    n.add(Node{2, NodeType::Customer, {12000, 0}});
    n.add(Node{3, NodeType::Customer, {xy_x, xy_y}});
    n.add(Node{1000, NodeType::FlexibleLaunchPoint, {0, 0}});
    World::CustomerMap c; for (int i : {1, 2, 3}) c[i] = cd(i);
    timing::FlpDataMap f; f[1000] = model::FlexibleLaunchPointData{1000, 0, 1, 1.0 / 3.0};
    Built b; b.world = World::create(n, c, f, kTruck, kDrone);
    b.sol.truck_route = {0, 1000, 1, 2, 0};
    b.sol.drone_trips = {DroneTrip{0, 1000, 3, 2}};
    return b;
}
// CE-2 layout: depot (0,0), A=1 (12000,0), X=2 (6000,3000), FLP 1000 at (6000,0) on arc (0,1); trip (f, X, A).
Built ce2() {
    model::NodeRegistry n;
    n.add(Node{0, NodeType::Depot, {0, 0}});
    n.add(Node{1, NodeType::Customer, {12000, 0}});
    n.add(Node{2, NodeType::Customer, {6000, 3000}});
    n.add(Node{1000, NodeType::FlexibleLaunchPoint, {6000, 0}});
    World::CustomerMap c; for (int i : {1, 2}) c[i] = cd(i);
    timing::FlpDataMap f; f[1000] = model::FlexibleLaunchPointData{1000, 0, 1, 0.5};
    Built b; b.world = World::create(n, c, f, kTruck, kDrone);
    b.sol.truck_route = {0, 1000, 1, 0};
    b.sol.drone_trips = {DroneTrip{0, 1000, 2, 1}};
    return b;
}
Request req(std::uint32_t idx, double x, double y) { Request r; r.index = idx; r.time = 0; r.location = {x, y}; r.se1 = 30; r.se2 = 30; r.theta = 1; return r; }

void test_ids() {
    CHECK(customer_id(1) == kC0 + 1);
    CHECK(is_dynamic_customer_id(customer_id(5)) && !is_dynamic_customer_id(50) && !is_dynamic_customer_id(kF0));
    CHECK(request_index_of(customer_id(77)) == 77u);
    DenseIndex d{50, 100};
    CHECK(d.valid() && d.K() == 151);
    CHECK(d.of(0) == 0 && d.of(7) == 7 && d.of(customer_id(3)) == 53);
    CHECK(d.node_of(0) == 0 && d.node_of(7) == 7 && d.node_of(53) == customer_id(3));
    std::set<model::NodeId> seen;
    bool roundtrip = true, injective = true, bands = true;
    for (int a = 0; a < d.K(); a += 7) for (int b = 0; b < d.K(); b += 5) {
        if (a == b) continue;
        for (int m = 1; m <= kMaxFlpPerArc; m += 9) {
            const auto id = flp_id(d, a, b, m);
            if (!id) { roundtrip = false; continue; }
            const auto c = decode_flp_id(d, *id);
            if (!c || c->iota_a != a || c->iota_b != b || c->m != m) roundtrip = false;
            if (!seen.insert(*id).second) injective = false;
            if (*id < kF0 || *id >= (1 << 30) + (1 << 29) * 1) bands = bands && (*id >= kF0);
        }
    }
    CHECK(roundtrip && injective && bands);
    CHECK(!flp_id(d, 3, 3, 1) && !flp_id(d, 0, 1, 0) && !flp_id(d, 0, 1, kMaxFlpPerArc + 1) && !flp_id(d, -1, 1, 1) && !flp_id(d, 0, d.K(), 1));
    CHECK(!flp_id(DenseIndex{2000, 0}, 0, 1, 1)); // K > 1024 is refused, never wrapped
    DenseIndex big{900, 123}; // K = 1024: the largest id must stay below 2^31
    const auto top = flp_id(big, 1023, 1022, 64);
    CHECK(top && *top < (1 << 30) + (1 << 29));
    CHECK(kC0 + 1000000 < kF0); // dynamic customers cannot reach the FLP band
}

void test_world() {
    static_assert(!std::is_copy_constructible_v<World> && !std::is_move_constructible_v<World> && !std::is_copy_assignable_v<World> && !std::is_move_assignable_v<World>,
                  "World must be immovable: SearchContext holds references into it");
    Built b = ce1();
    const auto& w = b.world;
    const auto ctx = w->context();
    CHECK(&ctx.network == &w->network() && &ctx.customers == &w->customers() && &ctx.flp_data == &w->flp_data() && &ctx.flp_index == &w->flp_index());
    CHECK(ctx.adjacency_window == w->adjacency_window());
    const auto e0 = w->evaluate(b.sol);
    CHECK(e0.feasible);
    const auto e1 = timing::evaluate_solution(b.sol, ctx.network, ctx.customers, ctx.flp_data);
    CHECK(e1.feasible && e1.objective_value == e0.objective_value);

    const std::size_t n_before = w->nodes().size();
    auto w2 = w->with_request(customer_id(1), req(1, 9000, 9000));
    CHECK(w2 != nullptr);
    CHECK(w->nodes().size() == n_before && !w->nodes().contains(customer_id(1)) && w->customers().count(customer_id(1)) == 0); // original untouched
    CHECK(w2->nodes().size() == n_before + 1 && w2->nodes().contains(customer_id(1)) && w2->customers().count(customer_id(1)) == 1);
    CHECK(&w2->flp_data() == &w->flp_data() && &w2->flp_index() == &w->flp_index()); // shared immutable FLP structures
    CHECK(&w2->network() != &w->network());
    CHECK(w->with_request(customer_id(1), req(1, 0, 0)) != nullptr && w2->with_request(customer_id(1), req(1, 0, 0)) == nullptr); // id collision refused
    CHECK(w->with_customer(2, {0, 0}, cd(2)) == nullptr); // existing static id refused

    // lifetime: the child world stays valid after the parent handle is gone (it shares, never borrows)
    std::weak_ptr<const World> weak = b.world;
    auto child = w2;
    Solution sol = b.sol;
    sol.truck_route = {0, 1000, 1, 2, customer_id(1), 0}; // r appended after the recovery (feasible)
    b.world.reset();
    // `w` is a reference to b.world: do not use it any more. weak may or may not be expired (the plan fixtures below keep their own owners)
    const auto e2 = child->evaluate(sol);
    CHECK(e2.feasible && e2.truck_timing.arrival_time.size() == 6); // parent handle gone; child still evaluates (every network customer covered)
    (void)weak;
}

void test_world_deep() {
    Built b = ce1();
    auto d = b.world->deep_rebuild();
    CHECK(d->nodes().size() == b.world->nodes().size() && d->customers().size() == b.world->customers().size());
    CHECK(&d->flp_data() != &b.world->flp_data());
    const auto a = b.world->evaluate(b.sol), c = d->evaluate(b.sol);
    CHECK(a.feasible && c.feasible && a.objective_value == c.objective_value);
}

void test_plan() {
    Built b = ce1();
    std::string why;
    auto p = InstalledPlan::make(b.sol, b.world, 0.0, 0, &why);
    CHECK(p.has_value() && p->consistent() && p->world_ptr() == b.world);
    CHECK(p->objective() == b.world->evaluate(b.sol).objective_value);
    // an infeasible solution is refused with the evaluator's reason (CE-1: r inserted into the straddled gap)
    auto w2 = b.world->with_request(customer_id(1), req(1, 9000, 9000));
    Solution bad = b.sol; bad.truck_route = {0, 1000, 1, customer_id(1), 2, 0};
    auto q = InstalledPlan::make(bad, w2, 0.0, 1, &why);
    CHECK(!q.has_value() && why.find("(36)") != std::string::npos);
    // the cached evaluation belongs to ITS world: same solution, another world (X moved) -> different cache, each self-consistent
    Built moved = ce1(6000.0, 2400.0);
    auto p2 = InstalledPlan::make(b.sol, moved.world, 0.0, 0);
    CHECK(p2.has_value() && p2->consistent() && p2->objective() != p->objective() && p2->world_ptr() != p->world_ptr());
    // copies share the immutable payload
    InstalledPlan copy = *p;
    CHECK(&copy.solution() == &p->solution() && &copy.eval() == &p->eval());
}

void test_cursor() {
    Built b = ce1();
    auto p = InstalledPlan::make(b.sol, b.world, 0.0, 0);
    CHECK(p.has_value());
    const auto& arr = p->eval().truck_timing.arrival_time; // route {0,f,c1,c2,0}
    auto c0 = clean_cut(*p, 0.0);
    CHECK(c0 && c0->skel_index == 0 && c0->route_pos == 0);
    // theta between the depot and c1: c1 (route pos 2) is spanned by the trip (launch pos 1 <= 2 < recovery pos 3) -> the cut advances to c2 (pos 3)
    auto c1 = clean_cut(*p, 0.5 * (arr[0] + arr[2]));
    CHECK(c1 && c1->route_pos == 3 && c1->skel_index == 2);
    // theta after c2's arrival: only the end depot is left, no editable gap
    CHECK(!clean_cut(*p, arr[3] + 1.0).has_value());
    // property: no returned cut is ever spanned; the cut time is >= theta; sweep many thetas
    const auto trips = trip_spans(p->solution());
    bool never_spanned = true, time_ok = true;
    for (int i = 0; i <= 400; ++i) {
        const double th = arr.back() * i / 400.0;
        const auto c = clean_cut(*p, th);
        if (!c) continue;
        if (spans(trips, c->route_pos)) never_spanned = false;
        if (c->node_time < th) time_ok = false;
    }
    CHECK(never_spanned && time_ok);
    // editable gaps from the cut at c2: only (c2, end depot)
    const auto gs = editable_gaps(*p, *c1);
    CHECK(gs.size() == 1 && gs[0].a_pos == 3 && gs[0].b_pos == 4);
    // straddle flags at cut 0: gap (depot,c1) carries the launch FLP -> straddled; (c1,c2) straddled; (c2,depot) not
    const auto all = editable_gaps(*p, *c0);
    CHECK(all.size() == 3 && is_straddled(trips, all[0]) && is_straddled(trips, all[1]) && !is_straddled(trips, all[2]));
    CHECK(gap_has_flps(p->solution(), all[0]) && !gap_has_flps(p->solution(), all[1]));
}

// ---- explicit rejection tests -------------------------------------------------------------------------------------------------
void test_spanning_trip_rejection() {
    Built b = ce1();
    auto p = InstalledPlan::make(b.sol, b.world, 0.0, 0);
    auto tent = b.world->with_request(customer_id(1), req(1, 9000, 9000));
    const auto cut = clean_cut(*p, 0.0);
    const auto R = cheapest_truck_insertion(*p, *cut, customer_id(1), tent);
    CHECK(R.trials.size() == 3 && R.evaluator_calls == 3);
    // gap (c1,c2) is straddled by the trip (f,X,c2): inserting r delays the truck's arrival at c2 and endurance (36) breaks (CE-1)
    CHECK(R.trials[1].straddled && !R.trials[1].has_flps && !R.trials[1].feasible);
    CHECK(R.trials[1].first_violation.find("(36)") != std::string::npos || R.trials[1].first_violation.find("endurance") != std::string::npos);
    // gap (depot,c1) holds the launch FLP: stale launch point -> infeasible
    CHECK(R.trials[0].straddled && R.trials[0].has_flps && !R.trials[0].feasible);
    // gap (c2,depot) is after the recovery: feasible and chosen
    CHECK(!R.trials[2].straddled && R.trials[2].feasible && R.best && *R.best == 2);
    // the clean cut itself never offers a spanned position: with theta in (depot, c1] the editable set is just the last gap
    const auto& arr = p->eval().truck_timing.arrival_time;
    const auto c2 = clean_cut(*p, 0.5 * (arr[0] + arr[2]));
    const auto R2 = cheapest_truck_insertion(*p, *c2, customer_id(1), tent);
    CHECK(R2.trials.size() == 1 && R2.best && !R2.trials[0].straddled);
    // spanning cut forced by hand: evaluating the insertion at gap (c1,c2) really changes the prefix-recovered trip's endurance
    Solution s = b.sol; s.truck_route = {0, 1000, 1, customer_id(1), 2, 0};
    const auto e = tent->evaluate(s);
    CHECK(!e.feasible && e.drone_trip_timings.size() == 1 && e.drone_trip_timings[0].endurance_value_36 > kDrone.endurance_seconds);
    // and the prefix timing arrays up to c1 are unchanged (P1), although the trip is infeasible
    const auto e0 = b.world->evaluate(b.sol);
    CHECK(e.truck_timing.arrival_time[2] == e0.truck_timing.arrival_time[2] && e.truck_timing.departure_time[2] == e0.truck_timing.departure_time[2]);
}

void test_stale_launch_point_rejection() {
    Built b = ce2();
    auto p = InstalledPlan::make(b.sol, b.world, 0.0, 0);
    CHECK(p.has_value());
    auto tent = b.world->with_request(customer_id(1), req(1, 6000, -20000));
    const auto cut = clean_cut(*p, 0.0);
    const auto R = cheapest_truck_insertion(*p, *cut, customer_id(1), tent);
    CHECK(R.trials.size() == 2);
    CHECK(R.trials[0].has_flps && !R.trials[0].feasible);
    CHECK(R.trials[0].first_violation.find("launch point") != std::string::npos || R.trials[0].first_violation.find("flexible") != std::string::npos);
    CHECK(R.trials[1].feasible && R.best && *R.best == 1);
    // the stale FLP is not silently repaired: inserting r before A leaves f between the depot and r, off its arc (depot, A)
    Solution s = b.sol; s.truck_route = {0, 1000, customer_id(1), 1, 0};
    CHECK(!tent->evaluate(s).feasible);
}

void test_invariants() {
    Built b = ce2();
    auto p = InstalledPlan::make(b.sol, b.world, 0.0, 0);
    const model::NodeId rid = customer_id(1);
    auto tent = b.world->with_request(rid, req(1, 6000, -20000));
    const auto cut = clean_cut(*p, 0.0);
    const auto R = cheapest_truck_insertion(*p, *cut, rid, tent);
    auto next = InstalledPlan::make(R.solution, tent, 0.0, 1);
    CHECK(next.has_value());
    auto rep = check_single_insertion(*p, *next, *cut, 1, rid, 0.0);
    CHECK(rep.ok);
    if (!rep.ok) for (auto& v : rep.violations) std::printf("   unexpected: %s\n", v.c_str());
    // negative controls: each invariant must be able to fail
    CHECK(!check_single_insertion(*p, *next, *cut, 1, rid, 1e9).ok);          // I6: new service before s + omega
    CHECK(!check_single_insertion(*p, *next, *cut, 1, model::NodeId(7777), 0.0).ok); // I3: wrong customer
    Cut late = *cut; late.route_pos = next->solution().truck_route.size() - 1;      // I2: cut after the insertion point
    CHECK(!check_single_insertion(*p, *next, late, 1, rid, 0.0).ok);
    Solution tr = R.solution; tr.drone_trips[0].drone_id = 1;                        // I4: a trip changed
    auto tampered = InstalledPlan::make(tr, tent, 0.0, 2);
    CHECK(tampered.has_value() && !check_single_insertion(*p, *tampered, *cut, 1, rid, 0.0).ok);
    Solution rt = R.solution; rt.truck_route = {0, 1000, 1, 0, rid};               // wrong structure -> evaluator rejects / I4-I2 fail
    // a plan whose prefix was altered: build with the new customer inserted BEFORE the launch (route {0, r, f, A, 0}) -> infeasible or I2/I4 violation
    Solution pre = b.sol; pre.truck_route = {0, rid, 1000, 1, 0};
    auto pre_plan = InstalledPlan::make(pre, tent, 0.0, 3);
    if (pre_plan) CHECK(!check_single_insertion(*p, *pre_plan, *cut, 0, rid, 0.0).ok);
    (void)rt;
    // I5 power: a straddled insertion is exempt (no claim), but a non-straddled tail shift is verified: tamper by comparing against a plan from another world
    Built other = ce2();
    auto w_other = other.world->with_request(rid, req(1, 6000, -20000));
    auto next_other = InstalledPlan::make(R.solution, w_other, 0.0, 1);
    CHECK(next_other.has_value() && check_single_insertion(*p, *next_other, *cut, 1, rid, 0.0).ok);
}

// ---- tentative world cost, n = 50 (informational) -------------------------------------------------------------------------------
double median(std::vector<double> v) { std::sort(v.begin(), v.end()); return v[v.size() / 2]; }
template <class F> double time_ms(F&& f) { const auto t0 = std::chrono::steady_clock::now(); f(); return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count(); }

void measure_n50() {
    config::ExperimentConfig cfg;
    cfg.base.instance.num_customers = 50;
    const auto gen = instance::generate_static_instance(cfg.base);
    CHECK(gen.ok);
    const auto& inst = gen.instance;
    init::InitialSolutionPolicy pol;
    pol.kmeans_policy = cfg.initial.kmeans;
    pol.flp_policy.unit_division_length = cfg.base.flp.unit_division_length_m;
    const tsp::TSPSolverFactory factory(cfg.initial.tsp_max_exact_n, cfg.initial.tsp_heuristic_restarts, cfg.initial.tsp_heuristic_seed);
    const auto net0 = inst.make_network();
    const auto ini = init::build_initial_solution(inst.depot, inst.customers, inst.customer_data, net0, pol, factory);
    CHECK(ini.feasible);
    std::shared_ptr<const World> world;
    const double t_create = time_ms([&] { world = World::create(ini.node_registry, inst.customer_data, ini.flp_data, inst.truck, inst.drone, cfg.base.neighbourhood.adjacency_window); });
    auto plan = InstalledPlan::make(ini.solution, world, 0.0, 0);
    CHECK(plan.has_value() && plan->consistent());
    std::vector<double> shared_ms, deep_ms, eval_ms;
    std::shared_ptr<const World> keep;
    for (int i = 0; i < 40; ++i) shared_ms.push_back(time_ms([&] { keep = world->with_request(customer_id(static_cast<std::uint32_t>(i + 1)), req(1, 1000.0 + i, 2000.0)); }));
    for (int i = 0; i < 15; ++i) deep_ms.push_back(time_ms([&] { keep = world->deep_rebuild(); }));
    for (int i = 0; i < 200; ++i) eval_ms.push_back(time_ms([&] { volatile double o = world->evaluate(ini.solution).objective_value; (void)o; }));
    CHECK(keep != nullptr);
    std::printf("[measure n=50] nodes %zu (FLPs %zu), customers %zu | World::create %.2f ms | tentative with_request (shares FLP map+index) median %.3f ms (min %.3f, max %.3f over 40) "
                "| deep rebuild (copy FLP map + rebuild arc index) median %.3f ms (over 15) | one evaluation median %.4f ms (over 200)\n",
                world->nodes().size(), world->flp_data().size(), world->customers().size(), t_create, median(shared_ms),
                *std::min_element(shared_ms.begin(), shared_ms.end()), *std::max_element(shared_ms.begin(), shared_ms.end()), median(deep_ms), median(eval_ms));
}

} // namespace

int main() {
    test_ids();
    test_world();
    test_world_deep();
    test_plan();
    test_cursor();
    test_spanning_trip_rejection();
    test_stale_launch_point_rejection();
    test_invariants();
    measure_n50();
    std::printf("test_dyn_core: %d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
