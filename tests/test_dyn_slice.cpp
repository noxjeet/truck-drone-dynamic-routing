// DM4 vertical slice: SeedStream additivity, scenario generator, simulator (hand-built and end-to-end), writer, config, determinism/replay,
// base-plan ablation pairing (U6). No wall-clock quantity is asserted.
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <vector>

#include "dynamic/runner.hpp"

using namespace tdr;
using namespace tdr::dynamic;
using tdr::model::DroneTrip;
using tdr::model::Node;
using tdr::model::NodeType;
using tdr::model::Solution;

static int g_checks = 0, g_fail = 0;
#define CHECK(cond) do { ++g_checks; if (!(cond)) { ++g_fail; std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

namespace {

// static seed values must be exactly as before the dynamic enumerators were appended
static_assert(static_cast<std::uint64_t>(util::SeedStream::Instance) == 1 && static_cast<std::uint64_t>(util::SeedStream::InitialSolution) == 2 &&
              static_cast<std::uint64_t>(util::SeedStream::Vns) == 3 && static_cast<std::uint64_t>(util::SeedStream::Shake) == 4 &&
              static_cast<std::uint64_t>(util::SeedStream::Fuzz) == 5, "static SeedStream values changed");

std::uint64_t derive_literal(std::uint64_t base, std::uint64_t stream, std::uint64_t index) { // independent copy of the formula, numeric stream ids
    util::SplitMix64 a(base);
    const std::uint64_t k1 = a.next();
    util::SplitMix64 b(k1 ^ (stream * 0xd1b54a32d192ed03ULL));
    const std::uint64_t k2 = b.next();
    util::SplitMix64 c(k2 ^ (index * 0x8cb92ba72f3d8dd7ULL));
    return c.next();
}

void test_seed_streams() {
    using util::SeedStream; using util::derive_seed;
    for (std::uint64_t s = 1; s <= 5; ++s)
        for (std::uint64_t idx : {0ULL, 1ULL, 17ULL, 123456ULL})
            CHECK(derive_seed(20261109ULL, static_cast<SeedStream>(s), idx) == derive_literal(20261109ULL, s, idx));
    std::set<std::uint64_t> seeds;
    for (std::uint64_t s = 1; s <= 12; ++s) seeds.insert(derive_seed(20261109ULL, static_cast<SeedStream>(s), 0));
    CHECK(seeds.size() == 12); // all streams, static and dynamic, give distinct seeds
}

void test_scenario() {
    ScenarioParams p; p.rho = 0.5; p.sigma = 3;
    const Scenario a = generate_scenario(p, 10, 10000.0), b = generate_scenario(p, 10, 10000.0);
    CHECK(a.ok && b.ok && a.digest == b.digest && a.requests.size() == b.requests.size());
    bool same = a.requests.size() == b.requests.size();
    for (std::size_t i = 0; same && i < a.requests.size(); ++i) same = a.requests[i].time == b.requests[i].time && a.requests[i].location.x == b.requests[i].location.x && a.requests[i].location.y == b.requests[i].location.y;
    CHECK(same);
    // golden digest pins the scenario across platforms
    CHECK(a.digest == "4f46e73a729dfc3e");
    if (a.digest != "4f46e73a729dfc3e") std::printf("   digest(n0=10,T0=10000,rho=0.5,sigma=3,seed=20261109) = %s\n", a.digest.c_str());
    ScenarioParams q = p; q.sigma = 4;
    CHECK(generate_scenario(q, 10, 10000.0).digest != a.digest);
    // structure: 1-based indices, sorted times in [0, T_A), locations in the area
    bool ok = true; double prev = -1;
    for (std::size_t i = 0; i < a.requests.size(); ++i) {
        const auto& r = a.requests[i];
        ok = ok && r.index == i + 1 && r.time >= prev && r.time >= 0 && r.time < a.horizon && r.location.x >= 0 && r.location.x < p.area_side_m && r.location.y >= 0 && r.location.y < p.area_side_m;
        prev = r.time;
    }
    CHECK(ok);
    // fixed count
    ScenarioParams f = p; f.count = ArrivalCount::Fixed; f.rho = 0.5;
    CHECK(generate_scenario(f, 10, 10000.0).requests.size() == 5);
    // the spatial/attribute draws do not change arrival times when only the area changes (separate Locations stream): arrival digest part equal
    ScenarioParams wide = p; wide.area_side_m = 30000.0;
    const Scenario w = generate_scenario(wide, 10, 10000.0);
    bool times_equal = w.requests.size() == a.requests.size();
    for (std::size_t i = 0; times_equal && i < a.requests.size(); ++i) times_equal = a.requests[i].time == w.requests[i].time;
    CHECK(times_equal);
    // Poisson mean (statistical sanity: 400 scenarios, Lambda = 5, tolerance 5 standard errors)
    double mean = 0;
    for (int s = 0; s < 400; ++s) { ScenarioParams t = p; t.sigma = 1000 + s; mean += static_cast<double>(generate_scenario(t, 10, 10000.0).requests.size()); }
    mean /= 400.0;
    CHECK(std::fabs(mean - 5.0) < 5.0 * std::sqrt(5.0 / 400.0));
    // loud failure, never truncation
    ScenarioParams big = p; big.rho = 50; big.m_max = 20;
    const Scenario bs = generate_scenario(big, 10, 10000.0);
    CHECK(!bs.ok && bs.requests.empty());
    ScenarioParams bad = p; bad.rho = -1;
    CHECK(!generate_scenario(bad, 10, 10000.0).ok);
    CHECK(!generate_scenario(p, 2000, 10000.0).ok); // K > 1024
}

// CE-2 style hand world: depot (0,0), A=1 (12000,0), X=2 (6000,3000), FLP 1000 at (6000,0); trip (f, X, A).
std::shared_ptr<const World> hand_world(Solution& sol) {
    model::NodeRegistry n;
    n.add(Node{0, NodeType::Depot, {0, 0}});
    n.add(Node{1, NodeType::Customer, {12000, 0}});
    n.add(Node{2, NodeType::Customer, {6000, 3000}});
    n.add(Node{1000, NodeType::FlexibleLaunchPoint, {6000, 0}});
    World::CustomerMap c;
    for (int i : {1, 2}) { model::CustomerData d; d.node_id = i; d.truck_service_time = 30; d.drone_service_time = 30; d.generalized_flight_time_theta = 1; c[i] = d; }
    timing::FlpDataMap f; f[1000] = model::FlexibleLaunchPointData{1000, 0, 1, 0.5};
    sol.truck_route = {0, 1000, 1, 0};
    sol.drone_trips = {DroneTrip{0, 1000, 2, 1}};
    return World::create(n, c, f, model::TruckParameters{15.0}, model::DroneParameters{12.0, 1800.0, 60.0, 40.0, 2});
}
Request mk(std::uint32_t i, double t, double x, double y) { Request r; r.index = i; r.time = t; r.location = {x, y}; r.se1 = 30; r.se2 = 30; r.theta = 1; return r; }

void test_simulator_hand() {
    Solution sol;
    const auto w = hand_world(sol);
    auto base = InstalledPlan::make(sol, w, 0.0, 0);
    CHECK(base.has_value());
    Scenario sc; sc.ok = true; sc.n0 = 2; sc.t0_ref = 2000; sc.horizon = 1e7;
    sc.requests = {mk(1, 0.0, 3000, -2000), mk(2, 5.0, 9000, -2000), mk(3, 1.0e6, 1000, 1000)};
    ScanAll g0; ScanNone none; AdmitA0 a0;
    SimOptions so; so.omega = 10.0; so.scan = &g0; so.admission = &a0;
    const SimResult R = simulate(*base, sc, so);
    CHECK(R.ok && R.errors.empty() && R.records.size() == 3);
    CHECK(R.records[0].outcome == Outcome::Accepted && R.records[1].outcome == Outcome::Accepted && R.records[2].outcome == Outcome::Stale);
    // FIFO sequencing: r2 arrived during r1's decision, starts when r1 installs
    CHECK(R.records[0].start == 0.0 && R.records[0].install == 10.0 && R.records[1].start == 10.0 && R.records[1].install == 20.0 && R.records[1].queue_len_at_arrival == 1);
    CHECK(R.records[0].plan_version == 1 && R.records[1].plan_version == 2);
    // partition: every request exactly one outcome, counts add up
    std::size_t total = 0; for (auto c : R.counts) total += c;
    CHECK(total == 3 && R.counts[static_cast<int>(Outcome::Accepted)] == 2 && R.counts[static_cast<int>(Outcome::Stale)] == 1);
    // objective accounting: final = base + sum of accepted deltas; ETAs at install >= s + omega
    CHECK(std::fabs(R.final_objective - (R.base_objective + R.records[0].delta_objective + R.records[1].delta_objective)) < 1e-6);
    CHECK(R.records[0].eta_at_install >= R.records[0].install && R.records[1].eta_at_install >= R.records[1].install);
    CHECK(R.final_plan && R.final_plan->consistent() && R.final_plan->solution().truck_route.size() == sol.truck_route.size() + 2);
    CHECK(R.final_plan->world().customers().count(customer_id(1)) && R.final_plan->world().customers().count(customer_id(2)) && !R.final_plan->world().customers().count(customer_id(3)));
    // the base plan object is unchanged (immutability)
    CHECK(base->version() == 0 && base->solution().truck_route == sol.truck_route);
    // B0 baseline: same scenario, nothing is ever considered; the plan stays the base plan
    so.scan = &none;
    const SimResult N = simulate(*base, sc, so);
    CHECK(N.ok && N.counts[static_cast<int>(Outcome::OutOfScan)] == 3 && N.final_plan->version() == 0 && N.final_objective == N.base_objective);
    // replay: identical inputs -> identical records
    so.scan = &g0;
    const SimResult R2 = simulate(*base, sc, so);
    bool same = R2.records.size() == R.records.size() && R2.final_objective == R.final_objective;
    for (std::size_t i = 0; same && i < R.records.size(); ++i) same = R.records[i].outcome == R2.records[i].outcome && R.records[i].delta_objective == R2.records[i].delta_objective;
    CHECK(same);
    // a late request still sees the last installed plan at the same instant of an installation: ARRIVAL before INSTALL
    Scenario tie = sc;
    tie.requests = {mk(1, 0.0, 3000, -2000), mk(2, 10.0, 9000, -2000)}; // r2 arrives exactly when r1 installs
    const SimResult T = simulate(*base, tie, so);
    CHECK(T.ok && T.records[1].queue_len_at_arrival == 1 && T.records[1].start == 10.0);
}

void test_config() {
    DynamicConfig c;
    CHECK(validate(c).empty());
    c.regime = 'F'; apply_regime(c); apply_regime(c);
    CHECK(c.base.base.vehicle.endurance_s == 3600.0 && c.base.base.vehicle.truck_speed_mps == 15.0 && c.base.base.vehicle.drone_speed_mps == 12.0 &&
          c.base.base.vehicle.launch_prep_time_s == 60.0 && c.base.base.vehicle.recovery_time_s == 40.0 && c.base.base.vehicle.num_drones == 2 && c.base.base.instance.theta == 1.0);
    DynamicConfig p; apply_regime(p);
    CHECK(p.base.base.vehicle.endurance_s == 1800.0);
    DynamicConfig b; b.regime = 'X'; CHECK(!validate(b).empty());
    b = DynamicConfig{}; b.scan = "g1"; CHECK(!validate(b).empty());
    b = DynamicConfig{}; b.admission = "a-pm"; CHECK(!validate(b).empty()); // only "apm" is spelled
    b = DynamicConfig{}; b.omega = -1; CHECK(!validate(b).empty());
    b = DynamicConfig{}; b.m_max = 2000; CHECK(!validate(b).empty());
}

DynamicConfig small_cfg(int n = 10) {
    DynamicConfig c;
    c.base.base.instance.num_customers = n;
    c.base.vns.iter_max = 150;
    c.rho = 1.0; c.scenarios = 4; c.omega = 10.0;
    return c;
}

void check_run(const DynamicRun& R, const char* label) {
    CHECK(R.ok && R.errors.empty());
    if (!R.ok) for (auto& e : R.errors) std::printf("   %s: %s\n", label, e.c_str());
    for (const auto& r : R.runs) {
        std::size_t total = 0; for (auto c : r.sim.counts) total += c;
        CHECK(total == r.scenario.requests.size() && r.sim.records.size() == total);               // (c) exactly one outcome per request
        CHECK(r.sim.ok && r.sim.final_plan && r.sim.final_plan->consistent());                      // (f) final plan re-evaluates feasible
        double sum = 0; std::uint32_t v = 0; bool inv = true, mono = true;
        for (const auto& d : r.sim.records) { inv = inv && d.invariants_ok; if (d.outcome == Outcome::Accepted) { sum += d.delta_objective; mono = mono && d.plan_version == v + 1; v = d.plan_version; } }
        CHECK(inv && mono);                                                                         // (b) invariants, versions
        CHECK(std::fabs(r.sim.final_objective - (r.sim.base_objective + sum)) < 1e-6 * std::max(1.0, r.sim.final_objective));
        CHECK(r.sim.counts[static_cast<int>(Outcome::Accepted)] == v && static_cast<int>(r.sim.invariant_checks) >= static_cast<int>(v));
    }
}

void test_end_to_end() {
    const DynamicConfig c = small_cfg();
    const DynamicRun R = run_dynamic(c);
    check_run(R, "n=10 P");
    CHECK(R.runs.size() == 4);
    std::size_t acc = 0, gen = 0;
    for (const auto& r : R.runs) { acc += r.sim.counts[static_cast<int>(Outcome::Accepted)]; gen += r.scenario.requests.size(); }
    CHECK(gen > 0 && acc > 0);
    // (d) replay: same config -> byte-identical outputs
    const DynamicRun R2 = run_dynamic(c);
    const auto o1 = render_outputs(R.cfg, R.base, R.runs), o2 = render_outputs(R2.cfg, R2.base, R2.runs);
    bool identical = o1.size() == o2.size();
    for (std::size_t i = 0; identical && i < o1.size(); ++i) identical = o1[i].name == o2[i].name && o1[i].content == o2[i].content;
    CHECK(identical);
    // (e) policy change leaves the scenario untouched: B0 (scan none) has the same scenario digests, base plan only
    DynamicConfig n = c; n.scan = "none";
    const DynamicRun RN = run_dynamic(n);
    CHECK(RN.ok && RN.runs.size() == R.runs.size());
    for (std::size_t i = 0; i < R.runs.size(); ++i) {
        CHECK(RN.runs[i].scenario.digest == R.runs[i].scenario.digest);
        CHECK(RN.runs[i].sim.final_objective == RN.runs[i].sim.base_objective && RN.runs[i].sim.counts[0] == RN.runs[i].scenario.requests.size());
    }
    // U6 pairing: the constructive-base ablation uses the same scenarios (scale from the static-VNS reference makespan)
    DynamicConfig k = c; k.base_kind = BaseKind::Constructive;
    const DynamicRun RK = run_dynamic(k);
    check_run(RK, "n=10 constructive");
    CHECK(RK.ok && RK.base.reference_makespan == R.base.reference_makespan);
    for (std::size_t i = 0; i < R.runs.size() && i < RK.runs.size(); ++i) CHECK(RK.runs[i].scenario.digest == R.runs[i].scenario.digest);
    // regime F: endurance doubled, everything else identical; runs end to end
    DynamicConfig f = c; f.regime = 'F';
    const DynamicRun RF = run_dynamic(f);
    check_run(RF, "n=10 F");
    CHECK(RF.cfg.base.base.vehicle.endurance_s == 3600.0 && RF.base.plan->world().drone().endurance_seconds == 3600.0 && R.base.plan->world().drone().endurance_seconds == 1800.0);
    const auto of = render_outputs(RF.cfg, RF.base, RF.runs);
    CHECK(of.front().content.find("\"regime\": \"F\"") != std::string::npos && of.front().content.find("\"endurance_s\": 3600") != std::string::npos);
    // large omega: queueing and stale outcomes, still a partition and still feasible
    DynamicConfig slow = c; slow.omega = 2000.0; slow.rho = 1.0;
    const DynamicRun RS = run_dynamic(slow);
    check_run(RS, "omega=2000");
    std::size_t maxq = 0; for (const auto& r : RS.runs) maxq = std::max(maxq, r.sim.max_queue);
    std::printf("   [info] omega=2000: max queue length %zu\n", maxq);
    // invalid inputs fail cleanly
    DynamicConfig bad = c; bad.regime = 'Z';
    CHECK(!run_dynamic(bad).ok);
}

void test_writer() {
    const DynamicConfig c = small_cfg();
    const DynamicRun R = run_dynamic(c);
    const auto files = render_outputs(R.cfg, R.base, R.runs);
    CHECK(files.size() == R.runs.size() + 2 && files.front().name == "results.json" && files.back().name == "manifest.json");
    CHECK(files.front().content.find("tdr.dynamic.results/1") != std::string::npos && files.front().content.find("static-baseline-m10") != std::string::npos);
    CHECK(files.front().content.find(R.runs[0].scenario.digest) != std::string::npos);
    std::size_t lines = 0; for (char ch : files[1].content) lines += (ch == '\n');
    CHECK(lines == R.runs[0].scenario.requests.size() + 1);
    const auto dir = (std::filesystem::temp_directory_path() / "tdr_dyn_writer_test").string();
    std::filesystem::remove_all(dir);
    CHECK(write_outputs(dir, files, false).empty());
    CHECK(!write_outputs(dir, files, false).empty());  // refuses to overwrite
    CHECK(write_outputs(dir, files, true).empty());
    std::ifstream in(dir + "/results.json"); std::stringstream ss; ss << in.rdbuf();
    CHECK(ss.str() == files.front().content);
    std::filesystem::remove_all(dir);
}

} // namespace

int main() {
    test_seed_streams();
    test_scenario();
    test_simulator_hand();
    test_config();
    test_end_to_end();
    test_writer();
    std::printf("test_dyn_slice: %d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
