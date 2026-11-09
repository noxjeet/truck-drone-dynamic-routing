// DM1 read-only probes T1-T6 + CE-1..CE-4 for the dynamic design (dynamic/dm0-design). NOT part of the default build or CTest.
// Calls the UNCHANGED evaluator (timing/timing_engine.hpp), SpatialNetwork and the FLP generator. It edits nothing in the repository.
// A PASS here is an OBSERVATION on the generated cases; it is never a proof. See docs/dynamic/math_spec_dynamic.md for the status labels.
//   g++ -std=c++20 -O2 -I include bench/dm1_probes.cpp -o <outside-repo>/dm1_probes ; dm1_probes [N_scale]
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "flp/flexible_launch_point_generator.hpp"
#include "graph/spatial_network.hpp"
#include "model/customer.hpp"
#include "model/solution.hpp"
#include "timing/timing_engine.hpp"

using namespace tdr::model;
using namespace tdr::graph;
using namespace tdr::timing;
using std::size_t;

namespace {
struct Params { double vT = 15, vD = 12, e = 1800, SL = 60, SR = 40, Se1 = 30, Se2 = 30, theta = 1; int U = 2; };
Params regimeP() { return Params{}; }
Params regimeF() { Params p; p.e = 2.0 * p.e; return p; } // regime F: endurance doubled, everything else identical to P
double reach_radius(const Params& p) { return p.vD * (p.e - (1.0 + p.theta) * p.Se2) / (2.0 + p.theta); } // formula, never hard-coded

struct World {
    Params p; NodeRegistry nodes; std::unordered_map<NodeId, CustomerData> cust; FlpDataMap flp; Solution sol;
};
bool is_flp(NodeId id) { return id >= 1000 && id < 5000; }
SolutionEvaluationResult ev(const World& w) {
    SpatialNetwork net(w.nodes, TruckParameters{w.p.vT}, DroneParameters{w.p.vD, w.p.e, w.p.SL, w.p.SR, w.p.U});
    return evaluate_solution(w.sol, net, w.cust, w.flp);
}
Point2D xy(const World& w, NodeId id) { return w.nodes.at(id).coordinates; }
double dist(Point2D a, Point2D b) { return std::hypot(a.x - b.x, a.y - b.y); }
double dd(const World& w, NodeId a, NodeId b) { return dist(xy(w, a), xy(w, b)); }
void add_depot(World& w, double x, double y) { w.nodes.add(Node{0, NodeType::Depot, {x, y}}); }
void add_cust(World& w, NodeId id, double x, double y) { w.nodes.add(Node{id, NodeType::Customer, {x, y}}); w.cust[id] = CustomerData{id, w.p.Se1, w.p.Se2, w.p.theta}; }
void add_flp(World& w, NodeId id, NodeId from, NodeId to, double ratio) {
    const Point2D a = xy(w, from), b = xy(w, to);
    w.nodes.add(Node{id, NodeType::FlexibleLaunchPoint, {a.x + ratio * (b.x - a.x), a.y + ratio * (b.y - a.y)}});
    w.flp[id] = FlexibleLaunchPointData{id, from, to, ratio};
}
int pos(const TruckRoute& r, NodeId id) { for (size_t i = 0; i < r.size(); ++i) if (r[i] == id) return (int)i; return -1; }
struct TripPos { int lp, rp; NodeId cust; };
std::vector<TripPos> trip_pos(const World& w) {
    std::vector<TripPos> t;
    for (const auto& d : w.sol.drone_trips) t.push_back({pos(w.sol.truck_route, d.launch_node), d.recovery_node == w.sol.truck_route.front() ? (int)w.sol.truck_route.size() - 1 : pos(w.sol.truck_route, d.recovery_node), d.service_customer}); // depot recovery = END depot
    return t;
}
std::vector<int> skeleton(const World& w) { std::vector<int> s; for (size_t i = 0; i < w.sol.truck_route.size(); ++i) if (!is_flp(w.sol.truck_route[i])) s.push_back((int)i); return s; }
double eta_of(const SolutionEvaluationResult& r, NodeId c) { for (const auto& cr : r.customer_records) if (cr.customer_id == c) return cr.service_arrival_time; return -1; }
bool bit_equal(double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0; }

// ---- statistics ---------------------------------------------------------------------------------------------------
int g_fail = 0, g_inconclusive = 0;
void report(const std::string& name, bool ok, const std::string& detail) {
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << name << " :: " << detail << "\n";
    if (!ok) ++g_fail;
}

// ---- random feasible plan -----------------------------------------------------------------------------------------
World gen_plan(std::mt19937_64& g, int n, double area, const Params& p, int max_trips) {
    auto U = [&](double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(g); };
    World w; w.p = p; add_depot(w, U(0, area), U(0, area));
    for (int i = 1; i <= n; ++i) add_cust(w, i, U(0, area), U(0, area));
    std::vector<NodeId> ids; for (int i = 1; i <= n; ++i) ids.push_back(i);
    std::shuffle(ids.begin(), ids.end(), g);
    const int m = std::min(max_trips, std::max(0, n - 3));
    std::vector<NodeId> drone(ids.begin(), ids.begin() + m), truck(ids.begin() + m, ids.end());
    w.sol.truck_route.push_back(0); for (NodeId c : truck) w.sol.truck_route.push_back(c); w.sol.truck_route.push_back(0);
    NodeId next_flp = 1000;
    for (NodeId j : drone) {
        for (int attempt = 0; attempt < 60; ++attempt) {
            const auto sk = skeleton(w);
            const int ai = std::uniform_int_distribution<int>(0, (int)sk.size() - 2)(g);
            const int pa = sk[ai], pb = sk[ai + 1];
            const int ki = std::uniform_int_distribution<int>(ai + 1, (int)sk.size() - 1)(g);
            const NodeId a = w.sol.truck_route[pa], b = w.sol.truck_route[pb], k = w.sol.truck_route[sk[ki]];
            World t = w;
            add_flp(t, next_flp, a, b, U(0.05, 0.95));
            t.sol.truck_route.insert(t.sol.truck_route.begin() + pb, next_flp);
            for (int u = 0; u < p.U; ++u) {
                World t2 = t; t2.sol.drone_trips.push_back(DroneTrip{u, next_flp, j, k});
                if (ev(t2).feasible) { w = t2; ++next_flp; goto placed; }
            }
        }
        // could not place: serve by truck instead (insert at random skeleton gap)
        { const auto sk = skeleton(w); const int pb = sk[std::uniform_int_distribution<int>(1, (int)sk.size() - 1)(g)]; w.sol.truck_route.insert(w.sol.truck_route.begin() + pb, j); }
    placed:;
    }
    return w;
}
struct Gap { int pa, pb; };
std::vector<Gap> gaps(const World& w) { std::vector<Gap> r; const auto sk = skeleton(w); for (size_t i = 0; i + 1 < sk.size(); ++i) r.push_back({sk[i], sk[i + 1]}); return r; }
constexpr NodeId kNew = 5000;
World insert_new(const World& w, int pb, Point2D at) { // new truck customer immediately before the skeleton node at route index pb
    World t = w; add_cust(t, kNew, at.x, at.y); t.sol.truck_route.insert(t.sol.truck_route.begin() + pb, kNew); return t;
}
bool straddled(const World& w, const Gap& gp) { for (const auto& t : trip_pos(w)) if (t.lp < gp.pb && gp.pb <= t.rp) return true; return false; }
// tail set T(g): truck customers at positions >= pb, drone customers of trips launched at positions >= pb
std::vector<NodeId> tail_set(const World& w, const Gap& gp, bool include_recovered_before_a = false, bool include_straddling = false) {
    std::vector<NodeId> s;
    for (size_t i = gp.pb; i < w.sol.truck_route.size(); ++i) { const NodeId c = w.sol.truck_route[i]; if (w.nodes.at(c).type == NodeType::Customer) s.push_back(c); }
    for (const auto& t : trip_pos(w)) {
        if (t.lp >= gp.pb) s.push_back(t.cust);
        else if (include_straddling && t.rp >= gp.pb) s.push_back(t.cust);
        else if (include_recovered_before_a && t.rp <= gp.pa) s.push_back(t.cust);
    }
    return s;
}
Point2D rand_pt(std::mt19937_64& g, double area) { return {std::uniform_real_distribution<double>(0, area)(g), std::uniform_real_distribution<double>(0, area)(g)}; }

// ---- T1 -------------------------------------------------------------------------------------------------------------
void run_T1(std::mt19937_64& g, int N) {
    long clean = 0, spanning = 0, ident_clean = 0, ident_span = 0, feas_change_clean = 0, feas_change_span = 0, nc = 0, nc_differ = 0, skipped = 0, eta_bad = 0;
    for (int it = 0; it < N; ++it) {
        const Params p = (it % 2) ? regimeF() : regimeP();
        World w = gen_plan(g, std::uniform_int_distribution<int>(6, 12)(g), 20000, p, std::uniform_int_distribution<int>(0, 3)(g));
        const auto r0 = ev(w); if (!r0.feasible) { ++skipped; continue; }
        const auto sk = skeleton(w); const auto tp = trip_pos(w);
        const int B = sk[std::uniform_int_distribution<int>(1, (int)sk.size() - 2)(g)];
        bool is_clean = true; for (const auto& t : tp) if (t.lp <= B && B < t.rp) is_clean = false;
        const auto gp = gaps(w);
        std::vector<Gap> after; for (const auto& x : gp) if (x.pa >= B) after.push_back(x);
        if (!after.empty()) {
            const Gap e = after[std::uniform_int_distribution<size_t>(0, after.size() - 1)(g)];
            World w2 = insert_new(w, e.pb, rand_pt(g, 20000)); const auto r1 = ev(w2);
            bool same = r1.truck_timing.arrival_time.size() > (size_t)B;
            for (int q = 0; same && q <= B; ++q) same = bit_equal(r0.truck_timing.arrival_time[q], r1.truck_timing.arrival_time[q]) && bit_equal(r0.truck_timing.departure_time[q], r1.truck_timing.departure_time[q]);
            // prefix customers: truck customers at positions <= B, drone customers of trips launched at <= B
            for (int q = 0; q <= B; ++q) if (w.nodes.at(w.sol.truck_route[q]).type == NodeType::Customer && !bit_equal(eta_of(r0, w.sol.truck_route[q]), eta_of(r1, w.sol.truck_route[q]))) { same = false; ++eta_bad; }
            for (const auto& t : tp) if (t.lp <= B && !bit_equal(eta_of(r0, t.cust), eta_of(r1, t.cust))) { same = false; ++eta_bad; }
            (is_clean ? clean : spanning)++; if (same) (is_clean ? ident_clean : ident_span)++;
            if (!r1.feasible) (is_clean ? feas_change_clean : feas_change_span)++;
        }
        // negative control: edit at a gap strictly before B (insertion point pb <= B) must change prefix arrays
        std::vector<Gap> before; for (const auto& x : gp) if (x.pb <= B) before.push_back(x);
        if (!before.empty()) {
            const Gap e = before[std::uniform_int_distribution<size_t>(0, before.size() - 1)(g)];
            World w2 = insert_new(w, e.pb, rand_pt(g, 20000)); const auto r1 = ev(w2);
            bool differ = false; for (int q = 0; q <= B && (size_t)q < r1.truck_timing.arrival_time.size(); ++q) if (!bit_equal(r0.truck_timing.arrival_time[q], r1.truck_timing.arrival_time[q])) differ = true;
            ++nc; if (differ) ++nc_differ;
        }
    }
    std::ostringstream o; o << "plans " << N << " (skipped infeasible " << skipped << "); tail edits: clean cut " << clean << " (bit-identical prefix " << ident_clean << ", S' infeasible " << feas_change_clean << "), SPANNING cut " << spanning << " (bit-identical prefix " << ident_span << ", S' infeasible " << feas_change_span << "); prefix-ETA mismatches " << eta_bad << "; negative control (edit before cut) " << nc << " cases, prefix arrays differ in " << nc_differ;
    report("T1 prefix invariance (P1)", clean > 0 && spanning > 0 && ident_clean == clean && ident_span == spanning && eta_bad == 0 && nc > 0 && nc_differ == nc, o.str());
}

// ---- T2 -------------------------------------------------------------------------------------------------------------
void run_T2(std::mt19937_64& g, int N) {
    long cases = 0, ident = 0, trip_in_region = 0, tail_trips_checked = 0, infeas = 0, Fbad = 0, nc_span = 0, nc_span_fail = 0, nc_tail = 0, nc_tail_fail = 0, nc_tail_end = 0, vacuous = 0, endur_bad = 0;
    double maxdev = 0;
    for (int it = 0; it < N; ++it) {
        const Params p = (it % 2) ? regimeF() : regimeP();
        World w = gen_plan(g, std::uniform_int_distribution<int>(6, 12)(g), 20000, p, std::uniform_int_distribution<int>(0, 3)(g));
        const auto r0 = ev(w); if (!r0.feasible) continue;
        const auto sk = skeleton(w); const auto tp = trip_pos(w);
        for (int rep = 0; rep < 3; ++rep) {
            int ia = std::uniform_int_distribution<int>(0, (int)sk.size() - 2)(g), ib = std::uniform_int_distribution<int>(ia + 1, (int)sk.size() - 1)(g);
            const int pa = sk[ia], pb = sk[ib];
            bool closed = true, region_trip = false, tail_trip = false;
            for (const auto& t : tp) { const bool pre = t.rp <= pa, reg = t.lp > pa && t.rp <= pb, tl = t.lp > pb; if (!(pre || reg || tl)) closed = false; if (reg) region_trip = true; if (tl) tail_trip = true; }
            if (!closed) continue;
            const auto gp = gaps(w); std::vector<Gap> in; for (const auto& x : gp) if (x.pa >= pa && x.pb <= pb) in.push_back(x);
            World w2 = w; bool did = false;
            if (rep == 0 && region_trip == false && pb - pa >= 3) { // E1: reverse the interior of a trip-free, FLP-free region
                bool flp_free = true; for (int q = pa; q <= pb; ++q) if (is_flp(w.sol.truck_route[q])) flp_free = false;
                if (flp_free) { std::reverse(w2.sol.truck_route.begin() + pa + 1, w2.sol.truck_route.begin() + pb); did = true; }
            }
            if (!did) { const Gap e = in[std::uniform_int_distribution<size_t>(0, in.size() - 1)(g)]; w2 = insert_new(w, e.pb, rand_pt(g, 20000)); did = true; } // E2
            const auto r1 = ev(w2); if (!r1.feasible) { ++infeas; continue; }
            const int shift_pb = (pb == (int)w.sol.truck_route.size() - 1) ? (int)w2.sol.truck_route.size() - 1 : pos(w2.sol.truck_route, w.sol.truck_route[pb]); // b unchanged as a node (end depot: last index)
            if (pb == (int)w.sol.truck_route.size() - 1) { ++vacuous; }
            const double D = r1.truck_timing.departure_time[shift_pb] - r0.truck_timing.departure_time[pb];
            ++cases; if (region_trip) ++trip_in_region; if (tail_trip) ++tail_trips_checked;
            bool ok = true; const int off = shift_pb - pb;
            for (size_t q = pb + 1; q < w.sol.truck_route.size(); ++q) {
                const double da = r1.truck_timing.arrival_time[q + off] - r0.truck_timing.arrival_time[q] - D, dd2 = r1.truck_timing.departure_time[q + off] - r0.truck_timing.departure_time[q] - D;
                maxdev = std::max({maxdev, std::fabs(da), std::fabs(dd2)}); if (std::fabs(da) > 1e-9 * std::max(1.0, r0.truck_timing.arrival_time[q]) || std::fabs(dd2) > 1e-9 * std::max(1.0, r0.truck_timing.departure_time[q])) ok = false;
            }
            for (size_t k = 0; k < tp.size(); ++k) if (tp[k].lp > pb) { // tail trips: same order in drone_trips vector
                const auto& a = r0.drone_trip_timings[k]; const auto& b = r1.drone_trip_timings[k];
                const double s = b.customer_arrival_time - a.customer_arrival_time - D; maxdev = std::max(maxdev, std::fabs(s));
                if (std::fabs(s) > 1e-9 * std::max(1.0, a.customer_arrival_time)) ok = false;
                if (std::fabs(a.endurance_value_36 - b.endurance_value_36) > 1e-9 * std::max(1.0, a.endurance_value_36) || std::fabs(a.endurance_value_37 - b.endurance_value_37) > 1e-9 * std::max(1.0, a.endurance_value_37)) { ++endur_bad; ok = false; }
            }
            // F identity: prefix + region(a,b] + m_tail*D + tail
            double Fpre = 0, Freg1 = 0, Ftail0 = 0; long mt = 0;
            auto classify = [&](const World& ww, const SolutionEvaluationResult& rr, int pA, int pB, double& pre, double& reg, double& tl, long& m) {
                const auto t2 = trip_pos(ww); pre = reg = tl = 0; m = 0;
                for (size_t q = 0; q < ww.sol.truck_route.size(); ++q) { const NodeId c = ww.sol.truck_route[q]; if (ww.nodes.at(c).type != NodeType::Customer) continue; const double e = eta_of(rr, c); if ((int)q <= pA) pre += e; else if ((int)q <= pB) reg += e; else { tl += e; ++m; } }
                for (const auto& t : t2) { const double e = eta_of(rr, t.cust); if (t.lp <= pA) pre += e; else if (t.lp <= pB) reg += e; else { tl += e; ++m; } }
            };
            double reg0, tl0, pre0, pre1, reg1, tl1; long m0, m1;
            classify(w, r0, pa, pb, pre0, reg0, tl0, m0); classify(w2, r1, pa, shift_pb, pre1, reg1, tl1, m1);
            const double Fpred = pre0 + reg1 + (double)m0 * D + tl0; (void)Fpre; (void)Freg1; (void)Ftail0; (void)mt; (void)m1; (void)pre1; (void)tl1;
            if (std::fabs(Fpred - r1.objective_value) > 1e-6) { ++Fbad; static int shown2 = 0; if (std::getenv("DM1_DEBUG") && shown2++ < 4) std::cout << "   [dbg T2] Fpred " << Fpred << " actual " << r1.objective_value << " pa " << pa << " pb " << pb << " n " << w.sol.truck_route.size() << " m0 " << m0 << " D " << D << " pre0 " << pre0 << " reg0 " << reg0 << " reg1 " << reg1 << " tl0 " << tl0 << " F0 " << r0.objective_value << "\n"; }
            if (ok) ++ident;
        }
        // negative control NC1: insertion in a region whose right boundary b is spanned by a trip (lp < b < rp)
        for (const auto& t : tp) if (t.rp > t.lp + 1) {
            const auto gp = gaps(w); for (const auto& x : gp) if (t.lp < x.pb && x.pb < t.rp) {
                World w2 = insert_new(w, x.pb, rand_pt(g, 20000)); const auto r1 = ev(w2); const int spb = x.pb; (void)spb;
                const double D = r1.truck_timing.departure_time[x.pb] - r0.truck_timing.departure_time[x.pb]; // b now at x.pb+1
                const int b_new = x.pb + 1; const double Db = r1.truck_timing.departure_time[b_new] - r0.truck_timing.departure_time[x.pb]; (void)D;
                bool uniform = true; for (size_t q = x.pb + 1; q < w.sol.truck_route.size(); ++q) if (std::fabs(r1.truck_timing.departure_time[q + 1] - r0.truck_timing.departure_time[q] - Db) > 1e-9 * std::max(1.0, r0.truck_timing.departure_time[q])) uniform = false;
                ++nc_span; if (!uniform || !r1.feasible) ++nc_span_fail; break;
            }
            break;
        }
        // negative control NC2: tail modified in addition to the region (second insertion after b)
        { const auto gp = gaps(w); if (gp.size() >= 3) { const Gap a1 = gp[0], a2 = gp[gp.size() - 2]; if (a1.pb < a2.pb) {
            World w2 = insert_new(w, a1.pb, rand_pt(g, 20000)); World w3 = w2; add_cust(w3, kNew + 1, 7000 + it % 100, 9000); w3.sol.truck_route.insert(w3.sol.truck_route.begin() + (a2.pb + 1), kNew + 1);
            const auto r1 = ev(w3); const double D = r1.truck_timing.departure_time[a1.pb + 1] - r0.truck_timing.departure_time[a1.pb];
            bool uniform = true; for (size_t q = a1.pb + 1; q < a2.pb; ++q) if (std::fabs(r1.truck_timing.departure_time[q + 1] - r0.truck_timing.departure_time[q] - D) > 1e-9 * std::max(1.0, r0.truck_timing.departure_time[q])) uniform = false;
            bool after_ok = true; for (size_t q = a2.pb; q < w.sol.truck_route.size(); ++q) if (std::fabs(r1.truck_timing.departure_time[q + 2] - r0.truck_timing.departure_time[q] - D) > 1e-9 * std::max(1.0, r0.truck_timing.departure_time[q])) after_ok = false;
            if (a2.pb == (int)w.sol.truck_route.size() - 1) ++nc_tail_end; else { ++nc_tail; if (!after_ok) ++nc_tail_fail; } (void)uniform; } } }
    }
    std::ostringstream o; o << "identity cases " << cases << " (identity held " << ident << "; with region trips " << trip_in_region << ", with tail trips " << tail_trips_checked << "; S' infeasible/skipped " << infeas << "; end-depot (vacuous tail) " << vacuous << "); max |deviation| " << std::scientific << maxdev << "; (36)/(37) mismatches " << endur_bad << "; F-identity violations " << Fbad << " | NC spanning b: " << nc_span << " cases, identity/feasibility broken in " << nc_span_fail << " | NC extra tail edit (second edit before end depot; " << nc_tail_end << " end-gap cases excluded, vacuous there): " << nc_tail << " cases, uniform shift broken behind second edit in " << nc_tail_fail << " (unbroken " << nc_tail - nc_tail_fail << ": absorbed by synchronization waiting; informative)";
    report("T2 tail shift (P2) + negative controls", cases > 0 && ident == cases && Fbad == 0 && endur_bad == 0 && nc_tail > 0 && nc_tail_fail >= 0.9 * nc_tail, o.str());
    std::cout << "      [observation] NC spanning-trip: " << nc_span_fail << "/" << nc_span << " broke (informative, not a pass criterion; CE-1 is the deterministic case)\n";
}

// ---- T3 / T4 slack fuzz ---------------------------------------------------------------------------------------------
struct Cells { long a = 0, b = 0, c = 0, d = 0; }; // test-pass&truth, test-pass&!truth, test-fail&truth, test-fail&!truth
void run_T3_T4(std::mt19937_64& g, int N) {
    long eq_n = 0, eq_bad = 0, strad_feas = 0, strad_viol = 0, strad_infeas = 0, flp_gap = 0, flp_gap_infeas = 0, trucks_only_plans = 0, eq_with_tail_trips = 0;
    double eq_max = 0;
    Cells nonstrad, strad_unrelaxed, naive_all, incl_before_a, trucks_only_min;
    long spec_false_excl = 0, spec_total_truth = 0;
    for (int it = 0; it < N; ++it) {
        const Params p = (it % 2) ? regimeF() : regimeP();
        const int mt = (it % 5 == 0) ? 0 : std::uniform_int_distribution<int>(1, 3)(g);
        World w = gen_plan(g, std::uniform_int_distribution<int>(6, 12)(g), 20000, p, mt);
        const auto r0 = ev(w); if (!r0.feasible) continue; if (w.sol.drone_trips.empty()) ++trucks_only_plans;
        const auto tp = trip_pos(w);
        // random slack bounds for every customer: D_i = ETA_i + slack_i
        std::map<NodeId, double> slack; for (const auto& cr : r0.customer_records) slack[cr.customer_id] = (std::uniform_int_distribution<int>(0, 9)(g) == 0) ? 0.0 : std::uniform_real_distribution<double>(0, 900)(g);
        for (const auto& gp : gaps(w)) {
            const Point2D x = rand_pt(g, 20000);
            World w2 = insert_new(w, gp.pb, x); const auto r1 = ev(w2);
            const NodeId a = w.sol.truck_route[gp.pa], b = w.sol.truck_route[gp.pb];
            const double tau = [&](NodeId u, Point2D q) { return dist(xy(w, u), q) / p.vT; }(a, x);
            const double delta = tau + p.Se1 + dist(x, xy(w, b)) / p.vT - dd(w, a, b) / p.vT;
            const bool strad = straddled(w, gp);
            const bool flp_on = [&] { for (int q = gp.pa + 1; q < gp.pb; ++q) if (is_flp(w.sol.truck_route[q])) return true; return false; }();
            // T3: equality / inequality
            const auto T = tail_set(w, gp); const double pred = (r0.truck_timing.departure_time[gp.pa] + tau) + (double)T.size() * delta;
            if (!strad) { // implies no FLP entries on the arc
                if (r1.feasible) { const double act = r1.objective_value - r0.objective_value; ++eq_n; eq_max = std::max(eq_max, std::fabs(act - pred)); if (std::fabs(act - pred) > 1e-6) ++eq_bad; if (!w.sol.drone_trips.empty()) ++eq_with_tail_trips; }
                else { ++eq_bad; static int shown = 0; if (std::getenv("DM1_DEBUG") && shown++ < 4) { std::cout << "   [dbg T3] non-straddled infeasible: gap(" << gp.pa << "," << gp.pb << ") route:"; for (NodeId q : w.sol.truck_route) std::cout << ' ' << q; std::cout << " trips:"; for (const auto& t : tp) std::cout << " (" << t.lp << "->" << t.rp << ")"; std::cout << " reasons:"; for (const auto& rs : r1.violation_reasons) std::cout << " | " << rs; std::cout << "\n"; } }
            } else if (flp_on) { ++flp_gap; if (!r1.feasible) ++flp_gap_infeas; }
            else { if (r1.feasible) { ++strad_feas; if (r1.objective_value - r0.objective_value > pred + 1e-6) ++strad_viol; } else ++strad_infeas; }
            // T4: slack test vs truth (evaluator + bounds on every pre-existing customer)
            bool truth = r1.feasible; if (truth) for (const auto& cr : r0.customer_records) if (eta_of(r1, cr.customer_id) > cr.service_arrival_time + slack[cr.customer_id] + 1e-9) truth = false;
            auto minslack = [&](const std::vector<NodeId>& S) { double m = 1e300; for (NodeId c : S) m = std::min(m, slack[c]); return m; };
            const double ms_spec = strad ? 1e300 : minslack(T);
            const bool spec_pass = delta <= ms_spec + 1e-9;
            if (truth) { ++spec_total_truth; if (!spec_pass) ++spec_false_excl; }
            auto cell = [&](Cells& c, bool pass) { (pass ? (truth ? c.a : c.b) : (truth ? c.c : c.d))++; };
            if (!strad) cell(nonstrad, spec_pass);
            else {
                cell(strad_unrelaxed, delta <= minslack(T) + 1e-9);                    // min over T(g) WITHOUT the +inf relaxation
                cell(naive_all, delta <= minslack(tail_set(w, gp, false, true)) + 1e-9);   // naive: also includes straddling-trip customers
            }
            cell(incl_before_a, delta <= minslack(tail_set(w, gp, true, false)) + 1e-9 || strad); // set-definition control (relaxation kept)
            cell(trucks_only_min, delta <= [&] { double m = 1e300; for (size_t i = gp.pb; i < w.sol.truck_route.size(); ++i) { NodeId c = w.sol.truck_route[i]; if (w.nodes.at(c).type == NodeType::Customer) m = std::min(m, slack[c]); } return m; }() + 1e-9 || strad);
        }
    }
    std::ostringstream o; o << "non-straddled gaps (equality P3): " << eq_n << " feasible cases checked (" << eq_with_tail_trips << " in plans with trips), max |dF - pred| " << std::scientific << eq_max << ", violations " << eq_bad << std::fixed
        << " | straddled, no launch on gap: feasible " << strad_feas << " (inequality dF<=pred violated " << strad_viol << "), infeasible " << strad_infeas << " | launch FLP on the gap: " << flp_gap << " cases, evaluator-infeasible " << flp_gap_infeas;
    report("T3 truck-only insertion cost (P3): equality and applicable inequality", eq_bad == 0 && strad_viol == 0 && eq_n > 0, o.str());
    std::ostringstream q; q << "non-straddled gaps: pass&truth " << nonstrad.a << ", pass&!truth " << nonstrad.b << ", fail&truth " << nonstrad.c << ", fail&!truth " << nonstrad.d
        << " (iff holds iff b=c=0) | spec bound (+inf on straddled): false exclusions " << spec_false_excl << "/" << spec_total_truth << " truth-feasible insertions";
    report("T4a slack test: necessary and sufficient on non-straddled gaps; spec bound has no false exclusion", nonstrad.b == 0 && nonstrad.c == 0 && spec_false_excl == 0 && nonstrad.a + nonstrad.d > 0, q.str());
    std::cout << "      [observation, straddled gaps, bound WITHOUT +inf relaxation]: test-fail&truth-feasible (false exclusion, CE-4 type) " << strad_unrelaxed.c << ", test-pass&truth-infeasible (CE-1 type) " << strad_unrelaxed.b << ", pass&truth " << strad_unrelaxed.a << ", fail&!truth " << strad_unrelaxed.d << "\n"
              << "      [observation, naive set incl. straddling-trip customers]: false exclusions " << naive_all.c << ", pass&infeasible " << naive_all.b << "\n"
              << "      [observation, set-definition controls (relaxation kept)] T(g) incl. customers of trips recovered <= a: false exclusions " << incl_before_a.c << " | truck customers only (omits drone customers launched behind b): pass&infeasible " << trucks_only_min.b << "\n";
    if (strad_unrelaxed.c == 0) { ++g_inconclusive; std::cout << "      [INCONCLUSIVE] fuzz did not exhibit a CE-4-type false exclusion without the relaxation; see the fixed CE-4 case\n"; }
}

// ---- T4 fixed cases: CE-1..CE-4 -------------------------------------------------------------------------------------
World ce1_world() {
    World w; w.p = regimeP(); w.nodes.add(Node{0, NodeType::Depot, {-3000, 0}}); add_cust(w, 1, 6000, 0); add_cust(w, 2, 12000, 0); add_cust(w, 3, 6000, 2000);
    add_flp(w, 1000, 0, 1, 0.0); w.nodes = [&] { NodeRegistry n; for (const auto& kv : w.nodes.all()) if (kv.first != 1000) n.add(kv.second); n.add(Node{1000, NodeType::FlexibleLaunchPoint, {0, 0}}); return n; }();
    w.flp[1000] = FlexibleLaunchPointData{1000, 0, 1, 0.5};
    w.sol.truck_route = {0, 1000, 1, 2, 0}; w.sol.drone_trips = {DroneTrip{0, 1000, 3, 2}}; return w;
}
void run_CE() {
    // CE-1 (spanning trip). Spec coordinates: depot (-3000,0); f (0,0); c1 (6000,0); c2 (12000,0); X (6000,2000); trip (f,X,c2). c2 is node 2 here, X is node 3.
    World w = ce1_world(); const auto r0 = ev(w);
    World w2 = w; add_cust(w2, kNew, 9000, 9000); w2.sol.truck_route.insert(w2.sol.truck_route.begin() + 3, kNew); const auto r1 = ev(w2);
    std::ostringstream o; o << std::fixed << std::setprecision(2) << "feasible " << r0.feasible << ", (36)=" << r0.drone_trip_timings[0].endurance_value_36 << ", (37)=" << r0.drone_trip_timings[0].endurance_value_37 << ", F=" << r0.objective_value
        << " | after insertion: feasible " << r1.feasible << ", t_c2 " << r0.truck_timing.arrival_time[3] << " -> " << r1.truck_timing.arrival_time[4] << ", drone ETA " << r0.drone_trip_timings[0].customer_arrival_time << " -> " << r1.drone_trip_timings[0].customer_arrival_time << ", (36)=" << r1.drone_trip_timings[0].endurance_value_36;
    report("CE-1 spanning trip", r0.feasible && !r1.feasible && r1.drone_trip_timings[0].endurance_value_36 > w.p.e, o.str());
    // prefix arrays identical (P1 with spanning trip)
    bool same = true; for (int q = 0; q <= 2; ++q) same = same && bit_equal(r0.truck_timing.arrival_time[q], r1.truck_timing.arrival_time[q]) && bit_equal(r0.truck_timing.departure_time[q], r1.truck_timing.departure_time[q]);
    report("CE-1 prefix arrays [0..c1] bit-identical although the trip spans the cut", same, "");
    // CE-2: depot (0,0); f (6000,0) on arc (depot,A); A (12000,0); X (6000,3000); trip (f,X,A); r at (6000,-20000) inserted into the gap; FLPs at d=2000 m on (depot,r),(r,A)
    World v; v.p = regimeP(); add_depot(v, 0, 0); add_cust(v, 1, 12000, 0); add_cust(v, 2, 6000, 3000);
    v.nodes.add(Node{1000, NodeType::FlexibleLaunchPoint, {6000, 0}}); v.flp[1000] = FlexibleLaunchPointData{1000, 0, 1, 0.5};
    v.sol.truck_route = {0, 1000, 1, 0}; v.sol.drone_trips = {DroneTrip{0, 1000, 2, 1}}; const auto q0 = ev(v);
    World v2 = v; add_cust(v2, kNew, 6000, -20000); // insert r between depot and A: new route depot - r - A - depot, trip needs a launch FLP on (depot,r) or (r,A)
    tdr::flp::FlpGenerationPolicy pol; pol.unit_division_length = 2000; const std::vector<tdr::flp::Arc> arcs = {{0, kNew}, {kNew, 1}};
    const auto gen = tdr::flp::generate_flexible_launch_points_on_arcs(arcs, v2.nodes, pol, 2000);
    int feasible_candidates = 0, total = 0;
    for (const auto& pt : gen.points) {
        World v3 = v2; v3.nodes.add(pt.node); v3.flp[pt.node.id] = pt.data;
        v3.sol.truck_route = {0, kNew, 1, 0}; v3.sol.truck_route.insert(v3.sol.truck_route.begin() + (pt.data.arc_from == 0 ? 1 : 2), pt.node.id);
        v3.sol.drone_trips = {DroneTrip{0, pt.node.id, 2, 1}}; ++total; if (ev(v3).feasible) ++feasible_candidates;
    }
    std::ostringstream o2; o2 << std::fixed << std::setprecision(2) << "original feasible " << q0.feasible << ", F=" << q0.objective_value << "; replacement FLPs generated " << total << ", evaluator-feasible " << feasible_candidates << " (stale-FLP plan with r inserted is infeasible: "
        << [&] { World v4 = v2; v4.sol.truck_route = {0, 1000, kNew, 1, 0}; return !ev(v4).feasible; }() << ")";
    report("CE-2 launch FLP on the split arc", q0.feasible && total == 20 && feasible_candidates == 0, o2.str());
    // CE-3 re-sequencing: depot D(0,0), P(20000,0), R(20000,20000), Q(0,20000). Current route D-P-R-Q-D (square). X at centre (10000,10000).
    World c; c.p = regimeP(); add_depot(c, 0, 0); add_cust(c, 1, 20000, 0); add_cust(c, 2, 20000, 20000); add_cust(c, 3, 0, 20000); add_cust(c, 4, 10000, 10000);
    // (a) no FLP on any arc of the CURRENT route (fine grid, 100 m) with any recovery node makes X drone-servable
    int cur_feas = 0, cur_total = 0; std::vector<std::pair<NodeId, NodeId>> cur = {{0, 1}, {1, 2}, {2, 3}, {3, 0}}; NodeId idc = 1000;
    for (size_t ai = 0; ai < cur.size(); ++ai) for (int m = 1; m < 200; ++m) {
        World t = c; const double ra = m / 200.0; add_flp(t, idc, cur[ai].first, cur[ai].second, ra);
        // route: 0 1 2 3 0 with the flp inserted on arc ai; recovery any later skeleton node
        std::vector<NodeId> rt = {0, 1, 2, 3, 0}; rt.insert(rt.begin() + ai + 1, idc);
        for (size_t kk = ai + 1; kk < cur.size() + 1; ++kk) { NodeId k = (kk == cur.size()) ? 0 : cur[kk].first; if (k == cur[ai].first) continue; World t2 = t; t2.sol.truck_route = rt; t2.sol.drone_trips = {DroneTrip{0, idc, 4, k}}; ++cur_total; if (ev(t2).feasible) ++cur_feas; }
    }
    // (b) the re-sequenced route D-R-P-Q-D passes through the centre (diagonal D-R): FLP at the centre of arc (D,R) with recovery at R
    World t = c; add_flp(t, idc, 0, 2, 0.5); t.sol.truck_route = {0, idc, 2, 1, 3, 0}; t.sol.drone_trips = {DroneTrip{0, idc, 4, 2}}; const auto rr = ev(t);
    const std::vector<Point2D> poly = {{0, 0}, {20000, 0}, {20000, 20000}, {0, 20000}, {0, 0}};
    auto seg_d = [](Point2D p, Point2D a, Point2D b) { const double dx = b.x - a.x, dy = b.y - a.y; double tt = ((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy); tt = std::max(0.0, std::min(1.0, tt)); return dist(p, {a.x + tt * dx, a.y + tt * dy}); };
    double rho_line = 1e300; for (size_t i = 0; i + 1 < poly.size(); ++i) rho_line = std::min(rho_line, seg_d({10000, 10000}, poly[i], poly[i + 1]));
    const double nu = 14142.135623731, e = c.p.e;
    const bool line_filter_excludes = (1 + c.p.theta) * rho_line / c.p.vD + (1 + c.p.theta) * c.p.Se2 + nu / c.p.vD > e;
    std::ostringstream o3; o3 << std::fixed << std::setprecision(1) << "current route (square): X drone-servable in " << cur_feas << "/" << cur_total << " (FLP,recovery) combinations; polyline distance to X " << rho_line << " m, nearest node " << nu << " m, polyline-version bound violated (filter excludes X) " << line_filter_excludes
       << "; re-sequenced route D-R-P-Q-D with FLP at the centre: evaluator feasible " << rr.feasible << ", (37)=" << (rr.drone_trip_timings.empty() ? -1 : rr.drone_trip_timings[0].endurance_value_37) << ", (36)=" << (rr.drone_trip_timings.empty() ? -1 : rr.drone_trip_timings[0].endurance_value_36);
    report("CE-3 re-sequencing (drone service only after re-ordering; polyline filter excludes)", cur_feas == 0 && rr.feasible && line_filter_excludes, o3.str());
}

void run_CE4() {
    // CE-1 layout plus c3 after c2; route depot - f - c1 - c2 - c3 - depot; trip (f, X, c2); r at (9000,4758) between c1 and c2.
    World w = ce1_world(); add_cust(w, 4, 12000, -4000); w.sol.truck_route = {0, 1000, 1, 2, 4, 0}; const auto r0 = ev(w);
    World w2 = w; add_cust(w2, kNew, 9000, 4758); w2.sol.truck_route.insert(w2.sol.truck_route.begin() + 3, kNew); const auto r1 = ev(w2);
    const double tau_ar = dd(w2, 1, kNew) / w.p.vT, delta = tau_ar + w.p.Se1 + dd(w2, kNew, 2) / w.p.vT - dd(w, 1, 2) / w.p.vT;
    // positions: base route idx: 0 depot, 1 f, 2 c1, 3 c2, 4 c3, 5 depot ; new route has r at idx 3
    const double t_c2_0 = r0.truck_timing.arrival_time[3], t_c2_1 = r1.truck_timing.arrival_time[4];
    const double dep_c2_0 = r0.truck_timing.departure_time[3], dep_c2_1 = r1.truck_timing.departure_time[4];
    const double eta_c3_0 = eta_of(r0, 4), eta_c3_1 = eta_of(r1, 4), eta_c2_0 = eta_of(r0, 2), eta_c2_1 = eta_of(r1, 2);
    double wait0 = 0, wait1 = 0; for (const auto& s : r0.synchronization_records) if (s.recovery_node == 2) wait0 = s.wait_time; for (const auto& s : r1.synchronization_records) if (s.recovery_node == 2) wait1 = s.wait_time;
    // absorbed waiting w = (drone-side completion) - (truck-side completion) at c2 before insertion
    const double truck_side0 = t_c2_0 + w.p.Se1 + 1 * w.p.SR, drone_side0 = r0.drone_trip_timings[0].recovery_arrival_time + w.p.SR; const double w_abs = drone_side0 - truck_side0;
    const double shift_c3 = eta_c3_1 - eta_c3_0;
    std::ostringstream o; o << std::fixed << std::setprecision(3)
        << "base: feasible " << r0.feasible << ", t_c2 " << t_c2_0 << ", drone arrival at c2 " << r0.drone_trip_timings[0].recovery_arrival_time << ", sync wait_time (evaluator) " << wait0 << ", departure c2 " << dep_c2_0 << ", (36) " << r0.drone_trip_timings[0].endurance_value_36 << ", (37) " << r0.drone_trip_timings[0].endurance_value_37
        << " | insertion: delta_g " << delta << ", feasible " << r1.feasible << ", t_c2 " << t_c2_1 << " (+" << (t_c2_1 - t_c2_0) << "), departure c2 " << dep_c2_1 << " (+" << (dep_c2_1 - dep_c2_0) << "), (36) " << r1.drone_trip_timings[0].endurance_value_36 << ", (37) " << r1.drone_trip_timings[0].endurance_value_37
        << " | ETA c2 +" << (eta_c2_1 - eta_c2_0) << ", ETA c3 +" << shift_c3 << ", drone customer ETA " << r0.drone_trip_timings[0].customer_arrival_time << " -> " << r1.drone_trip_timings[0].customer_arrival_time
        << " | absorbed waiting w = drone-side minus truck-side completion at c2: " << w_abs << " (sync-record wait after insertion " << wait1 << ")";
    std::cout << "      CE-4 raw: " << o.str() << "\n";
    const double slack_c3 = 250, slack_c2 = 1e9; const bool feasible_phi = r1.feasible && shift_c3 <= slack_c3 + 1e-9 && (eta_c2_1 - eta_c2_0) <= slack_c2;
    const double naive_min = std::min(slack_c2, slack_c3); const bool naive_excludes = delta > naive_min;
    const double lim = dd(w, 1, 2) + w.p.vT * (naive_min - w.p.Se1); const double sum = dd(w2, 1, kNew) + dd(w2, kNew, 2); // ellipse test with the naive bound
    std::ostringstream o2; o2 << std::fixed << std::setprecision(3) << "hand-derived claims vs evaluator: delta_g=" << delta << " (derived ~380.0); (36)_after=" << r1.drone_trip_timings[0].endurance_value_36 << " (derived 1767.05); w=" << w_abs << " (derived w>=184 s); shift at c3=" << shift_c3 << " (derived <=196); insertion feasible and Phi-satisfying with slack_c3=250: " << feasible_phi
       << "; naive bound min=" << naive_min << " < delta_g -> naive ellipse excludes r: " << naive_excludes << " (ellipse sum " << sum << " > limit " << lim << ")";
    const bool derivation_ok = std::fabs(delta - 380.0) < 0.1 && r1.feasible && std::fabs(r1.drone_trip_timings[0].endurance_value_36 - 1767.05) < 0.2 && w_abs >= 184 && shift_c3 <= 196 + 1e-6 && feasible_phi && naive_excludes && sum > lim;
    report("CE-4 absorbed delay behind a straddling trip (evaluator vs hand derivation), negative control (naive min) falsely excludes", derivation_ok, o2.str());
}

// ---- T5 ---------------------------------------------------------------------------------------------------------------
void run_T5(std::mt19937_64& g, int N) {
    const Params p = regimeP(); long band = 0, in_band_disagree = 0, out_band_disagree = 0, inside = 0;
    auto U = [&](double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(g); };
    for (int it = 0; it < N; ++it) {
        const Point2D a{U(0, 60000), U(0, 60000)}, b{U(0, 60000), U(0, 60000)}, x{U(0, 60000), U(0, 60000)};
        if (dist(a, b) < 1) continue;
        World w; w.p = p; add_depot(w, a.x - 1000, a.y); add_cust(w, 1, a.x, a.y); add_cust(w, 2, b.x, b.y); w.sol.truck_route = {0, 1, 2, 0};
        const auto r0 = ev(w); World w2 = insert_new(w, 2, x); const auto r1 = ev(w2);
        const double det_eval = r1.truck_timing.arrival_time[3] - r0.truck_timing.arrival_time[2]; // t_b' - t_b = delta_g
        const double det_exact = (dist(a, x) + dist(x, b) - dist(a, b)) / p.vT + p.Se1;
        // delta_max: half of the draws exactly on the boundary scale (within +-1e-6 relative of det_eval), others uniform
        const double dmax = (it % 2) ? det_eval * (1 + U(-1e-6, 1e-6)) : U(0, 4000);
        const bool in_ellipse = dist(a, x) + dist(x, b) <= dist(a, b) + p.vT * (dmax - p.Se1);
        const bool in_exact = det_eval <= dmax; const double scale = std::max(1.0, std::fabs(dmax));
        if (in_ellipse) ++inside;
        if (in_ellipse != in_exact) { if (std::fabs(det_eval - dmax) <= 1e-9 * scale) ++in_band_disagree; else ++out_band_disagree; }
        if (std::fabs(det_eval - dmax) <= 1e-9 * scale) ++band; (void)det_exact;
    }
    std::ostringstream o; o << "points " << N << " (inside ellipse " << inside << "), within 1e-9-relative boundary band " << band << ", disagreements outside the band " << out_band_disagree << ", inside the band " << in_band_disagree << " (rounding)";
    report("T5 ellipse membership vs evaluator detour time (P5)", out_band_disagree == 0, o.str());
}

// ---- T6 ---------------------------------------------------------------------------------------------------------------
double seg_dist(Point2D p, Point2D a, Point2D b) { const double dx = b.x - a.x, dy = b.y - a.y; const double L2 = dx * dx + dy * dy; double t = L2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / L2 : 0; t = std::max(0.0, std::min(1.0, t)); return dist(p, {a.x + t * dx, a.y + t * dy}); }
double tri_dist(Point2D p, Point2D a, Point2D b, Point2D c) {
    auto cr = [](Point2D o, Point2D u, Point2D v) { return (u.x - o.x) * (v.y - o.y) - (u.y - o.y) * (v.x - o.x); };
    const double d1 = cr(a, b, p), d2 = cr(b, c, p), d3 = cr(c, a, p);
    if ((d1 >= 0 && d2 >= 0 && d3 >= 0) || (d1 <= 0 && d2 <= 0 && d3 <= 0)) return 0;
    return std::min({seg_dist(p, a, b), seg_dist(p, b, c), seg_dist(p, c, a)});
}
// bisection on a scalar s in [lo,hi]: largest s with feasible(s); assumes feasible at lo, infeasible at hi
template <class F> double bisect(F feasible, double lo, double hi) { for (int i = 0; i < 100; ++i) { const double m = 0.5 * (lo + hi); (feasible(m) ? lo : hi) = m; } return lo; }
void run_T6(std::mt19937_64& g, int N) {
    auto U = [&](double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(g); };
    // boundary constructions R1-R3, regimes P and F (exact formulas recomputed from the parameters)
    for (int regime = 0; regime < 2; ++regime) {
        const Params p = regime ? regimeF() : regimeP();
        const double rstar = reach_radius(p), vD = p.vD, e = p.e, th = p.theta, S2 = p.Se2;
        // R1: f = depot (0,0), j = (D,0), k = (D+1,0); d_jk = 1 m
        auto feas1 = [&](double D) { World w; w.p = p; add_depot(w, 0, 0); add_cust(w, 1, D, 0); add_cust(w, 2, D + 1, 0); w.nodes.add(Node{1000, NodeType::FlexibleLaunchPoint, {0, 0}}); w.flp[1000] = FlexibleLaunchPointData{1000, 0, 2, 0.0}; w.sol.truck_route = {0, 1000, 2, 0}; w.sol.drone_trips = {DroneTrip{0, 1000, 1, 2}}; return ev(w).feasible; };
        const double R1 = bisect(feas1, 100, 100000), R1f = vD * (e - (1 + th) * S2) / (1 + th) - 0.5; // (37): (1+th)(d/vD+S2)+1/vD<=e  => d = vD(e-(1+th)S2-1/vD)/(1+th)
        const double R1formula = vD * (e / (1 + th) - S2) - 0.5 * 2.0 / (1 + th) * 1.0 / 1.0 * 0 ; (void)R1f; (void)R1formula;
        const double R1exact = vD * (e - (1 + th) * S2 - 1.0 / vD) / (1 + th);
        // R2: depot (-5000,0), f (-0.5,0), k (0.5,0), j (0,rho): d_ij ~ d_jk ~ rho
        auto feas2 = [&](double rho) { World w; w.p = p; add_depot(w, -5000, 0); add_cust(w, 1, 0, rho); add_cust(w, 2, 0.5, 0); w.nodes.add(Node{1000, NodeType::FlexibleLaunchPoint, {-0.5, 0}}); w.flp[1000] = FlexibleLaunchPointData{1000, 0, 2, 0.5}; w.sol.truck_route = {0, 1000, 2, 0}; w.sol.drone_trips = {DroneTrip{0, 1000, 1, 2}}; return ev(w).feasible; };
        const double R2 = bisect(feas2, 100, 100000);
        // R3: f = depot, j = (1,0), k = (D,0): d_ij = 1, return leg D-1
        auto feas3 = [&](double D) { World w; w.p = p; add_depot(w, 0, 0); add_cust(w, 1, 1, 0); add_cust(w, 2, D, 0); w.nodes.add(Node{1000, NodeType::FlexibleLaunchPoint, {0, 0}}); w.flp[1000] = FlexibleLaunchPointData{1000, 0, 2, 0.0}; w.sol.truck_route = {0, 1000, 2, 0}; w.sol.drone_trips = {DroneTrip{0, 1000, 1, 2}}; return ev(w).feasible; };
        const double R3 = bisect(feas3, 100, 200000);
        const double R3exact = 1.0 + vD * (e - (1 + th) * (1.0 / vD + S2)); // d_jk = vD(e-(1+th)L)
        std::ostringstream o; o << std::fixed << std::setprecision(3) << (regime ? "regime F (e=" : "regime P (e=") << e << "): r_D* formula " << rstar << " m | R1 (d_jk=1) bisected " << R1 << " vs (37)-exact " << R1exact << " | R2 symmetric bisected " << R2 << " (distance of j to the route = " << R2 << ") | R3 (d_ij=1, k at x=D) bisected D " << R3 << " vs formula " << R3exact << " (return leg d_jk = D-1 = " << (R3 - 1) << "; total d_ij+d_jk = D) | v_D*e = " << vD * e;
        report(std::string("T6 boundary constructions ") + (regime ? "F" : "P"), std::fabs(R1 - R1exact) < 1e-3 && std::fabs(R2 - rstar) < 1e-2 && std::fabs(R3 - R3exact) < 1e-2, o.str());
        // boundary at r* +- 1e-3 for R2
        report(std::string("T6 boundary at r_D* +-1e-3 m ") + (regime ? "F" : "P"), feas2(rstar - 1e-3) && !feas2(rstar + 1e-3), "feasible just inside, infeasible just outside");
    }
    // fuzz: random parameters and random trips; every evaluator-feasible trip must have rho_hull(j) <= r*
    long total = 0, feasible = 0, viol = 0, lemma_bad = 0; double maxratio = 0; long infeasible_param = 0;
    for (int it = 0; it < N; ++it) {
        Params p; p.e = U(600, 4000); p.theta = U(0, 2); p.Se2 = U(0, 120); p.vD = U(8, 20); p.SL = U(0, 120); p.SR = U(0, 80);
        if (it % 3 == 0) p = regimeP(); else if (it % 3 == 1) p = regimeF();
        const double rstar = reach_radius(p); if (rstar <= 0) ++infeasible_param;
        World w; w.p = p; const Point2D D{U(0, 20000), U(0, 20000)}, C{U(0, 20000), U(0, 20000)}, K{U(0, 20000), U(0, 20000)}, J{U(0, 20000), U(0, 20000)};
        add_depot(w, D.x, D.y); add_cust(w, 1, C.x, C.y); add_cust(w, 2, K.x, K.y); add_cust(w, 3, J.x, J.y);
        add_flp(w, 1000, 0, 1, U(0.02, 0.98));
        w.sol.truck_route = {0, 1000, 1, 2, 0}; w.sol.drone_trips = {DroneTrip{0, 1000, 3, 2}};
        ++total; const auto r = ev(w); if (!r.feasible) continue; ++feasible;
        const double rho = tri_dist(J, D, C, K);
        const Point2D F = xy(w, 1000); if (dist(F, J) < rho - 1e-6 || dist(J, K) < rho - 1e-6) ++lemma_bad;
        if (rho > rstar + 1e-6) ++viol; if (rstar > 0) maxratio = std::max(maxratio, rho / rstar);
    }
    std::ostringstream o; o << std::fixed << std::setprecision(4) << "random trips " << total << " (parameters incl. regimes P/F), evaluator-feasible " << feasible << ", violations of rho_hull <= r_D* " << viol << ", geometric-lemma failures (d_ij,d_jk >= rho) " << lemma_bad << ", max rho/r* among feasible " << maxratio << ", draws with r*<=0: " << infeasible_param;
    report("T6 reach is a necessary condition (fuzz)", viol == 0 && lemma_bad == 0 && feasible > 100, o.str());
}
} // namespace

int main(int argc, char** argv) {
    const double scale = argc > 1 ? std::atof(argv[1]) : 1.0;
    std::cout << "DM1 probes (read-only; unchanged evaluator). Regime P r_D* = " << reach_radius(regimeP()) << " m; regime F (endurance doubled) r_D* = " << reach_radius(regimeF()) << " m\n";
    std::mt19937_64 g(20261109);
    run_CE(); run_CE4();
    run_T1(g, (int)(4000 * scale)); run_T2(g, (int)(4000 * scale)); run_T3_T4(g, (int)(4000 * scale)); run_T5(g, (int)(1000000 * scale)); run_T6(g, (int)(300000 * scale));
    std::cout << "\nfailures " << g_fail << ", inconclusive " << g_inconclusive << "\n";
    return g_fail ? 1 : 0;
}
