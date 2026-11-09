import os, shutil, subprocess, sys
# M4 mutation harness: injects one bug at a time into a COPY of include/ and tests/ and checks that the M4 tests (test_n3_n6, test_n3_n6_oracle) fail.
# usage: TDR_REPO=<repo> TDR_LIB=<path of libtdr_core.a from a Release build> TDR_WORK=<scratch dir> python3 m4_mutate.py [name-prefix ...]
# The repository is never modified. 'CW-*' mutants are applied to every occurrence of their pattern.
R = os.environ.get('TDR_REPO', os.getcwd())
W = os.environ.get('TDR_WORK', '/tmp/tdr_m4_mut') + '/w'
LIB = os.environ['TDR_LIB']

N3 = 'include/search/neighbourhood_n3.hpp'
N6 = 'include/search/neighbourhood_n6.hpp'
TE = 'include/search/trip_exchange.hpp'
CW = 'include/search/candidate_window.hpp'

M = [
 # ---------------- N3 ----------------
 ('N3-01 predecessor not searched', N3, 'if (at != seq.begin()) detail_exchange::judge', 'if (false) detail_exchange::judge'),
 ('N3-02 successor not searched', N3, 'if (at + 1 != seq.end()) detail_exchange::judge', 'if (false) detail_exchange::judge'),
 ('N3-03 successor before predecessor', N3, '    if (at != seq.begin()) detail_exchange::judge(base, ctx, opt, MoveKind::N3Exchange, trip_index, *(at - 1), res); // predecessor\n    if (at + 1 != seq.end()) detail_exchange::judge(base, ctx, opt, MoveKind::N3Exchange, trip_index, *(at + 1), res); // successor\n',
  '    if (at + 1 != seq.end()) detail_exchange::judge(base, ctx, opt, MoveKind::N3Exchange, trip_index, *(at + 1), res);\n    if (at != seq.begin()) detail_exchange::judge(base, ctx, opt, MoveKind::N3Exchange, trip_index, *(at - 1), res);\n'),
 ('N3-04 drone sequence ordered by trip index, not launch position', N3, 'std::tie(a.launch, a.recovery, a.index) < std::tie(b.launch, b.recovery, b.index)', 'std::tie(a.index, a.launch, a.recovery) < std::tie(b.index, b.launch, b.recovery)'),
 ('N3-05 same-drone filter dropped (all trips in one sequence)', N3, 'if (t.drone_id != drone) continue;', ''),
 ('N3-06 sequence in descending launch order', N3, 'std::tie(a.launch, a.recovery, a.index) < std::tie(b.launch, b.recovery, b.index)', 'std::tie(a.launch, a.recovery, a.index) > std::tie(b.launch, b.recovery, b.index)'),
 ('N3-07 successor = LAST trip of the drone (non-consecutive)', N3, '*(at + 1), res); // successor', 'seq.back(), res); // successor'),
 ('N3-08 predecessor = FIRST trip of the drone (non-consecutive)', N3, '*(at - 1), res); // predecessor', 'seq.front(), res); // predecessor'),
 ('N3-09 the adjacency window wrongly applied (w = 0 -> nothing)', N3, 'if (!detail_exchange::begin(base, ctx, OperatorId::N3, trip_index, res)) return res;', 'if (!detail_exchange::begin(base, ctx, OperatorId::N3, trip_index, res)) return res;\n    if (ctx.adjacency_window < 1) return res;'),
 ('N3-10 Rng overload always picks trip 0', N3, 'rng.uniform_index(base.drone_trips.size())', '0'),
 ('N3-11 Rng overload without the no-trip guard', N3, 'if (base.drone_trips.empty()) {', 'if (false) {'),
 ('N3-12 sequence uses the recovery position as the launch position', N3, 'placed.push_back(Placed{l, rp ? *rp : s.truck_route.size(), k});', 'placed.push_back(Placed{rp ? *rp : s.truck_route.size(), l, k});'),
 ('N3-13 operator id reported as N6', N3, 'OperatorId::N3, trip_index, res)) return res;\n\n    const', 'OperatorId::N6, trip_index, res)) return res;\n\n    const'),
 # ---------------- N6 ----------------
 ('N6-01 same-drone partners not excluded', N6, 'if (!opt.include_same_drone && base.drone_trips[j].drone_id == drone) continue;', ''),
 ('N6-02 include_same_drone ignored (always excluded)', N6, 'if (!opt.include_same_drone && base.drone_trips[j].drone_id == drone) continue;', 'if (base.drone_trips[j].drone_id == drone) continue;'),
 ('N6-03 drone filter inverted (only same-drone partners)', N6, 'if (!opt.include_same_drone && base.drone_trips[j].drone_id == drone) continue;', 'if (!opt.include_same_drone && base.drone_trips[j].drone_id != drone) continue;'),
 ('N6-04 window +1', N6, 'pi.neighbours(res.anchor_customer, ctx.adjacency_window)', 'pi.neighbours(res.anchor_customer, ctx.adjacency_window + 1)'),
 ('N6-05 window -1', N6, 'pi.neighbours(res.anchor_customer, ctx.adjacency_window)', 'pi.neighbours(res.anchor_customer, ctx.adjacency_window - 1)'),
 ('N6-06 truck customers not skipped', N6, 'if (trips.empty()) continue; // a truck-served customer: that is N5', ''),
 ('N6-07 self-pair allowed', N6, 'if (j == trip_index) continue;', ''),
 ('N6-08 partners visited farthest first', N6, 'for (tdr::model::NodeId d : pi.neighbours(res.anchor_customer, ctx.adjacency_window)) {', 'for (tdr::model::NodeId d : [&] { auto v = pi.neighbours(res.anchor_customer, ctx.adjacency_window); std::reverse(v.begin(), v.end()); return v; }()) {'),
 ('N6-09 partners visited by customer id only', N6, 'for (tdr::model::NodeId d : pi.neighbours(res.anchor_customer, ctx.adjacency_window)) {', 'for (tdr::model::NodeId d : [&] { auto v = pi.neighbours(res.anchor_customer, ctx.adjacency_window); std::sort(v.begin(), v.end()); return v; }()) {'),
 ('N6-10 move kind reported as N3Exchange', N6, 'MoveKind::N6Swap, trip_index, j, res);', 'MoveKind::N3Exchange, trip_index, j, res);'),
 ('N6-11 Rng overload always picks trip 0', N6, 'rng.uniform_index(base.drone_trips.size())', '0'),
 ('N6-12 Rng overload without the no-trip guard', N6, 'if (base.drone_trips.empty()) {', 'if (false) {'),
 ('N6-13 default include_same_drone = false (A8 default reverted)', N6, 'bool include_same_drone = true;', 'bool include_same_drone = false;'),
 ('N6-14 operator id reported as N3', N6, 'OperatorId::N6, trip_index, res)) return res;', 'OperatorId::N3, trip_index, res)) return res;'),
 # ---------------- shared exchange step ----------------
 ('TE-01 tie-break last wins (<=)', TE, 'if (!res.best || v.objective < res.best->objective) {', 'if (!res.best || v.objective <= res.best->objective) {'),
 ('TE-02 customers not exchanged', TE, '    std::swap(cand.drone_trips[i].service_customer, cand.drone_trips[j].service_customer);\n', ''),
 ('TE-03 launch nodes exchanged too', TE, '    std::swap(cand.drone_trips[i].service_customer, cand.drone_trips[j].service_customer);\n', '    std::swap(cand.drone_trips[i].service_customer, cand.drone_trips[j].service_customer);\n    std::swap(cand.drone_trips[i].launch_node, cand.drone_trips[j].launch_node);\n'),
 ('TE-04 recovery nodes exchanged too', TE, '    std::swap(cand.drone_trips[i].service_customer, cand.drone_trips[j].service_customer);\n', '    std::swap(cand.drone_trips[i].service_customer, cand.drone_trips[j].service_customer);\n    std::swap(cand.drone_trips[i].recovery_node, cand.drone_trips[j].recovery_node);\n'),
 ('TE-05 drone ids exchanged too', TE, '    std::swap(cand.drone_trips[i].service_customer, cand.drone_trips[j].service_customer);\n', '    std::swap(cand.drone_trips[i].service_customer, cand.drone_trips[j].service_customer);\n    std::swap(cand.drone_trips[i].drone_id, cand.drone_trips[j].drone_id);\n'),
 ('TE-06 evaluate_candidates ignored', TE, 'if (opt.evaluate_candidates) {', 'if (true) {'),
 ('TE-07 generated not counted', TE, '    ++res.generated;\n', ''),
 ('TE-08 evaluated not counted', TE, '        ++res.evaluated;\n', ''),
 ('TE-09 evaluator rejection counted twice', TE, '            ++res.rejected[static_cast<std::size_t>(v.reason)];', '            ++res.rejected[static_cast<std::size_t>(v.reason)]; ++res.rejected[static_cast<std::size_t>(v.reason)];'),
 ('TE-10 evaluator rejection not counted', TE, '            ++res.rejected[static_cast<std::size_t>(v.reason)];\n', ''),
 ('TE-11 evaluator reason overridden to EvalOther', TE, 'rec.reason = v.reason;', 'rec.reason = v.feasible ? v.reason : RejectReason::EvalOther;'),
 ('TE-12 move.customer taken from the partner trip', TE, 'm.customer = base.drone_trips[i].service_customer;', 'm.customer = base.drone_trips[j].service_customer;'),
 ('TE-13 partner_trip not recorded', TE, '    m.partner_trip = j;\n', ''),
 ('TE-14 replaced_trip not recorded', TE, '    m.replaced_trip = i;\n', ''),
 ('TE-15 infeasible input accepted', TE, 'if (!base_eval.feasible) {', 'if (false) {'),
 ('TE-16 anchor range check skipped', TE, 'if (trip_index >= base.drone_trips.size()) {', 'if (false) {'),
 ('TE-17 base objective not reported', TE, '    res.base_objective = base_eval.objective_value;\n', ''),
 ('TE-18 anchor customer not reported', TE, '    res.anchor_customer = base.drone_trips[trip_index].service_customer;\n', ''),
 ('TE-19 candidate solution not recorded', TE, 'rec.solution = std::move(cand);', ''),
 ('TE-20 best carries the base solution', TE, 'c.solution = cand;', 'c.solution = base;'),
 ('TE-21 infeasible candidates accepted', TE, 'if (v.feasible) {', 'if (true) {'),
 ('TE-22 accepted not counted', TE, '            ++res.accepted;\n', ''),
 ('TE-23 anchor trip not reported', TE, '    res.anchor_trip = trip_index;\n', ''),
 ('TE-24 best objective not strict (equal to base accepted as best on ties across candidates)', TE, 'if (!res.best || v.objective < res.best->objective) {', 'if (!res.best || v.objective < res.best->objective || (v.objective == res.best->objective && j > i)) {'),
 # ---------------- neighbour search helper used by N6 ----------------
 ('CW-01 neighbour window strict (<)', CW, 'if (d <= 2 * window) found.emplace_back(d, kv.first);', 'if (d < 2 * window) found.emplace_back(d, kv.first);'),
 ('CW-02 neighbour window +1 half-step', CW, 'if (d <= 2 * window) found.emplace_back(d, kv.first);', 'if (d <= 2 * window + 1) found.emplace_back(d, kv.first);'),
 ('CW-03 neighbours farthest first', CW, '        std::sort(found.begin(), found.end());\n        std::vector<tdr::model::NodeId> out;', '        std::sort(found.begin(), found.end(), [](const auto& a, const auto& b) { return b < a; });\n        std::vector<tdr::model::NodeId> out;'),
 ('CW-04 drone customer half-step 2g (instead of 2g+1)', CW, 'half_.emplace(t.service_customer, 2 * static_cast<int>(*g) + 1);', 'half_.emplace(t.service_customer, 2 * static_cast<int>(*g));'),
 ('CW-05 drone customer half-step 2g+2', CW, 'half_.emplace(t.service_customer, 2 * static_cast<int>(*g) + 1);', 'half_.emplace(t.service_customer, 2 * static_cast<int>(*g) + 2);'),
 ('CW-06 truck customer half-step 2k+1', CW, 'half_[v] = 2 * static_cast<int>(k);', 'half_[v] = 2 * static_cast<int>(k) + 1;'),
 ('CW-07 neighbours: ties not broken by id (stable on map order)', CW, '        std::sort(found.begin(), found.end());\n        std::vector<tdr::model::NodeId> out;', '        std::stable_sort(found.begin(), found.end(), [](const auto& a, const auto& b) { return a.first < b.first; });\n        std::vector<tdr::model::NodeId> out;'),
]

only = sys.argv[1:] 
for name, f, old, new in M:
    if only and not any(name.startswith(o) for o in only): continue
    shutil.rmtree(W, ignore_errors=True)
    os.makedirs(W)
    shutil.copytree(R + '/include', W + '/include')
    shutil.copytree(R + '/tests', W + '/tests')
    p = W + '/' + f
    s = open(p).read()
    allc = name.startswith('CW')
    if (s.count(old) != 1) and not allc:
        print(f'{name}: PATTERN COUNT {s.count(old)}', flush=True); continue
    open(p, 'w').write(s.replace(old, new))
    kills = []
    tests = tuple(os.environ.get('TDR_TESTS', 'test_n3_n6,test_n3_n6_oracle').split(','))  # TDR_TESTS=a,b selects other test programs (e.g. the M2/M3 suites)
    procs = [(t, subprocess.Popen(['g++', '-std=c++20', '-O1', '-I' + W + '/include', '-I' + W + '/tests', '-o', W + '/' + t, W + '/tests/' + t + '.cpp', LIB], stdout=subprocess.PIPE, stderr=subprocess.PIPE)) for t in tests]
    for t, pr in procs:
        pr.communicate()
        if pr.returncode != 0:
            kills.append(t + ':COMPILE-ERROR'); continue
        r = subprocess.run([W + '/' + t], capture_output=True, text=True, timeout=300)
        if r.returncode != 0: kills.append(t)
    if any('COMPILE-ERROR' in k for k in kills): status = 'INVALID MUTANT (does not compile): ' + ', '.join(kills)
    else: status = 'KILLED by ' + ', '.join(kills) if kills else 'SURVIVED'
    print(f'{name}: {status}', flush=True)
