import os, shutil, subprocess, sys
# M5 mutation harness: injects one bug at a time into a COPY of include/ and tests/ and checks that the M5 tests (test_n1_n2, test_n1_n2_oracle) fail.
# usage: TDR_REPO=<repo> TDR_LIB=<path of libtdr_core.a from a Release build> TDR_WORK=<scratch dir> python3 m5_mutate.py [name-prefix ...]
# The repository is never modified. 'CW-*' mutants are applied to every occurrence of their pattern.
R = os.environ.get('TDR_REPO', os.getcwd())
W = os.environ.get('TDR_WORK', '/tmp/tdr_m5_mut') + '/w'
LIB = os.environ['TDR_LIB']

N1 = 'include/search/neighbourhood_n1.hpp'
N2 = 'include/search/neighbourhood_n2.hpp'
TR = 'include/search/trip_relocate.hpp'
RO = 'include/search/route_ops.hpp'
CW = 'include/search/candidate_window.hpp'

M = [
 # ---------------- N1 ----------------
 ('N1-01 window +1 (gap coordinate)', N1, 'ctx.adjacency_window)) {\n        for (NodeId f', 'ctx.adjacency_window + 1)) {\n        for (NodeId f'),
 ('N1-02 window -1', N1, 'ctx.adjacency_window)) {\n        for (NodeId f', 'ctx.adjacency_window - 1)) {\n        for (NodeId f'),
 ('N1-03 window ignored (only the launch gap)', N1, 'ctx.adjacency_window)) {\n        for (NodeId f', '0)) {\n        for (NodeId f'),
 ('N1-04 half-step of the launch gap taken as 2g (a truck-customer coordinate)', N1, '2 * static_cast<int>(site->launch_gap) + 1', '2 * static_cast<int>(site->launch_gap)'),
 ('N1-05 current launch not excluded', N1, 'if (f == old_launch) continue;', ''),
 ('N1-06 FLP inserted at the gap start (not ratio order)', N1, 'const auto pos = flp_insertion_position(cand.truck_route, sk, g, f, ctx.flp_data);', 'const std::optional<std::size_t> pos = sk.route_position[g] + 1;'),
 ('N1-07 FLP inserted at the gap end (not ratio order)', N1, 'const auto pos = flp_insertion_position(cand.truck_route, sk, g, f, ctx.flp_data);', 'const std::optional<std::size_t> pos = sk.route_position[g + 1];'),
 ('N1-08 FLP inserted even if already on the route (duplicate entry)', N1, 'if (std::find(cand.truck_route.begin(), cand.truck_route.end(), f) == cand.truck_route.end()) {', 'if (true) {'),
 ('N1-09 FLP never inserted', N1, 'cand.truck_route.insert(cand.truck_route.begin() + static_cast<std::ptrdiff_t>(*pos), f);', ''),
 ('N1-10 launch not changed', N1, '    cand.drone_trips[trip_index].launch_node = f;\n', ''),
 ('N1-11 old launch always dropped (even if shared)', N1, 'if (trips_launching_from(cand, old_launch).empty()) {', 'if (true) {'),
 ('N1-12 old launch never dropped (idle FLP)', N1, 'if (trips_launching_from(cand, old_launch).empty()) {', 'if (false) {'),
 ('N1-13 dropped FLP not reported', N1, '                dropped.push_back(old_launch);\n', ''),
 ('N1-14 recovery changed to repair the order', N1, '            if (site->recovery_sk < g + 1) detail_relocate::reject_invalid_recovery(std::move(cand), m, trip_index, std::move(dropped), opt, res);',
  '            if (site->recovery_sk < g + 1) { cand.drone_trips[trip_index].recovery_node = sk.node[g + 1]; detail_relocate::judge(std::move(cand), m, std::move(dropped), ctx, opt, res); }'),
 ('N1-15 InvalidRecovery check off by one (recovery == next skeleton node rejected)', N1, 'if (site->recovery_sk < g + 1)', 'if (site->recovery_sk < g + 2)'),
 ('N1-16 InvalidRecovery check off by one (recovery before launch allowed at the boundary)', N1, 'if (site->recovery_sk < g + 1)', 'if (site->recovery_sk < g)'),
 ('N1-17 InvalidRecovery never raised (evaluator sees the violation)', N1, 'if (site->recovery_sk < g + 1)', 'if (false)'),
 ('N1-18 gap order reversed', N1, 'for (std::size_t g : gaps_within_window(sk, 2 * static_cast<int>(site->launch_gap) + 1, ctx.adjacency_window)) {', 'for (std::size_t g : [&] { auto v = gaps_within_window(sk, 2 * static_cast<int>(site->launch_gap) + 1, ctx.adjacency_window); std::reverse(v.begin(), v.end()); return v; }()) {'),
 ('N1-19 move kind reported as N2', N1, 'm.kind = MoveKind::N1LaunchRelocate;', 'm.kind = MoveKind::N2RecoveryRelocate;'),
 ('N1-20 replaced_node not recorded', N1, '            m.replaced_node = old_launch;\n', ''),
 ('N1-21 drone id not recorded', N1, '            m.drone_id = trip.drone_id;\n', ''),
 ('N1-22 operator id reported as N2', N1, 'OperatorId::N1, trip_index, res)) return res;', 'OperatorId::N2, trip_index, res)) return res;'),
 ('N1-23 Rng overload always picks trip 0', N1, 'rng.uniform_index(base.drone_trips.size())', '0'),
 ('N1-24 Rng overload without the no-trip guard', N1, 'if (base.drone_trips.empty()) {', 'if (false) {'),
 ('N1-25 drone id changed (trip moved to drone 0)', N1, '    cand.drone_trips[trip_index].launch_node = f;\n', '    cand.drone_trips[trip_index].launch_node = f; cand.drone_trips[trip_index].drone_id = 0;\n'),
 ('N1-26 depot-arc gap (g = 0) skipped', N1, '            if (f == old_launch) continue;\n', '            if (f == old_launch) continue;\n            if (g == 0) continue;\n'),
 # ---------------- N2 ----------------
 ('N2-01 window +1', N2, 'const long w = ctx.adjacency_window;', 'const long w = ctx.adjacency_window + 1;'),
 ('N2-02 window -1', N2, 'const long w = ctx.adjacency_window;', 'const long w = ctx.adjacency_window - 1;'),
 ('N2-03 start depot allowed as recovery (s from 0)', N2, 'for (long s = std::max<long>(1, s_k - w);', 'for (long s = std::max<long>(0, s_k - w);'),
 ('N2-04 end depot excluded', N2, 'std::min<long>(last, s_k + w)', 'std::min<long>(last - 1, s_k + w)'),
 ('N2-05 current recovery not excluded', N2, '        if (s == s_k) continue;\n', ''),
 ('N2-06 order descending', N2, 'for (long s = std::max<long>(1, s_k - w); s <= std::min<long>(last, s_k + w); ++s) {', 'for (long s = std::min<long>(last, s_k + w); s >= std::max<long>(1, s_k - w); --s) {'),
 ('N2-07 recovery not changed', N2, '        cand.drone_trips[trip_index].recovery_node = sk.node[si];\n', ''),
 ('N2-08 drone id changed (trip moved to drone 0)', N2, '        cand.drone_trips[trip_index].recovery_node = sk.node[si];\n', '        cand.drone_trips[trip_index].recovery_node = sk.node[si]; cand.drone_trips[trip_index].drone_id = 0;\n'),
 ('N2-09 InvalidRecovery off by one (recovery at gap+1 rejected)', N2, 'if (si < site->launch_gap + 1)', 'if (si < site->launch_gap + 2)'),
 ('N2-10 InvalidRecovery off by one (recovery at the launch gap start allowed)', N2, 'if (si < site->launch_gap + 1)', 'if (si < site->launch_gap)'),
 ('N2-11 InvalidRecovery never raised', N2, 'if (si < site->launch_gap + 1)', 'if (false)'),
 ('N2-12 move kind reported as N1', N2, 'm.kind = MoveKind::N2RecoveryRelocate;', 'm.kind = MoveKind::N1LaunchRelocate;'),
 ('N2-13 replaced_node not recorded', N2, '        m.replaced_node = trip.recovery_node;\n', ''),
 ('N2-14 operator id reported as N1', N2, 'OperatorId::N2, trip_index, res)) return res;', 'OperatorId::N1, trip_index, res)) return res;'),
 ('N2-15 Rng overload always picks trip 0', N2, 'rng.uniform_index(base.drone_trips.size())', '0'),
 ('N2-16 Rng overload without the no-trip guard', N2, 'if (base.drone_trips.empty()) {', 'if (false) {'),
 ('N2-17 an FLP is dropped/changed in the route (route not preserved)', N2, '        cand.drone_trips[trip_index].recovery_node = sk.node[si];\n', '        cand.drone_trips[trip_index].recovery_node = sk.node[si];\n        if (cand.truck_route.size() > 3) cand.truck_route.erase(cand.truck_route.begin() + 1);\n'),
 # ---------------- shared relocate step ----------------
 ('TR-01 tie-break last wins (<=)', TR, 'if (!res.best || v.objective < res.best->objective) {', 'if (!res.best || v.objective <= res.best->objective) {'),
 ('TR-02 evaluate_candidates ignored', TR, 'if (opt.evaluate_candidates) {', 'if (true) {'),
 ('TR-03 generated not counted (judge)', TR, '    ++res.generated;\n    CandidateRecord rec;', '    CandidateRecord rec;'),
 ('TR-04 generated not counted (invalid recovery)', TR, '    ++res.generated;\n    ++res.rejected[static_cast<std::size_t>(RejectReason::InvalidRecovery)];', '    ++res.rejected[static_cast<std::size_t>(RejectReason::InvalidRecovery)];'),
 ('TR-05 evaluated not counted', TR, '        ++res.evaluated;\n', ''),
 ('TR-06 invalid recovery not counted as rejected', TR, '    ++res.rejected[static_cast<std::size_t>(RejectReason::InvalidRecovery)];\n', ''),
 ('TR-07 invalid recovery recorded with another reason', TR, 'rec.reason = RejectReason::InvalidRecovery;', 'rec.reason = RejectReason::EvalOther;'),
 ('TR-08 invalid recovery trip not named', TR, '        rec.affected_trips = {trip};\n', ''),
 ('TR-09 evaluator rejection not counted', TR, '            ++res.rejected[static_cast<std::size_t>(v.reason)];\n', ''),
 ('TR-10 evaluator reason overridden to EvalOther', TR, 'rec.reason = v.reason;', 'rec.reason = v.feasible ? v.reason : RejectReason::EvalOther;'),
 ('TR-11 infeasible candidates accepted', TR, 'if (v.feasible) {', 'if (true) {'),
 ('TR-12 accepted not counted', TR, '            ++res.accepted;\n', ''),
 ('TR-13 best carries the base solution', TR, '                c.solution = cand;\n', ''),
 ('TR-14 dropped FLPs not reported in the record', TR, '    rec.flps_dropped = dropped;\n', ''),
 ('TR-15 candidate solution not recorded', TR, '        rec.solution = std::move(cand);\n        res.log.push_back(std::move(rec));\n    }\n}\n\n// The trip', '        res.log.push_back(std::move(rec));\n    }\n}\n\n// The trip'),
 ('TR-16 launch gap located one too early', TR, 'site.launch_gap = *g;', 'site.launch_gap = *g > 0 ? *g - 1 : 0;'),
 ('TR-17 recovery located at the launch position (search from lp instead of lp+1)', TR, 'find_route_position_from(base.truck_route, t.recovery_node, *lp + 1)', 'find_route_position_from(base.truck_route, t.recovery_node, 0)'),
 # ---------------- helpers used by N1 ----------------
 ('RO-01 FLP ratio order reversed in the insertion position', RO, 'other->second.position_ratio < mine->second.position_ratio ||', 'other->second.position_ratio > mine->second.position_ratio ||'),
 ('CW-01 gap window strict (<)', CW, 'if (std::abs(2 * static_cast<int>(g) + 1 - h) <= 2 * window) out.push_back(g);', 'if (std::abs(2 * static_cast<int>(g) + 1 - h) < 2 * window) out.push_back(g);'),
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
    tests = tuple(os.environ.get('TDR_TESTS', 'test_n1_n2,test_n1_n2_oracle').split(','))
    procs = [(t, subprocess.Popen(['g++', '-std=c++20', '-O1', '-I' + W + '/include', '-I' + W + '/tests', '-o', W + '/' + t, W + '/tests/' + t + '.cpp', LIB], stdout=subprocess.PIPE, stderr=subprocess.PIPE)) for t in tests]
    for t, pr in procs:
        pr.communicate()
        if pr.returncode != 0:
            kills.append(t + ':COMPILE-ERROR'); continue
        try:
            r = subprocess.run([W + '/' + t], capture_output=True, text=True, timeout=300)
            if r.returncode != 0: kills.append(t)
        except subprocess.TimeoutExpired:
            kills.append(t + ':TIMEOUT')
    if any('COMPILE-ERROR' in k for k in kills): status = 'INVALID MUTANT (does not compile): ' + ', '.join(kills)
    else: status = 'KILLED by ' + ', '.join(kills) if kills else 'SURVIVED'
    print(f'{name}: {status}', flush=True)
