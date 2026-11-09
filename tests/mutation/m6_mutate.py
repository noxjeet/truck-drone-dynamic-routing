import os, shutil, subprocess, sys
# M6 mutation harness: injects one bug at a time into a COPY of include/ and tests/ and checks that the M6 tests (test_shakes, test_shakes_oracle) fail.
# usage: TDR_REPO=<repo> TDR_LIB=<path of libtdr_core.a from a Release build> TDR_WORK=<scratch dir> python3 m6_mutate.py [name-prefix ...]
# The repository is never modified. 'CW-*' mutants are applied to every occurrence of their pattern.
R = os.environ.get('TDR_REPO', os.getcwd())
W = os.environ.get('TDR_WORK', '/tmp/tdr_m6_mut') + '/w'
LIB = os.environ['TDR_LIB']

SH = 'include/search/shakes.hpp'
M = [
 ('S1-01 reverse excludes the last node of the segment', SH, 'mid.begin() + static_cast<std::ptrdiff_t>(b));\n    detail_shake::settle(detail_shake::rebuild(base, ctx, mid, res), base, ctx, -1, rng, opt, res);\n    return res;\n}\n\ninline ShakeResult shake_random_reverse', 'mid.begin() + static_cast<std::ptrdiff_t>(b - 1));\n    detail_shake::settle(detail_shake::rebuild(base, ctx, mid, res), base, ctx, -1, rng, opt, res);\n    return res;\n}\n\ninline ShakeResult shake_random_reverse'),
 ('S1-02 reverse accepts a == b', SH, 'if (a < 1 || a >= b || b > m)', 'if (a < 1 || a > b || b > m)'),
 ('S1-03 reverse allows b = m + 1 (the end depot)', SH, 'if (a < 1 || a >= b || b > m)', 'if (a < 1 || a >= b || b > m + 1)'),
 ('S1-04 random reverse: second index not skipping x (can pick equal)', SH, 'if (y >= x) ++y;', ''),
 ('S1-05 random reverse: one draw fewer (y drawn from m)', SH, 'std::size_t y = rng.uniform_index(m - 1);', 'std::size_t y = rng.uniform_index(m);'),
 ('S1-06 random reverse: min/max swapped (b < a rejected as invalid)', SH, 'return shake_reverse(base, ctx, std::min(x, y) + 1, std::max(x, y) + 1, rng, opt);', 'return shake_reverse(base, ctx, y + 1, x + 1, rng, opt);'),
 ('S2-01 exchange swaps one element less', SH, 'mid.begin() + static_cast<std::ptrdiff_t>(a - 1 + len), mid.begin() + static_cast<std::ptrdiff_t>(b - 1));', 'mid.begin() + static_cast<std::ptrdiff_t>(a - 1 + len - 1), mid.begin() + static_cast<std::ptrdiff_t>(b - 1));'),
 ('S2-02 exchange allows overlapping segments', SH, 'a + len > b ||', 'a + len > b + 1 ||'),
 ('S2-03 exchange allows segments past the end', SH, 'b + len - 1 > m)', 'b + len - 1 > m + 1)'),
 ('S2-04 random exchange: L bound m instead of m/2', SH, 'static_cast<long>(m / 2)', 'static_cast<long>(m - 1)'),
 ('S2-05 random exchange: L_max ignored', SH, 'std::min<long>(detail_shake::lmax(opt), static_cast<long>(m / 2))', 'static_cast<long>(m / 2)'),
 ('S2-06 random exchange: pair count off by one', SH, 'total += m - 2 * len - a + 2;', 'total += m - 2 * len - a + 1;'),
 ('S2-07 random exchange: adjacent segments never chosen', SH, 'return shake_exchange(base, ctx, a, a + len + r, len, rng, opt);', 'return shake_exchange(base, ctx, a, a + len + 1 + r, len, rng, opt);'),
 ('S3-01 shift forward rotates the wrong range', SH, 'std::rotate(at(A - 1), at(A - 1 + L), at(A - 1 + L + delta));', 'std::rotate(at(A - 1), at(A - 1 + L), at(A - 1 + L + delta - 1));'),
 ('S3-02 shift backward direction swapped', SH, 'else std::rotate(at(A - 1 + delta), at(A - 1), at(A - 1 + L));', 'else std::rotate(at(A - 1 + delta), at(A - 1 + L), at(A - 1 + L));'),
 ('S3-03 shift by zero accepted', SH, 'delta != 0 &&', 'true &&'),
 ('S3-04 shift forward bound off by one', SH, 'A + L - 1 + delta <= m', 'A + L - 1 + delta <= m + 1'),
 ('S3-05 shift backward bound off by one', SH, 'A + delta >= 1)', 'A + delta >= 0)'),
 ('S3-06 random shift: L bound m instead of m-1', SH, 'std::min<long>(detail_shake::lmax(opt), m - 1)', 'std::min<long>(detail_shake::lmax(opt), m)'),
 ('S3-07 random shift: direction always forward when possible', SH, 'const int dir = dirs[rng.uniform_index(dirs.size())];', 'const int dir = dirs.back();'),
 ('S3-08 random shift: D_max ignored', SH, 'std::min<long>(detail_shake::dmax(opt), room)', 'room'),
 ('S3-09 random shift: direction draw skipped', SH, 'const int dir = dirs[rng.uniform_index(dirs.size())];', 'const int dir = dirs[0];'),
 ('S4-01 drone remove: removes only the first trip of the drone', SH, '                if (t.drone_id == removed_drone) {', '                if (t.drone_id == removed_drone && res.removed.empty()) {'),
 ('S4-02 drone remove: removed trips reported as dissolved', SH, 'res.removed.push_back(RemovedTrip{origin[k], t.drone_id, t.service_customer, t.launch_node, t.recovery_node});', 'res.dissolved.push_back(DissolvedTrip{origin[k], t.drone_id, t.service_customer, t.launch_node, t.recovery_node, DissolveReason::LaunchArcVanished});'),
 ('S4-04 random drone remove: first id always', SH, 'ids[rng.uniform_index(ids.size())]', 'ids[0]'),
 ('S4-05 random drone remove: no-trip guard off', SH, 'if (ids.empty()) { none.status = ShakeStatus::NotApplicable; return none; }', ''),
 ('S4-06 drone remove: absent drone accepted', SH, 'if (!has) { res.status = ShakeStatus::InvalidParameters; return res; }', ''),
 ('D6-01 vanished-launch trips are preserved (not dissolved)', SH, 'if (lp == cur.truck_route.size()) { bad = true; why = DissolveReason::LaunchArcVanished; }', 'if (false) {}'),
 ('D6-02 recovery-order check dropped', SH, 'else if (route_index(cur.truck_route, t.recovery_node, lp + 1) == cur.truck_route.size()) {', 'else if (false) {'),
 ('D6-03 recovery order checked from the launch position (not strictly after)', SH, 'route_index(cur.truck_route, t.recovery_node, lp + 1) == cur.truck_route.size()', 'route_index(cur.truck_route, t.recovery_node, lp) == cur.truck_route.size()'),
 ('D6-04 recovery order checked from 0 (any occurrence)', SH, 'route_index(cur.truck_route, t.recovery_node, lp + 1) == cur.truck_route.size()', 'route_index(cur.truck_route, t.recovery_node, 0) == cur.truck_route.size()'),
 ('D6-05 reason of a vanished launch recorded as recovery-order', SH, 'if (lp == cur.truck_route.size()) { bad = true; why = DissolveReason::LaunchArcVanished; }', 'if (lp == cur.truck_route.size()) { bad = true; why = DissolveReason::RecoveryNotAfterLaunch; }'),
 ('D6-06 dissolved trip not erased', SH, '        erase_flagged(cur, origin, drop);\n    }\n    for (NodeId f : drop_unused_flps', '    }\n    for (NodeId f : drop_unused_flps'),
 ('D6-07 dissolved customers not queued for reinsertion (customer lost)', SH, '                res.dissolved.push_back(DissolvedTrip{origin[k], t.drone_id, t.service_customer, t.launch_node, t.recovery_node, why});\n                queue.push_back(t.service_customer);', '                res.dissolved.push_back(DissolvedTrip{origin[k], t.drone_id, t.service_customer, t.launch_node, t.recovery_node, why});'),
 ('D6-08 dissolved trip not recorded', SH, '                res.dissolved.push_back(DissolvedTrip{origin[k], t.drone_id, t.service_customer, t.launch_node, t.recovery_node, why});\n', ''),
 ('D6-09 idle FLPs not dropped', SH, '    for (NodeId f : drop_unused_flps(cur, ctx.nodes())) res.flps_dropped.push_back(f);', ''),
 ('D6-10 dropped FLPs not reported', SH, 'for (NodeId f : drop_unused_flps(cur, ctx.nodes())) res.flps_dropped.push_back(f);', 'drop_unused_flps(cur, ctx.nodes());'),
 ('D6-11 origin bookkeeping: preserved list not set', SH, '    res.preserved = origin;\n', ''),
 ('FL-01 FLPs of a vanished arc kept on the route (foreign FLP)', SH, 'const auto it = by_arc.find(arc_key(sk[k], sk[k + 1]));\n        if (it == by_arc.end()) continue;', 'auto it = by_arc.find(arc_key(sk[k], sk[k + 1]));\n        if (it == by_arc.end()) it = by_arc.begin();'),
 ('FL-02 FLPs of a surviving arc dropped', SH, 'for (NodeId f : it->second) { out.truck_route.push_back(f); ++kept; }', 'for (NodeId f : it->second) { ++kept; }'),
 ('FL-03 FLPs of an arc in reversed order', SH, 'for (NodeId f : it->second) { out.truck_route.push_back(f); ++kept; }', 'for (auto q = it->second.rbegin(); q != it->second.rend(); ++q) { out.truck_route.push_back(*q); ++kept; }'),
 ('FL-04 lost FLPs not recorded', SH, 'res.flps_lost_with_arcs.push_back(v);', '(void)v;'),
 ('FL-05 arc key ignores direction', SH, 'return (static_cast<std::uint64_t>(a) << 32) ^ static_cast<std::uint64_t>(b);', 'return (static_cast<std::uint64_t>(std::min(a, b)) << 32) ^ static_cast<std::uint64_t>(std::max(a, b));'),
 ('RI-01 reinsertion into any gap (not only free ones)', SH, 'for (std::size_t g = 0; g < gaps; ++g) if (sk.route_position[g + 1] == sk.route_position[g] + 1) free_gaps.push_back(g);', 'for (std::size_t g = 0; g < gaps; ++g) free_gaps.push_back(g);'),
 ('RI-02 free-gap draw replaced by the first free gap', SH, 'g = free_gaps[rng.uniform_index(free_gaps.size())];', 'g = free_gaps[0];'),
 ('RI-03 free-gap draw replaced by the last free gap', SH, 'g = free_gaps[rng.uniform_index(free_gaps.size())];', 'g = free_gaps.back();'),
 ('RI-04 fallback gap draw over gaps - 1', SH, 'g = rng.uniform_index(gaps);', 'g = rng.uniform_index(gaps - 1);'),
 ('RI-05 fallback does not dissolve the trips launching in the gap', SH, '                if (std::find(in_gap.begin(), in_gap.end(), t.launch_node) == in_gap.end()) continue;', '                continue;'),
 ('RI-06 fallback does not erase the gap FLPs from the route', SH, 'cur.truck_route.erase(cur.truck_route.begin() + static_cast<std::ptrdiff_t>(sk.route_position[g] + 1), cur.truck_route.begin() + static_cast<std::ptrdiff_t>(sk.route_position[g + 1]));', ''),
 ('RI-08 fallback flag not recorded', SH, '            ri.fallback = true;\n', ''),
 ('RI-09 customer inserted before the anchor node (gap start - 1)', SH, 'cur.truck_route.insert(cur.truck_route.begin() + static_cast<std::ptrdiff_t>(sk.route_position[g] + 1), c);', 'cur.truck_route.insert(cur.truck_route.begin() + static_cast<std::ptrdiff_t>(sk.route_position[g]), c);'),
 ('RI-10 customers reinserted in reverse queue order', SH, 'const NodeId c = queue[qi];', 'const NodeId c = queue[queue.size() - 1 - qi];'),
 ('RI-11 anchor recorded as the next skeleton node', SH, 'ri.after = sk.node[g];', 'ri.after = sk.node[g + 1];'),
 ('RI-12 free-gap count not recorded', SH, '        ri.free_gaps = free_gaps.size();\n', ''),
 ('EV-01 result not evaluated', SH, 'if (opt.evaluate_result) {', 'if (false) {'),
 ('EV-02 evaluate_result ignored', SH, 'if (opt.evaluate_result) {', 'if (true) {'),
 ('EV-03 verdict forced feasible', SH, 'res.feasible = v.feasible;', 'res.feasible = true;'),
 ('EV-04 objective not recorded', SH, 'res.objective = v.objective;', ''),
 ('EV-05 reject reason not recorded', SH, 'res.reason = v.reason;', ''),
 ('EV-06 base objective not recorded', SH, 'res.base_objective = e.objective_value;', ''),
 ('EV-07 infeasible input not rejected', SH, 'if (!e.feasible) { res.status = ShakeStatus::InputInfeasible; return false; }', ''),
 ('SH-01 shake(): fixed kind', SH, 'switch (rng.uniform_index(4)) {', 'switch (rng.uniform_index(4) * 0) {'),
 ('SH-02 shake(): drone-remove drawn with probability 0', SH, 'switch (rng.uniform_index(4)) {', 'switch (rng.uniform_index(3)) {'),
 ('SH-03 shake(): Exchange and Shift swapped', SH, 'case 1: return shake_random_exchange(base, ctx, rng, opt);\n        case 2: return shake_random_shift(base, ctx, rng, opt);', 'case 1: return shake_random_shift(base, ctx, rng, opt);\n        case 2: return shake_random_exchange(base, ctx, rng, opt);'),
 ('SH-04 shake(): NotApplicable redrawn as reverse', SH, 'default: return shake_random_drone_remove(base, ctx, rng, opt);', 'default: { auto r = shake_random_drone_remove(base, ctx, rng, opt); if (r.status == ShakeStatus::NotApplicable) return shake_random_reverse(base, ctx, rng, opt); return r; }'),
 ('NA-01 route shakes applicable with m = 1', SH, 'if (m < 2) { none.status = ShakeStatus::NotApplicable; return none; }\n    const std::size_t x', 'if (m < 1) { none.status = ShakeStatus::NotApplicable; return none; }\n    const std::size_t x'),
 ('NA-02 depot included in the interior', SH, 'for (std::size_t s = 1; s + 1 < sk.size(); ++s) mid.push_back(sk.node[s]);', 'for (std::size_t s = 0; s + 1 < sk.size(); ++s) mid.push_back(sk.node[s]);'),
 ('OP-01 max_segment_length default changed', SH, 'int max_segment_length = 3;', 'int max_segment_length = 4;'),
 ('OP-02 max_shift default changed', SH, 'int max_shift = 3;', 'int max_shift = 2;'),
 ('OP-03 lmax floor removed', SH, 'inline long lmax(const ShakeOptions& o) { return std::max(1, o.max_segment_length); }', 'inline long lmax(const ShakeOptions& o) { return o.max_segment_length; }'),
 ('OP-04 dmax floor removed', SH, 'inline long dmax(const ShakeOptions& o) { return std::max(1, o.max_shift); }', 'inline long dmax(const ShakeOptions& o) { return o.max_shift; }'),
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
    tests = tuple(os.environ.get('TDR_TESTS', 'test_shakes,test_shakes_oracle').split(','))
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
